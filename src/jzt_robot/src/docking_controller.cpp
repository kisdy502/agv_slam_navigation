#include "jzt_robot/docking_controller.hpp"

#include <algorithm>
#include <cmath>
#include <tf2/utils.h>

namespace jzt_robot {

DockingController::DockingController(const rclcpp::NodeOptions &options)
    : rclcpp::Node("docking_controller", options) {

  // Declare parameters
  declare_parameter("controller_frequency", 50.0);
  declare_parameter("min_turning_radius", 1.40);
  declare_parameter("max_linear_vel", 0.3);
  declare_parameter("max_angular_vel", 0.5);
  declare_parameter("linear_kp", 1.0);
  declare_parameter("angular_kp", 2.0);
  declare_parameter("approach_linear_kp", 1.5);
  declare_parameter("approach_angular_kp", 3.0);
  declare_parameter("slowdown_distance", 0.5);
  declare_parameter("final_approach_distance", 0.15);
  declare_parameter("cmd_vel_timeout", 0.5);
  declare_parameter("transform_tolerance", 1.0);
  declare_parameter("max_linear_accel", 1.0);
  declare_parameter("max_angular_accel", 2.0);

  // Get parameters
  controller_frequency_ = get_parameter("controller_frequency").as_double();
  min_turning_radius_ = get_parameter("min_turning_radius").as_double();
  max_linear_vel_ = get_parameter("max_linear_vel").as_double();
  max_angular_vel_ = get_parameter("max_angular_vel").as_double();
  linear_kp_ = get_parameter("linear_kp").as_double();
  angular_kp_ = get_parameter("angular_kp").as_double();
  approach_linear_kp_ = get_parameter("approach_linear_kp").as_double();
  approach_angular_kp_ = get_parameter("approach_angular_kp").as_double();
  slowdown_distance_ = get_parameter("slowdown_distance").as_double();
  final_approach_distance_ =
      get_parameter("final_approach_distance").as_double();
  cmd_vel_timeout_ = get_parameter("cmd_vel_timeout").as_double();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();
  max_linear_accel_ = get_parameter("max_linear_accel").as_double();
  max_angular_accel_ = get_parameter("max_angular_accel").as_double();

  RCLCPP_INFO(get_logger(), "DockingController starting...");

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publishers
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel_docking", rclcpp::QoS(1).reliable());
  select_pub_ = create_publisher<std_msgs::msg::String>(
      "/mux/select", rclcpp::QoS(1).reliable());

  // Subscribers
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_odom_ = msg;
      });

  // Action Server
  action_server_ = rclcpp_action::create_server<DockAction>(
      this, "/docking_controller/dock",
      std::bind(&DockingController::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&DockingController::handle_cancel, this, std::placeholders::_1),
      std::bind(&DockingController::handle_accepted, this,
                std::placeholders::_1));

  // Create control timer at 50Hz
  auto timer_period =
      std::chrono::duration<double>(1.0 / controller_frequency_);
  control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&DockingController::control_loop, this));

  is_active_ = true;

  RCLCPP_INFO(get_logger(), "DockingController started, action server ready");
}

DockingController::~DockingController() {
  // Stop robot on shutdown
  geometry_msgs::msg::Twist stop;
  cmd_vel_pub_->publish(stop);
}

// ==================== Action Server ====================

rclcpp_action::GoalResponse
DockingController::handle_goal(const rclcpp_action::GoalUUID &,
                               std::shared_ptr<const DockAction::Goal> goal) {
  RCLCPP_INFO(get_logger(),
              "Received docking goal: frame=%s, xy_tol=%.3f, yaw_tol=%.3f",
              goal->target_frame.c_str(), goal->xy_tolerance,
              goal->yaw_tolerance);

  if (!is_active_) {
    RCLCPP_WARN(get_logger(), "Node not active, rejecting goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DockingController::handle_cancel(
    const std::shared_ptr<GoalHandleDock> goal_handle) {
  RCLCPP_INFO(get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DockingController::handle_accepted(
    const std::shared_ptr<GoalHandleDock> goal_handle) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  // Cancel previous goal if any
  if (active_goal_ && active_goal_->is_active()) {
    auto result = std::make_shared<DockAction::Result>();
    result->success = false;
    result->message = "Preempted by new goal";
    active_goal_->abort(result);
  }

  active_goal_ = goal_handle;
  RCLCPP_INFO(get_logger(), "Docking goal accepted and active");
  is_docking_active_ = true; // 标记 docking 开始

  // 通知 cmd_vel_mux 切换到 docking 源
  auto msg = std::make_shared<std_msgs::msg::String>();
  msg->data = "docking";
  select_pub_->publish(*msg);

  RCLCPP_INFO(get_logger(), "Published /mux/select -> docking");
}

// ==================== Control Loop ====================

void DockingController::control_loop() {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!active_goal_ || !active_goal_->is_active()) {
    // 只在 docking 刚结束的那个周期发停止、切回 nav2，然后保持沉默
    // 空闲时不发任何 cmd_vel，避免与其他节点（遥控器等）冲突
    if (is_docking_active_) {
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);
      last_cmd_vel_ = stop;

      auto msg = std::make_shared<std_msgs::msg::String>();
      msg->data = "nav2";
      select_pub_->publish(*msg);
      RCLCPP_INFO(get_logger(), "Docking done, switched back to nav2");
      is_docking_active_ = false;
    }
    return;
  }

  if (!current_odom_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "No odometry received");
    return;
  }

  auto goal = active_goal_->get_goal();
  geometry_msgs::msg::PoseStamped target_base;

  // Transform target to base_link frame
  if (!get_target_in_base_link(goal->target_pose, target_base)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Failed to transform target to base_link");
    return;
  }

  // ===== DEBUG: 每秒输出一次当前误差和cmd_vel =====
  {
    double x_err = target_base.pose.position.x;
    double y_err = target_base.pose.position.y;
    double yaw_err = tf2::getYaw(target_base.pose.orientation);
    double dist = std::hypot(x_err, y_err);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "DOCKING_DEBUG: dist=%.4fm x=%.4f y=%.4f yaw=%.4f(%.1f°) | "
        "cmd_v=%.3f cmd_w=%.3f",
        dist, x_err, y_err, yaw_err, yaw_err * 180.0 / M_PI,
        last_cmd_vel_.linear.x, last_cmd_vel_.angular.z);
  }
  // ===== DEBUG END =====

  // Check timeout
  auto elapsed =
      (now() - rclcpp::Time(goal->target_pose.header.stamp)).seconds();
  if (elapsed > goal->timeout && goal->timeout > 0) {
    auto result = std::make_shared<DockAction::Result>();
    result->success = false;
    result->message = "Timeout";
    result->final_x_error = target_base.pose.position.x;
    result->final_y_error = target_base.pose.position.y;
    result->final_yaw_error = tf2::getYaw(target_base.pose.orientation);
    active_goal_->abort(result);
    active_goal_.reset();

    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    last_cmd_vel_ = stop;
    return;
  }

  // Check if reached
  if (is_goal_reached(target_base, goal->xy_tolerance, goal->yaw_tolerance)) {
    auto result = std::make_shared<DockAction::Result>();
    result->success = true;
    result->message = "Docking succeeded";
    result->final_x_error = target_base.pose.position.x;
    result->final_y_error = target_base.pose.position.y;
    result->final_yaw_error = tf2::getYaw(target_base.pose.orientation);
    active_goal_->succeed(result);
    active_goal_.reset();

    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    last_cmd_vel_ = stop;

    RCLCPP_INFO(
        get_logger(), "Docking succeeded! Errors: x=%.4f, y=%.4f, yaw=%.4f",
        result->final_x_error, result->final_y_error, result->final_yaw_error);
    return;
  }

  // Compute control
  auto cmd =
      compute_control(target_base, goal->xy_tolerance, goal->yaw_tolerance);
  cmd = smooth_velocity(cmd);

  // Publish
  cmd_vel_pub_->publish(cmd);
  last_cmd_vel_ = cmd;

  // Publish feedback
  auto feedback = std::make_shared<DockAction::Feedback>();
  float dist =
      std::hypot(target_base.pose.position.x, target_base.pose.position.y);
  feedback->distance_to_target = dist;
  feedback->current_x_error = target_base.pose.position.x;
  feedback->current_y_error = target_base.pose.position.y;
  feedback->current_yaw_error = tf2::getYaw(target_base.pose.orientation);

  // Phase determination
  if (dist > slowdown_distance_) {
    feedback->phase = 0;
  } else if (dist > final_approach_distance_) {
    feedback->phase = 1;
  } else {
    feedback->phase = 2;
  }

  active_goal_->publish_feedback(feedback);
}

bool DockingController::get_target_in_base_link(
    const geometry_msgs::msg::PoseStamped &target,
    geometry_msgs::msg::PoseStamped &target_base) {
  try {
    geometry_msgs::msg::PoseStamped target_odom = target;
    target_odom.header.stamp = now();

    if (target.header.frame_id != "base_link") {
      auto transform = tf_buffer_->lookupTransform(
          "base_link", target.header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(transform_tolerance_));

      tf2::doTransform(target, target_base, transform);
    } else {
      target_base = target;
    }

    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Transform failed: %s", ex.what());
    return false;
  }
}

geometry_msgs::msg::Twist DockingController::compute_control(
    const geometry_msgs::msg::PoseStamped &target_base,
    [[maybe_unused]] float xy_tol, [[maybe_unused]] float yaw_tol) {
  geometry_msgs::msg::Twist cmd;

  double x_err = target_base.pose.position.x;
  double y_err = target_base.pose.position.y;
  double yaw_err = tf2::getYaw(target_base.pose.orientation);

  while (yaw_err > M_PI)
    yaw_err -= 2.0 * M_PI;
  while (yaw_err < -M_PI)
    yaw_err += 2.0 * M_PI;

  double distance = std::hypot(x_err, y_err);

  double v = 0.0;
  double omega = 0.0;

  if (distance > slowdown_distance_) {
    double angle_to_target = std::atan2(y_err, x_err);
    double kp_l = approach_linear_kp_;
    double kp_a = approach_angular_kp_;
    v = kp_l * distance * std::cos(angle_to_target);
    double heading_weight =
        std::clamp(1.0 - distance / slowdown_distance_, 0.0, 1.0);
    double desired_heading =
        (1.0 - heading_weight) * angle_to_target + heading_weight * yaw_err;
    omega = kp_a * desired_heading;

  } else if (distance > final_approach_distance_) {
    double kp_l = linear_kp_;
    double kp_a = angular_kp_;
    v = kp_l * x_err;
    double stanley_term = std::atan2(2.0 * y_err, std::max(distance, 0.1));
    omega = kp_a * (yaw_err + stanley_term);

  } else {
    // 最终逼近 (distance < 0.15m)
    // BUGFIX v2: 上一版修复了原地旋转，但 yaw_err 和 y_err 的对冲
    //   导致 ω≈0（如 3.0×(-0.018) + 0.5×0.11 ≈ 0），机器人直线来回碾压
    //   侧向偏差，永不到达。
    // 修正：用 angle_to_target 指向目标点驱动，不依赖两项平衡。
    double angle_to_target = std::atan2(y_err, x_err);

    if (std::abs(y_err) > xy_tol || std::abs(x_err) > xy_tol) {
      // 位置偏移 → 转向指向目标点，需要足够前进速度来转弯
      v = std::max(0.10, distance * 0.6);   // ≥0.10m/s 保证转弯能力
      v = std::clamp(v, -0.15, 0.15);
      omega = angular_kp_ * (angle_to_target + yaw_err * 0.5);
    } else if (std::abs(yaw_err) > yaw_tol) {
      v = std::max(0.08, x_err * 1.5);
      v = std::clamp(v, -0.15, 0.15);
      omega = angular_kp_ * yaw_err;
    } else {
      // 位置和朝向都已接近 → 缓慢直进
      v = linear_kp_ * x_err;
      // if (v > 0 && v < 0.02) v = 0.02;
      // if (v < 0 && v > -0.02) v = -0.02;
      // omega = 0.0;
      omega = approach_angular_kp_ * yaw_err + 1.0 * y_err;

    }
  }

  // 阿克曼约束：速度太低时转弯能力不足，提升最低速度
  if (std::abs(v) < 0.03 && std::abs(omega) > 0.001) {
    v = (omega > 0) ? 0.08 : -0.08;
  }
  if (std::abs(v) > 0.001) {
    double max_omega_for_v = std::abs(v) / min_turning_radius_;
    omega = std::clamp(omega, -max_omega_for_v, max_omega_for_v);
  }

  v = std::clamp(v, -max_linear_vel_, max_linear_vel_);
  omega = std::clamp(omega, -max_angular_vel_, max_angular_vel_);

  if (distance < final_approach_distance_) {
    v = std::clamp(v, -0.1, 0.1);
    omega = std::clamp(omega, -0.3, 0.3);
  }

  cmd.linear.x = v;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = omega;

  return cmd;
}

bool DockingController::is_goal_reached(
    const geometry_msgs::msg::PoseStamped &target_base, float xy_tol,
    float yaw_tol) {
  double x_err = target_base.pose.position.x;
  double y_err = target_base.pose.position.y;
  double yaw_err = tf2::getYaw(target_base.pose.orientation);

  while (yaw_err > M_PI)
    yaw_err -= 2.0 * M_PI;
  while (yaw_err < -M_PI)
    yaw_err += 2.0 * M_PI;

  double xy_error = std::hypot(x_err, y_err);

  RCLCPP_DEBUG(get_logger(), "Errors: xy=%.4f (tol=%.3f), yaw=%.4f (tol=%.3f)",
               xy_error, xy_tol, std::abs(yaw_err), yaw_tol);

  return (xy_error < xy_tol) && (std::abs(yaw_err) < yaw_tol);
}

geometry_msgs::msg::Twist
DockingController::smooth_velocity(const geometry_msgs::msg::Twist &raw_cmd) {
  geometry_msgs::msg::Twist smoothed = raw_cmd;

  auto now_time = now();
  double dt = 0.0;
  if (last_cmd_time_.nanoseconds() > 0) {
    dt = (now_time - last_cmd_time_).seconds();
  }
  last_cmd_time_ = now_time;

  if (dt <= 0.0 || dt > 0.1) {
    dt = 1.0 / controller_frequency_;
  }

  double dv = raw_cmd.linear.x - last_cmd_vel_.linear.x;
  double max_dv = max_linear_accel_ * dt;
  if (std::abs(dv) > max_dv) {
    smoothed.linear.x = last_cmd_vel_.linear.x + std::copysign(max_dv, dv);
  }

  double dw = raw_cmd.angular.z - last_cmd_vel_.angular.z;
  double max_dw = max_angular_accel_ * dt;
  if (std::abs(dw) > max_dw) {
    smoothed.angular.z = last_cmd_vel_.angular.z + std::copysign(max_dw, dw);
  }

  if (std::abs(smoothed.linear.x) < 0.001)
    smoothed.linear.x = 0.0;
  if (std::abs(smoothed.angular.z) < 0.001)
    smoothed.angular.z = 0.0;

  return smoothed;
}

} // namespace jzt_robot

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(jzt_robot::DockingController)