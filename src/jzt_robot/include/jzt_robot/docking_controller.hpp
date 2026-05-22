#ifndef JZT_ROBOT_DOCKING_CONTROLLER_HPP_
#define JZT_ROBOT_DOCKING_CONTROLLER_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "jzt_robot_msgs/action/dock.hpp"
#include "std_msgs/msg/string.hpp"

namespace jzt_robot {

using DockAction = jzt_robot_msgs::action::Dock;
using GoalHandleDock = rclcpp_action::ServerGoalHandle<DockAction>;

class DockingController : public rclcpp::Node // 改成普通 Node
{
public:
  explicit DockingController(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~DockingController() override;

protected:
  // Action Server callbacks
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const DockAction::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleDock> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleDock> goal_handle);

  // Main control loop
  void control_loop();

  // Transform target pose to base_link frame
  bool get_target_in_base_link(const geometry_msgs::msg::PoseStamped &target,
                               geometry_msgs::msg::PoseStamped &target_base);

  // ISE2 controller for Ackermann
  geometry_msgs::msg::Twist
  compute_control(const geometry_msgs::msg::PoseStamped &target_base,
                  float xy_tol, float yaw_tol);

  // Check if goal reached
  bool is_goal_reached(const geometry_msgs::msg::PoseStamped &target_base,
                       float xy_tol, float yaw_tol);

  // Smooth velocity command (rate limiter)
  geometry_msgs::msg::Twist
  smooth_velocity(const geometry_msgs::msg::Twist &raw_cmd);

  // Parameters
  double controller_frequency_;
  double min_turning_radius_;
  double max_linear_vel_;
  double max_angular_vel_;
  double linear_kp_;
  double angular_kp_;
  double approach_linear_kp_;
  double approach_angular_kp_;
  double slowdown_distance_;
  double final_approach_distance_;
  double cmd_vel_timeout_;
  double transform_tolerance_;

  // Rate limiting
  double max_linear_accel_;
  double max_angular_accel_;

  // Members
  rclcpp_action::Server<DockAction>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      select_pub_; // /mux/select，只创建一次
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State
  std::mutex state_mutex_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  std::shared_ptr<GoalHandleDock> active_goal_;
  bool is_active_ = false;

  // Velocity smoothing state
  geometry_msgs::msg::Twist last_cmd_vel_;
  rclcpp::Time last_cmd_time_;

  // Error history for integral term (optional)
  std::deque<std::pair<double, double>> error_history_;
  static constexpr size_t ERROR_HISTORY_SIZE = 10;
  bool is_docking_active_ = false;
};

} // namespace jzt_robot

#endif // JZT_ROBOT_DOCKING_CONTROLLER_HPP_