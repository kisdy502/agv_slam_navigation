#include "jzt_robot/plugins/action/bt_compute_staging_pose.hpp"
#include <cmath>
// BUGFIX: 原来用 tf2::getYaw()，导致运行时找不到 tf2::fromMsg 符号
//        改用自包含的四元数转 yaw，不依赖 tf2，避免链接问题

#include "rclcpp/rclcpp.hpp"  // 用于日志输出

namespace jzt_robot {

ComputeStagingPose::ComputeStagingPose(const std::string &name,
                                       const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList ComputeStagingPose::providedPorts() {
  return {BT::InputPort<geometry_msgs::msg::PoseStamped>(
              "goal", "Final docking goal pose"),
          BT::OutputPort<geometry_msgs::msg::PoseStamped>(
              "staging_pose", "Computed staging pose before goal"),
          BT::InputPort<double>("distance", 0.75,
                                "Distance from goal to staging pose (meters)")};
}

// 自包含的四元数转 yaw，不依赖 tf2，避免链接时找不到 tf2::fromMsg 符号
static double quaternionToYaw(const geometry_msgs::msg::Quaternion &q) {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

BT::NodeStatus ComputeStagingPose::tick() {
  geometry_msgs::msg::PoseStamped goal;
  double distance = 0.75;

  if (!getInput("goal", goal)) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeStagingPose"),
                 "Missing required input [goal]");
    return BT::NodeStatus::FAILURE;
  }
  getInput("distance", distance);

  // 从 goal 的朝向反方向偏移 distance
  double yaw = quaternionToYaw(goal.pose.orientation);

  // ===== DEBUG 日志 =====
  RCLCPP_INFO(rclcpp::get_logger("ComputeStagingPose"),
              "=== 收到目标点 ===");
  RCLCPP_INFO(rclcpp::get_logger("ComputeStagingPose"),
              "  goal: x=%.4f, y=%.4f, yaw=%.4f (%.1f°), frame=%s",
              goal.pose.position.x, goal.pose.position.y,
              yaw, yaw * 180.0 / M_PI, goal.header.frame_id.c_str());

  geometry_msgs::msg::PoseStamped staging = goal;
  staging.pose.position.x -= distance * std::cos(yaw);
  staging.pose.position.y -= distance * std::sin(yaw);

  RCLCPP_INFO(rclcpp::get_logger("ComputeStagingPose"),
              "=== 计算预停靠点 (distance=%.3f) ===", distance);
  RCLCPP_INFO(rclcpp::get_logger("ComputeStagingPose"),
              "  staging: x=%.4f, y=%.4f, yaw=%.4f (%.1f°), frame=%s",
              staging.pose.position.x, staging.pose.position.y,
              yaw, yaw * 180.0 / M_PI, staging.header.frame_id.c_str());
  RCLCPP_INFO(rclcpp::get_logger("ComputeStagingPose"),
              "  goal→staging 偏移: dx=%.4f, dy=%.4f, dist=%.4f",
              goal.pose.position.x - staging.pose.position.x,
              goal.pose.position.y - staging.pose.position.y,
              std::hypot(goal.pose.position.x - staging.pose.position.x,
                         goal.pose.position.y - staging.pose.position.y));
  // ===== DEBUG END =====

  setOutput("staging_pose", staging);

  return BT::NodeStatus::SUCCESS;
}

} // namespace jzt_robot

// ==================== Plugin Registration ====================
#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<jzt_robot::ComputeStagingPose>("ComputeStagingPose");
}