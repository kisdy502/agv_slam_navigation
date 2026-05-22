#include "jzt_robot/plugins/action/bt_docking_action.hpp"

namespace jzt_robot {

BtDockingAction::BtDockingAction(const std::string &xml_tag_name,
                                 const std::string &action_name,
                                 const BT::NodeConfiguration &conf)
    : BtActionNode<DockAction>(xml_tag_name, action_name, conf) {
  // 从 XML 属性获取默认值
  getInput("xy_tolerance", xy_tolerance_);
  getInput("yaw_tolerance", yaw_tolerance_);
  getInput("timeout", timeout_);
  getInput("target_frame", target_frame_);
}

void BtDockingAction::on_tick() {
  if (!getInput("target", target_pose_)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [target]");
    throw BT::RuntimeError("Missing required input [target]");
  }

  // 确保目标帧正确
  if (target_pose_.header.frame_id.empty()) {
    target_pose_.header.frame_id = target_frame_;
  }
  target_pose_.header.stamp = node_->now();

  // 填充 goal
  goal_.target_pose = target_pose_;
  goal_.xy_tolerance = xy_tolerance_;
  goal_.yaw_tolerance = yaw_tolerance_;
  goal_.timeout = timeout_;
  goal_.target_frame = target_frame_;

  RCLCPP_INFO(node_->get_logger(),
              "DockingAction: target=[%.3f, %.3f] frame=%s, xy_tol=%.3f, "
              "yaw_tol=%.3f, timeout=%.1f",
              target_pose_.pose.position.x, target_pose_.pose.position.y,
              target_frame_.c_str(), xy_tolerance_, yaw_tolerance_, timeout_);
}

BT::NodeStatus BtDockingAction::on_success() {
  RCLCPP_INFO(node_->get_logger(), "DockingAction succeeded");
  setOutput("docking_success", true);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BtDockingAction::on_aborted() {
  // result_ 没有 message 字段，直接用日志输出
  RCLCPP_ERROR(node_->get_logger(), "DockingAction aborted");
  setOutput("docking_success", false);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus BtDockingAction::on_cancelled() {
  RCLCPP_WARN(node_->get_logger(), "DockingAction cancelled");
  setOutput("docking_success", false);
  return BT::NodeStatus::FAILURE;
}

} // namespace jzt_robot

// ==================== Plugin Registration ====================
#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<jzt_robot::BtDockingAction>(
        name, "docking_controller/dock", config); // 去掉开头的 /
  };

  factory.registerBuilder<jzt_robot::BtDockingAction>("DockingAction", builder);
}