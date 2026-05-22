#ifndef JZT_ROBOT_BT_DOCKING_ACTION_HPP_
#define JZT_ROBOT_BT_DOCKING_ACTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "jzt_robot_msgs/action/dock.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace jzt_robot {

using DockAction = jzt_robot_msgs::action::Dock;

/**
 * @brief BT Action Node that calls the DockingController action server
 *
 * XML Parameters:
 *   - target: blackboard key containing PoseStamped (goal pose in map frame)
 *   - target_frame: frame_id to use for target lookup (default: "map")
 *   - xy_tolerance: position tolerance in meters (default: 0.05)
 *   - yaw_tolerance: orientation tolerance in radians (default: 0.17)
 *   - timeout: maximum time in seconds (default: 30.0)
 *   - docking_success: output blackboard key (bool)
 */
class BtDockingAction : public nav2_behavior_tree::BtActionNode<DockAction> {
public:
  BtDockingAction(const std::string &xml_tag_name,
                  const std::string &action_name,
                  const BT::NodeConfiguration &conf);

  ~BtDockingAction() override = default;

  // BT input/output ports
  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<geometry_msgs::msg::PoseStamped>("target",
                                                        "Target docking pose"),
         BT::InputPort<std::string>("target_frame", "map",
                                    "Frame of target pose"),
         BT::InputPort<float>("xy_tolerance", 0.05f,
                              "XY position tolerance (m)"),
         BT::InputPort<float>("yaw_tolerance", 0.17f, "Yaw tolerance (rad)"),
         BT::InputPort<float>("timeout", 30.0f, "Timeout in seconds"),
         BT::OutputPort<bool>("docking_success", "Whether docking succeeded")});
  }

protected:
  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

private:
  geometry_msgs::msg::PoseStamped target_pose_;
  float xy_tolerance_;
  float yaw_tolerance_;
  float timeout_;
  std::string target_frame_;
};

} // namespace jzt_robot

#endif // JZT_ROBOT_BT_DOCKING_ACTION_HPP_