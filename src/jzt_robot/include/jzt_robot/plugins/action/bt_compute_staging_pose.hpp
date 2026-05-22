#ifndef JZT_ROBOT_BT_COMPUTE_STAGING_POSE_HPP_
#define JZT_ROBOT_BT_COMPUTE_STAGING_POSE_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace jzt_robot {

class ComputeStagingPose : public BT::SyncActionNode {
public:
  ComputeStagingPose(const std::string &name,
                     const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace jzt_robot

#endif // JZT_ROBOT_BT_COMPUTE_STAGING_POSE_HPP_