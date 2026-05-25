#ifndef JZT_ROBOT_BT_SET_NAV2_PARAM_HPP_
#define JZT_ROBOT_BT_SET_NAV2_PARAM_HPP_

#include "behaviortree_cpp_v3/action_node.h"

namespace jzt_robot {

/**
 * @brief 运行时动态修改 Nav2 节点的参数，并保存旧值以便恢复。
 *
 * 典型用法（窄过道场景）：
 *   <SetNav2Param node="local_costmap/local_costmap"
 *                 param="inflation_layer.inflation_radius"
 *                 value="0.2" old_value="{saved}"/>
 *   ... 通过窄过道 ...
 *   <SetNav2Param node="local_costmap/local_costmap"
 *                 param="inflation_layer.inflation_radius"
 *                 value="{saved}"/>
 */
class SetNav2Param : public BT::SyncActionNode {
public:
  SetNav2Param(const std::string &name,
               const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace jzt_robot

#endif // JZT_ROBOT_BT_SET_NAV2_PARAM_HPP_
