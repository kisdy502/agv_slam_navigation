#include "jzt_robot/plugins/action/bt_set_nav2_param.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

namespace jzt_robot
{

  SetNav2Param::SetNav2Param(const std::string &name,
                             const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  BT::PortsList SetNav2Param::providedPorts()
  {
    return {
        BT::InputPort<std::string>("node", "Target node name, e.g. local_costmap/local_costmap"),
        BT::InputPort<std::string>("param", "Parameter name, e.g. inflation_layer.inflation_radius"),
        BT::InputPort<double>("value", "New value to set"),
        BT::OutputPort<double>("old_value", "Previous value (for restoration)"),
    };
  }

  BT::NodeStatus SetNav2Param::tick()
  {
    std::string node_name, param_name;
    double new_value;

    if (!getInput("node", node_name))
    {
      RCLCPP_ERROR(rclcpp::get_logger("SetNav2Param"),
                   "Missing required input [node]");
      return BT::NodeStatus::FAILURE;
    }
    if (!getInput("param", param_name))
    {
      RCLCPP_ERROR(rclcpp::get_logger("SetNav2Param"),
                   "Missing required input [param]");
      return BT::NodeStatus::FAILURE;
    }
    if (!getInput("value", new_value))
    {
      RCLCPP_ERROR(rclcpp::get_logger("SetNav2Param"),
                   "Missing required input [value]");
      return BT::NodeStatus::FAILURE;
    }

    // 通过 blackboard 获取 ROS 节点，复用已有的 rclcpp 上下文
    auto ros_node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!ros_node)
    {
      RCLCPP_ERROR(rclcpp::get_logger("SetNav2Param"),
                   "No ROS node in blackboard");
      return BT::NodeStatus::FAILURE;
    }

    // 1. 获取旧值
    auto get_client = ros_node->create_client<rcl_interfaces::srv::GetParameters>(
        node_name + "/get_parameters");
    if (!get_client->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(rclcpp::get_logger("SetNav2Param"),
                   "GetParameters service not available for node: %s",
                   node_name.c_str());
      return BT::NodeStatus::FAILURE;
    }

    auto get_req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_req->names = {param_name};
    auto get_future = get_client->async_send_request(get_req);

    if (get_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    {
      RCLCPP_ERROR(rclcpp::get_logger("SetNav2Param"),
                   "GetParameters timeout for node: %s", node_name.c_str());
      return BT::NodeStatus::FAILURE;
    }

    auto get_result = get_future.get();
    double old_value = 0.0;
    if (!get_result->values.empty())
    {
      old_value = get_result->values[0].double_value;
    }

    RCLCPP_INFO(rclcpp::get_logger("SetNav2Param"),
                "Node [%s] param [%s]: %.4f -> %.4f",
                node_name.c_str(), param_name.c_str(), old_value, new_value);

    // 2. 设置新值
    auto set_client = ros_node->create_client<rcl_interfaces::srv::SetParameters>(
        node_name + "/set_parameters");
    if (!set_client->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(rclcpp::get_logger("SetNav2Param"),
                   "SetParameters service not available for node: %s",
                   node_name.c_str());
      return BT::NodeStatus::FAILURE;
    }

    auto set_req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter p;
    p.name = param_name;
    p.value.double_value = new_value;
    p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    set_req->parameters = {p};

    auto set_future = set_client->async_send_request(set_req);
    if (set_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    {
      RCLCPP_ERROR(rclcpp::get_logger("SetNav2Param"),
                   "SetParameters timeout for node: %s", node_name.c_str());
      return BT::NodeStatus::FAILURE;
    }

    auto set_result = set_future.get();
    for (const auto &res : set_result->results)
    {
      if (!res.successful)
      {
        RCLCPP_ERROR(rclcpp::get_logger("SetNav2Param"),
                     "Failed to set param [%s] on node [%s]: %s",
                     param_name.c_str(), node_name.c_str(), res.reason.c_str());
        return BT::NodeStatus::FAILURE;
      }
    }

    setOutput("old_value", old_value);
    return BT::NodeStatus::SUCCESS;
  }

} // namespace jzt_robot

// ==================== Plugin Registration ====================
#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<jzt_robot::SetNav2Param>("SetNav2Param");
}
