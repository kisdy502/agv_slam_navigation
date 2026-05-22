// cmd_vel_mux.hpp
#ifndef CMD_VEL_MUX_HPP_
#define CMD_VEL_MUX_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CmdVelMux : public rclcpp::Node
{
public:
  CmdVelMux();

private:
  void nav2CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void dockingCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void selectCallback(const std_msgs::msg::String::SharedPtr msg);
  void publishLoop();

  // 当前激活的输入源
  std::string activeSource_; // "nav2" or "docking"

  // 缓存的速度指令
  geometry_msgs::msg::Twist nav2Cmd_;
  geometry_msgs::msg::Twist dockingCmd_;
  bool hasNav2Cmd_;
  bool hasDockingCmd_;

  // 超时检测
  rclcpp::Time lastNav2Time_;
  rclcpp::Time lastDockingTime_;
  double timeoutSec_;

  // ROS 接口
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2Sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr dockingSub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selectSub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr outputPub_;
  rclcpp::TimerBase::SharedPtr publishTimer_;
};

#endif // CMD_VEL_MUX_HPP_