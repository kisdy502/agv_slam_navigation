#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "pure_pursuit.h"

class PurePursuitNode : public rclcpp::Node {
public:
    explicit PurePursuitNode();

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    PurePursuit controller_;
};