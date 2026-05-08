#include "pure_pursuit/pure_pursuit_node.hpp"

PurePursuitNode::PurePursuitNode()
    : Node("pure_pursuit_node"), controller_(2.8, 5.0) {

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "path", 10,
      std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void PurePursuitNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {

  if (msg->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty path");
    return;
  }

  // ===== 当前位姿（工程简化）=====
  const auto &pose = msg->poses.back().pose;
  Pose car{pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)};

  // ===== nav_msgs/Path → Path =====
  Path path;
  path.reserve(msg->poses.size());
  for (const auto &p : msg->poses) {
    path.emplace_back(PathPoint{p.pose.position.x, p.pose.position.y,
                                tf2::getYaw(p.pose.orientation)});
  }

  double steer = controller_.compute(car, path);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.5;
  cmd.angular.z = steer;

  cmd_pub_->publish(cmd);

  RCLCPP_INFO(this->get_logger(), "Steer=%.3f, v=%.2f", steer, cmd.linear.x);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}