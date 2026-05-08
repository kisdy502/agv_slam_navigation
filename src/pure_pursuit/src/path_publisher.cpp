#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 

class PathPublisher : public rclcpp::Node {
public:
  PathPublisher() : Node("path_publisher") {
    pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1),
                               std::bind(&PathPublisher::publishPath, this));
  }

private:
  void publishPath() {
    auto msg = createTestPath();
    pub_->publish(msg);
  }

  nav_msgs::msg::Path createTestPath() {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = rclcpp::Clock().now();

    std::vector<std::tuple<double, double, double>> points = {
        {0, 0, 0}, {5, 0, 0}, {10, 0, 0}, {15, 5, 0.5}, {20, 10, 0.7}};

    for (const auto &[x, y, yaw] : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;

      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation = tf2::toMsg(q);

      path.poses.push_back(pose);
    }

    return path;
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPublisher>());
  rclcpp::shutdown();
  return 0;
}