#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace triple_steer_robot
{

struct WheelConfig {
  std::string name;
  double x;   // wheel position in base_link frame [m]
  double y;   // wheel position in base_link frame [m]
};

class TripleSteerKinematics : public rclcpp::Node
{
public:
  TripleSteerKinematics()
  : Node("triple_steer_kinematics")
  {
    // Robot geometry (must match URDF robot_base.xacro)
    const double R = 0.25;
    const double sqrt3_2 = 0.86602540378;
    wheel_radius_ = 0.1;

    wheels_ = {
      WheelConfig{"front",  R,        0.0},
      WheelConfig{"left",  -R / 2.0,  R * sqrt3_2},
      WheelConfig{"right", -R / 2.0, -R * sqrt3_2},
    };

    last_steer_.assign(3, 0.0);

    // Subscribers & Publishers
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&TripleSteerKinematics::cmdVelCallback, this, std::placeholders::_1));

    steer_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/steer_position_controller/commands", 10);

    wheel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/wheel_velocity_controller/commands", 10);

    RCLCPP_INFO(get_logger(), "Triple-steer kinematics node started");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double vx = msg->linear.x;
    const double vy = msg->linear.y;
    const double omega = msg->angular.z;

    std::vector<double> steer_cmds(3, 0.0);
    std::vector<double> wheel_cmds(3, 0.0);

    for (size_t i = 0; i < wheels_.size(); ++i) {
      const double xi = wheels_[i].x;
      const double yi = wheels_[i].y;

      // Desired velocity at wheel contact point
      const double vxi = vx - omega * yi;
      const double vyi = vy + omega * xi;

      const double speed = std::hypot(vxi, vyi);
      const double wheel_vel = speed / wheel_radius_;

      if (speed > 1e-6) {
        steer_cmds[i] = std::atan2(vyi, vxi);
        last_steer_[i] = steer_cmds[i];
        wheel_cmds[i] = wheel_vel;
      } else {
        // Hold previous steering angle when stationary
        steer_cmds[i] = last_steer_[i];
        wheel_cmds[i] = 0.0;
      }
    }

    std_msgs::msg::Float64MultiArray steer_msg;
    steer_msg.data = steer_cmds;
    steer_pub_->publish(steer_msg);

    std_msgs::msg::Float64MultiArray wheel_msg;
    wheel_msg.data = wheel_cmds;
    wheel_pub_->publish(wheel_msg);
  }

  // Wheel geometry
  double wheel_radius_{0.0};
  std::vector<WheelConfig> wheels_;
  std::vector<double> last_steer_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
};

}  // namespace triple_steer_robot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<triple_steer_robot::TripleSteerKinematics>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
