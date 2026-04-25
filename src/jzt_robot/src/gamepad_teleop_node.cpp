#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class GamepadTeleopNode : public rclcpp::Node
{
public:
  GamepadTeleopNode() : Node("gamepad_teleop_node")
  {
    this->declare_parameter<int>("axis_linear", 1);
    this->declare_parameter<int>("axis_angular", 0);
    this->declare_parameter<double>("deadzone", 0.1);
    this->declare_parameter<int>("btn_stop", 1);
    this->declare_parameter<int>("btn_speed_up", 0);   // X
    this->declare_parameter<int>("btn_speed_down", 3); // Y

    this->get_parameter("axis_linear", axis_linear_);
    this->get_parameter("axis_angular", axis_angular_);
    this->get_parameter("deadzone", deadzone_);
    this->get_parameter("btn_stop", btn_stop_);
    this->get_parameter("btn_speed_up", btn_speed_up_);
    this->get_parameter("btn_speed_down", btn_speed_down_);

    RCLCPP_INFO(this->get_logger(),
                "Gamepad Teleop started: speed_level=%d (low=0.3, mid=0.7, high=1.2)",
                speed_level_);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&GamepadTeleopNode::joyCallback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  double applyDeadzone(double value)
  {
    if (std::abs(value) < deadzone_)
      return 0.0;
    return value;
  }

  double getLinearScale() const
  {
    switch (speed_level_)
    {
    case 0:
      return 0.3; // 低速
    case 1:
      return 0.7; // 中速
    case 2:
      return 1.2; // 高速
    default:
      return 0.7;
    }
  }

  double getAngularScale() const
  {
    // 角速度随线速度比例调整，高速时转向稍慢更稳
    switch (speed_level_)
    {
    case 0:
      return 0.6;
    case 1:
      return 1.0;
    case 2:
      return 1.5;
    default:
      return 1.0;
    }
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto twist = geometry_msgs::msg::Twist();
    // RCLCPP_INFO(this->get_logger(), "buttons[0]=%d, buttons[3]=%d",
    //             msg->buttons[0], msg->buttons[3]);
    // 紧急停止 (A键)
    if (btn_stop_ >= 0 && btn_stop_ < static_cast<int>(msg->buttons.size()))
    {
      if (msg->buttons[btn_stop_] == 1)
      {
        vel_pub_->publish(twist);
        RCLCPP_WARN(this->get_logger(), "Emergency stop!");
        return;
      }
    }

    // 速度档位切换：X上增档，Y下降档
    if (btn_speed_up_ >= 0 && btn_speed_up_ < static_cast<int>(msg->buttons.size()))
    {
      if (msg->buttons[btn_speed_up_] == 1 && !speed_up_pressed_)
      {
        if (speed_level_ < 2)
        {
          speed_level_++;
          RCLCPP_INFO(this->get_logger(), "Speed UP -> level %d (linear=%.1f)",
                      speed_level_, getLinearScale());
        }
        speed_up_pressed_ = true;
      }
      else if (msg->buttons[btn_speed_up_] == 0)
      {
        speed_up_pressed_ = false;
      }
    }

    if (btn_speed_down_ >= 0 && btn_speed_down_ < static_cast<int>(msg->buttons.size()))
    {
      if (msg->buttons[btn_speed_down_] == 1 && !speed_down_pressed_)
      {
        if (speed_level_ > 0)
        {
          speed_level_--;
          RCLCPP_INFO(this->get_logger(), "Speed DOWN -> level %d (linear=%.1f)",
                      speed_level_, getLinearScale());
        }
        speed_down_pressed_ = true;
      }
      else if (msg->buttons[btn_speed_down_] == 0)
      {
        speed_down_pressed_ = false;
      }
    }

    if (axis_linear_ >= static_cast<int>(msg->axes.size()) ||
        axis_angular_ >= static_cast<int>(msg->axes.size()))
    {
      return;
    }

    double linear_input = applyDeadzone(msg->axes[axis_linear_]);
    double angular_input = applyDeadzone(msg->axes[axis_angular_]);

    twist.linear.x = linear_input * getLinearScale();
    twist.angular.z = angular_input * getAngularScale();

    vel_pub_->publish(twist);
  }

  int axis_linear_;
  int axis_angular_;
  double deadzone_;
  int btn_stop_;
  int btn_speed_up_;
  int btn_speed_down_;
  int speed_level_ = 1; // 默认中速 (0=低速, 1=中速, 2=高速)
  bool speed_up_pressed_ = false;
  bool speed_down_pressed_ = false;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamepadTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
