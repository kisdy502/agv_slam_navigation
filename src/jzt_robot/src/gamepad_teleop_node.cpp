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
    this->declare_parameter<double>("dominant_threshold", 0.5);   // 主导方向阈值
    this->declare_parameter<double>("cross_suppress_ratio", 0.3); // 交叉抑制比例
    this->declare_parameter<int>("btn_stop", 0);
    this->declare_parameter<int>("btn_speed_up", 3);
    this->declare_parameter<int>("btn_speed_down", 4);
    this->declare_parameter<std::string>("cmd_topic", cmd_topic_);

    this->get_parameter("axis_linear", axis_linear_);
    this->get_parameter("axis_angular", axis_angular_);
    this->get_parameter("deadzone", deadzone_);
    this->get_parameter("dominant_threshold", dominant_threshold_);
    this->get_parameter("cross_suppress_ratio", cross_suppress_ratio_);
    this->get_parameter("btn_stop", btn_stop_);
    this->get_parameter("btn_speed_up", btn_speed_up_);
    this->get_parameter("cmd_topic", cmd_topic_);

    RCLCPP_INFO(this->get_logger(),
                "Gamepad Teleop: deadzone=%.2f, dominant_threshold=%.2f, cross_suppress=%.2f",
                deadzone_, dominant_threshold_, cross_suppress_ratio_);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&GamepadTeleopNode::joyCallback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
  }

private:
  // ========== 核心优化：主导方向抑制交叉抖动 ==========
  void applyCrossSuppression(double &linear, double &angular)
  {
    double abs_linear = std::abs(linear);
    double abs_angular = std::abs(angular);

    if (abs_linear > dominant_threshold_ && abs_angular > dominant_threshold_)
    {
      return; // 都很大，不抑制
    }

    // 线性主导：angular < 30% of linear → 抑制 angular
    if (abs_linear > dominant_threshold_ &&
        abs_angular > deadzone_ &&
        abs_angular < cross_suppress_ratio_ * abs_linear)
    {
      angular = 0.0;
    }
    else if (abs_linear > dominant_threshold_ &&
             abs_angular <= deadzone_)
    {
      angular = 0.0;
    }

    // 角速度主导：linear < 30% of angular → 抑制 linear
    else if (abs_angular > dominant_threshold_ &&
             abs_linear > deadzone_ &&
             abs_linear < cross_suppress_ratio_ * abs_angular)
    {
      linear = 0.0;
    }

    else if (abs_angular > dominant_threshold_ &&
             abs_linear <= deadzone_)
    {
      linear = 0.0;
    }
  }

  // 平滑滤波：低通滤波器，抑制高频抖动
  double lowPassFilter(double current, double &prev, double alpha = 0.7)
  {
    double filtered = alpha * current + (1.0 - alpha) * prev;
    prev = filtered;
    return filtered;
  }

  void publishVelocity(const geometry_msgs::msg::Twist &twist)
  {
    vel_pub_->publish(twist);
    if (twist.linear.x == 0.0 && twist.angular.z == 0.0)
    {
      RCLCPP_INFO(this->get_logger(), "发送了停止速度控制！");
    }
  }

  bool hasAnyInput(const sensor_msgs::msg::Joy::SharedPtr &msg) const
  {
    for (size_t i = 0; i < msg->axes.size(); i++)
    {
      auto axesValue = msg->axes[i];
      if (i == 4 || i == 5)
      { // LT RT 我手上有两个北通遥控器，居然一个1.0一个-1.0，真是醉了，先兼容一下，这两个按键忽略
        // if (axesValue < 1.0 - deadzone_)
        //   return true;
      }
      else
      {
        if (std::abs(axesValue) > deadzone_)
          return true;
      }
    }
    for (const auto &btn : msg->buttons)
    {
      if (btn != 0)
        return true;
    }
    return false;
  }

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
      return 0.3;
    case 1:
      return 0.7;
    case 2:
      return 1.2;
    default:
      return 0.7;
    }
  }

  double getAngularScale() const
  {
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
  /**
   * 如果是阿克曼就机器人，只有角速度，没有线速度，是无法移动的，这是由其运动模型决定的
   */
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const bool has_input = hasAnyInput(msg);
    auto twist = geometry_msgs::msg::Twist();
    // RCLCPP_INFO(this->get_logger(), "has_input=%s, axes[0]=%.2f, axes[1]=%.2f",
    //             has_input ? "true" : "false",
    //             msg->axes.size() > 0 ? msg->axes[0] : 0.0,
    //             msg->axes.size() > 1 ? msg->axes[1] : 0.0);
    if (!has_input)
    {
      if (joy_no_trigger_send_zero < 6)
      {
        joy_no_trigger_send_zero++;
        publishVelocity(twist);
      }
      return;
    }
    joy_no_trigger_send_zero = 0;

    // 紧急停止
    if (btn_stop_ >= 0 && btn_stop_ < static_cast<int>(msg->buttons.size()))
    {
      if (msg->buttons[btn_stop_] == 1)
      {
        publishVelocity(twist);
        return;
      }
    }

    // 速度档位
    handleSpeedButtons(msg);

    if (axis_linear_ >= static_cast<int>(msg->axes.size()) ||
        axis_angular_ >= static_cast<int>(msg->axes.size()))
    {
      return;
    }

    // 获取原始输入
    double raw_linear = applyDeadzone(msg->axes[axis_linear_]);
    double raw_angular = applyDeadzone(msg->axes[axis_angular_]);

    // 低通滤波（抑制高频抖动）
    double filtered_linear = lowPassFilter(raw_linear, prev_linear_);
    double filtered_angular = lowPassFilter(raw_angular, prev_angular_);
    // RCLCPP_INFO(this->get_logger(), "raw_linear=%.2f, raw_angular=%.2f, filtered_linear=%.2f, filtered_angular=%.2f",
    //             raw_linear, raw_angular, filtered_linear, filtered_angular);
    // 核心：主导方向交叉抑制
    applyCrossSuppression(filtered_linear, filtered_angular);
    // RCLCPP_INFO(this->get_logger(), "after setting raw_linear=%.2f, raw_angular=%.2f, filtered_linear=%.2f, filtered_angular=%.2f",
    //             raw_linear, raw_angular, filtered_linear, filtered_angular);
    // 再次应用死区（抑制后可能产生新的微小值）
    filtered_linear = applyDeadzone(filtered_linear);
    filtered_angular = applyDeadzone(filtered_angular);

    twist.linear.x = filtered_linear * getLinearScale();
    twist.angular.z = filtered_angular * getAngularScale();
    publishVelocity(twist);
  }

  void handleSpeedButtons(const sensor_msgs::msg::Joy::SharedPtr &msg)
  {
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
  }

  int axis_linear_;
  int axis_angular_;
  double deadzone_ = 0.1;
  double dominant_threshold_ = 0.5;   // 主导方向阈值
  double cross_suppress_ratio_ = 0.3; // 交叉抑制比例
  int btn_stop_;
  int btn_speed_up_;
  int btn_speed_down_;
  int speed_level_ = 1;
  bool speed_up_pressed_ = false;
  bool speed_down_pressed_ = false;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  int joy_no_trigger_send_zero = 6;
  std::string cmd_topic_ = "/cmd_vel";

  // 低通滤波状态
  double prev_linear_ = 0.0;
  double prev_angular_ = 0.0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamepadTeleopNode>());
  rclcpp::shutdown();
  return 0;
}