#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class GamepadTeleopNode : public rclcpp::Node {
public:
  GamepadTeleopNode() : Node("gamepad_teleop_node") {
    this->declare_parameter<int>("axis_linear", 1);
    this->declare_parameter<int>("axis_angular", 0);
    this->declare_parameter<double>("deadzone", 0.1);
    this->declare_parameter<int>("btn_stop", 0);       // A
    this->declare_parameter<int>("btn_speed_up", 3);   // X
    this->declare_parameter<int>("btn_speed_down", 4); // Y
    // this->declare_parameter<std::string>(
    //     "cmd_topic", "/ackermann_steering_controller/reference_unstamped");

    this->get_parameter("axis_linear", axis_linear_);
    this->get_parameter("axis_angular", axis_angular_);
    this->get_parameter("deadzone", deadzone_);
    this->get_parameter("btn_stop", btn_stop_);
    this->get_parameter("btn_speed_up", btn_speed_up_);
    this->get_parameter("btn_speed_down", btn_speed_down_);
    // this->get_parameter("cmd_topic", cmd_topic_);


    RCLCPP_INFO(
        this->get_logger(),
        "Gamepad Teleop started: speed_level=%d (low=0.3, mid=0.7, high=1.2)",
        speed_level_);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&GamepadTeleopNode::joyCallback, this,
                  std::placeholders::_1));

    vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
  }

private:
  void publishVelocity(const geometry_msgs::msg::Twist &twist) {
    vel_pub_->publish(twist);
    // 检查是否是零速度
    if (twist.linear.x == 0.0 && twist.linear.y == 0.0 &&
        twist.linear.z == 0.0 && twist.angular.x == 0.0 &&
        twist.angular.y == 0.0 && twist.angular.z == 0.0) {
      RCLCPP_INFO(this->get_logger(), "发送了停止速度控制！");
    }
  }
  // 适配北通
  bool hasAnyInput(const sensor_msgs::msg::Joy::SharedPtr &msg) const {
    // 检查所有轴是否有超出死区的
    for (size_t i = 0; i < msg->axes.size(); i++) {
      auto axesValue = msg->axes[i];
      // LT RT 特殊处理，这两个不压下去时候，默认是1，按到底是-1
      if (i == 4 || i == 5) {
        if (axesValue < 1.0 - deadzone_) {
          return true;
        }
      } else {
        if (std::abs(axesValue) > deadzone_) {
          return true;
        }
      }
    }

    // 检查所有按钮是否有被按下的（值 != 0）
    for (const auto &btn : msg->buttons) {
      if (btn != 0) {
        return true;
      }
    }

    return false; // 无任何操作
  }

  double applyDeadzone(double value) {
    if (std::abs(value) < deadzone_)
      return 0.0;
    return value;
  }

  double getLinearScale() const {
    switch (speed_level_) {
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

  double getAngularScale() const {
    // 角速度随线速度比例调整，高速时转向稍慢更稳
    switch (speed_level_) {
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

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    const bool has_input = hasAnyInput(msg);

    auto twist = geometry_msgs::msg::Twist();
    if (!has_input) {
      if (joy_no_trigger_send_zero < 6) {
        joy_no_trigger_send_zero++;
        RCLCPP_INFO(this->get_logger(),
                    "摇杆松开了，没有发零速度，发一个，让机器人停下！");
        publishVelocity(twist);
      }
      return;
    }
    joy_no_trigger_send_zero = 0;

    // 紧急停止 (A键)
    if (btn_stop_ >= 0 && btn_stop_ < static_cast<int>(msg->buttons.size())) {
      if (msg->buttons[btn_stop_] == 1) {
        publishVelocity(twist);
        return;
      }
    }

    // 速度档位切换：X上增档，Y下降档
    if (btn_speed_up_ >= 0 &&
        btn_speed_up_ < static_cast<int>(msg->buttons.size())) {
      if (msg->buttons[btn_speed_up_] == 1 && !speed_up_pressed_) {
        if (speed_level_ < 2) {
          speed_level_++;
          RCLCPP_INFO(this->get_logger(), "Speed UP -> level %d (linear=%.1f)",
                      speed_level_, getLinearScale());
        }
        speed_up_pressed_ = true;
      } else if (msg->buttons[btn_speed_up_] == 0) {
        speed_up_pressed_ = false;
      }
    }

    if (btn_speed_down_ >= 0 &&
        btn_speed_down_ < static_cast<int>(msg->buttons.size())) {
      if (msg->buttons[btn_speed_down_] == 1 && !speed_down_pressed_) {
        if (speed_level_ > 0) {
          speed_level_--;
          RCLCPP_INFO(this->get_logger(),
                      "Speed DOWN -> level %d (linear=%.1f)", speed_level_,
                      getLinearScale());
        }
        speed_down_pressed_ = true;
      } else if (msg->buttons[btn_speed_down_] == 0) {
        speed_down_pressed_ = false;
      }
    }

    if (axis_linear_ >= static_cast<int>(msg->axes.size()) ||
        axis_angular_ >= static_cast<int>(msg->axes.size())) {
      return;
    }

    double linear_input = applyDeadzone(msg->axes[axis_linear_]);
    double angular_input = applyDeadzone(msg->axes[axis_angular_]);

    twist.linear.x = linear_input * getLinearScale();
    twist.angular.z = angular_input * getAngularScale();
    publishVelocity(twist);
  }

  int axis_linear_;
  int axis_angular_;
  double deadzone_ = 0.1;
  int btn_stop_;
  int btn_speed_up_;
  int btn_speed_down_;
  int speed_level_ = 1; // 默认中速 (0=低速, 1=中速, 2=高速)
  bool speed_up_pressed_ = false;
  bool speed_down_pressed_ = false;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  int joy_no_trigger_send_zero =
      10; // 摇杆松开时候发送0速度 是否发送过了，发送过了，后面就不发了
  std::string cmd_topic_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamepadTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
