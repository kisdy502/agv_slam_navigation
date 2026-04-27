#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class GamepadTeleopNode : public rclcpp::Node {
public:
  GamepadTeleopNode() : Node("gamepad_teleop_node") {
    this->declare_parameter<int>("axis_linear", 1);
    this->declare_parameter<int>("axis_angular", 0);
    this->declare_parameter<double>("deadzone", 0.05);
    this->declare_parameter<int>("btn_stop", 1);
    this->declare_parameter<int>("btn_speed_up", 0);   // X
    this->declare_parameter<int>("btn_speed_down", 3); // Y

    this->get_parameter("axis_linear", axis_linear_);
    this->get_parameter("axis_angular", axis_angular_);
    this->get_parameter("deadzone", deadzone_);
    this->get_parameter("btn_stop", btn_stop_);
    this->get_parameter("btn_speed_up", btn_speed_up_);
    this->get_parameter("btn_speed_down", btn_speed_down_);

    RCLCPP_INFO(
        this->get_logger(),
        "Gamepad Teleop started: speed_level=%d (low=0.3, mid=0.7, high=1.2)",
        speed_level_);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&GamepadTeleopNode::joyCallback, this,
                  std::placeholders::_1));

    vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  void publishVelocity(const geometry_msgs::msg::Twist &twist) {
    vel_pub_->publish(twist);
    // 检查是否是零速度
    if (twist.linear.x == 0.0 && twist.linear.y == 0.0 &&
        twist.linear.z == 0.0 && twist.angular.x == 0.0 &&
        twist.angular.y == 0.0 && twist.angular.z == 0.0) {
      RCLCPP_INFO(this->get_logger(), "发送停止速度控制！，第%d次。",
                  stop_twist_count_);
    }
  }
  // 适配北通
  bool hasAnyInput(const sensor_msgs::msg::Joy::SharedPtr &msg) const {
    // 检查所有轴是否有超出死区的
    for (size_t i = 0; i < msg->axes.size(); i++) {
      // RCLCPP_INFO(this->get_logger(), "axes[i]%f,deadzone_:%f", msg->axes[i],deadzone_);
      if (i == 4 || i == 5) {
        continue;
      }
      if (std::abs(msg->axes[i]) > deadzone_) {
        return true;
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
    // RCLCPP_INFO(this->get_logger(), "has_input=%s, ", has_input? "YES" :
    // "NO");
    //  如果没有输入，且上一次已经发布了零速度，就不再继续发布
    if (!has_input) {
      // 构造零速度
      geometry_msgs::msg::Twist zero_twist;
      // 如果上一次发布的速度不是零，或者停止计数器未满，则发布零速并更新计数
      if (stop_twist_count_ < 5) {
        publishVelocity(zero_twist);
        stop_twist_count_++;
        return;
      }
      // 若已连续发布多次零速，则不再重复发布
      return;
    }
    // RCLCPP_INFO(this->get_logger(), "has_input=%s, ", has_input ? "YES" : "NO");
    stop_twist_count_ = 0; // 只要有输入，就重置停止计数器

    auto twist = geometry_msgs::msg::Twist();

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
  double deadzone_ = 0.05;
  int btn_stop_;
  int btn_speed_up_;
  int btn_speed_down_;
  int speed_level_ = 1; // 默认中速 (0=低速, 1=中速, 2=高速)
  bool speed_up_pressed_ = false;
  bool speed_down_pressed_ = false;
  int stop_twist_count_ = 100; // 连续发布停止命令的次数，初始值设置为100，防止没有动遥控器就发停止速度指令

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamepadTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
