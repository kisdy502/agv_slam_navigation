#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

class CmdVelMux : public rclcpp::Node {
public:
  CmdVelMux() : Node("cmd_vel_mux"), hasNav2Cmd_(false), hasDockingCmd_(false) {
    this->declare_parameter("timeout", 2.0);  // 2秒看门狗，够覆盖 nav2↔docking 切换间隙
    this->declare_parameter("default_source", "nav2");
    this->declare_parameter("nav2_input_topic", "/cmd_vel");
    this->declare_parameter("docking_input_topic", "/cmd_vel_docking");
    this->declare_parameter(
        "output_topic", "/ackermann_steering_controller/reference_unstamped");

    timeoutSec_ = this->get_parameter("timeout").as_double();
    activeSource_ = this->get_parameter("default_source").as_string();
    std::string nav2_input = this->get_parameter("nav2_input_topic").as_string();
    std::string docking_input = this->get_parameter("docking_input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();

    nav2Sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        nav2_input, 10,
        std::bind(&CmdVelMux::nav2CmdVelCallback, this, std::placeholders::_1));

    dockingSub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        docking_input, 10,
        std::bind(&CmdVelMux::dockingCmdVelCallback, this,
                  std::placeholders::_1));

    selectSub_ = this->create_subscription<std_msgs::msg::String>(
        "/mux/select", 10,
        std::bind(&CmdVelMux::selectCallback, this, std::placeholders::_1));

    outputPub_ =
        this->create_publisher<geometry_msgs::msg::Twist>(output_topic, 10);

    publishTimer_ =
        this->create_wall_timer(std::chrono::milliseconds(10),
                                std::bind(&CmdVelMux::publishLoop, this));

    RCLCPP_INFO(
        this->get_logger(),
        "CmdVelMux started. Default source: %s, Output: %s, Timeout: %.1fs",
        activeSource_.c_str(), output_topic.c_str(), timeoutSec_);
  }

private:
  void nav2CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    nav2Cmd_ = *msg;
    hasNav2Cmd_ = true;
    lastNav2Time_ = this->now();
    nav2TimedOut_ = false;  // 新数据到达，清除超时冷却标记
  }

  void dockingCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    dockingCmd_ = *msg;
    hasDockingCmd_ = true;
    lastDockingTime_ = this->now();
    dockingTimedOut_ = false;
  }

  void selectCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "nav2" || msg->data == "docking") {
      if (activeSource_ != msg->data) {
        RCLCPP_INFO(this->get_logger(), "Switching cmd_vel source: %s -> %s",
                    activeSource_.c_str(), msg->data.c_str());
        activeSource_ = msg->data;
      }
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Unknown source: %s. Use 'nav2' or 'docking'",
                  msg->data.c_str());
    }
  }

  void publishLoop() {
    geometry_msgs::msg::Twist output;
    auto now = this->now();
    const double cooldownSec = 5.0;  // 超时 5 秒后不再发零速，让遥控器接管

    if (activeSource_ == "nav2") {
      if (hasNav2Cmd_ && (now - lastNav2Time_).seconds() < timeoutSec_) {
        output = nav2Cmd_;
        nav2TimedOut_ = false;
      } else if (hasNav2Cmd_ && !nav2TimedOut_) {
        // 刚超时：记录时间，发零速保护
        nav2TimeoutStart_ = now;
        nav2TimedOut_ = true;
        RCLCPP_WARN(this->get_logger(),
                    "Nav2 cmd_vel timeout! Sending zero velocity.");
      } else if (nav2TimedOut_ &&
                 (now - nav2TimeoutStart_).seconds() < cooldownSec) {
        // 冷却期内：继续发零速
      } else {
        // 冷却期结束 → 停止发零速，遥控器接管
        return;
      }
    } else if (activeSource_ == "docking") {
      if (hasDockingCmd_ && (now - lastDockingTime_).seconds() < timeoutSec_) {
        output = dockingCmd_;
        dockingTimedOut_ = false;
      } else if (hasDockingCmd_ && !dockingTimedOut_) {
        dockingTimeoutStart_ = now;
        dockingTimedOut_ = true;
        RCLCPP_WARN(this->get_logger(),
                    "Docking cmd_vel timeout! Sending zero velocity.");
      } else if (dockingTimedOut_ &&
                 (now - dockingTimeoutStart_).seconds() < cooldownSec) {
        // 冷却期内继续发零速
      } else {
        return;  // 冷却结束，静默
      }
    }

    outputPub_->publish(output);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2Sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr dockingSub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selectSub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr outputPub_;
  rclcpp::TimerBase::SharedPtr publishTimer_;

  geometry_msgs::msg::Twist nav2Cmd_;
  geometry_msgs::msg::Twist dockingCmd_;
  bool hasNav2Cmd_;
  bool hasDockingCmd_;
  std::string activeSource_;
  double timeoutSec_;
  rclcpp::Time lastNav2Time_;
  rclcpp::Time lastDockingTime_;
  bool nav2TimedOut_ = false;
  bool dockingTimedOut_ = false;
  rclcpp::Time nav2TimeoutStart_;
  rclcpp::Time dockingTimeoutStart_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelMux>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}