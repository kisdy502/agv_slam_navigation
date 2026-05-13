#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class AckermannCalibTrajectoryNode : public rclcpp::Node {
public:
  AckermannCalibTrajectoryNode() : Node("ackermann_calib_trajectory_node") {
    // ---------- 参数声明 ----------
    this->declare_parameter<std::string>("mode", "circle");
    this->declare_parameter<double>("linear_velocity", 0.3);
    this->declare_parameter<double>("radius", 1.0);
    this->declare_parameter<double>("angular_velocity", 0.0);
    this->declare_parameter<int>("num_cycles", 3);
    this->declare_parameter<double>("publish_rate", 20.0);

    mode_ = this->get_parameter("mode").as_string();
    v_ = this->get_parameter("linear_velocity").as_double();
    radius_ = this->get_parameter("radius").as_double();
    w_override_ = this->get_parameter("angular_velocity").as_double();
    num_cycles_ = this->get_parameter("num_cycles").as_int();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    // ---------- 计算角速度 ----------
    if (std::abs(w_override_) > 1e-6) {
      w_ = w_override_;
    } else {
      if (std::abs(radius_) < 1e-6) {
        RCLCPP_ERROR(this->get_logger(), "radius too small, using w=0");
        w_ = 0.0;
      } else {
        w_ = v_ / radius_;
      }
    }

    if (std::abs(w_) < 1e-6) {
      RCLCPP_ERROR(this->get_logger(), "angular velocity is zero, aborting");
      rclcpp::shutdown();
      return;
    }

    half_cycle_duration_ = M_PI / std::abs(w_);
    full_cycle_duration_ = 2.0 * M_PI / std::abs(w_);

    RCLCPP_INFO(this->get_logger(),
                "Mode=%s, v=%.2f m/s, w=%.3f rad/s, radius=%.2f m, num_cycles=%d",
                mode_.c_str(), v_, w_, radius_, num_cycles_);
    RCLCPP_INFO(this->get_logger(),
                "Half-cycle=%.1fs, Full-cycle=%.1fs",
                half_cycle_duration_, full_cycle_duration_);

    // ---------- 发布者与定时器 ----------
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
    timer_ = this->create_wall_timer(
        period, std::bind(&AckermannCalibTrajectoryNode::timerCallback, this));

    // ---------- 初始化状态机 ----------
    current_cycle_ = 0;
    state_start_time_ = this->now();

    if (mode_ == "circle") {
      state_ = State::CIRCLE;
      RCLCPP_INFO(this->get_logger(), ">>> Starting CIRCLE test");
      publishTwist(v_, w_);
    } else if (mode_ == "figure8") {
      state_ = State::FIG8_LEFT;
      RCLCPP_INFO(this->get_logger(), ">>> Starting FIGURE8 test");
      publishTwist(v_, w_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown mode '%s', use 'circle' or 'figure8'", mode_.c_str());
      rclcpp::shutdown();
    }
  }

private:
  enum class State { CIRCLE, FIG8_LEFT, FIG8_RIGHT, DONE };

  State state_ = State::DONE;
  rclcpp::Time state_start_time_;
  int current_cycle_ = 0;

  std::string mode_;
  double v_ = 0.0;
  double w_ = 0.0;
  double radius_ = 0.0;
  double w_override_ = 0.0;
  int num_cycles_ = 3;
  double half_cycle_duration_ = 0.0;
  double full_cycle_duration_ = 0.0;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publishTwist(double v, double w) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = v;
    msg.angular.z = w;
    cmd_pub_->publish(msg);
  }

  void publishStop() {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    cmd_pub_->publish(msg);
  }

  void transitionTo(State new_state, const rclcpp::Time& now, const std::string& text) {
    state_ = new_state;
    state_start_time_ = now;
    RCLCPP_INFO(this->get_logger(), "%s", text.c_str());
  }

  void timerCallback() {
    auto now = this->now();
    double elapsed = (now - state_start_time_).seconds();

    switch (state_) {
      case State::CIRCLE: {
        if (elapsed >= full_cycle_duration_) {
          current_cycle_++;
          if (current_cycle_ >= num_cycles_) {
            transitionTo(State::DONE, now, "CIRCLE test finished");
            publishStop();
          } else {
            state_start_time_ = now;
            RCLCPP_INFO(this->get_logger(), "Cycle %d/%d completed", current_cycle_, num_cycles_);
            publishTwist(v_, w_);
          }
        } else {
          publishTwist(v_, w_);
        }
        break;
      }
      case State::FIG8_LEFT: {
        if (elapsed >= full_cycle_duration_) {
          transitionTo(State::FIG8_RIGHT, now, "Switching to FIG8_RIGHT");
          publishTwist(v_, -w_);
        } else {
          publishTwist(v_, w_);
        }
        break;
      }
      case State::FIG8_RIGHT: {
        if (elapsed >= full_cycle_duration_) {
          current_cycle_++;
          if (current_cycle_ >= num_cycles_) {
            transitionTo(State::DONE, now, "FIGURE8 test finished");
            publishStop();
          } else {
            transitionTo(State::FIG8_LEFT, now, "Switching to FIG8_LEFT");
            publishTwist(v_, w_);
          }
        } else {
          publishTwist(v_, -w_);
        }
        break;
      }
      case State::DONE: {
        publishStop();
        break;
      }
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannCalibTrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
