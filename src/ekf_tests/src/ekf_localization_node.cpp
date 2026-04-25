#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "ekf_tests/ekf_localization.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class EKFLocalizationNode : public rclcpp::Node {
public:
    EKFLocalizationNode() : Node("ekf_localization"), last_odom_time_(this->now()) {
        RCLCPP_INFO(this->get_logger(), "EKF Localization Node Started");

        // 声明参数：观测话题名称（默认 /pose）
        this->declare_parameter<std::string>("pose_topic", "/pose");
        std::string pose_topic = this->get_parameter("pose_topic").as_string();

        // 订阅里程计（预测）
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                double v = msg->twist.twist.linear.x;
                double omega = msg->twist.twist.angular.z;
                rclcpp::Time now = this->now();
                double dt = (now - last_odom_time_).seconds();
                if (dt > 0 && dt < 1.0) {
                    ekf_.predict(v, omega, dt);
                    RCLCPP_DEBUG(this->get_logger(), "Predicted: v=%.2f, omega=%.2f, dt=%.3f", v, omega, dt);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid dt=%.3f, skip predict", dt);
                }
                last_odom_time_ = now;
            });

        // 订阅绝对位姿观测（例如激光/视觉里程计、AMCL 等）→ 替代 GPS
        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                ekf_.update_landmark(msg->pose.pose.position.x, msg->pose.pose.position.y);
                RCLCPP_INFO(this->get_logger(), "Updated with AMCL: (%.2f, %.2f)", 
                            msg->pose.pose.position.x, msg->pose.pose.position.y);
            });


        // 可选：订阅 IMU 数据用于增强预测（若需要可扩展，此处暂不实现）
        // imu_sub_ = ...

        // 发布融合结果
        pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ekf_odom", 10);

        // 定时器发布 EKF 位姿
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                         [this]() { publish_ekf_pose(); });
    }

private:
    void publish_ekf_pose() {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";
        msg.pose.pose.position.x = ekf_.x_(0);
        msg.pose.pose.position.y = ekf_.x_(1);
        tf2::Quaternion q;
        q.setRPY(0, 0, ekf_.x_(2));
        msg.pose.pose.orientation = tf2::toMsg(q);
        pose_pub_->publish(msg);
        
        static rclcpp::Time last_print = this->now();
        if ((this->now() - last_print).seconds() > 1.0) {
            RCLCPP_INFO(this->get_logger(), "EKF Pose: x=%.2f, y=%.2f, theta=%.2f°",
                        ekf_.x_(0), ekf_.x_(1), ekf_.x_(2)*180/M_PI);
            last_print = this->now();
        }
    }

    EKFLocalization ekf_;
    rclcpp::Time last_odom_time_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKFLocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}