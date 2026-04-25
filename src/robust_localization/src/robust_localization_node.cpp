// src/robust_localization_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "robust_localization/degeneracy_detector.hpp"
#include "robust_localization/adaptive_ekf.hpp"
#include "robust_localization/robust_scan_matcher.hpp"

namespace robust_localization
{

    class RobustLocalizationNode : public rclcpp::Node
    {
    public:
        RobustLocalizationNode() : Node("robust_localization_node")
        {
            // 参数声明
            declare_parameter("use_degeneracy_detector", true);
            declare_parameter("use_adaptive_ekf", true);
            declare_parameter("use_robust_matcher", true);
            declare_parameter("publish_tf", true);
            declare_parameter("odom_frame", "odom");
            declare_parameter("map_frame", "map");
            declare_parameter("base_frame", "base_link");

            // 初始化组件
            degeneracy_detector_ = std::make_unique<DegeneracyDetector>();
            ekf_ = std::make_unique<AdaptiveEKF>();
            scan_matcher_ = std::make_unique<RobustScanMatcher>();

            // 配置退化检测器
            double cond_thresh = declare_parameter("degeneracy.condition_number_threshold", 1000.0);
            double eigen_thresh = declare_parameter("degeneracy.min_eigenvalue_threshold", 0.001);
            double match_thresh = declare_parameter("degeneracy.valid_match_ratio_threshold", 0.3);
            degeneracy_detector_->setConditionNumberThreshold(cond_thresh);
            degeneracy_detector_->setMinEigenvalueThreshold(eigen_thresh);
            degeneracy_detector_->setValidMatchRatioThreshold(match_thresh);

            // 配置EKF
            SensorConfig laser_config;
            laser_config.base_noise_x = declare_parameter("laser.noise_x", 0.01);
            laser_config.base_noise_y = declare_parameter("laser.noise_y", 0.01);
            laser_config.base_noise_theta = declare_parameter("laser.noise_theta", 0.01);
            laser_config.degeneracy_scale = declare_parameter("laser.degeneracy_scale", 100.0);
            laser_config.min_weight = declare_parameter("laser.min_weight", 0.001);
            ekf_->setLaserConfig(laser_config);

            SensorConfig odom_config;
            odom_config.base_noise_x = declare_parameter("odom.noise_x", 0.1);
            odom_config.base_noise_y = declare_parameter("odom.noise_y", 0.1);
            odom_config.base_noise_theta = declare_parameter("odom.noise_theta", 0.1);
            ekf_->setOdomConfig(odom_config);

            // TF
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

            // 订阅
            sub_scan_front_ = create_subscription<sensor_msgs::msg::LaserScan>(
                "scan_front", 10,
                std::bind(&RobustLocalizationNode::scanFrontCallback, this, std::placeholders::_1));
            sub_scan_rear_ = create_subscription<sensor_msgs::msg::LaserScan>(
                "scan_rear", 10,
                std::bind(&RobustLocalizationNode::scanRearCallback, this, std::placeholders::_1));
            sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10,
                std::bind(&RobustLocalizationNode::odomCallback, this, std::placeholders::_1));
            sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
                "map", 1,
                std::bind(&RobustLocalizationNode::mapCallback, this, std::placeholders::_1));
            sub_initial_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "initialpose", 1,
                std::bind(&RobustLocalizationNode::initialPoseCallback, this, std::placeholders::_1));

            // 发布
            pub_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "robust_pose", 10);
            pub_degeneracy_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "degeneracy_status", 10); // 用协方差矩阵传递退化信息

            // 定时器：融合发布
            timer_ = create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(&RobustLocalizationNode::fusionTimerCallback, this));

            RCLCPP_INFO(get_logger(), "Robust Localization Node initialized");
        }

    private:
        void scanFrontCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            scan_front_ = msg;
            processLaser(msg, "front");
        }

        void scanRearCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            scan_rear_ = msg;
            processLaser(msg, "rear");
        }

        void processLaser(const sensor_msgs::msg::LaserScan::SharedPtr &scan, const std::string &source)
        {
            if (!map_ || !ekf_initialized_)
                return;

            // 获取当前EKF预测位姿作为初始猜测
            auto state = ekf_->getState();
            geometry_msgs::msg::Pose2D guess;
            guess.x = state.x(0);
            guess.y = state.x(1);
            guess.theta = state.x(2);

            // 鲁棒匹配
            if (get_parameter("use_robust_matcher").as_bool())
            {
                auto match_result = scan_matcher_->match(scan, map_, guess);

                if (match_result.success && match_result.inlier_ratio > 0.2)
                {
                    geometry_msgs::msg::PoseWithCovarianceStamped laser_pose;
                    laser_pose.header.stamp = scan->header.stamp;
                    laser_pose.header.frame_id = get_parameter("map_frame").as_string();
                    laser_pose.pose.pose.position.x = match_result.pose.x;
                    laser_pose.pose.pose.position.y = match_result.pose.y;
                    laser_pose.pose.pose.orientation.z = sin(match_result.pose.theta / 2);
                    laser_pose.pose.pose.orientation.w = cos(match_result.pose.theta / 2);

                    // 退化检测
                    DegeneracyReport degeneracy;
                    if (get_parameter("use_degeneracy_detector").as_bool())
                    {
                        geometry_msgs::msg::PoseWithCovarianceStamped current_est;
                        current_est.pose.pose = laser_pose.pose.pose;
                        degeneracy = degeneracy_detector_->analyze(scan, map_, current_est);

                        // 发布退化状态用于监控
                        publishDegeneracyStatus(degeneracy);
                    }

                    // 自适应EKF更新
                    if (get_parameter("use_adaptive_ekf").as_bool())
                    {
                        ekf_->updateLaser(laser_pose, degeneracy);
                    }
                    else
                    {
                        DegeneracyReport no_degeneracy;
                        ekf_->updateLaser(laser_pose, no_degeneracy);
                    }
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                         "Laser %s match failed: score=%.2f, inlier_ratio=%.2f",
                                         source.c_str(), match_result.score, match_result.inlier_ratio);
                }
            }
        }

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            odom_ = msg;
            if (ekf_initialized_)
            {
                ekf_->predict(*msg, msg->header.stamp);
            }
        }

        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            map_ = msg;
        }

        void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
            ekf_->initialize(*msg);
            ekf_initialized_ = true;
            RCLCPP_INFO(get_logger(), "EKF initialized with initial pose");
        }

        void fusionTimerCallback()
        {
            if (!ekf_initialized_)
                return;

            auto state = ekf_->getState();

            // 发布位姿
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = now();
            pose_msg.header.frame_id = get_parameter("map_frame").as_string();
            pose_msg.pose.pose.position.x = state.x(0);
            pose_msg.pose.pose.position.y = state.x(1);
            pose_msg.pose.pose.position.z = 0;
            pose_msg.pose.pose.orientation.z = sin(state.x(2) / 2);
            pose_msg.pose.pose.orientation.w = cos(state.x(2) / 2);

            // 协方差
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    pose_msg.pose.covariance[i * 6 + j] = state.P(i, j);
                }
            }

            pub_pose_->publish(pose_msg);

            // 发布 TF
            if (get_parameter("publish_tf").as_bool())
            {
                geometry_msgs::msg::TransformStamped tf;
                tf.header.stamp = now();
                tf.header.frame_id = get_parameter("map_frame").as_string();
                tf.child_frame_id = get_parameter("base_frame").as_string();
                tf.transform.translation.x = state.x(0);
                tf.transform.translation.y = state.x(1);
                tf.transform.rotation.z = sin(state.x(2) / 2);
                tf.transform.rotation.w = cos(state.x(2) / 2);
                tf_broadcaster_->sendTransform(tf);
            }
        }

        void publishDegeneracyStatus(const DegeneracyReport &degeneracy)
        {
            geometry_msgs::msg::PoseWithCovarianceStamped status;
            status.header.stamp = degeneracy.timestamp;
            status.header.frame_id = "degeneracy";

            // 用位姿传递退化信息（RViz可视化用）
            status.pose.pose.position.x = degeneracy.is_degenerate ? 1.0 : 0.0;
            status.pose.pose.position.y = degeneracy.condition_number;
            status.pose.pose.position.z = degeneracy.valid_match_ratio;

            // 用协方差传递特征值
            status.pose.covariance[0] = degeneracy.eigenvalues(0);          // x
            status.pose.covariance[7] = degeneracy.eigenvalues(1);          // y
            status.pose.covariance[14] = degeneracy.eigenvalues(2);         // theta
            status.pose.covariance[21] = degeneracy.eigenvalues.minCoeff(); // min eigen

            pub_degeneracy_->publish(status);
        }

        // 成员
        std::unique_ptr<DegeneracyDetector> degeneracy_detector_;
        std::unique_ptr<AdaptiveEKF> ekf_;
        std::unique_ptr<RobustScanMatcher> scan_matcher_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_front_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_rear_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;

        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_degeneracy_;

        rclcpp::TimerBase::SharedPtr timer_;

        sensor_msgs::msg::LaserScan::SharedPtr scan_front_;
        sensor_msgs::msg::LaserScan::SharedPtr scan_rear_;
        nav_msgs::msg::Odometry::SharedPtr odom_;
        nav_msgs::msg::OccupancyGrid::SharedPtr map_;

        bool ekf_initialized_ = false;
    };

} // namespace robust_localization

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robust_localization::RobustLocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}