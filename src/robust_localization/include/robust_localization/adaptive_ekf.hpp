// include/robust_localization/adaptive_ekf.hpp
#ifndef ROBUST_LOCALIZATION__ADAPTIVE_EKF_HPP_
#define ROBUST_LOCALIZATION__ADAPTIVE_EKF_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "degeneracy_detector.hpp"

namespace robust_localization {

struct EKFState {
    Eigen::Vector3d x;      // [x, y, theta]
    Eigen::Matrix3d P;      // 协方差
    rclcpp::Time timestamp;
};

struct SensorConfig {
    double base_noise_x;
    double base_noise_y;
    double base_noise_theta;
    double degeneracy_scale;  // 退化时噪声放大倍数
    double min_weight;        // 退化时最小权重
};

class AdaptiveEKF {
public:
    AdaptiveEKF();
    
    // 初始化
    void initialize(const geometry_msgs::msg::PoseWithCovarianceStamped& initial_pose);
    
    // 预测步（IMU+轮式里程计）
    void predict(const nav_msgs::msg::Odometry& odom, const rclcpp::Time& now);
    void predictIMU(const sensor_msgs::msg::Imu& imu, double dt);
    
    // 更新步（激光），传入退化报告以自适应调整
    void updateLaser(
        const geometry_msgs::msg::PoseWithCovarianceStamped& laser_pose,
        const DegeneracyReport& degeneracy);
    
    // 获取当前状态
    EKFState getState() const { return state_; }
    
    // 配置
    void setOdomConfig(const SensorConfig& config) { odom_config_ = config; }
    void setLaserConfig(const SensorConfig& config) { laser_config_ = config; }
    void setIMUConfig(const SensorConfig& config) { imu_config_ = config; }

private:
    // 根据退化报告调整激光噪声
    Eigen::Matrix3d adaptLaserNoise(const DegeneracyReport& degeneracy);
    
    // 计算过程噪声
    Eigen::Matrix3d computeProcessNoise(double dt, double linear_vel, double angular_vel);
    
    EKFState state_;
    SensorConfig odom_config_;
    SensorConfig laser_config_;
    SensorConfig imu_config_;
    
    // 运动模型参数
    double alpha1_ = 0.2;  // 旋转->旋转噪声
    double alpha2_ = 0.2;  // 旋转->平移噪声
    double alpha3_ = 0.2;  // 平移->平移噪声
    double alpha4_ = 0.2;  // 平移->旋转噪声
    
    bool initialized_ = false;
};

} // namespace robust_localization

#endif // ROBUST_LOCALIZATION__ADAPTIVE_EKF_HPP_