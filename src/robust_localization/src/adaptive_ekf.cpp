// src/adaptive_ekf.cpp
#include "robust_localization/adaptive_ekf.hpp"
#include <cmath>

namespace robust_localization
{

    AdaptiveEKF::AdaptiveEKF() {}

    void AdaptiveEKF::initialize(const geometry_msgs::msg::PoseWithCovarianceStamped &initial_pose)
    {
        state_.x << initial_pose.pose.pose.position.x,
            initial_pose.pose.pose.position.y,
            atan2(initial_pose.pose.pose.orientation.z,
                  initial_pose.pose.pose.orientation.w) *
                2;

        // 从消息协方差初始化
        state_.P << initial_pose.pose.covariance[0], 0, 0,
            0, initial_pose.pose.covariance[7], 0,
            0, 0, initial_pose.pose.covariance[35];

        state_.timestamp = initial_pose.header.stamp;
        initialized_ = true;
    }

    void AdaptiveEKF::predict(const nav_msgs::msg::Odometry &odom, const rclcpp::Time &now)
    {
        if (!initialized_)
            return;

        double dt = (now - state_.timestamp).seconds();
        if (dt <= 0 || dt > 1.0)
        {
            state_.timestamp = now;
            return;
        }

        double v = odom.twist.twist.linear.x;
        double w = odom.twist.twist.angular.z;

        // 非线性运动模型（差速模型）
        double theta = state_.x(2);
        Eigen::Vector3d dx;

        if (fabs(w) > 1e-6)
        {
            double r = v / w;
            dx << r * (sin(theta + w * dt) - sin(theta)),
                r * (-cos(theta + w * dt) + cos(theta)),
                w * dt;
        }
        else
        {
            dx << v * dt * cos(theta),
                v * dt * sin(theta),
                0;
        }

        state_.x += dx;
        state_.x(2) = atan2(sin(state_.x(2)), cos(state_.x(2))); // 归一化角度

        // 计算雅可比
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        if (fabs(w) > 1e-6)
        {
            double r = v / w;
            F(0, 2) = r * (cos(theta + w * dt) - cos(theta));
            F(1, 2) = r * (sin(theta + w * dt) - sin(theta));
        }
        else
        {
            F(0, 2) = -v * dt * sin(theta);
            F(1, 2) = v * dt * cos(theta);
        }

        // 过程噪声
        Eigen::Matrix3d Q = computeProcessNoise(dt, v, w);

        state_.P = F * state_.P * F.transpose() + Q;
        state_.timestamp = now;
    }

    void AdaptiveEKF::predictIMU(const sensor_msgs::msg::Imu &imu, double dt)
    {
        // IMU 用于航向修正，这里简化处理
        // 实际应该做 IMU 预积分
        if (!initialized_)
            return;

        double w_imu = imu.angular_velocity.z;
        state_.x(2) += w_imu * dt;
        state_.x(2) = atan2(sin(state_.x(2)), cos(state_.x(2)));

        // IMU 噪声
        Eigen::Matrix3d Q_imu = Eigen::Matrix3d::Zero();
        Q_imu(2, 2) = imu_config_.base_noise_theta * dt;

        state_.P += Q_imu;
    }

    void AdaptiveEKF::updateLaser(
        const geometry_msgs::msg::PoseWithCovarianceStamped &laser_pose,
        const DegeneracyReport &degeneracy)
    {

        if (!initialized_)
            return;

        // 观测模型：直接位姿观测
        Eigen::Vector3d z;
        z << laser_pose.pose.pose.position.x,
            laser_pose.pose.pose.position.y,
            atan2(laser_pose.pose.pose.orientation.z,
                  laser_pose.pose.pose.orientation.w) *
                2;

        // 自适应调整激光噪声
        Eigen::Matrix3d R = adaptLaserNoise(degeneracy);

        // 观测矩阵（直接观测）
        Eigen::Matrix3d H = Eigen::Matrix3d::Identity();

        // 卡尔曼增益
        Eigen::Matrix3d S = H * state_.P * H.transpose() + R;
        Eigen::Matrix3d K = state_.P * H.transpose() * S.inverse();

        // 残差
        Eigen::Vector3d y = z - H * state_.x;
        y(2) = atan2(sin(y(2)), cos(y(2))); // 角度归一化

        // 更新
        state_.x += K * y;
        state_.x(2) = atan2(sin(state_.x(2)), cos(state_.x(2)));

        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        state_.P = (I - K * H) * state_.P;

        // 确保协方差对称正定
        state_.P = (state_.P + state_.P.transpose()) / 2;
    }

    Eigen::Matrix3d AdaptiveEKF::adaptLaserNoise(const DegeneracyReport &degeneracy)
    {
        Eigen::Matrix3d R = Eigen::Matrix3d::Zero();

        // 基础噪声
        R(0, 0) = laser_config_.base_noise_x;
        R(1, 1) = laser_config_.base_noise_y;
        R(2, 2) = laser_config_.base_noise_theta;

        if (!degeneracy.is_degenerate)
            return R;

        // 退化时根据方向放大噪声
        double scale = laser_config_.degeneracy_scale;

        if (degeneracy.degenerate_direction == "x" ||
            degeneracy.degenerate_direction == "mixed")
        {
            R(0, 0) *= scale;
        }
        if (degeneracy.degenerate_direction == "y" ||
            degeneracy.degenerate_direction == "mixed")
        {
            R(1, 1) *= scale;
        }
        if (degeneracy.degenerate_direction == "theta" ||
            degeneracy.degenerate_direction == "mixed")
        {
            R(2, 2) *= scale;
        }

        // 确保不低于最小权重
        R(0, 0) = std::max(R(0, 0), laser_config_.min_weight);
        R(1, 1) = std::max(R(1, 1), laser_config_.min_weight);
        R(2, 2) = std::max(R(2, 2), laser_config_.min_weight);

        return R;
    }

    Eigen::Matrix3d AdaptiveEKF::computeProcessNoise(double dt, double linear_vel, double angular_vel)
    {
        // 基于速度的过程噪声（差速模型标准形式）
        double v = linear_vel;
        double w = angular_vel;

        double v2 = v * v;
        double w2 = w * w;

        Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();

        // 简化过程噪声模型
        double sigma_v2 = alpha3_ * v2 + alpha4_ * w2;
        double sigma_w2 = alpha1_ * w2 + alpha2_ * v2;

        Q(0, 0) = sigma_v2 * dt;
        Q(1, 1) = sigma_v2 * dt;
        Q(2, 2) = sigma_w2 * dt;

        return Q;
    }

} // namespace robust_localization