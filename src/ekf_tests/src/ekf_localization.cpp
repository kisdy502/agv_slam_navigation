#include "ekf_tests/ekf_localization.hpp"

EKFLocalization::EKFLocalization() {
    // 初始化状态
    x_ = Eigen::Vector3d::Zero();
    
    // 初始化协方差矩阵（初始不确定性）
    P_ = Eigen::Matrix3d::Identity() * 0.1;
}

Eigen::Matrix3d EKFLocalization::compute_Q(double dt) {
    // 过程噪声协方差矩阵 Q
    Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
    Q(0, 0) = q_v_ * q_v_ * dt * dt;
    Q(1, 1) = q_v_ * q_v_ * dt * dt;
    Q(2, 2) = q_omega_ * q_omega_ * dt * dt;
    return Q;
}

Eigen::Matrix<double, 2, 3> EKFLocalization::compute_H() {
    // 观测雅可比矩阵：直接观测 x, y
    Eigen::Matrix<double, 2, 3> H;
    H << 1, 0, 0,
         0, 1, 0;
    return H;
}

void EKFLocalization::predict(double v, double omega, double dt) {
    // 运动模型雅可比矩阵 F
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -v * sin(x_(2)) * dt;
    F(1, 2) =  v * cos(x_(2)) * dt;
    
    // 过程噪声
    Eigen::Matrix3d Q = compute_Q(dt);
    
    // 状态更新（运动模型）
    x_(0) += v * cos(x_(2)) * dt;
    x_(1) += v * sin(x_(2)) * dt;
    x_(2) += omega * dt;
    
    // 协方差更新
    P_ = F * P_ * F.transpose() + Q;
}

void EKFLocalization::update_landmark(double z_x, double z_y) {
    // 预测的观测值
    Eigen::Vector2d z_pred;
    z_pred(0) = x_(0);
    z_pred(1) = x_(1);
    
    // 观测雅可比
    Eigen::Matrix<double, 2, 3> H = compute_H();
    
    // 观测噪声协方差
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 0.1;
    
    // 新息（观测残差）
    Eigen::Vector2d y = Eigen::Vector2d(z_x, z_y) - z_pred;
    
    // 新息协方差
    Eigen::Matrix2d S = H * P_ * H.transpose() + R;
    
    // 卡尔曼增益
    Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S.inverse();
    
    // 状态更新
    x_ += K * y;
    
    // 协方差更新
    P_ = (Eigen::Matrix3d::Identity() - K * H) * P_;
}
