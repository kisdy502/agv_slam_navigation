#ifndef EKF_LOCALIZATION_HPP
#define EKF_LOCALIZATION_HPP

#include <Eigen/Dense>

class EKFLocalization {
public:
    EKFLocalization();

    // 状态: [x, y, theta]
    Eigen::Vector3d x_;
    
    // 协方差矩阵
    Eigen::Matrix3d P_;

    // 运动模型预测
    void predict(double v, double omega, double dt);

    // 路标观测更新
    void update_landmark(double z_x, double z_y);

private:
    // 过程噪声协方差矩阵
    Eigen::Matrix3d compute_Q(double dt);

    // 观测雅可比矩阵
    Eigen::Matrix<double, 2, 3> compute_H();

    // 过程噪声参数
    double q_v_ = 0.1;     // 速度噪声
    double q_omega_ = 0.05; // 角速度噪声
};

#endif // EKF_LOCALIZATION_HPP
