#include <ceres/ceres.h>
#include <iostream>

// 平移残差：希望优化后的 (x,y) 接近目标值
struct TranslationResidual {
  TranslationResidual(double target_x, double target_y, double weight)
      : tx_(target_x), ty_(target_y), w_(weight) {}

  template <typename T>
  bool operator()(const T *const pose, T *residual) const {
    residual[0] = w_ * (pose[0] - T(tx_)); // x 误差
    residual[1] = w_ * (pose[1] - T(ty_)); // y 误差
    return true;
  }

  double tx_, ty_, w_;
};

// 旋转残差：希望优化后的角度接近目标角度
struct RotationResidual {
  RotationResidual(double target_angle, double weight)
      : ta_(target_angle), w_(weight) {}

  template <typename T>
  bool operator()(const T *const pose, T *residual) const {
    residual[0] = w_ * (pose[2] - T(ta_)); // θ 误差
    return true;
  }

  double ta_, w_;
};

int main() {
  // “真实”位姿（模拟我们想要恢复的理想值）
  double true_pose[3] = {3.0, 4.0, 0.5}; // x, y, θ

  // 初始估计（故意偏离真值）
  double init_pose[3] = {2.8, 4.2, 0.45};

  // 构建 Ceres 问题
  ceres::Problem problem;
  problem.AddParameterBlock(init_pose, 3);

  // 添加平移先验（权重 10，相当于强约束）
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<TranslationResidual, 2, 3>(
          new TranslationResidual(true_pose[0], true_pose[1], 10.0)),
      nullptr, init_pose);

  // 添加旋转先验（权重 40，更强约束）
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<RotationResidual, 1, 3>(
          new RotationResidual(true_pose[2], 40.0)),
      nullptr, init_pose);

  // 配置求解器
  ceres::Solver::Options options;
  options.max_num_iterations = 20;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true; // 打印迭代过程

  // 求解
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // 输出结果
  std::cout << summary.FullReport() << "\n";
  std::cout << "Final pose: (" << init_pose[0] << ", " << init_pose[1] << ", "
            << init_pose[2] << ")\n";
  std::cout << "Expected:   (" << true_pose[0] << ", " << true_pose[1] << ", "
            << true_pose[2] << ")\n";

  return 0;
}