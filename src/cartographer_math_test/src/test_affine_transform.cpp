#include <cmath>
#include <iostream>
#include <vector>

#include "Eigen/Core"
#include "Eigen/LU"
#include "Eigen/SVD"

using Point2d = Eigen::Vector3d;  // 齐次坐标 [x, y, 1]^T

// 打印 3x3 矩阵
void PrintMatrix(const Eigen::Matrix3d &M, const std::string &name) {
  std::cout << name << ":\n";
  for (int i = 0; i < 3; ++i) {
    std::cout << "  [";
    for (int j = 0; j < 3; ++j) {
      std::cout << (j == 0 ? "" : ", ") << M(i, j);
    }
    std::cout << "]\n";
  }
}

// 打印点集
void PrintPoints(const std::vector<Point2d> &pts, const std::string &name) {
  std::cout << name << ":\n";
  for (size_t i = 0; i < pts.size(); ++i) {
    std::cout << "  P" << i << " = [" << pts[i].x() << ", " << pts[i].y()
              << "]\n";
  }
}

// 齐次矩阵作用于点集
std::vector<Point2d> TransformPoints(const std::vector<Point2d> &pts,
                                     const Eigen::Matrix3d &T) {
  std::vector<Point2d> result;
  result.reserve(pts.size());
  for (const auto &p : pts) {
    result.push_back(T * p);
  }
  return result;
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  // 单位正方形，四个顶点（齐次坐标）
  std::vector<Point2d> square = {
      {0.0, 0.0, 1.0},  // 原点
      {1.0, 0.0, 1.0},  // 右下
      {1.0, 1.0, 1.0},  // 右上
      {0.0, 1.0, 1.0}   // 左上
  };

  std::cout << "========================================\n";
  std::cout << "=== 2D 仿射变换演示（3x3 齐次矩阵） ===\n";
  std::cout << "========================================\n\n";

  PrintPoints(square, "原始正方形（边长=1）");
  std::cout << "\n";

  // ========================================================================
  // 1. 平移
  // ========================================================================
  std::cout << "【1. 平移】P' = T * P\n";
  std::cout << "  矩阵形式：T = [I | t]\n\n";

  Eigen::Matrix3d T_translate = Eigen::Matrix3d::Identity();
  T_translate(0, 2) = 2.0;  // tx = 2
  T_translate(1, 2) = 1.0;  // ty = 1

  PrintMatrix(T_translate, "平移矩阵 T(2,1)");
  auto sq_translated = TransformPoints(square, T_translate);
  PrintPoints(sq_translated, "平移后");
  std::cout << "  数学：x' = x + 2, y' = y + 1\n\n";

  // ========================================================================
  // 2. 旋转（绕原点逆时针 45°）
  // ========================================================================
  std::cout << "【2. 旋转】P' = R * P\n";
  std::cout << "  矩阵形式：R = [cosθ -sinθ 0; sinθ cosθ 0; 0 0 1]\n\n";

  double theta = M_PI / 4.0;  // 45°
  Eigen::Matrix3d T_rotate = Eigen::Matrix3d::Identity();
  T_rotate(0, 0) = std::cos(theta);
  T_rotate(0, 1) = -std::sin(theta);
  T_rotate(1, 0) = std::sin(theta);
  T_rotate(1, 1) = std::cos(theta);

  PrintMatrix(T_rotate, "旋转矩阵 R(45°)");
  auto sq_rotated = TransformPoints(square, T_rotate);
  PrintPoints(sq_rotated, "旋转后");
  std::cout << "  数学：x' = x*cosθ - y*sinθ, y' = x*sinθ + y*cosθ\n\n";

  // ========================================================================
  // 3. 缩放
  // ========================================================================
  std::cout << "【3. 缩放】P' = S * P\n";
  std::cout << "  矩阵形式：S = diag(sx, sy, 1)\n\n";

  Eigen::Matrix3d T_scale = Eigen::Matrix3d::Identity();
  T_scale(0, 0) = 2.0;  // sx = 2
  T_scale(1, 1) = 0.5;  // sy = 0.5

  PrintMatrix(T_scale, "缩放矩阵 S(2, 0.5)");
  auto sq_scaled = TransformPoints(square, T_scale);
  PrintPoints(sq_scaled, "缩放后");
  std::cout << "  数学：x' = 2*x, y' = 0.5*y\n\n";

  // ========================================================================
  // 4. 错切（Shear，X 方向，因子 k=0.5）
  // ========================================================================
  std::cout << "【4. 错切】P' = Sh * P\n";
  std::cout << "  矩阵形式：Sh = [1 k 0; 0 1 0; 0 0 1]\n\n";

  double k = 0.5;
  Eigen::Matrix3d T_shear = Eigen::Matrix3d::Identity();
  T_shear(0, 1) = k;

  PrintMatrix(T_shear, "错切矩阵 Shx(0.5)");
  auto sq_sheared = TransformPoints(square, T_shear);
  PrintPoints(sq_sheared, "错切后");
  std::cout << "  数学：x' = x + 0.5*y, y' = y\n";
  std::cout << "  几何：y 越大，x 偏移越大，形成推倒效果\n\n";

  // ========================================================================
  // 5. 复合变换：先旋转后平移（R 然后 T）
  // ========================================================================
  std::cout << "【5. 复合变换】P' = T * R * P（先旋转45°，再平移(2,1)）\n";
  std::cout << "  注意：矩阵乘法从右往左作用\n\n";

  Eigen::Matrix3d T_composite = T_translate * T_rotate;
  PrintMatrix(T_composite, "复合矩阵 T * R");
  auto sq_composite = TransformPoints(square, T_composite);
  PrintPoints(sq_composite, "复合变换后");
  std::cout << "  对比：先旋转后平移 ≠ 先平移后旋转\n\n";

  // ========================================================================
  // 6. 矩阵基本运算：转置、行列式、逆
  // ========================================================================
  std::cout << "========================================\n";
  std::cout << "=== 矩阵代数运算验证 ===\n";
  std::cout << "========================================\n\n";

  // 转置
  std::cout << "【转置】R^T:\n";
  PrintMatrix(T_rotate.transpose(), "旋转矩阵的转置");

  // 行列式
  std::cout << "\n【行列式】det(R) = " << T_rotate.determinant() << "\n";
  std::cout << "  旋转矩阵 det = 1（保面积）\n";
  std::cout << "  缩放矩阵 det = " << T_scale.determinant() << " (= sx*sy = 2*0.5)\n\n";

  // 逆矩阵
  std::cout << "【逆矩阵】R^{-1}:\n";
  PrintMatrix(T_rotate.inverse(), "旋转矩阵的逆");

  // 正交性验证：R^T * R = I
  std::cout << "\n【正交性验证】R^T * R:\n";
  PrintMatrix(T_rotate.transpose() * T_rotate, "R^T * R");
  std::cout << "  结果 ≈ I，说明旋转矩阵是正交矩阵：R^{-1} = R^T\n\n";

  // 逆矩阵验证：R * R^{-1} = I
  std::cout << "【逆矩阵验证】R * R^{-1}:\n";
  PrintMatrix(T_rotate * T_rotate.inverse(), "R * R^{-1}");
  std::cout << "  结果 ≈ I\n\n";

  // 错切矩阵的逆
  std::cout << "【错切矩阵的逆】Sh^{-1}:\n";
  PrintMatrix(T_shear.inverse(), "Sh^{-1}");
  std::cout << "  数学：Sh = [1 k; 0 1] 的逆为 [1 -k; 0 1]\n";
  std::cout << "  验证：k=0.5，逆矩阵应为 [1 -0.5; 0 1]\n\n";

  // ========================================================================
  // 7. SVD 奇异值分解
  // ========================================================================
  std::cout << "========================================\n";
  std::cout << "=== SVD 奇异值分解 ===\n";
  std::cout << "========================================\n\n";

  std::cout << "【原理】任意矩阵 A 可分解为 A = U * Σ * V^T\n";
  std::cout << "  U    : 正交矩阵（左奇异向量，表示输出空间的旋转）\n";
  std::cout << "  Σ    : 对角矩阵（奇异值 σ₁, σ₂, ...，表示各轴缩放倍数）\n";
  std::cout << "  V^T  : 正交矩阵的转置（右奇异向量，表示输入空间的旋转）\n\n";
  std::cout << "  几何意义：任何线性变换 = 旋转(V^T) → 缩放(Σ) → 旋转(U)\n\n";

  // 构造一个任意 2x2 矩阵（去掉齐次坐标的第三行/列）
  Eigen::Matrix2d A;
  A << 3.0, 1.0,
       1.0, 2.0;
  std::cout << "【测试矩阵】A =\n";
  std::cout << "  [" << A(0,0) << ", " << A(0,1) << "]\n";
  std::cout << "  [" << A(1,0) << ", " << A(1,1) << "]\n\n";

  // Eigen 的 SVD
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix2d U = svd.matrixU();
  Eigen::Matrix2d V = svd.matrixV();
  Eigen::Vector2d sigma = svd.singularValues();

  std::cout << "【分解结果】\n";
  std::cout << "  奇异值 σ₁=" << sigma(0) << ", σ₂=" << sigma(1) << "\n";
  std::cout << "  → σ₁/σ₂ = " << sigma(0)/sigma(1) << "（比值越大，变换越【拉长】）\n\n";

  std::cout << "  U =\n";
  std::cout << "    [" << U(0,0) << ", " << U(0,1) << "]\n";
  std::cout << "    [" << U(1,0) << ", " << U(1,1) << "]\n\n";

  std::cout << "  Σ =\n";
  std::cout << "    [" << sigma(0) << ", 0]\n";
  std::cout << "    [0, " << sigma(1) << "]\n\n";

  std::cout << "  V^T =\n";
  Eigen::Matrix2d Vt = V.transpose();
  std::cout << "    [" << Vt(0,0) << ", " << Vt(0,1) << "]\n";
  std::cout << "    [" << Vt(1,0) << ", " << Vt(1,1) << "]\n\n";

  // 验证 A = U * Σ * V^T
  Eigen::Matrix2d Sigma = Eigen::Matrix2d::Zero();
  Sigma(0,0) = sigma(0);
  Sigma(1,1) = sigma(1);
  Eigen::Matrix2d A_reconstructed = U * Sigma * Vt;

  std::cout << "【验证】U * Σ * V^T =\n";
  std::cout << "    [" << A_reconstructed(0,0) << ", " << A_reconstructed(0,1) << "]\n";
  std::cout << "    [" << A_reconstructed(1,0) << ", " << A_reconstructed(1,1) << "]\n";
  std::cout << "  与原始 A 的差范数 = " << (A_reconstructed - A).norm() << " ✅\n\n";

  // 对正方形做 A 变换，观察几何效果
  std::cout << "【几何演示】用矩阵 A 变换单位正方形：\n";
  std::vector<Eigen::Vector3d> sq2d = {
      {0.0, 0.0, 1.0},
      {1.0, 0.0, 1.0},
      {1.0, 1.0, 1.0},
      {0.0, 1.0, 1.0}};

  std::cout << "  原始正方形:\n";
  for (size_t i = 0; i < sq2d.size(); ++i) {
    std::cout << "    P" << i << " = [" << sq2d[i].x() << ", " << sq2d[i].y() << "]\n";
  }

  std::cout << "\n  经过 A 变换后:\n";
  for (size_t i = 0; i < sq2d.size(); ++i) {
    Eigen::Vector2d p(sq2d[i].x(), sq2d[i].y());
    Eigen::Vector2d p2 = A * p;
    std::cout << "    P" << i << "' = [" << p2.x() << ", " << p2.y() << "]\n";
  }
  std::cout << "\n  几何解读：\n";
  std::cout << "    1. V^T 先将正方形旋转到主轴方向\n";
  std::cout << "    2. Σ 沿主轴分别缩放 " << sigma(0) << " 倍和 " << sigma(1) << " 倍\n";
  std::cout << "    3. U 再将结果旋转到最终方向\n\n";

  std::cout << "========================================\n";
  std::cout << "=== 测试结束 ===\n";
  std::cout << "========================================\n";

  return 0;
}
