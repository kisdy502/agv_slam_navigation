#include <cmath>
#include <iostream>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

// ============================================================================
// 雷达极坐标 → 笛卡尔坐标 转换测试
//
// 数学原理：
//   2D激光雷达输出的原始数据是极坐标 (r, θ)：
//     r   = 激光束测得的距离（range）
//     θ   = 激光束的水平角度（angle），通常以雷达中心为原点，
//           正前方为 0°，逆时针为正
//
//   极坐标转笛卡尔坐标（雷达自身坐标系 lidar_frame）：
//     x = r * cos(θ)
//     y = r * sin(θ)
//     z = 0  （2D雷达默认水平安装，所有点在同一水平面）
//
//   坐标系变换（lidar_frame → base_link）：
//     雷达安装在机器人底盘(base_link)上，存在一个固定的安装位姿 T_base_lidar：
//       P_base = T_base_lidar * P_lidar
//     其中 T_base_lidar = [R | t]，R 是旋转矩阵，t 是平移向量
//
//   总公式：
//     P_base = R * [r*cos(θ), r*sin(θ), 0]^T + t
// ============================================================================

// 模拟的单个雷达点（极坐标原始数据）
struct PolarPoint {
  double range;   // 距离 (m)
  double angle;   // 角度 (rad)，逆时针为正，前方为0
};

// 笛卡尔坐标点（3D）
struct CartesianPoint {
  double x, y, z;
};

// 打印极坐标点
void PrintPolarPoint(const std::string &name, const PolarPoint &p) {
  std::cout << name << ": range=" << p.range
            << " m, angle=" << p.angle * 180.0 / M_PI << " deg\n";
}

// 打印笛卡尔坐标点
void PrintCartesianPoint(const std::string &name, const CartesianPoint &p) {
  std::cout << name << ": [" << p.x << ", " << p.y << ", " << p.z << "]\n";
}

// 打印齐次变换矩阵
void PrintTransform2D(const std::string &name,
                      const Eigen::Matrix3d &T) {
  std::cout << name << " (2D齐次变换矩阵 T = [R | t]):\n";
  std::cout << "  " << T(0, 0) << "  " << T(0, 1) << "  |  " << T(0, 2)
            << "\n";
  std::cout << "  " << T(1, 0) << "  " << T(1, 1) << "  |  " << T(1, 2)
            << "\n";
  std::cout << "  " << T(2, 0) << "  " << T(2, 1) << "  |  " << T(2, 2)
            << "\n";
}

// ============================================================================
// 核心函数1：单点极坐标 → 笛卡尔坐标（雷达自身坐标系）
// ============================================================================
CartesianPoint PolarToCartesian(const PolarPoint &p) {
  CartesianPoint cp;
  cp.x = p.range * std::cos(p.angle);
  cp.y = p.range * std::sin(p.angle);
  cp.z = 0.0;
  return cp;
}

// ============================================================================
// 核心函数2：雷达坐标系下的点 → base_link坐标系下的点
//
// 数学：P_base = R * P_lidar + t
//
// T_base_lidar 的含义：
//   描述的是 "lidar坐标系" 相对于 "base_link坐标系" 的位姿
//   即：把 lidar坐标系 中的点，变换到 base_link坐标系 中
// ============================================================================
CartesianPoint TransformPointToBaseLink(
    const CartesianPoint &p_lidar,
    const Eigen::Matrix3d &T_base_lidar) {
  Eigen::Vector3d p(p_lidar.x, p_lidar.y, p_lidar.z);
  Eigen::Vector3d p_base = T_base_lidar * p;
  return {p_base.x(), p_base.y(), p_base.z()};
}

// ============================================================================
// 核心函数3：批量转换一整帧点云
// ============================================================================
std::vector<CartesianPoint> ScanToCartesianInBaseLink(
    const std::vector<PolarPoint> &scan,
    const Eigen::Matrix3d &T_base_lidar) {
  std::vector<CartesianPoint> points;
  points.reserve(scan.size());

  for (const auto &p : scan) {
    // 步骤1：极坐标 → 笛卡尔（lidar坐标系）
    CartesianPoint cp = PolarToCartesian(p);
    // 步骤2：lidar坐标系 → base_link坐标系
    CartesianPoint cp_base = TransformPointToBaseLink(cp, T_base_lidar);
    points.push_back(cp_base);
  }
  return points;
}

// ============================================================================
// 主函数
// ============================================================================
int main(int argc, char **argv) {
  std::cout << "========================================\n";
  std::cout << "=== 雷达极坐标 → 笛卡尔坐标变换测试 ===\n";
  std::cout << "========================================\n\n";

  // ========================================================================
  // 第一部分：单点转换示例（数学原理验证）
  // ========================================================================
  std::cout << "【第一部分】单点极坐标 → 笛卡尔坐标\n";
  std::cout << "----------------------------------------\n";

  // 示例：雷达正前方 5 米处有一个障碍物
  PolarPoint p1{5.0, 0.0};  // range=5m, angle=0°
  PrintPolarPoint("  输入点 P1", p1);

  CartesianPoint cp1 = PolarToCartesian(p1);
  PrintCartesianPoint("  笛卡尔坐标 (lidar系)", cp1);
  std::cout << "  → 数学验证：x = 5*cos(0) = 5, y = 5*sin(0) = 0 ✅\n\n";

  // 示例：雷达左前方 45°，距离 3 米
  PolarPoint p2{3.0, M_PI / 4.0};  // range=3m, angle=45°
  PrintPolarPoint("  输入点 P2", p2);

  CartesianPoint cp2 = PolarToCartesian(p2);
  PrintCartesianPoint("  笛卡尔坐标 (lidar系)", cp2);
  std::cout << "  → 数学验证：x = 3*cos(45°) ≈ " << 3.0 * std::cos(M_PI / 4.0)
            << ", y = 3*sin(45°) ≈ " << 3.0 * std::sin(M_PI / 4.0) << " ✅\n\n";

  // 示例：雷达正左方 90°，距离 2 米
  PolarPoint p3{2.0, M_PI / 2.0};  // range=2m, angle=90°
  PrintPolarPoint("  输入点 P3", p3);

  CartesianPoint cp3 = PolarToCartesian(p3);
  PrintCartesianPoint("  笛卡尔坐标 (lidar系)", cp3);
  std::cout << "  → 数学验证：x = 2*cos(90°) = 0, y = 2*sin(90°) = 2 ✅\n\n";

  // 示例：雷达右后方 135°，距离 4 米
  PolarPoint p4{4.0, -3.0 * M_PI / 4.0};  // range=4m, angle=-135°
  PrintPolarPoint("  输入点 P4", p4);

  CartesianPoint cp4 = PolarToCartesian(p4);
  PrintCartesianPoint("  笛卡尔坐标 (lidar系)", cp4);
  std::cout << "  → 数学验证：x = 4*cos(-135°) ≈ "
            << 4.0 * std::cos(-3.0 * M_PI / 4.0)
            << ", y = 4*sin(-135°) ≈ "
            << 4.0 * std::sin(-3.0 * M_PI / 4.0) << " ✅\n\n";

  // ========================================================================
  // 第二部分：坐标系变换（lidar → base_link）
  // ========================================================================
  std::cout << "【第二部分】坐标系变换：lidar_frame → base_link\n";
  std::cout << "----------------------------------------\n";

  // 假设雷达安装在机器人底盘(base_link)上：
  //   - 安装位置：底盘前方 0.2m，高度 0.3m（相对于base_link原点）
  //   - 安装角度：与底盘朝向一致（无旋转）
  //
  // 在2D平面内，用3x3齐次变换矩阵表示：
  //   T = | R  t |
  //       | 0  1 |
  //
  // 其中 R 是2x2旋转矩阵，t 是2x1平移向量

  std::cout << "  假设雷达安装参数（相对于base_link）：\n";
  std::cout << "    → 安装位置：base_link前方 0.2m，高度 0.3m\n";
  std::cout << "    → 安装角度：与base_link朝向一致（偏航角 0°）\n\n";

  // 构建齐次变换矩阵 T_base_lidar
  // 含义：将 lidar坐标系 中的点变换到 base_link坐标系
  double lidar_x = 0.2;   // lidar在base_link中的x位置
  double lidar_y = 0.0;   // lidar在base_link中的y位置
  double lidar_yaw = 0.0; // lidar相对于base_link的偏航角

  Eigen::Matrix3d T_base_lidar = Eigen::Matrix3d::Identity();
  T_base_lidar(0, 0) = std::cos(lidar_yaw);
  T_base_lidar(0, 1) = -std::sin(lidar_yaw);
  T_base_lidar(1, 0) = std::sin(lidar_yaw);
  T_base_lidar(1, 1) = std::cos(lidar_yaw);
  T_base_lidar(0, 2) = lidar_x;
  T_base_lidar(1, 2) = lidar_y;

  PrintTransform2D("  T_base_lidar", T_base_lidar);
  std::cout << "  → 含义：P_base = T_base_lidar * P_lidar\n\n";

  // 转换前面那几个点到 base_link 坐标系
  std::cout << "  将前面4个点变换到 base_link 坐标系：\n";
  CartesianPoint cp1_base = TransformPointToBaseLink(cp1, T_base_lidar);
  PrintCartesianPoint("    P1_base (正前方5m)", cp1_base);
  std::cout << "      → x = 5 + 0.2 = 5.2, y = 0 ✅\n";

  CartesianPoint cp2_base = TransformPointToBaseLink(cp2, T_base_lidar);
  PrintCartesianPoint("    P2_base (左前45°, 3m)", cp2_base);
  std::cout << "      → x = 3*cos(45°) + 0.2 ≈ " << cp2_base.x
            << ", y = 3*sin(45°) ≈ " << cp2_base.y << " ✅\n";

  CartesianPoint cp3_base = TransformPointToBaseLink(cp3, T_base_lidar);
  PrintCartesianPoint("    P3_base (正左方2m)", cp3_base);
  std::cout << "      → x = 0 + 0.2 = 0.2, y = 2 ✅\n";

  CartesianPoint cp4_base = TransformPointToBaseLink(cp4, T_base_lidar);
  PrintCartesianPoint("    P4_base (右后135°, 4m)", cp4_base);
  std::cout << "      → x = 4*cos(-135°) + 0.2 ≈ " << cp4_base.x
            << ", y = 4*sin(-135°) ≈ " << cp4_base.y << " ✅\n\n";

  // ========================================================================
  // 第三部分：安装角度偏移的情况（雷达安装时有旋转）
  // ========================================================================
  std::cout << "【第三部分】雷达安装时有旋转（偏航角30°）\n";
  std::cout << "----------------------------------------\n";

  std::cout << "  假设雷达安装参数（相对于base_link）：\n";
  std::cout << "    → 安装位置：base_link前方 0.2m\n";
  std::cout << "    → 安装角度：相对于base_link偏航 30°\n";
  std::cout << "    → 场景：雷达向前安装但朝右偏了30°（比如为了覆盖右侧盲区）\n\n";

  double lidar_yaw2 = M_PI / 6.0;  // 30°
  Eigen::Matrix3d T_base_lidar2 = Eigen::Matrix3d::Identity();
  T_base_lidar2(0, 0) = std::cos(lidar_yaw2);
  T_base_lidar2(0, 1) = -std::sin(lidar_yaw2);
  T_base_lidar2(1, 0) = std::sin(lidar_yaw2);
  T_base_lidar2(1, 1) = std::cos(lidar_yaw2);
  T_base_lidar2(0, 2) = lidar_x;
  T_base_lidar2(1, 2) = lidar_y;

  PrintTransform2D("  T_base_lidar (偏航30°)", T_base_lidar2);

  // 重新转换 P1（雷达正前方5m → 在base_link中看是右前方30°方向）
  CartesianPoint cp1_base2 = TransformPointToBaseLink(cp1, T_base_lidar2);
  PrintCartesianPoint("    P1_base (雷达正前方5m)", cp1_base2);
  std::cout << "      → 在base_link看来：障碍物在右前方30°方向，距离约"
            << std::sqrt(cp1_base2.x * cp1_base2.x +
                         cp1_base2.y * cp1_base2.y)
            << " m\n\n";

  // ========================================================================
  // 第四部分：完整一帧点云批量转换
  // ========================================================================
  std::cout << "【第四部分】批量转换一整帧点云\n";
  std::cout << "----------------------------------------\n";

  // 模拟一帧激光雷达数据（16线，每线8个点）
  std::vector<PolarPoint> scan;
  std::cout << "  模拟一帧16线雷达扫描数据（简化，每线取部分点）：\n";

  // 模拟360°扫描，每隔45°一个点
  for (int i = 0; i < 8; ++i) {
    double angle = i * M_PI / 4.0;  // 0, 45, 90, 135, 180, 225, 270, 315°
    double range = 2.0 + i * 0.5;   // 距离递增
    scan.push_back({range, angle});
  }

  std::cout << "  原始极坐标数据（共 " << scan.size() << " 个点）：\n";
  for (const auto &p : scan) {
    std::cout << "    angle=" << p.angle * 180.0 / M_PI << "°, range="
              << p.range << " m\n";
  }

  // 批量转换
  std::vector<CartesianPoint> points_base =
      ScanToCartesianInBaseLink(scan, T_base_lidar);

  std::cout << "\n  转换后 base_link 坐标系下的笛卡尔坐标：\n";
  for (size_t i = 0; i < points_base.size(); ++i) {
    std::cout << "    Point[" << i << "]: [" << points_base[i].x << ", "
              << points_base[i].y << ", " << points_base[i].z << "]\n";
  }

  // ========================================================================
  // 第五部分：典型双雷达安装——右前方安装，顺时针旋转45°
  // ========================================================================
  std::cout << "\n【第五部分】典型双雷达安装：右前方 + 顺时针旋转45°\n";
  std::cout << "----------------------------------------\n";

  std::cout << "  假设雷达安装参数（相对于base_link）：\n";
  std::cout << "    → 安装位置：base_link右前方\n";
  std::cout << "       x = +0.5 m（前方0.5m）\n";
  std::cout << "       y = -0.2 m（右侧0.2m，ROS2中y向左为正）\n";
  std::cout << "    → 安装角度：相对于base_link顺时针旋转 45°\n";
  std::cout << "       即偏航角 yaw = -45°\n";
  std::cout << "    → 场景：双雷达配置中，副雷达安装在底盘右前方，\n";
  std::cout << "       朝向右前方45°以覆盖右侧盲区\n\n";

  // 构建齐次变换矩阵
  double lidar_x3 = 0.5;           // base_link前方0.5m
  double lidar_y3 = -0.2;          // base_link右侧0.2m
  double lidar_yaw3 = -M_PI / 4.0; // 顺时针45° = -45°

  Eigen::Matrix3d T_base_lidar3 = Eigen::Matrix3d::Identity();
  T_base_lidar3(0, 0) = std::cos(lidar_yaw3);
  T_base_lidar3(0, 1) = -std::sin(lidar_yaw3);
  T_base_lidar3(1, 0) = std::sin(lidar_yaw3);
  T_base_lidar3(1, 1) = std::cos(lidar_yaw3);
  T_base_lidar3(0, 2) = lidar_x3;
  T_base_lidar3(1, 2) = lidar_y3;

  PrintTransform2D("  T_base_lidar (右前方, 顺时针45°)", T_base_lidar3);

  // 打印旋转矩阵分量验证
  std::cout << "  → 旋转矩阵验证：\n";
  std::cout << "     cos(-45°) = " << std::cos(lidar_yaw3)
            << ", -sin(-45°) = " << -std::sin(lidar_yaw3) << "\n";
  std::cout << "     sin(-45°) = " << std::sin(lidar_yaw3)
            << ",  cos(-45°) = " << std::cos(lidar_yaw3) << "\n\n";

  // 转换 P1 ~ P4
  std::cout << "  将 P1~P4 变换到 base_link 坐标系：\n\n";

  // P1: 雷达正前方5m
  std::cout << "  --- P1：雷达正前方 5m ---\n";
  PrintPolarPoint("    原始极坐标", p1);
  PrintCartesianPoint("    lidar系笛卡尔", cp1);
  CartesianPoint p1_base3 = TransformPointToBaseLink(cp1, T_base_lidar3);
  PrintCartesianPoint("    base_link坐标", p1_base3);
  std::cout << "    → 手算验证：\n";
  std::cout << "       x = cos(-45°)*5 + (-sin(-45°))*0 + 0.5\n";
  std::cout << "         = 0.7071*5 + 0.7071*0 + 0.5 = "
            << std::cos(lidar_yaw3)*5.0 + 0.5 << "\n";
  std::cout << "       y = sin(-45°)*5 + cos(-45°)*0 - 0.2\n";
  std::cout << "         = -0.7071*5 + 0.7071*0 - 0.2 = "
            << std::sin(lidar_yaw3)*5.0 - 0.2 << "\n";
  std::cout << "    → 物理意义：障碍物在base_link右前方45°方向，\n";
  std::cout << "       距离base_link原点约 "
            << std::sqrt(p1_base3.x*p1_base3.x + p1_base3.y*p1_base3.y)
            << " m\n\n";

  // P2: 雷达左前45°，3m
  std::cout << "  --- P2：雷达左前45°，3m ---\n";
  PrintPolarPoint("    原始极坐标", p2);
  PrintCartesianPoint("    lidar系笛卡尔", cp2);
  CartesianPoint p2_base3 = TransformPointToBaseLink(cp2, T_base_lidar3);
  PrintCartesianPoint("    base_link坐标", p2_base3);
  std::cout << "    → 手算验证：\n";
  std::cout << "       P2在lidar系: [3*cos(45°), 3*sin(45°)] = ["
            << 3.0*std::cos(M_PI/4.0) << ", " << 3.0*std::sin(M_PI/4.0) << "]\n";
  std::cout << "       x = 0.7071*2.1213 + 0.7071*2.1213 + 0.5 = 3.5\n";
  std::cout << "       y = -0.7071*2.1213 + 0.7071*2.1213 - 0.2 = -0.2\n";
  std::cout << "    → 物理意义：雷达左前45° + 雷达本身顺时针45°\n";
  std::cout << "       = base_link正前方方向！y=-0.2 恰好是雷达安装位置的y坐标\n\n";

  // P3: 雷达正左方90°，2m
  std::cout << "  --- P3：雷达正左方90°，2m ---\n";
  PrintPolarPoint("    原始极坐标", p3);
  PrintCartesianPoint("    lidar系笛卡尔", cp3);
  CartesianPoint p3_base3 = TransformPointToBaseLink(cp3, T_base_lidar3);
  PrintCartesianPoint("    base_link坐标", p3_base3);
  std::cout << "    → 手算验证：\n";
  std::cout << "       P3在lidar系: [0, 2]\n";
  std::cout << "       x = 0.7071*0 + 0.7071*2 + 0.5 = 1.9142\n";
  std::cout << "       y = -0.7071*0 + 0.7071*2 - 0.2 = 1.2142\n";
  std::cout << "    → 物理意义：在base_link的左前方区域\n\n";

  // P4: 雷达右后方135°，4m
  std::cout << "  --- P4：雷达右后方135°，4m ---\n";
  PrintPolarPoint("    原始极坐标", p4);
  PrintCartesianPoint("    lidar系笛卡尔", cp4);
  CartesianPoint p4_base3 = TransformPointToBaseLink(cp4, T_base_lidar3);
  PrintCartesianPoint("    base_link坐标", p4_base3);
  std::cout << "    → 手算验证：\n";
  std::cout << "       P4在lidar系: [4*cos(-135°), 4*sin(-135°)] = ["
            << 4.0*std::cos(-3.0*M_PI/4.0) << ", "
            << 4.0*std::sin(-3.0*M_PI/4.0) << "]\n";
  std::cout << "       x = 0.7071*(-2.828) + 0.7071*(-2.828) + 0.5 = -3.5\n";
  std::cout << "       y = -0.7071*(-2.828) + 0.7071*(-2.828) - 0.2 = -0.2\n";
  std::cout << "    → 物理意义：雷达右后135° + 雷达顺时针45°\n";
  std::cout << "       = base_link正后方方向！y=-0.2 恰好是雷达安装位置的y坐标\n";
  std::cout << "       x=-3.5 表示在base_link后方3.5m处\n\n";

  // ========================================================================
  // 几何直观图（文字描述）
  // ========================================================================
  std::cout << "【几何直观】base_link 坐标系下的四个点位置：\n";
  std::cout << "----------------------------------------\n";
  std::cout << "  base_link 坐标系：x向前(+), y向左(+)\n";
  std::cout << "  雷达安装位置：[0.5, -0.2]（右前方）\n";
  std::cout << "  雷达朝向：右前方45°（顺时针偏转）\n\n";
  std::cout << "         y(左)↑\n";
  std::cout << "              |\n";
  std::cout << "    P3 ●     |     ● P2 (x轴上, y=-0.2)\n";
  std::cout << "   (1.2,1.9)|   (3.5,-0.2)\n";
  std::cout << "              |\n";
  std::cout << "  ───────────┼──────────→ x(前)\n";
  std::cout << "              |\n";
  std::cout << "    P4 ●     | 雷达● P1\n";
  std::cout << "  (-3.5,-0.2)| (4.0,-3.7)\n";
  std::cout << "              |\n\n";

  std::cout << "========================================\n";
  std::cout << "=== 测试结束 ===\n";
  std::cout << "========================================\n";
  std::cout << "\n【数学总结】\n";
  std::cout << "  1. 极坐标转笛卡尔：x = r*cos(θ), y = r*sin(θ), z = 0\n";
  std::cout << "  2. 坐标系变换：P_base = R * P_lidar + t\n";
  std::cout << "  3. 齐次矩阵表示：P_base_homo = T_base_lidar * P_lidar_homo\n";
  std::cout << "  4. 雷达安装位姿 T_base_lidar 包含了旋转R和平移t两部分\n";
  std::cout << "     - 旋转R由安装偏航角决定\n";
  std::cout << "     - 平移t由安装位置决定\n";
  std::cout << "  5. 角度叠加：雷达局部角度 + 雷达安装偏航角 = base_link中的绝对角度\n";
  std::cout << "     - P2: 雷达左前45° + 雷达顺时针45° = base_link正前方0°\n";
  std::cout << "     - P4: 雷达右后135° + 雷达顺时针45° = base_link正后方180°\n";

  return 0;
}
