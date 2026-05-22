#include <cmath>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/transform/rigid_transform.h"

// 辅助函数：打印 Rigid3d
void PrintRigid3d(const std::string &name,
                  const cartographer::transform::Rigid3d &pose) {
  const auto &t = pose.translation();
  const auto &q = pose.rotation();
  std::cout << name << ":\n"
            << "  translation: [" << t.x() << ", " << t.y() << ", " << t.z()
            << "]\n"
            << "  rotation quaternion: [" << q.w() << ", " << q.x() << ", "
            << q.y() << ", " << q.z() << "]\n"
            << "  yaw (deg): "
            << q.toRotationMatrix().eulerAngles(2, 1, 0)(0) * 180.0 / M_PI
            << "\n\n";
}

int main(int argc, char **argv) {
  std::cout << "========================================\n";
  std::cout << "=== Rigid3d 全局坐标与局部坐标互转测试 ===\n";
  std::cout << "========================================\n\n";

  // ========== 1. 设定具体的子图全局位姿 (Submap Global Pose) ==========
  // 假设子图在世界坐标系中的位置：平移 (10, 5, 0)，旋转绕Z轴45度
  std::cout << "【步骤1】设定子图的全局位姿 T_submap_global\n";
  std::cout << "    → 子图位于世界坐标系下，平移 (10, 5, 0)，绕Z轴旋转 45°\n";
  Eigen::Vector3d submap_translation(10.0, 5.0, 0.0);
  Eigen::Quaterniond submap_rotation(
      Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ())); // 45°
  cartographer::transform::Rigid3d T_submap_global(submap_translation,
                                                   submap_rotation);
  PrintRigid3d("T_submap_global (子图在世界坐标系中的位姿)", T_submap_global);

  // ========== 2. 设定一个局部点 P_local ==========
  // 该点在子图坐标系下的坐标：(2, 3, 0)
  std::cout << "【步骤2】设定一个子图局部坐标系下的点 P_local\n";
  std::cout << "    → 该点在子图坐标系下的坐标为 (2, 3, 0)\n";
  Eigen::Vector3d P_local(2.0, 3.0, 0.0);
  std::cout << "P_local (子图局部坐标): [" << P_local.x() << ", "
            << P_local.y() << ", " << P_local.z() << "]\n\n";

  // ========== 3. 局部 → 全局：P_global = T_submap_global * P_local ==========
  std::cout << "【步骤3】局部坐标 → 全局坐标\n";
  std::cout << "    → P_global = T_submap_global * P_local\n";
  std::cout << "    → 意义：将子图局部坐标系下的点，变换到世界坐标系\n";
  Eigen::Vector3d P_global = T_submap_global * P_local;
  std::cout << "P_global (世界全局坐标) = [" << P_global.x()
            << ", " << P_global.y() << ", " << P_global.z() << "]\n\n";

  // ========== 4. 全局 → 局部：P_local2 = T_submap_global.inverse() * P_global ==========
  std::cout << "【步骤4】全局坐标 → 局部坐标（反向恢复）\n";
  std::cout << "    → T_global_submap = T_submap_global.inverse()\n";
  std::cout << "    → P_local2 = T_global_submap * P_global\n";
  std::cout << "    → 意义：将世界坐标系下的点，变换回子图局部坐标系\n";
  cartographer::transform::Rigid3d T_global_submap = T_submap_global.inverse();
  Eigen::Vector3d P_local2 = T_global_submap * P_global;
  std::cout << "P_local_recovered (恢复后的局部坐标) = ["
            << P_local2.x() << ", " << P_local2.y() << ", " << P_local2.z()
            << "]\n\n";

  // 验证恢复的相等性
  double error = (P_local2 - P_local).norm();
  std::cout << "【验证】恢复误差 norm(P_local2 - P_local) = " << error << "\n";
  if (error < 1e-6) {
    std::cout << "    ✅ 成功：恢复的局部点与原局部点一致\n";
  } else {
    std::cout << "    ❌ 失败：误差过大\n";
  }

  // ========== 5. 测试一个完整的节点位姿 (包含旋转) ==========
  // 节点在子图坐标系中的位姿：平移(1,0,0)，旋转30°绕Z
  std::cout << "\n========================================\n";
  std::cout << "=== 测试完整位姿（平移 + 旋转）互转 ===\n";
  std::cout << "========================================\n\n";

  std::cout << "【步骤5】设定节点在子图局部坐标系中的位姿 T_node_local\n";
  std::cout << "    → 节点平移 (1, 0, 0)，绕Z轴旋转 30°\n";
  Eigen::Vector3d node_local_translation(1.0, 0.0, 0.0);
  Eigen::Quaterniond node_local_rotation(
      Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitZ()));
  cartographer::transform::Rigid3d T_node_local(node_local_translation,
                                                node_local_rotation);
  PrintRigid3d("T_node_local (节点在子图局部坐标系)", T_node_local);

  // 计算节点全局位姿
  std::cout << "【步骤6】局部位姿 → 全局位姿\n";
  std::cout << "    → T_node_global = T_submap_global * T_node_local\n";
  std::cout << "    → 意义：将子图局部坐标系下的节点位姿，变换到世界坐标系\n";
  cartographer::transform::Rigid3d T_node_global =
      T_submap_global * T_node_local;
  PrintRigid3d("T_node_global (节点在世界坐标系)", T_node_global);

  // 再反向恢复
  std::cout << "【步骤7】全局位姿 → 局部位姿（反向恢复）\n";
  std::cout << "    → T_node_local_recovered = T_global_submap * T_node_global\n";
  std::cout << "    → 意义：将世界坐标系下的节点位姿，变换回子图局部坐标系\n";
  cartographer::transform::Rigid3d T_node_local_recovered =
      T_global_submap * T_node_global;
  PrintRigid3d("T_node_local_recovered (恢复后的局部位姿)", T_node_local_recovered);

  // 验证
  double translation_error =
      (T_node_local_recovered.translation() - T_node_local.translation())
          .norm();
  double rotation_error = T_node_local_recovered.rotation().angularDistance(
      T_node_local.rotation());
  std::cout << "【验证】平移误差 = " << translation_error << "\n";
  std::cout << "【验证】旋转误差 (弧度) = " << rotation_error << "\n";
  if (translation_error < 1e-6 && rotation_error < 1e-6) {
    std::cout << "    ✅ 成功：完整位姿正向/反向变换一致\n";
  } else {
    std::cout << "    ❌ 失败：位姿恢复误差过大\n";
  }

  std::cout << "\n========================================\n";
  std::cout << "=== 测试结束 ===\n";
  std::cout << "========================================\n";

  return 0;
}