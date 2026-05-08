#include "pure_pursuit/pure_pursuit.h"
#include <cmath>
#include <iostream>
#include <limits>

PurePursuit::PurePursuit(double wheelbase, double lookahead)
    : wheelbase_(wheelbase), lookahead_(lookahead) {}

/**
 * @brief 在路径中寻找“前视点”
 *
 * 前视点定义：
 *  - 距离车辆 ≥ lookahead_
 *  - 在满足条件下，距离最近
 *
 * 返回值：
 *  path 中的索引
 */
size_t PurePursuit::findLookaheadPoint(const Pose &car, const Path &path) {
  size_t idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path.size(); ++i) {
    double dx = path[i].x - car.x;
    double dy = path[i].y - car.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // 找到第一个满足前视距离且最近的点
    if (dist >= lookahead_ && dist < min_dist) {
      min_dist = dist;
      idx = i;
    }
  }

  std::cout << "[PurePursuit] Lookahead point index: " << idx
            << ", distance: " << min_dist << std::endl;

  return idx;
}

/**
 * @brief 计算 Pure Pursuit 转向角
 *
 * 输入：
 *  car  : 当前车辆位姿 (x, y, yaw)
 *  path : 参考路径
 *
 * 输出：
 *  前轮转角 δ（rad）
 */
double PurePursuit::compute(const Pose &car, const Path &path) {
  if (path.empty()) {
    std::cerr << "[PurePursuit] ERROR: Path is empty!" << std::endl;
    return 0.0;
  }

  // 1. 找前视点
  size_t target_idx = findLookaheadPoint(car, path);
  const auto &target = path[target_idx];

  // 2. 计算车辆到前视点的向量（世界坐标）
  double dx = target.x - car.x;
  double dy = target.y - car.y;

  std::cout << "[PurePursuit] Car: (" << car.x << ", " << car.y << ", "
            << car.yaw << ")" << std::endl;

  std::cout << "[PurePursuit] Target: (" << target.x << ", " << target.y << ")"
            << std::endl;

  // 3. 坐标变换：世界 → 车体
  double local_x = std::cos(-car.yaw) * dx - std::sin(-car.yaw) * dy;

  double local_y = std::sin(-car.yaw) * dx + std::cos(-car.yaw) * dy;

  std::cout << "[PurePursuit] Local vector: (" << local_x << ", " << local_y
            << ")" << std::endl;

  // 4. 计算航向误差角 α
  double alpha = std::atan2(local_y, local_x);

  std::cout << "[PurePursuit] Alpha: " << alpha << " rad" << std::endl;

  // 5. Pure Pursuit 核心公式
  double steer = std::atan2(2.0 * wheelbase_ * std::sin(alpha), lookahead_);

  std::cout << "[PurePursuit] Steering angle: " << steer << " rad" << std::endl;

  return steer;
}