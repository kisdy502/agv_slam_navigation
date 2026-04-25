#include <cmath>
#include <vector>
#include <limits>
#include <Eigen/Dense>

struct Pose {
    double x, y, theta;
};

struct Twist {
    double linear_x, angular_z;
};

class DWA {
public:
    DWA(double max_v, double max_w, double acc_v, double acc_w, double dt_control)
        : max_vel_(max_v), max_omega_(max_w),
          max_acc_v_(acc_v), max_acc_w_(acc_w),
          dt_(dt_control)
    {}

    // 核心：根据当前状态、目标点、局部代价地图，输出最优速度指令
    Twist compute_command(const Pose& current, const Pose& goal,
                          const std::vector<std::vector<double>>& costmap,
                          double resolution, double origin_x, double origin_y,
                          const Twist& current_speed)
    {
        // 1. 计算动态窗口
        double v_min = std::max(current_speed.linear_x - max_acc_v_ * dt_, -max_vel_);
        double v_max = std::min(current_speed.linear_x + max_acc_v_ * dt_, max_vel_);
        double w_min = std::max(current_speed.angular_z - max_acc_w_ * dt_, -max_omega_);
        double w_max = std::min(current_speed.angular_z + max_acc_w_ * dt_, max_omega_);

        // 采样步长
        double v_step = 0.05;   // m/s
        double w_step = 0.2;    // rad/s

        double best_score = -std::numeric_limits<double>::max();
        Twist best_cmd = {0.0, 0.0};

        // 2. 采样并评估
        for (double v = v_min; v <= v_max; v += v_step) {
            for (double w = w_min; w <= w_max; w += w_step) {
                // 模拟轨迹
                double sim_time = 2.0;   // 模拟未来2秒
                double dt_sim = 0.1;
                Pose sim_pose = current;
                bool obstacle = false;

                for (double t = 0; t < sim_time; t += dt_sim) {
                    // 运动学更新（欧拉法）
                    double dx = v * cos(sim_pose.theta) * dt_sim;
                    double dy = v * sin(sim_pose.theta) * dt_sim;
                    sim_pose.x += dx;
                    sim_pose.y += dy;
                    sim_pose.theta += w * dt_sim;

                    // 检查是否碰到障碍物
                    if (!is_free(sim_pose, costmap, resolution, origin_x, origin_y)) {
                        obstacle = true;
                        break;
                    }
                }
                if (obstacle) continue;

                // 计算三个代价
                double heading = heading_cost(sim_pose, goal);
                double dist = distance_cost(sim_pose, costmap, resolution, origin_x, origin_y);
                double velocity = v / max_vel_;

                // 加权求和
                double score = alpha_ * heading + beta_ * dist + gamma_ * velocity;
                if (score > best_score) {
                    best_score = score;
                    best_cmd = {v, w};
                }
            }
        }
        return best_cmd;
    }

private:
    double max_vel_, max_omega_, max_acc_v_, max_acc_w_, dt_;
    double alpha_ = 0.5, beta_ = 0.3, gamma_ = 0.2;   // 权重

    bool is_free(const Pose& pose,
                 const std::vector<std::vector<double>>& costmap,
                 double res, double ox, double oy)
    {
        int ix = floor((pose.x - ox) / res);
        int iy = floor((pose.y - oy) / res);
        if (ix < 0 || iy < 0 || ix >= (int)costmap[0].size() || iy >= (int)costmap.size())
            return false;   // 超出地图范围视为障碍
        return costmap[iy][ix] < 0.5;   // 假设 0=空闲, 1=障碍
    }

    double heading_cost(const Pose& pose, const Pose& goal) {
        double dx = goal.x - pose.x;
        double dy = goal.y - pose.y;
        double angle_to_goal = atan2(dy, dx);
        double diff = fabs(angle_to_goal - pose.theta);
        return 1.0 - diff / M_PI;   // 归一化到 [0,1]，1表示方向最佳
    }

    double distance_cost(const Pose& pose,
                         const std::vector<std::vector<double>>& costmap,
                         double res, double ox, double oy)
    {
        // 简化为检查机器人周围一个圆形区域的最小代价
        double min_dist = 1.0;  // 最大归一化距离（超过此值认为安全）
        for (double dx = -0.5; dx <= 0.5; dx += 0.1) {
            for (double dy = -0.5; dy <= 0.5; dy += 0.1) {
                double wx = pose.x + dx;
                double wy = pose.y + dy;
                int ix = floor((wx - ox) / res);
                int iy = floor((wy - oy) / res);
                if (ix >=0 && iy>=0 && ix<(int)costmap[0].size() && iy<(int)costmap.size()) {
                    double occ = costmap[iy][ix];
                    if (occ > 0.5) {
                        double d = sqrt(dx*dx + dy*dy);
                        if (d < min_dist) min_dist = d;
                    }
                }
            }
        }
        return std::min(1.0, min_dist);   // 越靠近障碍物得分越低
    }
};