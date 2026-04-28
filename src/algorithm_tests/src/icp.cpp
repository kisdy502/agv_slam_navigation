
/**
 * ICP的数学本质
 *  源点云：P 目标点云：Q
    目标：找到旋转矩阵 
    R
    R 和平移向量 
    t
    t，最小化误差
    求解R和t的闭式解
    计算质心
    去质心化
    构建协方差矩阵

 */
#include "icp.hpp"
#include <iostream>
#include <cmath>

// 核心函数：给定两点云，求旋转R和平移t
void ICP::align(
    const std::vector<Point>& source,
    const std::vector<Point>& target,
    Eigen::Matrix2d& R,
    Eigen::Vector2d& t)
{
    int n = source.size();

    // 1. 计算质心
    Eigen::Vector2d src_center(0, 0), tgt_center(0, 0);
    for (int i = 0; i < n; i++) {
        src_center += Eigen::Vector2d(source[i].x, source[i].y);
        tgt_center += Eigen::Vector2d(target[i].x, target[i].y);
    }
    src_center /= n;
    tgt_center /= n;

    // 2. 构建协方差矩阵H
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    for (int i = 0; i < n; i++) {
        Eigen::Vector2d ps(source[i].x - src_center.x(), source[i].y - src_center.y());
        Eigen::Vector2d pt(target[i].x - tgt_center.x(),target[i].y - tgt_center.y());
        H += ps * pt.transpose();
    }

    std::cout << "H = \n" << H << std::endl;

    // 3. SVD分解
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H,
        Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    std::cout << "U = \n" << U << std::endl;
    std::cout << "V = \n" << V << std::endl;


    // 4. 计算旋转矩阵
    R = V * U.transpose();

    std::cout << "R (before correction) = \n" << R << std::endl;
    std::cout << "det(R) = " << R.determinant() << std::endl;


    // 特殊处理：反射情况（det = -1）
    if (R.determinant() < 0) {
        Eigen::Matrix2d correction = Eigen::Matrix2d::Identity();
        correction(1, 1) = -1;
        R = V * correction * U.transpose();
    }

    std::cout << "R (after correction) = \n" << R << std::endl;

    // 5. 计算平移
    t = tgt_center - R * src_center;
    std::cout << "t = " << t.transpose() << std::endl;
}

// 完整的ICP迭代（自动找对应点）
void ICP::icp_iterative(
    std::vector<Point>& source,
    const std::vector<Point>& target,
    int max_iterations)
{
    for (int iter = 0; iter < max_iterations; iter++) {
        // 1. 找最近点对应
        std::vector<Point> matched_target(source.size());
        for (size_t i = 0; i < source.size(); i++) {
            matched_target[i] = find_nearest(source[i], target);
        }

        // 2. 计算最优变换
        Eigen::Matrix2d R;
        Eigen::Vector2d t;
        align(source, matched_target, R, t);

        // 3. 应用变换到源点云
        for (auto& p : source) {
            double new_x = R(0, 0) * p.x + R(0, 1) * p.y + t(0);
            double new_y = R(1, 0) * p.x + R(1, 1) * p.y + t(1);
            p.x = new_x;
            p.y = new_y;
        }

        // 4. 计算误差
        double error = compute_error(source, matched_target);
        std::cout << "Iteration " << iter << ", error = " << error << std::endl;

        // 5. 收敛判断
        if (error < 1e-6) break;
    }
}

Point ICP::find_nearest(const Point& p, const std::vector<Point>& cloud) {
    double min_dist = 1e10;
    Point nearest;
    for (const auto& q : cloud) {
        double dx = p.x - q.x;
        double dy = p.y - q.y;
        double dist = dx * dx + dy * dy;
        if (dist < min_dist) {
            min_dist = dist;
            nearest = q;
        }
    }
       // 打印目标点和找到的最近点的信息
    std::cout << "目标点: (" << p.x << ", " << p.y << ")\n";
    std::cout << "最近点: (" << nearest.x << ", " << nearest.y << "), 距离^2 = " << min_dist << "\n";
    return nearest;
}

double ICP::compute_error(const std::vector<Point>& source,
                          const std::vector<Point>& target) {
    double error = 0;
    for (size_t i = 0; i < source.size(); i++) {
        double dx = source[i].x - target[i].x;
        double dy = source[i].y - target[i].y;
        error += dx * dx + dy * dy;
    }
    return error / source.size();
}

// ICP测试方法
void ICP::test() {
    std::cout << "\n========== ICP Algorithm Test ==========" << std::endl;

    // 生成测试数据：原始点云
    std::vector<Point> source;
    for (int i = 0; i < 100; i++) {
        double angle = 2 * M_PI * i / 100;
        source.push_back(Point(cos(angle), sin(angle)));
    }

    // 对源点云施加变换：旋转30度，平移(1, 0.5)
    Eigen::Matrix2d R_true;
    R_true << cos(30 * M_PI / 180), -sin(30 * M_PI / 180),
              sin(30 * M_PI / 180),  cos(30 * M_PI / 180);
    Eigen::Vector2d t_true(1.0, 0.5);

    //R_true 旋转矩阵
    //t_true 平移向量
    std::vector<Point> target = source;
    for (auto& p : target) {
        double new_x = R_true(0, 0) * p.x + R_true(0, 1) * p.y + t_true(0);
        double new_y = R_true(1, 0) * p.x + R_true(1, 1) * p.y + t_true(1);
        p.x = new_x;
        p.y = new_y;
    }

    // ICP配准
    ICP icp;
    std::vector<Point> source_copy = source;
    icp.icp_iterative(source_copy, target);

    // 输出结果
    std::cout << "\n=== 配准结果 ===" << std::endl;
    std::cout << "源点云第一个点: (" << source[0].x << ", " << source[0].y << ")" << std::endl;
    std::cout << "变换后点: (" << source_copy[0].x << ", " << source_copy[0].y << ")" << std::endl;
    std::cout << "目标点云第一个点: (" << target[0].x << ", " << target[0].y << ")" << std::endl;
    std::cout << "========== Test Complete ==========" << std::endl;
}
