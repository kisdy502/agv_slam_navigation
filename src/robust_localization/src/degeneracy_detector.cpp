// src/degeneracy_detector.cpp
#include "robust_localization/degeneracy_detector.hpp"
#include <cmath>
#include <algorithm>

namespace robust_localization {

DegeneracyDetector::DegeneracyDetector() {}

DegeneracyReport DegeneracyDetector::analyze(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const geometry_msgs::msg::PoseWithCovarianceStamped& current_estimate)
{
    DegeneracyReport report;
    report.timestamp = scan->header.stamp;
    
    // 1. 计算有效匹配比例
    report.valid_match_ratio = computeValidMatchRatio(scan, map, current_estimate);
    
    // 2. 计算匹配Hessian矩阵
    Eigen::Matrix3d hessian = computeMatchingHessian(scan, map, current_estimate);
    
    // 3. 特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(hessian);
    report.eigenvalues = solver.eigenvalues();
    report.eigenvectors = solver.eigenvectors();
    
    // 4. 计算条件数
    double lambda_max = report.eigenvalues.maxCoeff();
    double lambda_min = report.eigenvalues.minCoeff();
    report.condition_number = (lambda_min > 1e-10) ? (lambda_max / lambda_min) : 1e10;
    
    // 5. 判断退化方向
    report.degenerate_direction = determineDegenerateDirection(
        report.eigenvalues, report.eigenvectors);
    
    // 6. 综合判断
    report.is_degenerate = (report.condition_number > condition_number_threshold_) ||
                           (lambda_min < min_eigenvalue_threshold_) ||
                           (report.valid_match_ratio < valid_match_ratio_threshold_);
    
    return report;
}

Eigen::Matrix2d DegeneracyDetector::computeScanGeometry(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan)
{
    // 将激光点转为笛卡尔坐标，做PCA
    std::vector<Eigen::Vector2d> points;
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double r = scan->ranges[i];
        if (r > scan->range_min && r < scan->range_max && 
            !std::isinf(r) && !std::isnan(r)) {
            double angle = scan->angle_min + i * scan->angle_increment;
            points.emplace_back(r * cos(angle), r * sin(angle));
        }
    }
    
    if (points.size() < 3) return Eigen::Matrix2d::Identity();
    
    // 计算协方差矩阵（PCA）
    Eigen::Vector2d mean(0, 0);
    for (const auto& p : points) mean += p;
    mean /= points.size();
    
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const auto& p : points) {
        Eigen::Vector2d diff = p - mean;
        cov += diff * diff.transpose();
    }
    cov /= points.size();
    
    return cov;
}

Eigen::Matrix3d DegeneracyDetector::computeMatchingHessian(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const geometry_msgs::msg::PoseWithCovarianceStamped& estimate)
{
    // 简化实现：基于扫描几何和地图占据情况估计Hessian
    // 实际应该用ICP的雅可比，这里用几何近似
    
    Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
    
    // 提取当前位姿
    double x = estimate.pose.pose.position.x;
    double y = estimate.pose.pose.position.y;
    double yaw = atan2(estimate.pose.pose.orientation.z, estimate.pose.pose.orientation.w) * 2;
    
    // 计算扫描几何
    Eigen::Matrix2d scan_cov = computeScanGeometry(scan);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(scan_cov);
    Eigen::Vector2d scan_eigen = solver.eigenvalues();
    
    // 将扫描特征值映射到Hessian的x,y分量
    // 如果扫描主要沿Y方向分布（通道两侧墙），则X方向约束强
    hessian(0, 0) = scan_eigen(1);  // x方向约束（垂直于主方向）
    hessian(1, 1) = scan_eigen(0);  // y方向约束
    
    // theta方向约束：基于角点/曲率数量
    double theta_constraint = 0.0;
    int corner_count = 0;
    for (size_t i = 1; i < scan->ranges.size() - 1; ++i) {
        double r_prev = scan->ranges[i-1];
        double r_curr = scan->ranges[i];
        double r_next = scan->ranges[i+1];
        if (std::isfinite(r_prev) && std::isfinite(r_curr) && std::isfinite(r_next)) {
            double curvature = fabs(r_prev - 2*r_curr + r_next);
            if (curvature > 0.1) corner_count++;
        }
    }
    hessian(2, 2) = std::max(0.01, corner_count * 0.001);
    
    return hessian;
}

double DegeneracyDetector::computeValidMatchRatio(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const geometry_msgs::msg::PoseWithCovarianceStamped& estimate)
{
    if (!map) return 1.0;  // 无地图时默认全有效
    
    double x = estimate.pose.pose.position.x;
    double y = estimate.pose.pose.position.y;
    double yaw = atan2(estimate.pose.pose.orientation.z, estimate.pose.pose.orientation.w) * 2;
    
    int valid_count = 0;
    int total_count = 0;
    
    // 地图参数
    double map_res = map->info.resolution;
    double map_origin_x = map->info.origin.position.x;
    double map_origin_y = map->info.origin.position.y;
    int map_width = map->info.width;
    int map_height = map->info.height;
    
    for (size_t i = 0; i < scan->ranges.size(); i += 2) {  // 采样加速
        double r = scan->ranges[i];
        if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) continue;
        
        double angle = scan->angle_min + i * scan->angle_increment + yaw;
        double px = x + r * cos(angle);
        double py = y + r * sin(angle);
        
        // 转换到地图坐标
        int mx = (px - map_origin_x) / map_res;
        int my = (py - map_origin_y) / map_res;
        
        if (mx >= 0 && mx < map_width && my >= 0 && my < map_height) {
            int idx = my * map_width + mx;
            int8_t occupancy = map->data[idx];
            total_count++;
            
            // 如果地图显示被占据或未知，认为是有效匹配
            if (occupancy > 50 || occupancy == -1) {
                // 进一步检查：如果预期是空的但实际有东西，可能是动态障碍
                // 这里简化：只要地图有信息就算有效
                valid_count++;
            }
        }
    }
    
    return (total_count > 0) ? (double)valid_count / total_count : 1.0;
}

std::string DegeneracyDetector::determineDegenerateDirection(
    const Eigen::Vector3d& eigenvalues,
    const Eigen::Matrix3d& eigenvectors)
{
    // 找出最小特征值对应的方向
    int min_idx;
    double min_val = eigenvalues.minCoeff(&min_idx);
    
    if (min_val > min_eigenvalue_threshold_ * 10) {
        return "none";  // 无明显退化
    }
    
    // 分析特征向量判断退化方向
    Eigen::Vector3d min_eigenvec = eigenvectors.col(min_idx);
    
    double x_component = fabs(min_eigenvec(0));
    double y_component = fabs(min_eigenvec(1));
    double theta_component = fabs(min_eigenvec(2));
    
    if (theta_component > 0.8) return "theta";
    if (x_component > y_component && x_component > 0.5) return "x";
    if (y_component > x_component && y_component > 0.5) return "y";
    
    return "mixed";
}

} // namespace robust_localization