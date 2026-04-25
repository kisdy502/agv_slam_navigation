// include/robust_localization/degeneracy_detector.hpp
#ifndef ROBUST_LOCALIZATION__DEGENERACY_DETECTOR_HPP_
#define ROBUST_LOCALIZATION__DEGENERACY_DETECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace robust_localization
{

    struct DegeneracyReport
    {
        bool is_degenerate;               // 是否退化
        double condition_number;          // Hessian条件数
        Eigen::Vector3d eigenvalues;      // 三个特征值 (x, y, theta方向)
        Eigen::Matrix3d eigenvectors;     // 特征向量
        double valid_match_ratio;         // 有效匹配比例
        std::string degenerate_direction; // "x", "y", "theta", "xy", etc.
        rclcpp::Time timestamp;
    };

    class DegeneracyDetector
    {
    public:
        DegeneracyDetector();

        // 主接口：分析当前激光扫描和地图的匹配质量
        DegeneracyReport analyze(
            const sensor_msgs::msg::LaserScan::SharedPtr &scan,
            const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
            const geometry_msgs::msg::PoseWithCovarianceStamped &current_estimate);

        // 配置参数
        void setConditionNumberThreshold(double threshold)
        {
            condition_number_threshold_ = threshold;
        }
        void setMinEigenvalueThreshold(double threshold)
        {
            min_eigenvalue_threshold_ = threshold;
        }
        void setValidMatchRatioThreshold(double ratio)
        {
            valid_match_ratio_threshold_ = ratio;
        }

    private:
        // 从激光扫描提取几何特征（PCA分析点云分布）
        Eigen::Matrix2d computeScanGeometry(const sensor_msgs::msg::LaserScan::SharedPtr &scan);

        // 计算匹配Hessian矩阵（基于ICP的雅可比）
        Eigen::Matrix3d computeMatchingHessian(
            const sensor_msgs::msg::LaserScan::SharedPtr &scan,
            const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
            const geometry_msgs::msg::PoseWithCovarianceStamped &estimate);

        // 计算有效匹配比例
        double computeValidMatchRatio(
            const sensor_msgs::msg::LaserScan::SharedPtr &scan,
            const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
            const geometry_msgs::msg::PoseWithCovarianceStamped &estimate);

        // 判断退化方向
        std::string determineDegenerateDirection(
            const Eigen::Vector3d &eigenvalues,
            const Eigen::Matrix3d &eigenvectors);

        double condition_number_threshold_ = 1000.0; // 条件数阈值
        double min_eigenvalue_threshold_ = 0.001;    // 最小特征值阈值
        double valid_match_ratio_threshold_ = 0.3;   // 有效匹配比例阈值
    };

} // namespace robust_localization

#endif // ROBUST_LOCALIZATION__DEGENERACY_DETECTOR_HPP_