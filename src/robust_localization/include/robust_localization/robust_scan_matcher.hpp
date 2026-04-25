// include/robust_localization/robust_scan_matcher.hpp
#ifndef ROBUST_LOCALIZATION__ROBUST_SCAN_MATCHER_HPP_
#define ROBUST_LOCALIZATION__ROBUST_SCAN_MATCHER_HPP_

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <Eigen/Dense>

namespace robust_localization
{

    struct MatchResult
    {
        geometry_msgs::msg::Pose2D pose;
        double score;
        int inlier_count;
        int total_count;
        double inlier_ratio;
        bool success;
    };

    class RobustScanMatcher
    {
    public:
        RobustScanMatcher();

        // 主匹配接口
        MatchResult match(
            const sensor_msgs::msg::LaserScan::SharedPtr &scan,
            const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
            const geometry_msgs::msg::Pose2D &initial_guess);

        // 配置
        void setMaxIterations(int iters) { max_iterations_ = iters; }
        void setConvergenceThreshold(double thresh) { convergence_threshold_ = thresh; }
        void setInlierThreshold(double thresh) { inlier_threshold_ = thresh; }
        void setMaxCorrespondenceDist(double dist) { max_correspondence_dist_ = dist; }

    private:
        // 点到地图的对应关系
        struct Correspondence
        {
            Eigen::Vector2d point;
            Eigen::Vector2d map_point;
            double distance;
            bool valid;
        };

        // 寻找对应点
        std::vector<Correspondence> findCorrespondences(
            const sensor_msgs::msg::LaserScan::SharedPtr &scan,
            const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
            const geometry_msgs::msg::Pose2D &pose);

        // 鲁棒位姿估计（RANSAC + ICP）
        Eigen::Vector3d estimatePoseRobust(
            const std::vector<Correspondence> &correspondences,
            const Eigen::Vector3d &initial_pose);

        // 计算匹配得分
        double computeScore(
            const std::vector<Correspondence> &correspondences);

        int max_iterations_ = 50;
        double convergence_threshold_ = 1e-4;
        double inlier_threshold_ = 0.3;        // 米
        double max_correspondence_dist_ = 1.0; // 米
    };

} // namespace robust_localization

#endif // ROBUST_LOCALIZATION__ROBUST_SCAN_MATCHER_HPP_