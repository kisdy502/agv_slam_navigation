// src/robust_scan_matcher.cpp
#include "robust_localization/robust_scan_matcher.hpp"
#include <cmath>
#include <random>

namespace robust_localization
{

    RobustScanMatcher::RobustScanMatcher() {}

    MatchResult RobustScanMatcher::match(
        const sensor_msgs::msg::LaserScan::SharedPtr &scan,
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        const geometry_msgs::msg::Pose2D &initial_guess)
    {

        MatchResult result;
        result.pose = initial_guess;
        result.success = false;

        if (!scan || !map)
            return result;

        Eigen::Vector3d current_pose;
        current_pose << initial_guess.x, initial_guess.y, initial_guess.theta;

        double best_score = -1;
        Eigen::Vector3d best_pose = current_pose;

        // 迭代 ICP + RANSAC
        for (int iter = 0; iter < max_iterations_; ++iter)
        {
            auto correspondences = findCorrespondences(scan, map,
                                                       geometry_msgs::msg::Pose2D().set__x(current_pose(0)).set__y(current_pose(1)).set__theta(current_pose(2)));

            if (correspondences.empty())
                break;

            // 统计内点
            int inliers = 0;
            for (const auto &c : correspondences)
            {
                if (c.valid && c.distance < inlier_threshold_)
                    inliers++;
            }

            // RANSAC：如果内点比例太低，可能是动态障碍/遮挡
            double inlier_ratio = (double)inliers / correspondences.size();
            if (inlier_ratio < 0.2)
            {
                // 尝试用 RANSAC 找到最大一致集
                current_pose = estimatePoseRobust(correspondences, current_pose);
            }
            else
            {
                // 标准 ICP 一步
                // 简化：用对应点计算变换
                Eigen::Vector2d src_mean(0, 0), dst_mean(0, 0);
                int count = 0;
                for (const auto &c : correspondences)
                {
                    if (!c.valid)
                        continue;
                    src_mean += c.point;
                    dst_mean += c.map_point;
                    count++;
                }
                if (count == 0)
                    break;
                src_mean /= count;
                dst_mean /= count;

                // 计算旋转和平移（简化 SVD）
                Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
                for (const auto &c : correspondences)
                {
                    if (!c.valid)
                        continue;
                    H += (c.point - src_mean) * (c.map_point - dst_mean).transpose();
                }

                Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
                Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
                if (R.determinant() < 0)
                {
                    Eigen::Matrix2d S = Eigen::Matrix2d::Identity();
                    S(1, 1) = -1;
                    R = svd.matrixV() * S * svd.matrixU().transpose();
                }

                double delta_theta = atan2(R(1, 0), R(0, 0));
                current_pose(2) += delta_theta * 0.1; // 步长控制
                current_pose(0) += (dst_mean(0) - src_mean(0)) * 0.1;
                current_pose(1) += (dst_mean(1) - src_mean(1)) * 0.1;
            }

            double score = computeScore(correspondences);
            if (score > best_score)
            {
                best_score = score;
                best_pose = current_pose;
            }

            // 收敛检查
            if (fabs(score - best_score) < convergence_threshold_ && iter > 5)
            {
                break;
            }
        }

        result.pose.x = best_pose(0);
        result.pose.y = best_pose(1);
        result.pose.theta = best_pose(2);
        result.score = best_score;
        result.success = (best_score > 0);

        // 统计最终内点
        auto final_corr = findCorrespondences(scan, map, result.pose);
        result.inlier_count = 0;
        for (const auto &c : final_corr)
        {
            if (c.valid && c.distance < inlier_threshold_)
                result.inlier_count++;
        }
        result.total_count = final_corr.size();
        result.inlier_ratio = (result.total_count > 0) ? (double)result.inlier_count / result.total_count : 0;

        return result;
    }

    std::vector<RobustScanMatcher::Correspondence> RobustScanMatcher::findCorrespondences(
        const sensor_msgs::msg::LaserScan::SharedPtr &scan,
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        const geometry_msgs::msg::Pose2D &pose)
    {

        std::vector<Correspondence> correspondences;

        double map_res = map->info.resolution;
        double map_origin_x = map->info.origin.position.x;
        double map_origin_y = map->info.origin.position.y;
        int map_width = map->info.width;
        int map_height = map->info.height;

        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            double r = scan->ranges[i];
            if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max)
                continue;

            double angle = pose.theta + scan->angle_min + i * scan->angle_increment;
            double px = pose.x + r * cos(angle);
            double py = pose.y + r * sin(angle);

            Correspondence c;
            c.point << px, py;

            // 转换到地图坐标
            int mx = (px - map_origin_x) / map_res;
            int my = (py - map_origin_y) / map_res;

            if (mx >= 0 && mx < map_width && my >= 0 && my < map_height)
            {
                int idx = my * map_width + mx;
                int8_t occupancy = map->data[idx];

                // 找到地图中最近的被占据单元格
                double min_dist = max_correspondence_dist_;
                Eigen::Vector2d best_map_point;
                bool found = false;

                // 5x5 局部搜索
                for (int dy = -2; dy <= 2; ++dy)
                {
                    for (int dx = -2; dx <= 2; ++dx)
                    {
                        int nx = mx + dx, ny = my + dy;
                        if (nx < 0 || nx >= map_width || ny < 0 || ny >= map_height)
                            continue;
                        int nidx = ny * map_width + nx;
                        if (map->data[nidx] > 50)
                        { // 被占据
                            double wx = map_origin_x + nx * map_res;
                            double wy = map_origin_y + ny * map_res;
                            double d = hypot(wx - px, wy - py);
                            if (d < min_dist)
                            {
                                min_dist = d;
                                best_map_point << wx, wy;
                                found = true;
                            }
                        }
                    }
                }

                c.map_point = best_map_point;
                c.distance = min_dist;
                c.valid = found && (occupancy > 50 || occupancy == -1);
            }
            else
            {
                c.valid = false;
                c.distance = max_correspondence_dist_;
            }

            correspondences.push_back(c);
        }

        return correspondences;
    }

    Eigen::Vector3d RobustScanMatcher::estimatePoseRobust(
        const std::vector<Correspondence> &correspondences,
        const Eigen::Vector3d &initial_pose)
    {

        // 简化 RANSAC：随机采样找到最大一致集
        if (correspondences.size() < 10)
            return initial_pose;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, correspondences.size() - 1);

        int best_inliers = 0;
        Eigen::Vector3d best_pose = initial_pose;

        // RANSAC 迭代
        for (int ransac_iter = 0; ransac_iter < 20; ++ransac_iter)
        {
            // 随机选3个对应点
            std::vector<int> indices;
            while (indices.size() < 3)
            {
                int idx = dis(gen);
                if (correspondences[idx].valid)
                    indices.push_back(idx);
            }

            // 用这3点估计变换（简化）
            Eigen::Vector3d test_pose = initial_pose;
            // ... 计算变换 ...

            // 统计内点
            int inliers = 0;
            for (const auto &c : correspondences)
            {
                if (!c.valid)
                    continue;
                // 变换后检查距离
                double dx = c.point(0) - test_pose(0);
                double dy = c.point(1) - test_pose(1);
                double dist = hypot(dx, dy); // 简化
                if (dist < inlier_threshold_)
                    inliers++;
            }

            if (inliers > best_inliers)
            {
                best_inliers = inliers;
                best_pose = test_pose;
            }
        }

        return best_pose;
    }

    double RobustScanMatcher::computeScore(
        const std::vector<Correspondence> &correspondences)
    {

        double score = 0;
        int valid_count = 0;

        for (const auto &c : correspondences)
        {
            if (!c.valid)
                continue;
            valid_count++;
            // 高斯似然得分
            score += exp(-(c.distance * c.distance) / (2 * 0.1 * 0.1));
        }

        return (valid_count > 0) ? score / valid_count : 0;
    }

} // namespace robust_localization