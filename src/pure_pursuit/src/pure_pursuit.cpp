// src/pure_pursuit.cpp
#include "pure_pursuit/pure_pursuit.h"
#include <cmath>
#include <limits>

PurePursuit::PurePursuit(double wheelbase, double lookahead)
    : wheelbase_(wheelbase), lookahead_(lookahead) {}

size_t PurePursuit::findLookaheadPoint(const Pose &car, const Path &path)
{
    size_t idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < path.size(); ++i)
    {
        double dx = path[i].x - car.x;
        double dy = path[i].y - car.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist >= lookahead_ && dist < min_dist)
        {
            min_dist = dist;
            idx = i;
        }
    }
    return idx;
}

double PurePursuit::compute(const Pose &car, const Path &path)
{
    if (path.empty())
        return 0.0;

    size_t target_idx = findLookaheadPoint(car, path);
    const auto &target = path[target_idx];

    double dx = target.x - car.x;
    double dy = target.y - car.y;

    double local_x = std::cos(-car.yaw) * dx - std::sin(-car.yaw) * dy;
    double local_y = std::sin(-car.yaw) * dx + std::cos(-car.yaw) * dy;

    double alpha = std::atan2(local_y, local_x);

    return std::atan2(2.0 * wheelbase_ * std::sin(alpha), lookahead_);
}