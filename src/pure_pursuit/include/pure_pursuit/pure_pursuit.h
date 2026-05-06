// include/pure_pursuit/pure_pursuit.h
#pragma once
#include <cstddef>
#include "types.h"

class PurePursuit
{
public:
    PurePursuit(double wheelbase, double lookahead);

    double compute(const Pose &car, const Path &path);

private:
    double wheelbase_; // 轴距
    double lookahead_; // 前视距离

    size_t findLookaheadPoint(const Pose &car, const Path &path);
};