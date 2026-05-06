// include/control/types.h
#pragma once
#include <vector>

struct Pose
{
    double x;
    double y;
    double yaw;
};

struct PathPoint
{
    double x;
    double y;
    double yaw;
};

using Path = std::vector<PathPoint>;