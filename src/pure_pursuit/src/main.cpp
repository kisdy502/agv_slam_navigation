// src/main.cpp
#include "pure_pursuit/pure_pursuit.h"
#include <iostream>

int main()
{
    Path path = {
        {0, 0, 0},
        {5, 0, 0},
        {10, 0, 0},
        {15, 5, 0.5},
        {20, 10, 0.7}};

    Pose car{2.0, 0.5, 0.1};
    PurePursuit pp(2.8, 5.0);

    double steer = pp.compute(car, path);

    std::cout << "[Pure Pursuit] Steering angle: "
              << steer << " rad" << std::endl;

    return 0;
}