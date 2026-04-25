#ifndef ICP_HPP
#define ICP_HPP

#include <vector>
#include <Eigen/Dense>

struct Point {
    double x, y;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
};

class ICP {
public:
    // 核心函数：给定两点云，求旋转R和平移t
    void align(
        const std::vector<Point>& source,
        const std::vector<Point>& target,
        Eigen::Matrix2d& R,
        Eigen::Vector2d& t);

    // 完整的ICP迭代（自动找对应点）
    void icp_iterative(
        std::vector<Point>& source,
        const std::vector<Point>& target,
        int max_iterations = 10);

    // 测试ICP算法
    static void test();

private:
    Point find_nearest(const Point& p, const std::vector<Point>& cloud);
    double compute_error(const std::vector<Point>& source,
                        const std::vector<Point>& target);
};

#endif // ICP_HPP
