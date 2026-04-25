#ifndef AGV_GLOBAL_PLANNER__ASTAR_PLANNER_HPP_
#define AGV_GLOBAL_PLANNER__ASTAR_PLANNER_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace agv_global_planner
{

struct Point
{
    int x, y;
    Point(int x = 0, int y = 0) : x(x), y(y) {}
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
};

} // namespace agv_global_planner

namespace std
{
template <>
struct hash<agv_global_planner::Point>
{
    size_t operator()(const agv_global_planner::Point& p) const
    {
        return hash<int>()(p.x) ^ (hash<int>()(p.y) << 1);
    }
};
} // namespace std

namespace agv_global_planner
{

struct NodeInfo
{
    Point parent;
    double g;
    double h;
    double f;
    bool in_open;
    bool in_closed;
    NodeInfo() : parent(-1, -1), g(0), h(0), f(0), in_open(false), in_closed(false) {}
};

class AStarPlanner
{
public:
    using HeuristicFunc = double (*)(const Point&, const Point&);

    AStarPlanner();
    
    // 设置地图（OccupancyGrid → 2D grid, 1=free, 0=obstacle）
    void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg,
                int unknown_threshold = 50, int occupied_threshold = 50);

    geometry_msgs::msg::Pose gridToWorldPose(const Point& grid, double theta = 0.0);
    
    // 设置起点和终点（世界坐标 → 栅格坐标）
    bool setStart(const geometry_msgs::msg::Point& world_pt);
    bool setGoal(const geometry_msgs::msg::Point& world_pt);
    
    // 执行规划，返回 Path 消息（若成功）
    bool plan(nav_msgs::msg::Path& path_msg);
    
    // 设置启发式函数类型
    void setHeuristic(HeuristicFunc func) { heuristic_ = func; }

    // 设置运动参数（是否允许对角移动、直线/对角代价）
    void setMotionParams(bool allow_diagonal, double straight_cost, double diag_cost);
    
    // 静态启发式函数
    static double manhattan(const Point& a, const Point& b);
    static double euclidean(const Point& a, const Point& b);
    static double diagonal(const Point& a, const Point& b); // 对角线距离
    
private:
    bool worldToGrid(const geometry_msgs::msg::Point& world, Point& grid);
    
    bool isValid(int x, int y) const;
    
    // 地图数据
    std::vector<std::vector<int>> grid_;   // 1=free, 0=obstacle
    int width_, height_;
    double origin_x_, origin_y_, resolution_;
    
    // 规划起点终点
    Point start_, goal_;
    bool start_valid_, goal_valid_;
    
    // 启发式函数
    HeuristicFunc heuristic_;
    
    // 移动代价配置
    bool allow_diagonal_;   // 是否允许八方向移动
    double straight_cost_;  // 直线移动代价
    double diag_cost_;      // 对角线移动代价（通常 sqrt(2)）
};

} // namespace agv_global_planner

#endif // AGV_GLOBAL_PLANNER__ASTAR_PLANNER_HPP_