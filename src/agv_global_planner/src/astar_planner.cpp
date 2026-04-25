#include "agv_global_planner/astar_planner.hpp"
#include <algorithm>
#include <queue>
#include <tuple>

namespace agv_global_planner
{

AStarPlanner::AStarPlanner()
    : width_(0), height_(0), origin_x_(0.0), origin_y_(0.0), resolution_(0.05),
      start_valid_(false), goal_valid_(false), heuristic_(&manhattan),
      allow_diagonal_(false), straight_cost_(1.0), diag_cost_(1.414)
{
}

void AStarPlanner::setMotionParams(bool allow_diagonal, double straight_cost, double diag_cost)
{
    allow_diagonal_ = allow_diagonal;
    straight_cost_ = straight_cost;
    diag_cost_ = diag_cost;
}

void AStarPlanner::setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg,
                          int unknown_threshold, int occupied_threshold)
{
    width_ = msg->info.width;
    height_ = msg->info.height;
    origin_x_ = msg->info.origin.position.x;
    origin_y_ = msg->info.origin.position.y;
    resolution_ = msg->info.resolution;
    
    grid_.assign(height_, std::vector<int>(width_, 0));
    
    for (int y = 0; y < height_; ++y)
    {
        for (int x = 0; x < width_; ++x)
        {
            int idx = y * width_ + x;
            int8_t occ = msg->data[idx];
            // 占用值: 0-100, -1 表示未知
            if (occ < 0)
                grid_[y][x] = (unknown_threshold >= 50) ? 1 : 0; // 未知是否可通行，按参数决定
            else if (occ >= occupied_threshold)
                grid_[y][x] = 0; // 障碍物
            else
                grid_[y][x] = 1; // 空闲
        }
    }
}

bool AStarPlanner::worldToGrid(const geometry_msgs::msg::Point& world, Point& grid)
{
    double gx = (world.x - origin_x_) / resolution_;
    double gy = (world.y - origin_y_) / resolution_;
    int ix = static_cast<int>(std::round(gx));
    int iy = static_cast<int>(std::round(gy));
    if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_)
        return false;
    grid.x = ix;
    grid.y = iy;
    return true;
}

geometry_msgs::msg::Pose AStarPlanner::gridToWorldPose(const Point& grid, double theta)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = origin_x_ + grid.x * resolution_;
    pose.position.y = origin_y_ + grid.y * resolution_;
    pose.position.z = 0.0;
    // 可选：设置朝向（沿路径方向）
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    pose.orientation = tf2::toMsg(q);
    return pose;
}

bool AStarPlanner::setStart(const geometry_msgs::msg::Point& world_pt)
{
    start_valid_ = worldToGrid(world_pt, start_);
    return start_valid_;
}

bool AStarPlanner::setGoal(const geometry_msgs::msg::Point& world_pt)
{
    goal_valid_ = worldToGrid(world_pt, goal_);
    return goal_valid_;
}

bool AStarPlanner::isValid(int x, int y) const
{
    if (x < 0 || y < 0 || x >= width_ || y >= height_)
        return false;
    return grid_[y][x] == 1;
}

double AStarPlanner::manhattan(const Point& a, const Point& b)
{
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

double AStarPlanner::euclidean(const Point& a, const Point& b)
{
    int dx = a.x - b.x;
    int dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

double AStarPlanner::diagonal(const Point& a, const Point& b)
{
    int dx = std::abs(a.x - b.x);
    int dy = std::abs(a.y - b.y);
    return (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
}

bool AStarPlanner::plan(nav_msgs::msg::Path& path_msg)
{
    if (!start_valid_ || !goal_valid_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("AStarPlanner"), "Start or goal not set");
        return false;
    }
    if (!isValid(start_.x, start_.y) || !isValid(goal_.x, goal_.y))
    {
        RCLCPP_ERROR(rclcpp::get_logger("AStarPlanner"), "Start or goal is obstacle");
        return false;
    }
    
    // 存储节点信息
    std::unordered_map<Point, NodeInfo> node_map;
    
    // 优先队列: (f, g, x, y)
    using QueueNode = std::tuple<double, double, int, int>;
    auto cmp = [](const QueueNode& a, const QueueNode& b) {
        return std::get<0>(a) > std::get<0>(b);
    };
    std::priority_queue<QueueNode, std::vector<QueueNode>, decltype(cmp)> open_set(cmp);
    
    // 初始化起点
    NodeInfo& start_info = node_map[start_];
    start_info.g = 0;
    start_info.h = heuristic_(start_, goal_);
    start_info.f = start_info.g + start_info.h;
    start_info.in_open = true;
    start_info.parent = start_;
    open_set.emplace(start_info.f, start_info.g, start_.x, start_.y);
    
    // 邻居方向
    std::vector<std::pair<int,int>> dirs;
    if (allow_diagonal_)
    {
        dirs = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    }
    else
    {
        dirs = {{1,0},{-1,0},{0,1},{0,-1}};
    }
    
    while (!open_set.empty())
    {
        auto [f_val, g_val, x, y] = open_set.top();
        open_set.pop();
        Point current(x, y);
        
        NodeInfo& cur_info = node_map[current];
        if (cur_info.in_closed) continue;
        cur_info.in_closed = true;
        
        if (current == goal_)
        {
            // 回溯路径
            std::vector<Point> grid_path;
            Point trace = goal_;
            while (!(trace == start_))
            {
                grid_path.push_back(trace);
                trace = node_map[trace].parent;
            }
            grid_path.push_back(start_);
            std::reverse(grid_path.begin(), grid_path.end());
            
            // 转换为 Path 消息
            path_msg.header.stamp = rclcpp::Clock().now();
            path_msg.header.frame_id = "map";
            path_msg.poses.clear();
            for (size_t i = 0; i < grid_path.size(); ++i)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path_msg.header;
                // 计算朝向（指向下一个点）
                double theta = 0.0;
                if (i < grid_path.size() - 1)
                {
                    double dx = (grid_path[i+1].x - grid_path[i].x) * resolution_;
                    double dy = (grid_path[i+1].y - grid_path[i].y) * resolution_;
                    theta = std::atan2(dy, dx);
                }
                else if (grid_path.size() > 1)
                {
                    double dx = (grid_path[i].x - grid_path[i-1].x) * resolution_;
                    double dy = (grid_path[i].y - grid_path[i-1].y) * resolution_;
                    theta = std::atan2(dy, dx);
                }
                pose.pose = gridToWorldPose(grid_path[i], theta);
                path_msg.poses.push_back(pose);
            }
            return true;
        }
        
        for (const auto& dir : dirs)
        {
            int nx = x + dir.first;
            int ny = y + dir.second;
            if (!isValid(nx, ny)) continue;
            
            double move_cost;
            if (dir.first != 0 && dir.second != 0)
                move_cost = diag_cost_;
            else
                move_cost = straight_cost_;
            
            double tentative_g = cur_info.g + move_cost;
            Point neighbor(nx, ny);
            NodeInfo& nb_info = node_map[neighbor];
            
            if (nb_info.in_closed && tentative_g >= nb_info.g)
                continue;
            
            if (!nb_info.in_open || tentative_g < nb_info.g)
            {
                nb_info.parent = current;
                nb_info.g = tentative_g;
                nb_info.h = heuristic_(neighbor, goal_);
                nb_info.f = nb_info.g + nb_info.h;
                nb_info.in_open = true;
                open_set.emplace(nb_info.f, nb_info.g, nx, ny);
            }
        }
    }
    
    RCLCPP_ERROR(rclcpp::get_logger("AStarPlanner"), "No path found");
    return false;
}

} // namespace agv_global_planner