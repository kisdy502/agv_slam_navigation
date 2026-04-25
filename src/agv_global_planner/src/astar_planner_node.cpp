#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "agv_global_planner/astar_planner.hpp"
#include <memory>
#include <string>

class AStarPlannerNode : public rclcpp::Node
{
public:
    AStarPlannerNode() : Node("astar_planner_node")
    {
        RCLCPP_INFO(this->get_logger(), "A* Planner Node started");
        
        // 统一在此声明所有参数
        this->declare_parameter<bool>("allow_diagonal", false);
        this->declare_parameter<std::string>("heuristic", "manhattan");
        this->declare_parameter<double>("straight_cost", 1.0);
        this->declare_parameter<double>("diag_cost", 1.414);
        this->declare_parameter<double>("start_x", 0.0);
        this->declare_parameter<double>("start_y", 0.0);
        
        planner_ = std::make_unique<agv_global_planner::AStarPlanner>();
        
        // 设置启发式函数
        std::string heur = this->get_parameter("heuristic").as_string();
        if (heur == "euclidean")
            planner_->setHeuristic(agv_global_planner::AStarPlanner::euclidean);
        else if (heur == "diagonal")
            planner_->setHeuristic(agv_global_planner::AStarPlanner::diagonal);
        else
            planner_->setHeuristic(agv_global_planner::AStarPlanner::manhattan);
        
        // 设置移动代价参数
        bool allow_diag = this->get_parameter("allow_diagonal").as_bool();
        double straight_cost = this->get_parameter("straight_cost").as_double();
        double diag_cost = this->get_parameter("diag_cost").as_double();
        planner_->setMotionParams(allow_diag, straight_cost, diag_cost);
        
        // 订阅地图
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&AStarPlannerNode::mapCallback, this, std::placeholders::_1));
        
        // 订阅目标点
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&AStarPlannerNode::goalCallback, this, std::placeholders::_1));
        
        // 发布全局路径
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_plan", 10);
    }
    
private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_received_ = true;
        planner_->setMap(msg);
        RCLCPP_INFO(this->get_logger(), "Map received: %dx%d, resolution %.3f", 
                    msg->info.width, msg->info.height, msg->info.resolution);
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!map_received_)
        {
            RCLCPP_WARN(this->get_logger(), "Map not received yet, cannot plan");
            return;
        }
        
        // 获取起点参数（不再重复声明）
        double start_x = this->get_parameter("start_x").as_double();
        double start_y = this->get_parameter("start_y").as_double();
        
        // 设置目标点
        geometry_msgs::msg::Point goal_pt;
        goal_pt.x = msg->pose.position.x;
        goal_pt.y = msg->pose.position.y;
        if (!planner_->setGoal(goal_pt))
        {
            RCLCPP_ERROR(this->get_logger(), "Goal is outside map bounds");
            return;
        }
        
        // 设置起点
        geometry_msgs::msg::Point start_pt;
        start_pt.x = start_x;
        start_pt.y = start_y;
        if (!planner_->setStart(start_pt))
        {
            RCLCPP_ERROR(this->get_logger(), "Start is outside map bounds");
            return;
        }
        
        // 规划
        nav_msgs::msg::Path path;
        if (planner_->plan(path))
        {
            path_pub_->publish(path);
            RCLCPP_INFO(this->get_logger(), "Path published with %zu poses", path.poses.size());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
        }
    }
    
    std::unique_ptr<agv_global_planner::AStarPlanner> planner_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    bool map_received_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AStarPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}