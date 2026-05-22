#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "demo_action/action/count_to_n.hpp"

using CountToN = demo_action::action::CountToN;
using GoalHandleCountToN = rclcpp_action::ServerGoalHandle<CountToN>;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CountLifecycleServer : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit CountLifecycleServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : LifecycleNode("count_server", options)
    {
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_configure: 创建 Action Server");

        this->action_server_ = rclcpp_action::create_server<CountToN>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "count_to_n",
            std::bind(&CountLifecycleServer::handle_goal,
                      this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CountLifecycleServer::handle_cancel,
                      this, std::placeholders::_1),
            std::bind(&CountLifecycleServer::handle_accepted,
                      this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "CountLifecycleServer 已配置，等待 Goal...");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_activate: 激活");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_deactivate: 停用");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_cleanup: 销毁 Action Server");
        this->action_server_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_shutdown: 关闭");
        this->action_server_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_error(const rclcpp_lifecycle::State &)
    {
        RCLCPP_ERROR(this->get_logger(), "on_error: 出错");
        return CallbackReturn::SUCCESS;
    }

private:
    rclcpp_action::Server<CountToN>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const CountToN::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "收到 Goal: 目标数字 = %d", goal->target_number);

        if (goal->target_number <= 0)
        {
            RCLCPP_WARN(this->get_logger(), "目标数字非法，拒绝 Goal");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleCountToN> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "收到 Cancel 请求，同意取消");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleCountToN> goal_handle)
    {
        std::thread{std::bind(&CountLifecycleServer::execute,
                              this, std::placeholders::_1),
                    goal_handle}
            .detach();
    }

    void execute(const std::shared_ptr<GoalHandleCountToN> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<CountToN::Result>();
        auto feedback = std::make_shared<CountToN::Feedback>();

        rclcpp::Rate loop_rate(2);

        for (int i = 1; i <= goal->target_number; ++i)
        {
            if (goal_handle->is_canceling())
            {
                result->success = false;
                result->message = "任务被取消，当前数到: " + std::to_string(i);
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "任务已取消");
                return;
            }

            feedback->current_number = i;
            feedback->progress_percentage = static_cast<float>(i) / goal->target_number;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Feedback: %d / %d (%.1f%%)",
                        i, goal->target_number, feedback->progress_percentage * 100);

            loop_rate.sleep();
        }

        if (goal_handle->is_active())
        {
            result->success = true;
            result->message = "成功数到 " + std::to_string(goal->target_number);
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "任务完成: %s", result->message.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CountLifecycleServer>();

    // 生命周期节点需要先配置再激活
    node->configure();
    node->activate();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
