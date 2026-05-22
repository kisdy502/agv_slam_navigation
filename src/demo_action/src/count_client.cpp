#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "demo_action/action/count_to_n.hpp"

using CountToN = demo_action::action::CountToN;
using GoalHandleCountToN = rclcpp_action::ClientGoalHandle<CountToN>;

class CountClient : public rclcpp::Node
{
public:
    explicit CountClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("count_client", options)
    {
        this->client_ = rclcpp_action::create_client<CountToN>(this, "count_to_n");

        if (!this->client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action Server 未上线，退出");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = CountToN::Goal();
        goal_msg.target_number = 8;

        RCLCPP_INFO(this->get_logger(), "发送 Goal: 目标 = %d", goal_msg.target_number);

        auto send_goal_options = rclcpp_action::Client<CountToN>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            std::bind(&CountClient::goal_response_callback, this, std::placeholders::_1);

        send_goal_options.feedback_callback =
            std::bind(&CountClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

        send_goal_options.result_callback =
            std::bind(&CountClient::result_callback, this, std::placeholders::_1);

        auto goal_handle_future = this->client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<CountToN>::SharedPtr client_;
    GoalHandleCountToN::SharedPtr goal_handle_;

    void goal_response_callback(const GoalHandleCountToN::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal 被服务端拒绝");
            rclcpp::shutdown();
        }
        else
        {
            this->goal_handle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Goal 已被接受");

            // 测试取消，如果要验证完成回调，取消这段代码注释掉
            std::thread([this]()
                        {
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    if (this->goal_handle_) {
                        // RCLCPP_INFO(this->get_logger(), "发送 Cancel 请求！");
                        // this->client_->async_cancel_goal(this->goal_handle_);
                    } })
                .detach();
        }
    }

    void feedback_callback(
        GoalHandleCountToN::SharedPtr,
        const std::shared_ptr<const CountToN::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(),
                    "收到 Feedback: 当前=%d, 进度=%.1f%%",
                    feedback->current_number,
                    feedback->progress_percentage * 100);
    }

    void result_callback(const GoalHandleCountToN::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "结果: 成功, %s", result.result->message.c_str());
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "结果: 被取消, %s", result.result->message.c_str());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "结果: 异常中断");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "结果: 未知状态");
            break;
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}