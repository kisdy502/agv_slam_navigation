#include <rclcpp/rclcpp.hpp>
#include <iostream>

#include "icp.hpp"
#include "matrix_demo.hpp"

class AlgorithmTestNode : public rclcpp::Node {
public:
    AlgorithmTestNode() : Node("algorithm_test_node") {
        this->declare_parameter<std::string>("type", "ipc");
    }

    void run_test() {
        std::string type;
        this->get_parameter("type", type);

        RCLCPP_INFO(this->get_logger(), "Algorithm test node started, type: %s", type.c_str());

        if (type == "ipc") {
            ICP::test();
        }  else if (type == "matrix") {
            MatrixDemo::test();
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown algorithm type: %s", type.c_str());
        }
    }

private:
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AlgorithmTestNode>();
    node->run_test();

    rclcpp::shutdown();
    return 0;
}
