// Entry point for poll_data_sdk_node. The node class lives in poll_data_sdk.cpp
// (declared in poll_data_sdk_node.hpp) so it can be unit-tested without hardware.

#include "robotiq_tsf/poll_data_sdk_node.hpp"

#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // The node registers itself for the SDK callback in its constructor.
    auto node = std::make_shared<PollDataSdkNode>();

    if (!node->startSensor() || !node->waitForStreaming())
    {
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
