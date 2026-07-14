// Regression test pinning the ROS surface of poll_data_sdk_node: the set of
// advertised topics and the service. This is exactly what silently regressed
// once (the TactileSensor/Quaternion topic was dropped in the SDK-node port),
// and the sdk_bridge/device_autodetect gtests don't cover the node's own graph.
//
// The node's publishers/service are created in its constructor, independently
// of startSensor()/waitForStreaming(), so this needs no hardware sensor.

#include "robotiq_tsf/poll_data_sdk_node.hpp"

#include "rclcpp/rclcpp.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
// Wait (spinning to let intra-process graph discovery settle) until `node`
// reports all of `expected`, or the timeout elapses. Returns the last snapshot.
template <typename QueryFn>
std::map<std::string, std::vector<std::string>> waitForNames(
    const rclcpp::Node::SharedPtr &node,
    const std::vector<std::string> &expected,
    QueryFn query)
{
    using namespace std::chrono;
    const auto deadline = steady_clock::now() + seconds(10);
    std::map<std::string, std::vector<std::string>> names;
    while (steady_clock::now() < deadline)
    {
        rclcpp::spin_some(node);
        names = query();
        bool all = true;
        for (const auto &e : expected)
            if (names.find(e) == names.end())
            {
                all = false;
                break;
            }
        if (all)
            break;
        std::this_thread::sleep_for(milliseconds(50));
    }
    return names;
}
}  // namespace

TEST(PollDataSdkNodeSurface, AdvertisesExpectedTopicsAndService)
{
    rclcpp::init(0, nullptr);
    {
        auto node = std::make_shared<PollDataSdkNode>();

        const std::vector<std::string> expected_topics = {
            "/TactileSensor/StaticData",
            "/TactileSensor/Dynamic",
            "/TactileSensor/Accelerometer",
            "/TactileSensor/Gyroscope",
            "/TactileSensor/EulerAngle",
            "/TactileSensor/Quaternion",
            "/TactileSensor/Timestamp",
        };

        const auto topics = waitForNames(
            node, expected_topics,
            [&] { return node->get_topic_names_and_types(); });
        for (const auto &t : expected_topics)
            EXPECT_TRUE(topics.find(t) != topics.end())
                << "missing advertised topic: " << t;

        const std::vector<std::string> expected_services = {
            "/tactile_sensors_service",
        };
        const auto services = waitForNames(
            node, expected_services,
            [&] { return node->get_service_names_and_types(); });
        EXPECT_TRUE(services.find("/tactile_sensors_service") != services.end())
            << "missing tactile_sensors_service";
    }
    rclcpp::shutdown();
}
