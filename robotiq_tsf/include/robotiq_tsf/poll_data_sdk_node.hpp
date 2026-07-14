#ifndef ROBOTIQ_TSF_POLL_DATA_SDK_NODE_HPP
#define ROBOTIQ_TSF_POLL_DATA_SDK_NODE_HPP

// SDK-backed ROS 2 wrapper for the Robotiq TSF tactile sensor.
//
// This node owns a RobotiqTactileSensor (from extern/tactile_sensors/sdk_cpp)
// and republishes the SDK's Fingers callback onto the same topics and message
// types as the legacy PollData.cpp node. The USB / packet / parsing logic lives
// in the SDK — the node contains only the bridge into ROS + the AHRS fusion.

#include "rclcpp/node.hpp"  // brings rclcpp::{Node, Publisher, Service}

// Sensor is the aggregate message and transitively defines every per-field
// message type the publisher members are typed on (StaticData, Dynamic, …), so
// the individual msg headers are pulled in by the .cpp (where create_publisher
// needs their typesupport), not here.
#include "robotiq_tsf/msg/sensor.hpp"
#include "robotiq_tsf/srv/tactile_sensors.hpp"

#include "robotiq_tsf/MadgwickAHRS.h"  // MadgwickFilter (by-value array member)
#include "robotiq_tsf/fusion.hpp"      // AhrsConfig (by-value member)

#include "finger_data.h"  // Fingers, FINGER_COUNT

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>

// Defined by the SDK (extern/tactile_sensors/sdk_cpp); forward-declared here so
// this header doesn't pull in libserialport. The node's out-of-line destructor
// keeps the unique_ptr happy with the incomplete type.
class RobotiqTactileSensor;

class PollDataSdkNode : public rclcpp::Node
{
public:
    PollDataSdkNode();
    ~PollDataSdkNode() override;

    bool startSensor();
    bool waitForStreaming();
    void handleFingers(const Fingers &fingers);

private:
    void serviceCallback(
        const std::shared_ptr<robotiq_tsf::srv::TactileSensors::Request> req,
        std::shared_ptr<robotiq_tsf::srv::TactileSensors::Response> res);

    rclcpp::Publisher<robotiq_tsf::msg::StaticData>::SharedPtr static_pub_;
    rclcpp::Publisher<robotiq_tsf::msg::Dynamic>::SharedPtr dynamic_pub_;
    rclcpp::Publisher<robotiq_tsf::msg::Accelerometer>::SharedPtr accel_pub_;
    rclcpp::Publisher<robotiq_tsf::msg::Gyroscope>::SharedPtr gyro_pub_;
    rclcpp::Publisher<robotiq_tsf::msg::EulerAngle>::SharedPtr euler_pub_;
    rclcpp::Publisher<robotiq_tsf::msg::Quaternion>::SharedPtr quat_pub_;
    rclcpp::Publisher<robotiq_tsf::msg::Timestamp>::SharedPtr ts_pub_;
    rclcpp::Service<robotiq_tsf::srv::TactileSensors>::SharedPtr service_;

    std::unique_ptr<RobotiqTactileSensor> sensor_;
    robotiq_tsf::msg::Sensor sensors_data_;

    // Publish-rate throttle: set via the publish_rate_hz parameter, from which
    // this is derived as 1/publish_rate_hz. Default 0 = publish every packet at
    // the sensor's native rate (~700 Hz), as poll_data_node did. A positive
    // publish_rate_hz decimates the publishes for downstream consumers that
    // don't need the full rate; the IMU fusion + bias calc still run on every
    // packet regardless (fusion accuracy needs the full rate).
    // Read/written only from the single SDK callback thread.
    double publish_period_s_ = 0.0;
    std::chrono::steady_clock::time_point last_pub_{};

    int bias_iter_ = 0;
    float ax1_bias_ = 0, ay1_bias_ = 0, az1_bias_ = 0;
    float ax2_bias_ = 0, ay2_bias_ = 0, az2_bias_ = 0;
    float gx1_bias_ = 0, gy1_bias_ = 0, gz1_bias_ = 0;
    float gx2_bias_ = 0, gy2_bias_ = 0, gz2_bias_ = 0;

    // AHRS filter — one instance per finger; integrated on every SDK packet.
    robotiq_tsf::AhrsConfig ahrs_cfg_;
    MadgwickFilter filter_[FINGER_COUNT];
    // Per-finger MCU timestamp (ms) of the last integrated sample; 0 = not yet
    // seeded (start of stream, or just after the bias calibration).
    uint64_t last_ts_ms_[FINGER_COUNT] = {0, 0};

    std::atomic<bool> stopped_{false};
    std::atomic<uint64_t> frames_received_{0};
};

#endif  // ROBOTIQ_TSF_POLL_DATA_SDK_NODE_HPP
