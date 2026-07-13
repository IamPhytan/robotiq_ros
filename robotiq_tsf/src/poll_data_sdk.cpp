// SDK-backed ROS 2 wrapper for the Robotiq TSF tactile sensor.
//
// This node owns a RobotiqTactileSensor (from extern/tactile_sensors/sdk_cpp)
// and republishes the SDK's Fingers callback onto the same topics and
// message types as the legacy PollData.cpp node. The USB / packet / parsing
// logic lives in the SDK — this file contains only the bridge into ROS.

#include "rclcpp/rclcpp.hpp"
#include "robotiq_tsf/msg/accelerometer.hpp"
#include "robotiq_tsf/msg/dynamic.hpp"
#include "robotiq_tsf/msg/euler_angle.hpp"
#include "robotiq_tsf/msg/gyroscope.hpp"
#include "robotiq_tsf/msg/sensor.hpp"
#include "robotiq_tsf/msg/static_data.hpp"
#include "robotiq_tsf/msg/timestamp.hpp"
#include "robotiq_tsf/srv/tactile_sensors.hpp"

#include "robotiq_tsf/MadgwickAHRS.h"
#include "robotiq_tsf/device_autodetect.hpp"
#include "robotiq_tsf/sdk_bridge.hpp"

#include "RobotiqTactileSensor.h"
#include "finger_data.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <strings.h>
#include <thread>

namespace msg = robotiq_tsf::msg;
namespace srv = robotiq_tsf::srv;

namespace
{
constexpr int kBiasCalculationIterations = 5000;
constexpr const char *kDefaultDevice = robotiq_tsf::kDefaultSensorDevice;
constexpr float kAccelRes = 2.0f / 32768.0f;
constexpr float kGyroRes = 250.0f / 32768.0f;
// SDK sampling period: 1 ms (~1 kHz request rate).
constexpr unsigned int kSamplePeriodMs = 1;
}

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
        const std::shared_ptr<srv::TactileSensors::Request> req,
        std::shared_ptr<srv::TactileSensors::Response> res);

    rclcpp::Publisher<msg::StaticData>::SharedPtr static_pub_;
    rclcpp::Publisher<msg::Dynamic>::SharedPtr dynamic_pub_;
    rclcpp::Publisher<msg::Accelerometer>::SharedPtr accel_pub_;
    rclcpp::Publisher<msg::Gyroscope>::SharedPtr gyro_pub_;
    rclcpp::Publisher<msg::EulerAngle>::SharedPtr euler_pub_;
    rclcpp::Publisher<msg::Timestamp>::SharedPtr ts_pub_;
    rclcpp::Service<srv::TactileSensors>::SharedPtr service_;

    std::unique_ptr<RobotiqTactileSensor> sensor_;
    msg::Sensor sensors_data_;

    // Optional publish-rate throttle. Default 0 = publish every packet at the
    // sensor's native rate (~700 Hz), as poll_data_node did. A positive
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
    MadgwickFilter filter_[FINGER_COUNT];
    std::chrono::steady_clock::time_point last_update_{};

    std::atomic<bool> stopped_{false};
    std::atomic<uint64_t> frames_received_{0};
};

namespace
{
// The SDK callback is a plain C function pointer with no user-data argument,
// so we reach the node instance through this file-scoped pointer. It is set
// once the node is constructed (before the SDK reader thread starts) and
// cleared in the node's destructor; atomic because the reader thread reads it
// concurrently with those writes.
std::atomic<PollDataSdkNode *> g_node_for_callback{nullptr};

void onSensorData(const Fingers &fingers)
{
    if (auto *node = g_node_for_callback.load(std::memory_order_acquire))
    {
        node->handleFingers(fingers);
    }
}
}

// Stop dispatching to this node, then join the SDK reader thread (in
// sensor_'s destructor) while every member it touches is still alive. Doing
// this here rather than relying on member-destruction order avoids a
// use-after-free: the reader thread outlives the node otherwise.
PollDataSdkNode::~PollDataSdkNode()
{
    g_node_for_callback.store(nullptr, std::memory_order_release);
    sensor_.reset();
}

PollDataSdkNode::PollDataSdkNode()
    : rclcpp::Node("poll_data_sdk")
{
    declare_parameter<std::string>("device", kDefaultDevice);

    // Cap the published topic rate (Hz). Default 0 = unthrottled: publish on
    // every sensor packet, matching the legacy poll_data_node. Set a positive
    // rate to decimate (fusion still runs at the full packet rate).
    const double publish_rate_hz =
        declare_parameter<double>("publish_rate_hz", 0.0);
    publish_period_s_ = publish_rate_hz > 0.0 ? 1.0 / publish_rate_hz : 0.0;

    auto qos = rclcpp::SensorDataQoS();
    static_pub_ = create_publisher<msg::StaticData>("TactileSensor/StaticData", qos);
    dynamic_pub_ = create_publisher<msg::Dynamic>("TactileSensor/Dynamic", qos);
    accel_pub_ = create_publisher<msg::Accelerometer>("TactileSensor/Accelerometer", qos);
    gyro_pub_ = create_publisher<msg::Gyroscope>("TactileSensor/Gyroscope", qos);
    euler_pub_ = create_publisher<msg::EulerAngle>("TactileSensor/EulerAngle", qos);
    ts_pub_ = create_publisher<msg::Timestamp>("TactileSensor/Timestamp", qos);

    service_ = create_service<srv::TactileSensors>(
        "tactile_sensors_service",
        [this](const std::shared_ptr<srv::TactileSensors::Request> req,
               std::shared_ptr<srv::TactileSensors::Response> res)
        { serviceCallback(req, res); });
}

bool PollDataSdkNode::startSensor()
{
    std::string device = get_parameter("device").as_string();

    // If the user left the default device and it isn't present, fall back to
    // the shared autodetect (rq_tsf85_*, then ttyACM*/ttyUSB* by USB
    // descriptor string).
    if (device == kDefaultDevice && !robotiq_tsf::deviceExists(device))
    {
        const std::string detected = robotiq_tsf::autoDetectSensorDevice();
        if (!detected.empty())
        {
            RCLCPP_INFO(get_logger(),
                        "Auto-detected tactile sensor on %s (default %s not present)",
                        detected.c_str(), device.c_str());
            device = detected;
            set_parameter(rclcpp::Parameter("device", device));
        }
        else
        {
            RCLCPP_WARN(get_logger(),
                        "Could not auto-detect tactile sensor; falling back to %s",
                        device.c_str());
        }
    }

    RCLCPP_INFO(get_logger(), "Opening tactile sensor at %s via SDK", device.c_str());

    // The SDK (libserialport) opens the port in raw mode. That is load-bearing,
    // not incidental: the sensor stream is binary, and a freshly re-enumerated
    // CDC-ACM tty defaults to cooked line discipline (ICRNL rewrites 0x0D, IXON
    // eats 0x11/0x13), which mangles the stream and makes the taxels read as
    // rail-to-rail garbage ("flashing red"). This is exactly the bug the old
    // hand-rolled poll_data_node hit (it never forced raw mode); the SDK path is
    // immune only because libserialport sets raw mode for us. If this ever moves
    // off libserialport, the replacement MUST put the tty in raw mode.
    sensor_ = std::make_unique<RobotiqTactileSensor>(device.c_str(), &onSensorData, kSamplePeriodMs);
    if (!sensor_->isConnected())
    {
        RCLCPP_FATAL(get_logger(), "SDK failed to open sensor: %s",
                     sensor_->getLastError());
        sensor_.reset();
        return false;
    }

    // NOTE: the firmware version is purely informational and nothing here
    // uses it, and this sensor/firmware doesn't reliably reply to GET_VERSION
    // (it consistently times out). So we don't query it at all — no point
    // delaying startup for a value we don't need.
    return true;
}

// The sensor can come up opened-but-silent (seen after an unclean shutdown of
// a previous reader: the port opens fine but no packets flow until the stream
// is kicked). Wait for the first callback; if none arrives, cycle the SDK's
// stop/start to re-trigger streaming before giving up.
bool PollDataSdkNode::waitForStreaming()
{
    using namespace std::chrono;
    constexpr int kAttempts = 3;
    constexpr auto kFirstDataTimeout = seconds(2);

    for (int attempt = 1; attempt <= kAttempts; ++attempt)
    {
        const uint64_t before = frames_received_.load();
        const auto deadline = steady_clock::now() + kFirstDataTimeout;
        while (steady_clock::now() < deadline)
        {
            if (frames_received_.load() > before)
                return true;
            std::this_thread::sleep_for(milliseconds(50));
        }
        if (attempt < kAttempts)
        {
            RCLCPP_WARN(get_logger(),
                        "No data from sensor after %lds; cycling the stream "
                        "(attempt %d/%d)",
                        static_cast<long>(kFirstDataTimeout.count()),
                        attempt, kAttempts);
            sensor_->stop();
            std::this_thread::sleep_for(milliseconds(200));
            sensor_->start();
        }
    }
    RCLCPP_FATAL(get_logger(),
                 "Sensor opened but never streamed data; replug the sensor "
                 "(or USB-reset it) and relaunch.");
    return false;
}

void PollDataSdkNode::serviceCallback(
    const std::shared_ptr<srv::TactileSensors::Request> req,
    std::shared_ptr<srv::TactileSensors::Response> res)
{
    RCLCPP_INFO(get_logger(), "TactileSensors service request: [%s]", req->request.c_str());
    if (strcasecmp(req->request.c_str(), "start") == 0)
    {
        stopped_ = false;
        if (sensor_) sensor_->start();
        res->response = true;
        return;
    }
    if (strcasecmp(req->request.c_str(), "stop") == 0)
    {
        stopped_ = true;
        if (sensor_) sensor_->stop();
        res->response = true;
        return;
    }
    RCLCPP_WARN(get_logger(), "Invalid TactileSensors request");
    res->response = false;
}

void PollDataSdkNode::handleFingers(const Fingers &fingers)
{
    frames_received_.fetch_add(1, std::memory_order_relaxed);
    if (stopped_.load())
        return;

    robotiq_tsf::fillSensorMessages(fingers, sensors_data_);

    // Measured dt for the filter integration (the SDK streams at ~700 Hz).
    const auto now_upd = std::chrono::steady_clock::now();
    float dt = 1.0f / 700.0f;  // fallback on the first sample
    if (last_update_.time_since_epoch().count() != 0)
        dt = std::chrono::duration<float>(now_upd - last_update_).count();
    last_update_ = now_upd;

    if (bias_iter_ > kBiasCalculationIterations)
    {
        // Bias-corrected IMU: accel in g, gyro in deg/s.
        const float ax1 = fingers.finger[0].accelerometer[0] * kAccelRes - ax1_bias_;
        const float ay1 = fingers.finger[0].accelerometer[1] * kAccelRes - ay1_bias_;
        const float az1 = fingers.finger[0].accelerometer[2] * kAccelRes - az1_bias_;
        const float ax2 = fingers.finger[1].accelerometer[0] * kAccelRes - ax2_bias_;
        const float ay2 = fingers.finger[1].accelerometer[1] * kAccelRes - ay2_bias_;
        const float az2 = fingers.finger[1].accelerometer[2] * kAccelRes - az2_bias_;
        const float gx1 = fingers.finger[0].gyroscope[0] * kGyroRes - gx1_bias_;
        const float gy1 = fingers.finger[0].gyroscope[1] * kGyroRes - gy1_bias_;
        const float gz1 = fingers.finger[0].gyroscope[2] * kGyroRes - gz1_bias_;
        const float gx2 = fingers.finger[1].gyroscope[0] * kGyroRes - gx2_bias_;
        const float gy2 = fingers.finger[1].gyroscope[1] * kGyroRes - gy2_bias_;
        const float gz2 = fingers.finger[1].gyroscope[2] * kGyroRes - gz2_bias_;

        constexpr float deg_to_rad = static_cast<float>(M_PI / 180.0);
        filter_[0].updateIMU(gx1 * deg_to_rad, gy1 * deg_to_rad, gz1 * deg_to_rad,
                             ax1, ay1, az1, dt);
        filter_[1].updateIMU(gx2 * deg_to_rad, gy2 * deg_to_rad, gz2 * deg_to_rad,
                             ax2, ay2, az2, dt);

        filter_[0].getEulerDeg(sensors_data_.eulerangle.data[0].values[0],
                               sensors_data_.eulerangle.data[0].values[1],
                               sensors_data_.eulerangle.data[0].values[2]);
        filter_[1].getEulerDeg(sensors_data_.eulerangle.data[1].values[0],
                               sensors_data_.eulerangle.data[1].values[1],
                               sensors_data_.eulerangle.data[1].values[2]);
    }
    else if (bias_iter_ == kBiasCalculationIterations)
    {
        gx1_bias_ /= kBiasCalculationIterations;
        gy1_bias_ /= kBiasCalculationIterations;
        gz1_bias_ /= kBiasCalculationIterations;
        gx2_bias_ /= kBiasCalculationIterations;
        gy2_bias_ /= kBiasCalculationIterations;
        gz2_bias_ /= kBiasCalculationIterations;
        ax1_bias_ /= kBiasCalculationIterations;
        ay1_bias_ /= kBiasCalculationIterations;
        az1_bias_ /= kBiasCalculationIterations;
        ax2_bias_ /= kBiasCalculationIterations;
        ay2_bias_ /= kBiasCalculationIterations;
        az2_bias_ /= kBiasCalculationIterations;

        // Gravity in the body frame at rest is the signal, not a bias: seed
        // each filter's quaternion from the accel mean, then stop subtracting
        // the accel offset. Mirrors PollData.cpp's new MadgwickFilter path.
        filter_[0].initFromAccel(ax1_bias_, ay1_bias_, az1_bias_);
        filter_[1].initFromAccel(ax2_bias_, ay2_bias_, az2_bias_);
        ax1_bias_ = ay1_bias_ = az1_bias_ = 0.0f;
        ax2_bias_ = ay2_bias_ = az2_bias_ = 0.0f;
        ++bias_iter_;
    }
    else
    {
        gx1_bias_ += fingers.finger[0].gyroscope[0] * kGyroRes;
        gy1_bias_ += fingers.finger[0].gyroscope[1] * kGyroRes;
        gz1_bias_ += fingers.finger[0].gyroscope[2] * kGyroRes;
        gx2_bias_ += fingers.finger[1].gyroscope[0] * kGyroRes;
        gy2_bias_ += fingers.finger[1].gyroscope[1] * kGyroRes;
        gz2_bias_ += fingers.finger[1].gyroscope[2] * kGyroRes;
        ax1_bias_ += fingers.finger[0].accelerometer[0] * kAccelRes;
        ay1_bias_ += fingers.finger[0].accelerometer[1] * kAccelRes;
        az1_bias_ += fingers.finger[0].accelerometer[2] * kAccelRes;
        ax2_bias_ += fingers.finger[1].accelerometer[0] * kAccelRes;
        ay2_bias_ += fingers.finger[1].accelerometer[1] * kAccelRes;
        az2_bias_ += fingers.finger[1].accelerometer[2] * kAccelRes;
        ++bias_iter_;
    }

    // Decimate publishing to publish_period_s_ (fusion above already ran at
    // full rate). Skip the very first packet's elapsed check by seeding
    // last_pub_ on first use.
    if (publish_period_s_ > 0.0)
    {
        const auto now = std::chrono::steady_clock::now();
        if (last_pub_.time_since_epoch().count() != 0)
        {
            const double dt =
                std::chrono::duration<double>(now - last_pub_).count();
            if (dt < publish_period_s_)
                return;
        }
        last_pub_ = now;
    }

    static_pub_->publish(sensors_data_.staticdata);
    dynamic_pub_->publish(sensors_data_.dynamic);
    accel_pub_->publish(sensors_data_.accelerometer);
    gyro_pub_->publish(sensors_data_.gyroscope);
    ts_pub_->publish(sensors_data_.timestamp);
    if (bias_iter_ > kBiasCalculationIterations)
    {
        euler_pub_->publish(sensors_data_.eulerangle);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PollDataSdkNode>();
    g_node_for_callback.store(node.get(), std::memory_order_release);

    if (!node->startSensor() || !node->waitForStreaming())
    {
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
