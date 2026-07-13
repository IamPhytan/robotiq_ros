#include "robotiq_tsf/sdk_bridge.hpp"

#include <cstring>

namespace robotiq_tsf
{

void fillSensorMessages(const Fingers &fingers, msg::Sensor &out)
{
    // The memcpy below is only safe while the ROS message arrays and the SDK
    // finger arrays are the same size. Fail the build if either side changes.
    static_assert(sizeof(out.staticdata.taxels[0].values) ==
                      sizeof(fingers.finger[0].staticTactile),
                  "StaticData taxel array size != SDK staticTactile array");
    static_assert(sizeof(out.accelerometer.data[0].values) ==
                      sizeof(fingers.finger[0].accelerometer),
                  "Accelerometer array size != SDK accelerometer array");
    static_assert(sizeof(out.gyroscope.data[0].values) ==
                      sizeof(fingers.finger[0].gyroscope),
                  "Gyroscope array size != SDK gyroscope array");

    for (int f = 0; f < FINGER_COUNT; ++f)
    {
        const FingerData &finger = fingers.finger[f];

        std::memcpy(out.staticdata.taxels[f].values.data(),
                    finger.staticTactile, sizeof(finger.staticTactile));

        out.dynamic.data[f].value = finger.dynamicTactile[0];

        std::memcpy(out.accelerometer.data[f].values.data(),
                    finger.accelerometer, sizeof(finger.accelerometer));

        std::memcpy(out.gyroscope.data[f].values.data(),
                    finger.gyroscope, sizeof(finger.gyroscope));

        out.timestamp.values[f] = finger.timestamp;
    }
}

}  // namespace robotiq_tsf
