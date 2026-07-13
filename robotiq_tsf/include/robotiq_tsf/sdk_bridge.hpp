#ifndef ROBOTIQ_TSF_SDK_BRIDGE_HPP
#define ROBOTIQ_TSF_SDK_BRIDGE_HPP

#include "robotiq_tsf/msg/sensor.hpp"

#include "finger_data.h"

namespace robotiq_tsf
{

// Copy one SDK Fingers sample into the aggregate Sensor message: staticdata
// taxels, dynamic, accelerometer, gyroscope and per-finger timestamps.
// (EulerAngle/Quaternion are filled by the caller's AHRS fusion, not here.)
void fillSensorMessages(const Fingers &fingers, msg::Sensor &out);

}  // namespace robotiq_tsf

#endif  // ROBOTIQ_TSF_SDK_BRIDGE_HPP
