// Unit tests for sdk_bridge: SDK Fingers sample -> aggregate Sensor message
// field mapping (taxels, dynamic, accelerometer, gyroscope, timestamps).

#include <cstdint>

#include <gtest/gtest.h>

#include "robotiq_tsf/sdk_bridge.hpp"

namespace
{

Fingers makeSample()
{
    Fingers fingers{};
    for (int f = 0; f < FINGER_COUNT; ++f)
    {
        FingerData &d = fingers.finger[f];
        for (int i = 0; i < FINGER_STATIC_TACTILE_COUNT; ++i)
            d.staticTactile[i] = static_cast<uint16_t>(1000*(f + 1) + i);
        d.dynamicTactile[0] = static_cast<int16_t>(f == 0 ? -123 : 456);
        for (int i = 0; i < 3; ++i)
        {
            d.accelerometer[i] = static_cast<int16_t>(-100*(f + 1) - i);
            d.gyroscope[i] = static_cast<int16_t>(200*(f + 1) + i);
        }
        d.timestamp = 40000 + f;
    }
    return fingers;
}

TEST(SdkBridge, MapsAllFields)
{
    const Fingers fingers = makeSample();
    robotiq_tsf::msg::Sensor out;
    robotiq_tsf::fillSensorMessages(fingers, out);

    for (int f = 0; f < FINGER_COUNT; ++f)
    {
        for (int i = 0; i < FINGER_STATIC_TACTILE_COUNT; ++i)
            EXPECT_EQ(out.staticdata.taxels[f].values[i], 1000*(f + 1) + i)
                << "finger " << f << " taxel " << i;
        EXPECT_EQ(out.dynamic.data[f].value, f == 0 ? -123 : 456);
        for (int i = 0; i < 3; ++i)
        {
            EXPECT_EQ(out.accelerometer.data[f].values[i], -100*(f + 1) - i);
            EXPECT_EQ(out.gyroscope.data[f].values[i], 200*(f + 1) + i);
        }
        EXPECT_EQ(out.timestamp.values[f], 40000 + f);
    }
}

TEST(SdkBridge, OverwritesPreviousSample)
{
    robotiq_tsf::msg::Sensor out;
    robotiq_tsf::fillSensorMessages(makeSample(), out);

    Fingers zeroed{};
    robotiq_tsf::fillSensorMessages(zeroed, out);
    for (int f = 0; f < FINGER_COUNT; ++f)
    {
        for (int i = 0; i < FINGER_STATIC_TACTILE_COUNT; ++i)
            EXPECT_EQ(out.staticdata.taxels[f].values[i], 0);
        EXPECT_EQ(out.dynamic.data[f].value, 0);
        EXPECT_EQ(out.timestamp.values[f], 0);
    }
}

TEST(SdkBridge, NegativeImuValuesSurvive)
{
    Fingers fingers{};
    fingers.finger[0].accelerometer[0] = -32768;
    fingers.finger[0].gyroscope[2] = -1;
    fingers.finger[1].dynamicTactile[0] = -32768;

    robotiq_tsf::msg::Sensor out;
    robotiq_tsf::fillSensorMessages(fingers, out);
    EXPECT_EQ(out.accelerometer.data[0].values[0], -32768);
    EXPECT_EQ(out.gyroscope.data[0].values[2], -1);
    EXPECT_EQ(out.dynamic.data[1].value, -32768);
}

}  // namespace
