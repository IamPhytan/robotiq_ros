// Unit tests for the AHRS fusion helpers (robotiq_tsf/fusion.hpp): per-finger
// MCU-timestamp dt derivation (with its [lo, hi] clamp), still-detection and
// bias-EMA trim. These regressed silently once (the SDK node integrated on an
// unbounded host-clock dt with a frozen gyro bias), so they're pinned here.

#include "robotiq_tsf/fusion.hpp"

#include <gtest/gtest.h>

using robotiq_tsf::deriveDt;
using robotiq_tsf::sampleIsStill;
using robotiq_tsf::trimBias;

namespace
{
constexpr float kLo = 1e-4f;   // 0.1 ms
constexpr float kHi = 0.1f;    // 100 ms
}

TEST(DeriveDt, NormalDeltaIsMsToSeconds)
{
    // 10 ms apart -> 0.01 s, within [lo, hi].
    EXPECT_FLOAT_EQ(deriveDt(1000, 1010, kLo, kHi), 0.010f);
}

TEST(DeriveDt, FirstSampleAfterSeedReturnsZero)
{
    // prev == 0 means "not yet seeded" -> skip integration.
    EXPECT_FLOAT_EQ(deriveDt(0, 1234, kLo, kHi), 0.0f);
}

TEST(DeriveDt, DuplicateTimestampReturnsZero)
{
    EXPECT_FLOAT_EQ(deriveDt(1000, 1000, kLo, kHi), 0.0f);
}

TEST(DeriveDt, BackwardsTimestampReturnsZero)
{
    EXPECT_FLOAT_EQ(deriveDt(2000, 1000, kLo, kHi), 0.0f);
}

TEST(DeriveDt, StalledDeltaIsClampedToHi)
{
    // 500 ms stall -> capped at 100 ms so a delayed sample can't inject an
    // outsized integration step.
    EXPECT_FLOAT_EQ(deriveDt(1000, 1500, kLo, kHi), kHi);
}

TEST(SampleIsStill, TrueWhenGyroSmallAndAccelNearOneG)
{
    EXPECT_TRUE(sampleIsStill(0.1f, -0.1f, 0.05f,
                              0.0f, 0.0f, 1.0f, 0.8f, 0.05f));
}

TEST(SampleIsStill, FalseWhenRotating)
{
    EXPECT_FALSE(sampleIsStill(5.0f, 0.0f, 0.0f,
                               0.0f, 0.0f, 1.0f, 0.8f, 0.05f));
}

TEST(SampleIsStill, FalseUnderLinearAcceleration)
{
    // |a| well away from 1 g -> not stationary even with zero angular rate.
    EXPECT_FALSE(sampleIsStill(0.0f, 0.0f, 0.0f,
                               0.0f, 0.0f, 1.5f, 0.8f, 0.05f));
}

TEST(TrimBias, NudgesTowardResidualByAlpha)
{
    // bias 0.10, residual 0.20, alpha 0.5 -> 0.10 + 0.5 * 0.20 = 0.20.
    EXPECT_FLOAT_EQ(trimBias(0.10f, 0.20f, 0.5f), 0.20f);
}

TEST(TrimBias, ZeroResidualLeavesBiasUnchanged)
{
    EXPECT_FLOAT_EQ(trimBias(0.42f, 0.0f, 0.0005f), 0.42f);
}
