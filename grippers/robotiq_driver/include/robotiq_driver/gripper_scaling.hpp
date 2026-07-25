// Copyright (c) 2022 PickNik, Inc.
// Copyright (c) 2026 Robotiq, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//! \brief Conversions between ros2_control's SI joint quantities and the
//!        gripper's 0..255 register counts.
//! Pulled out of the hardware interface so the arithmetic — the part that
//! silently mis-commands a gripper when it is wrong — is unit-testable
//! without a gripper, a serial port, or a controller manager.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace robotiq_driver {

// The usable travel band of the 2F fingers in register counts. The gripper
// reports and accepts 0..255, but the extremes are outside the mechanical
// range, so the joint mapping is anchored on these two.
inline constexpr uint8_t kGripperMinPos = 3;
inline constexpr uint8_t kGripperMaxPos = 230;
inline constexpr uint8_t kGripperRange = kGripperMaxPos - kGripperMinPos;

//! Register count (gPO: 0 open .. 255 closed) -> joint position, linear over
//! the usable travel band. \p closed_position is the joint value at a fully
//! closed gripper and sets the unit: the mapping carries whatever the URDF
//! uses. The shipped 2F descriptions model the knuckle as a revolute joint,
//! so there it is radians.
[[nodiscard]] inline double jointPositionFromRegister(uint8_t position, double closed_position)
{
   return closed_position * (position - kGripperMinPos) / kGripperRange;
}

//! Joint position -> register count (rPR), in the same unit as
//! \p closed_position. Counts outside the byte are clamped; the caller is
//! responsible for rejecting a NaN command.
//!
//! Truncates rather than rounds, so a position read off the gripper and
//! commanded straight back can land one count below where it came from.
[[nodiscard]] inline uint8_t registerFromJointPosition(double joint_position, double closed_position)
{
   const double counts = (joint_position / closed_position) * kGripperRange + kGripperMinPos;
   return static_cast<uint8_t>(std::clamp(counts, 0.0, 255.0));
}

//! An SI speed or force command -> its 0x00..0xFF register (rSP / rFR),
//! as the fraction of \p maximum it represents. Sign is ignored: the
//! registers set a magnitude, direction comes from the position request.
//! Truncates, as above.
[[nodiscard]] inline uint8_t registerFromFractionOf(double value, double maximum)
{
   const double fraction = std::clamp(std::fabs(value) / maximum, 0.0, 1.0);
   return static_cast<uint8_t>(fraction * 0xFF);
}
} // namespace robotiq_driver
