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

#include <robotiq_driver/hardware_parameters.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>

#include <rclcpp/logging.hpp>

namespace robotiq_driver {
namespace {
constexpr auto kComPortParam = "COM_port";
constexpr auto kBaudrateParam = "baudrate";
constexpr auto kTimeoutParam = "timeout";
constexpr auto kSlaveAddressParam = "slave_address";
constexpr auto kConnectionFrequencyParam = "connection_frequency";
constexpr auto kClosedPositionParam = "gripper_closed_position";
constexpr auto kMaxSpeedParam = "gripper_max_speed";
constexpr auto kMaxForceParam = "gripper_max_force";
constexpr auto kSpeedMultiplierParam = "gripper_speed_multiplier";
constexpr auto kForceMultiplierParam = "gripper_force_multiplier";
constexpr auto kActivationTimeoutParam = "activation_timeout";
constexpr auto kUseDummyParam = "use_dummy";

// The dummy is off unless the parameter reads as anything other than this.
constexpr auto kUseDummyDefault = "false";

//! Look a parameter up and convert it with \p parse. An absent parameter
//! keeps \p fallback silently; a present but unparsable one keeps it loudly.
template <typename T, typename Parse>
T parameterOr(const hardware_interface::HardwareInfo& info,
              const rclcpp::Logger& logger,
              const char* name,
              T fallback,
              Parse parse)
{
   const auto entry = info.hardware_parameters.find(name);
   if(entry == info.hardware_parameters.end())
   {
      return fallback;
   }
   try
   {
      return parse(entry->second);
   }
   catch(const std::exception& e)
   {
      RCLCPP_ERROR(logger, "Ignoring malformed '%s' parameter '%s': %s", name, entry->second.c_str(), e.what());
      return fallback;
   }
}

double asDouble(const std::string& text)
{
   return std::stod(text);
}
} // namespace

GripperParameters parseParameters(const hardware_interface::HardwareInfo& info, const rclcpp::Logger& logger)
{
   GripperParameters parameters;

   parameters.connection.serial.port =
      parameterOr<std::string>(info, logger, kComPortParam, kComPortDefault, [](const std::string& text) {
         return text;
      });
   parameters.connection.serial.baudrate =
      parameterOr<uint32_t>(info, logger, kBaudrateParam, kBaudrateDefault, [](const std::string& text) {
         return static_cast<uint32_t>(std::stoul(text));
      });
   // The URDF spells the timeout in seconds; SerialConfig wants milliseconds.
   parameters.connection.serial.timeout =
      parameterOr<std::chrono::milliseconds>(info, logger, kTimeoutParam, kTimeoutDefault, [](const std::string& text) {
         return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(std::stod(text)));
      });
   // Base 16: the address is written as the manual prints it ("0x9").
   parameters.connection.modbusSlaveAddress =
      parameterOr<uint8_t>(info, logger, kSlaveAddressParam, kSlaveAddressDefault, [](const std::string& text) {
         return static_cast<uint8_t>(std::stoul(text, nullptr, 16));
      });
   parameters.connection.connectionFrequency =
      parameterOr<double>(info, logger, kConnectionFrequencyParam, kConnectionFrequencyDefault, asDouble);

   // No default: the joint mapping is meaningless without the gripper's
   // closed angle, and guessing one would silently mis-scale every command.
   parameters.closed_position = std::stod(info.hardware_parameters.at(kClosedPositionParam));

   parameters.max_speed = parameterOr<double>(info, logger, kMaxSpeedParam, parameters.max_speed, asDouble);
   parameters.max_force = parameterOr<double>(info, logger, kMaxForceParam, parameters.max_force, asDouble);
   parameters.speed_multiplier =
      parameterOr<double>(info, logger, kSpeedMultiplierParam, parameters.speed_multiplier, asDouble);
   parameters.force_multiplier =
      parameterOr<double>(info, logger, kForceMultiplierParam, parameters.force_multiplier, asDouble);
   parameters.activation_timeout = parameterOr<std::chrono::milliseconds>(
      info,
      logger,
      kActivationTimeoutParam,
      parameters.activation_timeout,
      [](const std::string& text) {
         return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(std::stod(text)));
      });

   // Anything but the literal "false" enables the dummy, "0" included.
   parameters.use_dummy = info.hardware_parameters.count(kUseDummyParam) != 0
                       && info.hardware_parameters.at(kUseDummyParam) != kUseDummyDefault;

   return parameters;
}
} // namespace robotiq_driver
