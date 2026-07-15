# Copyright (c) 2026 Robotiq
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# The gripper controller can only activate if the command interfaces it is
# configured to claim exist under the exact names the hardware exports (see
# RobotiqGripperHardwareInterface::export_command_interfaces in
# robotiq_driver). A mismatch is silent until runtime, where
# robotiq_gripper_controller fails to activate.

from pathlib import Path

import yaml

CONFIG = Path(__file__).parents[1] / "config" / "robotiq_controllers.yaml"

# Names exported by robotiq_driver's hardware interface. Kept in sync by
# robotiq_driver's test_robotiq_gripper_hardware_interface, which asserts the
# hardware exports exactly these.
JOINT = "robotiq_85_left_knuckle_joint"
EXPORTED_COMMAND_INTERFACES = {
    f"{JOINT}/position",
    f"{JOINT}/set_gripper_max_velocity",
    f"{JOINT}/set_gripper_max_effort",
}


def gripper_controller_params():
    with open(CONFIG) as f:
        config = yaml.safe_load(f)
    return config["robotiq_gripper_controller"]["ros__parameters"]


def test_max_effort_interface_is_exported():
    params = gripper_controller_params()
    assert params["max_effort_interface"] in EXPORTED_COMMAND_INTERFACES


def test_max_velocity_interface_is_exported():
    params = gripper_controller_params()
    assert params["max_velocity_interface"] in EXPORTED_COMMAND_INTERFACES


def test_joint_matches_exported_interfaces():
    params = gripper_controller_params()
    assert params["joint"] == JOINT
