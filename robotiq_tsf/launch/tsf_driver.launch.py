"""Bring up the TSF-85 driver (StaticData publisher) only.

Included by tactile_viz.launch.py and gripper_tactile_viz.launch.py; usable
standalone for a headless driver.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    poller_arg = DeclareLaunchArgument(
        'poller', default_value='poll_data_node',
        description='Driver executable publishing StaticData. Override to '
                    'point the viz at an alternative poller.')

    # Sensor driver — autodetects the device.
    poll = Node(
        package='robotiq_tsf', executable=LaunchConfiguration('poller'),
        name='poll_data', output='screen')

    return LaunchDescription([poller_arg, poll])
