"""Bring up the TSF-85 driver + a minimal RViz tactile heatmap.

  ros2 launch robotiq_tsf tactile_viz.launch.py            # driver + viz + rviz
  ros2 launch robotiq_tsf tactile_viz.launch.py rviz:=false

Assumes the sensor is connected (see docker/run.sh sensor for device mapping).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('robotiq_tsf')
    default_rviz = os.path.join(pkg_share, 'rviz', 'tactile_viz.rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz with the tactile viz config.')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz,
        description='Path to an RViz config file.')
    input_topic_arg = DeclareLaunchArgument(
        'input_topic', default_value='TactileSensor/StaticData',
        description='Raw StaticData topic published by the driver.')

    # Sensor driver (poller:= arg declared there).
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('robotiq_tsf'), 'launch', 'tsf_driver.launch.py'])))

    viz = Node(
        package='robotiq_tsf', executable='tactile_viz_node',
        name='tactile_viz', output='screen',
        parameters=[{'input_topic': LaunchConfiguration('input_topic')}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription(
        [rviz_arg, rviz_config_arg, input_topic_arg, driver, viz, rviz])
