"""Starts the Stretch sim (original table scene) and the pickup node."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('stretch_simulation'),
            'launch', 'stretch_mujoco_driver.launch.py')),
        launch_arguments={
            'mode': 'navigation',
            'use_robocasa': 'false',
            'use_rviz': 'false',
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('MUJOCO_GL', 'egl'),
        sim,
        Node(package='trash_pickup', executable='pickup', output='screen'),
    ])
