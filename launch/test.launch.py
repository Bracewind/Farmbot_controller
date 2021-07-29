import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from farmbot_controller import pkg_name


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "node_prefix",
                default_value=[EnvironmentVariable("USER"), "_"],
                description="Prefix for node names",
            ),
            Node(
                package=pkg_name,
                executable="controller",
                name="farmbot_controller",
                output="screen",
            ),
        ]
    )
