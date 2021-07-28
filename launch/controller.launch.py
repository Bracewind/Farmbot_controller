import os
from ament_index_python.packages import get_package_share_directory
import farmbot_simpleurdf
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from farmbot_controller import pkg_name


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package=pkg_name,
                executable="controller",
                name="farmbot_controller",
                output="screen",
            ),
        ]
    )
