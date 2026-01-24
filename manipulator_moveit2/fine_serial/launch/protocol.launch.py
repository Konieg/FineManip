import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():

    package_path = get_package_share_directory("protocol")
    config_path = os.path.join(package_path, 'config')
    config_file = "protocol.yaml"

    pcd2pgm = Node(
        package='protocol',
        executable='protocol',
        name='protocol_navigation',
        output='screen',
        parameters=[PathJoinSubstitution([config_path, config_file])]
    )

    ld = LaunchDescription()
    ld.add_action(pcd2pgm)

    return ld
