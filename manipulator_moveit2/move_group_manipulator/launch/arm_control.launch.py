import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_arm").to_moveit_configs()

    # MoveGroupInterface demo executable
    pick_and_place_node = Node(
        name="arm_control",
        package="move_group_manipulator",
        executable="pick_and_place_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    trajectory_buffer_node = Node(
        name="trajectory_buffer_node",
        package="move_group_manipulator",
        executable="trajectory_buffer_node",
        output="screen",
    )

    # fines_serial_node = Node(
    #     name="fines_serial_node",
    #     package="fines_serial",
    #     executable="fines_serial",
    #     output="screen",
    # )

    return LaunchDescription(
        [
            pick_and_place_node, 
            trajectory_buffer_node,
            # fines_serial_node,
        ]
    )
