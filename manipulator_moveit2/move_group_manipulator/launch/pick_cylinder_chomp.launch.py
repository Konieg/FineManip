from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("manipulator").to_moveit_configs()

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="pick_cylinder_chomp_demo",
        package="move_group_manipulator",
        executable="pick_cylinder_chomp_demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([move_group_demo])
