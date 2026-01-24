import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

import re
def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    robot_name_in_model = 'manipulator'
    package_name = 'manipulator_description'
    urdf_file_name = "manipulator.urdf"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # 将 xacro 或 URDF 文件处理成 XML 字符串 ---
    xacro_file_path = os.path.join(pkg_share, 'urdf', urdf_file_name)
    robot_description_config = xacro.process_file(xacro_file_path)
    robot_desc_raw = remove_comments(robot_description_config.toxml())
    
    # 从字符串中移除 XML 声明头
    # The lxml parser in spawn_entity.py does not like the XML declaration
    robot_desc = robot_desc_raw[robot_desc_raw.find('<'):]

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, {"publish_frequency":15.0}, 
                    {'robot_description': robot_desc},                 # 使用处理过的字符串
        ] 
    )

    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, 
                   '-topic', 'robot_description'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        start_gazebo_cmd,
        robot_state_publisher_node,
        spawn_entity_cmd,
        joint_state_broadcaster_spawner,
    ])