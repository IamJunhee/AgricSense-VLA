# localization_zed.launch.py (수정됨 - jackal_control 기준으로 경로 반영)
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_jackal_control = FindPackageShare('jackal_control')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            pkg_jackal_control, 'config', 'localization_zed.yaml'
        ]), {'use_sim_time': use_sim_time}]
    )

    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            pkg_jackal_control, 'config', 'localization_zed.yaml'
        ]), {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        ekf_node,
        navsat_node
    ])
