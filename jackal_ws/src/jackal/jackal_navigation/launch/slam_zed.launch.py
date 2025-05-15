from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_jackal_navigation = FindPackageShare('jackal_navigation')
    pkg_jackal_viz = FindPackageShare('jackal_viz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('jackal_navigation'),
                'config',
                'slam_zed.yaml'
            ),
            description='Full path to the slam_zed.yaml parameter file'
        ),

        # Node(
        #     package='depthimage_to_laserscan',
        #     executable='depthimage_to_laserscan_node',
        #     name='depthimage_to_laserscan',
        #     output='screen',
        #     parameters=[LaunchConfiguration('params_file')],
        #     remappings=[
        #         ('/scan', '/zed/scan'),  # → navsat_transform_node가 구독하는 토픽에 맞춤
        #         ('depth', '/zed/depth/image_raw'),
        #         ('depth_camera_info', '/zed/depth/camera_info')
        #     ],
        # ),

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([pkg_jackal_viz, 'rviz', 'slam.rviz'])],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
