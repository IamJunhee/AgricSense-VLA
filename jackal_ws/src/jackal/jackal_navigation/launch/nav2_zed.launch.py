from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    pkg_jackal_navigation = FindPackageShare('jackal_navigation')
    pkg_jackal_viz = FindPackageShare('jackal_viz')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [pkg_jackal_navigation, 'config', 'nav2_zed.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true')
    
    map_yaml_dir = PathJoinSubstitution(
            [pkg_jackal_navigation, 'maps', 'agricsense_field.yaml'])
    
    map_ply_dir = PathJoinSubstitution(
            [pkg_jackal_navigation, 'maps', 'mid_presentation_rev.ply']),
    

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        output='screen',
        parameters=[params_file]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_yaml_dir
        }]
    )
    
    lifecycle_manager_localization_node=Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[params_file])


    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_nav2_bringup, '/launch/navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_jackal_viz, 'rviz', 'nav2.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    depthimage_to_laserscan_node=Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('/scan', '/zed/scan'),  # → navsat_transform_node가 구독하는 토픽에 맞춤
            ('depth', '/zed/depth/image_raw'),
            ('depth_camera_info', '/zed/depth/camera_info')
        ],
    )

    publish_map_pointcloud=Node(
        package='sensor_tools',
        executable='pub_map_ply',
        name='pub_map_ply',
        output='screen',
        parameters=[{'use_sim_time': True,
                    'map_path': map_ply_dir}]
    )
    
    ld = LaunchDescription()
   
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(depthimage_to_laserscan_node)
    # ld.add_action(amcl_node)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_localization_node)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(rviz_node)
    ld.add_action(publish_map_pointcloud)
    return ld
