from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('jackal_navigation'),  # 패키지 이름
        'config', 
        'rtab.yaml'
    ])

    return LaunchDescription([
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('rgb/image', '/zed/image_raw'),
                ('depth/image', '/zed/depth/image_raw'),
                ('rgb/camera_info', '/zed/camera_info'),
                ('rgbd_image', '/rgbd_image')
            ]
        ),
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('rgb/image', '/zed/image_raw'),
                ('depth/image', '/zed/depth/image_raw'),
                ('rgb/camera_info', '/zed/camera_info'),
                ('odom', '/odometry/visual'),
            ]
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('rgb/image', '/zed/image_raw'),
                ('depth/image', '/zed/depth/image_raw'),
                ('rgb/camera_info', '/zed/camera_info'),
                ('odom', '/odometry/visual'),
            ]
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('odom', '/odometry/visual'),
                ('rgbd_image', '/rgbd_image')
            ]
        )
    ])
