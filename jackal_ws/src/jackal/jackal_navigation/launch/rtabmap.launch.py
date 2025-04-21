from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 1. RGB + Depth + CameraInfo 동기화 (approx sync)
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[{'approx_sync': True, 'queue_size': 30}],
            remappings=[
                ('rgb/image', '/zed/image_raw'),
                ('depth/image', '/zed/depth/image_raw'),
                ('rgb/camera_info', '/zed/camera_info'),
                ('rgbd_image', '/rgbd_image')
            ]
        ),

        # 2. 시각적 Odometry
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'approx_sync': True,
                'queue_size': 30,
                'use_sim_time': True
            }],
            remappings=[
                ('rgb/image', '/zed/image_raw'),
                ('depth/image', '/zed/depth/image_raw'),
                ('rgb/camera_info', '/zed/camera_info'),
                ('odom', '/odometry/filtered'),
            ]
        ),

        # 3. RTAB-Map SLAM
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_rgbd': True,
                'use_sim_time': True,
                'approx_sync': True,
                'grid_map': True,
                'queue_size': 30
            }],
            remappings=[
                ('rgb/image', '/zed/image_raw'),
                ('depth/image', '/zed/depth/image_raw'),
                ('rgb/camera_info', '/zed/camera_info'),
                ('odom', '/odometry/filtered'),
            ]
        ),

        # 4. RTAB-Map 시각화
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
