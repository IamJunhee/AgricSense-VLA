from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    world_path = PathJoinSubstitution(
        [FindPackageShare('jackal_gazebo'),
        'worlds',
        'jackal_race_world.sdf'],
    )
    launch_jackal_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'launch', 'teleop_base.launch.py'])))

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
             # IMU
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU@/world/default/model/jackal/link/imu_link/sensor/imu_sensor/imu',

            # GPS
            '/gps@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat@/world/default/model/jackal/link/navsat_link/sensor/gps_sensor/navsat',

            # Odometry
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry@/model/jackal/odometry',

            '/zed_camera/rgb/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image@/world/default/model/jackal/link/.../sensor/zed_rgb_camera/image',
            '/zed_camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo@/world/default/model/jackal/link/.../sensor/zed_rgb_camera/camera_info',

            # Depth camera
            '/zed_camera/depth/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image@/world/default/model/jackal/link/zed_2i_left_lens_gazebo/sensor/zed_sensor/depth_image',
            '/zed_camera/depth/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo@/world/default/model/jackal/link/zed_2i_left_lens_gazebo/sensor/zed_sensor/camera_info',
            '/zed_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked@/world/default/model/jackal/link/zed_2i_left_lens_gazebo/sensor/zed_sensor/depth_image/points',
        ],
        output='screen'
    )
    


    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_path, '-v', '4', '--render-engine', 'ogre2'],
            output='screen'
        ),
        launch_jackal_teleop_base,
        bridge
    ])
