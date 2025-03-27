from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue  # ✅ 이거 추가

def generate_launch_description():
    jackal_description_path = FindPackageShare('jackal_description')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(  # ✅ 여기도 수정
                    Command([
                        'xacro ',
                        PathJoinSubstitution([jackal_description_path, 'sdf', 'jackal.sdf.xacro']),
                        ' ',
                        'zed_mount_path:=', PathJoinSubstitution([jackal_description_path, 'sdf', 'zed_mount.sdf']),
                        ' ',
                        'plugin_path:=', PathJoinSubstitution([jackal_description_path, 'sdf', 'jackal_ignition_plugins.sdf'])
                    ]),
                    value_type=str  # ✅ string으로 명시!
                )
            }]
        )
    ])
