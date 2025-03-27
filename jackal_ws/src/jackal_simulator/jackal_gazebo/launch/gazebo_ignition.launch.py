from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    world_path = PathJoinSubstitution(
        [FindPackageShare('jackal_gazebo'),
        'worlds',
        'jackal_race_world.sdf'],
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_path, '-v', '4', '--render-engine', 'ogre2'],
            output='screen'
        ),
    ])
