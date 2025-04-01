import os
import tempfile
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from xacro import process_file
from pathlib import Path

def generate_robot_description(context):
    # Xacro를 URDF로 변환
    xacro_path = os.path.join(
        get_package_share_directory('jackal_description'),
        'urdf',
        'jackal.urdf.xacro'
    )
    urdf_path = Path(__file__).resolve().parent.parent.parent / 'jackal_description' / 'urdf' / 'jackal.urdf'
    
    doc = process_file(xacro_path, mappings={
        'is_sim': 'true',
        'gazebo_controllers': os.path.join(
            get_package_share_directory('jackal_control'),
            'config', 'control.yaml'
        )
    })

    robot_description_xml = doc.toxml()

    # URDF를 임시 파일로 저장 (spawn_entity를 위한 파일)
    with open(urdf_path, 'w') as f:
        f.write(robot_description_xml)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_xml,
        }],
        output='screen'
    )

    

    return [robot_state_publisher_node]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=generate_robot_description)
    ])