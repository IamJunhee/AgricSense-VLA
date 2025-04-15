import os
import tempfile
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from xacro import process_file
from pathlib import Path
def generate_robot_description(context):
    jackal_description_path = FindPackageShare('jackal_description')
    xacro_path = os.path.join(
        get_package_share_directory('jackal_description'),
        'urdf',
        'jackal.urdf.xacro'
    )
    
    urdf_path = Path(__file__).resolve().parent.parent.parent / 'jackal_description' / 'urdf' / 'jackal.urdf'
    sdf_path = Path(__file__).resolve().parent.parent.parent / 'jackal_description' / 'sdf' / 'jackal.sdf'
    
    doc = process_file(xacro_path, mappings={
        'is_sim': 'true',
        'gazebo_controllers': os.path.join(
            get_package_share_directory('jackal_control'),
            'config', 'control.yaml'
        )
    })
    robot_description_xml = doc.toxml()
    
    # with open(sdf_path, 'w') as f:
    #     f.write(robot_description_xml)
    
    # Save URDF to a temporary file
    tmp_urdf = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf')
    tmp_urdf.write(robot_description_xml)
    tmp_urdf.close()

    # Publish robot_description as a topic
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_xml}],
        output='screen'
    )
    return [rsp_node]
            
def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=generate_robot_description)
    ])
