import os
import tempfile
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from xacro import process_file

def generate_robot_description(context):
    # Xacro to URDF
    xacro_path = os.path.join(
        get_package_share_directory('jackal_description'),
        'urdf',
        'jackal.urdf.xacro'
    )

    # Process xacro with required arguments
    doc = process_file(xacro_path, mappings={
        'is_sim': 'true',
        'gazebo_controllers': os.path.join(
            get_package_share_directory('jackal_control'),
            'config', 'control.yaml'
        )
    })
    robot_description_xml = doc.toxml()

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

    # Provide robot_description param to ros2_control_node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_xml},
            os.path.join(
                get_package_share_directory('jackal_control'),
                'config', 'control.yaml'
            )
        ],
        output='screen'
    )

    return [rsp_node, control_node]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=generate_robot_description)
    ])
