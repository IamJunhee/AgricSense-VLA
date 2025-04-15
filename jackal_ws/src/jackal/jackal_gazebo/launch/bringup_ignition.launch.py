import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

def generate_launch_description():
    # --------------------------------------------------------------------------
    # 1) Gather paths
    # --------------------------------------------------------------------------
    jackal_description_path = FindPackageShare('jackal_description')
    jackal_viz_path = FindPackageShare('jackal_viz')
    
    xacro_file = PathJoinSubstitution([jackal_description_path, 'sdf', 'jackal.sdf.xacro'])
    zed_mount_path = PathJoinSubstitution([jackal_description_path, 'sdf', 'zed_mount.sdf'])
    wheel_path = PathJoinSubstitution([jackal_description_path, 'sdf', 'wheel.sdf.xacro'])
    output_sdf_path = PathJoinSubstitution([jackal_description_path, 'sdf', 'jackal.sdf'])
    
    # Where we'll store the generated SDF file
    # Option 1: Temporary dir
    # tmp_sdf_file = os.path.join(tempfile.gettempdir(), 'jackal_generated.sdf')
    # Option 2: Or a fixed path in the workspace
    

    # --------------------------------------------------------------------------
    # 2) Generate SDF content from xacro
    # --------------------------------------------------------------------------
    # Use `Command` substitution to get the xacro output as a string
    # This runs: xacro <xacro_file> zed_mount_path:=... wheel_path:=...
    ExecuteProcess(
        cmd=[
            'ros2', 
            'run', 
            'xacro', 
            'xacro',          # 네 번째 토큰: 실제 실행될 xacro 노드
            xacro_file,       # 다섯 번째 토큰: xacro 파일 경로
            'zed_mount_path:=',
            zed_mount_path,
            'wheel_path:=',
            wheel_path,
            '>', 
            output_sdf_path
        ],
        shell=False,
        output='screen'
    )


    # We'll store the result in a function that writes to disk

    # --------------------------------------------------------------------------
    # 3) Spawn robot in Ignition using the generated SDF file
    # --------------------------------------------------------------------------
    spawn = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'jackal',
            '-x', '0', '-y', '0', '-z', '0',
            '-file', output_sdf_path # Use the file we just wrote
        ]
    )

    # --------------------------------------------------------------------------
    # 4) Optionally include the rest of your launch stuff
    # --------------------------------------------------------------------------
    # Example: your ignition world file
    ignition_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jackal_gazebo'), 'launch', 'gazebo_ignition.launch.py'])
        )
    )

    # Example: your robot_state_publisher or URDF-based TF
    # description_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([jackal_description_path, 'launch', 'description_ignition.launch.py'])
    #     )
    # )

    # Example: RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([jackal_viz_path, 'rviz', 'jackal_ign.rviz'])
        ],
        output='screen'
    )

    # --------------------------------------------------------------------------
    # 5) Build final LaunchDescription
    # --------------------------------------------------------------------------
    return LaunchDescription([
        # Step A) Create + write the SDF file

        # Step B) Launch Ignition world
        ignition_world_launch,

        # Step C) Launch robot model in Ignition
        spawn,

        # Step D) Launch your URDF-based TF (optional)
        # description_launch,

        # Step E) Launch RViz (optional)
        rviz,
    ])
