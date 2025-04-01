from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
]

def generate_launch_description():


    jackal_urdf_path = Path(__file__).resolve().parent.parent.parent.parent /'jackal' / 'jackal_description' / 'urdf' / 'jackal.urdf'

    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
        EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
        '/usr/share/gazebo-11/models/:',
        str(Path(get_package_share_directory('jackal_description')).parent.resolve())])

    world_path = LaunchConfiguration('world_path')

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', '--verbose', world_path],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'jackal', '-file', str(jackal_urdf_path)],
        output='screen'
    )

    # Launch description.launch.py (추가 인자 전달하지 않음)
    launch_jackal_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('jackal_description'), 'launch', 'description.launch.py']
            )
        )
    )

    # Launch jackal_control/control.launch.py (마찬가지로 추가 인자 전달하지 않음)
    launch_jackal_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('jackal_control'), 'launch', 'control.launch.py']
        )),
        launch_arguments=[('is_sim', 'True')]
    )

    launch_jackal_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('jackal_control'), 'launch', 'teleop_base.launch.py']
        ))
    )
    # 포인트클라우드 생성 노드 추가
    # pointcloud_node = Node(
    #     package='depth_image_proc',
    #     executable='point_cloud_xyzrgb_node',
    #     name='pointcloud_generator',
    #     remappings=[
    #         ('image_rect', '/zed/image_raw'),
    #         ('camera_info', '/zed/camera_info'),
    #         ('image_depth', '/zed/depth/image_raw'),
    #         ('points', '/zed/points')
    #     ],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )
    
    # reliable_relay_node = Node(
    #     package='sensor_tools',
    #     executable='reliable_relay',
    #     name='reliable_relay_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(launch_jackal_description)
    ld.add_action(spawn_robot)
    ld.add_action(launch_jackal_control)
    ld.add_action(launch_jackal_teleop_base)

    return ld
