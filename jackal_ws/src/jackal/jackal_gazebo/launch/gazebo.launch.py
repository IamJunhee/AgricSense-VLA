from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    LogInfo
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
# 타입 힌트용으로 사용되었던 List, Optional, Action, ProzessExit 임포트는 제거했습니다.
# LogInfo, Node, IncludeLaunchDescription 등은 Action의 서브클래스이므로,
# 콜백 함수가 이들을 리스트에 담아 반환하는 것은 문제가 없습니다.

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

    # Gazebo client (필요에 따라 주석 해제)
    # gzclient = ExecuteProcess(
    #     cmd=['gzclient'],
    #     output='screen',
    # )

    # robot_description 토픽을 발행 (URDF 로드 등)
    launch_jackal_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('jackal_description'), 'launch', 'description.launch.py']
            )
        )
    )

    # 로봇 스폰 노드
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'jackal',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'
        ],
        output='screen'
    )

    # Jackal 제어 launch 파일
    launch_jackal_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('jackal_control'), 'launch', 'control.launch.py']
        )),
        launch_arguments=[('is_sim', 'True')]
    )

    # spawn_robot 노드 종료 시 실행될 콜백 함수 (타입 힌트 제거)
    def on_spawn_robot_exit_callback(event, context):
        if event.returncode == 0:
            # spawn_robot이 성공적으로 종료되었을 때
            return [
                LogInfo(msg="[INFO] Robot spawn successful. Launching jackal_control..."),
                launch_jackal_control
            ]
        else:
            # spawn_robot이 오류로 종료되었을 때
            return [
                LogInfo(msg=f"[ERROR] Robot spawn failed with exit code: {event.returncode}. Not launching jackal_control.")
            ]

    # spawn_robot 노드의 종료 이벤트를 감지하는 핸들러 등록
    event_handler_spawn_robot_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=on_spawn_robot_exit_callback
        )
    )
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(gzserver)
    # ld.add_action(gzclient)
    
    ld.add_action(launch_jackal_description)
    
    ld.add_action(spawn_robot)
    
    ld.add_action(event_handler_spawn_robot_exit)
    
    return ld