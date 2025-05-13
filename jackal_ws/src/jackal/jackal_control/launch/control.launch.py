from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time')
    # Configs
    config_jackal_ekf = PathJoinSubstitution(
        [FindPackageShare('jackal_control'),
         'config',
         'localization_zed.yaml'],
    )
    
    filepath_config_twist_mux = PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'config', 'twist_mux.yaml']
    )

    config_imu_filter = PathJoinSubstitution(
        [FindPackageShare('jackal_control'),
         'config',
         'imu_filter.yaml'],
    )
    
    filepath_config_twist_mux = PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'config', 'twist_mux.yaml']
    )
    
    config_jackal_velocity_controller = PathJoinSubstitution(
        [FindPackageShare('jackal_control'),
        'config',
        'control.yaml'],
    )
    gazebo_controllers_arg = DeclareLaunchArgument(
        'gazebo_controllers_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('jackal_control'),
            'config',
            'control.yaml'
        ])
    )

    # Launch Arguments

    robot_description_command_arg = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('jackal_description'), 'urdf', 'jackal.urdf.xacro']
            ),
            ' ',
            'is_sim:=true',
            ' ',
            'gazebo_controllers:=',
            LaunchConfiguration('gazebo_controllers_path'),
        ]
    )

    is_sim = LaunchConfiguration('is_sim', default=False)

    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value=is_sim)

    robot_description_content = ParameterValue(
        Command(LaunchConfiguration('robot_description_command')),
        value_type=str
    )

    # Localization
    localization_group_action = GroupAction([
        # Extended Kalman Filter

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[config_jackal_ekf, {'use_sim_time': use_sim_time}]
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[config_jackal_ekf, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/odometry/filtered', '/odometry/global') 
            ],
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[config_jackal_ekf, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/imu', '/imu/data'),
                ('/odometry/filtered', '/odometry/local')   
            ],
        ),
        
        # Madgwick Filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            output='screen',
            parameters=[config_imu_filter, {'use_sim_time': use_sim_time}],
        ),
    ])
    
    # ROS2 Controllers
    control_group_action = GroupAction([
        # ROS2 Control
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description_content},
                        config_jackal_velocity_controller],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            remappings=[
                ('/jackal_velocity_controller/odom', '/odom')
            ],
            condition=UnlessCondition(is_sim)
        ),

        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            parameters=[config_jackal_velocity_controller],
            output='screen',
        ),

        # Velocity Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['jackal_velocity_controller'],
            output='screen',
        ),
        
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('/cmd_vel_out', '/jackal_velocity_controller/cmd_vel_unstamped')},
            parameters=[filepath_config_twist_mux]
        )


    ])
    
    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('/cmd_vel_out', '/jackal_velocity_controller/cmd_vel_unstamped')},
        parameters=[filepath_config_twist_mux]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(gazebo_controllers_arg)
    ld.add_action(robot_description_command_arg)
    ld.add_action(is_sim_arg)
    ld.add_action(localization_group_action)
    ld.add_action(control_group_action)
    ld.add_action(node_twist_mux)
    return ld