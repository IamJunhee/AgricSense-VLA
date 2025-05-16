from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    agricsense_path_config_path = PathJoinSubstitution(
        [FindPackageShare('gemma_ros_client'), 'config', 'agricsense_path.yaml']
    )

    prompt_path = PathJoinSubstitution(
        [FindPackageShare('gemma_ros_client'), 'resource', 'prompt_ko']
    )

    farm_info = PathJoinSubstitution(
        [FindPackageShare('gemma_ros_client'), 'resource', 'farm_info']
    )

    pannel_html = PathJoinSubstitution(
        [FindPackageShare('gemma_ros_client'), 'resource', 'pannel', "index.html"]
    )

    webview_runner_node = Node(
        package='gemma_ros_client',
        executable="webview",
        name='webview_panel_runner',
        output='screen',
        arguments=[
            pannel_html
        ]
    )

    declare_control_by_human = DeclareLaunchArgument(
        'control_by_human',
        default_value='true',
        description='Agricsense by human')
    
    control_by_human = LaunchConfiguration("control_by_human")

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare('jackal_gazebo'),
        'launch',
        'jackal_world.launch.py'],
    )

    nav2_launch = PathJoinSubstitution(
        [FindPackageShare('jackal_navigation'),
        'launch',
        'nav2_zed.launch.py'],
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch])
    )

    nav2_jackal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch])
    )

    depth_convert = Node(
        package='gemma_ros_client',
        executable='depth_converter_node'
    )

    agricsense_server = Node(
        name="agricsense",
        package='gemma_ros_client',
        executable='agricsense_server',
        parameters=[
            agricsense_path_config_path,
            {
                "control_by_human": control_by_human,
                "prompt_template_path": prompt_path,
                "csv_path": farm_info
            }
        ]
    )

    gemma_server = Node(
        package='gemma_ros_client',
        executable='gemma_server'
    )


    ld = LaunchDescription()
    ld.add_action(declare_control_by_human)
    ld.add_action(gazebo_sim)
    ld.add_action(nav2_jackal)
    ld.add_action(gemma_server)
    ld.add_action(depth_convert)
    ld.add_action(agricsense_server)
    # ld.add_action(webview_runner_node)

    return ld