from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='behave_package',
        #     executable='move_to_goal_node',
        #     name='move_to_goal_node',
        #     output='screen'
        # ),
        Node(
            package='behave_package',
            executable='spin_command_node',
            name='spin_command_node',
            output='screen'
        ),
        Node(
            package='behave_package',
            executable='move_and_spin_node',
            name='move_and_spin_node',
            output='screen'
        ),
    ])

