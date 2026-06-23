from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    env_file_arg = DeclareLaunchArgument(
        'env_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('bt_planner'), 'environment', 'env.json']
        ),
        description='Absolute path to the environment JSON file',
    )

    simulator_node = Node(
        package='bt_planner',
        executable='simulator_node.py',
        name='simulator',
        arguments=[LaunchConfiguration('env_file')],
        output='screen',
    )

    # Delay robot node to give the simulator time to start its action server
    robot_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='bt_planner',
                executable='robot_node.py',
                name='robot',
                arguments=[LaunchConfiguration('env_file')],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        env_file_arg,
        simulator_node,
        robot_node,
    ])
