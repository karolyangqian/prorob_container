from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robot_control'), '/robot_control.launch.py'
        ])
    )

    waypoint_server_node = Node(
        package='waypoint_server',
        executable='waypoint_server_node',
        name='waypoint_server',
        output='screen',
    )

    waypoint_client_node = Node(
        package='waypoint_client',
        executable='waypoint_client_node',
        name='waypoint_client',
        output='screen',
    )

    delayed_client = TimerAction(
        period=1.0,
        actions=[waypoint_client_node],
    )

    return LaunchDescription([
        robot_control_launch,
        waypoint_server_node,
        delayed_client,
    ])
