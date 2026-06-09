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

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': '/dev/video0',
            'pixel_format': 'mjpeg2rgb',
            'image_width': 640,
            'image_height': 480,
        }],
    )

    aruco_node = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        name='aruco_tracker',
        parameters=[{
            'cam_base_topic': '/image_raw',
            'marker_size': 0.07,
        }],
    )

    docking_controller_node = Node(
        package='aruco_detector',
        executable='aruco_docking_controller',
        name='aruco_docking_controller',
        output='screen',
        parameters=[{
            'kp': 0.5,
            'ki': 0.01,
            'kd': 0.1,
            'control_rate': 20.0,
            'error_tolerance': 0.02,
            'max_linear_x': 0.5,
        }],
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
        usb_cam_node,
        aruco_node,
        docking_controller_node,
        robot_control_launch,
        waypoint_server_node,
        delayed_client,
    ])
