from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	serial_port_arg = DeclareLaunchArgument(
		'serial_port',
		default_value='/dev/ttyUSB0',
		description='Serial port used to send drivetrain commands',
	)
	config_file = PathJoinSubstitution([
		FindPackageShare('robot_control'),
		'config',
		'robot_control.yaml',
	])

	return LaunchDescription([
		serial_port_arg,
		Node(
			package='robot_control',
			executable='robot_control',
			name='robot_control',
			output='screen',
			emulate_tty=True,
			parameters=[config_file, {'serial_port': LaunchConfiguration('serial_port')}],
		),
	])
