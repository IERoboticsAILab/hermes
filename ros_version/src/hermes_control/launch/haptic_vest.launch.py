from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_arg = DeclareLaunchArgument(
        "vest_config",
        default_value=PathJoinSubstitution([FindPackageShare("hermes_control"), "config", "haptic_vest_pi.yaml"]),
    )
    serial_port_arg = DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0")
    baud_rate_arg = DeclareLaunchArgument("baud_rate", default_value="921600")
    use_serial_output_arg = DeclareLaunchArgument("use_serial_output", default_value="true")
    serial_frame_topic_arg = DeclareLaunchArgument("serial_frame_topic", default_value="/hermes/vest_serial_tx")

    node = Node(
        package="hermes_control",
        executable="haptic_vest_node",
        name="haptic_vest_node",
        output="screen",
        parameters=[
            LaunchConfiguration("vest_config"),
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "baud_rate": ParameterValue(LaunchConfiguration("baud_rate"), value_type=int),
                "use_serial_output": ParameterValue(LaunchConfiguration("use_serial_output"), value_type=bool),
                "serial_frame_topic": LaunchConfiguration("serial_frame_topic"),
            },
        ],
    )

    return LaunchDescription([config_arg, serial_port_arg, baud_rate_arg, use_serial_output_arg, serial_frame_topic_arg, node])
