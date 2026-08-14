from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bridge_config_arg = DeclareLaunchArgument(
        "bridge_config",
        default_value=PathJoinSubstitution([FindPackageShare("hermes_control"), "config", "vest_serial_bridge_pi.yaml"]),
    )
    vest_config_arg = DeclareLaunchArgument(
        "vest_config",
        default_value=PathJoinSubstitution([FindPackageShare("hermes_control"), "config", "haptic_vest_pi.yaml"]),
    )
    serial_port_arg = DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0")
    baud_rate_arg = DeclareLaunchArgument("baud_rate", default_value="921600")
    deadman_always_true_arg = DeclareLaunchArgument("deadman_always_true", default_value="false")

    bridge_node = Node(
        package="hermes_control",
        executable="vest_serial_bridge_node",
        name="vest_serial_bridge_node",
        output="screen",
        parameters=[
            LaunchConfiguration("bridge_config"),
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "baud_rate": ParameterValue(LaunchConfiguration("baud_rate"), value_type=int),
            },
        ],
    )

    gesture_node = Node(
        package="hermes_control",
        executable="gesture_pipeline_node",
        name="gesture_pipeline_node",
        output="screen",
        parameters=[
            {
                "deadman_always_true": ParameterValue(
                    LaunchConfiguration("deadman_always_true"),
                    value_type=bool,
                ),
            },
        ],
    )

    swarm_node = Node(
        package="hermes_control",
        executable="swarm_control_node",
        name="swarm_control_node",
        output="screen",
        # 30 Hz instead of the 10 Hz default — used to be the dominant
        # ~100 ms latency hop between a fresh command packet and
        # /hermes/swarm_intent landing at downstream consumers.
        parameters=[{"intent_hz": 30.0}],
    )

    haptic_node = Node(
        package="hermes_control",
        executable="haptic_vest_node",
        name="haptic_vest_node",
        output="screen",
        parameters=[
            LaunchConfiguration("vest_config"),
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "baud_rate": ParameterValue(LaunchConfiguration("baud_rate"), value_type=int),
                "use_serial_output": ParameterValue(False, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            bridge_config_arg,
            vest_config_arg,
            serial_port_arg,
            baud_rate_arg,
            deadman_always_true_arg,
            bridge_node,
            gesture_node,
            swarm_node,
            haptic_node,
        ]
    )
