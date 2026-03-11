from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="hermes_control",
                executable="swarm_control_node",
                name="swarm_control_node",
                output="screen",
            ),
            Node(
                package="hermes_control",
                executable="keyboard_teleop_node",
                name="keyboard_teleop_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
