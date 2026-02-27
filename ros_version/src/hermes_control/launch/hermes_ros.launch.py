from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="hermes_control",
                executable="gesture_pipeline_node",
                name="gesture_pipeline_node",
                output="screen",
            ),
            Node(
                package="hermes_control",
                executable="swarm_control_node",
                name="swarm_control_node",
                output="screen",
            ),
        ]
    )
