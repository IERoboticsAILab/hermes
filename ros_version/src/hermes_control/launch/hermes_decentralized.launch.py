from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value="r1",
        description="Robot ID for this local decentralized agent instance.",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odom",
        description="Odometry topic for this robot.",
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="Command velocity topic for this robot.",
    )

    return LaunchDescription(
        [
            robot_id_arg,
            odom_topic_arg,
            cmd_vel_topic_arg,
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
            Node(
                package="hermes_control",
                executable="decentralized_robot_agent_node",
                name="decentralized_robot_agent_node",
                output="screen",
                parameters=[
                    {
                        "robot_id": LaunchConfiguration("robot_id"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    }
                ],
            ),
            Node(
                package="hermes_control",
                executable="robot_state_beacon_node",
                name="robot_state_beacon_node",
                output="screen",
                parameters=[
                    {
                        "robot_id": LaunchConfiguration("robot_id"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                    }
                ],
            ),
        ]
    )
