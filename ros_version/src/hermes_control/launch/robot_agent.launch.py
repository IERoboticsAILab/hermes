from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot_id_arg = DeclareLaunchArgument("robot_id", default_value="r1")
    odom_topic_arg = DeclareLaunchArgument("odom_topic", default_value="/odom")
    cmd_vel_topic_arg = DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel")
    intent_topic_arg = DeclareLaunchArgument("intent_topic", default_value="/hermes/swarm_intent")
    robot_state_topic_arg = DeclareLaunchArgument("robot_state_topic", default_value="/hermes/robot_state_beacon")

    return LaunchDescription(
        [
            robot_id_arg,
            odom_topic_arg,
            cmd_vel_topic_arg,
            intent_topic_arg,
            robot_state_topic_arg,
            Node(
                package="hermes_control",
                executable="robot_state_beacon_node",
                name="robot_state_beacon_node",
                output="screen",
                parameters=[
                    {
                        "robot_id": LaunchConfiguration("robot_id"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "state_topic": LaunchConfiguration("robot_state_topic"),
                    }
                ],
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
                        "intent_topic": LaunchConfiguration("intent_topic"),
                        "robot_states_topic": LaunchConfiguration("robot_state_topic"),
                    }
                ],
            ),
        ]
    )
