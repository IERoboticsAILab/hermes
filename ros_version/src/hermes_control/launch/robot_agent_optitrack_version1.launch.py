from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    robot_id_arg = DeclareLaunchArgument("robot_id", default_value="r1")
    odom_topic_arg = DeclareLaunchArgument("odom_topic", default_value="/odom")
    cmd_vel_topic_arg = DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel")
    cmd_vel_stamped_arg = DeclareLaunchArgument("cmd_vel_stamped", default_value="false")
    cmd_vel_frame_id_arg = DeclareLaunchArgument("cmd_vel_frame_id", default_value="")
    intent_topic_arg = DeclareLaunchArgument("intent_topic", default_value="/hermes/swarm_intent")
    robot_state_topic_arg = DeclareLaunchArgument("robot_state_topic", default_value="/hermes/robot_state_beacon")
    expected_state_frame_arg = DeclareLaunchArgument("expected_state_frame", default_value="optitrack")

    return LaunchDescription(
        [
            robot_id_arg,
            odom_topic_arg,
            cmd_vel_topic_arg,
            cmd_vel_stamped_arg,
            cmd_vel_frame_id_arg,
            intent_topic_arg,
            robot_state_topic_arg,
            expected_state_frame_arg,
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
                        "cmd_vel_stamped": ParameterValue(LaunchConfiguration("cmd_vel_stamped"), value_type=bool),
                        "cmd_vel_frame_id": LaunchConfiguration("cmd_vel_frame_id"),
                        "use_beacon_pose_for_self": True,
                        "intent_topic": LaunchConfiguration("intent_topic"),
                        "robot_states_topic": LaunchConfiguration("robot_state_topic"),
                        "expected_state_frame": LaunchConfiguration("expected_state_frame"),
                    }
                ],
            ),
        ]
    )
