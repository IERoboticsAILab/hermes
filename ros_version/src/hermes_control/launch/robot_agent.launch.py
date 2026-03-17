from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    robot_id_arg = DeclareLaunchArgument("robot_id", default_value="r1")
    odom_topic_arg = DeclareLaunchArgument("odom_topic", default_value="/odom")
    cmd_vel_topic_arg = DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel")
    cmd_vel_stamped_arg = DeclareLaunchArgument(
        "cmd_vel_stamped",
        default_value="false",
        description="Publish geometry_msgs/TwistStamped instead of geometry_msgs/Twist on cmd_vel_topic.",
    )
    cmd_vel_frame_id_arg = DeclareLaunchArgument(
        "cmd_vel_frame_id",
        default_value="",
        description="Optional frame_id for TwistStamped cmd_vel output.",
    )
    use_local_io_bridge_arg = DeclareLaunchArgument(
        "use_local_io_bridge",
        default_value="true",
        description="Isolate local odom/cmd_vel through a localhost-only bridge process on each robot.",
    )
    pose_udp_port_arg = DeclareLaunchArgument("pose_udp_port", default_value="15000")
    cmd_udp_port_arg = DeclareLaunchArgument("cmd_udp_port", default_value="15001")
    cmd_timeout_ms_arg = DeclareLaunchArgument("cmd_timeout_ms", default_value="500")
    intent_topic_arg = DeclareLaunchArgument("intent_topic", default_value="/hermes/swarm_intent")
    robot_state_topic_arg = DeclareLaunchArgument("robot_state_topic", default_value="/hermes/robot_state_beacon")
    global_frame_arg = DeclareLaunchArgument("global_frame", default_value="map")
    base_frame_arg = DeclareLaunchArgument("base_frame", default_value="base_link")
    use_tf_pose_arg = DeclareLaunchArgument("use_tf_pose", default_value="true")
    fallback_to_odom_arg = DeclareLaunchArgument("fallback_to_odom", default_value="false")

    return LaunchDescription(
        [
            robot_id_arg,
            odom_topic_arg,
            cmd_vel_topic_arg,
            cmd_vel_stamped_arg,
            cmd_vel_frame_id_arg,
            use_local_io_bridge_arg,
            pose_udp_port_arg,
            cmd_udp_port_arg,
            cmd_timeout_ms_arg,
            intent_topic_arg,
            robot_state_topic_arg,
            global_frame_arg,
            base_frame_arg,
            use_tf_pose_arg,
            fallback_to_odom_arg,
            Node(
                package="hermes_control",
                executable="local_robot_io_node",
                name="local_robot_io_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_local_io_bridge")),
                additional_env={
                    "ROS_LOCALHOST_ONLY": "1",
                    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
                },
                parameters=[
                    {
                        "robot_id": LaunchConfiguration("robot_id"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "cmd_vel_stamped": ParameterValue(LaunchConfiguration("cmd_vel_stamped"), value_type=bool),
                        "cmd_vel_frame_id": LaunchConfiguration("cmd_vel_frame_id"),
                        "global_frame": LaunchConfiguration("global_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "use_tf_pose": ParameterValue(LaunchConfiguration("use_tf_pose"), value_type=bool),
                        "fallback_to_odom": ParameterValue(LaunchConfiguration("fallback_to_odom"), value_type=bool),
                        "pose_udp_host": "127.0.0.1",
                        "pose_udp_port": LaunchConfiguration("pose_udp_port"),
                        "cmd_udp_bind_host": "127.0.0.1",
                        "cmd_udp_port": LaunchConfiguration("cmd_udp_port"),
                        "cmd_timeout_ms": LaunchConfiguration("cmd_timeout_ms"),
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
                        "state_topic": LaunchConfiguration("robot_state_topic"),
                        "global_frame": LaunchConfiguration("global_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "enable_ros_local_io": ParameterValue(
                            PythonExpression(
                                ["'", LaunchConfiguration("use_local_io_bridge"), "' not in ['true','True','1','yes','on']"]
                            ),
                            value_type=bool,
                        ),
                        "use_tf_pose": ParameterValue(LaunchConfiguration("use_tf_pose"), value_type=bool),
                        "fallback_to_odom": ParameterValue(LaunchConfiguration("fallback_to_odom"), value_type=bool),
                        "pose_udp_bind_host": "127.0.0.1",
                        "pose_udp_port": LaunchConfiguration("pose_udp_port"),
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
                        "cmd_vel_stamped": ParameterValue(LaunchConfiguration("cmd_vel_stamped"), value_type=bool),
                        "cmd_vel_frame_id": LaunchConfiguration("cmd_vel_frame_id"),
                        "enable_ros_local_io": ParameterValue(
                            PythonExpression(
                                ["'", LaunchConfiguration("use_local_io_bridge"), "' not in ['true','True','1','yes','on']"]
                            ),
                            value_type=bool,
                        ),
                        "cmd_udp_host": "127.0.0.1",
                        "cmd_udp_port": LaunchConfiguration("cmd_udp_port"),
                        "intent_topic": LaunchConfiguration("intent_topic"),
                        "robot_states_topic": LaunchConfiguration("robot_state_topic"),
                        "expected_state_frame": LaunchConfiguration("global_frame"),
                    }
                ],
            ),
        ]
    )
