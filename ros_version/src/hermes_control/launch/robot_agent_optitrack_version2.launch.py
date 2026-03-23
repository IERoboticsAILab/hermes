from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    server_ip_arg = DeclareLaunchArgument("serverIP", default_value="192.168.0.1")
    client_ip_arg = DeclareLaunchArgument("clientIP", default_value="192.168.0.2")
    server_type_arg = DeclareLaunchArgument("serverType", default_value="multicast")
    robot_id_arg = DeclareLaunchArgument("robot_id", default_value="r1")
    rigid_body_name_arg = DeclareLaunchArgument("rigid_body_name", default_value="rosbot_1")
    optitrack_config_arg = DeclareLaunchArgument(
        "optitrack_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("hermes_control"), "config", "optitrack_version2_single_robot_placeholder.yaml"]
        ),
    )
    odom_topic_arg = DeclareLaunchArgument("odom_topic", default_value="/odom")
    cmd_vel_topic_arg = DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel")
    cmd_vel_stamped_arg = DeclareLaunchArgument("cmd_vel_stamped", default_value="false")
    cmd_vel_frame_id_arg = DeclareLaunchArgument("cmd_vel_frame_id", default_value="")
    intent_topic_arg = DeclareLaunchArgument("intent_topic", default_value="/hermes/swarm_intent")
    robot_state_topic_arg = DeclareLaunchArgument("robot_state_topic", default_value="/hermes/robot_state_beacon")
    expected_state_frame_arg = DeclareLaunchArgument("expected_state_frame", default_value="optitrack")

    natnet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("natnet_ros2"), "launch", "natnet_ros2.launch.py"])
        ),
        launch_arguments={
            "serverIP": LaunchConfiguration("serverIP"),
            "clientIP": LaunchConfiguration("clientIP"),
            "serverType": LaunchConfiguration("serverType"),
        }.items(),
    )

    beacon_bridge = Node(
        package="hermes_control",
        executable="optitrack_pose_beacon_node",
        name="optitrack_pose_beacon_node",
        output="screen",
        parameters=[
            LaunchConfiguration("optitrack_config"),
            {
                "robot_id": LaunchConfiguration("robot_id"),
                "rigid_body_name": LaunchConfiguration("rigid_body_name"),
            },
        ],
    )

    robot_agent = Node(
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
    )

    return LaunchDescription(
        [
            server_ip_arg,
            client_ip_arg,
            server_type_arg,
            robot_id_arg,
            rigid_body_name_arg,
            optitrack_config_arg,
            odom_topic_arg,
            cmd_vel_topic_arg,
            cmd_vel_stamped_arg,
            cmd_vel_frame_id_arg,
            intent_topic_arg,
            robot_state_topic_arg,
            expected_state_frame_arg,
            natnet_launch,
            beacon_bridge,
            robot_agent,
        ]
    )
