import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _config_params(config_path: str) -> dict:
    with open(config_path, "r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}
    return dict(payload.get("optitrack_pose_beacon_node", {}).get("ros__parameters", {}))


def _pick(override: str, fallback: object, default: str) -> str:
    value = "" if override is None else str(override).strip()
    if value:
        return value
    value = "" if fallback is None else str(fallback).strip()
    if value:
        return value
    return default


def _launch_setup(context, *_args, **_kwargs):
    config_path = LaunchConfiguration("optitrack_config").perform(context)
    config_params = _config_params(config_path)

    frame_id = _pick("", config_params.get("frame_id"), "optitrack")
    server_ip = _pick(LaunchConfiguration("serverIP").perform(context), config_params.get("server_ip"), "")
    client_ip = _pick(LaunchConfiguration("clientIP").perform(context), config_params.get("client_ip"), "")
    server_type = _pick(LaunchConfiguration("serverType").perform(context), config_params.get("server_type"), "multicast")
    robot_id = _pick(LaunchConfiguration("robot_id").perform(context), config_params.get("robot_id"), "r1")
    rigid_body_name = _pick(
        LaunchConfiguration("rigid_body_name").perform(context), config_params.get("rigid_body_name"), "rosbot_1"
    )
    expected_state_frame = _pick(
        LaunchConfiguration("expected_state_frame").perform(context), config_params.get("frame_id"), frame_id
    )

    natnet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("natnet_ros2"), "launch", "natnet_ros2.launch.py"])
        ),
        launch_arguments={
            "serverIP": server_ip,
            "clientIP": client_ip,
            "serverType": server_type,
        }.items(),
    )

    beacon_bridge = Node(
        package="hermes_control",
        executable="optitrack_pose_beacon_node",
        name="optitrack_pose_beacon_node",
        output="screen",
        parameters=[
            config_path,
            {
                "frame_id": frame_id,
                "robot_id": robot_id,
                "rigid_body_name": rigid_body_name,
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
                "robot_id": robot_id,
                "odom_topic": LaunchConfiguration("odom_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "cmd_vel_stamped": ParameterValue(LaunchConfiguration("cmd_vel_stamped"), value_type=bool),
                "cmd_vel_frame_id": LaunchConfiguration("cmd_vel_frame_id"),
                "use_beacon_pose_for_self": True,
                "intent_topic": LaunchConfiguration("intent_topic"),
                "robot_states_topic": LaunchConfiguration("robot_state_topic"),
                "expected_state_frame": expected_state_frame,
            }
        ],
    )

    return [natnet_launch, beacon_bridge, robot_agent]


def generate_launch_description() -> LaunchDescription:
    server_ip_arg = DeclareLaunchArgument("serverIP", default_value="")
    client_ip_arg = DeclareLaunchArgument("clientIP", default_value="")
    server_type_arg = DeclareLaunchArgument("serverType", default_value="")
    robot_id_arg = DeclareLaunchArgument("robot_id", default_value="")
    rigid_body_name_arg = DeclareLaunchArgument("rigid_body_name", default_value="")
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
    expected_state_frame_arg = DeclareLaunchArgument("expected_state_frame", default_value="")

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
            OpaqueFunction(function=_launch_setup),
        ]
    )
