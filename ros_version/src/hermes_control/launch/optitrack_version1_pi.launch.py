from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    server_ip_arg = DeclareLaunchArgument("serverIP", default_value="192.168.0.1")
    client_ip_arg = DeclareLaunchArgument("clientIP", default_value="192.168.0.2")
    server_type_arg = DeclareLaunchArgument("serverType", default_value="multicast")
    optitrack_config_arg = DeclareLaunchArgument(
        "optitrack_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("hermes_control"), "config", "optitrack_version1_placeholders.yaml"]
        ),
    )

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
        parameters=[LaunchConfiguration("optitrack_config")],
    )

    return LaunchDescription(
        [
            server_ip_arg,
            client_ip_arg,
            server_type_arg,
            optitrack_config_arg,
            natnet_launch,
            beacon_bridge,
        ]
    )
