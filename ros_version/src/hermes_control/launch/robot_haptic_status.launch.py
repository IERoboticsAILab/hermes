from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_arg = DeclareLaunchArgument(
        "status_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("hermes_control"), "config", "robot_haptic_status_defaults.yaml"]
        ),
    )
    robot_id_arg = DeclareLaunchArgument("robot_id", default_value="r1")

    node = Node(
        package="hermes_control",
        executable="robot_haptic_status_node",
        name="robot_haptic_status_node",
        output="screen",
        parameters=[
            LaunchConfiguration("status_config"),
            {
                "robot_id": LaunchConfiguration("robot_id"),
            },
        ],
    )

    return LaunchDescription([config_arg, robot_id_arg, node])
