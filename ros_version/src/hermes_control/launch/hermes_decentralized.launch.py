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
    global_frame_arg = DeclareLaunchArgument(
        "global_frame",
        default_value="map",
        description="Shared global frame for decentralized robot beacons.",
    )
    base_frame_arg = DeclareLaunchArgument(
        "base_frame",
        default_value="base_link",
        description="Robot base frame for TF lookup.",
    )
    use_tf_pose_arg = DeclareLaunchArgument(
        "use_tf_pose",
        default_value="true",
        description="Publish beacon from TF global_frame->base_frame transform.",
    )
    fallback_to_odom_arg = DeclareLaunchArgument(
        "fallback_to_odom",
        default_value="false",
        description="Fallback to raw odometry if TF is unavailable.",
    )

    return LaunchDescription(
        [
            robot_id_arg,
            odom_topic_arg,
            cmd_vel_topic_arg,
            cmd_vel_stamped_arg,
            cmd_vel_frame_id_arg,
            global_frame_arg,
            base_frame_arg,
            use_tf_pose_arg,
            fallback_to_odom_arg,
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
                        "cmd_vel_stamped": LaunchConfiguration("cmd_vel_stamped"),
                        "cmd_vel_frame_id": LaunchConfiguration("cmd_vel_frame_id"),
                        "expected_state_frame": LaunchConfiguration("global_frame"),
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
                        "global_frame": LaunchConfiguration("global_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "use_tf_pose": LaunchConfiguration("use_tf_pose"),
                        "fallback_to_odom": LaunchConfiguration("fallback_to_odom"),
                    }
                ],
            ),
        ]
    )
