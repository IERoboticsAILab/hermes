"""Start the H.E.R.M.E.S 3D viz: bridge node + Chromium kiosk.

Usage:
    ros2 launch hermes_viz viz.launch.py
    ros2 launch hermes_viz viz.launch.py vuer_host:=0.0.0.0   # laptop fallback

Launch arguments:
    vuer_host       Host the Vuer server binds to.
                    Default: "localhost". Set to "0.0.0.0" for laptop fallback.
    vuer_port       Vuer server port. Default: 8012.
    autostart_browser  If "true", launch Chromium kiosk locally. Default: "true".
                    Set "false" when running headless or rendering from a laptop.

The bridge is read-only and publishes nothing; killing it does not disturb
the wearable stack.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    vuer_host = LaunchConfiguration("vuer_host")
    vuer_port = LaunchConfiguration("vuer_port")
    autostart_browser = LaunchConfiguration("autostart_browser")

    bridge = Node(
        package="hermes_viz",
        executable="viz_bridge_node",
        name="hermes_viz_bridge",
        output="screen",
        parameters=[{
            "vuer_host": vuer_host,
            "vuer_port": vuer_port,
        }],
    )

    # Build the Chromium URL with substituted port. We expand it lazily via
    # PythonExpression so the LaunchConfiguration value is resolved at launch time.
    chromium_url = PythonExpression(
        ["'http://localhost:' + str(", vuer_port, ")"]
    )

    chromium = ExecuteProcess(
        condition=IfCondition(autostart_browser),
        cmd=[
            "chromium-browser",
            "--kiosk",
            chromium_url,
            "--disable-features=TranslateUI",
            "--noerrdialogs",
            "--disable-infobars",
        ],
        output="log",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "vuer_host",
            default_value="localhost",
            description="Host the Vuer server binds to. Use 0.0.0.0 for laptop fallback over WiFi.",
        ),
        DeclareLaunchArgument(
            "vuer_port",
            default_value="8012",
            description="Vuer server port.",
        ),
        DeclareLaunchArgument(
            "autostart_browser",
            default_value="true",
            description="If true, launch Chromium kiosk locally. Set false when rendering remotely.",
        ),
        bridge,
        chromium,
    ])
