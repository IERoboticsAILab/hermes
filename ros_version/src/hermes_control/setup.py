from setuptools import find_packages, setup

package_name = "hermes_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/hermes_ros.launch.py",
                "launch/hermes_decentralized.launch.py",
                "launch/hermes_keyboard_teleop.launch.py",
                "launch/robot_agent.launch.py",
                "launch/optitrack_version1_pi.launch.py",
                "launch/robot_agent_optitrack_version1.launch.py",
                "launch/robot_agent_optitrack_version2.launch.py",
                "launch/haptic_vest.launch.py",
                "launch/robot_haptic_status.launch.py",
            ],
        ),
        (
            f"share/{package_name}/config",
            [
                "config/optitrack_version1_placeholders.yaml",
                "config/optitrack_version2_single_robot_placeholder.yaml",
                "config/optitrack_r1.yaml",
                "config/optitrack_r2.yaml",
                "config/optitrack_r3.yaml",
                "config/optitrack_r4.yaml",
                "config/optitrack_r5.yaml",
                "config/optitrack_r6.yaml",
                "config/haptic_vest_pi.yaml",
                "config/robot_haptic_status_defaults.yaml",
                "config/robot_haptic_status_r1.yaml",
                "config/robot_haptic_status_r2.yaml",
                "config/robot_haptic_status_r3.yaml",
                "config/robot_haptic_status_r4.yaml",
                "config/robot_haptic_status_r5.yaml",
                "config/robot_haptic_status_r6.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="salehabdelrahman",
    maintainer_email="saleh@example.com",
    description="ROS2 packaging of H.E.R.M.E.S gesture recognition and swarm control.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gesture_pipeline_node = hermes_control.gesture_pipeline_node:main",
            "swarm_control_node = hermes_control.swarm_control_node:main",
            "keyboard_teleop_node = hermes_control.keyboard_teleop_node:main",
            "decentralized_robot_agent_node = hermes_control.decentralized_robot_agent_node:main",
            "robot_state_beacon_node = hermes_control.robot_state_beacon_node:main",
            "optitrack_pose_beacon_node = hermes_control.optitrack_pose_beacon_node:main",
            "robot_haptic_status_node = hermes_control.robot_haptic_status_node:main",
            "haptic_vest_node = hermes_control.haptic_vest_node:main",
        ],
    },
)
