from setuptools import find_packages, setup

package_name = "hermes_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/hermes_ros.launch.py"]),
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
        ],
    },
)
