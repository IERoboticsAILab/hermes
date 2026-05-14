from setuptools import find_packages, setup

package_name = "hermes_viz"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/viz.launch.py"]),
    ],
    install_requires=["setuptools", "vuer", "numpy"],
    zip_safe=True,
    maintainer="Saleh Abd-Elrahman",
    maintainer_email="saleh@deanna.pro",
    description="Browser-based 3D digital twin for H.E.R.M.E.S",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "viz_bridge_node = hermes_viz.viz_bridge_node:main",
        ],
    },
)
