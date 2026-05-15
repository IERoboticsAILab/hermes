import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hermes_viz"


def _asset_data_files():
    """Recursively collect all files under hermes_viz/assets/ for installation."""
    result = []
    assets_root = os.path.join("hermes_viz", "assets")
    for dirpath, _dirnames, filenames in os.walk(assets_root):
        if not filenames:
            continue
        install_dir = os.path.join("share", package_name, dirpath)
        result.append((install_dir, [os.path.join(dirpath, f) for f in filenames]))
    return result


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/viz.launch.py"]),
        *_asset_data_files(),
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
