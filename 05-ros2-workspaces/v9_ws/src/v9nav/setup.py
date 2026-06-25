from setuptools import find_packages, setup
from glob import glob
import os

package_name = "v9nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orinnano",
    maintainer_email="pbartholomy333@gmail.com",
    description="v9 SLAM mapping + pure pursuit nav",
    license="MIT",
    entry_points={
        "console_scripts": [
            "car_node          = v9nav.car_node:main",
            "corridor_node     = v9nav.corridor_node:main",
            "corridor_node_v11 = v9nav.corridor_node_v11:main",
            "joy_node          = v9nav.joy_node:main",
            "mode_manager_node = v9nav.mode_manager_node:main",
            "mapping_node      = v9nav.mapping_node:main",
            "pure_pursuit_node = v9nav.pure_pursuit_node:main",
            "visualizer_node   = v9nav.visualizer_node:main",
            "zed_obstacle_node = v9nav.zed_obstacle_node:main",
        ],
    },
)
