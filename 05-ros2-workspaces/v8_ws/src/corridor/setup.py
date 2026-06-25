from setuptools import find_packages, setup
from glob import glob
import os

package_name = "corridor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orinnano",
    maintainer_email="pbartholomy333@gmail.com",
    description="v8 corridor-only reactive nav",
    license="MIT",
    entry_points={
        "console_scripts": [
            "car_node          = corridor.car_node:main",
            "corridor_node      = corridor.corridor_node:main",
            "corridor_node_v11  = corridor.corridor_node_v11:main",
            "corridor_node_comp = corridor.corridor_node_comp:main",
            "joy_node           = corridor.joy_node:main",
            "mode_manager_node  = corridor.mode_manager_node:main",
            "visualizer_node    = corridor.visualizer_node:main",
            "zed_obstacle_node      = corridor.zed_obstacle_node:main",
            "zed_obstacle_node_comp = corridor.zed_obstacle_node_comp:main",
        ],
    },
)
