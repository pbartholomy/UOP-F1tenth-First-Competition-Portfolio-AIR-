"""
race.launch.py — pure pursuit with reactive fallback.

Usage:
  bash run_race.sh ~/f1tenth_waypoints/waypoints_<timestamp>.csv

The car starts in pure_pursuit mode and follows the recorded path.
Press R1 to toggle back to reactive if needed.
L1 is still the kill switch.

IMPORTANT: place the car at the same physical starting position used
during the mapping run — odometry resets to (0,0,0) on each L1 release.
"""

import sys
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wp_arg = DeclareLaunchArgument(
        "waypoints_file",
        default_value=os.path.expanduser("~/f1tenth_waypoints/latest.csv"),
        description="Path to waypoints CSV recorded during mapping run",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )

    corridor_node = Node(
        package="corridor",
        executable="corridor_node",
        name="corridor_node",
    )

    mode_manager = Node(
        package="corridor",
        executable="mode_manager_node",
        name="mode_manager_node",
        parameters=[{"mode": "pure_pursuit"}],
    )

    pure_pursuit = Node(
        package="corridor",
        executable="pure_pursuit_node",
        name="pure_pursuit_node",
        parameters=[{
            "waypoints_file": LaunchConfiguration("waypoints_file"),
            "speed_scale":    1.0,
            "max_speed":      3.5,
            "min_speed":      0.5,
            "k_dd":           0.5,
            "lookahead_min":  0.4,
            "lookahead_max":  2.0,
        }],
    )

    return LaunchDescription([
        wp_arg,
        joy_node,
        corridor_node,
        mode_manager,
        pure_pursuit,
    ])
