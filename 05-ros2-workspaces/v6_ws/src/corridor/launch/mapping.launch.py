"""
mapping.launch.py — drive manually to record waypoints.

Steps:
  1. bash run_mapping.sh
  2. Release L1 to start driving (reactive mode)
  3. Press Triangle to begin recording waypoints
  4. Drive one full lap at a consistent pace
  5. Press Triangle again to stop and save the CSV
  6. Ctrl+C to exit

Waypoints saved to ~/f1tenth_waypoints/waypoints_<timestamp>.csv
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
        parameters=[{"mode": "reactive"}],
    )

    waypoint_logger = Node(
        package="corridor",
        executable="waypoint_logger_node",
        name="waypoint_logger_node",
    )

    return LaunchDescription([
        joy_node,
        corridor_node,
        mode_manager,
        waypoint_logger,
    ])
