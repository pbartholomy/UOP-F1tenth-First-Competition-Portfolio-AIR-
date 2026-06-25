from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package="roboracer",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    corridor_node = Node(
        package="corridor",
        executable="corridor_node",
        name="corridor_node",
        output="screen",
    )

    return LaunchDescription([joy_node, corridor_node])
