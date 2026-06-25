from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    racing_manager = Node(
        package="f1tenth_lifelong_racing",
        executable="racing_manager_node",
        name="racing_manager_node",
        output="screen",
        parameters=[{"loop_closure_topic": "/slam/loop_closure_detected"}],
    )

    return LaunchDescription([racing_manager])
