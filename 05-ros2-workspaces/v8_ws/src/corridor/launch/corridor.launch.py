from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    viz_arg = DeclareLaunchArgument(
        "viz", default_value="true",
        description="Launch real-time LiDAR + ZED visualizer windows")

    return LaunchDescription([
        viz_arg,
        Node(package="corridor", executable="joy_node",          name="joy_node",          output="screen"),
        Node(package="corridor", executable="car_node",          name="car_node",          output="screen"),
        Node(package="corridor", executable="corridor_node",     name="corridor_node",     output="screen"),
        Node(package="corridor", executable="mode_manager_node", name="mode_manager_node", output="screen",
             parameters=[{"initial_mode": "manual"}]),
        Node(package="corridor", executable="visualizer_node",   name="visualizer_node",   output="screen",
             condition=IfCondition(LaunchConfiguration("viz"))),
        Node(package="corridor", executable="zed_obstacle_node", name="zed_obstacle_node", output="screen",
             condition=IfCondition(LaunchConfiguration("viz"))),
    ])
