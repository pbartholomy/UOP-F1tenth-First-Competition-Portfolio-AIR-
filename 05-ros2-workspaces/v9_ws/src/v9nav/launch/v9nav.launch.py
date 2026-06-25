from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    viz_arg = DeclareLaunchArgument(
        "viz",
        default_value="true",
        description="Launch visualizer and ZED obstacle node (true/false)",
    )
    viz = LaunchConfiguration("viz")

    return LaunchDescription([
        viz_arg,

        # ── Core nodes ─────────────────────────────────────────────
        Node(
            package="v9nav",
            executable="joy_node",
            name="joy_node",
            output="screen",
        ),

        Node(
            package="v9nav",
            executable="car_node",
            name="car_node",
            output="screen",
        ),

        Node(
            package="v9nav",
            executable="corridor_node",
            name="corridor_node",
            output="screen",
        ),

        Node(
            package="v9nav",
            executable="mode_manager_node",
            name="mode_manager_node",
            output="screen",
            parameters=[{"initial_mode": "manual"}],
        ),

        Node(
            package="v9nav",
            executable="mapping_node",
            name="mapping_node",
            output="screen",
        ),

        Node(
            package="v9nav",
            executable="pure_pursuit_node",
            name="pure_pursuit_node",
            output="screen",
        ),

        # ── Optional viz nodes ─────────────────────────────────────
        Node(
            package="v9nav",
            executable="visualizer_node",
            name="visualizer_node",
            output="screen",
            condition=IfCondition(viz),
        ),

        Node(
            package="v9nav",
            executable="zed_obstacle_node",
            name="zed_obstacle_node",
            output="screen",
            condition=IfCondition(viz),
        ),

        # ── Static TF transforms for SLAM ─────────────────────────
        # odom → base_link  (identity; SLAM / odometry fills this in practice,
        #                    but a static identity prevents TF tree gaps on startup)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_odom_base",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            output="screen",
        ),

        # base_link → laser  (Hokuyo mounted ~5 cm above base)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_laser",
            arguments=["0", "0", "0.05", "0", "0", "0", "base_link", "laser"],
            output="screen",
        ),

        # base_link → base_footprint  (identity; required by slam_toolbox)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_footprint",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
            output="screen",
        ),
    ])
