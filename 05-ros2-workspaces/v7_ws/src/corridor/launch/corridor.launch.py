from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def generate_launch_description():
    wp_arg = DeclareLaunchArgument(
        "waypoints_file", default_value="",
        description="Path to waypoints CSV. Empty = reactive-only autonomous.")

    use_zed_arg = DeclareLaunchArgument(
        "use_zed", default_value="true",
        description="Launch ZED 2i camera and obstacle node.")

    joy_node = Node(
        package="corridor",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    car_node = Node(
        package="corridor",
        executable="car_node",
        name="car_node",
        output="screen",
    )

    corridor_node = Node(
        package="corridor",
        executable="corridor_node",
        name="corridor_node",
        output="screen",
    )

    mode_manager = Node(
        package="corridor",
        executable="mode_manager_node",
        name="mode_manager_node",
        parameters=[{"initial_mode": "manual"}],
        output="screen",
    )

    pure_pursuit = Node(
        package="corridor",
        executable="pure_pursuit_node",
        name="pure_pursuit_node",
        parameters=[{
            "waypoints_file": LaunchConfiguration("waypoints_file"),
            "use_zed_odom":   True,
        }],
        output="screen",
    )

    waypoint_logger = Node(
        package="corridor",
        executable="waypoint_logger_node",
        name="waypoint_logger_node",
        output="screen",
    )

    # ZED gap detection + recording — always launch (uses pyzed directly)
    zed_gap_node = Node(
        package="corridor",
        executable="zed_obstacle_node",
        name="zed_obstacle_node",
        output="screen",
        parameters=[{"fps": 30.0, "width": 672, "height": 376}],
    )

    actions = [
        wp_arg,
        use_zed_arg,
        joy_node,
        car_node,
        corridor_node,
        mode_manager,
        pure_pursuit,
        waypoint_logger,
        zed_gap_node,
    ]

    return LaunchDescription(actions)
