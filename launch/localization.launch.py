from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_path = PathJoinSubstitution([
        FindPackageShare("glim_localization"),
        "config",
    ])

    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_config_path,
        description="Directory containing config.json and localization.json.",
    )

    glim_node = Node(
        package="glim_ros",
        executable="glim_rosnode",
        name="glim_rosnode",
        output="screen",
        parameters=[{"config_path": LaunchConfiguration("config_path")}],
    )

    return LaunchDescription([
        config_path_arg,
        glim_node,
    ])
