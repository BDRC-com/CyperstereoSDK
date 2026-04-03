from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("cyperstereo_ros2_bridge"), "config", "capture_image_imu.yaml"]
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the parameter YAML file for cyperstereo_ros2_bridge",
            ),
            Node(
                package="cyperstereo_ros2_bridge",
                executable="cyperstereo_ros2_bridge",
                name="cyperstereo_ros2_bridge",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )