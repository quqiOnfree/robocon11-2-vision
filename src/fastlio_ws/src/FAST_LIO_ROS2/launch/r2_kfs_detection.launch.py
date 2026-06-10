import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("fast_lio")
    config_dir = os.path.join(package_share, "config")

    detector_config = LaunchConfiguration("detector_config")
    map_path = LaunchConfiguration("map_path")

    detector = Node(
        package="fast_lio",
        executable="r2_kfs_detector",
        output="screen",
        parameters=[
            detector_config,
            {"map_path": map_path},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detector_config",
                default_value=os.path.join(config_dir, "r2_kfs_detector.yaml"),
            ),
            DeclareLaunchArgument(
                "map_path",
                default_value="/root/fastlio_ws/MapPCD/R2_Field_Map_v1.pcd",
            ),
            detector,
        ]
    )
