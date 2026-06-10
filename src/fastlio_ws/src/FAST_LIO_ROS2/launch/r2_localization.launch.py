import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("fast_lio")
    config_dir = os.path.join(package_share, "config")
    rviz_config = os.path.join(package_share, "rviz", "r2_localization.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    mapping_config = LaunchConfiguration("mapping_config")
    localizer_config = LaunchConfiguration("localizer_config")
    map_path = LaunchConfiguration("map_path")
    metrics_csv_path = LaunchConfiguration("metrics_csv_path")
    use_rviz = LaunchConfiguration("rviz")

    mapping = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        output="screen",
        parameters=[mapping_config, {"use_sim_time": use_sim_time}],
    )
    localizer = Node(
        package="fast_lio",
        executable="r2_localizer",
        output="screen",
        parameters=[
            localizer_config,
            {
                "map_path": map_path,
                "metrics_csv_path": metrics_csv_path,
                "use_sim_time": use_sim_time,
            },
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("mapping_config", default_value=os.path.join(config_dir, "mid360_localization.yaml")),
            DeclareLaunchArgument("localizer_config", default_value=os.path.join(config_dir, "r2_localizer.yaml")),
            DeclareLaunchArgument("map_path", default_value="/root/fastlio_ws/MapPCD/R2_Field_Map_v3.pcd"),
            DeclareLaunchArgument("metrics_csv_path", default_value="/tmp/r2_localizer_metrics.csv"),
            DeclareLaunchArgument("rviz", default_value="false"),
            mapping,
            localizer,
            rviz,
        ]
    )
