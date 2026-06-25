from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_save_directory = LaunchConfiguration("map_save_directory")

    default_config = PathJoinSubstitution([
        FindPackageShare("fast_lio_sam_sc_qn_ros2"),
        "config",
        "fast_lio_sam_sc_qn.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_save_directory", default_value="/tmp/r2_sam_keyframe_map"),
        Node(
            package="fast_lio_sam_sc_qn_ros2",
            executable="fast_lio_sam_sc_qn_node",
            name="fast_lio_sam_sc_qn",
            output="screen",
            parameters=[config, {
                "use_sim_time": use_sim_time,
                "map_save.directory": map_save_directory,
            }],
        ),
    ])
