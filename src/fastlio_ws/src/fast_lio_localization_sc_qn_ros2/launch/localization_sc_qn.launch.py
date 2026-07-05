from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_directory = LaunchConfiguration('map_directory')
    qos_reliability = LaunchConfiguration('qos_reliability')
    use_position_prior = LaunchConfiguration('use_position_prior')
    expected_x_mm = LaunchConfiguration('expected_x_mm')
    expected_y_mm = LaunchConfiguration('expected_y_mm')

    default_config = PathJoinSubstitution([
        FindPackageShare('fast_lio_localization_sc_qn_ros2'),
        'config',
        'localization_sc_qn.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_directory', default_value=''),
        DeclareLaunchArgument('qos_reliability', default_value='reliable'),
        DeclareLaunchArgument('use_position_prior', default_value='false'),
        DeclareLaunchArgument('expected_x_mm', default_value='0.0'),
        DeclareLaunchArgument('expected_y_mm', default_value='0.0'),
        Node(
            package='fast_lio_localization_sc_qn_ros2',
            executable='fast_lio_localization_sc_qn_node',
            name='fast_lio_localization_sc_qn',
            output='screen',
            parameters=[
                config,
                {
                    'use_sim_time': use_sim_time,
                    'map.directory': map_directory,
                    'sync.qos_reliability': qos_reliability,
                    'cold_start.use_position_prior': use_position_prior,
                    'cold_start.expected_x_mm': expected_x_mm,
                    'cold_start.expected_y_mm': expected_y_mm,
                },
            ],
        ),
    ])
