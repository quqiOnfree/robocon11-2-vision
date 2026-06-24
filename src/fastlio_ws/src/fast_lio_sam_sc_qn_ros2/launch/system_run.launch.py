from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    fastlio_config_file = LaunchConfiguration('fastlio_config_file')
    backend_config = LaunchConfiguration('backend_config')
    backend_qos_reliability = LaunchConfiguration('backend_qos_reliability')
    run_simple_odom = LaunchConfiguration('run_simple_odom')
    simple_odom_topic = LaunchConfiguration('simple_odom_topic')
    downlink_packet_topic = LaunchConfiguration('downlink_packet_topic')
    uplink_packet_topic = LaunchConfiguration('uplink_packet_topic')

    default_backend_config = PathJoinSubstitution([
        FindPackageShare('fast_lio_sam_sc_qn_ros2'),
        'config',
        'fast_lio_sam_sc_qn.yaml',
    ])

    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('fast_lio'),
            'launch',
            'mapping.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz,
            'config_file': fastlio_config_file,
        }.items(),
    )

    sam_backend = Node(
        package='fast_lio_sam_sc_qn_ros2',
        executable='fast_lio_sam_sc_qn_node',
        name='fast_lio_sam_sc_qn',
        output='screen',
        parameters=[
            backend_config,
            {
                'use_sim_time': use_sim_time,
                # reliable 匹配当前 FAST-LIO 默认发布；若点云发布者是 best-effort，启动时改成 best_effort。
                'sync.qos_reliability': backend_qos_reliability,
            },
        ],
    )

    simple_odom = Node(
        package='fast_lio',
        executable='simple_odom',
        name='r2_pose_reporter',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(run_simple_odom),
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': simple_odom_topic,
            'downlink_packet_topic': downlink_packet_topic,
            'uplink_packet_topic': uplink_packet_topic,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('fastlio_config_file', default_value='mid360.yaml'),
        DeclareLaunchArgument('backend_config', default_value=default_backend_config),
        DeclareLaunchArgument('backend_qos_reliability', default_value='reliable'),
        DeclareLaunchArgument('run_simple_odom', default_value='true'),
        DeclareLaunchArgument('simple_odom_topic', default_value='/r2/global_odometry'),
        DeclareLaunchArgument('downlink_packet_topic', default_value='/r2_serial/downlink/packet'),
        DeclareLaunchArgument('uplink_packet_topic', default_value='/r2_serial/uplink/packet'),
        fastlio_launch,
        sam_backend,
        simple_odom,
    ])
