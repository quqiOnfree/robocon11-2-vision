from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_debug_raw = LaunchConfiguration('serial_debug_raw')
    debug_print_pose_tx = LaunchConfiguration('debug_print_pose_tx')
    write_rate_limit_enabled = LaunchConfiguration('write_rate_limit_enabled')
    write_min_interval_ms = LaunchConfiguration('write_min_interval_ms')
    reconnect_enabled = LaunchConfiguration('reconnect_enabled')
    reconnect_interval_ms = LaunchConfiguration('reconnect_interval_ms')
    debug_pose_tx_summary_ms = LaunchConfiguration('debug_pose_tx_summary_ms')
    pose_odom_topic = LaunchConfiguration('pose_odom_topic')
    raw_packet_r2_topic = LaunchConfiguration('raw_packet_r2_topic')
    raw_packet_legacy_topic = LaunchConfiguration('raw_packet_legacy_topic')
    uplink_packet_r2_topic = LaunchConfiguration('uplink_packet_r2_topic')
    uplink_event_code_r2_topic = LaunchConfiguration('uplink_event_code_r2_topic')
    vision_weapon_pole_state_topic = LaunchConfiguration('vision_weapon_pole_state_topic')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('serial_debug_raw', default_value='false'),
        DeclareLaunchArgument('debug_print_pose_tx', default_value='false'),
        DeclareLaunchArgument('write_rate_limit_enabled', default_value='true'),
        DeclareLaunchArgument('write_min_interval_ms', default_value='10'),
        DeclareLaunchArgument('reconnect_enabled', default_value='true'),
        DeclareLaunchArgument('reconnect_interval_ms', default_value='1000'),
        DeclareLaunchArgument('debug_pose_tx_summary_ms', default_value='5000'),
        DeclareLaunchArgument('pose_odom_topic', default_value=''),
        DeclareLaunchArgument('raw_packet_r2_topic', default_value='/r2/downlink/packet'),
        DeclareLaunchArgument('raw_packet_legacy_topic', default_value=''),
        DeclareLaunchArgument('uplink_packet_r2_topic', default_value='/r2/uplink/packet'),
        DeclareLaunchArgument('uplink_event_code_r2_topic', default_value='/r2/uplink/event_code'),
        DeclareLaunchArgument('vision_weapon_pole_state_topic', default_value='/vision/weapon_pole_cmd_state_2'),
        Node(
            package='r2_serial',
            executable='r2_data_downlink',
            name='r2_data_downlink',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'serial_debug_raw': serial_debug_raw,
                'debug.print_pose_tx': debug_print_pose_tx,
                'write_rate_limit.enabled': write_rate_limit_enabled,
                'write_rate_limit.min_interval_ms': write_min_interval_ms,
                'reconnect.enabled': reconnect_enabled,
                'reconnect.interval_ms': reconnect_interval_ms,
                'debug.pose_tx_summary_ms': debug_pose_tx_summary_ms,
                'topics.pose_odom': pose_odom_topic,
                'topics.raw_packet_r2': raw_packet_r2_topic,
                'topics.raw_packet_legacy': raw_packet_legacy_topic,
                'topics.uplink_packet_r2': uplink_packet_r2_topic,
                'topics.uplink_event_code_r2': uplink_event_code_r2_topic,
                'topics.vision_weapon_pole_state': vision_weapon_pole_state_topic,
            }],
        ),
    ])
