from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    names = [
        'serial_port', 'serial_debug_raw', 'debug_print_pose_tx',
        'write_rate_limit_enabled', 'write_min_interval_ms',
        'debug_drop_summary_every_n', 'reconnect_enabled',
        'reconnect_interval_ms', 'debug_pose_tx_summary_ms',
        'reconnect_log_every_n', 'pose_odom_topic', 'match_zone_topic',
        'match_zone_send_interval_ms', 'raw_packet_r2_topic',
        'raw_packet_legacy_topic', 'uplink_packet_r2_topic',
        'uplink_event_code_r2_topic', 'vision_weapon_pole_state_topic',
    ]
    cfg = {name: LaunchConfiguration(name) for name in names}

    defaults = {
        'serial_port': '/dev/ttyACM0',
        'serial_debug_raw': 'false',
        'debug_print_pose_tx': 'false',
        'write_rate_limit_enabled': 'false',
        'write_min_interval_ms': '10',
        'debug_drop_summary_every_n': '50',
        'reconnect_enabled': 'true',
        'reconnect_interval_ms': '1000',
        'debug_pose_tx_summary_ms': '50000',
        'reconnect_log_every_n': '10',
        'pose_odom_topic': '',
        'match_zone_topic': '/r2/match_zone',
        'match_zone_send_interval_ms': '1000',
        'raw_packet_r2_topic': '/r2/downlink/packet',
        'raw_packet_legacy_topic': '',
        'uplink_packet_r2_topic': '/r2/uplink/packet',
        'uplink_event_code_r2_topic': '/r2/uplink/event_code',
        'vision_weapon_pole_state_topic': '/vision/weapon_pole_cmd_state_2',
    }

    arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in defaults.items()
    ]
    node = Node(
        package='r2_serial',
        executable='r2_data_downlink',
        name='r2_data_downlink',
        output='screen',
        parameters=[{
            'serial_port': cfg['serial_port'],
            'serial_debug_raw': cfg['serial_debug_raw'],
            'debug.print_pose_tx': cfg['debug_print_pose_tx'],
            'write_rate_limit.enabled': cfg['write_rate_limit_enabled'],
            'write_rate_limit.min_interval_ms': cfg['write_min_interval_ms'],
            'debug.drop_summary_every_n': cfg['debug_drop_summary_every_n'],
            'reconnect.enabled': cfg['reconnect_enabled'],
            'reconnect.interval_ms': cfg['reconnect_interval_ms'],
            'debug.pose_tx_summary_ms': cfg['debug_pose_tx_summary_ms'],
            'reconnect.log_every_n': cfg['reconnect_log_every_n'],
            'topics.pose_odom': cfg['pose_odom_topic'],
            'topics.match_zone': cfg['match_zone_topic'],
            'match_zone.send_interval_ms': cfg['match_zone_send_interval_ms'],
            'topics.raw_packet_r2': cfg['raw_packet_r2_topic'],
            'topics.raw_packet_legacy': cfg['raw_packet_legacy_topic'],
            'topics.uplink_packet_r2': cfg['uplink_packet_r2_topic'],
            'topics.uplink_event_code_r2': cfg['uplink_event_code_r2_topic'],
            'topics.vision_weapon_pole_state': cfg['vision_weapon_pole_state_topic'],
        }],
    )
    return LaunchDescription(arguments + [node])
