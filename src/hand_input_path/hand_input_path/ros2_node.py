"""ROS2 通信层：Ros2Node 及 PathSignalEmitter。"""

import json

from PySide6.QtCore import Signal, Slot, QObject

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Empty, UInt16
import r2_serial.msg._serial_packet as serial_packet


class PathSignalEmitter(QObject):
    path_signal = Signal(list)
    scene_signal = Signal(int)
    lidar_position_signal = Signal(int, int, int)


class Ros2Node(Node):
    def __init__(self):
        Node.__init__(self, "hand_input_path_node")
        self.path_signal = PathSignalEmitter()

        # Publisher
        self.grid_publisher = self.create_publisher(String, "grid_data", 10)
        self.set_start_zone_pub = self.create_publisher(
            UInt16, "/r2_serial/downlink/set_start_zone", 10)
        self.start_command_pub = self.create_publisher(
            Empty, "/r2_serial/downlink/start_command", 10)

        # Subscriber
        self.scene_subcription = self.create_subscription(
            Int8, "/r2/match_zone", self.scene_received, 10)
        self.subscriber = self.create_subscription(
            String, "path_commands", self.path_received, 10)
        self.serial_subscriber = self.create_subscription(
            serial_packet.SerialPacket,
            "/r2_serial/downlink/packet", self.serial_received, 10)

    @Slot(dict)
    def publish_grid(self, grid: dict):
        grid_data = dict()
        grid_data["grid"] = [[block_type.value for block_type in row]
                             for row in grid["grid"]]
        grid_data["level"] = [[block_level.value for block_level in row]
                              for row in grid["level"]]
        json_data = json.dumps(grid_data)
        msg = String()
        msg.data = json_data
        self.grid_publisher.publish(msg)
        print("Published grid data:", json_data)

    def publish_set_zone(self, zone: int):
        """发布设置启动区域命令，zone: 0=1区, 1=2区, 2=3区重试区"""
        msg = UInt16()
        msg.data = zone
        self.set_start_zone_pub.publish(msg)
        self.get_logger().info(f"已发送设置启动区域: zone={zone}")

    def publish_start_command(self):
        """发布开始命令"""
        msg = Empty()
        self.start_command_pub.publish(msg)
        self.get_logger().info("已发送开始命令")

    def path_received(self, msg: String):
        try:
            path_data = json.loads(msg.data)
            self.path_signal.path_signal.emit(path_data['path'])
            print("Received path command:", path_data)
        except json.JSONDecodeError:
            print("Failed to decode path command:", msg.data)

    def scene_received(self, msg: Int8):
        code = int(msg.data)
        if code not in (0, 1):
            self.get_logger().warn(f"Unknown match zone: {code}")
            return
        self.path_signal.scene_signal.emit(code)

    def serial_received(self, msg: serial_packet.SerialPacket):
        if msg.code != 0x0101:
            return
        if len(msg.payload) != 6:
            self.get_logger().warn("error format of serial packet")
            return
        x_mm = (msg.payload[0] | (msg.payload[1] << 8)) - 32768
        y_mm = (msg.payload[2] | (msg.payload[3] << 8)) - 32768
        yaw_deg = (msg.payload[4] | (msg.payload[5] << 8)) - 32768
        self.path_signal.lidar_position_signal.emit(x_mm, y_mm, yaw_deg)
