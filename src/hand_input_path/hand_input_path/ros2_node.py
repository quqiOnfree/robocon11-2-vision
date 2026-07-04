"""ROS2 通信层：Ros2Node 及 PathSignalEmitter。"""

import json
import math

from PySide6.QtCore import Signal, Slot, QObject

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Empty, UInt16, Bool, Float64
from nav_msgs.msg import Odometry


class PathSignalEmitter(QObject):
    path_signal = Signal(list)
    odom_signal = Signal(int, int, int, int)            # x_mm, y_mm, z_mm, yaw_deg
    localized_signal = Signal(bool)                    # localized
    fitness_signal = Signal(float)                     # fitness score
    connection_signal = Signal(bool)                   # downlink connected
    mcu_event_signal = Signal(int)                     # event_code


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
        self.match_zone_pub = self.create_publisher(
            Int8, "/hand_input/match_zone", 10)

        # Subscriber
        self.subscriber = self.create_subscription(
            String, "path_commands", self.path_received, 10)

        # Subscriber (新增：位姿状态)
        odom_topic = self.declare_parameter(
            "odom_topic", "/r2/global_odometry").value
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        self.localized_sub = self.create_subscription(
            Bool, "/r2/localized", self.localized_callback, 10)
        self.fitness_sub = self.create_subscription(
            Float64, "/r2/fitness_score", self.fitness_callback, 10)
        self.uplink_event_sub = self.create_subscription(
            UInt16, "/r2_serial/uplink/event_code", self.uplink_event_callback, 10)

        # 状态缓存（避免重复 emit 相同值）
        self._last_localized = None
        self._last_fitness = None
        self._last_connection = None

    # ── 已有方法 ──

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
        msg = UInt16()
        msg.data = zone
        self.set_start_zone_pub.publish(msg)
        self.get_logger().info(f"已发送设置启动区域: zone={zone}")

    def publish_start_command(self):
        msg = Empty()
        self.start_command_pub.publish(msg)
        self.get_logger().info("已发送开始命令")

    def publish_match_zone(self, zone: int):
        msg = Int8()
        msg.data = zone
        self.match_zone_pub.publish(msg)
        self.get_logger().info(f"已发布半场设置: zone={zone}")

    def path_received(self, msg: String):
        try:
            path_data = json.loads(msg.data)
            self.path_signal.path_signal.emit(path_data['path'])
            print("Received path command:", path_data)
        except json.JSONDecodeError:
            print("Failed to decode path command:", msg.data)

    def odom_callback(self, msg: Odometry):
        """提取 X/Y/Z (mm) 和 Yaw (deg)."""
        x_mm = int(round(msg.pose.pose.position.x * 1000.0))
        y_mm = int(round(msg.pose.pose.position.y * 1000.0))
        z_mm = int(round(msg.pose.pose.position.z * 1000.0))

        # 四元数 → yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = int(round(math.atan2(siny, cosy) * 180.0 / math.pi))

        self.path_signal.odom_signal.emit(x_mm, y_mm, z_mm, yaw_deg)

    def localized_callback(self, msg: Bool):
        if self._last_localized != msg.data:
            self._last_localized = msg.data
            self.path_signal.localized_signal.emit(msg.data)

    def fitness_callback(self, msg: Float64):
        if self._last_fitness != msg.data:
            self._last_fitness = msg.data
            self.path_signal.fitness_signal.emit(msg.data)

    def uplink_event_callback(self, msg: UInt16):
        self.path_signal.mcu_event_signal.emit(int(msg.data))

    def check_connection(self):
        """检查下发节点是否在线（由 QTimer 周期调用）。"""
        count = self.set_start_zone_pub.get_subscription_count()
        connected = count > 0
        if self._last_connection != connected:
            self._last_connection = connected
            self.path_signal.connection_signal.emit(connected)

