"""ROS2 通信层：Ros2Node 及 PathSignalEmitter。"""

import json

from PySide6.QtCore import Signal, Slot, QObject

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, UInt16, Bool, Float64
from nav_msgs.msg import Odometry
from r2_serial.msg import CurrentPose, InitialPosition, StartupConfig


class PathSignalEmitter(QObject):
    path_signal = Signal(list)
    odom_signal = Signal(int, int, int, int)            # x_mm, y_mm, z_mm, yaw_deg
    localized_signal = Signal(bool)                    # localized
    fitness_signal = Signal(float)                     # fitness score
    connection_signal = Signal(bool)                   # downlink connected
    mcu_event_signal = Signal(int)                     # event_code
    debug_msg_signal = Signal(str)                     # MCU debug message
    initial_position_signal = Signal(int, int)          # x_mm, y_mm (启动坐标)


class Ros2Node(Node):
    def __init__(self):
        Node.__init__(self, "hand_input_path_node")
        self.path_signal = PathSignalEmitter()

        # Publisher
        self.grid_publisher = self.create_publisher(String, "grid_data", 10)
        self.startup_config_pub = self.create_publisher(
            StartupConfig, "/r2_serial/downlink/startup_config", 10)

        # Subscriber
        self.subscriber = self.create_subscription(
            String, "path_commands", self.path_received, 10)

        # Subscriber (新增：位姿状态)
        odom_topic = self.declare_parameter(
            "odom_topic", "/Odometry").value
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        current_pose_topic = self.declare_parameter(
            "current_pose_topic", "/r2/current_pose_mm").value
        self.current_pose_sub = self.create_subscription(
            CurrentPose, current_pose_topic, self.current_pose_callback, 20)
        self.localized_sub = self.create_subscription(
            Bool, "/r2/localized", self.localized_callback, 10)
        self.fitness_sub = self.create_subscription(
            Float64, "/r2/fitness_score", self.fitness_callback, 10)
        self.uplink_event_sub = self.create_subscription(
            UInt16, "/r2_serial/uplink/event_code", self.uplink_event_callback, 10)
        self.debug_msg_sub = self.create_subscription(
            String, "/r2_serial/uplink/debug_msg", self.debug_msg_callback, 10)

        # 启动坐标（transient_local，仅发布一次）
        initial_position_topic = self.declare_parameter(
            "initial_position_topic", "/r2/initial_position").value
        initial_position_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.initial_position_sub = self.create_subscription(
            InitialPosition,
            initial_position_topic,
            self.initial_position_callback,
            initial_position_qos,
        )

        # 状态缓存（避免重复 emit 相同值）
        self._last_localized = None
        self._last_fitness = None
        self._last_connection = None

        # 起点坐标缓存（来自 /r2/initial_position，启动时发布一次）
        self._initial_x = None
        self._initial_y = None

        # 实时位姿缓存（供 lidar panel 打开时回放）
        self._last_odom_x = 0
        self._last_odom_y = 0
        self._last_odom_z = 0
        self._last_odom_yaw = 0

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

    def publish_startup_config(self, area_type: int, begin_type: int,
                                kfs_amount: int = 0) -> bool:
        if self._initial_x is None or self._initial_y is None:
            self.get_logger().warn("尚未收到起点坐标，无法发送启动配置")
            return False
        msg = StartupConfig()
        msg.area_type = area_type
        msg.begin_type = begin_type
        msg.origin_x = self._initial_x
        msg.origin_y = self._initial_y
        msg.kfs_amount = kfs_amount
        self.startup_config_pub.publish(msg)
        self.get_logger().info(
            f"已发送合并启动配置: area={area_type} begin={begin_type} "
            f"origin=({self._initial_x}, {self._initial_y}) "
            f"kfs_amount={kfs_amount}")
        return True

    def clear_lidar_cache(self):
        """关闭雷达时清空所有暂存数据，避免残留到下次启动。"""
        self._initial_x = None
        self._initial_y = None
        self._last_odom_x = 0
        self._last_odom_y = 0
        self._last_odom_z = 0
        self._last_odom_yaw = 0
        self._last_localized = None
        self._last_fitness = None
        self._last_connection = None

    @property
    def has_initial_position(self) -> bool:
        return self._initial_x is not None

    def path_received(self, msg: String):
        try:
            path_data = json.loads(msg.data)
            self.path_signal.path_signal.emit(path_data['path'])
            print("Received path command:", path_data)
        except json.JSONDecodeError:
            print("Failed to decode path command:", msg.data)

    def odom_callback(self, msg: Odometry):
        """FAST-LIO 原始里程计仅用于补充 UI 的 Z 高度。"""
        self._last_odom_z = int(round(msg.pose.pose.position.z * 1000.0))

    def current_pose_callback(self, msg: CurrentPose):
        """显示 r2_pose_reporter 准备下发给 MCU 的 int16 位姿。"""
        self._last_odom_x = int(msg.x_mm)
        self._last_odom_y = int(msg.y_mm)
        self._last_odom_yaw = int(msg.yaw_deg)
        self.path_signal.odom_signal.emit(
            self._last_odom_x, self._last_odom_y,
            self._last_odom_z, self._last_odom_yaw)

    def initial_position_callback(self, msg: InitialPosition):
        self._initial_x = int(msg.x_mm)
        self._initial_y = int(msg.y_mm)
        self.path_signal.initial_position_signal.emit(self._initial_x, self._initial_y)

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

    def debug_msg_callback(self, msg: String):
        self.path_signal.debug_msg_signal.emit(msg.data)

    def check_connection(self):
        """检查下发节点是否在线（由 QTimer 周期调用）。"""
        count = self.startup_config_pub.get_subscription_count()
        connected = count > 0
        if self._last_connection != connected:
            self._last_connection = connected
            self.path_signal.connection_signal.emit(connected)

