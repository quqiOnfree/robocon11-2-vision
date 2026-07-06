"""Lidar 状态面板：独立弹窗显示 R2 位姿、定位、串口状态。"""

from PySide6.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QLabel,
    QPushButton,
    QGroupBox,
    QHBoxLayout,
)
from PySide6.QtGui import QFont
from PySide6.QtCore import Qt

# MCU 事件码 → 中文描述
EVENT_MAP = {
    0x0201: "运动完成",
    0x0202: "高位模式已进入",
    0x0203: "降位完成",
    0x0204: "上台阶完成",
    0x0205: "下台阶完成",
    0x0206: "急停完成",
    0x0301: "路径请求",
}


class LidarPanelWidget(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("R2 位姿状态")
        self.setMinimumWidth(320)

        layout = QVBoxLayout(self)

        # ── 定位状态 ──
        loc_group = QGroupBox("定位状态")
        loc_layout = QVBoxLayout(loc_group)

        self.localized_label = QLabel("定位: ⏳ 等待中")
        self.localized_label.setFont(QFont("Arial", 12))
        loc_layout.addWidget(self.localized_label)

        self.fitness_label = QLabel("得分: --")
        self.fitness_label.setFont(QFont("Arial", 12))
        loc_layout.addWidget(self.fitness_label)

        layout.addWidget(loc_group)

        # ── 启动坐标（发布一次，transient_local）──
        start_pos_group = QGroupBox("启动坐标")
        start_pos_layout = QHBoxLayout(start_pos_group)

        start_left_col = QVBoxLayout()
        self.start_x_label = QLabel("X: ---- mm")
        self.start_x_label.setFont(QFont("Arial", 12))
        start_left_col.addWidget(self.start_x_label)

        start_right_col = QVBoxLayout()
        self.start_y_label = QLabel("Y: ---- mm")
        self.start_y_label.setFont(QFont("Arial", 12))
        start_right_col.addWidget(self.start_y_label)

        start_pos_layout.addLayout(start_left_col)
        start_pos_layout.addLayout(start_right_col)
        layout.addWidget(start_pos_group)

        # ── 实时坐标 ──
        coord_group = QGroupBox("实时坐标")
        coord_layout = QHBoxLayout(coord_group)

        left_col = QVBoxLayout()
        self.x_label = QLabel("X: ---- mm")
        self.x_label.setFont(QFont("Arial", 12))
        left_col.addWidget(self.x_label)

        self.z_label = QLabel("Z: ---- mm")
        self.z_label.setFont(QFont("Arial", 12))
        left_col.addWidget(self.z_label)

        right_col = QVBoxLayout()
        self.y_label = QLabel("Y: ---- mm")
        self.y_label.setFont(QFont("Arial", 12))
        right_col.addWidget(self.y_label)

        self.yaw_label = QLabel("Yaw: ---- °")
        self.yaw_label.setFont(QFont("Arial", 12))
        right_col.addWidget(self.yaw_label)

        coord_layout.addLayout(left_col)
        coord_layout.addLayout(right_col)
        layout.addWidget(coord_group)

        # ── 串口状态 ──
        serial_group = QGroupBox("串口状态")
        serial_layout = QVBoxLayout(serial_group)

        self.connection_label = QLabel("下发节点: ⏳ 检测中")
        self.connection_label.setFont(QFont("Arial", 12))
        serial_layout.addWidget(self.connection_label)

        self.mcu_label = QLabel("MCU 事件: --")
        self.mcu_label.setFont(QFont("Arial", 12))
        serial_layout.addWidget(self.mcu_label)

        layout.addWidget(serial_group)

        # ── 关闭按钮 ──
        close_btn = QPushButton("关闭")
        close_btn.setFixedHeight(40)
        close_btn.clicked.connect(self.close)
        layout.addWidget(close_btn)

    # ── Public update slots ──

    def update_odom(self, x_mm: int, y_mm: int, z_mm: int, yaw_deg: int):
        self.x_label.setText(f"X: {x_mm} mm")
        self.y_label.setText(f"Y: {y_mm} mm")
        self.z_label.setText(f"Z: {z_mm} mm")
        self.yaw_label.setText(f"Yaw: {yaw_deg}°")

    def update_localized(self, localized: bool):
        if localized:
            self.localized_label.setText("定位: ✓ 已定位")
            self.localized_label.setStyleSheet("color: green;")
        else:
            self.localized_label.setText("定位: ✗ 未定位")
            self.localized_label.setStyleSheet("color: red;")

    def update_fitness(self, fitness: float):
        self.fitness_label.setText(f"得分: {fitness:.2f}")

    def update_connection(self, connected: bool):
        if connected:
            self.connection_label.setText("下发节点: ● 已连接")
            self.connection_label.setStyleSheet("color: green;")
        else:
            self.connection_label.setText("下发节点: ○ 未连接")
            self.connection_label.setStyleSheet("color: red;")

    def refresh_from_cache(self, node):
        """从 ros_node 缓存刷新所有标签（面板打开时调用, 无需等下次 callback）."""
        # 定位状态
        if node._last_localized is not None:
            self.update_localized(node._last_localized)
        if node._last_fitness is not None:
            self.update_fitness(node._last_fitness)
        # 实时位姿
        self.update_odom(node._last_odom_x, node._last_odom_y,
                         node._last_odom_z, node._last_odom_yaw)
        # 启动坐标
        if node._initial_x is not None:
            self.update_initial_position(node._initial_x, node._initial_y)
        # 连接状态
        if node._last_connection is not None:
            self.update_connection(node._last_connection)

    def update_initial_position(self, x_mm: int, y_mm: int):
        self.start_x_label.setText(f"X: {x_mm} mm")
        self.start_y_label.setText(f"Y: {y_mm} mm")

    def update_mcu_event(self, event_code: int):
        desc = EVENT_MAP.get(event_code, f"未知事件")
        self.mcu_label.setText(f"MCU 事件: 0x{event_code:04X} {desc}")
