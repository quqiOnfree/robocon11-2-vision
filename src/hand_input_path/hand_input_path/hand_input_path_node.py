from PySide6.QtCore import Qt, Signal, Slot, QObject, QTimer
from PySide6.QtWidgets import (
    QMainWindow,
    QVBoxLayout,
    QGridLayout,
    QWidget,
    QPushButton,
    QGraphicsScene,
    QGraphicsView,
    QGraphicsRectItem,
    QGraphicsItem,
    QDockWidget,
    QButtonGroup,
    QStyleOptionGraphicsItem,
    QApplication,
    QMessageBox,
    QLabel
)
from PySide6.QtGui import QPainter, QColor, QFont, QPen
import sys
import json
from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8
import r2_serial.msg._serial_packet as serial_packet

class BlockLevel(Enum):
    Ground = 0
    Low = 1
    Medium = 2
    High = 3


class BlockType(Enum):
    Empty = 0
    R1_KFS = 1
    R2_KFS = 2
    False_KFS = 3


class BlockItem(QGraphicsRectItem):
    def __init__(self, x, y, width, height, block_level, block_type=BlockType.Empty):
        super().__init__(x, y, width, height)
        self.block_level = block_level
        self.block_type = block_type
        self.is_path = False
        self.setBrush(self.get_color())
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)

    def get_color(self):
        if self.block_level == BlockLevel.Low:
            return QColor("darkgreen")
        elif self.block_level == BlockLevel.Medium:
            return QColor("green")
        else:
            return QColor("yellow")

    def set_type(self, new_type: BlockType):
        self.block_type = new_type
        self.update_color()

    def update_color(self):
        self.setBrush(self.get_color())
        self.update()

    def update_path_status(self, is_path: bool):
        self.is_path = is_path
        self.update()

    def paint(
        self,
        painter: QPainter,
        option: QStyleOptionGraphicsItem,
        widget: QWidget | None = None,
    ):
        # 先绘制矩形背景
        super().paint(painter, option, widget)

        # 如果是路径上的方块，绘制一个半透明的覆盖层
        if (self.is_path):
            painter.setBrush(QColor(255, 0, 255))
            rect = self.rect()
            small_rect = rect.adjusted(20, 20, -20, -20)
            painter.drawRect(small_rect)

        # 根据 block_type 绘制不同的标识

        types = [
            ("空", BlockType.Empty, QColor("lightgray"), QColor("black")),
            ("R1KFS", BlockType.R1_KFS, QColor("blue"), QColor("white")),
            ("R2KFS", BlockType.R2_KFS, QColor("red"), QColor("white")),
            ("FalseKFS", BlockType.False_KFS, QColor("darkred"), QColor("white")),
        ]

        if self.block_type == BlockType.Empty:
            rect = self.rect()
            painter.setBrush(types[0][2])  # lightgray
            small_rect = rect.adjusted(30, 30, -30, -30)
            painter.drawRect(small_rect)
            painter.setFont(QFont("Arial", 10))
            painter.setPen(types[0][3])  # black
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, "Empty")
        elif self.block_type == BlockType.R1_KFS:
            rect = self.rect()
            painter.setBrush(types[1][2])  # blue
            small_rect = rect.adjusted(30, 30, -30, -30)
            painter.drawRect(small_rect)
            painter.setFont(QFont("Arial", 10))
            painter.setPen(types[1][3])  # white
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, "R1_KFS")
        elif self.block_type == BlockType.R2_KFS:
            rect = self.rect()
            painter.setBrush(types[2][2])  # red
            small_rect = rect.adjusted(30, 30, -30, -30)
            painter.drawRect(small_rect)
            painter.setFont(QFont("Arial", 10))
            painter.setPen(types[2][3])  # white
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, "R2_KFS")
        elif self.block_type == BlockType.False_KFS:
            rect = self.rect()
            painter.setBrush(types[3][2])  # darkred
            small_rect = rect.adjusted(30, 30, -30, -30)
            painter.drawRect(small_rect)
            painter.setFont(QFont("Arial", 10))
            painter.setPen(types[3][3])  # white
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, "False_KFS")

        if self.isSelected():
            painter.setPen(QPen(QColor("red"), 5))
            painter.setBrush(QColor("red"))
            bounding = self.boundingRect()
            painter.drawPolyline([bounding.topLeft(), bounding.topRight(),
                                  bounding.bottomRight(), bounding.bottomLeft(),
                                  bounding.topLeft()])

class MainWindow(QMainWindow):
    emit_grid = Signal(dict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("手输命令 - 方格编辑器")
        self.grid_items: list[list[BlockItem]] = []

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout()
        central_widget.setLayout(layout)

        self.graphics_scene = QGraphicsScene(self)
        self.graphics_scene.setSceneRect(0, 0, 500, 400 + 20)
        self.graphics_view = QGraphicsView(self.graphics_scene, parent=self)
        self.graphics_view.setMinimumSize(500, 400 + 20)
        layout.addWidget(self.graphics_view)

        self.scene_index = 1

        self.create_side_panel()
        self.create_lidar_panel()
        self.create_menu()
        self.change_scene(0)  # 默认加载蓝色场景

    def create_grid(self, grid: list[list[BlockLevel]]):
        self.grid_items = []
        for r in range(len(grid)):
            row_items = []
            for c in range(len(grid[0])):
                x = (r + 1) * 100
                y = c * 100 + 10
                item = BlockItem(x, y, 100, 100, grid[r][c])
                self.graphics_scene.addItem(item)
                row_items.append(item)
            self.grid_items.append(row_items)

    def reset_grid(self, grid: list[list[BlockLevel]]):
        self.graphics_scene.clear()
        self.grid_items = []
        self.create_grid(grid)

    def create_side_panel(self):
        # 右侧 self.right_dock 窗口
        self.right_dock = QDockWidget("方块类型", self)
        widget = QWidget(self)
        layout = QGridLayout(widget)

        # scene_combo = QComboBox()
        # scene_combo.addItems(["蓝色场景", "红色场景"])
        # scene_combo.currentIndexChanged.connect(self.change_scene)
        # scene_combo.setFixedSize(100, 100)
        # layout.addWidget(scene_combo, 0, 0)

        select_blue_scene = QPushButton(self)
        select_blue_scene.setText("蓝色场景")
        select_blue_scene.setStyleSheet(f"background-color: {QColor('lightblue').name()};")
        select_blue_scene.clicked.connect(lambda: self.change_scene(0))
        select_blue_scene.setFixedSize(100, 100)
        layout.addWidget(select_blue_scene, 0, 0)

        select_red_scene = QPushButton(self)
        select_red_scene.setText("红色场景")
        select_red_scene.setStyleSheet(f"background-color: {QColor('lightcoral').name()};")
        select_red_scene.clicked.connect(lambda: self.change_scene(1))
        select_red_scene.setFixedSize(100, 100)
        layout.addWidget(select_red_scene, 0, 1)

        # 按钮组（互斥效果，但不强制）
        self.type_buttons = QButtonGroup(self)

        types = [
            ("空", BlockType.Empty, QColor("lightgray"), QColor("black")),
            ("R1KFS", BlockType.R1_KFS, QColor("blue"), QColor("white")),
            ("R2KFS", BlockType.R2_KFS, QColor("red"), QColor("white")),
            ("FalseKFS", BlockType.False_KFS, QColor("darkred"), QColor("white")),
        ]

        count = 0
        for name, type_id, color, text_color in types:
            btn = QPushButton(name)
            btn.setCheckable(True)
            btn.clicked.connect(lambda checked, t=type_id: self.set_selected_type(t))
            btn.setFixedSize(100, 100)
            btn.setStyleSheet(
                f"background-color: {color.name()}; color: {text_color.name()};"
            )
            layout.addWidget(btn, count // 2 + 1, count % 2)
            count += 1
            self.type_buttons.addButton(btn)

        emit_btn = QPushButton("发布")
        emit_btn.clicked.connect(self.send_grid)
        emit_btn.setFixedSize(100, 100)
        emit_btn.setStyleSheet(
            f"background-color: {QColor('lightgray').name()}; color: {QColor('black').name()};"
        )
        layout.addWidget(emit_btn)

        # 添加一个清除选中的按钮
        clear_btn = QPushButton("清除选中")
        clear_btn.clicked.connect(self.clear_selection)
        clear_btn.setFixedSize(100, 100)
        layout.addWidget(clear_btn)

        self.right_dock.setWidget(widget)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.right_dock)

    def create_lidar_panel(self):
        self.left_dock = QDockWidget("lidar panel", self)
        widget = QWidget(self)
        layout = QVBoxLayout(widget)
        
        self.lidar_pos_label = QLabel("lidar position: (null, null, null)", widget)
        layout.addWidget(self.lidar_pos_label)

        self.left_dock.setWidget(widget)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.left_dock)
        self.left_dock.setVisible(False)

    def create_menu(self):
        self.toolbar = self.addToolBar("toolbar")

        self.lidar_panel = self.toolbar.addAction("lidar panel")
        self.lidar_panel.triggered.connect(lambda: self.left_dock.setVisible(True))
        
        self.grid_panel = self.toolbar.addAction("grid_panel")
        self.grid_panel.triggered.connect(lambda: self.right_dock.setVisible(True))

    def get_kfs_type(self) -> list[list[BlockType]]:
        return [[item.block_type for item in row] for row in self.grid_items]

    def get_selected_kfs_type(self) -> BlockType:
        for row in self.grid_items:
            for item in row:
                if item.isSelected():
                    return item.block_type
        return BlockType.Empty

    def print_grid(self):
        grid = self.get_kfs_type()
        print("grid:")
        for row in grid:
            print([item.name for item in row])

    def closeEvent(self, event):
        # self.print_grid()
        super().closeEvent(event)

    @Slot(list)
    def update_path(self, path_data: list):
        for row in range(len(self.grid_items)):
            for col in range(len(self.grid_items[row])):
                is_path = [row, col + 1] in path_data
                self.grid_items[row][col].update_path_status(is_path)

    @Slot(int, int, int)
    def update_lidar_position(self, x_mm: int, y_mm: int, yaw_degree: int):
        self.lidar_pos_label.setText(f"lidar position: ({x_mm}, {y_mm}, {yaw_degree})")

    # ---------- 核心逻辑：把选中的方块设为指定类型 ----------
    def set_selected_type(self, new_type: BlockType):
        selected_items = self.graphics_scene.selectedItems()
        for item in selected_items:
            if isinstance(item, BlockItem):
                item.set_type(new_type)

    def clear_selection(self):
        self.graphics_scene.clearSelection()

    def change_scene(self, scene_index: int):
        if scene_index == self.scene_index:
            return
        self.scene_index = scene_index
        new_grid = []
        if scene_index == 0:  # 蓝色场景
            new_grid = [
                [BlockLevel.Medium, BlockLevel.High, BlockLevel.Medium, BlockLevel.Low],
                [BlockLevel.Low, BlockLevel.Medium, BlockLevel.High, BlockLevel.Medium],
                [BlockLevel.Medium, BlockLevel.Low, BlockLevel.Medium, BlockLevel.Low],
            ]
            self.graphics_scene.setBackgroundBrush(QColor("lightblue"))
        elif scene_index == 1:  # 红色场景
            new_grid = [
                [BlockLevel.Medium, BlockLevel.Low, BlockLevel.Medium, BlockLevel.Low],
                [BlockLevel.Low, BlockLevel.Medium, BlockLevel.High, BlockLevel.Medium],
                [BlockLevel.Medium, BlockLevel.High, BlockLevel.Medium, BlockLevel.Low],
            ]
            self.graphics_scene.setBackgroundBrush(QColor("lightcoral"))
        self.load_grid(new_grid)

    def load_grid(self, grid: list[list[BlockLevel]]):
        self.reset_grid(grid)

    def send_grid(self):
        grid = dict()
        grid["grid"] = self.get_kfs_type()
        grid["level"] = [[item.block_level for item in row] for row in self.grid_items]
        self.emit_grid.emit(grid)
        QMessageBox.information(self, "发布成功", "方格数据已发布到 ROS2！")


class PathSignalEmitter(QObject):
    path_signal = Signal(list)
    scene_signal = Signal(int)
    lidar_position_signal = Signal(int, int, int)

class Ros2Node(Node):
    def __init__(self):
        Node.__init__(self, "hand_input_path_node")
        self.path_signal = PathSignalEmitter()
        # 这里可以初始化 ROS2 节点和发布者
        self.grid_publisher = self.create_publisher(String, "grid_data", 10)
        self.scene_subcription = self.create_subscription(Int8, "/r2/match_zone", self.scene_received, 10)
        self.subscriber = self.create_subscription(String, "path_commands", self.path_received, 10)
        self.serial_subscriber = self.create_subscription(serial_packet.SerialPacket,
                                                          "/r2_serial/downlink/packet", self.serial_received, 10)

    @Slot(dict)
    def publish_grid(self, grid: dict):
        # 这里实现发布逻辑，例如将 grid 转换为 ROS2 消息并发布
        grid_data = dict()
        grid_data["grid"] = [[block_type.value for block_type in row] for row in grid["grid"]]
        grid_data["level"] = [[block_level.value for block_level in row] for row in grid["level"]]
        json_data = json.dumps(grid_data)
        msg = String()
        msg.data = json_data
        self.grid_publisher.publish(msg)
        print("Published grid data:", json_data)

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

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    window = MainWindow()
    node = Ros2Node()
    window.emit_grid.connect(node.publish_grid)
    node.path_signal.path_signal.connect(window.update_path)
    node.path_signal.scene_signal.connect(window.change_scene)
    node.path_signal.lidar_position_signal.connect(window.update_lidar_position)
    window.show()
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)  # 每10毫秒检查一次ROS2事件

    # 关闭时的清理
    def cleanup():
        timer.stop()
        node.destroy_node()
        rclpy.shutdown()

    app.aboutToQuit.connect(cleanup)
    app.exec()

if __name__ == "__main__":
    main()
