from PySide6.QtCore import Qt, Signal, Slot, QObject, QTimer
from PySide6.QtWidgets import (
    QMainWindow,
    QVBoxLayout,
    QWidget,
    QPushButton,
    QGraphicsScene,
    QGraphicsView,
    QGraphicsRectItem,
    QGraphicsItem,
    QDockWidget,
    QButtonGroup,
    QComboBox,
    QStyleOptionGraphicsItem,
    QApplication,
    QMessageBox,
)
from PySide6.QtGui import QPainter, QColor, QFont
import sys
import json
from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


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

    def paint(
        self,
        painter: QPainter,
        option: QStyleOptionGraphicsItem,
        widget: QWidget | None = None,
    ):
        # 先绘制矩形背景
        super().paint(painter, option, widget)

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
        self.graphics_scene.setSceneRect(0, 0, 500, 600)
        self.graphics_view = QGraphicsView(self.graphics_scene, parent=self)
        self.graphics_view.setMinimumSize(500, 600)
        layout.addWidget(self.graphics_view)

        self.create_side_panel()
        self.change_scene(0)  # 默认加载蓝色场景

    def create_grid(self, grid: list[list[BlockLevel]]):
        self.grid_items = []
        for r in range(len(grid)):
            row_items = []
            for c in range(len(grid[0])):
                x = (r + 1) * 100
                y = (c + 1) * 100
                item = BlockItem(x, y, 100, 100, grid[r][c])
                self.graphics_scene.addItem(item)
                row_items.append(item)
            self.grid_items.append(row_items)

    def reset_grid(self, grid: list[list[BlockLevel]]):
        self.graphics_scene.clear()
        self.grid_items = []
        self.create_grid(grid)

    def create_side_panel(self):
        # 右侧 Dock 窗口
        dock = QDockWidget("方块类型", self)
        widget = QWidget()
        layout = QVBoxLayout(widget)

        scene_combo = QComboBox()
        scene_combo.addItems(["蓝色场景", "红色场景"])
        scene_combo.currentIndexChanged.connect(self.change_scene)
        scene_combo.setFixedSize(100, 100)
        layout.addWidget(scene_combo)

        # 按钮组（互斥效果，但不强制）
        self.type_buttons = QButtonGroup(self)

        types = [
            ("空", BlockType.Empty, QColor("lightgray"), QColor("black")),
            ("R1KFS", BlockType.R1_KFS, QColor("blue"), QColor("white")),
            ("R2KFS", BlockType.R2_KFS, QColor("red"), QColor("white")),
            ("FalseKFS", BlockType.False_KFS, QColor("darkred"), QColor("white")),
        ]

        for name, type_id, color, text_color in types:
            btn = QPushButton(name)
            btn.setCheckable(True)
            btn.clicked.connect(lambda checked, t=type_id: self.set_selected_type(t))
            btn.setFixedSize(100, 100)
            btn.setStyleSheet(
                f"background-color: {color.name()}; color: {text_color.name()};"
            )
            layout.addWidget(btn)
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
        layout.addWidget(clear_btn)

        layout.addStretch()
        dock.setWidget(widget)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dock)

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
        self.print_grid()
        super().closeEvent(event)

    # ---------- 核心逻辑：把选中的方块设为指定类型 ----------
    def set_selected_type(self, new_type: BlockType):
        selected_items = self.graphics_scene.selectedItems()
        for item in selected_items:
            if isinstance(item, BlockItem):
                item.set_type(new_type)

    def clear_selection(self):
        self.graphics_scene.clearSelection()

    def change_scene(self, scene_index: int):
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


class Ros2Node(Node, QObject):
    def __init__(self):
        super().__init__("hand_input_path_node")
        # 这里可以初始化 ROS2 节点和发布者
        self.publisher = self.create_publisher(String, "grid_data", 10)

    @Slot(dict)
    def publish_grid(self, grid: dict):
        # 这里实现发布逻辑，例如将 grid 转换为 ROS2 消息并发布
        grid_data = dict()
        grid_data["grid"] = [[block_type.value for block_type in row] for row in grid["grid"]]
        grid_data["level"] = [[block_level.value for block_level in row] for row in grid["level"]]
        json_data = json.dumps(grid_data)
        msg = String()
        msg.data = json_data
        self.publisher.publish(msg)
        print("Published grid data:", json_data)


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    window = MainWindow()
    node = Ros2Node()
    window.emit_grid.connect(node.publish_grid)
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
