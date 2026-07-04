"""主窗口：方格编辑器 UI 及交互逻辑。"""

from PySide6.QtCore import Qt, Signal, Slot
from PySide6.QtWidgets import (
    QMainWindow,
    QVBoxLayout,
    QGridLayout,
    QWidget,
    QPushButton,
    QGraphicsScene,
    QGraphicsView,
    QDockWidget,
    QButtonGroup,
    QLabel,
    QMessageBox,
)
from PySide6.QtGui import QColor

try:
    from .block_item import BlockLevel, BlockItem, BlockType
    from .launch_control_widget import LaunchControlWidget
except ImportError:
    from block_item import BlockLevel, BlockItem, BlockType
    from launch_control_widget import LaunchControlWidget


class MainWindow(QMainWindow):
    emit_grid = Signal(dict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("手输命令 - 方格编辑器")
        self.grid_items: list[list[BlockItem]] = []
        self.ros_node = None

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
        self.debug_mode = False

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

    def create_side_panel(self, debug_mode=False):
        if self.debug_mode == debug_mode and hasattr(self, "right_dock"):
            return
        if hasattr(self, "right_dock"):
            self.removeDockWidget(self.right_dock)
        self.debug_mode = debug_mode
        self.right_dock = QDockWidget("方块类型", self)
        widget = QWidget(self)
        layout = QGridLayout(widget)

        if debug_mode:
            select_blue_scene = QPushButton(self)
            select_blue_scene.setText("蓝色场景")
            select_blue_scene.setStyleSheet(
                f"background-color: {QColor('lightblue').name()};")
            select_blue_scene.clicked.connect(lambda: self.change_scene(0))
            select_blue_scene.setFixedSize(100, 100)
            layout.addWidget(select_blue_scene, 0, 0)

            select_red_scene = QPushButton(self)
            select_red_scene.setText("红色场景")
            select_red_scene.setStyleSheet(
                f"background-color: {QColor('lightcoral').name()};")
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
            btn.clicked.connect(
                lambda checked, t=type_id: self.set_selected_type(t))
            btn.setFixedSize(100, 100)
            btn.setStyleSheet(
                f"background-color: {color.name()}; color: {text_color.name()};"
            )
            layout.addWidget(btn, count // 2 + (1 if debug_mode else 0),
                             count % 2)
            count += 1
            self.type_buttons.addButton(btn)

        emit_btn = QPushButton("发布")
        emit_btn.clicked.connect(self.send_grid)
        emit_btn.setFixedSize(100, 100)
        emit_btn.setStyleSheet(
            f"background-color: {QColor('lightgray').name()}; "
            f"color: {QColor('black').name()};"
        )
        layout.addWidget(emit_btn)

        # 启动控制按钮
        launch_btn = QPushButton("启动控制")
        launch_btn.clicked.connect(self._open_launch_control)
        launch_btn.setFixedSize(100, 100)
        launch_btn.setStyleSheet(
            "background-color: orange; color: black; font-weight: bold;"
        )
        layout.addWidget(launch_btn)

        self.right_dock.setWidget(widget)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea,
                           self.right_dock)

    def create_lidar_panel(self):
        self.left_dock = QDockWidget("lidar panel", self)
        widget = QWidget(self)
        layout = QVBoxLayout(widget)

        self.lidar_pos_label = QLabel(
            "lidar position: (null, null, null)", widget)
        layout.addWidget(self.lidar_pos_label)

        self.left_dock.setWidget(widget)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea,
                           self.left_dock)
        self.left_dock.setVisible(False)

    def create_menu(self):
        self.toolbar = self.addToolBar("toolbar")

        self.lidar_panel_action = self.toolbar.addAction("lidar panel")
        self.lidar_panel_action.triggered.connect(
            lambda: self.left_dock.setVisible(True))

        self.grid_panel_action = self.toolbar.addAction("grid panel")
        self.grid_panel_action.triggered.connect(
            lambda: self.right_dock.setVisible(True))

        self.debug_mode_action = self.toolbar.addAction("debug mode")
        self.debug_mode_action.triggered.connect(
            lambda: self.create_side_panel(True)
            if not self.debug_mode else self.create_side_panel(False))

    def get_kfs_type(self) -> list[list[BlockType]]:
        return [[item.block_type for item in row] for row in self.grid_items]

    def get_selected_kfs_type(self) -> BlockType:
        for row in self.grid_items:
            for item in row:
                if item.isSelected():
                    return item.block_type
        return BlockType.Empty

    def closeEvent(self, event):
        super().closeEvent(event)

    @Slot(list)
    def update_path(self, path_data: list):
        for row in range(len(self.grid_items)):
            for col in range(len(self.grid_items[row])):
                is_path = [row, col + 1] in path_data
                self.grid_items[row][col].update_path_status(is_path)

    @Slot(int, int, int)
    def update_lidar_position(self, x_mm: int, y_mm: int, yaw_degree: int):
        self.lidar_pos_label.setText(
            f"lidar position: ({x_mm}, {y_mm}, {yaw_degree})")

    def set_selected_type(self, new_type: BlockType):
        selected_items = self.graphics_scene.selectedItems()
        for item in selected_items:
            if isinstance(item, BlockItem):
                item.set_type(new_type)

    def clear_selection(self):
        self.graphics_scene.clearSelection()

    def _open_launch_control(self):
        self.launch_control_dialog = LaunchControlWidget(self)
        self.launch_control_dialog.show()

    def change_scene(self, scene_index: int):
        if scene_index == self.scene_index:
            return
        self.scene_index = scene_index
        new_grid = []
        if scene_index == 0:  # 蓝色场景
            new_grid = [
                [BlockLevel.Medium, BlockLevel.High, BlockLevel.Medium,
                 BlockLevel.Low],
                [BlockLevel.Low, BlockLevel.Medium, BlockLevel.High,
                 BlockLevel.Medium],
                [BlockLevel.Medium, BlockLevel.Low, BlockLevel.Medium,
                 BlockLevel.Low],
            ]
            self.graphics_scene.setBackgroundBrush(QColor("lightblue"))
        elif scene_index == 1:  # 红色场景
            new_grid = [
                [BlockLevel.Medium, BlockLevel.Low, BlockLevel.Medium,
                 BlockLevel.Low],
                [BlockLevel.Low, BlockLevel.Medium, BlockLevel.High,
                 BlockLevel.Medium],
                [BlockLevel.Medium, BlockLevel.High, BlockLevel.Medium,
                 BlockLevel.Low],
            ]
            self.graphics_scene.setBackgroundBrush(QColor("lightcoral"))
        self.load_grid(new_grid)

    def load_grid(self, grid: list[list[BlockLevel]]):
        self.reset_grid(grid)

    def send_grid(self):
        grid = dict()
        grid["grid"] = self.get_kfs_type()
        grid["level"] = [[item.block_level for item in row]
                         for row in self.grid_items]
        self.emit_grid.emit(grid)
        QMessageBox.information(self, "发布成功", "方格数据已发布到 ROS2！")
