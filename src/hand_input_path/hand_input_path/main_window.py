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
    from .block_item import BlockLevel, BlockItem, BlockType, BLOCK_TYPE_DISPLAY
    from .launch_control_widget import LaunchControlWidget
    from .lidar_panel_widget import LidarPanelWidget
except ImportError:
    from block_item import BlockLevel, BlockItem, BlockType, BLOCK_TYPE_DISPLAY
    from launch_control_widget import LaunchControlWidget
    from lidar_panel_widget import LidarPanelWidget


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

        self.scene_index = -1

        self.create_side_panel()
        self.create_menu()

        # 占位提示文字
        self._placeholder = self.graphics_scene.addText("请选择半场")
        font = self._placeholder.font()
        font.setPointSize(24)
        self._placeholder.setFont(font)
        self._placeholder.setPos(150, 170)

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
        if hasattr(self, '_placeholder') and self._placeholder is not None:
            self.graphics_scene.removeItem(self._placeholder)
            self._placeholder = None
        self.graphics_scene.clear()
        self.grid_items = []
        self.create_grid(grid)

    def create_side_panel(self):
        if hasattr(self, "right_dock"):
            self.removeDockWidget(self.right_dock)
        self.right_dock = QDockWidget("方块类型", self)
        widget = QWidget(self)
        layout = QGridLayout(widget)

        # 蓝/红场景按钮（常驻）
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

        count = 0
        for name, type_id, color, text_color in BLOCK_TYPE_DISPLAY:
            btn = QPushButton(name)
            btn.clicked.connect(
                lambda checked, t=type_id: self.set_selected_type(t))
            btn.setFixedSize(100, 100)
            btn.setStyleSheet(
                f"background-color: {color.name()}; color: {text_color.name()};"
            )
            layout.addWidget(btn, count // 2 + 1, count % 2)  # row 从 1 开始
            count += 1
            self.type_buttons.addButton(btn)

        self.emit_btn = QPushButton("发布")
        self.emit_btn.clicked.connect(self.send_grid)
        self.emit_btn.setFixedSize(100, 100)
        self.emit_btn.setStyleSheet(
            f"background-color: {QColor('lightgray').name()}; "
            f"color: {QColor('black').name()};"
        )
        layout.addWidget(self.emit_btn)

        # 启动控制按钮
        self.launch_btn = QPushButton("启动控制")
        self.launch_btn.clicked.connect(self._open_launch_control)
        self.launch_btn.setFixedSize(100, 100)
        self.launch_btn.setStyleSheet(
            "background-color: orange; color: black; font-weight: bold;"
        )
        layout.addWidget(self.launch_btn)

        self._set_widgets_enabled(False)

        self.right_dock.setWidget(widget)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea,
                           self.right_dock)

    def create_menu(self):
        self.toolbar = self.addToolBar("toolbar")

        self.lidar_panel_action = self.toolbar.addAction("lidar panel")
        self.lidar_panel_action.triggered.connect(self._open_lidar_panel)

        self.grid_panel_action = self.toolbar.addAction("grid panel")
        self.grid_panel_action.triggered.connect(
            lambda: self.right_dock.setVisible(True))

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

    def set_selected_type(self, new_type: BlockType):
        selected_items = self.graphics_scene.selectedItems()
        for item in selected_items:
            if isinstance(item, BlockItem):
                item.set_type(new_type)

    def clear_selection(self):
        self.graphics_scene.clearSelection()

    def _set_widgets_enabled(self, enabled: bool):
        """启用/禁用所有依赖场景的控件。"""
        for btn in self.type_buttons.buttons():
            btn.setEnabled(enabled)
        self.emit_btn.setEnabled(enabled)
        self.launch_btn.setEnabled(enabled)

    def _open_launch_control(self):
        if hasattr(self, 'launch_control_dialog') and self.launch_control_dialog is not None:
            self.launch_control_dialog.show()
            self.launch_control_dialog.raise_()
            return
        self.launch_control_dialog = LaunchControlWidget(self)
        self.launch_control_dialog.show()

    def _open_lidar_panel(self):
        if hasattr(self, 'lidar_panel_dialog') and self.lidar_panel_dialog is not None:
            self.lidar_panel_dialog.show()
            self.lidar_panel_dialog.raise_()
            return
        self.lidar_panel_dialog = LidarPanelWidget(self)
        if self.ros_node:
            sig = self.ros_node.path_signal
            sig.odom_signal.connect(
                self.lidar_panel_dialog.update_odom)
            sig.localized_signal.connect(
                self.lidar_panel_dialog.update_localized)
            sig.fitness_signal.connect(
                self.lidar_panel_dialog.update_fitness)
            sig.connection_signal.connect(
                self.lidar_panel_dialog.update_connection)
            sig.mcu_event_signal.connect(
                self.lidar_panel_dialog.update_mcu_event)
        self.lidar_panel_dialog.show()

    def change_scene(self, scene_index: int):
        if scene_index == self.scene_index:
            return
        self.scene_index = scene_index
        self._set_widgets_enabled(True)
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
        # 发布半场设置
        if self.ros_node:
            self.ros_node.publish_match_zone(scene_index)

    def load_grid(self, grid: list[list[BlockLevel]]):
        self.reset_grid(grid)

    def send_grid(self):
        if self.scene_index < 0 or not self.grid_items:
            QMessageBox.warning(self, "未选择场景", "请先选择蓝色或红色场景！")
            return
        grid = dict()
        grid["grid"] = self.get_kfs_type()
        grid["level"] = [[item.block_level for item in row]
                         for row in self.grid_items]
        self.emit_grid.emit(grid)
        QMessageBox.information(self, "发布成功", "方格数据已发布到 ROS2！")
