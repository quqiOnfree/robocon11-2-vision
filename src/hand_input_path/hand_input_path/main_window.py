"""主窗口：方格编辑器 UI 及交互逻辑（Tab 化单窗口）。"""

from PySide6.QtCore import Qt, Signal, Slot
from PySide6.QtWidgets import (
    QMainWindow,
    QGridLayout,
    QHBoxLayout,
    QWidget,
    QPushButton,
    QGraphicsScene,
    QGraphicsView,
    QTabWidget,
    QButtonGroup,
    QMessageBox,
)
from PySide6.QtGui import QColor, QFont

try:
    from .block_item import BlockLevel, BlockItem, BlockType, BLOCK_TYPE_DISPLAY
    from .launch_control_widget import LaunchControlWidget
    from .lidar_panel_widget import LidarPanelWidget
    from .debug_widget import DebugWidget
except ImportError:
    from block_item import BlockLevel, BlockItem, BlockType, BLOCK_TYPE_DISPLAY
    from launch_control_widget import LaunchControlWidget
    from lidar_panel_widget import LidarPanelWidget
    from debug_widget import DebugWidget


class MainWindow(QMainWindow):
    emit_grid = Signal(dict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("手输命令 - 方格编辑器")
        self.grid_items: list[list[BlockItem]] = []
        self.ros_node = None
        self.scene_index = -1

        # ── QTabWidget 作为中央控件 ──
        self.tab_widget = QTabWidget()
        self.tab_widget.setFont(QFont("Arial", 16, QFont.Bold))
        self.tab_widget.setStyleSheet(
            "QTabBar::tab { padding: 12px 28px; min-width: 80px; min-height: 32px; }"
        )
        self.setCentralWidget(self.tab_widget)

        # ── Tab 0: Grid ──
        self._create_grid_tab()

        # ── Tab 1: Lidar ──
        self.lidar_panel = LidarPanelWidget()
        self.tab_widget.addTab(self.lidar_panel, "Lidar")

        # ── Tab 2: Debug ──
        self.debug_panel = DebugWidget()
        self.tab_widget.addTab(self.debug_panel, "Debug")

        # ── Tab 3: Launch ──
        self.launch_panel = LaunchControlWidget(main_window=self)
        self.tab_widget.addTab(self.launch_panel, "Launch")

        self._set_widgets_enabled(False)

        # 每个 Tab 用不同颜色区分
        tab_bar = self.tab_widget.tabBar()
        tab_bar.setTabTextColor(0, QColor("#2196F3"))  # Grid 蓝色
        tab_bar.setTabTextColor(1, QColor("#4CAF50"))  # Lidar 绿色
        tab_bar.setTabTextColor(2, QColor("#FF9800"))  # Debug 橙色
        tab_bar.setTabTextColor(3, QColor("#F44336"))  # Launch 红色

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

    def _create_grid_tab(self):
        """创建 Grid Tab：左侧网格编辑器 + 右侧方块类型面板。"""
        grid_tab = QWidget()
        h_layout = QHBoxLayout(grid_tab)
        h_layout.setContentsMargins(4, 4, 4, 4)

        # ── 左侧：网格编辑器 ──
        self.graphics_scene = QGraphicsScene(self)
        self.graphics_scene.setSceneRect(0, 0, 500, 400 + 20)
        self.graphics_view = QGraphicsView(self.graphics_scene, parent=self)
        self.graphics_view.setMinimumSize(500, 400 + 20)
        h_layout.addWidget(self.graphics_view, stretch=1)

        # ── 右侧：方块类型面板 ──
        side_panel = QWidget()
        side_layout = QGridLayout(side_panel)

        select_blue_scene = QPushButton("蓝色场景")
        select_blue_scene.setStyleSheet(
            f"background-color: {QColor('lightblue').name()};")
        select_blue_scene.clicked.connect(lambda: self.change_scene(0))
        select_blue_scene.setFixedSize(100, 100)
        side_layout.addWidget(select_blue_scene, 0, 0)

        select_red_scene = QPushButton("红色场景")
        select_red_scene.setStyleSheet(
            f"background-color: {QColor('lightcoral').name()};")
        select_red_scene.clicked.connect(lambda: self.change_scene(1))
        select_red_scene.setFixedSize(100, 100)
        side_layout.addWidget(select_red_scene, 0, 1)

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
            side_layout.addWidget(btn, count // 2 + 1, count % 2)
            count += 1
            self.type_buttons.addButton(btn)

        self.emit_btn = QPushButton("发布")
        self.emit_btn.clicked.connect(self.send_grid)
        self.emit_btn.setFixedSize(100, 100)
        self.emit_btn.setStyleSheet(
            f"background-color: {QColor('lightgray').name()}; "
            f"color: {QColor('black').name()};"
        )
        side_layout.addWidget(self.emit_btn)

        h_layout.addWidget(side_panel, stretch=0)

        # 占位提示文字
        self._placeholder = self.graphics_scene.addText("请选择半场")
        font = self._placeholder.font()
        font.setPointSize(24)
        self._placeholder.setFont(font)
        self._placeholder.setPos(150, 170)

        self.tab_widget.addTab(grid_tab, "Grid")

    def get_kfs_type(self) -> list[list[BlockType]]:
        return [[item.block_type for item in row] for row in self.grid_items]

    def get_selected_kfs_type(self) -> BlockType:
        for row in self.grid_items:
            for item in row:
                if item.isSelected():
                    return item.block_type
        return BlockType.Empty

    def connect_ros_signals(self):
        """连接 ROS2 信号到 Lidar/Debug 面板（ros_node 就绪后调用）。"""
        if self.ros_node is None:
            return
        sig = self.ros_node.path_signal
        sig.odom_signal.connect(self.lidar_panel.update_odom)
        sig.localized_signal.connect(self.lidar_panel.update_localized)
        sig.fitness_signal.connect(self.lidar_panel.update_fitness)
        sig.connection_signal.connect(self.lidar_panel.update_connection)
        sig.mcu_event_signal.connect(self.lidar_panel.update_mcu_event)
        sig.initial_position_signal.connect(
            self.lidar_panel.update_initial_position)
        sig.debug_msg_signal.connect(self.debug_panel.update_debug_msg)
        # 立刻用缓存值刷新面板
        self.lidar_panel.refresh_from_cache(self.ros_node)

    def closeEvent(self, event):
        # 关闭前先停雷达，避免子进程变成孤儿
        if self.launch_panel._radar_running:
            self.launch_panel._stop_radar(silent=True)
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
        grid["is_blue_scene"] = (self.scene_index == 0)
        self.emit_grid.emit(grid)
        QMessageBox.information(self, "发布成功", "方格数据已发布到 ROS2！")
