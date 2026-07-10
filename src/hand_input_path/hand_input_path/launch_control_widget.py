"""启动控制面板：选择区域、模式、启动/关闭雷达、发送比赛命令。"""

import os
import signal
import subprocess
import threading
import time

try:
    from .debug_widget import emit_debug_line
except ImportError:
    from debug_widget import emit_debug_line

def _read_output(p: subprocess.Popen, tag: str):
    """逐行读取 subprocess 输出, 发送到 debug panel."""
    try:
        for line in iter(p.stdout.readline, ""):
            if line:
                emit_debug_line(tag, line.rstrip("\n"))
    except (ValueError, OSError):
        pass


from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QRadioButton,
    QButtonGroup,
    QMessageBox,
)
from PySide6.QtGui import QFont
from PySide6.QtCore import Qt, QTimer


class CountdownDialog(QDialog):
    """3 秒倒计时弹窗，可取消。"""

    def __init__(self, seconds: int, parent=None):
        super().__init__(parent)
        self.setWindowTitle("倒计时")
        self.setFixedSize(250, 150)
        self._remaining = seconds

        layout = QVBoxLayout(self)

        self.label = QLabel(f"{self._remaining}...")
        self.label.setFont(QFont("Arial", 48))
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.label)

        cancel_btn = QPushButton("取消")
        cancel_btn.setFixedHeight(40)
        cancel_btn.clicked.connect(self.reject)
        layout.addWidget(cancel_btn)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(1000)

    def _tick(self):
        self._remaining -= 1
        if self._remaining <= 0:
            self._timer.stop()
            self.accept()
        else:
            self.label.setText(f"{self._remaining}...")


class LaunchControlWidget(QWidget):
    def __init__(self, main_window, parent=None):
        super().__init__(parent)
        self._main_window = main_window

        layout = QVBoxLayout(self)

        # ── 上半部分：三列 radio button ──
        top_row = QHBoxLayout()

        # 左列：选择启动区域
        zone_col = QVBoxLayout()
        zone_title = QLabel("选择启动区域")
        zone_title.setFont(QFont("Arial", 14, QFont.Bold))
        zone_col.addWidget(zone_title)
        self.zone_group = QButtonGroup(self)
        zones = [
            ("1区", 0),
            ("2区", 1),
            ("3区（坡前启动）", 2),
            ("3区（重试区）", 3),
        ]
        for name, value in zones:
            radio = QRadioButton(name)
            radio.setFont(QFont("Arial", 14))
            self.zone_group.addButton(radio, value)
            zone_col.addWidget(radio)
            if value == 0:
                radio.setChecked(True)
        zone_col.addStretch()
        top_row.addLayout(zone_col)

        # 中列：启动模式
        mode_col = QVBoxLayout()
        mode_title = QLabel("启动模式")
        mode_title.setFont(QFont("Arial", 14, QFont.Bold))
        mode_col.addWidget(mode_title)
        self.mode_group = QButtonGroup(self)
        loc_radio = QRadioButton("定位模式 (Localization)")
        loc_radio.setFont(QFont("Arial", 14))
        self.mode_group.addButton(loc_radio, 0)
        mode_col.addWidget(loc_radio)
        odo_radio = QRadioButton("里程计模式 (Odometry)")
        odo_radio.setFont(QFont("Arial", 14))
        self.mode_group.addButton(odo_radio, 1)
        odo_radio.setChecked(True)
        mode_col.addWidget(odo_radio)
        mode_col.addStretch()
        top_row.addLayout(mode_col)

        # 右列：车内初始方块数量
        block_col = QVBoxLayout()
        block_title = QLabel("车内初始方块数量")
        block_title.setFont(QFont("Arial", 14, QFont.Bold))
        block_col.addWidget(block_title)
        self.block_count_group = QButtonGroup(self)
        for i in range(4):  # 0, 1, 2, 3
            radio = QRadioButton(f"{i} 个")
            radio.setFont(QFont("Arial", 14))
            self.block_count_group.addButton(radio, i)
            block_col.addWidget(radio)
            if i == 0:
                radio.setChecked(True)
        block_col.addStretch()
        top_row.addLayout(block_col)

        # 右二列：三区额外配置（仅三区启动时生效）
        arena_col = QVBoxLayout()
        arena_title = QLabel("三区额外配置")
        arena_title.setFont(QFont("Arial", 14, QFont.Bold))
        arena_col.addWidget(arena_title)

        load_label = QLabel("在三区应装载方块数:")
        load_label.setFont(QFont("Arial", 12))
        arena_col.addWidget(load_label)
        load_row = QHBoxLayout()
        self.arena_load_kfs_group = QButtonGroup(self)
        for i in range(3):  # 0, 1, 2
            radio = QRadioButton(f"{i} 个")
            radio.setFont(QFont("Arial", 14))
            self.arena_load_kfs_group.addButton(radio, i)
            load_row.addWidget(radio)
            if i == 0:
                radio.setChecked(True)
        arena_col.addLayout(load_row)

        delay_label = QLabel("等待秒数:")
        delay_label.setFont(QFont("Arial", 12))
        arena_col.addWidget(delay_label)
        self.arena_delay_group = QButtonGroup(self)
        delay_values = [10, 15, 20, 25, 30, 40, 50, 60]
        delay_row1 = QHBoxLayout()
        delay_row2 = QHBoxLayout()
        for idx, val in enumerate(delay_values):
            radio = QRadioButton(f"{val}s")
            radio.setFont(QFont("Arial", 14))
            self.arena_delay_group.addButton(radio, val)
            if idx < 4:
                delay_row1.addWidget(radio)
            else:
                delay_row2.addWidget(radio)
            if val == 10:
                radio.setChecked(True)
        arena_col.addLayout(delay_row1)
        arena_col.addLayout(delay_row2)
        arena_col.addStretch()

        top_row.addLayout(arena_col)

        layout.addLayout(top_row)
        layout.addSpacing(10)

        # 收集三区配置控件，用于根据 zone 选择启用/禁用
        self._arena_widgets = [arena_title, load_label, delay_label]
        for btn in self.arena_load_kfs_group.buttons():
            self._arena_widgets.append(btn)
        for btn in self.arena_delay_group.buttons():
            self._arena_widgets.append(btn)

        # zone 切换时启用/禁用三区配置
        self.zone_group.idToggled.connect(self._on_zone_changed)
        self._on_zone_changed(self.zone_group.checkedId(), True)

        # ── 下半部分：操作按钮（横向）──
        btn_row = QHBoxLayout()

        self.radar_btn = QPushButton("启动雷达")
        self.radar_btn.setFixedHeight(60)
        self.radar_btn.setFont(QFont("Arial", 14, QFont.Bold))
        self.radar_btn.setStyleSheet("background-color: #2196F3; color: white;")
        self.radar_btn.clicked.connect(self._on_toggle_radar)
        btn_row.addWidget(self.radar_btn)

        start_btn = QPushButton("设置启动区域并开始比赛")
        start_btn.setFixedHeight(60)
        start_btn.setFont(QFont("Arial", 14, QFont.Bold))
        start_btn.setStyleSheet("background-color: red; color: white;")
        start_btn.clicked.connect(self._on_start_command)
        btn_row.addWidget(start_btn)

        layout.addLayout(btn_row)

        self._radar_running = False
        self._radar_processes = []

    def get_mode(self) -> str:
        return "localization" if self.mode_group.checkedId() == 0 else "odometry"

    def get_block_count(self) -> int:
        return self.block_count_group.checkedId()

    def get_arena_load_kfs(self) -> int:
        return self.arena_load_kfs_group.checkedId()

    def get_arena_delay(self) -> int:
        return self.arena_delay_group.checkedId()

    def _on_zone_changed(self, zone_id: int, checked: bool):
        """仅三区（坡前=2, 重试=3）时启用额外配置控件。"""
        if not checked:
            return
        enabled = zone_id in (2, 3)
        for w in self._arena_widgets:
            w.setEnabled(enabled)

    def _on_toggle_radar(self):
        if self._radar_running:
            self._stop_radar()
        else:
            self._start_radar()

    def _start_radar(self):
        scene_index = getattr(self._main_window, "scene_index", -1)
        if scene_index < 0:
            QMessageBox.warning(self, "未选择半场",
                                "请先在主窗口选择蓝色或红色场景！")
            return

        mode = self.get_mode()
        # 红蓝半场对应不同地图和 prior（半场对称, 与 run_competition.sh 一致）
        if scene_index == 0:  # 蓝方
            zone_name = "blue"
            map_path = "maps/official_map_blue"
            prior_x = 0.0
            prior_y = 0.0
        else:  # 红方
            zone_name = "red"
            map_path = "maps/official_map_red"
            prior_x = 0.0
            prior_y = 0.0  # 实测后替换
        odom_topic = "/r2/global_odometry" if mode == "localization" else "/Odometry"

        self.radar_btn.setEnabled(False)
        self.radar_btn.setText("启动中...")
        self.radar_btn.setStyleSheet("background-color: #FFC107; color: black;")

        # 强制刷新 UI，确保按钮变色立即可见
        QApplication.processEvents()

        try:
            def _launch(cmd, tag):
                p = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, bufsize=1)
                self._radar_processes.append(p)
                t = threading.Thread(target=_read_output, args=(p, tag), daemon=True)
                t.start()
                return p

            # (3) Livox 驱动
            _launch("ros2 launch livox_ros_driver2 msg_MID360s_launch.py",
                    tag="livox")
            time.sleep(3)

            # (4) FAST-LIO 前端
            _launch("ros2 launch fast_lio mapping.launch.py "
                    "use_sim_time:=false rviz:=false",
                    tag="fast_lio")

            if mode == "localization" and map_path:
                # (5) SC-QN 全局重定位
                _launch(
                    f"ros2 launch fast_lio_localization_sc_qn_ros2 "
                    f"localization_sc_qn.launch.py "
                    f"use_sim_time:=false map_directory:={map_path} "
                    f"use_position_prior:=true "
                    f"expected_x_mm:={prior_x} expected_y_mm:={prior_y}",
                    tag="sc_qn")

            # (6) simple_odom — 位姿桥梁 + 红蓝坐标镜射
            _launch(
                f"ros2 run fast_lio simple_odom --ros-args "
                f"-p mode:={mode} -p zone:={zone_name} "
                f"-p odom_topic:={odom_topic} -p localized_topic:=/r2/localized",
                tag="simple_odom")

            self._radar_running = True
            self.radar_btn.setText("关闭雷达")
            self.radar_btn.setStyleSheet("background-color: #FF5722; color: white;")
            QMessageBox.information(self, "雷达启动成功",
                                    f"雷达节点已启动 (模式: {mode}, 半场: {zone_name})")

        except Exception as e:
            self.radar_btn.setText("启动雷达")
            self.radar_btn.setStyleSheet("background-color: #2196F3; color: white;")
            QMessageBox.critical(self, "雷达启动失败", f"启动雷达节点失败: {e}")
        finally:
            self.radar_btn.setEnabled(True)

    def _stop_radar(self, silent: bool = False):
        for p in self._radar_processes:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except (ProcessLookupError, OSError):
                pass
        self._radar_processes.clear()
        self._radar_running = False
        self.radar_btn.setText("启动雷达")
        self.radar_btn.setStyleSheet("background-color: #2196F3; color: white;")

        # 清空 lidar 暂存数据，避免残留到下次启动
        main_window = self._main_window
        if main_window is not None:
            ros_node = getattr(main_window, "ros_node", None)
            if ros_node is not None:
                ros_node.clear_lidar_cache()
            main_window.lidar_panel.reset_display()

        if not silent:
            QMessageBox.information(self, "雷达已关闭", "雷达节点已停止")

    def _on_start_command(self):
        ros_node = getattr(self._main_window, "ros_node", None)
        if ros_node is None:
            return
        if not ros_node.has_initial_position:
            QMessageBox.warning(self, "等待中",
                                "尚未收到起点坐标，请等待定位完成后再开始比赛。")
            return
        scene_index = getattr(self._main_window, "scene_index", -1)
        if scene_index < 0:
            QMessageBox.warning(self, "未选择半场",
                                "请先在主窗口选择蓝色或红色场景！")
            return
        zone = self.zone_group.checkedId()
        if zone < 0:
            return
        dialog = CountdownDialog(3, self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            if not ros_node.publish_startup_config(
                    scene_index, zone, self.get_block_count(),
                    self.get_arena_load_kfs(), self.get_arena_delay()):
                QMessageBox.warning(self, "发送失败",
                                    "启动配置发送失败，请确认已收到起点坐标。")
