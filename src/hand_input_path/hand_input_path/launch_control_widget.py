"""启动控制子窗口：选择区域、模式、启动/关闭雷达、发送比赛命令。"""

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
    QDialog,
    QVBoxLayout,
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


class LaunchControlWidget(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("启动控制")
        self.setMinimumWidth(300)

        layout = QVBoxLayout(self)

        # ── 选择启动区域 ──
        zone_title = QLabel("选择启动区域")
        zone_title.setFont(QFont("Arial", 14, QFont.Bold))
        layout.addWidget(zone_title)

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
            layout.addWidget(radio)
            if value == 0:
                radio.setChecked(True)

        layout.addSpacing(10)

        # ── 启动模式 ──
        mode_title = QLabel("启动模式")
        mode_title.setFont(QFont("Arial", 14, QFont.Bold))
        layout.addWidget(mode_title)

        self.mode_group = QButtonGroup(self)
        loc_radio = QRadioButton("定位模式 (Localization)")
        loc_radio.setFont(QFont("Arial", 14))
        self.mode_group.addButton(loc_radio, 0)
        layout.addWidget(loc_radio)
        loc_radio.setChecked(True)

        odo_radio = QRadioButton("里程计模式 (Odometry)")
        odo_radio.setFont(QFont("Arial", 14))
        self.mode_group.addButton(odo_radio, 1)
        layout.addWidget(odo_radio)

        layout.addSpacing(10)

        # ── 启动/关闭雷达 (单按钮互斥) ──
        self.radar_btn = QPushButton("启动雷达")
        self.radar_btn.setFixedHeight(60)
        self.radar_btn.setFont(QFont("Arial", 14, QFont.Bold))
        self.radar_btn.setStyleSheet("background-color: #2196F3; color: white;")
        self.radar_btn.clicked.connect(self._on_toggle_radar)
        layout.addWidget(self.radar_btn)

        layout.addSpacing(10)

        # ── 设置启动区域并开始比赛 ──
        start_btn = QPushButton("设置启动区域并开始比赛")
        start_btn.setFixedHeight(80)
        start_btn.setFont(QFont("Arial", 16, QFont.Bold))
        start_btn.setStyleSheet(
            "background-color: red; color: white;"
        )
        start_btn.clicked.connect(self._on_start_command)
        layout.addWidget(start_btn)

        self._radar_running = False
        self._radar_processes = []

    def get_mode(self) -> str:
        return "localization" if self.mode_group.checkedId() == 0 else "odometry"

    def _on_toggle_radar(self):
        if self._radar_running:
            self._stop_radar()
        else:
            self._start_radar()

    def _start_radar(self):
        parent = self.parent()
        scene_index = getattr(parent, "scene_index", -1)
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

    def _stop_radar(self):
        for p in self._radar_processes:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except (ProcessLookupError, OSError):
                pass
        self._radar_processes.clear()
        self._radar_running = False
        self.radar_btn.setText("启动雷达")
        self.radar_btn.setStyleSheet("background-color: #2196F3; color: white;")
        QMessageBox.information(self, "雷达已关闭", "雷达节点已停止")

    def _on_start_command(self):
        ros_node = getattr(self.parent(), "ros_node", None)
        if ros_node is None:
            return
        if not ros_node.has_initial_position:
            QMessageBox.warning(self, "等待中",
                                "尚未收到起点坐标，请等待定位完成后再开始比赛。")
            return
        scene_index = getattr(self.parent(), "scene_index", -1)
        if scene_index < 0:
            QMessageBox.warning(self, "未选择半场",
                                "请先在主窗口选择蓝色或红色场景！")
            return
        zone = self.zone_group.checkedId()
        if zone < 0:
            return
        dialog = CountdownDialog(3, self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            if not ros_node.publish_startup_config(scene_index, zone):
                QMessageBox.warning(self, "发送失败",
                                    "启动配置发送失败，请确认已收到起点坐标。")
