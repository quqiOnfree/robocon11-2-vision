"""手输命令入口：启动 Qt 应用和 ROS2 节点，连接信号槽。"""

import os
import signal
import subprocess
import sys
import threading
import time

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

import rclpy

try:
    from .main_window import MainWindow
    from .ros2_node import Ros2Node
    from .debug_widget import emit_debug_line
except ImportError:
    from main_window import MainWindow
    from ros2_node import Ros2Node
    from debug_widget import emit_debug_line

_lidar_processes = []


def _start_subprocess(cmd: str, tag: str = ""):
    """后台启动子进程, 进程组独立, stdout/stderr 捕获到 debug panel."""
    p = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, bufsize=1)
    _lidar_processes.append(p)
    if tag:
        t = threading.Thread(target=_read_output, args=(p, tag), daemon=True)
        t.start()
    return p


def _read_output(p: subprocess.Popen, tag: str):
    """逐行读取 subprocess 输出, 发送到 debug panel."""
    try:
        for line in iter(p.stdout.readline, ""):
            if line:
                emit_debug_line(tag, line.rstrip("\n"))
    except (ValueError, OSError):
        pass


def _kill_all_subprocesses():
    """向所有子进程组发送 SIGINT."""
    for p in _lidar_processes:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
    _lidar_processes.clear()


def main():
    rclpy.init()
    app = QApplication(sys.argv)

    # ── 非雷达节点: 按照 run_competition.sh 顺序 ──
    # (1) 路径规划
    _start_subprocess("ros2 run path_planning_node path_planning_node",
                      tag="path_planning")
    time.sleep(1)
    # (2) r2_serial 串口收发
    _start_subprocess(
        "ros2 launch r2_serial r2_data_downlink.launch.py "
        "serial_port:=/dev/ttyACM0 serial_debug_raw:=false",
        tag="r2_serial")

    # ── GUI ──
    window = MainWindow()
    node = Ros2Node()
    window.ros_node = node

    # 启动配置默认值, 供 launch_control_widget 使用
    node._startup_mode = "localization"
    node._startup_map_path = ""
    node._startup_prior_x = 0.0
    node._startup_prior_y = 0.0

    # 连接信号槽
    window.emit_grid.connect(node.publish_grid)
    node.path_signal.path_signal.connect(window.update_path)
    window.connect_ros_signals()

    window.show()

    # ROS2 spin timer
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    spin_timer.start(10)

    # 下发节点连接检测 timer（500ms）
    conn_timer = QTimer()
    conn_timer.timeout.connect(node.check_connection)
    conn_timer.start(500)

    def cleanup():
        spin_timer.stop()
        conn_timer.stop()
        _kill_all_subprocesses()
        node.destroy_node()
        rclpy.shutdown()

    app.aboutToQuit.connect(cleanup)
    app.exec()


if __name__ == "__main__":
    main()
