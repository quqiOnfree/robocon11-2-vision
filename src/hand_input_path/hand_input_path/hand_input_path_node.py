"""手输命令入口：启动 Qt 应用和 ROS2 节点，连接信号槽。"""

import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

import rclpy

try:
    from .main_window import MainWindow
    from .ros2_node import Ros2Node
except ImportError:
    from main_window import MainWindow
    from ros2_node import Ros2Node


def main():
    rclpy.init()
    app = QApplication(sys.argv)

    window = MainWindow()
    node = Ros2Node()
    window.ros_node = node

    # 连接信号槽
    window.emit_grid.connect(node.publish_grid)
    node.path_signal.path_signal.connect(window.update_path)
    node.path_signal.scene_signal.connect(window.change_scene)

    window.show()

    # ROS2 spin timer
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    spin_timer.start(10)  # 每 10 毫秒检查一次 ROS2 事件

    # 下发节点连接检测 timer（500ms）
    conn_timer = QTimer()
    conn_timer.timeout.connect(node.check_connection)
    conn_timer.start(500)

    def cleanup():
        spin_timer.stop()
        conn_timer.stop()
        node.destroy_node()
        rclpy.shutdown()

    app.aboutToQuit.connect(cleanup)
    app.exec()


if __name__ == "__main__":
    main()
