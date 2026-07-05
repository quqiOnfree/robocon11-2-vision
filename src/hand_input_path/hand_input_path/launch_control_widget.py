"""启动控制子窗口，用于设置启动区域和发送开始命令。"""

from PySide6.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QLabel,
    QPushButton,
    QRadioButton,
    QButtonGroup,
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
        self.setMinimumWidth(250)

        layout = QVBoxLayout(self)

        # 标题
        title = QLabel("选择启动区域")
        title.setFont(QFont("Arial", 14))
        layout.addWidget(title)

        # 区域单选按钮组
        self.zone_group = QButtonGroup(self)
        zones = [
            ("1区", 0),
            ("2区", 1),
            ("3区（重试区）", 2),
            ("3区（坡前启动）", 3),
        ]
        for name, value in zones:
            radio = QRadioButton(name)
            radio.setFont(QFont("Arial", 12))
            self.zone_group.addButton(radio, value)
            layout.addWidget(radio)
            if value == 0:
                radio.setChecked(True)

        layout.addSpacing(20)

        # 设置启动区域按钮
        set_zone_btn = QPushButton("设置启动区域")
        set_zone_btn.setFixedHeight(60)
        set_zone_btn.clicked.connect(self._on_set_zone)
        layout.addWidget(set_zone_btn)

        # 开始命令按钮（红色醒目）
        start_btn = QPushButton("开始比赛命令")
        start_btn.setFixedHeight(80)
        start_btn.setStyleSheet(
            "background-color: red; color: white; font-size: 20px; font-weight: bold;"
        )
        start_btn.clicked.connect(self._on_start_command)
        layout.addWidget(start_btn)

    def _on_set_zone(self):
        zone = self.zone_group.checkedId()
        if zone < 0:
            return
        ros_node = getattr(self.parent(), "ros_node", None)
        if ros_node is not None:
            ros_node.publish_set_zone(zone)

    def _on_start_command(self):
        ros_node = getattr(self.parent(), "ros_node", None)
        if ros_node is None:
            return
        dialog = CountdownDialog(3, self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            ros_node.publish_start_command()
