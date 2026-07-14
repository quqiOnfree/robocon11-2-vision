"""Color 彩幕显示组件：全屏纯色背景 + 居中反色文字。"""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PySide6.QtGui import QFont, QColor, QPalette


class ColorShowerWidget(QWidget):
    """显示 MCU 下发的纯色背景和文字。

    背景色由 R/G/B 决定，文字居中显示，颜色为反色 (255-R, 255-G, 255-B)。
    文字最多 16 个字符。
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self._bg_color = QColor(0, 0, 0)
        self._fg_color = QColor(255, 255, 255)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self._label = QLabel("")
        self._label.setFont(QFont("Arial", 48, QFont.Bold))
        self._label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._label.setWordWrap(True)
        layout.addWidget(self._label)

        self.setAutoFillBackground(True)
        self._apply_bg_color()
        self._apply_label_color()

    def update_color_shower(self, r: int, g: int, b: int, text: str):
        """更新彩幕显示。"""
        r = max(0, min(255, r))
        g = max(0, min(255, g))
        b = max(0, min(255, b))
        text = text[:16]

        self._bg_color = QColor(r, g, b)
        self._fg_color = QColor(255 - r, 255 - g, 255 - b)
        self._apply_bg_color()
        self._apply_label_color()
        self._label.setText(text if text else "")

    def _apply_bg_color(self):
        """通过 QPalette 设置 widget 背景色，避免 stylesheet 对子控件的副作用。"""
        pal = self.palette()
        pal.setColor(QPalette.Window, self._bg_color)
        self.setPalette(pal)

    def _apply_label_color(self):
        self._label.setStyleSheet(
            f"color: {self._fg_color.name()};"
            f"background-color: transparent;"
        )
