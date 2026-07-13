"""Color 彩幕显示组件：全屏纯色背景 + 居中反色文字。"""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PySide6.QtGui import QFont


class ColorShowerWidget(QWidget):
    """显示 MCU 下发的纯色背景和文字。

    背景色由 R/G/B 决定，文字居中显示，颜色为反色 (255-R, 255-G, 255-B)。
    文字最多 16 个字符。
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self._label = QLabel("")
        self._label.setFont(QFont("Arial", 48, QFont.Bold))
        self._label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._label.setWordWrap(True)
        layout.addWidget(self._label)

        # 初始状态：黑色背景
        self.setAutoFillBackground(True)
        self._apply_colors(0, 0, 0)

    def update_color_shower(self, r: int, g: int, b: int, text: str):
        """更新彩幕显示。

        Args:
            r: 红色分量 (0-255)
            g: 绿色分量 (0-255)
            b: 蓝色分量 (0-255)
            text: 居中显示的文字（最多 16 字符）
        """
        r = max(0, min(255, r))
        g = max(0, min(255, g))
        b = max(0, min(255, b))
        text = text[:16]

        self._apply_colors(r, g, b)
        self._label.setText(text if text else "")

    def _apply_colors(self, r: int, g: int, b: int):
        """设置背景颜色和反色文字。"""
        bg_color = f"rgb({r},{g},{b})"
        inv_r, inv_g, inv_b = 255 - r, 255 - g, 255 - b
        fg_color = f"rgb({inv_r},{inv_g},{inv_b})"

        self.setStyleSheet(f"background-color: {bg_color};")
        self._label.setStyleSheet(
            f"color: {fg_color}; background-color: transparent;"
        )
