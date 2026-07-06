"""Debug 消息终端窗口：标签页切换信源 + subprocess 输出 + 自动裁剪。"""

from PySide6.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QTextEdit,
    QPushButton,
    QTabWidget,
    QWidget,
)
from PySide6.QtGui import QFont, QColor, QTextCursor
from PySide6.QtCore import Signal, QObject

MAX_LINES = 500

TABS = [
    ("MCU", 0),
    ("路径规划", 1),
    ("r2_serial", 2),
    ("Livox", 3),
    ("FAST-LIO", 4),
    ("SC-QN", 5),
    ("simple_odom", 6),
]

TAG_TO_TAB = {
    "mcu": 0,
    "path_planning": 1,
    "r2_serial": 2,
    "livox": 3,
    "fast_lio": 4,
    "sc_qn": 5,
    "simple_odom": 6,
}


class _DebugSignal(QObject):
    line_signal = Signal(str, str)  # tag, line


_line_buffer = []  # widget 未打开时缓存

_debug_signal = _DebugSignal()


def emit_debug_line(tag: str, line: str):
    """模块级入口, 供 subprocess reader 线程调用. widget 未打开时自动缓存."""
    _line_buffer.append((tag, line))
    _debug_signal.line_signal.emit(tag, line)


class DebugWidget(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._force_quit = False
        self.setWindowTitle("Debug Panel")
        self.setMinimumSize(650, 450)

        layout = QVBoxLayout(self)

        # ── 标签页 ──
        self.tab_widget = QTabWidget()
        self.tab_widget.setFont(QFont("Arial", 11))

        self._editors = []
        self._line_counts = []
        for name, _ in TABS:
            tab = QWidget()
            tab_layout = QVBoxLayout(tab)
            tab_layout.setContentsMargins(0, 0, 0, 0)

            editor = QTextEdit()
            editor.setReadOnly(True)
            editor.setFont(QFont("Courier New", 9))
            editor.setStyleSheet(
                f"background-color: {QColor('black').name()}; "
                f"color: {QColor('lime').name()};"
            )
            tab_layout.addWidget(editor)
            self.tab_widget.addTab(tab, name)
            self._editors.append(editor)
            self._line_counts.append(0)

        layout.addWidget(self.tab_widget)

        # ── 大关闭按钮 ──
        close_btn = QPushButton("关闭")
        close_btn.setFixedHeight(50)
        close_btn.setFont(QFont("Arial", 16, QFont.Bold))
        close_btn.clicked.connect(self.close)
        layout.addWidget(close_btn)

        # ── 连接信号 ──
        _debug_signal.line_signal.connect(self._on_debug_line)

        # 回放 widget 打开前缓存的 subprocess 输出
        for tag, line in _line_buffer:
            self._on_debug_line(tag, line)

    def closeEvent(self, event):
        if self._force_quit:
            event.accept()
        else:
            event.ignore()
            self.hide()

    def update_debug_msg(self, text: str):
        """MCU debug 消息 (原接口, 追加到 tab 0)."""
        self.append_to_tab(0, text)

    def append_to_tab(self, tab_index: int, text: str):
        """追加文本到指定 tab."""
        if tab_index < 0 or tab_index >= len(self._editors):
            return
        editor = self._editors[tab_index]
        editor.append(text)
        self._line_counts[tab_index] += 1
        self._trim_tab(tab_index)

    def _on_debug_line(self, tag: str, line: str):
        tab_index = TAG_TO_TAB.get(tag)
        if tab_index is not None:
            self.append_to_tab(tab_index, line)

    def _trim_tab(self, tab_index: int):
        if self._line_counts[tab_index] <= MAX_LINES:
            return
        editor = self._editors[tab_index]
        excess = self._line_counts[tab_index] - MAX_LINES
        cursor = editor.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.Start)
        for _ in range(excess):
            cursor.movePosition(QTextCursor.MoveOperation.Down,
                                QTextCursor.MoveMode.KeepAnchor)
        cursor.removeSelectedText()
        self._line_counts[tab_index] = MAX_LINES
