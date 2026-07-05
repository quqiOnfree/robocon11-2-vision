"""MCU Debug 消息终端窗口。"""

from PySide6.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QTextEdit,
    QPushButton,
)
from PySide6.QtGui import QFont, QColor, QTextCursor
from PySide6.QtCore import Qt

MAX_LINES = 500


class DebugWidget(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("MCU Debug")
        self.setMinimumSize(600, 400)

        layout = QVBoxLayout(self)

        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)
        self.text_edit.setFont(QFont("Courier New", 10))
        self.text_edit.setStyleSheet(
            f"background-color: {QColor('black').name()}; "
            f"color: {QColor('lime').name()};"
        )

        layout.addWidget(self.text_edit)

        clear_btn = QPushButton("清空")
        clear_btn.setFixedHeight(30)
        clear_btn.clicked.connect(self.text_edit.clear)
        layout.addWidget(clear_btn)

        self._line_count = 0

    def update_debug_msg(self, text: str):
        """追加一行 debug 文本。超过 MAX_LINES 行时裁剪旧行。"""
        self.text_edit.append(text)
        self._line_count += 1

        if self._line_count > MAX_LINES:
            # 删掉文档开头多余的行
            cursor = self.text_edit.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.Start)
            for _ in range(self._line_count - MAX_LINES):
                cursor.movePosition(QTextCursor.MoveOperation.Down,
                                    QTextCursor.MoveMode.KeepAnchor)
            cursor.removeSelectedText()
            self._line_count = MAX_LINES
