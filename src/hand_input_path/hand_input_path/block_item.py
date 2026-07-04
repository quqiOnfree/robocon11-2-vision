"""方块渲染层：BlockLevel、BlockType 枚举和 BlockItem 图形项。"""

from enum import Enum

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGraphicsRectItem,
    QGraphicsItem,
    QStyleOptionGraphicsItem,
    QWidget,
)
from PySide6.QtGui import QPainter, QColor, QFont, QPen


class BlockLevel(Enum):
    Ground = 0
    Low = 1
    Medium = 2
    High = 3


class BlockType(Enum):
    Empty = 0
    R1_KFS = 1
    R2_KFS = 2
    False_KFS = 3


class BlockItem(QGraphicsRectItem):
    def __init__(self, x, y, width, height, block_level, block_type=BlockType.Empty):
        super().__init__(x, y, width, height)
        self.block_level = block_level
        self.block_type = block_type
        self.is_path = False
        self.setBrush(self.get_color())
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)

    def get_color(self):
        if self.block_level == BlockLevel.Low:
            return QColor("darkgreen")
        elif self.block_level == BlockLevel.Medium:
            return QColor("green")
        else:
            return QColor("yellow")

    def set_type(self, new_type: BlockType):
        self.block_type = new_type
        self.update_color()

    def update_color(self):
        self.setBrush(self.get_color())
        self.update()

    def update_path_status(self, is_path: bool):
        self.is_path = is_path
        self.update()

    def paint(
        self,
        painter: QPainter,
        option: QStyleOptionGraphicsItem,
        widget: QWidget | None = None,
    ):
        # 先绘制矩形背景
        super().paint(painter, option, widget)

        # 如果是路径上的方块，绘制一个半透明的覆盖层
        if self.is_path:
            painter.setBrush(QColor(255, 0, 255))
            rect = self.rect()
            small_rect = rect.adjusted(20, 20, -20, -20)
            painter.drawRect(small_rect)

        # 根据 block_type 绘制不同的标识
        types = [
            ("空", BlockType.Empty, QColor("lightgray"), QColor("black")),
            ("R1KFS", BlockType.R1_KFS, QColor("blue"), QColor("white")),
            ("R2KFS", BlockType.R2_KFS, QColor("red"), QColor("white")),
            ("FalseKFS", BlockType.False_KFS, QColor("darkred"), QColor("white")),
        ]

        if self.block_type == BlockType.Empty:
            rect = self.rect()
            painter.setBrush(types[0][2])  # lightgray
            small_rect = rect.adjusted(30, 30, -30, -30)
            painter.drawRect(small_rect)
            painter.setFont(QFont("Arial", 10))
            painter.setPen(types[0][3])  # black
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, "Empty")
        elif self.block_type == BlockType.R1_KFS:
            rect = self.rect()
            painter.setBrush(types[1][2])  # blue
            small_rect = rect.adjusted(30, 30, -30, -30)
            painter.drawRect(small_rect)
            painter.setFont(QFont("Arial", 10))
            painter.setPen(types[1][3])  # white
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, "R1_KFS")
        elif self.block_type == BlockType.R2_KFS:
            rect = self.rect()
            painter.setBrush(types[2][2])  # red
            small_rect = rect.adjusted(30, 30, -30, -30)
            painter.drawRect(small_rect)
            painter.setFont(QFont("Arial", 10))
            painter.setPen(types[2][3])  # white
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, "R2_KFS")
        elif self.block_type == BlockType.False_KFS:
            rect = self.rect()
            painter.setBrush(types[3][2])  # darkred
            small_rect = rect.adjusted(30, 30, -30, -30)
            painter.drawRect(small_rect)
            painter.setFont(QFont("Arial", 10))
            painter.setPen(types[3][3])  # white
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, "False_KFS")

        if self.isSelected():
            painter.setPen(QPen(QColor("red"), 5))
            painter.setBrush(QColor("red"))
            bounding = self.boundingRect()
            painter.drawPolyline([bounding.topLeft(), bounding.topRight(),
                                  bounding.bottomRight(), bounding.bottomLeft(),
                                  bounding.topLeft()])
