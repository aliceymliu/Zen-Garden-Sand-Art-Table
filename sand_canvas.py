"""
- buttons for all three modes (draw, joystick, coordinate) plus erase button to clear sandbox
- arduino sends ball position and joystick direction as text
- live canvas shows live tracing of ball in sandbox
"""

from collections import deque
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Qt, QPointF, QRectF
from PySide6.QtGui import QPainter, QPen, QColor, QBrush, QFont


# Grid dimensions (matches GRID_MAX in Arduino firmware)
GRID_SIZE = 24

# Maximum number of positions kept in the path history
MAX_TRAIL_POINTS = 2000


class SandCanvas(QWidget):
    # Widget that draws the sandbox and updates the ball position

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)

        self._trail: deque[QPointF] = deque(maxlen=MAX_TRAIL_POINTS)
        self._current_pos: QPointF | None = None

        # Visual settings
        self._sand_color   = QColor("#D4B483")
        self._trail_color  = QColor("#8B6914")
        self._ball_color   = QColor("#404040")
        self._grid_color   = QColor("#C4A473")
        self._border_color = QColor("#7A5C30")

        self._margin = 16

    def update_position(self, x: float, y: float):
        # Called when a new position arrives from the arduino
        pt = QPointF(float(x), float(y))
        self._trail.append(pt)
        self._current_pos = pt
        self.update()

    def clear_trail(self):
        # Called when the erase button is pressed
        self._trail.clear()
        self._current_pos = None
        self.update()

    #### COORDINATE MAPPING ####
    def _sandbox_rect(self) -> QRectF:
        # Calculate the centre square area for the sandbox, leaving margins on all sides
        m           = self._margin
        available_w = self.width()  - 2 * m
        available_h = self.height() - 2 * m
        side        = min(available_w, available_h)
        x           = m + (available_w - side) / 2
        y           = m + (available_h - side) / 2
        return QRectF(x, y, side, side)

    def _grid_to_px(self, pt: QPointF) -> QPointF:
        # Convert from grid coordinates (0-24) to pixel coordinates within the sandbox
        rect = self._sandbox_rect()
        px_x = rect.x() + (pt.x() / GRID_SIZE) * rect.width()
        px_y = rect.y() + ((GRID_SIZE - pt.y()) / GRID_SIZE) * rect.height()
        return QPointF(px_x, px_y)

    #### DRAWING THE CANVAS ####
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self._sandbox_rect()

        # Sand background
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(self._sand_color))
        painter.drawRoundedRect(rect, 6, 6)

        # Grid lines every 4 grid units
        painter.setPen(QPen(self._grid_color, 0.5, Qt.DotLine))
        for i in range(4, GRID_SIZE, 4):
            painter.drawLine(
                self._grid_to_px(QPointF(i, 0)),
                self._grid_to_px(QPointF(i, GRID_SIZE)),
            )
            painter.drawLine(
                self._grid_to_px(QPointF(0, i)),
                self._grid_to_px(QPointF(GRID_SIZE, i)),
            )

        # Border
        painter.setPen(QPen(self._border_color, 2.0))
        painter.setBrush(Qt.NoBrush)
        painter.drawRoundedRect(rect, 6, 6)

        # Path
        if len(self._trail) >= 2:
            trail_list = list(self._trail)
            painter.setPen(QPen(self._trail_color, 1.5, Qt.SolidLine, Qt.RoundCap))
            for i in range(1, len(trail_list)):
                painter.drawLine(
                    self._grid_to_px(trail_list[i - 1]),
                    self._grid_to_px(trail_list[i]),
                )

        # Ball
        if self._current_pos is not None:
            center      = self._grid_to_px(self._current_pos)
            ball_radius = max(4.0, rect.width() / 50)
            painter.setPen(QPen(QColor("#222222"), 1.0))
            painter.setBrush(QBrush(self._ball_color))
            painter.drawEllipse(center, ball_radius, ball_radius)

        # Coordinate label
        if self._current_pos is not None:
            label = f"Grid: ({int(self._current_pos.x())}, {int(self._current_pos.y())})"
            painter.setFont(QFont("Consolas", 9))
            painter.setPen(QPen(QColor("#5A4520")))
            painter.drawText(rect.x(), rect.y() + rect.height() + 14, label)

        painter.end()