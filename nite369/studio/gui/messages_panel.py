"""Messages/log panel for Astra Studio Pro.

Displays timestamped, color-coded messages from the application.
"""

from datetime import datetime
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QTextEdit
from PyQt5.QtGui import QColor, QTextCharFormat, QFont
from PyQt5.QtCore import Qt

from . import palette as P


class MessagesPanel(QWidget):
    """Scrollable log panel with color-coded messages."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 6)
        layout.setSpacing(4)

        header = QHBoxLayout()
        title = QLabel("Activity Log")
        title.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-weight: bold; background: transparent; border: none;")
        header.addWidget(title)
        header.addStretch()

        self.clear_btn = QPushButton("Clear")
        self.clear_btn.setFixedHeight(22)
        self.clear_btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=9, padding="1px 8px"))
        self.clear_btn.clicked.connect(self.clear)
        header.addWidget(self.clear_btn)
        layout.addLayout(header)

        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)
        self.text_edit.setStyleSheet(f"""
            QTextEdit {{
                background: {P.DARK_INPUT}; color: {P.DARK_TEXT};
                border: 1px solid {P.DARK_BORDER}; border-radius: 3px;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 11px;
            }}
        """)
        layout.addWidget(self.text_edit, 1)

    def log(self, message, level="info"):
        ts = datetime.now().strftime("%H:%M:%S")
        colors = {
            "info": P.DARK_TEXT_DIM,
            "success": P.DARK_SUCCESS,
            "error": P.DARK_ERROR,
            "warning": P.DARK_WARNING,
            "command": P.DARK_ACCENT,
        }
        color = colors.get(level, P.DARK_TEXT_DIM)
        prefix_colors = {
            "info": P.DARK_TEXT_MUTED,
            "success": P.DARK_SUCCESS_DIM,
            "error": P.DARK_ERROR_DIM,
            "warning": P.DARK_WARNING_DIM,
            "command": P.DARK_ACCENT,
        }
        prefix_color = prefix_colors.get(level, P.DARK_TEXT_MUTED)

        cursor = self.text_edit.textCursor()
        cursor.movePosition(cursor.End)

        ts_format = QTextCharFormat()
        ts_format.setForeground(QColor(prefix_color))
        ts_format.setFontFamily("Consolas")
        ts_format.setFontPointSize(9)
        cursor.insertText(f"[{ts}] ", ts_format)

        msg_format = QTextCharFormat()
        msg_format.setForeground(QColor(color))
        msg_format.setFontFamily("Consolas")
        msg_format.setFontPointSize(9)
        cursor.insertText(f"{message}\n", msg_format)

        self.text_edit.setTextCursor(cursor)
        self.text_edit.ensureCursorVisible()

    def clear(self):
        self.text_edit.clear()

    def info(self, message):
        self.log(message, "info")

    def success(self, message):
        self.log(message, "success")

    def error(self, message):
        self.log(message, "error")

    def warning(self, message):
        self.log(message, "warning")

    def command(self, message):
        self.log(message, "command")
