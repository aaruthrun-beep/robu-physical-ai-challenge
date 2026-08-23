"""LED control panel for Nite 369.

Controls the WS2812B RGB LED strip on Slave 2 (GP23).
Features:
- Color picker with RGB sliders
- Preset colors
- Brightness control
- Individual pixel control
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QSlider, QSpinBox, QFrame, QGroupBox, QColorDialog, QGridLayout,
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QBrush, QPen

from . import palette as P


class ColorPreview(QWidget):
    """Widget that displays the current LED color."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._color = QColor(0, 0, 0)
        self.setMinimumSize(60, 60)
        self.setMaximumSize(60, 60)

    def set_color(self, r: int, g: int, b: int):
        """Set the preview color."""
        self._color = QColor(r, g, b)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setBrush(QBrush(self._color))
        painter.setPen(QPen(QColor(P.DARK_BORDER), 2))
        painter.drawRoundedRect(2, 2, self.width() - 4, self.height() - 4, 8, 8)


class LEDControlPanel(QWidget):
    """LED control panel for Nite 369 robot."""

    led_command = pyqtSignal(str)  # Send command to robot

    # Preset colors
    PRESETS = [
        ("Red", (255, 0, 0)),
        ("Green", (0, 255, 0)),
        ("Blue", (0, 0, 255)),
        ("Yellow", (255, 255, 0)),
        ("Cyan", (0, 255, 255)),
        ("Magenta", (255, 0, 255)),
        ("White", (255, 255, 255)),
        ("Off", (0, 0, 0)),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connection_manager = None
        self._r = 0
        self._g = 0
        self._b = 0
        self._brightness = 32
        self._selected_pixel = 0  # 0 = all
        self._setup_ui()

    def set_connection_manager(self, cm):
        """Set the connection manager."""
        self._connection_manager = cm

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # ── Title ──────────────────────────────────────────────
        title = QLabel("LED Control")
        title.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 15px; font-weight: bold; background: transparent; border: none;")
        layout.addWidget(title)

        # ── Color Preview + RGB Sliders ────────────────────────
        color_group = QGroupBox("Color")
        color_group.setStyleSheet(P.groupbox_style())
        color_layout = QHBoxLayout(color_group)

        # Preview
        self.preview = ColorPreview()
        color_layout.addWidget(self.preview, 0, Qt.AlignCenter)

        # RGB sliders
        sliders_layout = QVBoxLayout()

        for label, attr, max_val in [("R", "r", 255), ("G", "g", 255), ("B", "b", 255)]:
            row = QHBoxLayout()
            lbl = QLabel(label)
            lbl.setFixedWidth(20)
            lbl.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 13px; font-weight: bold; background: transparent; border: none;")
            row.addWidget(lbl)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, max_val)
            slider.setValue(0)
            slider.setMinimumWidth(120)
            slider.setStyleSheet(P.slider_style(height=6, handle=14))
            setattr(self, f"_{attr}_slider", slider)
            row.addWidget(slider, 1)

            value_lbl = QLabel("0")
            value_lbl.setFixedWidth(35)
            value_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value_lbl.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-family: 'Consolas', monospace; background: transparent; border: none;")
            setattr(self, f"_{attr}_label", value_lbl)
            row.addWidget(value_lbl)

            sliders_layout.addLayout(row)

            # Connect slider
            slider.valueChanged.connect(lambda v, a=attr: self._on_slider_changed(a, v))

        color_layout.addLayout(sliders_layout)
        layout.addWidget(color_group)

        # ── Brightness ─────────────────────────────────────────
        brightness_group = QGroupBox("Brightness")
        brightness_group.setStyleSheet(P.groupbox_style())
        brightness_layout = QHBoxLayout(brightness_group)

        self.brightness_slider = QSlider(Qt.Horizontal)
        self.brightness_slider.setRange(0, 255)
        self.brightness_slider.setValue(32)
        self.brightness_slider.setMinimumWidth(120)
        self.brightness_slider.setStyleSheet(P.slider_style(accent=P.DARK_WARNING, height=6, handle=14))
        self.brightness_slider.setToolTip("Brightness 0-255")
        self.brightness_slider.valueChanged.connect(self._on_brightness_changed)
        brightness_layout.addWidget(self.brightness_slider, 1)

        self.brightness_label = QLabel("32")
        self.brightness_label.setFixedWidth(35)
        self.brightness_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.brightness_label.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-family: 'Consolas', monospace; background: transparent; border: none;")
        brightness_layout.addWidget(self.brightness_label)
        layout.addWidget(brightness_group)

        # ── Pixel Select ───────────────────────────────────────
        pixel_group = QGroupBox("Pixel Select")
        pixel_group.setStyleSheet(P.groupbox_style())
        pixel_layout = QHBoxLayout(pixel_group)

        self.pixel_spin = QSpinBox()
        self.pixel_spin.setRange(0, 7)
        self.pixel_spin.setValue(0)
        self.pixel_spin.setPrefix("Pixel ")
        self.pixel_spin.setSpecialValueText("All Pixels")
        self.pixel_spin.setFixedWidth(130)
        self.pixel_spin.setStyleSheet(P.input_style(font_size=12))
        self.pixel_spin.valueChanged.connect(self._on_pixel_changed)
        pixel_layout.addWidget(self.pixel_spin)
        pixel_layout.addStretch()
        layout.addWidget(pixel_group)

        # ── Preset Colors (4×2 grid so buttons don't squeeze) ──
        preset_group = QGroupBox("Preset Colors")
        preset_group.setStyleSheet(P.groupbox_style())
        preset_layout = QGridLayout(preset_group)
        preset_layout.setSpacing(4)

        for i, (name, (r, g, b)) in enumerate(self.PRESETS):
            btn = QPushButton(name)
            btn.setFixedHeight(34)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background: rgb({r}, {g}, {b});
                    color: {'white' if (r + g + b) < 384 else 'black'};
                    border: 1px solid {P.DARK_BORDER}; border-radius: 4px;
                    font-size: 11px; font-weight: bold; padding: 4px 8px;
                }}
                QPushButton:hover {{ border: 1px solid {P.DARK_ACCENT}; }}
            """)
            btn.clicked.connect(lambda checked, r=r, g=g, b=b: self._set_color(r, g, b))
            preset_layout.addWidget(btn, i // 4, i % 4)

        layout.addWidget(preset_group)

        # ── Apply Button ───────────────────────────────────────
        self.apply_btn = QPushButton("Apply Color")
        self.apply_btn.setFixedHeight(38)
        self.apply_btn.setStyleSheet(P.accent_btn_style(font_size=13, padding="8px 24px"))
        self.apply_btn.clicked.connect(self._apply_color)
        layout.addWidget(self.apply_btn)

        # ── Status Bar ─────────────────────────────────────────
        status_frame = QFrame()
        status_frame.setStyleSheet(f"QFrame {{ background: {P.DARK_INPUT}; border: 1px solid {P.DARK_BORDER}; border-radius: 4px; }}")
        status_layout = QHBoxLayout(status_frame)
        status_layout.setContentsMargins(10, 5, 10, 5)

        self.status_led = QLabel("●")
        self.status_led.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 12px; font-weight: bold; background: transparent; border: none;")
        status_layout.addWidget(self.status_led)

        self.status_text = QLabel("LED ready")
        self.status_text.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; border: none; background: transparent;")
        status_layout.addWidget(self.status_text)
        status_layout.addStretch()

        layout.addWidget(status_frame)
        layout.addStretch()

    def _on_slider_changed(self, channel: str, value: int):
        """Handle RGB slider change."""
        setattr(self, f"_{channel}", value)
        getattr(self, f"_{channel}_label").setText(str(value))
        self.preview.set_color(self._r, self._g, self._b)

    def _on_brightness_changed(self, value: int):
        """Handle brightness slider change."""
        self._brightness = value
        self.brightness_label.setText(str(value))

    def _on_pixel_changed(self, value: int):
        """Handle pixel selection change."""
        self._selected_pixel = value

    def _set_color(self, r: int, g: int, b: int):
        """Set color from preset."""
        self._r = r
        self._g = g
        self._b = b

        self._r_slider.setValue(r)
        self._g_slider.setValue(g)
        self._b_slider.setValue(b)

        self._r_label.setText(str(r))
        self._g_label.setText(str(g))
        self._b_label.setText(str(b))

        self.preview.set_color(r, g, b)

    def _apply_color(self):
        """Send LED command to robot."""
        mode = self._selected_pixel
        cmd = f"#LED{self._r},{self._g},{self._b},{mode}"
        self.led_command.emit(cmd)
        target = "all pixels" if mode == 0 else f"pixel {mode}"
        self.status_text.setText(f"LED: RGB({self._r}, {self._g}, {self._b}) → {target}")

    def set_connected(self, connected: bool):
        """Update UI for connection state."""
        if connected:
            self.status_led.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 12px; background: transparent; border: none;")
            self.status_text.setText("Connected")
        else:
            self.status_led.setStyleSheet(f"color: {P.DARK_ERROR}; font-size: 12px; background: transparent; border: none;")
            self.status_text.setText("Disconnected")
