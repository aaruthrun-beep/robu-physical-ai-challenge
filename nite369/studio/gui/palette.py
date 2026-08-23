"""Shared dark palette and style builders for Astra Studio Pro.

Every GUI panel should source its colors from here instead of hardcoding
retro/light hex values, so the whole app renders consistently with the
"dark" theme family from ``themes.ThemeManager``.
"""

# ── Dark palette tokens (match themes.py "dark" theme) ─────────────
# Primary accent: lime/leaf green (robot paint). Secondary accent:
# cyan-blue for axis labels / button borders. Stop stays red, bg dark.
DARK_BG            = "#141414"   # window / panel background
DARK_PANEL         = "#1a1a18"   # card / groupbox background
DARK_INPUT         = "#1a1a18"   # input background
DARK_TEXT          = "#ece4da"   # primary text
DARK_TEXT_DIM      = "#9a9488"   # secondary text
DARK_TEXT_MUTED    = "#6b6658"   # tertiary / muted text
DARK_BORDER        = "#2a2a24"   # default border
DARK_BORDER_SOFT   = "#24241e"   # subtle separators
DARK_BUTTON        = "#23231d"   # button face
DARK_BUTTON_HOVER  = "#31312a"   # button hover
DARK_BUTTON_ACTIVE = "#7CB342"   # button pressed (green)
DARK_ACCENT        = "#7CB342"   # lime/leaf green — primary accent
DARK_ACCENT_HOVER  = "#8bc34a"   # accent hover
DARK_ACCENT2       = "#1E88E5"   # cyan-blue — secondary accent
DARK_ACCENT2_HOVER = "#3a9af5"   # secondary hover
DARK_SLIDER_TRACK  = "#1a1a18"
DARK_SLIDER_HANDLE = "#7CB342"
DARK_SUCCESS       = "#2ECC71"
DARK_SUCCESS_DIM   = "#1f8a4d"
DARK_WARNING       = "#F39C12"
DARK_WARNING_DIM   = "#a86a0a"
DARK_ERROR         = "#E74C3C"
DARK_ERROR_DIM     = "#9e2b22"
DARK_GRID          = "#20242e"
DARK_ROW_ALT       = "#1e1e1a"

# Encoder / status colors
STATUS_OK          = DARK_SUCCESS
STATUS_WARN        = DARK_WARNING
STATUS_ERR         = DARK_ERROR


def lighten(hex_color, amount=30):
    """Brighten (positive) or darken (negative) a #RRGGBB color, clamped 0-255."""
    try:
        h = hex_color.lstrip("#")
        r, g, b = int(h[:2], 16), int(h[2:4], 16), int(h[4:], 16)
        return f"#{max(0, min(255, r + amount)):02x}{max(0, min(255, g + amount)):02x}{max(0, min(255, b + amount)):02x}"
    except Exception:
        return hex_color


def btn_style(bg, text=DARK_TEXT, hover=None, pressed=None, radius=6, padding="6px 16px",
              font_size=12, border="none"):
    """Base stylesheet for a flat button."""
    hover = hover or lighten(bg, 25)
    pressed = pressed or lighten(bg, -20)
    return f"""
        QPushButton {{
            background: {bg}; color: {text}; border: {border};
            border-radius: {radius}px; padding: {padding};
            font-size: {font_size}px; font-weight: bold;
        }}
        QPushButton:hover {{ background: {hover}; }}
        QPushButton:pressed {{ background: {pressed}; padding-top: 2px; padding-left: 2px; }}
        QPushButton:disabled {{
            background: {DARK_BUTTON}; color: {DARK_TEXT_MUTED};
        }}
    """


def accent_btn_style(radius=6, padding="6px 16px", font_size=12):
    return btn_style(DARK_ACCENT, text="#1a1a16", hover=DARK_ACCENT_HOVER,
                     radius=radius, padding=padding, font_size=font_size)


def success_btn_style(radius=6, padding="6px 16px", font_size=12):
    return btn_style(DARK_SUCCESS, text="#1a1a16", hover=lighten(DARK_SUCCESS, 15),
                     radius=radius, padding=padding, font_size=font_size)


def danger_btn_style(radius=6, padding="6px 16px", font_size=12):
    return btn_style(DARK_ERROR, text="#ffffff", hover=lighten(DARK_ERROR, 15),
                     radius=radius, padding=padding, font_size=font_size)


def warning_btn_style(radius=6, padding="6px 16px", font_size=12):
    return btn_style(DARK_WARNING, text="#1a1a16", hover=lighten(DARK_WARNING, 15),
                     radius=radius, padding=padding, font_size=font_size)


def input_style(font_size=12, padding="4px 8px", bg=DARK_INPUT, border=DARK_BORDER,
                text=DARK_TEXT):
    """Stylesheet for QLineEdit / QSpinBox / QDoubleSpinBox / QComboBox."""
    return f"""
        QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
            background: {bg}; color: {text};
            border: 1px solid {border}; border-radius: 3px;
            padding: {padding}; font-size: {font_size}px;
            selection-background-color: {DARK_ACCENT};
            selection-color: #1a1a16;
        }}
        QLineEdit:hover, QSpinBox:hover, QDoubleSpinBox:hover, QComboBox:hover,
        QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{
            border: 1px solid {DARK_ACCENT};
        }}
        QComboBox::drop-down {{ border: none; width: 18px; }}
        QComboBox QAbstractItemView {{
            background: {DARK_PANEL}; color: {text};
            border: 1px solid {border};
            selection-background-color: {DARK_ACCENT};
            selection-color: #1a1a16;
        }}
        QSpinBox::up-button, QDoubleSpinBox::up-button,
        QSpinBox::down-button, QDoubleSpinBox::down-button {{
            background: {DARK_BUTTON}; border: none; width: 14px;
        }}
        QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
        QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {{
            background: {DARK_BUTTON_HOVER};
        }}
    """


def slider_style(accent=DARK_ACCENT, groove=DARK_SLIDER_TRACK, border=DARK_BORDER,
                 handle_color=DARK_SLIDER_HANDLE, height=8, handle=18):
    """Stylesheet for horizontal QSlider."""
    return f"""
        QSlider::groove:horizontal {{
            background: {groove}; height: {height}px; border: 1px solid {border};
            border-radius: {height // 2}px;
        }}
        QSlider::handle:horizontal {{
            background: {handle_color}; width: {handle}px; height: {handle}px;
            margin: -{(handle - height) // 2 + 1}px 0; border-radius: {handle // 2}px;
        }}
        QSlider::handle:horizontal:hover {{ background: {lighten(handle_color, 15)}; }}
        QSlider::sub-page:horizontal {{
            background: {accent}; border-radius: {height // 2}px;
        }}
    """


def groupbox_style(font_size=12, padding="12px 10px 10px 10px"):
    """Stylesheet for QGroupBox with a titled border."""
    return f"""
        QGroupBox {{
            color: {DARK_TEXT}; font-size: {font_size}px; font-weight: bold;
            border: 1px solid {DARK_BORDER}; border-radius: 6px;
            margin-top: 10px; padding: {padding};
            background: {DARK_PANEL};
        }}
        QGroupBox::title {{
            subcontrol-origin: margin; left: 10px;
            padding: 0 6px; color: {DARK_ACCENT};
        }}
    """


def card_style(radius=10, padding="8px"):
    """Stylesheet for a floating 'card' panel — rounded, bordered, airy."""
    return f"""
        QWidget {{
            background: {DARK_PANEL};
            border: 1px solid {DARK_BORDER};
            border-radius: {radius}px;
            padding: {padding};
        }}
    """


def dock_style(title_height=30):
    """Stylesheet for a QDockWidget styled as a floating card with a slim header."""
    return f"""
        QDockWidget {{
            color: {DARK_TEXT};
            font-size: 12px;
            font-weight: bold;
            titlebar-close-icon: none;
            titlebar-normal-icon: none;
        }}
        QDockWidget::title {{
            background: {DARK_BUTTON};
            text-align: left;
            padding: 6px 12px;
            border-top-left-radius: 10px;
            border-top-right-radius: 10px;
            border: 1px solid {DARK_BORDER};
            border-bottom: none;
            font-size: 12px;
            font-weight: bold;
            color: {DARK_ACCENT};
        }}
        QDockWidget > QWidget {{
            background: {DARK_PANEL};
            border: 1px solid {DARK_BORDER};
            border-top: none;
            border-bottom-left-radius: 10px;
            border-bottom-right-radius: 10px;
        }}
        QDockWidget QTabWidget::pane {{
            border: none;
            background: {DARK_PANEL};
        }}
        QDockWidget QTabBar::tab {{
            background: {DARK_BUTTON};
            color: {DARK_TEXT_DIM};
            border: 1px solid {DARK_BORDER};
            border-bottom: none;
            border-top-left-radius: 6px;
            border-top-right-radius: 6px;
            padding: 5px 12px;
            font-size: 12px;
            font-weight: bold;
            margin-right: 2px;
        }}
        QDockWidget QTabBar::tab:selected {{
            background: {DARK_PANEL};
            color: {DARK_ACCENT};
        }}
    """


def apply_shadow(widget, blur=28, offset=0, alpha=120):
    """Give a widget a soft drop shadow so it visually floats above the viewport.

    Works well on docked/frameless panels; safe over GL viewports (no
    translucency required).
    """
    try:
        from PyQt5.QtWidgets import QGraphicsDropShadowEffect
        from PyQt5.QtGui import QColor
        effect = QGraphicsDropShadowEffect(widget)
        effect.setBlurRadius(blur)
        effect.setOffset(offset, offset)
        effect.setColor(QColor(0, 0, 0, alpha))
        widget.setGraphicsEffect(effect)
    except Exception:
        pass


def status_dot(color):
    """Stylesheet for a small colored status dot/label."""
    return f"color: {color}; font-size: 14px; font-weight: bold; border: none; background: transparent;"


def panel_style(border=DARK_BORDER, bg=DARK_PANEL, radius=4):
    """Stylesheet for a bordered panel / frame."""
    return f"background: {bg}; border: 1px solid {border}; border-radius: {radius}px;"
