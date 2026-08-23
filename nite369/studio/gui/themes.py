"""Theme manager for Astra Studio Pro.

Supports retro (Windows Classic), light, dark, and high-contrast themes with QSS stylesheets
and viewport rendering colors.
"""


class ThemeManager:
    THEMES = {
        "retro": {
            "bg": "#ECE9D8",
            "panel": "#FFFFFF",
            "text": "#000000",
            "text_secondary": "#444444",
            "accent": "#0A246A",
            "accent_hover": "#1A3A8A",
            "border": "#ACA899",
            "input_bg": "#FFFFFF",
            "input_border": "#ACA899",
            "slider_groove": "#D4D0C8",
            "slider_handle": "#0A246A",
            "button_bg": "#ECE9D8",
            "button_hover": "#E0DCC8",
            "button_active": "#0A246A",
            "error": "#CC0000",
            "success": "#008000",
            "warning": "#CC6600",
            "viewport_bg": "#E8E8E8",
            "grid_color": [0.6, 0.6, 0.65],
        },
        "light": {
            "bg": "#f0f0f0",
            "panel": "#ffffff",
            "text": "#333333",
            "text_secondary": "#666666",
            "accent": "#4A9BE8",
            "accent_hover": "#5AABF8",
            "border": "#d0d0d0",
            "input_bg": "#ffffff",
            "input_border": "#cccccc",
            "slider_groove": "#d0d0d0",
            "slider_handle": "#4A9BE8",
            "button_bg": "#e0e0e0",
            "button_hover": "#d0d0d0",
            "button_active": "#4A9BE8",
            "error": "#E74C3C",
            "success": "#2ECC71",
            "warning": "#F39C12",
            "viewport_bg": "#e8e8e8",
            "grid_color": [0.6, 0.6, 0.65],
        },
        "dark": {
            "bg": "#0d0d0d",
            "panel": "#141414",
            "text": "#ece4da",
            "text_secondary": "#998e84",
            "accent": "#7CB342",          # brand green (robot paint)
            "accent2": "#1E88E5",         # brand blue (secondary)
            "accent_hover": "#8bc34a",
            "border": "#2a2520",
            "input_bg": "#1a1a16",
            "input_border": "#2a2520",
            "slider_groove": "#1a1a16",
            "slider_handle": "#7CB342",
            "button_bg": "#252018",
            "button_hover": "#352820",
            "button_active": "#7CB342",
            "error": "#E74C3C",
            "success": "#2ECC71",
            "warning": "#F39C12",
            "viewport_bg": "#0d0d0d",
            "grid_color": [0.2, 0.22, 0.26],
        },
        "high_contrast": {
            "bg": "#000000",
            "panel": "#1a1a1a",
            "text": "#ffffff",
            "text_secondary": "#aaaaaa",
            "accent": "#00ff00",
            "accent_hover": "#33ff33",
            "border": "#444444",
            "input_bg": "#222222",
            "input_border": "#666666",
            "slider_groove": "#333333",
            "slider_handle": "#00ff00",
            "button_bg": "#333333",
            "button_hover": "#444444",
            "button_active": "#00ff00",
            "error": "#ff0000",
            "success": "#00ff00",
            "warning": "#ffff00",
            "viewport_bg": "#000000",
            "grid_color": [0.3, 0.3, 0.3],
        },
    }

    @staticmethod
    def get_theme(name: str) -> dict:
        return ThemeManager.THEMES.get(name, ThemeManager.THEMES["retro"])

    @staticmethod
    def get_stylesheet(name: str) -> str:
        t = ThemeManager.get_theme(name)
        return f"""
            QMainWindow {{
                background-color: {t['bg']};
            }}
            QToolBar {{
                background: {t['panel']};
                border-bottom: 1px solid {t['border']};
                spacing: 4px;
                padding: 2px 4px;
            }}
            QToolBar::separator {{
                background: {t['border']};
                width: 1px;
                margin: 2px 6px;
            }}
            QStatusBar {{
                background: {t['bg']};
                color: {t['text_secondary']};
                font-size: 10px;
                border-top: 1px solid {t['border']};
                padding: 1px 8px;
            }}
            QDockWidget {{
                color: {t['text_secondary']};
                font-size: 10px;
                font-weight: bold;
            }}
            QDockWidget::title {{
                background: {t['bg']};
                padding: 3px 8px;
                border-bottom: 1px solid {t['border']};
                color: {t.get('accent2', t['accent'])};
            }}
            QLabel {{
                color: {t['text_secondary']};
                font-size: 10px;
            }}
            QGroupBox {{
                color: {t['text']};
                font-size: 10px;
                font-weight: bold;
                border: 1px solid {t['border']};
                border-radius: 4px;
                margin-top: 8px;
                padding: 12px 4px 4px 4px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
                color: {t.get('accent2', t['accent'])};
            }}
            QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {{
                background: {t['input_bg']};
                color: {t['text']};
                border: 1px solid {t['input_border']};
                border-radius: 2px;
                padding: 2px 4px;
                font-size: 10px;
            }}
            QComboBox::drop-down {{
                border: none;
            }}
            QComboBox QAbstractItemView {{
                background: {t['input_bg']};
                color: {t['text']};
                selection-background-color: {t['accent']};
                selection-color: white;
            }}
            QPushButton {{
                background: {t['button_bg']};
                color: {t['text']};
                border: 1px solid {t['border']};
                border-radius: 3px;
                padding: 4px 12px;
                font-size: 10px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background: {t['button_hover']};
            }}
            QPushButton:pressed {{
                background: {t['button_active']};
                color: white;
            }}
            QSlider::groove:horizontal {{
                background: {t['slider_groove']};
                height: 4px;
                border-radius: 2px;
            }}
            QSlider::handle:horizontal {{
                background: {t['slider_handle']};
                width: 12px;
                height: 12px;
                margin: -4px 0;
                border-radius: 6px;
            }}
            QSlider::sub-page:horizontal {{
                background: {t['accent']};
                border-radius: 2px;
            }}
            QTextEdit {{
                background: {t['input_bg']};
                color: {t['text']};
                border: 1px solid {t['input_border']};
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 11px;
            }}
            QTreeWidget {{
                background: {t['input_bg']};
                color: {t['text']};
                border: 1px solid {t['border']};
                border-radius: 3px;
                font-size: 11px;
                outline: none;
            }}
            QTreeWidget::item {{
                padding: 4px 2px;
                border-bottom: 1px solid {t['border']};
            }}
            QTreeWidget::item:hover {{
                background: {t['button_hover']};
            }}
            QTreeWidget::item:selected {{
                background: {t['accent']};
                color: white;
            }}
            QListWidget {{
                background: {t['input_bg']};
                color: {t['text']};
                border: 1px solid {t['border']};
                font-size: 11px;
                outline: none;
            }}
            QListWidget::item {{
                padding: 6px 8px;
                border-bottom: 1px solid {t['border']};
            }}
            QListWidget::item:selected {{
                background: {t['accent']};
                color: white;
            }}
            QListWidget::item:hover {{
                background: {t['button_hover']};
            }}
            QTabWidget::pane {{
                border: 1px solid {t['border']};
                background: {t['panel']};
            }}
            QTabBar::tab {{
                background: {t['bg']};
                color: {t['text_secondary']};
                border: 1px solid {t['border']};
                padding: 5px 16px;
                font-size: 10px;
            }}
            QTabBar::tab:selected {{
                background: {t['panel']};
                color: {t['accent']};
                border-bottom: 2px solid {t.get('accent2', t['accent'])};
            }}
            QTabBar::tab:hover {{
                background: {t['button_hover']};
            }}
            QCheckBox {{
                color: {t['text']};
                font-size: 10px;
                spacing: 5px;
            }}
            QCheckBox::indicator {{
                width: 14px;
                height: 14px;
            }}
            QMenu {{
                background: {t['panel']};
                color: {t['text']};
                border: 1px solid {t['border']};
                font-size: 11px;
            }}
            QMenu::item {{
                padding: 5px 20px 5px 12px;
            }}
            QMenu::item:selected {{
                background: {t['accent']};
                color: white;
            }}
            QMenu::separator {{
                background: {t['border']};
                height: 1px;
                margin: 4px 8px;
            }}
            QScrollBar:vertical {{
                background: {t['bg']};
                width: 8px;
                border: none;
            }}
            QScrollBar::handle:vertical {{
                background: {t['border']};
                border-radius: 4px;
                min-height: 20px;
            }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
                height: 0px;
            }}
        """

    @staticmethod
    def get_viewport_colors(name: str) -> dict:
        t = ThemeManager.get_theme(name)
        return {
            "bg": t["viewport_bg"],
            "grid": t["grid_color"],
        }
