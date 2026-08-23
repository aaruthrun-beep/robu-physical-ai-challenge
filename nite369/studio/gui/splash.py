"""Branded startup splash screen for Nite 369.

A frameless, dark splash with the AVANI logo, Nite 369 title, the Tesla
portrait + quote, and a simple dark background. Shown before the main
window is constructed and closed once it appears.
"""

import os

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPixmap
from PyQt5.QtWidgets import QApplication, QFrame, QLabel, QProgressBar, QHBoxLayout, QVBoxLayout, QWidget

from . import palette as P

APP_TITLE = "Nite 369"
APP_SUBTITLE = "Robotics Simulation · Programming · Control"
APP_VERSION = "1.0.0"

SPLASH_W = 720
SPLASH_H = 480

# Minimum time the splash stays visible so it reads as a deliberate brand moment.
SPLASH_MIN_SECONDS = 3.0


def _assets_dir():
    """Absolute path to the app's assets folder (works from source or frozen)."""
    here = os.path.dirname(os.path.abspath(__file__))
    cand = os.path.join(here, "..", "assets")
    if os.path.isdir(cand):
        return os.path.abspath(cand)
    cand2 = os.path.join(here, "assets")
    if os.path.isdir(cand2):
        return os.path.abspath(cand2)
    return None


def _find_asset(keyword):
    """Return the absolute path of the first asset whose name contains keyword."""
    assets = _assets_dir()
    if not assets:
        return None
    for f in sorted(os.listdir(assets)):
        if keyword.lower() in f.lower():
            return os.path.join(assets, f)
    return None


def _to_black_on_transparent(img):
    """Convert black-on-white line art to black-on-transparent.

    White background pixels become fully transparent; the black line art
    stays black. Result composites cleanly over any background image
    (no black box around the logo/portrait).
    """
    img = img.convertToFormat(QImage.Format_ARGB32)
    w, h = img.width(), img.height()
    for y in range(h):
        for x in range(w):
            c = img.pixelColor(x, y)
            lum = (c.red() * 30 + c.green() * 59 + c.blue() * 11) // 100
            img.setPixelColor(x, y, QColor(0, 0, 0, 255 - lum))
    return img


def load_brand_pixmap(name, max_size=256, invert=False, black_on_transparent=False):
    """Load an asset image by filename (searches assets/).

    The AVANI logo and Tesla portrait are black-on-white line art.
    - invert=True flips to white-on-(black) for dark backgrounds.
    - black_on_transparent=True makes white transparent and keeps black
      lines (best over a bright background image).
    Returns a QPixmap or None.
    """
    path = _find_asset(name)
    if not path:
        return None
    img = QImage(path)
    if img.isNull():
        return None
    # Scale keeping aspect ratio
    img = img.scaled(max_size, max_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
    if black_on_transparent:
        img = _to_black_on_transparent(img)
    elif invert:
        img = img.convertToFormat(QImage.Format_ARGB32)
        img.invertPixels()
    pm = QPixmap.fromImage(img)
    return pm


def _make_logo_pixmap(size=96):
    """Generate the amber 'A' logo mark on a dark rounded tile.

    Falls back to the real AVANI logo image (black lines on transparent,
    for the robot-render background) if it's present in assets/.
    """
    real = load_brand_pixmap("Gemini_Generated_Image", size, black_on_transparent=True)
    if real is not None:
        return real
    pm = QPixmap(size, size)
    pm.fill(Qt.transparent)
    painter = QPainter(pm)
    painter.setRenderHint(QPainter.Antialiasing)

    # Rounded tile
    tile = pm.rect().adjusted(2, 2, -2, -2)
    painter.setBrush(QColor(P.DARK_BUTTON))
    painter.setPen(QColor(P.DARK_BORDER))
    painter.drawRoundedRect(tile, 18, 18)

    # Amber 'A'
    painter.setPen(QColor(P.DARK_ACCENT))
    painter.setFont(QFont("Segoe UI", int(size * 0.55), QFont.Bold))
    painter.drawText(tile, Qt.AlignCenter, "A")
    painter.end()
    return pm


class SplashScreen(QWidget):
    """Frameless branded splash shown during app startup."""

    def __init__(self):
        super().__init__(None, Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.setAttribute(Qt.WA_TranslucentBackground, True)
        self.setFixedSize(SPLASH_W, SPLASH_H)

        # Outer card
        self.setStyleSheet(f"""
            QWidget#splashCard {{
                background: {P.DARK_BG};
                border: 1px solid {P.DARK_BORDER};
                border-radius: 16px;
            }}
        """)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        card = QWidget(self)
        card.setObjectName("splashCard")

        # ── Background image (robot render from assets/, dimmed) ──
        bg_pm = load_brand_pixmap("befor_sholder_gear", SPLASH_W * 2, invert=False)
        bg_label = QLabel(card)
        bg_label.setGeometry(0, 0, SPLASH_W, SPLASH_H)
        bg_label.setScaledContents(True)
        if bg_pm is not None:
            bg_label.setPixmap(bg_pm)
        bg_label.show()
        bg_label.lower()

        # Scrim: dark translucent layer so branding + quote stay readable.
        scrim = QWidget(card)
        scrim.setGeometry(0, 0, SPLASH_W, SPLASH_H)
        scrim.setStyleSheet(
            "background: rgba(10, 12, 16, 170); border-radius: 16px;")
        scrim.show()
        scrim.lower()

        # Content container — raised above background + scrim.
        content = QWidget(card)
        content.setGeometry(0, 0, SPLASH_W, SPLASH_H)
        content.setAttribute(Qt.WA_TranslucentBackground, True)
        card_layout = QVBoxLayout(content)
        card_layout.setContentsMargins(24, 20, 24, 18)
        card_layout.setSpacing(8)

        # Horizontal: branding column + Tesla portrait on the right
        brand_row = QHBoxLayout()
        brand_row.setSpacing(24)

        left_col = QVBoxLayout()
        left_col.setSpacing(6)

        # Logo (left-aligned so it doesn't overlap the robot in the bg)
        logo = QLabel()
        logo.setPixmap(_make_logo_pixmap(200))
        logo.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        left_col.addWidget(logo)

        # Title (Nite 369)
        title = QLabel(APP_TITLE)
        title.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 34px; font-weight: bold; background: transparent; border: none;")
        title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        left_col.addWidget(title)

        # Subtitle
        sub = QLabel(APP_SUBTITLE)
        sub.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; background: transparent; border: none;")
        sub.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        left_col.addWidget(sub)

        # Version
        ver = QLabel(f"v{APP_VERSION}")
        ver.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 10px; background: transparent; border: none;")
        ver.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        left_col.addWidget(ver)

        brand_row.addLayout(left_col, 1)

        # Tesla portrait (black lines on transparent, over the bg image)
        tesla_pm = load_brand_pixmap("images (19)", 300, black_on_transparent=True)
        if tesla_pm is not None:
            tesla = QLabel()
            tesla.setPixmap(tesla_pm)
            tesla.setAlignment(Qt.AlignCenter)
            brand_row.addWidget(tesla)
        else:
            brand_row.addStretch(1)

        card_layout.addLayout(brand_row, 1)

        # ── Robot intro card (name + tagline only) ──
        intro = QFrame()
        intro.setStyleSheet(f"""
            QFrame {{
                background: rgba(20, 26, 38, 180);
                border: 1px solid {P.DARK_BORDER};
                border-radius: 10px;
            }}
        """)
        intro_row = QHBoxLayout(intro)
        intro_row.setContentsMargins(16, 8, 16, 8)
        intro_row.setSpacing(10)

        name_lbl = QLabel("NITE 369")
        name_lbl.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 18px; font-weight: bold; background: transparent; border: none;")
        tag_lbl = QLabel("The Zero-Machining Robotic Arm")
        tag_lbl.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 13px; background: transparent; border: none;")
        intro_row.addWidget(name_lbl)
        intro_row.addWidget(tag_lbl)
        intro_row.addStretch(1)
        card_layout.addWidget(intro)

        # Tesla quote
        quote = QLabel(
            "\u201cBe alone, that is the secret of invention; "
            "be alone, that is when ideas are born.\u201d"
        )
        quote.setStyleSheet(
            f"color: {P.DARK_TEXT_DIM}; font-size: 14px; font-style: italic; "
            f"background: transparent; border: none; padding: 4px 8px;"
        )
        quote.setAlignment(Qt.AlignCenter)
        quote.setWordWrap(True)
        card_layout.addWidget(quote)

        attr = QLabel("\u2014 Nikola Tesla")
        attr.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 11px; font-weight: bold; background: transparent; border: none;")
        attr.setAlignment(Qt.AlignCenter)
        card_layout.addWidget(attr)

        card_layout.addSpacing(4)

        # Progress bar (indeterminate, subtle)
        self._bar = QProgressBar()
        self._bar.setRange(0, 0)
        self._bar.setTextVisible(False)
        self._bar.setFixedHeight(4)
        self._bar.setStyleSheet(f"""
            QProgressBar {{
                background: {P.DARK_BUTTON};
                border: none;
                border-radius: 2px;
            }}
            QProgressBar::chunk {{
                background: {P.DARK_ACCENT};
                border-radius: 2px;
            }}
        """)
        card_layout.addWidget(self._bar)

        content.show()
        content.raise_()
        outer.addWidget(card)

    def close(self):
        """Close the splash."""
        super().close()

def show_splash():
    """Create, center, and show the splash. Returns the SplashScreen instance."""
    app = QApplication.instance()
    splash = SplashScreen()
    # Center on the primary screen
    screen = app.primaryScreen()
    if screen:
        geo = screen.availableGeometry()
        splash.move(geo.center() - splash.rect().center())
    splash.show()
    app.processEvents()
    return splash
