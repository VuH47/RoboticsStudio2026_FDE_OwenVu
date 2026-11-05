from pathlib import Path


# paths.py is always:
# fde_gui/operator_interface/paths.py

PACKAGE_ROOT = Path(__file__).resolve().parent.parent

ASSETS_DIR = PACKAGE_ROOT / "assets" / "images"
STYLES_DIR = PACKAGE_ROOT / "operator_interface" / "styles"
THEME_PATH = STYLES_DIR / "theme.qss"