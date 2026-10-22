"""Package-owned resources and runtime state, independent of the launch directory."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

PACKAGE_SHARE = Path(get_package_share_directory("ail_tb4_maze"))
source_root = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = source_root if (source_root / "package.xml").exists() else PACKAGE_SHARE
if "AIL_TB4_MAZE_RUNTIME_DIR" in os.environ:
    RUNTIME_ROOT = Path(os.environ["AIL_TB4_MAZE_RUNTIME_DIR"]).expanduser().resolve()
elif (source_root / "package.xml").exists():
    RUNTIME_ROOT = source_root / ".runtime"
else:
    RUNTIME_ROOT = (
        Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state")))
        / "ail_tb4_maze"
    )
STATE = RUNTIME_ROOT / "state"
RESULTS = RUNTIME_ROOT / "results"
NAVIGATION_ROOT = PACKAGE_ROOT / "navigation_interface"
