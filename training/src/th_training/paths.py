"""Where the robot models live. th_description is the single source of truth (see training/README.md)."""
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
ROBOTS_DIR = REPO_ROOT / "src" / "th_description" / "robots"

WALKER_V2_DIR = ROBOTS_DIR / "walker_v2"
WALKER_V2_XML = WALKER_V2_DIR / "mjcf" / "walker_v2.xml"
WALKER_V2_SCENE_XML = WALKER_V2_DIR / "mjcf" / "scene.xml"

WALKER_DEMO_DIR = ROBOTS_DIR / "walker_demo"
WALKER_DEMO_XML = WALKER_DEMO_DIR / "mjcf" / "walker_demo.xml"
WALKER_DEMO_SCENE_XML = WALKER_DEMO_DIR / "mjcf" / "scene.xml"
