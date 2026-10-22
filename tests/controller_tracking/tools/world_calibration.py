#!/usr/bin/env python3
"""Source-tree entry for the independent fixed map/mocap_world calibration."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from controller_tracking.world_calibration import main

if __name__ == '__main__':
    raise SystemExit(main())
