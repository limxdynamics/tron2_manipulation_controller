from __future__ import annotations

import sys
from pathlib import Path


TRON2_ARM_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = TRON2_ARM_ROOT.parent

for path in (TRON2_ARM_ROOT,):
    value = str(path)
    if value not in sys.path:
        sys.path.insert(0, value)
