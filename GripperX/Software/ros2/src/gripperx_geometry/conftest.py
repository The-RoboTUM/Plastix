"""Put the package's src layout on sys.path for in-source pytest runs."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "src"))
