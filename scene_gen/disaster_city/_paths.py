"""Where the code is vs where the data is.

The code lives here (git). The data -- the Blosm blend, the drone videos, the
COLMAP work dirs, the built USDs -- is tens of GB and lives outside git, in
`$DISASTER_CITY_DATA` or, if unset, `data/` next to this file (gitignored; make
it a symlink to wherever the data actually is).
"""
import os
from pathlib import Path

CODE = Path(__file__).resolve().parent
DATA = Path(os.environ.get("DISASTER_CITY_DATA", CODE / "data")).expanduser()
R = DATA / "recon"                       # every built artifact; package.py ships this tree
LABELS = CODE / "LABELS.yaml"            # feature names + positions, versioned with the code
SPECS = CODE / "specs"                   # hand-written hero specs (generated ones go to R/<id>/)
