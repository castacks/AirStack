"""Toolchain/device compatibility only; retain all upstream numeric constants."""
from pathlib import Path
import re

root=Path('/opt/DROID-SLAM')
setup=root/'thirdparty/lietorch/setup.py'
source=setup.read_text()
patched,count=re.subn(r"^\s*'-gencode=arch=compute_\d+,code=(?:sm|compute)_\d+',?\s*$",'',source,flags=re.MULTILINE)
if count!=10:
    raise ValueError('unexpected pinned lietorch architecture declarations')
setup.write_text(patched)
geometry=root/'droid_slam/geom/projective_ops.py'
source=geometry.read_text()
old='[-0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], device="cuda"'
if source.count(old)!=1:
    raise ValueError('unexpected pinned stereo device declaration')
geometry.write_text(source.replace(old,old.replace('device="cuda"','device=Gij.data.device')))
