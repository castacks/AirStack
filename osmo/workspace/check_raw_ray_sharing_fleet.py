#!/usr/bin/env python3
"""Run the raw-ray trial gate inside each real robot container."""
from pathlib import Path
import re
import subprocess
script = Path(__file__).with_name('check_raw_ray_sharing_trial.py').read_text()
names = subprocess.check_output(['docker','ps','--format','{{.Names}}'],text=True).splitlines()
names = sorted(n for n in names if re.search(r'-robot-desktop-\d+$',n))
assert len(names)==3, f'expected 3 robot containers, found {names}'
for name in names:
    index=re.search(r'(\d+)$',name).group(1)
    subprocess.run(['docker','exec','-i','-e',f'ROBOT_NAME=robot_{index}',name,'python3','-'],
                   input=script,text=True,check=True,timeout=90)
