#!/usr/bin/env python3
"""Invoke the trial check in each real container, with its own /tmp ledger."""
from pathlib import Path
import re
import subprocess

script = Path(__file__).with_name('check_map_sharing_trial.py').read_text()
names = subprocess.check_output(['docker', 'ps', '--format', '{{.Names}}'], text=True).splitlines()
names = sorted(n for n in names if re.search(r'-robot-desktop-\d+$', n))
assert names, 'no robot containers'
for name in names:
    index = re.search(r'(\d+)$', name).group(1)
    subprocess.run(['docker', 'exec', '-i', '-e', f'ROBOT_NAME=robot_{index}',
                    name, 'python3', '-'], input=script, text=True, check=True, timeout=90)
