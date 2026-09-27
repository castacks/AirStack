#!/usr/bin/env python3
"""Install the SVG Foxglove panel(s) into Foxglove Studio's extensions dir.

Copies every directory next to this script that has a ``package.json`` into
``~/.foxglove-studio/extensions/<publisher>.<name>-<version>``. Same behaviour
as ``gcs/foxglove_extensions/install.py`` for the general AirStack panels;
this copy lives with the rest of SVG ground control so the package is
self-contained. Run it on the host for a host Studio, and it runs in the robot
container at start-up (see robot/docker/docker-compose.yaml). Restart Studio
afterwards — extensions are loaded only at start-up.
"""
import json
import os
import re
import shutil

src = os.path.dirname(os.path.abspath(__file__))
dst = os.path.expanduser('~/.foxglove-studio/extensions')
os.makedirs(dst, exist_ok=True)


def _slug(s: str) -> str:
    return re.sub(r'[^a-z0-9-]+', '-', s.lower()).strip('-')


for ext in sorted(os.listdir(src)):
    pkg_path = os.path.join(src, ext, 'package.json')
    if not os.path.exists(pkg_path):
        continue
    with open(pkg_path) as f:
        pkg = json.load(f)
    name = '{}.{}-{}'.format(_slug(pkg['publisher']), pkg['name'], pkg['version'])
    shutil.copytree(os.path.join(src, ext), os.path.join(dst, name), dirs_exist_ok=True)
    print('Installed Foxglove extension:', name)
