#!/usr/bin/env python3
"""
Install AirStack Foxglove extensions into a Foxglove user-extensions dir.

By default this targets the GCS container's bundled Foxglove app
(/root/.foxglove-studio/extensions), which is the entrypoint that
gcs/docker/gcs-base-docker-compose.yaml runs on container start.

The src/dst paths can be overridden via env vars, which is how the
`airstack osmo:foxglove` wrapper reuses this same script to install the
extensions into the laptop's local Foxglove Desktop app before
port-forwarding the GCS bridge — that way the laptop's Foxglove sees
"Robot Tasks" / "Waypoint Editor" / "Polygon Editor" instead of the
"Unknown panel type: ..." placeholders.

Env vars:
    FOXGLOVE_EXT_SRC    directory containing the extension subdirectories
                        (each with a package.json + dist/extension.js)
    FOXGLOVE_EXT_DST    target user-extensions directory, e.g.
                        ~/.foxglove-studio/extensions on Linux/macOS.
"""

import json
import os
import re
import shutil

src = os.environ.get(
    'FOXGLOVE_EXT_SRC', '/root/AirStack/gcs/foxglove_extensions')
dst = os.path.expanduser(os.environ.get(
    'FOXGLOVE_EXT_DST', '/root/.foxglove-studio/extensions'))
os.makedirs(dst, exist_ok=True)


def _slug(s: str) -> str:
    return re.sub(r'[^a-z0-9-]+', '-', s.lower()).strip('-')


installed = 0
for ext in sorted(os.listdir(src)):
    pkg_path = os.path.join(src, ext, 'package.json')
    if not os.path.exists(pkg_path):
        continue
    pkg = json.load(open(pkg_path))
    name = '{}.{}-{}'.format(_slug(pkg['publisher']), pkg['name'], pkg['version'])
    shutil.copytree(os.path.join(src, ext), os.path.join(dst, name), dirs_exist_ok=True)
    print('Installed Foxglove extension:', name, '->', os.path.join(dst, name))
    installed += 1

if installed == 0:
    print('No Foxglove extensions found under', src)
