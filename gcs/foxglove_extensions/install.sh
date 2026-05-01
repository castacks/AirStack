#!/usr/bin/env python3
import json, os, shutil

src = '/root/AirStack/gcs/foxglove_extensions'
dst = '/root/.foxglove-studio/extensions'
os.makedirs(dst, exist_ok=True)

for ext in os.listdir(src):
    pkg_path = os.path.join(src, ext, 'package.json')
    if not os.path.exists(pkg_path):
        continue
    pkg = json.load(open(pkg_path))
    name = '{}.{}-{}'.format(pkg['publisher'], pkg['name'], pkg['version'])
    shutil.copytree(os.path.join(src, ext), os.path.join(dst, name), dirs_exist_ok=True)
    print('Installed Foxglove extension:', name)
