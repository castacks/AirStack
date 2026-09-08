#!/usr/bin/env python3
"""Crop the raw sim captures to the paper's panel aspect ratios and export JPEGs.
usage: crop_export.py <isaac_png> <gazebo_png> <out_dir>
 isaac: crop the Kit viewport region (given as fractions) then to aspect 1.6
 gazebo: crop to aspect 1.31 around the center-bottom (keeps drone + pillars)
"""
import sys
from PIL import Image

def crop_aspect(im, aspect, cx=0.5, cy=0.5):
    w, h = im.size
    if w / h > aspect:
        nw, nh = int(h * aspect), h
    else:
        nw, nh = w, int(w / aspect)
    x0 = min(max(int(cx * w - nw / 2), 0), w - nw)
    y0 = min(max(int(cy * h - nh / 2), 0), h - nh)
    return im.crop((x0, y0, x0 + nw, y0 + nh))

isaac, gz, out = sys.argv[1:4]
VIEW = tuple(float(v) for v in (sys.argv[4] if len(sys.argv) > 4 else "0,0,1,1").split(","))
im = Image.open(isaac).convert("RGB")
w, h = im.size
vp = im.crop((int(VIEW[0]*w), int(VIEW[1]*h), int(VIEW[2]*w), int(VIEW[3]*h)))
crop_aspect(vp, 1.6, cy=0.55).save(f"{out}/agent_scene_isaac.jpg", quality=90)
im = Image.open(gz).convert("RGB")
crop_aspect(im, 1.31, cx=0.5, cy=0.5).save(f"{out}/agent_scene_gazebo.jpg", quality=90)
print("exported to", out)
