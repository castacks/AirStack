#!/usr/bin/env python3
"""Capture an X11 window's own contents (unoccluded) via the Composite
extension's NameWindowPixmap. usage: xcap.py <window-name-substring> <out.png>"""
import sys
from Xlib import display, X
from Xlib.ext import composite
from PIL import Image

name_sub, out = sys.argv[1], sys.argv[2]
d = display.Display(":1")
root = d.screen().root
if not d.has_extension("Composite"):
    sys.exit("no Composite extension")

def find(win):
    try:
        n = win.get_wm_name()
    except Exception:
        n = None
    if n and name_sub in n:
        return win
    for c in win.query_tree().children:
        r = find(c)
        if r:
            return r
    return None

w = find(root)
if w is None:
    sys.exit("window not found")
# capture the top-level frame ancestor of the client window (Mutter redirects
# top-levels); walk up until parent is root
p = w
while True:
    q = p.query_tree()
    if q.parent.id == root.id:
        break
    p = q.parent
geom = p.get_geometry()
d.composite_redirect_window(p, composite.RedirectAutomatic) if False else None
pix = p.composite_name_window_pixmap()
img = pix.get_image(0, 0, geom.width, geom.height, X.ZPixmap, 0xffffffff)
im = Image.frombytes("RGBX", (geom.width, geom.height), img.data, "raw", "BGRX")
im.convert("RGB").save(out)
print("saved", out, geom.width, geom.height)
