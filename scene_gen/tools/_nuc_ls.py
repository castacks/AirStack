"""_nuc_ls.py — bare-pxr Nucleus listing (no Kit). Env: NUC_URL, NUC_DEPTH."""
import os, sys
import omni.client as oc
url = os.environ.get("NUC_URL", "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/standalone/buildings/intact/")
depth = int(os.environ.get("NUC_DEPTH", "2"))
SKIP = ("textures", "texture", "materials", "material", "maps", ".thumbs", "sourceimages", "images", "tex")
def walk(u, d):
    r, ents = oc.list(u)
    if r != oc.Result.OK:
        print("!!", u, r); return
    for e in sorted(ents, key=lambda e: e.relative_path):
        n = e.relative_path
        if e.flags & oc.ItemFlags.CAN_HAVE_CHILDREN:
            print("D", u + n + "/")
            if d < depth and n.lower() not in SKIP:
                walk(u + n + "/", d + 1)
        else:
            print("F", u + n, getattr(e, "size", 0))
walk(url, 1)
