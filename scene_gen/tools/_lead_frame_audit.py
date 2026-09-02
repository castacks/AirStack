"""Lead round-4 audit: where do interior/fit/backing/tear prims sit vs the
building's own pieces? Run under usd_python.sh with a stage path argument."""
import sys
from pxr import Usd, UsdGeom, Gf

stage = Usd.Stage.Open(sys.argv[1])
bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])

def wbox(prim):
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    return (mn[0], mn[1], mn[2], mx[0], mx[1], mx[2])

# top-2-level hierarchy
print("== hierarchy ==")
for p in stage.GetPseudoRoot().GetChildren():
    print(" /", p.GetName(), p.GetTypeName())
    for c in p.GetChildren():
        print("   ", c.GetName(), c.GetTypeName(), "kids=%d" % len(c.GetChildren()))

roots = {}
pieces_box = None
for prim in stage.Traverse():
    path = prim.GetPath().pathString
    low = path.lower()
    for key in ("interior", "backing", "fit", "tornadotear", "slab", "column", "partition"):
        if key in low:
            # bucket by the first ancestor segment containing the key
            segs = path.split("/")
            idx = next(i for i, s in enumerate(segs) if key in s.lower())
            root = "/".join(segs[: idx + 1])
            roots.setdefault(root, []).append(prim)
            break

# building pieces bbox: the biggest Xform holding many mesh kids named piece-ish
cands = {}
for prim in stage.Traverse():
    if prim.GetTypeName() == "Mesh":
        path = prim.GetPath().pathString
        low = path.lower()
        if any(k in low for k in ("interior", "backing", "fit", "tear", "debris", "berm", "frag")):
            continue
        par = prim.GetParent().GetPath().pathString
        cands.setdefault(par, 0)
        cands[par] += 1
big = sorted(cands.items(), key=lambda kv: -kv[1])[:3]
print("== biggest mesh parents (building pieces candidates) ==")
for par, n in big:
    box = wbox(stage.GetPrimAtPath(par))
    print("  %s n=%d box=%s" % (par, n, ["%.1f" % v for v in box] if box else None))
bx = wbox(stage.GetPrimAtPath(big[0][0]))
print("== interior-ish roots vs building box ==")
for root in sorted(roots):
    prim = stage.GetPrimAtPath(root)
    box = wbox(prim)
    n = len(roots[root])
    if not box:
        print("  %-70s n=%d EMPTY" % (root, n))
        continue
    dx = max(0.0, max(bx[0] - box[3], box[0] - bx[3]))
    dy = max(0.0, max(bx[1] - box[4], box[1] - bx[4]))
    dz_above = box[2] - bx[5]
    print("  %-70s n=%-4d box=[%s] gapXY=(%.1f,%.1f) z0-vs-roof=%.1f" % (
        root, n, ",".join("%.1f" % v for v in box), dx, dy, dz_above))
