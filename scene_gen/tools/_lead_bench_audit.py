"""Lead round-4: audit a FULL offline-authored bench stage for floaters.
Groups prims by cell holder (/W/bench children), and inside each cell
flags any Mesh subtree hovering: z0 > ground + 2.5 m while horizontally
offset from the cell's main building mass, or z0 > 2.5 m with empty space
below (no geometry within its own footprint below it).

Narrow fractured roof trim is checked separately.  Parapet/cornice shards
can remain inside the building footprint and therefore passed the old XY
test even though they were visibly suspended in open air.
Usage: usd_python.sh _lead_bench_audit.py <stage.usd>"""
import sys
from collections import defaultdict
from pxr import Usd, UsdGeom

stage = Usd.Stage.Open(sys.argv[1])
show_high = "--all-high" in sys.argv[2:]
bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])

def wbox(prim):
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    return [mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]]

root = None
for p in stage.GetPseudoRoot().GetChildren():
    for c in p.GetChildren():
        if c.GetName() in ("bench", "tornado_bench"):
            root = c
if root is None:
    print("no /*/bench root found; children:")
    for p in stage.GetPseudoRoot().GetChildren():
        print(" ", p.GetPath(), [c.GetName() for c in p.GetChildren()])
    sys.exit(1)

for holder in root.GetChildren():
    hbox = wbox(holder)
    if hbox is None:
        continue
    print("\n== %s box=[%s] ==" % (holder.GetName(),
          ",".join("%.1f" % v for v in hbox)))
    # building reference: biggest mesh-count child subtree with z0 <= 0.5
    groups = defaultdict(int)
    for prim in Usd.PrimRange(holder):
        if prim.GetTypeName() == "Mesh" and prim.IsActive():
            segs = prim.GetPath().pathString.split("/")
            hseg = holder.GetPath().pathString.count("/")
            groups["/".join(segs[: hseg + 3])] += 1
    ref_box = None
    ref_path = None
    for gpath, n in sorted(groups.items(), key=lambda kv: -kv[1]):
        b = wbox(stage.GetPrimAtPath(gpath))
        if b and b[2] < 0.6:
            ref_box, ref_path = b, gpath
            break
    print("  building-ref %s box=[%s]" % (ref_path,
          ",".join("%.1f" % v for v in ref_box) if ref_box else "?"))
    flagged = defaultdict(list)
    for gpath in sorted(groups):
        b = wbox(stage.GetPrimAtPath(gpath))
        if not b:
            continue
        z0 = b[2]
        out_xy = 0.0
        if ref_box:
            dx = max(0.0, max(ref_box[0] - b[3], b[0] - ref_box[3]))
            dy = max(0.0, max(ref_box[1] - b[4], b[1] - ref_box[4]))
            out_xy = max(dx, dy)
        low = gpath.lower()
        unsupported_trim = "/brk_" in low and any(
            token in low for token in
            ("parapet", "cornice", "coping", "ledge"))
        if unsupported_trim or (z0 > 2.5 and
                                (out_xy > 1.5 or ref_box is None)):
            flagged[gpath] = (b, out_xy, groups[gpath])
        elif show_high and z0 > 2.5:
            print("  HIGH  %-72s n=%-3d z0=%.1f outXY=%.1f box=[%s]" % (
                gpath, groups[gpath], b[2], out_xy,
                ",".join("%.1f" % v for v in b)))
    if not flagged:
        print("  no floating subtrees flagged")
    for gpath, (b, out_xy, n) in sorted(flagged.items()):
        print("  FLOAT %-72s n=%-3d z0=%.1f outXY=%.1f box=[%s]" % (
            gpath, n, b[2], out_xy, ",".join("%.1f" % v for v in b)))
