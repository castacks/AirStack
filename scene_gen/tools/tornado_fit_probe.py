"""tornado_fit_probe — does anything the tornado fit-out authors stick out
through the building it is inside?

ROUND 4 v7, the lead review's own words: *"There's still issues with the
roof, the floor is extending outside the side wall [A4]. What are these
random slabs you've made inside? ... Check the fire urban setting so that
it looks self contained in the building."*

WHAT IT MEASURES, and why this way. For every cell of an authored bench
stage it splits the prims into the BUILDING (everything that is not
fit-out, backing, debris or ground) and the FIT-OUT (`fit_<tag>/slab_*`,
`col_*`, `prop_*`, plus `tornado_interior_backing/*`), then, for each
fit-out prim, compares its world XY box against the building's OWN world XY
box RESTRICTED TO THE SAME Z BAND. A whole-building bbox is the wrong
reference on exactly the asset that showed the defect — `SM_Building_24`
steps in above its podium, so a slab that is inside the podium's plan is
still outside the curtain wall two storeys up. Banding by z is what makes
"outside the side wall" measurable.

    docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \\
      bash scene_gen/tools/usd_python.sh scene_gen/tools/tornado_fit_probe.py \\
      /isaac-sim/.cache/tornado_probe/bench_offline.usd"

Author that stage first with `scene_gen/tools/_lead_bench_offline.py` (the
headless bench author — no Kit, no SimulationApp). Exit status is 1 when
any fit prim overhangs by more than `TOL_M`, so this doubles as a gate.
"""
import sys
from collections import defaultdict

from pxr import Usd, UsdGeom

#: A fit prim may reach this far past the building's own XY box in its own
#: z band before it counts as an overhang.
TOL_M = 0.35
#: A fit prim is compared against every building mesh whose top is at or
#: above the prim's own base, less this slack — see the long note at the
#: comparison itself for the two references this replaced and why.
BASE_SLACK_M = 0.50

FIT_TOKENS = ("/fit_", "/tornado_interior_backing/")
SKIP_TOKENS = ("/tornado_debris", "/tornado_ground", "/Looks", "/TornadoDebrisLooks",
               "/TornadoFitLooks", "/QuakeLooks", "/tornado_roof", "/furniture")


def wbox(bc, prim):
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    return [mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]]


def kind_of(path):
    s = str(path)
    if any(t in s for t in FIT_TOKENS):
        if "/tornado_interior_backing/" in s:
            return "backing"
        name = s.rsplit("/", 1)[-1]
        for k in ("slab", "col", "prop", "part"):
            if name.startswith(k):
                return k
        return "fit"
    if any(t in s for t in SKIP_TOKENS):
        return None
    return "building"


def main(stage_path):
    stage = Usd.Stage.Open(stage_path)
    if stage is None:
        print("cannot open", stage_path)
        return 2
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    root = None
    for p in stage.GetPseudoRoot().GetChildren():
        for c in p.GetChildren():
            if c.GetName() in ("bench", "tornado_bench"):
                root = c
    if root is None:
        print("no bench root in", stage_path)
        return 2

    worst = 0.0
    n_bad = 0
    for holder in sorted(root.GetChildren(), key=lambda c: c.GetName()):
        building, fit = [], []
        for prim in Usd.PrimRange(holder):
            if not prim.IsA(UsdGeom.Mesh) and not prim.GetTypeName() == "Mesh":
                # a referenced prop is an Xform whose subtree carries the
                # meshes -- take the top of a `prop_*` subtree directly
                if kind_of(prim.GetPath()) != "prop":
                    continue
            k = kind_of(prim.GetPath())
            if k is None:
                continue
            b = wbox(bc, prim)
            if b is None:
                continue
            (building if k == "building" else fit).append((k, prim, b))
            if k == "prop":
                continue
        if not building or not fit:
            continue
        per_kind = defaultdict(lambda: [0, 0.0])
        rows = []
        for k, prim, b in fit:
            # EVERYTHING OF THE BUILDING AT OR ABOVE THIS PRIM'S OWN BASE.
            #
            # Two references were tried and rejected first, both measured:
            # a SYMMETRIC band around the prim reads 0.00 m everywhere
            # (a floor slab sits exactly on the storey boundary, so the
            # band always catches the wider storey BELOW a setback and its
            # bbox swallows the overhang); a band strictly ABOVE the prim
            # over-reports at a DAMAGED top storey, where the wall and the
            # coping are legitimately gone and the exposed floor plate has
            # nothing left to be "inside" (measured: B3 walkup T4 storey 6,
            # 0.45 m slab / 1.00 m backing flagged identically before AND
            # after the fix — i.e. not the fix's doing).
            #
            # The union of everything at or above the base keeps the
            # setback test sharp (above a step-in, ALL the geometry is the
            # narrow plan) while a damaged storey still has its roof and
            # its other three elevations in the reference.
            zfloor = b[2] - 0.50
            xs0 = xs1 = ys0 = ys1 = None
            for _bk, _bp, bb in building:
                if bb[5] < zfloor:
                    continue
                xs0 = bb[0] if xs0 is None else min(xs0, bb[0])
                ys0 = bb[1] if ys0 is None else min(ys0, bb[1])
                xs1 = bb[3] if xs1 is None else max(xs1, bb[3])
                ys1 = bb[4] if ys1 is None else max(ys1, bb[4])
            if xs0 is None:
                continue                      # nothing of the building here
            over = max(xs0 - b[0], b[3] - xs1, ys0 - b[1], b[4] - ys1)
            over = max(0.0, over)
            per_kind[k][0] += 1
            per_kind[k][1] = max(per_kind[k][1], over)
            if over > TOL_M:
                rows.append((over, str(prim.GetPath())))
        if not per_kind:
            continue
        print("\n== {0} ==".format(holder.GetName()))
        for k in sorted(per_kind):
            n, mx = per_kind[k]
            flag = "  <-- OVERHANG" if mx > TOL_M else ""
            print("   {0:8s} n={1:3d}  worst overhang {2:6.2f} m{3}".format(
                k, n, mx, flag))
            worst = max(worst, mx)
        for over, path in sorted(rows, reverse=True)[:6]:
            n_bad += 1
            print("     {0:6.2f} m  {1}".format(over, path))
    print("\nWORST OVERHANG {0:.2f} m over every cell (tol {1:.2f} m); "
          "{2} prim(s) flagged".format(worst, TOL_M, n_bad))
    return 1 if worst > TOL_M else 0


def slabs(stage_path):
    """`--slabs`: every fit slab's authored world XY size per cell. The
    column/slab SIZES are the direct evidence that the per-storey clamp
    bit, independent of any reference box: run it on a stage authored with
    `TU_FIT_CLAMP=0` and one without, and diff."""
    stage = Usd.Stage.Open(stage_path)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    root = None
    for p in stage.GetPseudoRoot().GetChildren():
        for c in p.GetChildren():
            if c.GetName() in ("bench", "tornado_bench"):
                root = c
    for holder in sorted(root.GetChildren(), key=lambda c: c.GetName()):
        rows = []
        ncol = 0
        for prim in Usd.PrimRange(holder):
            n = prim.GetName()
            if n.startswith("col_"):
                ncol += 1
            if not n.startswith("slab_"):
                continue
            b = wbox(bc, prim)
            if b:
                rows.append((n, b[3] - b[0], b[4] - b[1]))
        if rows:
            print("{0}: {1} col(s); {2}".format(
                holder.GetName(), ncol,
                "  ".join("%s %.2fx%.2f" % r for r in sorted(rows))))


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--slabs":
        slabs(sys.argv[2])
        raise SystemExit(0)
    raise SystemExit(main(sys.argv[1] if len(sys.argv) > 1
                          else "/isaac-sim/.cache/tornado_probe/bench_offline.usd"))
