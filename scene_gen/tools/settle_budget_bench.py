#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["usd-core", "numpy", "bpy", "pillow"]
# ///
"""settle_budget_bench.py — the LOOK-preserved half of SETTLE_BODY_BUDGET's
acceptance test (round 8).

    uv run --python 3.13 --with usd-core --with numpy --with bpy --with pillow \
        python scene_gen/tools/settle_budget_bench.py \
        --n 6000 --budget 3000 --out ~/scorch_previews/settle_budget/

WHY THIS EXISTS. `disaster.quake_collapse.apply_settle_budget` caps how many
of a style row's loose pieces get a PhysX rigid body — `block_residential`
DG3-5 measured 18,771 loose bodies and a settle that ran over 1.5 h before
being killed; the fix keeps physics for the `SETTLE_BODY_BUDGET` (default
3000) most visually-important pieces and places everything past that
GEOMETRICALLY instead (seated, laid flat, sunk a few cm — never simulated).
The unit tests (`tests/test_settle_budget.py`) prove the mechanism is
correct; they cannot prove the RESULT still looks like a rubble pile at
review distance. This renders one, twice, and puts the two PNGs side by
side.

OFFLINE ONLY — NO ISAAC SIM, matching this round's own constraint (no Isaac,
no pod, no SSH). That rules out ever actually running `settle.run` (PhysX)
here, so this bench cannot show "physics settled it" vs "physics did not
touch it" — instead it authors a pile that ALREADY looks like a converged
settle produced it (every piece resting on the dome's own surface, laid
flat, a small tilt baked in — the exact resting pose `quake_flow._a_lay_flat`
already tumbles a real fragment into), which is the fairest offline stand-in
for "what the settle would have produced" without a PhysX scene to prove it.
On that pile:

  FULL     every piece, untouched — `apply_settle_budget(..., budget=None)`,
           a documented no-op.
  BUDGETED `apply_settle_budget(..., budget=SETTLE_BODY_BUDGET)` — the top
           N (by volume x current height, the same "z-and-size" importance
           rank the real launcher uses) are left exactly as authored (the
           ones that would keep physics in production); everything past N
           is RE-SEATED right here (support under its own footprint via
           `quake_collapse._deck_support_z`, laid flat via `quake_flow.
           _a_lay_flat`, sunk a few cm) — the same code path the real
           archetype bake calls, just exercised at heap scale.

If the BUDGETED render reads the same as FULL at a normal review distance,
capping thousands of bodies down to `SETTLE_BODY_BUDGET` did not cost the
scene anything visible.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import sys
import time
from pathlib import Path

_HERE = os.path.dirname(os.path.abspath(__file__))          # scene_gen/tools
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))     # scene_gen/
sys.path.insert(0, _HERE)
sys.path.insert(0, _SCENE_GEN)

from disaster import quake_collapse as qc                    # noqa: E402
from disaster import quake_flow as qf                        # noqa: E402
import rubble_preview as rp                                   # noqa: E402


def _dome_height(x, y, R, crown):
    r = math.sqrt(x * x + y * y) / R
    if r >= 1.0:
        return 0.0
    return crown * math.sqrt(max(0.0, 1.0 - r * r))


def build_pile(stage, parent, n, seed, R=14.0, crown=7.0):
    """Author `n` debris pieces resting FLAT on a dome-shaped heap silhouette
    — a procedural stand-in for "what a converged PhysX settle already
    looks like" (see the module docstring: no Isaac Sim here at all). Every
    piece is a plain `quake_flow._box` mesh (never a `PointInstancer`
    instance) authored ALREADY thin-in-Z with a small baked-in tilt, roughly
    matching the size-with-height gradient a real pile has (bigger pieces
    near the crown, smaller toward the toe). Returns the list of loose
    paths, ranked-importance order irrelevant — `apply_settle_budget` does
    its own ranking.
    """
    from pxr import Gf
    rng = random.Random(seed)
    loose = []
    for i in range(n):
        ang = rng.uniform(0, 2 * math.pi)
        rad = R * math.sqrt(rng.random())          # uniform over the disc
        x, y = rad * math.cos(ang), rad * math.sin(ang)
        h = _dome_height(x, y, R, crown)
        size_bias = 1.0 - 0.55 * (rad / R)          # bigger near the crown
        footprint = rng.uniform(0.35, 1.9) * max(0.35, size_bias)
        thick = rng.uniform(0.06, 0.20)
        path = "{0}/frag_{1}".format(parent, i)
        qf._box(stage, path, x, y, h + thick * 0.5,
               footprint, footprint * rng.uniform(0.5, 1.0), thick,
               yaw_deg=rng.uniform(0, 360))
        # a small baked-in tilt (3-20 deg about a random horizontal axis) —
        # the SAME shape of imperfection `_a_lay_flat` gives a real fragment,
        # so a piece never resting perfectly level does not, on its own,
        # read as "the budget did something to it."
        from pxr import UsdGeom
        pr = stage.GetPrimAtPath(path)
        M = qf._rot_about((x, y, h), (math.cos(rng.uniform(0, math.pi)),
                                     math.sin(rng.uniform(0, math.pi)), 0.0),
                          rng.uniform(3.0, 20.0))
        qf._transform_prims(stage, [path], M)
        loose.append(path)
    return loose


def _ground_and_frame(stage, parent, R):
    from pxr import Gf, Sdf, UsdGeom, Vt
    from disaster import damage
    GROUND_Z = -0.03
    hx = hy = R * 2.2
    gpath = "{0}/ground".format(parent)
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(gpath))
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(-hx, -hy, GROUND_Z), Gf.Vec3f(hx, -hy, GROUND_Z),
                                       Gf.Vec3f(hx, hy, GROUND_Z), Gf.Vec3f(-hx, hy, GROUND_Z)]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * 4))
    m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateExtentAttr([Gf.Vec3f(-hx, -hy, GROUND_Z), Gf.Vec3f(hx, hy, GROUND_Z)])
    rgb = (0.42, 0.42, 0.41)
    from disaster import quake_rubble_usd as qru
    from pxr import UsdShade
    gmat = damage._pbr(stage, "{0}/Looks/ground_grey".format(parent), rgb, 0.96)
    qru._add_preview_fallback(stage, "{0}/Looks/ground_grey".format(parent), rgb, 0.96)
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(gmat)


def _bind_debris_material(stage, parent, paths):
    from disaster import damage
    from disaster import quake_rubble_usd as qru
    from pxr import UsdShade
    rgb = (0.55, 0.50, 0.46)
    mat = damage._pbr(stage, "{0}/Looks/debris".format(parent), rgb, 0.85)
    qru._add_preview_fallback(stage, "{0}/Looks/debris".format(parent), rgb, 0.85)
    api = UsdShade.MaterialBindingAPI
    for p in paths:
        pr = stage.GetPrimAtPath(p)
        if pr and pr.IsValid():
            api.Apply(pr).Bind(mat)


def make_variant(n, seed, budget, R, crown, ground_z_fallback=0.0):
    """Builds one fresh pile and (optionally) applies the settle budget.
    Returns (stage, parent, loose_paths_authored, kept, geometric, report)."""
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    parent = "/World/Pile"
    stage.DefinePrim(parent, "Xform")
    authored = build_pile(stage, parent, n, seed, R=R, crown=crown)
    _ground_and_frame(stage, parent, R)

    kept, geometric, report = authored, [], []
    if budget is not None:
        kept, geometric, report = qc.apply_settle_budget(
            stage, authored, budget, root=parent,
            ground_z=ground_z_fallback, rng=random.Random(SEED_BUDGET))
    _bind_debris_material(stage, parent, authored)
    return stage, parent, authored, kept, geometric, report


SEED_BUDGET = 12345


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=6000,
                    help="pieces in the pile — heavier than apartment_tall's "
                         "measured 3,144-body DG3-5 row, lighter than "
                         "block_residential's killed 18,771 (default 6000)")
    ap.add_argument("--budget", type=int, default=None,
                    help="default: quake_collapse.SETTLE_BODY_BUDGET_DEFAULT")
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--R", type=float, default=14.0, help="pile footprint radius (m)")
    ap.add_argument("--crown", type=float, default=7.0, help="pile crown height (m)")
    ap.add_argument("--out", default="~/scorch_previews/settle_budget/")
    ap.add_argument("--cpu", action="store_true")
    ap.add_argument("--samples", type=int, default=64)
    ap.add_argument("--rw", type=int, default=1280)
    ap.add_argument("--rh", type=int, default=720)
    args = ap.parse_args()

    budget = args.budget if args.budget is not None else qc.SETTLE_BODY_BUDGET_DEFAULT
    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    print("[sbb] building FULL pile (n={0}, budget=None -> no-op)".format(args.n))
    stage_full, parent, authored_full, kept_full, geo_full, _ = make_variant(
        args.n, args.seed, None, args.R, args.crown)
    assert kept_full == authored_full and geo_full == []
    usd_full = out_dir / "full.usda"
    stage_full.GetRootLayer().Export(str(usd_full))
    print("[sbb] FULL: {0} pieces, all loose (physics-eligible in "
          "production), 0 geometric -> {1}".format(len(authored_full), usd_full))

    print("[sbb] building BUDGETED pile (n={0}, budget={1})".format(args.n, budget))
    stage_bud, parent, authored_bud, kept_bud, geo_bud, report_bud = make_variant(
        args.n, args.seed, budget, args.R, args.crown)
    usd_bud = out_dir / "budgeted.usda"
    stage_bud.GetRootLayer().Export(str(usd_bud))
    print("[sbb] BUDGETED: {0} authored -> {1} kept for physics (budget {2}) "
          "+ {3} placed geometrically -> {4}".format(
              len(authored_bud), len(kept_bud), budget, len(geo_bud), usd_bud))

    # ------------------------------------------------------------------ #
    # render both, same camera rig, reusing `rubble_preview`'s own bpy
    # helpers (the idiom this round's brief asks for) rather than a new one
    # ------------------------------------------------------------------ #
    pile_center = (0.0, 0.0, args.crown * 0.35)
    pile_diag = math.sqrt((2.2 * args.R) ** 2 * 2 + (args.crown * 1.4) ** 2)
    pile_dist = 1.5 * pile_diag
    crown_pt = (0.0, 0.0, args.crown)
    fall_dir = (0.0, -1.0)

    tiles_all = []
    for tag, usd_path in (("full", usd_full), ("budgeted", usd_bud)):
        print("[sbb] rendering {0} ...".format(tag))
        t0 = time.time()
        tiles, sheet = rp.render_views(
            usd_path, out_dir, tag, pile_center, pile_dist, crown_pt, fall_dir,
            res=(args.rw, args.rh), samples=args.samples, cpu=args.cpu,
            close_dist=max(6.0, args.crown * 0.9))
        tiles_all.extend(tiles)
        print("[sbb]   {0} done in {1:.1f}s -> {2}".format(tag, time.time() - t0, sheet))

    combined = out_dir / "full_vs_budgeted_contact.png"
    rp._contact_sheet(tiles_all, combined, "full (top) vs budgeted (bottom)")
    print("[sbb] combined contact sheet -> {0}".format(combined))

    print("\n" + "=" * 72)
    print("SETTLE BUDGET BENCH")
    print("  pile size (n)          : {0}".format(args.n))
    print("  FULL     loose (physics-eligible in production)  : {0}".format(len(authored_full)))
    print("  BUDGETED loose (physics-eligible in production)  : {0}".format(len(kept_bud)))
    print("  BUDGETED placed geometrically (never physics)    : {0}".format(len(geo_bud)))
    print("  reduction                                        : {0:.1f}%".format(
        100.0 * (1.0 - len(kept_bud) / max(1, len(authored_full)))))
    print("=" * 72 + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
