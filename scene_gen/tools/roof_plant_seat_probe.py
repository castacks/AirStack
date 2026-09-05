#!/usr/bin/env python
"""roof_plant_seat_probe — DOES A BAKED QUAKE ARCHETYPE'S ROOF PLANT ACTUALLY
REST ON SOMETHING, offline, against the exported `.usd` on disk?

    python3 scene_gen/tools/roof_plant_seat_probe.py
    python3 scene_gen/tools/roof_plant_seat_probe.py --usd scene_gen/assets/archetype/bld_brownstone_row_DG3.usd
    pytest -q scene_gen/tests/test_roof_plant_seat_probe.py

THE BUG THIS EXISTS FOR (round-6 review, `b0_apartment_DG5_obl.png` /
`b4_brownstone_row_DG4_obl.png`, three water tanks standing bolt upright on
stilts over shattered rooftops): `disaster/quake_collapse.py`'s round-5 fix
(`_sweep_roof_props`) gave a fallen roof prop a 0.3-0.9 m/s velocity kick and
handed it to `settle.run`'s fixed step budget — its own docstring already
named the failure mode ("a body that does not make it down in time bakes
frozen mid-air") and it happened anyway: MEASURED (bare `pxr`, no Kit, no
Isaac Sim) on `scene_gen/assets/archetype/bld_brownstone_row_DG3.usd`, a
tank fell ~1 m of a required ~3 m storey and froze; on
`bld_office_wide_DG4.usd`, a tank on a `soft_storey` crush (excluded from
`_sweep_roof_props` entirely — `_author_band` never carried it) never moved
at all until the same generic settle dropped it partway, landing 2-4.7 m
short of the true roof.

`disaster/quake_collapse.py`'s round-6 fix replaces the kick with a
geometric support probe (`_deck_support_z`, the `tri_soup` idiom this file's
`_tri_soup`/`_up_faces` below are copied from `tools/fc_roof_deck_probe.py`)
run AT AUTHOR TIME, live against the stage. This probe is the SAME check run
the OTHER way: against the flattened, exported, already-merged `.usd` a bake
actually produced, so it can gate a rebake instead of trusting the authoring
code ran correctly.

WHY IT CANNOT JUST READ `ctx["roof_plant"]`. `bake_quake_archetypes_launch_
script.py` exports every row through `bake.export_object(..., merge="on")`
by default, which merges every prim sharing a material into ONE Mesh — a
building with 2 tanks drawn by `quake_flow.dress_roof` (`rng.choice((0, 1,
1, 2))`) exports as ONE `merged_tank_dark` mesh holding both barrels, and
the individual `tank_<tag>_<uid>` prim paths that existed at author time are
gone. So this probe does not look for a path — it CLUSTERS a merged mesh's
own triangles into distinct physical objects by mesh connectivity (a
union-find over shared point indices: two triangles that share a vertex are
the same authored object; `bake.export_object`'s merge is a concatenation of
independent source meshes, so it never welds one prop's points to another's,
whichever order the merge happens to visit them in) and tests EACH cluster's
own footprint independently — the same shape the earlier manual
investigation used by hand (sorting a merged tank mesh's points by X and
looking for a gap) that this probe replaces with something a rebake can gate
on automatically.

CANDIDATES ARE FOUND BY MATERIAL, NOT BY NAME. `quake_flow._tank` binds
every water tank to the `tank_wood` material key, which resolves to a
Material prim literally named `tank_dark` (`quake_flow.materials`:
`out["tank_wood"] = out.get("tank_dark") or out.get("timber")`); AC units
bind to `plant_metal` directly (`quake_flow.dress_roof`). Both names are
used for NOTHING else in this pipeline (grepped: `tank_dark` only ever binds
a tank barrel; `plant_metal`, within `disaster/quake_flow.py`, only ever
binds an AC unit — `disaster/urban_fire.py` reuses the same key name for its
own, unrelated fire-city roof dressing, a different pipeline this probe
never opens), so a mesh bound to either is a rooftop-plant candidate and
nothing else is.

THE VERDICT. For each cluster: the world Z of its lowest point (`base_z`,
the same "pivot" convention every author-time helper in `quake_flow.py`
uses) against `_deck_support_z`'s own resolved support height under that
cluster's exact XY footprint, searched STRICTLY BELOW `base_z`. A gap
(`base_z - support`) past `--tolerance` (default 0.12 m — "a few cm" plus
slack for the legitimate few-cm interpenetration `quake_collapse.
ROOF_PROP_SUPPORT_MARGIN_M` already tolerates at author time) fails; no
support found AT ALL (a genuine hole all the way down) fails outright.

An item with no building support but whose base is already at ground is also
valid: it fell through a destroyed roof and reached grade.  That is distinct
from a no-support item still suspended above grade, which fails.

NO KIT, NO SIMULATIONAPP. Bare `pxr` + `numpy` only, exactly like `tools/
fc_roof_deck_probe.py` — safe to run beside a live Isaac session, and safe
to run with no Isaac Sim at all, which is how this probe was written and
first run (2026-08-31, offline, this repo's working tree).
"""
import argparse
import glob
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, _SG)

from pxr import Usd, UsdGeom, UsdShade                          # noqa: E402

from disaster import quake_collapse as qc                       # noqa: E402

# The two Material prim names a rooftop-plant mesh binds to (see the module
# docstring: `tank_dark` via `quake_flow._tank`, `plant_metal` via
# `quake_flow.dress_roof`'s AC-unit placement). Matched by the LAST path
# component of the bound material, so it is unaffected by which scope the
# export nested `Looks` under.
CANDIDATE_MATERIALS = ("tank_dark", "plant_metal")

DEFAULT_GLOB = os.path.join(_SG, "assets", "archetype", "*.usd")
DEFAULT_TOLERANCE_M = 0.12


def _mesh_prims_by_material(stage, material_names):
    """{mesh_prim: material_name} for every active Mesh bound to one of
    `material_names` (by the bound material's own prim name)."""
    out = {}
    names = set(material_names)
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or not prim.IsActive():
            continue
        mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        if not mat:
            continue
        mname = mat.GetPath().name
        if mname in names:
            out[prim] = mname
    return out


def _world_points(prim):
    """(N, 3) world-space points of one Mesh, plus its face table, exactly
    the extraction `quake_collapse._deck_support_z` uses on the live stage —
    duplicated here (not imported) because it is the read half of a
    triangle soup and the probe should not need `_deck_support_z`'s
    authoring-side signature to get it."""
    mesh = UsdGeom.Mesh(prim)
    pts = mesh.GetPointsAttr().Get()
    counts = mesh.GetFaceVertexCountsAttr().Get()
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    if not pts or not counts or not idx:
        return None, None, None
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    M = np.array(xc.GetLocalToWorldTransform(prim), dtype=np.float64)
    P = np.array([[q[0], q[1], q[2]] for q in pts], dtype=np.float64)
    P = (np.hstack([P, np.ones((len(P), 1))]) @ M)[:, :3]
    return P, np.asarray(counts), np.asarray(idx)


class _UnionFind:
    def __init__(self, n):
        self.parent = list(range(n))

    def find(self, a):
        while self.parent[a] != a:
            self.parent[a] = self.parent[self.parent[a]]
            a = self.parent[a]
        return a

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[ra] = rb


# A tank's OWN construction is not one connected mesh either
# (`quake_flow._tank`'s docstring: "ONE PRIM, NOT FIVE" is about the prim
# count, not the topology — `_add_box` for each leg and `_add_tube` for the
# barrel each start their own point range and never index back into an
# earlier part's points). So face-connectivity alone shatters ONE tank into
# ~6-14 pieces (measured: `bld_brownstone_row_DG3.usd`'s 2-tank merged mesh
# is 14 face-components, not 2) — legs from the barrel they hold up, which
# would then have the barrel's own support (its legs) `exclude=` filtered
# out from under it as "the same source mesh" and read as floating on every
# perfectly fine tank. XY_MERGE_M re-merges any two face-components whose
# (expanded) XY footprints touch, which reunites a tank's legs with its own
# barrel (built within ~1.4 m of each other, `quake_flow.dress_roof`'s own
# tank radius) while leaving two genuinely SEPARATE roof items — placed at
# least a few metres apart by `dress_roof`'s own `_spot` — as separate
# clusters.
XY_MERGE_M = 1.8


def _clusters(points, counts, idx, xy_merge_m=XY_MERGE_M):
    """Physically distinct objects in `points`: face-connected components,
    then re-merged across any pair whose XY footprints (widened by
    `xy_merge_m`) overlap — see the note above for why the second pass is
    necessary and why it is safe."""
    uf = _UnionFind(len(points))
    off = np.concatenate([[0], np.cumsum(counts)[:-1]])
    for n, o in zip(counts, off):
        face = idx[o:o + int(n)]
        for k in range(1, len(face)):
            uf.union(int(face[0]), int(face[k]))
    fine = {}
    for i in range(len(points)):
        fine.setdefault(uf.find(i), []).append(i)
    fine_groups = list(fine.values())
    if len(fine_groups) <= 1:
        return fine_groups

    boxes = []
    for g in fine_groups:
        pts = points[g]
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        boxes.append((float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1])))
    uf2 = _UnionFind(len(fine_groups))
    m = xy_merge_m
    for a in range(len(fine_groups)):
        ax0, ay0, ax1, ay1 = boxes[a]
        for b in range(a + 1, len(fine_groups)):
            bx0, by0, bx1, by1 = boxes[b]
            if (ax0 - m <= bx1 and bx0 - m <= ax1
                    and ay0 - m <= by1 and by0 - m <= ay1):
                uf2.union(a, b)
    merged = {}
    for gi, g in enumerate(fine_groups):
        merged.setdefault(uf2.find(gi), []).extend(g)
    return list(merged.values())


def check_archetype(usd_path, tolerance=DEFAULT_TOLERANCE_M,
                    material_names=CANDIDATE_MATERIALS, verbose=True):
    """Every rooftop-plant cluster in `usd_path`, checked against the real
    support under its own footprint. Returns a list of result dicts:
    `{usd, material, cluster, base_z, support_z, gap, ok}` — `support_z` is
    `None` when nothing at all was found under the footprint (an automatic
    `ok=False`)."""
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise RuntimeError("could not open " + usd_path)
    root = "/"
    by_mesh = _mesh_prims_by_material(stage, material_names)
    results = []
    # Exclude EVERY roof-plant mesh from the support search.  Export normally
    # merges by material, but merge=off/debug bakes can retain several meshes;
    # one tank or AC must never count another one as a roof.
    plant_paths = tuple(str(p.GetPath()) for p in by_mesh)
    for prim, mname in by_mesh.items():
        P, counts, idx = _world_points(prim)
        if P is None:
            continue
        clusters = _clusters(P, counts, idx)
        mesh_path = str(prim.GetPath())
        for ci, group in enumerate(clusters):
            pts = P[group]
            lo, hi = pts.min(axis=0), pts.max(axis=0)
            cx = float((lo[0] + hi[0]) / 2.0)
            cy = float((lo[1] + hi[1]) / 2.0)
            half_w = max(0.3, float(hi[0] - lo[0]) / 2.0 * 1.15)
            half_d = max(0.3, float(hi[1] - lo[1]) / 2.0 * 1.15)
            base_z = float(lo[2])
            support = qc._deck_support_z(stage, root, cx, cy, half_w, half_d,
                                         base_z, exclude=plant_paths)
            # A roof item can legitimately fall through a destroyed building
            # all the way to grade.  No structural support above ground is a
            # failure; no support when already grounded is not.
            if support is None and base_z <= tolerance:
                support = 0.0
            gap = None if support is None else round(base_z - support, 3)
            ok = support is not None and gap <= tolerance
            results.append(dict(usd=usd_path, material=mname,
                                mesh=mesh_path, cluster=ci, n_points=len(group),
                                cx=round(cx, 2), cy=round(cy, 2),
                                base_z=round(base_z, 3),
                                support_z=(round(support, 3)
                                          if support is not None else None),
                                gap=gap, ok=ok))
            if verbose:
                tag = "PASS" if ok else "FAIL"
                print("[roof_plant_seat] {0} {1:<16} cluster {2} "
                      "({3} pts) @ ({4:+.1f},{5:+.1f}): base_z={6:.2f} "
                      "support_z={7}  gap={8}  [{9}]".format(
                          tag, mname, ci, len(group), cx, cy, base_z,
                          "none" if support is None else "{0:.2f}".format(support),
                          "n/a" if gap is None else "{0:+.2f} m".format(gap),
                          os.path.basename(usd_path)))
    return results


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", action="append", default=None,
                    help="one archetype .usd (repeatable); default: every "
                         "file under scene_gen/assets/archetype/*.usd")
    ap.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE_M,
                    help="max allowed gap in metres between a resting prop "
                         "and the real deck under it (default %(default)s)")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument(
        "--repair", action="store_true",
        help="repair unsupported rooftop-plant clusters in place before "
             "checking (make a backup first for valuable bake outputs)")
    a = ap.parse_args(argv)

    paths = a.usd or sorted(glob.glob(DEFAULT_GLOB))
    if not paths:
        print("[roof_plant_seat] no archetype .usd found (looked under {0})"
              .format(DEFAULT_GLOB))
        return 0

    all_results = []
    for p in paths:
        try:
            if a.repair:
                from disaster import bake
                bake.reseat_roof_plant_clusters_in_file(
                    p, tolerance=a.tolerance, verbose=not a.quiet)
            all_results += check_archetype(p, tolerance=a.tolerance,
                                           verbose=not a.quiet)
        except Exception as exc:
            print("[roof_plant_seat] FAILED to check {0}: {1}".format(p, exc))

    n = len(all_results)
    n_bad = sum(1 for r in all_results if not r["ok"])
    print("\n[roof_plant_seat] {0} rooftop-plant cluster(s) checked across "
          "{1} file(s), {2} floating (gap > {3:.2f} m or no support at all)"
          .format(n, len(paths), n_bad, a.tolerance))
    if n_bad:
        print("[roof_plant_seat] FLOATERS:")
        for r in all_results:
            if not r["ok"]:
                print("  {0}  {1} cluster {2}  base_z={3}  support_z={4}  "
                      "gap={5}".format(os.path.basename(r["usd"]),
                                       r["material"], r["cluster"],
                                       r["base_z"], r["support_z"], r["gap"]))
    return 1 if n_bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
