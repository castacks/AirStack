#!/usr/bin/env python
"""qs_tear_probe.py — the round-6b ragged-boundary pass, end to end, on a
bare USD stage: no Kit, no Nucleus, no GPU. Mirrors `tools/quake_gac_probe.py`
(itself the `quake_sliced` analogue of `tools/gac_burn_probe.py`), extended
with real per-piece UV + GeomSubset + material authoring so `fire_collapse.
facade_skin` / `skin_fragment` — the part of the tear pass that decides
whether a torn piece matches the wall it extends — has something real to
read, which the bare unit-cube meshes `quake_gac_probe.py` authors do not
carry.

    uv run --python 3.13 --with usd-core --with numpy --with vtk \
        python scene_gen/tools/qs_tear_probe.py SM_Building_02 DG4

    docker exec isaac-sim bash -c \
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
       /isaac-sim/AirStack/scene_gen/tools/qs_tear_probe.py SM_Building_30 DG3"

Args: `NAME [GRADE [SEED]]` — defaults `SM_Building_02 DG4 7`.

WHERE THE ELEMENT TABLE COMES FROM — same as `quake_gac_probe.py`, imported
from it rather than re-derived: the merged GreatAmericanCity / downtowncity
source lives only on Nucleus, so this probe uses `kit_bake`'s local baked-kit
cache (`SM_Building_02`, `SM_Building_09` today) when it exists, or the
`test_quake_sliced.fake_sliced_building` synthetic table otherwise. Neither
carries real texture data (`kits.json`'s pieces are geometry only), so every
piece here is authored with a SYNTHETIC cladding material and a real UV
primvar rather than the actual GAC atlas — offline, there is no other source
for it. What this probe can still show honestly: the geometric/authoring
machinery (`_plan_tears` -> `_author_tears` -> `fire_collapse.plan_edges` /
`_tear_perimeter` / `facade_skin` / `skin_fragment`) runs end to end on real
piece footprints and produces a real edge census and a real uv/tone/none/
no_facade split — `none` and `no_facade` are the two buckets that would catch
a broken façade pick even with a fake material, because they fire on "no
readable façade at all", not on "the façade happened to be fake".

Exit 0 if the census reads 100 % (short of the QS_MAX_TEARS budget) and
`none`/`no_facade` are both 0; 1 otherwise.
"""
import os
import random
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
if _SG not in sys.path:
    sys.path.insert(0, _SG)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from detail import kit_bake as kb                              # noqa: E402
from disaster import fire_collapse as fc                      # noqa: E402
from disaster import quake_flow as qf                         # noqa: E402
from disaster import quake_sliced as qs                        # noqa: E402
import quake_gac_probe as qgp                                  # noqa: E402

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt            # noqa: E402


def _author_piece_mesh_faced(stage, p, mat):
    """A real closed box at the piece's own world position/size — explicit
    points, no scale op, so `fracture`/VTK see the same shape a real sliced
    piece's mesh would — PLUS a `primvars:st` UV primvar and one GeomSubset
    bound to `mat`, so `facade_skin` has a real façade candidate to find.

    UV = the piece's own LOCAL (x, z), which is a genuine 2D parameterisation
    of whichever face is normal to Y — exactly the S/N wall faces this
    probe's fixture cares about, and non-degenerate for the E/W ones too
    (z still varies even where x is constant on that face).
    """
    sx, sy, sz = (p.get("_size") or (1.0, 1.0, 3.0))
    sx, sy, sz = max(0.1, sx), max(0.1, sy), max(0.1, sz)
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    x0, y0, z0 = float(p["x_m"]), float(p["y_m"]), float(p["z_m"])
    corners = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
              (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
    pts = [Gf.Vec3f(x0 + cx, y0 + cy, z0 + cz) for cx, cy, cz in corners]
    faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
            [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(p["prim_path"]))
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4] * 6)
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    mesh.CreateExtentAttr([Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f(hx, hy, hz)])
    uv = [(cx / sx + 0.5, cz / sz + 0.5) for cx, _cy, cz in corners]
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    pv.Set(Vt.Vec2fArray([Gf.Vec2f(*q) for q in uv]))
    sub = UsdGeom.Subset.CreateGeomSubset(
        UsdGeom.Imageable(mesh), "cladding", UsdGeom.Tokens.face,
        Vt.IntArray(list(range(6))))
    UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat)


def get_element_table(name, seed):
    """`(pls, style, source)` — like `quake_gac_probe.get_element_table`,
    but gated on the manifest ROW alone (`kit_bake._entry`), not
    `kit_bake.have_kit`.

    OFFLINE-HOST FIX, THIS PROBE ONLY (not `kit_bake.py` / `quake_gac_
    probe.py`, neither touched): `have_kit` additionally requires
    `os.path.exists(e["usd"])` on the CONTAINER-side absolute path the
    manifest stores (`/isaac-sim/AirStack/...`), which is never present on
    a bare host, and a fresh `fingerprint()` match against the slicer
    version that made the bake, which a stale bake never has either — so
    `quake_gac_probe.get_element_table` falls through to the synthetic
    fixture unconditionally when run offline, even for an asset with a real
    baked kit sitting right there in `kits.json`. `load_real_kit` was
    explicitly written not to need that: its own docstring says it
    "DELIBERATELY NEVER OPENS THE REFERENCED .usd GEOMETRY" and reads only
    the manifest's plain-JSON `pieces`. So here: use the manifest row
    directly and let `load_real_kit` do what it already promises.
    """
    if kb._entry(name) is not None:
        try:
            pls, style, _grid = qgp.load_real_kit(name)
            return pls, style, "kit ({0}, offline manifest row)".format(name)
        except Exception as exc:
            print("[qs_tear_probe] baked kit for {0} failed to load ({1}); "
                  "falling back to synthetic".format(name, exc))
    pls, style, _grid = qgp.load_synthetic(name, seed)
    return pls, style, "synthetic (test_quake_sliced.fake_sliced_building)"


def build_stage_and_ctx(pls, style, btype, grade, seed):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/cell"))
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/cell/pieces"))
    mat = UsdShade.Material.Define(stage, Sdf.Path("/World/Cladding"))
    for p in pls:
        _author_piece_mesh_faced(stage, p, mat)

    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = btype
    rng = random.Random(seed)
    plan = qs.plan_damage(info, info["elements"], grade, btype, rng)

    ctx = {"stage": stage, "parent": "/World/cell", "info": info, "rng": rng,
          "nrng": None, "mats": qgp._AutoMats(stage, "/World/cell"), "cache": {},
          "tag": "p0", "loose": [], "static_extra": [], "velocity": {},
          "authored": [], "notes": [],
          "fit": {"slabs": {}, "columns": {}, "partitions": [], "props": {},
                  "all": []}}
    return stage, ctx, info, plan


def main(argv):
    name = argv[1] if len(argv) > 1 else "SM_Building_02"
    grade = argv[2] if len(argv) > 2 else "DG4"
    seed = int(argv[3]) if len(argv) > 3 else 7

    if grade not in qs.LEVELS:
        print("[qs_tear_probe] unknown grade {0!r}".format(grade))
        return 2

    pls, style, source = get_element_table(name, seed)
    btype = qs.construction_type(name)
    print("[qs_tear_probe] {0} {1}: {2} piece(s) from {3}, btype={4}, "
          "QS_RAGGED={5} QS_MAX_TEARS={6}".format(
              name, grade, len(pls), source, btype, qs.QS_RAGGED,
              qs.QS_MAX_TEARS))

    stage, ctx, info, plan = build_stage_and_ctx(pls, style, btype, grade, seed)
    print("[qs_tear_probe] plan stats: {0}".format(plan["stats"]))

    n_dropped = sum(1 for t in plan["tears"] if t["dropped"])
    census = fc.edge_census([{"classes": t["classes"], "torn": not t["dropped"]}
                             for t in plan["tears"]])
    print("[qs_tear_probe] edge census (class: neighbours/planned-to-tear):")
    ok_census = True
    for cls, (n_neigh, n_torn) in census.items():
        if n_neigh == 0:
            continue
        # PROBE FIX: the shortfall must be explained by a DROP IN THIS SAME
        # CLASS (budget overrun or the core/roof/moved exclusion), not by
        # "something, somewhere, got dropped" — a single dropped job on an
        # unrelated class used to blanket-excuse every other class's
        # shortfall, which would hide a real regression as long as the
        # budget was exceeded anywhere at all.
        n_drop_cls = sum(1 for t in plan["tears"]
                         if t["dropped"] and cls in t["classes"])
        pct = 100.0 * n_torn / n_neigh
        accounted = (n_torn + n_drop_cls >= n_neigh) or pct >= 99.999
        flag = "" if accounted else "  <-- SHORT"
        if not accounted:
            ok_census = False
        print("   {0:8s} {1:4d} neighbour(s), {2:4d} planned to tear "
              "({3:5.1f} %){4}".format(cls, n_neigh, n_torn, pct, flag))

    old_pp, old_au = qs.PLAN_PILE, qs.AUTHOR
    qs.PLAN_PILE, qs.AUTHOR = qgp._fake_plan_pile, qgp._fake_author
    t0 = time.time()
    try:
        qs.apply_plan(stage, ctx, plan, verbose=True)
    finally:
        qs.PLAN_PILE, qs.AUTHOR = old_pp, old_au
    dt = time.time() - t0

    skin = ctx.get("_tear_skin") or {}
    print("[qs_tear_probe] n_tears={0} n_tears_dropped={1} tear pass wall "
          "time (whole apply_plan) = {2:.3f}s".format(
              plan["stats"]["n_tears"], plan["stats"]["n_tears_dropped"], dt))
    print("[qs_tear_probe] _tear_skin: {0}".format(skin))
    print("[qs_tear_probe] ctx: {0} loose, {1} static_extra after apply_plan"
          .format(len(ctx["loose"]), len(ctx["static_extra"])))

    problems = []
    if not ok_census:
        problems.append("edge census did not reach 100% on some class with "
                        "no budget drop")
    if skin.get("none", 0):
        problems.append("_tear_skin['none'] = {0} (> 0)".format(skin["none"]))
    if skin.get("no_facade", 0):
        problems.append("_tear_skin['no_facade'] = {0} (> 0)".format(
            skin["no_facade"]))
    if (not (plan.get("tears") or []) and plan["stats"]["n_removed"] > 0
            and not plan.get("collapse")):
        # PROBE FIX, two edits from the original heuristic:
        # (1) `plan["collapse"]` (total/pancake) is excluded: the design's
        #     gate (Sketch 1 Sec.4.1) is "NO tears" there -- the shell is
        #     gone, nothing survives to tear -- and `test_a_total_collapse_
        #     plans_no_tears` already pins `plan["tears"] == []` there.
        #     Without this every DG5 total-collapse run false-positived.
        # (2) Gated on `plan["tears"]` being EMPTY (no candidate found at
        #     all), not `plan["stats"]["n_tears"] == 0` (zero SURVIVED the
        #     drop). A hole whose only neighbours are legitimately excluded
        #     (a `core` piece, or one already in the macroblock/panel
        #     "moved" set) correctly drops every one of its candidate jobs
        #     -- that is not a bug, and the per-class census check above
        #     already accounts for it. Only "found literally nothing" is
        #     worth a red flag here.
        problems.append("pieces were removed but no tear jobs were even "
                        "candidated (plan['tears'] is empty)")

    print()
    if problems:
        print("[qs_tear_probe] {0} PROBLEM(S):".format(len(problems)))
        for p in problems:
            print("  - " + p)
        return 1
    print("[qs_tear_probe] OK — {0} {1}: census 100% (short of budget), "
          "none=no_facade=0".format(name, grade))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
