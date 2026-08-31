#!/usr/bin/env python
"""quake_gac_probe.py — the earthquake-ladder offline check for a SLICED GAC/
downtowncity building, end to end on a bare stage: no Kit, no Nucleus, no
GPU. The `quake_sliced` analogue of `tools/gac_burn_probe.py` (which runs
`gac_fire.burn_gac` the same way for the fire ladder).

    uv run --python 3.13 --with usd-core --with numpy \
        python scene_gen/tools/quake_gac_probe.py SM_Building_02 DG5

    docker exec isaac-sim bash -c \
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
       /isaac-sim/AirStack/scene_gen/tools/quake_gac_probe.py SM_Building_09 DG4"

Args: `NAME [GRADE [SEED]]` — defaults `SM_Building_02 DG5 7`.

WHERE THE ELEMENT TABLE COMES FROM
-----------------------------------
There is no local mirror of a merged GreatAmericanCity/downtowncity asset in
this repo, and the merged sources live only on Nucleus — so this probe never
does a live `slice_to_kit` itself (that needs the real merged mesh). Instead:

  1. `detail.kit_bake.have_kit(NAME)` — a PRE-SLICED kit cache
     (`scene_gen/assets/kits/*.usd` + `kits.json`, `tools/bake_gac_kits.py`).
     Two assets are baked there today: `SM_Building_02`, `SM_Building_09`.
     `kit_bake.load_kit` needs only `pxr` (a `DefinePrim`+`AddReference` onto
     a LOCAL file, no Kit runtime) so this branch runs on the bare host.

     *** A DISCOVERED SCALE ARTIFACT, NOT THIS PROBE'S BUG ***. `kits.json`'s
     placement coordinates and its `grid["H"]/["W"]/["D"]` are ~100x too
     large to be metres (measured: `SM_Building_02`'s kit reports H=3858.27,
     but `_plans/gac_buildings.json`'s own real measurement of the same
     asset is H=38.6 m — a ratio of 99.95, i.e. `gac_fire.GAC_SCALE=0.01`
     applied to the wrong side). `bake_gac_kits.py` evidently recorded the
     merged source's RAW pre-scale coordinates rather than the post-
     `place_source`-scale ones a live slice measures. This probe CORRECTS
     for it (scaling every placement + the grid by the ratio against `_plans/
     gac_buildings.json`'s measured H, falling back to `gac_fire.GAC_SCALE`
     if that lookup fails) and prints what it did, because the actual bake
     launcher (`quake_gac_bake_launch_script.py`) never goes through this
     cache at all — it always does a LIVE `slice_to_kit`, which measures off
     the real, already-scaled placed source — so this artifact cannot reach
     a real bake. It is left here, loud, for whoever next touches
     `bake_gac_kits.py`/`kit_bake.py`.
  2. Otherwise: the SAME synthetic element table `test_quake_sliced.py`
     built and validated (`fake_sliced_building` — mirrors `gac_storey_
     slice.ring()`'s real bay/storey/corner grammar closely enough that
     `quake_flow.describe`, the real function, builds a real mass box off
     it). Imported, not re-derived, so this probe cannot drift from the one
     place that grammar is checked
     (`test_the_fixture_is_shaped_like_a_real_slice`).

WHAT THIS RUNS
---------------
`plan_damage` (pure) for the requested grade, `plan_to_json`/`plan_from_json`
round-tripped, a "nothing lands more than a small tolerance below the
building's own foundation line" sweep over every displaced piece (pure
arithmetic, `quake_sliced.apply_rigid` — the exact matrix `apply_plan` hands
to the real prim), and — whenever `pxr` is importable — `apply_plan` on a
bare in-memory `Usd.Stage` with `PLAN_PILE`/`AUTHOR` stubbed (`quake_sliced`'s
own documented seam, so this needs neither the real rubble modules nor
`RUBBLE_ASSET_ROOT`), checking every removed piece is deactivated, every
displaced piece's ACTUAL new world position matches the prediction, one pile
authored per pile spec with its numbers landing in `ctx["rubble"]`, and a
root-layer export that reopens cold with the same prim count.

Exit 0 if every check passes, 1 otherwise.
"""

import json
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
if _SG not in sys.path:
    sys.path.insert(0, _SG)

from detail import kit_bake as kb                              # noqa: E402
from disaster import quake_flow as qf                          # noqa: E402
from disaster import quake_sliced as qs                        # noqa: E402

try:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade            # noqa: E402
    HAVE_USD = True
except Exception:                                              # pragma: no cover
    HAVE_USD = False

_PLANS_DIR = os.path.join(_SG, "_plans")


def _true_h(name):
    """`_plans/gac_buildings.json`'s own measured H for `name`, or None."""
    path = os.path.join(_PLANS_DIR, "gac_buildings.json")
    try:
        with open(path) as fh:
            rows = json.load(fh)
        for r in rows:
            if r.get("name") == name:
                return float(r["H"])
    except Exception:
        pass
    return None


def _scale_placement(p, factor):
    q = dict(p)
    q["x_m"] = float(p["x_m"]) * factor
    q["y_m"] = float(p["y_m"]) * factor
    q["z_m"] = float(p["z_m"]) * factor
    if p.get("_size"):
        q["_size"] = tuple(float(v) * factor for v in p["_size"])
    return q


def load_real_kit(name):
    """`(pls, style, grid)` off `scene_gen/assets/kits/kits.json`'s own
    `"pieces"`/`"grid"` records, corrected for the scale artifact the module
    docstring describes. Raises if `name` has no baked kit — callers check
    `kit_bake.have_kit(name)` first.

    DELIBERATELY NEVER OPENS THE REFERENCED `.usd` GEOMETRY. `kits.json`
    stores the baked file's path CONTAINER-SIDE
    (`/isaac-sim/AirStack/scene_gen/assets/kits/<name>.usd`), which does not
    exist on a bare host — `kit_bake.load_kit` (which this function does NOT
    call) would raise `IOError` there. But `plan_damage`/`apply_plan` never
    read a piece's actual MESH, only its placement dict (`_role`/`_side`/
    `_storey`/`_bay`/`_size`/`x_m`/`y_m`/`z_m`/`prim_path`) — exactly what
    `kits.json`'s `"pieces"` list already carries as plain JSON, host-
    portable with no path to resolve at all.
    """
    e = kb._entry(name)
    if e is None:
        raise KeyError("no baked kit named {0!r} in {1}".format(
            name, kb.MANIFEST_PATH))
    grid = dict(e.get("grid") or {})
    pieces_scope = "/W/cell/kit/pieces"
    pls = []
    for p in (e.get("pieces") or []):
        d = dict(p)
        nm = d.pop("name", None)
        if not nm:
            continue
        d["prim_path"] = "{0}/{1}".format(pieces_scope, nm)
        pls.append(d)

    raw_h = float(grid.get("H") or 0.0)
    true_h = _true_h(name)
    if true_h and raw_h > 1e-6:
        factor = true_h / raw_h
    else:
        from disaster import gac_fire as gcf
        factor = gcf.GAC_SCALE
    if abs(factor - 1.0) > 1e-6:
        print("[qgac_probe] {0}: kits.json reports H={1:.1f} (raw), "
              "_plans/gac_buildings.json measures {2} m -- rescaling every "
              "placement by {3:.6f} (see the module docstring's 'DISCOVERED "
              "SCALE ARTIFACT')".format(
                  name, raw_h, true_h if true_h else "(no row)", factor))
    pls = [_scale_placement(p, factor) for p in pls]
    bays = {sd: dict(b, pitch=float(b.get("pitch", 0.0)) * factor)
           for sd, b in (grid.get("bays") or {}).items()}
    grid = dict(grid, W=float(grid.get("W", 0.0)) * factor,
               D=float(grid.get("D", 0.0)) * factor,
               H=float(grid.get("H", 0.0)) * factor,
               z0=float(grid.get("z0", 0.0)) * factor,
               storey_h=float(grid.get("storey_h", 0.0)) * factor,
               storeys=[float(s) * factor for s in (grid.get("storeys") or [])],
               bays=bays)

    # `gac_slice.register_style` is what makes `quake_flow.describe` (a
    # bare function of `style`/`placements`, no `pxr` needed for this half)
    # resolve a real footprint/height for `style` — needs no stage at all.
    from detail import gac_slice as gsl
    style = "gac_{0}".format(name)
    gsl.register_style(grid, style, pieces_of=pls)
    return pls, style, grid


def load_synthetic(name, seed):
    """The `test_quake_sliced.py` fixture — imported, not re-derived (see
    the module docstring)."""
    _TESTS_DIR = os.path.normpath(os.path.join(_HERE, "..", "tests"))
    if _TESTS_DIR not in sys.path:
        sys.path.insert(0, _TESTS_DIR)
    from test_quake_sliced import fake_sliced_building
    return fake_sliced_building(seed=seed)


def get_element_table(name, seed):
    """`(pls, style, source)` — `source` is `"kit"` or `"synthetic"`, purely
    for the printed report."""
    if kb.have_kit(name):
        try:
            pls, style, _grid = load_real_kit(name)
            return pls, style, "kit ({0})".format(name)
        except Exception as exc:
            print("[qgac_probe] baked kit for {0} failed to load ({1}); "
                  "falling back to synthetic".format(name, exc))
    pls, style, _grid = load_synthetic(name, seed)
    return pls, style, "synthetic (test_quake_sliced.fake_sliced_building)"


# ---------------------------------------------------------------------------
# stage-level helpers — a self-contained copy of `tests/test_quake_gac_bake.
# py`'s own (a `tools/` script does not import `tests/` for its mechanics,
# only for the synthetic FIXTURE above, which is the one piece the brief
# names explicitly as reusable)
# ---------------------------------------------------------------------------
def _author_piece_mesh(stage, p):
    pts = [Gf.Vec3f(x, y, z) for x in (-0.5, 0.5) for y in (-0.5, 0.5)
          for z in (-0.5, 0.5)]
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(p["prim_path"]))
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4, 4, 4, 4, 4, 4])
    mesh.CreateFaceVertexIndicesAttr(
        [0, 1, 2, 3, 4, 5, 6, 7, 0, 4, 7, 1, 1, 7, 6, 2, 2, 6, 5, 3, 3, 5, 4, 0])
    sx, sy, sz = (p.get("_size") or (1.0, 1.0, 3.0))
    xf = UsdGeom.Xformable(mesh)
    xf.AddTranslateOp().Set(Gf.Vec3d(float(p["x_m"]), float(p["y_m"]),
                                     float(p["z_m"])))
    xf.AddScaleOp().Set(Gf.Vec3f(max(sx, 0.1), max(sy, 0.1), max(sz, 0.1)))


def _safe(s):
    return "".join(c if (c.isalnum() or c in "._-") else "_" for c in str(s))


class _AutoMats(dict):
    """See `tests/test_quake_gac_bake.py._AutoMats` — same idea, kept here
    too so this probe needs no Nucleus material either."""

    def __init__(self, stage, parent):
        super().__init__()
        self._stage, self._parent = stage, parent

    def __missing__(self, key):
        path = "{0}/QuakeLooksFake/{1}".format(self._parent, _safe(key))
        mat = (UsdShade.Material.Get(self._stage, path)
              or UsdShade.Material.Define(self._stage, Sdf.Path(path)))
        self[key] = mat
        return mat


def _fake_plan_pile(m, btype, rng, **kw):
    sides = list(kw.get("sides") or ["S"])
    return {"mound": None, "large": [], "instances": {},
            "stats": {"fall_sides": sides, "reach_m": {s: 3.0 for s in sides},
                      "extent_m": {s: 2.5 for s in sides},
                      "crown_m": float(kw.get("crown_m") or 2.0)}}


def _fake_author(stage, parent, plan, **kw):
    path = "{0}/rubble_{1}".format(parent, _safe(kw.get("tag", "rubble")))
    UsdGeom.Xform.Define(stage, Sdf.Path(path))
    return {"mound": path, "apron": None, "static": [path], "large": [],
            "instancers": [], "all": [path]}


def run_stage_checks(pls, style, btype, grade, seed):
    """`apply_plan` on a bare stage. Returns a list of problem strings (empty
    means every check passed)."""
    problems = []
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/cell"))
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/cell/pieces"))
    for p in pls:
        _author_piece_mesh(stage, p)

    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = btype
    rng = random.Random(seed)
    plan = qs.plan_damage(info, info["elements"], grade, btype, rng)

    ctx = {"stage": stage, "parent": "/World/cell", "info": info, "rng": rng,
          "nrng": None, "mats": _AutoMats(stage, "/World/cell"), "cache": {},
          "tag": "p0", "loose": [], "static_extra": [], "authored": [],
          "notes": [], "fit": {"slabs": {}, "columns": {}, "partitions": [],
                                "props": {}, "all": []}}

    old_pp, old_au = qs.PLAN_PILE, qs.AUTHOR
    qs.PLAN_PILE, qs.AUTHOR = _fake_plan_pile, _fake_author
    try:
        qs.apply_plan(stage, ctx, plan, verbose=False)
    finally:
        qs.PLAN_PILE, qs.AUTHOR = old_pp, old_au

    by_path = {(e.get("p") or {}).get("prim_path"): e
              for e in info["elements"]}
    for path in plan["removed"]:
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        if not prim.IsValid() or prim.IsActive():
            problems.append("removed piece not deactivated: {0}".format(path))

    xc = UsdGeom.XformCache()
    for path, spec in plan["displaced"].items():
        e = by_path.get(path)
        if e is None:
            continue
        want = qs.apply_rigid(spec, (float(e["x"]), float(e["y"]), float(e["z"])))
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        if not prim.IsValid():
            problems.append("displaced piece missing from stage: {0}".format(path))
            continue
        got = xc.GetLocalToWorldTransform(prim).ExtractTranslation()
        if any(abs(a - b) > 1e-3 for a, b in zip(want, (got[0], got[1], got[2]))):
            problems.append("displaced piece {0}: predicted {1}, stage says "
                            "{2}".format(path, want, tuple(got)))

    if plan["piles"] and len(ctx.get("rubble") or []) != len(plan["piles"]):
        problems.append("{0} pile(s) planned, {1} recorded in ctx['rubble']"
                        .format(len(plan["piles"]), len(ctx.get("rubble") or [])))

    # export round trip
    from disaster import fire_bake as fb

    fb.strip_physics(stage, root=None, remove_prims=fb.STRIP_PRIMS, verbose=False)
    before = sum(1 for _ in Usd.PrimRange(stage.GetPseudoRoot(),
                                          Usd.PrimAllPrimsPredicate))
    import shutil
    import tempfile

    tmp_dir = tempfile.mkdtemp(prefix="qgac_probe_")
    try:
        out = os.path.join(tmp_dir, "gac_probe_{0}_{1}_s{2}.usd"
                           .format(_safe(style), grade, seed))
        stage.GetRootLayer().Export(out)
        reopened = Usd.Stage.Open(out)
        if reopened is None:
            problems.append("export did not reopen")
        else:
            after = sum(1 for _ in Usd.PrimRange(reopened.GetPseudoRoot(),
                                                 Usd.PrimAllPrimsPredicate))
            if after != before:
                problems.append("prim count changed on export round trip: "
                                "{0} -> {1}".format(before, after))
            info_v = fb.verify_export(out, doomed=("/no/such/path",),
                                      expect_root="/World/cell",
                                      check_remote=False, verbose=False)
            if not info_v.get("ok"):
                problems.append("verify_export: {0}".format(
                    {k: v for k, v in info_v.items()
                     if k.startswith("n_") and v}))
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    return problems, plan


def main(argv):
    name = argv[1] if len(argv) > 1 else "SM_Building_02"
    grade = argv[2] if len(argv) > 2 else "DG5"
    seed = int(argv[3]) if len(argv) > 3 else 7

    if grade not in qs.LEVELS:
        print("[qgac_probe] unknown grade {0!r} (expected one of {1})".format(
            grade, "/".join(qs.LEVELS)))
        return 2

    pls, style, source = get_element_table(name, seed)
    btype = qs.construction_type(name)
    print("[qgac_probe] {0} {1}: {2} piece(s) from {3}, btype={4}".format(
        name, grade, len(pls), source, btype))

    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = btype
    rng = random.Random(seed)
    plan = qs.plan_damage(info, info["elements"], grade, btype, rng)
    print("[qgac_probe] plan: {0}".format(plan["stats"]))
    for n in plan["notes"]:
        print("   note: " + n[:200])

    problems = []

    # plan_to_json / plan_from_json round trip
    try:
        data = qs.plan_to_json(plan)
        json.loads(json.dumps(data))
        restored = qs.plan_from_json(data)
        if restored["removed"] != plan["removed"]:
            problems.append("plan_to_json/plan_from_json did not round-trip "
                            "'removed'")
    except Exception as exc:
        problems.append("plan_to_json/plan_from_json raised: {0}".format(exc))

    # nothing at negative z (pure arithmetic)
    z0 = info["masses"]["main"]["z0"]
    by_path = {(e.get("p") or {}).get("prim_path"): e
              for e in info["elements"]}
    worst = None
    for path, spec in plan["displaced"].items():
        e = by_path.get(path)
        if e is None:
            continue
        _x, _y, nz = qs.apply_rigid(spec, (float(e["x"]), float(e["y"]),
                                           float(e["z"])))
        if worst is None or nz < worst[1]:
            worst = (path, nz)
    if worst is not None:
        print("[qgac_probe] lowest displaced piece after transform: {0} "
              "z={1:.2f} (z0={2:.2f})".format(worst[0], worst[1], z0))
        if worst[1] < z0 - 1.5:
            problems.append("a displaced piece lands {0:.1f} m below z0 "
                            "({1})".format(z0 - worst[1], worst[0]))

    if HAVE_USD:
        stage_problems, _plan2 = run_stage_checks(pls, style, btype, grade, seed)
        problems += stage_problems
    else:
        print("[qgac_probe] pxr not importable -- skipping the bare-stage "
              "apply_plan/export checks")

    print()
    if problems:
        print("[qgac_probe] {0} PROBLEM(S):".format(len(problems)))
        for p in problems:
            print("  - " + str(p))
        return 1
    print("[qgac_probe] OK — {0} {1} clean".format(name, grade))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
