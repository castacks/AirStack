#!/usr/bin/env python
"""tornado_collapse_probe — R11's own container probe (`_plans/
urban_tornado_plan.md` §8c): one `t_facade_collapse` on a REAL
`brownstone_row` kit build (`tornado_kit.wreck_kit`, the exact pattern
`tools/tornado_kit_probe.py` uses), and one `total`-grade industrial
collapse (`tornado_collapse.plan_industrial`/`apply_industrial`) at a REAL
shed's own measured footprint. No piece grid needed for the industrial
half at all — it never touches `tornado_kit`/`quake_flow.describe`.

    docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \\
        bash scene_gen/tools/usd_python.sh \\
        scene_gen/tools/tornado_collapse_probe.py"

    # optional args: SEED BEARING (defaults 7, 35 -- tornado_kit_probe.py's
    # own defaults, so the two probes' facade_collapse half is directly
    # comparable to that file's own brownstone_row/T3 numbers)

Runs in the container ONLY — `kit_substitute.build_kit` references REAL
`omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/...` kit
assets, which need the Nucleus resolver. Never inside a `SimulationApp` —
bare python via `usd_python.sh` (pxr + Nucleus, no Kit).

WHAT IT PRINTS: for the facade_collapse half, the same census/wind/plan/
debris shape `tornado_kit_probe.py` prints, PLUS whether `t_facade_
collapse` actually fired, its region's own side/storey span, the leaning
macroblock count, and the cap-exemption note (or its absence). For the
industrial half: the shed's measured footprint, the plan's own stats
(panels fallen/flat/leaning, roof coverage, joist/contents/rubble counts),
and `apply_industrial`'s authored mesh counts + a debris bbox.

Both probes export to `/isaac-sim/.cache/tornado_probe/` (`TP_OUT` env
overrides), a geometry probe not a bake — same convention as `tornado_kit_
probe.py`'s own export note.
"""
import json
import os
import random
import sys
import time
import traceback
from collections import Counter

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
import numpy as np                                              # noqa: E402
from pxr import Sdf, Usd, UsdGeom                                # noqa: E402

from disaster import kit_substitute as ksub                     # noqa: E402
from disaster import tornado as tn                                # noqa: E402
from disaster import tornado_collapse as tcol                     # noqa: E402
from disaster import tornado_kit as tk                            # noqa: E402

SEED = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1] else 7
BEARING = float(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2] else 35.0
OUT_DIR = os.environ.get("TP_OUT") or "/isaac-sim/.cache/tornado_probe"

# The four chosen sheds (`disaster.tornado_city.INDUSTRIAL_SHED_SUFFIXES`),
# measured (`config/harvested/standalone_buildings.json`). Probing the
# largest footprint, `Building_TypeB_C`, since a bigger shed exercises more
# wall segments / more roof coverage than the smallest.
SHED_USD = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/Stages/"
           "Dmytro/Assets/Game/FactoryDistrict/Meshes/Building_TypeB_C.usd")
SHED_W, SHED_D, SHED_H = 67.1, 45.1, 12.0


def _bbox(stage, path):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                           useExtentsHint=False)
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return None
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    lo, hi = r.GetMin(), r.GetMax()
    return [round(float(v), 2) for v in (lo[0], lo[1], lo[2], hi[0], hi[1], hi[2])]


def _dump_debris_material_bindings(stage, root_path):
    """D4 (round4): the bench-v3 verdict was "roof sheets/joists render
    PURE WHITE untextured; panels flat grey; the source shed bare grey
    concrete". Walk every authored `<root>/*` Mesh, resolve its bound
    material's Shader, and report whether `diffuse_color_constant` reads as
    white -- ground truth for whether `apply_industrial`'s own binding call
    (not `tuw.debris_material`'s colour choices, D's region) is the cause.

    Per the lead's own dichotomy (round4, relayed from D's MDL read): a
    BOUND material whose `diffuse_texture` path fails to RESOLVE falls back
    to `diffuse_color_constant` in OmniPBR -- so a white mesh is either (a)
    not bound to any material at all, or (b) bound to a material whose
    texture path is unresolvable while its constant happens to be light.
    This dump checks BOTH: binding presence AND, for any authored
    `diffuse_texture`, whether the resolved path actually exists on this
    container's filesystem (a local `airstack://`-resolved path; a Nucleus
    `omniverse://` path is reported but not filesystem-checked here).
    Returns `(n_mesh, n_unbound, n_white)`."""
    import os

    from pxr import Usd, UsdGeom, UsdShade
    n_mesh = n_unbound = n_white = 0
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        print("[tcp]   (no {0} to dump)".format(root_path))
        return 0, 0, 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        n_mesh += 1
        mat, _rel = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        if not mat or not mat.GetPrim().IsValid():
            n_unbound += 1
            print("[tcp]   {0} -> ** UNBOUND **".format(prim.GetPath()))
            continue
        sh = UsdShade.Shader.Get(stage, mat.GetPath().AppendChild("Shader"))
        dc = sh.GetInput("diffuse_color_constant").Get() if sh else None
        tex_input = sh.GetInput("diffuse_texture") if sh else None
        tex_val = tex_input.Get() if tex_input else None
        tex_note = ""
        if tex_val:
            # `Sdf.AssetPath.path` is the raw string; `str(Sdf.AssetPath(...))`
            # wraps it in "@...@" layer-path-expression delimiters, which
            # made an earlier draft of this check `os.path.isfile("@/isaac-
            # sim/.../foo.png@")` -- always False regardless of whether the
            # file is really there. Caught by cross-checking against a
            # direct `ls` on the container filesystem (see the report).
            tex_path = tex_val.path if hasattr(tex_val, "path") else str(tex_val)
            if tex_path.startswith("omniverse://"):
                tex_note = "  texture={0} (Nucleus, not filesystem-checked)".format(tex_path)
            else:
                exists = os.path.isfile(tex_path)
                tex_note = "  texture={0} exists_on_disk={1}".format(tex_path, exists)
                if not exists:
                    print("[tcp]   ** WARNING: {0}'s bound texture does NOT "
                          "resolve on this filesystem -- MDL falls back to "
                          "diffuse_color_constant={1} **".format(prim.GetPath(), dc))
        is_white = bool(dc) and all(float(c) > 0.9 for c in dc)
        if is_white:
            n_white += 1
        print("[tcp]   {0} -> {1}  diffuse_color_constant={2}{3}{4}".format(
            prim.GetPath(), mat.GetPath(), dc,
            "  ** WHITE **" if is_white else "", tex_note))
    print("[tcp]   {0}: {1} mesh(es), {2} unbound, {3} white".format(
        root_path, n_mesh, n_unbound, n_white))
    return n_mesh, n_unbound, n_white


def _dump_shed_subset_bindings(stage, src_path):
    """The shed's OWN look, GeomSubset by GeomSubset (a Dmytro merged shed
    is ONE Mesh with per-section GeomSubsets, each independently bound --
    `ComputeBoundMaterial` on the parent Mesh prim itself is legitimately
    empty by this pattern's own design; checking it alone is what made an
    early draft of this diagnostic misreport "0 bound" here, see the round4
    report). Returns `(n_subsets, n_bound, n_textured)`."""
    from pxr import Usd, UsdGeom, UsdShade
    prim = stage.GetPrimAtPath(src_path)
    if not prim or not prim.IsValid():
        print("[tcp]   (no {0} to dump)".format(src_path))
        return 0, 0, 0
    n_subsets = n_bound = n_tex = 0
    for mesh_prim in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if not mesh_prim.IsA(UsdGeom.Mesh):
            continue
        subsets = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh_prim))
        for subset in subsets:
            n_subsets += 1
            smat, _srel = UsdShade.MaterialBindingAPI(
                subset.GetPrim()).ComputeBoundMaterial()
            if not smat or not smat.GetPrim().IsValid():
                print("[tcp]   {0} -> UNBOUND".format(subset.GetPath()))
                continue
            n_bound += 1
            surface_ids, has_tex = [], False
            for p2 in Usd.PrimRange(smat.GetPrim()):
                if not p2.IsA(UsdShade.Shader):
                    continue
                shd = UsdShade.Shader(p2)
                idattr = shd.GetIdAttr()
                if idattr and idattr.Get():
                    surface_ids.append(str(idattr.Get()))
                # A `UsdUVTexture` node's actual image path lives on its
                # OWN `file` input, not on a "diffuse..."-named input on
                # `UsdPreviewSurface` (that one is CONNECTED, not a literal
                # value -- `.Get()` on it is always None, which made an
                # earlier version of this check, keyed only on the name
                # containing "diffuse", wrongly report `has_tex=False` on
                # every subset even when a real texture file was present).
                for inp in shd.GetInputs():
                    nm = inp.GetBaseName().lower()
                    if nm == "file" or "texture" in nm:
                        v = inp.Get()
                        if v:
                            has_tex = True
            if has_tex:
                n_tex += 1
            print("[tcp]   {0} -> {1}  shader_ids={2}  has_diffuse_tex={3}".format(
                subset.GetPath(), smat.GetPath(), surface_ids, has_tex))
    print("[tcp]   {0}: {1} GeomSubset(s), {2} bound, {3} with a diffuse "
          "texture input".format(src_path, n_subsets, n_bound, n_tex))
    return n_subsets, n_bound, n_tex


def _wind(bearing_deg):
    cfg = dict(tn.DEFAULTS)
    cfg.update({"origin_m": [0.0, 60.0], "heading_deg": bearing_deg,
               "width_m": 300.0, "wobble_m": 0.0, "edge_noise_m": 0.0,
               "along_min": 1.0, "width_min": 1.0})
    return tn.wind_at(cfg, 0.0, 0.0)


# ---------------------------------------------------------------------------
# HALF 1 — t_facade_collapse on a real brownstone-family kit build
# ---------------------------------------------------------------------------
# MEASURED, not assumed (first draft of this probe ran "brownstone_row" per
# the work order's own wording and found it REFUSED -- 5 real storeys on a
# 17.0 m building, over the recipe's own <= 4 storey cap; every OTHER
# lowrise kit style checked the same way -- "brownstone" (same family "03",
# 14.0 m), "dw_terrace" (17.5 m), "department_store" (17.0 m) -- measures
# exactly 4 storeys and fires cleanly). `STYLE` is `"brownstone"`, the
# closest same-family style that actually demonstrates the recipe; the
# `brownstone_row` refusal is reported separately, in this function's own
# summary, as the genuinely useful negative finding it is (§8c: "GUARDS...
# refuses otherwise, with a note" -- this IS that note, on real data).
STYLE = "brownstone"
REFUSAL_STYLE = "brownstone_row"


def probe_facade_collapse():
    print("\n[tcp] ==== facade_collapse: {0}, T4, seed {1}, "
         "bearing {2} ====".format(STYLE, SEED, BEARING))
    tk._refuse_if_unsupported(STYLE)

    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    UsdGeom.Scope.Define(st, "/W/bench")
    cell = "/W/bench/fc0"
    UsdGeom.Xform.Define(st, cell)

    btype = ksub.styles()[STYLE]["type"]
    H_expect = ksub.styles()[STYLE]["H"]
    print("[tcp] {0} btype {1} H(expect) {2:.1f} m".format(STYLE, btype, H_expect))

    wind = _wind(BEARING)
    print("[tcp] wind_at -> {0}".format(json.dumps(wind)))

    # T4 at 0.85 -- ABOVE `t_facade_collapse`'s own i >= 0.82 floor
    # (`tornado_kit.LEVEL_INTENSITY["T4"]`), so this level draw is expected
    # to make the recipe eligible on its own (still gated on storeys <= 4
    # and the wind-picked side having bays -- the recipe's own guard, not
    # this probe's business to second-guess).
    intensity = tk.LEVEL_INTENSITY["T4"]
    rng = random.Random(SEED)
    nrng = np.random.default_rng(SEED & 0xFFFFFFFF)

    t0 = time.time()
    ctx = tk.wreck_kit(st, cell, STYLE, "T4", rng, nrng, {}, "tcp", wind,
                       seed=SEED, btype=btype, intensity=intensity,
                       verbose=True)
    t_wreck = time.time() - t0

    plan = ctx.get("plan")
    if plan is None:
        print("[tcp] STOP: wreck_kit produced no plan")
        return None

    placements = [e["p"] for e in ctx["info"]["elements"]]
    print("[tcp] built + adapted {0} piece(s) in {1:.1f} s".format(
        len(placements), t_wreck))
    n_storeys = len(set(int(p.get("_storey", -1)) for p in placements))
    print("[tcp] storeys {0}, height_class {1}".format(
        n_storeys, plan.get("height_class")))

    fc_regions = [r for r in plan.get("regions", []) if r["recipe"] == "facade_collapse"]
    fired = bool(fc_regions)
    print("[tcp] facade_collapse fired: {0}".format(fired))
    if fc_regions:
        r = fc_regions[0]
        print("[tcp]   side {0}, {1} storey(s) {2}, {3} cell(s)".format(
            r["side"], len(r["storeys"]), r["storeys"], len(r["cells"])))
    macros = [m for m in (plan.get("macroblocks") or []) if m.get("recipe") == "facade_collapse"]
    print("[tcp]   {0} leaning macroblock(s), deg {1}".format(
        len(macros), [round(m["deg"], 1) for m in macros]))
    print("[tcp]   roof_shed: {0}".format(plan.get("roof_shed")))
    cap_notes = [n for n in plan.get("notes", []) if "facade_collapse" in n and "exempt" in n]
    print("[tcp]   cap-exemption note present: {0}".format(bool(cap_notes)))
    for n in cap_notes:
        print("[tcp]     {0}".format(n))

    for n in plan.get("notes") or []:
        print("[tcp]   note: {0}".format(n))
    print("[tcp] stats {0}".format(json.dumps(plan.get("stats"), sort_keys=True)))

    deb = plan.get("debris") or []
    if deb:
        kinds = Counter(d.get("kind") for d in deb)
        xs = [d["x"] for d in deb]
        ys = [d["y"] for d in deb]
        print("[tcp] debris {0} fragments {1}; x [{2:.1f}, {3:.1f}] "
              "y [{4:.1f}, {5:.1f}]".format(len(deb), dict(kinds),
                                            min(xs), max(xs), min(ys), max(ys)))
    json.dumps(plan)   # schema contract: JSON-serialisable throughout

    print("[tcp] apply counts {0}".format(json.dumps(ctx["counts"], sort_keys=True)))
    deb_root = cell + "/tornado_debris"
    print("[tcp] debris bbox {0}".format(_bbox(st, deb_root)))
    print("[tcp] cell bbox   {0}".format(_bbox(st, cell)))

    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, "facade_collapse_{0}_s{1}.usd".format(STYLE, SEED))
    st.GetRootLayer().Export(out)
    with open(out[:-4] + ".plan.json", "w") as fh:
        json.dump(plan, fh, indent=1, sort_keys=True)
    print("[tcp] exported {0} ({1:.1f} MB)".format(out, os.path.getsize(out) / 1e6))
    return {"fired": fired, "n_macroblocks": len(macros),
           "cap_exempt_noted": bool(cap_notes), "storeys": n_storeys}


def probe_facade_collapse_refusal():
    """`REFUSAL_STYLE` (`brownstone_row`, 17.0 m) on a REAL build measures
    5 storeys -- over `t_facade_collapse`'s own <= 4 storey cap. No stage
    export needed (this is a guard-refusal check, not a geometry probe);
    same `wreck_kit` call `probe_facade_collapse` makes, on the OTHER
    style."""
    print("\n[tcp] ==== facade_collapse GUARD CHECK: {0}, T4, seed {1} "
         "====".format(REFUSAL_STYLE, SEED))
    tk._refuse_if_unsupported(REFUSAL_STYLE)
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    cell = "/W/fc_refusal0"
    UsdGeom.Xform.Define(st, cell)
    btype = ksub.styles()[REFUSAL_STYLE]["type"]
    wind = _wind(BEARING)
    rng = random.Random(SEED)
    nrng = np.random.default_rng(SEED & 0xFFFFFFFF)
    ctx = tk.wreck_kit(st, cell, REFUSAL_STYLE, "T4", rng, nrng, {}, "tcp_ref",
                       wind, seed=SEED, btype=btype,
                       intensity=tk.LEVEL_INTENSITY["T4"], verbose=False)
    plan = ctx.get("plan")
    placements = [e["p"] for e in ctx["info"]["elements"]]
    n_storeys = len(set(int(p.get("_storey", -1)) for p in placements))
    fired = any(r["recipe"] == "facade_collapse" for r in plan.get("regions", []))
    refusal_notes = [n for n in plan.get("notes", []) if "facade_collapse: refused" in n]
    print("[tcp] {0}: {1} storeys, fired={2}".format(REFUSAL_STYLE, n_storeys, fired))
    for n in refusal_notes:
        print("[tcp]   {0}".format(n))
    return {"style": REFUSAL_STYLE, "storeys": n_storeys, "fired": fired,
           "refusal_notes": refusal_notes}


# ---------------------------------------------------------------------------
# HALF 2 — industrial total collapse at a real shed's measured footprint
# ---------------------------------------------------------------------------
def probe_industrial():
    print("\n[tcp] ==== industrial: {0}, total, seed {1}, bearing {2} "
         "====".format(os.path.basename(SHED_USD), SEED, BEARING))
    print("[tcp] measured footprint {0:.1f} x {1:.1f} x {2:.1f} m".format(
        SHED_W, SHED_D, SHED_H))

    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    UsdGeom.Scope.Define(st, "/W/bench")
    cell = "/W/bench/ind0"
    UsdGeom.Xform.Define(st, cell)

    # The intact shed reference, at `<cell>/src` -- the SAME convention
    # every other sliced-building probe in this pipeline uses
    # (`tools/tornado_kit_probe.py`/`tools/tornado_urban_probe.py`'s own
    # `<cell>/src`), so `apply_industrial`'s own deactivation step has
    # something real to act on here.
    # D4 (round4): the reference goes on a CHILD "asset" prim with NO
    # local typeName authored, not directly on `<cell>/src` itself --
    # `gac_fire.place_source`'s own pattern, matched here on purpose. An
    # earlier draft of this probe called `UsdGeom.Xform.Define(st, cell +
    # "/src")` and then referenced the shed DIRECTLY onto that SAME prim;
    # for `Building_TypeB_C.usd` (whose own default prim IS the mesh, no
    # wrapper) that authors an explicit LOCAL "Xform" typeName which is
    # STRONGER than the referenced asset's own "Mesh" opinion, so the
    # composed prim's resolved type reads back as "Xform" even though its
    # GeomSubsets/geometry are all still there -- `prim.IsA(UsdGeom.Mesh)`
    # (both here and in Hydra) then skips it, which made an early version
    # of the material dump below wrongly report "0 GeomSubsets" for THIS
    # shed specifically (`Building_TypeC_A.usd`, referenced the same way in
    # this file's OTHER probe half, happens to wrap its mesh in a "asset"-
    # named child already, so it did not trip this). `<cell>/src` stays the
    # WRAPPER (`apply_industrial`'s own deactivation target, unchanged);
    # the reference itself moves one level down.
    src_prim = UsdGeom.Xform.Define(st, cell + "/src")
    kid = st.DefinePrim(Sdf.Path(cell + "/src/asset"))
    kid.GetReferences().AddReference(SHED_USD)
    # A payload discovered by an edit made AFTER the stage was already
    # created (this AddReference, on an in-memory stage) is not always
    # auto-loaded -- `gac_fire.place_source`'s own explicit `Stage.Load`
    # call, reused here so this probe does not depend on it happening to
    # already be loaded.
    st.Load(Sdf.Path(cell + "/src"))

    wind = _wind(BEARING)
    print("[tcp] wind_at -> {0}".format(json.dumps(wind)))

    # D4 (round4): the shed's OWN material must be checked BEFORE
    # apply_industrial runs -- it deactivates <cell>/src as part of
    # collapsing the building, and a deactivated prim drops out of default
    # PrimRange traversal (an early draft of this check ran AFTER apply
    # and silently found "0 meshes" here -- a diagnostic-ordering bug, not
    # a real finding; see the round4 report).
    print("[tcp] -- shed's own per-section material --")
    _dump_shed_subset_bindings(st, cell + "/src")

    rng = random.Random(SEED)
    t0 = time.time()
    plan = tcol.plan_industrial(SHED_W, SHED_D, SHED_H, 0.0, 0.0, 0.0,
                                "total", wind, rng)
    t_plan = time.time() - t0
    print("[tcp] planned in {0:.3f} s".format(t_plan))
    print("[tcp] stats {0}".format(json.dumps(plan["stats"], sort_keys=True)))
    for n in plan.get("notes") or []:
        print("[tcp]   note: {0}".format(n))
    json.dumps(plan)   # schema contract: JSON-serialisable throughout

    ctx = {"stage": st, "parent": cell, "mats": {}, "verbose": True}
    t1 = time.time()
    counts = tcol.apply_industrial(st, ctx, plan)
    t_apply = time.time() - t1
    print("[tcp] applied in {0:.3f} s, counts {1}".format(
        t_apply, json.dumps(counts, sort_keys=True)))

    deb_root = cell + "/industrial_debris"
    print("[tcp] debris bbox {0}".format(_bbox(st, deb_root)))
    print("[tcp] cell bbox   {0}".format(_bbox(st, cell)))
    print("[tcp] src active after apply: {0}".format(
        st.GetPrimAtPath(cell + "/src").IsActive()))

    print("[tcp] -- authored debris material bindings --")
    n_mesh, n_unbound, n_white = _dump_debris_material_bindings(st, deb_root)
    assert n_unbound == 0, "D4: an unbound industrial_debris mesh"
    assert n_white == 0, "D4: a white-diffuse industrial_debris mesh"

    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, "industrial_{0}_total_s{1}.usd".format(
        os.path.basename(SHED_USD)[:-4], SEED))
    st.GetRootLayer().Export(out)
    with open(out[:-4] + ".plan.json", "w") as fh:
        json.dump(plan, fh, indent=1, sort_keys=True)
    print("[tcp] exported {0} ({1:.1f} MB)".format(out, os.path.getsize(out) / 1e6))
    return {"n_panels": counts["n_panels"], "n_meshes": counts["n_meshes"],
           "src_deactivated": counts["n_src_deactivated"] == 1}


def main():
    t0 = time.time()
    r1 = probe_facade_collapse()
    r1b = probe_facade_collapse_refusal()
    r2 = probe_industrial()
    print("\n[tcp] ==== SUMMARY ====")
    print("[tcp] facade_collapse ({0}):          {1}".format(STYLE, json.dumps(r1)))
    print("[tcp] facade_collapse guard ({0}): {1}".format(REFUSAL_STYLE, json.dumps(r1b)))
    print("[tcp] industrial:                          {0}".format(json.dumps(r2)))
    print("[tcp] total {0:.1f} s".format(time.time() - t0))


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.exit(1)
