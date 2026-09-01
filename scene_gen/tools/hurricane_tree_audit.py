#!/usr/bin/env python3
"""
hurricane_tree_audit.py — offline audit of every baked hurricane tree
archetype: what is visible, what material it binds, whether every texture
that material names resolves case-exact on THIS machine, height, and
ground-contact via POINT bounds (never `BBoxCache`'s AABB-of-AABB, which
`bake-tree-and-debris/SKILL.md` measured a 6.8x span error from).

Written to answer, for all 34 archetypes at once, the question the cotton-
ball bug raised: "is there a visible prim anywhere binding a material whose
texture will not resolve?" No Isaac Sim; bare `pxr` (this host has
`usd-core` via pip, so it needs no container bootstrap at all).

EXTENDED 2026-08-31 (STREAM T6) to answer a second, harder question: "is
there a visible LEAF/NEEDLE mesh at a DAMAGED level bound to anything other
than a genuine OmniPBR-with-`diffuse_tint` replacement?" -- see
`_is_omnipbr_with_tint`'s docstring for why this checks the shader's own
identity rather than the material's PATH (`/Root/tint_mats/...`): the
defect this stream fixes (a `diffuse_tint` override authored on a COPY of
the source's own custom-MDL material, which never reached the render) lived
at that exact path too.

Usage:
    python3 scene_gen/tools/hurricane_tree_audit.py [--dir DIR] [--json OUT]

Prints one line per archetype and a final PASS/FAIL summary; writes the full
per-archetype detail (visible/hidden prim counts, foliage fraction, height,
min/max world z, and every visible mesh's material + texture resolution) to
`--json` if given.
"""
import argparse
import json
import os
import sys

os.environ.setdefault("PXR_USDC_EMIT_DEPRECATION_WARNINGS", "0")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bake_hurricane_trees as bht  # noqa: E402

import numpy as np  # noqa: E402
from pxr import Sdf, Usd, UsdGeom, UsdShade  # noqa: E402

REPO = bht.REPO


def _all_relative_asset_paths(stage):
    """`[(prim_path, attr_name, path_str, resolved_str)]` for EVERY
    attribute, on EVERY prim (any type -- Material, Shader, NodeGraph,
    Mesh...), anywhere in the COMPOSED archetype stage, whose value is an
    `Sdf.AssetPath` with a RELATIVE `path` (an absolute filesystem path or
    a URL-schemed one, e.g. `omniverse://...`, is skipped -- neither can be
    verified against local disk, and neither is touched by the class of bug
    this exists to catch).

    THIS IS THE COMPOSED STAGE'S OWN RESOLUTION, not a hand-rolled
    `os.path.join` reconstruction of "what the anchor SHOULD be" the way
    `bake_hurricane_trees._material_ok` does at bake time -- `Usd.Stage.
    Open` + `Sdf.AssetPath.resolvedPath` on the SAVED archetype file is
    exactly what Hydra/Kit itself resolves against, so an empty
    `resolvedPath` here reproduces the launcher's own Hydra log line
    (`References an asset that can not be found: '<path>'`) verbatim,
    directly on the archetype -- no render needed. Confirmed against the
    pre-fix archetype for `Common_Apple`: this walk finds `/Root/tint_mats/
    Apple_leaf_Mat_Apple_leaf_Mat`'s `inputs:diffuse_texture` at
    `./materials/textures/hollyprivet_basecolor.png` with `resolvedPath ==
    ''`, the exact defect `bake_hurricane_trees._copy_tinted_material`'s
    `_reanchor_relative_assets` fixes.
    """
    out = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        for attr in prim.GetAttributes():
            if not attr.HasAuthoredValue():
                continue
            v = attr.Get()
            vals = v if isinstance(v, (list, tuple)) else [v]
            for item in vals:
                if not isinstance(item, Sdf.AssetPath) or not item.path:
                    continue
                # STREAM T6: delegates to `bht._is_relative_asset_path`
                # rather than a hand-rolled "://"/isabs check, so this walk
                # skips the SAME things the bake's own resolver does --
                # including a bare built-in Kit MDL module name
                # (`OmniPBR.mdl`, what `bake_hurricane_trees.
                # _omnipbr_leaf_material`'s fresh Shader authors, STREAM T6)
                # which is resolved through Kit's MDL search path, not
                # anchored next to this asset, and so is NEVER expected to
                # have a `resolvedPath` on a bare `pxr` with no Kit MDL
                # resolver plugin loaded -- flagging it here would be a
                # false positive on every correctly-authored OmniPBR
                # replacement, not a real defect.
                if not bht._is_relative_asset_path(item.path):
                    continue  # absolute/URL/built-in-MDL -- not ours to verify offline
                out.append((str(prim.GetPath()), attr.GetName(),
                           item.path, item.resolvedPath))
    return out


DEFAULT_DIR = os.path.join(REPO, "scene_gen", "assets", "archetypes_hurricane")


def _species_for(fn):
    stem = fn[len("tree_"):-len(".usd")] if fn.startswith("tree_") else fn
    for s in bht.TREE_SPECIES:
        if stem.startswith(s + "_"):
            return s, stem[len(s) + 1:]
    return None, None


def _world_point_bounds(stage):
    """`(zmin, zmax, n_visible_mesh_prims, n_hidden_mesh_prims)` using
    ACTUAL TRANSFORMED POINTS of every Mesh prim, never `BBoxCache` (see
    module docstring for why that undercounts/overcounts on this asset
    family's per-prim `xformOp:transform`s).
    """
    xcache = UsdGeom.XformCache()
    zmin, zmax = None, None
    n_vis = n_hid = 0
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.GetTypeName() != "Mesh":
            continue
        invisible = (UsdGeom.Imageable(prim).ComputeVisibility()
                    == UsdGeom.Tokens.invisible)
        if invisible:
            n_hid += 1
            continue
        pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if not pts or len(pts) == 0:
            continue
        n_vis += 1
        m = np.array(xcache.GetLocalToWorldTransform(prim))
        arr = np.array(pts, dtype=np.float64)
        world = np.hstack([arr, np.ones((len(arr), 1))]) @ m
        lo, hi = float(world[:, 2].min()), float(world[:, 2].max())
        zmin = lo if zmin is None else min(zmin, lo)
        zmax = hi if zmax is None else max(zmax, hi)
    return zmin, zmax, n_vis, n_hid


def _scope_of(prim_path_str):
    """'/Root/top' or '/Root/src' -- whichever this prim lives under, for
    per-scope fault reporting on a `snapped` archetype's two independent
    references (see `_material_report`'s docstring). Anything under
    neither (`/Root/tint_mats`, `/Root/safe_mats`) reports "other".
    """
    if prim_path_str.startswith("/Root/top/") or prim_path_str == "/Root/top":
        return "/Root/top"
    if prim_path_str.startswith("/Root/src/") or prim_path_str == "/Root/src":
        return "/Root/src"
    return "other"


def _is_omnipbr_with_tint(mat):
    """True if `mat`'s Shader IS a plain OmniPBR -- not one of this asset
    family's custom per-species MDL subgraphs that merely RE-EXPORTS
    OmniPBR's own parameter list (`export material X(*) = OmniPBR(...)`,
    subIdentifier `.::materials::X_leaf::X_leaf`, never `OmniPBR` itself) --
    AND carries an authored `diffuse_tint` input (STREAM T6).

    WHY THIS REPLACES THE OLD PATH-CONVENTION-ONLY CHECK. The STREAM T5
    defect this bake now fixes (see `bake_hurricane_trees`'s own "THE TINT
    WAS A DEAD KNOB" docstring section) was a `diffuse_tint` OVERRIDE
    authored on a COPY of the source species' own custom subIdentifier --
    that copy lived at the exact same `/Root/tint_mats/...` path a correct
    fix does, and it DID carry an authored `diffuse_tint` input, so a check
    that only asked "is this under `/Root/tint_mats/`?" (this function's
    predecessor) would NOT have caught a regression back to that exact
    shape -- it would have kept reporting "OK" on the precise defect this
    whole stream exists to close. This checks the SHADER'S OWN IDENTITY
    instead: `info:id == "OmniPBR"` (what `bake_hurricane_trees.
    _omnipbr_leaf_material`'s `CreateIdAttr` always authors) or an
    `info:mdl:sourceAsset` whose basename is `OmniPBR.mdl` (what its
    `SetSourceAsset` always authors) -- either is sufficient to prove this
    is NOT a custom species subgraph, which is never named `OmniPBR`.
    """
    if not mat:
        return False
    for shp in Usd.PrimRange(mat.GetPrim()):
        if shp.GetTypeName() != "Shader":
            continue
        shd = UsdShade.Shader(shp)
        is_omnipbr = False
        id_attr = shp.GetAttribute("info:id")
        if id_attr and id_attr.Get() == "OmniPBR":
            is_omnipbr = True
        src_attr = shp.GetAttribute("info:mdl:sourceAsset")
        if src_attr and src_attr.Get():
            if os.path.basename(str(src_attr.Get().path)).lower() == "omnipbr.mdl":
                is_omnipbr = True
        if not is_omnipbr:
            continue
        tint_inp = shd.GetInput("diffuse_tint")
        if tint_inp and tint_inp.GetAttr().HasAuthoredValue():
            return True
    return False


def _material_report(stage, species_src_abs, arch_abs, level):
    """For every VISIBLE mesh: its path, bound material path (or "" if
    none), whether it resolves, and (Correction 1c) whether a LEAF-bound
    mesh at a DAMAGED level is still bound to the raw, untouched original
    material instead of a tinted copy or the texture-free safety colour.
    Root-of-resolution depends on where the material lives -- see
    `_material_ok`'s callers in `test_hurricane_trees.py` for the same
    split.

    WALKS THE WHOLE COMPOSED STAGE -- for a `snapped` archetype this
    already covers BOTH `/Root/src` (the stump, first reference) AND
    `/Root/top` (the severed top, second reference) with the IDENTICAL
    check, since `Usd.PrimRange(stage.GetPseudoRoot())` does not stop at
    any one subtree. `_scope_of` is used only to LABEL which side of the
    snag a fault came from, not to decide whether to look.
    """
    rows = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.GetTypeName() != "Mesh":
            continue
        if UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        prim_path = str(prim.GetPath())
        scope = _scope_of(prim_path)
        mat = bht._bound_material(prim)
        if not mat:
            rows.append({"prim": prim_path, "scope": scope, "material": None,
                        "resolvable": False, "note": "NO MATERIAL BOUND",
                        "untinted_leaf": False})
            continue
        mp = str(mat.GetPath())
        root = arch_abs if ("/tint_mats/" in mp or "/safe_mats/" in mp) else species_src_abs
        ok = bool(root) and bht._material_ok(root, mat)
        # CORRECTION 1c, STRENGTHENED (STREAM T6) -- "never the untouched
        # green leaf material, AND never a copy of the untouched custom-MDL
        # material either". A mesh classified as foliage by its bound
        # material's NAME (the same `_is_foliage` rule the bake itself
        # uses) that is VISIBLE at a damaged (non-`pristine`) level must be
        # bound to a genuine OmniPBR-with-`diffuse_tint` replacement (see
        # `_is_omnipbr_with_tint`'s own docstring for why a PATH-convention
        # check alone -- "does it live under /Root/tint_mats/?" -- is not
        # enough: the STREAM T5 defect this whole stream fixes lived there
        # too) or the texture-free `/Root/safe_mats/...` constant fallback
        # -- never the raw original custom-MDL material, tinted-copy or
        # not, straight out of the referenced species asset. This is
        # INDEPENDENT of `resolvable`: the raw original leaf material
        # usually resolves JUST FINE (its texture is real and present), so
        # a resolvability-only check cannot see this fault class at all --
        # it is a "resolves, but is the wrong (green, or green-in-practice
        # because the tint never reached the render) material" bug, not a
        # "does not resolve" one.
        is_leaf = bht._is_foliage(prim)
        untinted_leaf = (is_leaf and level != "pristine"
                         and "/safe_mats/" not in mp
                         and not _is_omnipbr_with_tint(mat))
        note_parts = []
        if not ok:
            note_parts.append("unresolvable texture (resolved against {0})".format(root))
        if untinted_leaf:
            note_parts.append("LEAF material at damaged level '{0}' ({1}) is not "
                              "an OmniPBR shader with an authored diffuse_tint "
                              "input -- expected the STREAM T6 replacement under "
                              "/Root/tint_mats/ or a /Root/safe_mats/ fallback"
                              .format(level, mp))
        rows.append({"prim": prim_path, "scope": scope, "material": mp,
                    "resolvable": ok, "untinted_leaf": untinted_leaf,
                    "note": "; ".join(note_parts)})
    return rows


def audit_one(path, species, level):
    stage = Usd.Stage.Open(path)
    zmin, zmax, n_vis, n_hid = _world_point_bounds(stage)
    species_src_abs = os.path.join(REPO, bht.TREE_SPECIES[species]) if species else None
    mat_rows = _material_report(stage, species_src_abs, path, level)
    n_bad_mat = sum(1 for r in mat_rows if not r["resolvable"])
    n_untinted_leaf = sum(1 for r in mat_rows if r["untinted_leaf"])
    # Per-scope breakdown (Correction 1c: "walk the second reference exactly
    # like the first") -- a `snapped` archetype's stump (`/Root/src`) and
    # severed top (`/Root/top`) are two INDEPENDENT reference sites into the
    # same source asset, and a fault that is specific to the second one
    # (e.g. the reviewer's "second reference re-anchors differently"
    # concern) would be invisible in a single pooled count. Any future
    # regression is reported against the scope it actually lives in.
    by_scope = {}
    for r in mat_rows:
        s = by_scope.setdefault(r["scope"], {"bad_mat": 0, "untinted_leaf": 0})
        if not r["resolvable"]:
            s["bad_mat"] += 1
        if r["untinted_leaf"]:
            s["untinted_leaf"] += 1
    # Deliverable #3's own audit assertion: EVERY relative Asset input in
    # the COMPOSED archetype resolves on disk -- not just the ones reached
    # by walking from a VISIBLE mesh's bound material (`_material_report`
    # above), which is exactly the path the hollyprivet defect slipped
    # through (a `tint_mats` copy's Material-prim-level texture override,
    # found by `_all_relative_asset_paths`'s whole-stage walk regardless of
    # which mesh, visible or not, ends up bound to it).
    asset_rows = _all_relative_asset_paths(stage)
    bad_assets = [(p, a, rel) for p, a, rel, resolved in asset_rows if not resolved]
    return {
        "path": path,
        "level": level,
        "height_m": (zmax - min(0.0, zmin)) if (zmax is not None and zmin is not None) else None,
        "min_z_m": zmin,
        "max_z_m": zmax,
        "n_mesh_visible": n_vis,
        "n_mesh_hidden": n_hid,
        "materials": mat_rows,
        "n_unresolvable_materials": n_bad_mat,
        "n_untinted_leaf_at_damaged_level": n_untinted_leaf,
        "by_scope": by_scope,
        "n_asset_paths_checked": len(asset_rows),
        "unresolved_asset_paths": bad_assets,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=DEFAULT_DIR)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    if not os.path.isdir(args.dir):
        print("no such directory: {0}".format(args.dir))
        return 1

    files = sorted(f for f in os.listdir(args.dir)
                   if f.startswith("tree_") and f.endswith(".usd"))
    if not files:
        print("no tree_*.usd archetypes found in {0}".format(args.dir))
        return 1

    report = {}
    n_bad_total = 0
    n_bad_assets_total = 0
    n_untinted_leaf_total = 0
    for fn in files:
        species, level = _species_for(fn)
        path = os.path.join(args.dir, fn)
        info = audit_one(path, species, level)
        report[fn] = info
        n_bad_assets = len(info["unresolved_asset_paths"])
        n_untinted = info["n_untinted_leaf_at_damaged_level"]
        flag = ("OK" if info["n_unresolvable_materials"] == 0 and n_bad_assets == 0
                and n_untinted == 0 else "FAULT")
        n_bad_total += info["n_unresolvable_materials"]
        n_bad_assets_total += n_bad_assets
        n_untinted_leaf_total += n_untinted
        h = "{0:6.2f}".format(info["height_m"]) if info["height_m"] is not None else "  n/a "
        zmin = "{0:6.2f}".format(info["min_z_m"]) if info["min_z_m"] is not None else "  n/a "
        scope_bits = " ".join(
            "{0}[mat_bad={1} untinted={2}]".format(s, v["bad_mat"], v["untinted_leaf"])
            for s, v in sorted(info["by_scope"].items()))
        print("  {0:8s} {1:38s} h={2}m minz={3}m vis={4:3d} hid={5:3d} "
              "mats_bad={6} untinted_leaf={7} assets_bad={8}/{9}  {10}".format(
                  flag, fn, h, zmin, info["n_mesh_visible"], info["n_mesh_hidden"],
                  info["n_unresolvable_materials"], n_untinted, n_bad_assets,
                  info["n_asset_paths_checked"], scope_bits))
        if info["n_unresolvable_materials"] or n_untinted:
            for r in info["materials"]:
                if not r["resolvable"] or r["untinted_leaf"]:
                    print("      FAULT  [{0}] {1}  material={2}  {3}".format(
                        r["scope"], r["prim"], r["material"], r["note"]))
        for prim_path, attr_name, rel_path in info["unresolved_asset_paths"]:
            print("      FAULT  {0}.{1} = '{2}' does not resolve on disk"
                  .format(prim_path, attr_name, rel_path))

    print()
    print("{0} archetypes audited, {1} unresolvable material binding(s), "
          "{2} untinted-leaf-at-damaged-level fault(s), "
          "{3} unresolvable relative Asset path(s) total".format(
              len(files), n_bad_total, n_untinted_leaf_total, n_bad_assets_total))

    if args.json:
        with open(args.json, "w") as f:
            json.dump(report, f, indent=2)
        print("full report -> {0}".format(args.json))

    return 1 if (n_bad_total or n_bad_assets_total or n_untinted_leaf_total) else 0


if __name__ == "__main__":
    sys.exit(main())
