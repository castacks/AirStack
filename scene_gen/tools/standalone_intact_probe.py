#!/usr/bin/env python
"""standalone_intact_probe.py — factual kit-vs-monolith + texture survey of
the 31 standalone "intact" buildings under

    omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/
        scene_gen/assets/standalone/buildings/intact/{midrise,rowhouse,tower}/

Each asset is `<pool>/<name>/<name>.usdc` plus, usually, a sibling
`<name>/textures/` folder. The ONE question this answers per asset: is it a
KIT of parts (separate per-storey / per-wall / per-window meshes with
identity, like the ModernCityEnvironment facade kit `mono_parts_probe.py`
inspects) or a single merged mesh — and do its textures actually resolve.

Bare pxr + omni.client, NO Kit, NO SimulationApp, NO GPU — safe beside a
running sim:

    scene_gen/tools/_t_pxr.sh scene_gen/tools/standalone_intact_probe.py

Each `Usd.Stage.Open()` on a Nucleus asset can take 5-20 s; this prints one
line per asset as it finishes so progress is visible, then writes the full
per-asset record to `scene_gen/_plans/standalone_intact_probe.json` and a
markdown table to `scene_gen/_plans/standalone_intact_probe.md`. Every asset
is wrapped in its own try/except — one bad open does not stop the survey.

Texture resolution follows the pattern already proven in this repo
(`disaster/bake.py._reanchor_assets`, `disaster/quake_rubble_usd.py
._reference_diffuse_texture`): `Usd.Attribute.Get()` on an Asset-typed input
returns an `Sdf.AssetPath` whose `.resolvedPath` is already anchored against
the layer that authored it — for a Nucleus-opened stage this comes back as an
absolute `omniverse://...` URL with no path-joining of our own needed. Only
when that is empty (no active resolver context) do we fall back to
`Sdf.ComputeAssetPathRelativeToLayer` against the strongest layer in the
attribute's property stack. Existence is then checked with `omni.client.stat`.
"""
import json
import os
import re
import sys
import time
import traceback

from pxr import Usd, UsdGeom, UsdShade, Sdf
import omni.client as oc

ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
        "scene_gen/assets/standalone/buildings/intact/")
POOLS = ("midrise", "rowhouse", "tower")

REPO = "/isaac-sim/AirStack"
JSON_OUT = os.path.join(REPO, "scene_gen/_plans/standalone_intact_probe.json")
MD_OUT = os.path.join(REPO, "scene_gen/_plans/standalone_intact_probe.md")

# Names that read as auto-generated / undifferentiated (DCC export defaults),
# as opposed to authored per-part identity.
GENERIC_RE = re.compile(
    r'^(mesh|geom|geometry|shape|object|group|node|polysurface|pcube|pplane|'
    r'default|model|part|piece|element|item)[_.]?\d*$', re.I)
# Substrings that indicate a semantically-named building PART (kit evidence).
KIT_KEYWORDS = (
    "wall", "window", "win", "door", "floor", "storey", "story", "level",
    "roof", "balcony", "trim", "frame", "glass", "sill", "parapet",
    "cornice", "facade", "pillar", "column", "beam", "stair", "awning",
    "chimney", "vent", "ledge", "base", "corner", "panel", "brick", "pipe",
    "fence", "porch", "step", "railing", "canopy", "shutter", "signage",
    "sign", "gutter", "downpipe", "eave", "unit", "ac_")


def _discover():
    """List the `<pool>/<name>/<name>.usdc` assets under ROOT (should be 31:
    midrise/12, rowhouse/7, tower/12)."""
    assets = []
    for pool in POOLS:
        pool_url = ROOT + pool + "/"
        r, ents = oc.list(pool_url)
        if r != oc.Result.OK:
            print("!! list failed for pool {0}: {1}".format(pool, r))
            continue
        for e in sorted(ents, key=lambda e: e.relative_path):
            if not (e.flags & oc.ItemFlags.CAN_HAVE_CHILDREN):
                continue
            name = e.relative_path
            usdc = pool_url + name + "/" + name + ".usdc"
            assets.append((pool, name, usdc))
    return assets


def _textures_dir_present(pool, name):
    r, ents = oc.list(ROOT + pool + "/" + name + "/")
    if r != oc.Result.OK:
        return None
    for e in ents:
        if e.relative_path.lower() == "textures" and (e.flags & oc.ItemFlags.CAN_HAVE_CHILDREN):
            return True
    return False


def _identity_verdict(mesh_paths):
    """Do the mesh prim names carry per-part identity (kit) or read as a
    single / generic merged blob?"""
    names = [p.rsplit("/", 1)[-1] for p in mesh_paths]
    n = len(names)
    if n == 0:
        return False, "no meshes"
    if n == 1:
        return False, "single mesh: '{0}'".format(names[0])
    stems, kw_hits, generic = set(), set(), 0
    for nm in names:
        if GENERIC_RE.match(nm):
            generic += 1
        stems.add(re.sub(r'[\d_.]+$', '', nm).lower())
        low = nm.lower()
        for kw in KIT_KEYWORDS:
            if kw in low:
                kw_hits.add(kw)
    has_identity = (len(kw_hits) >= 2) or (len(stems) >= 3 and generic < n * 0.5)
    sample = ", ".join(names[:6])
    if has_identity:
        ev = "named parts e.g. [{0}]".format(sample)
        if kw_hits:
            ev += "; keywords seen: {0}".format(",".join(sorted(kw_hits)))
    else:
        ev = "generic/undifferentiated names e.g. [{0}]".format(sample)
    return has_identity, ev


def _resolve_asset(attr, val):
    """Best-effort absolute path for an Sdf.AssetPath value authored on
    `attr`. See module docstring for why `.resolvedPath` is preferred."""
    resolved = getattr(val, "resolvedPath", "") or ""
    if resolved:
        return resolved
    raw = getattr(val, "path", "") or ""
    if not raw:
        return ""
    try:
        stack = attr.GetPropertyStack(Usd.TimeCode.Default())
    except Exception:
        stack = []
    layer = stack[0].layer if stack else attr.GetStage().GetRootLayer()
    try:
        return Sdf.ComputeAssetPathRelativeToLayer(layer, raw)
    except Exception:
        return raw


def _material_report(mat_prim):
    """shader id(s) + every asset-path ("texture") input reachable from a
    Material prim's shader network, each resolved and existence-checked."""
    shader_ids = []
    textures = []
    for c in Usd.PrimRange(mat_prim):
        if not c.IsA(UsdShade.Shader):
            continue
        sh = UsdShade.Shader(c)
        sid = sh.GetIdAttr().Get() or ""
        mdl_src_attr = c.GetAttribute("info:mdl:sourceAsset")
        if mdl_src_attr and mdl_src_attr.IsValid():
            v = mdl_src_attr.Get()
            if isinstance(v, Sdf.AssetPath) and (v.path or v.resolvedPath):
                sub = c.GetAttribute("info:mdl:sourceAsset:subIdentifier")
                subid = sub.Get() if (sub and sub.IsValid()) else None
                base = os.path.basename(v.resolvedPath or v.path)
                sid = "mdl:" + base + (":" + subid if subid else "")
        if sid:
            shader_ids.append(str(sid))
        for attr in c.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            if attr.GetName() == "info:mdl:sourceAsset":
                continue  # the .mdl file itself, not a texture
            v = attr.Get()
            if not isinstance(v, Sdf.AssetPath) or not (v.path or v.resolvedPath):
                continue
            resolved = _resolve_asset(attr, v)
            found = None
            if resolved:
                try:
                    r = oc.stat(resolved)
                    found = (r[0] == oc.Result.OK)
                except Exception:
                    found = False
            textures.append({
                "shader": str(c.GetPath()), "input": attr.GetName(),
                "raw": v.path, "resolved": resolved, "found": found,
            })
    return shader_ids, textures


def probe(pool, name, url):
    t0 = time.time()
    rec = {"pool": pool, "name": name, "url": url}
    rec["textures_dir_present"] = _textures_dir_present(pool, name)

    stage = Usd.Stage.Open(url)
    if stage is None:
        rec["error"] = "Stage.Open returned None"
        return rec
    rec["open_s"] = round(time.time() - t0, 1)

    dp = stage.GetDefaultPrim()
    rec["default_prim"] = str(dp.GetPath()) if (dp and dp.IsValid()) else None
    rec["mpu"] = UsdGeom.GetStageMetersPerUnit(stage)
    try:
        rec["up_axis"] = str(UsdGeom.GetStageUpAxis(stage))
    except Exception:
        rec["up_axis"] = None

    root_for_children = dp if (dp and dp.IsValid()) else stage.GetPseudoRoot()
    rec["top_level_children"] = [c.GetName() for c in root_for_children.GetChildren()]

    n_mesh = n_xform = n_instance = n_ref_payload = 0
    total_points = 0
    total_faces = 0
    mesh_paths = []
    unbound_meshes = 0
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        try:
            if prim.IsInstance():
                n_instance += 1
        except Exception:
            pass
        try:
            if prim.HasAuthoredReferences() or prim.HasAuthoredPayloads():
                n_ref_payload += 1
        except Exception:
            pass
        if prim.IsA(UsdGeom.Xform):
            n_xform += 1
        if prim.IsA(UsdGeom.Mesh):
            n_mesh += 1
            mesh_paths.append(str(prim.GetPath()))
            me = UsdGeom.Mesh(prim)
            pts = me.GetPointsAttr().Get()
            if pts:
                total_points += len(pts)
            counts = me.GetFaceVertexCountsAttr().Get()
            if counts:
                total_faces += len(counts)
            # material binding: direct, else ALL GeomSubsets must resolve
            direct = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            direct_ok = bool(direct and direct.GetPrim().IsValid())
            if not direct_ok:
                subsets = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
                if subsets:
                    any_bound = False
                    for s in subsets:
                        m = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
                        if m and m.GetPrim().IsValid():
                            any_bound = True
                            break
                    if not any_bound:
                        unbound_meshes += 1
                else:
                    unbound_meshes += 1

    rec["n_meshes"] = n_mesh
    rec["n_xforms"] = n_xform
    rec["n_instances"] = n_instance
    rec["n_ref_or_payload"] = n_ref_payload
    rec["total_points"] = total_points
    rec["total_faces"] = total_faces
    rec["first_12_mesh_paths"] = mesh_paths[:12]
    rec["unbound_meshes"] = unbound_meshes

    has_identity, evidence = _identity_verdict(mesh_paths)
    rec["parts_have_identity"] = has_identity
    rec["identity_evidence"] = evidence

    # world-space bounding box, in metres
    try:
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_],
                                useExtentsHint=True)
        target = dp if (dp and dp.IsValid()) else stage.GetPseudoRoot()
        box = bc.ComputeWorldBound(target)
        rng = box.ComputeAlignedRange()
        lo, hi = rng.GetMin(), rng.GetMax()
        mpu = rec["mpu"] or 1.0
        rec["bbox_m"] = {
            "W": round((hi[0] - lo[0]) * mpu, 2),
            "D": round((hi[1] - lo[1]) * mpu, 2),
            "H": round((hi[2] - lo[2]) * mpu, 2),
        }
    except Exception as exc:
        rec["bbox_m"] = None
        rec["bbox_error"] = "{0}: {1}".format(type(exc).__name__, exc)

    # materials + textures
    materials = []
    n_tex_found = n_tex_missing = 0
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if not prim.IsA(UsdShade.Material):
            continue
        shader_ids, textures = _material_report(prim)
        for t in textures:
            if t["found"] is True:
                n_tex_found += 1
            elif t["found"] is False:
                n_tex_missing += 1
        materials.append({
            "path": str(prim.GetPath()),
            "shader_ids": shader_ids,
            "textures": textures,
        })
    rec["n_materials"] = len(materials)
    rec["materials"] = materials
    rec["n_textures_found"] = n_tex_found
    rec["n_textures_missing"] = n_tex_missing

    if has_identity:
        verdict = "kit-like (parts with identity)"
    elif n_tex_missing > 0:
        verdict = "merged monolith, textures missing"
    else:
        verdict = "merged monolith"
    rec["verdict"] = verdict
    rec["probe_s"] = round(time.time() - t0, 1)
    return rec


def _fmt_row(rec):
    pool = rec.get("pool", "?")
    name = rec.get("name", "?")
    if "error" in rec:
        return "{0:<10} {1:<22} ERROR: {2}".format(pool, name, rec["error"])
    bbox = rec.get("bbox_m") or {}
    dims = ("{0}x{1}x{2}".format(bbox.get("W"), bbox.get("D"), bbox.get("H"))
            if bbox else "?")
    return ("{0:<10} {1:<22} meshes={2:<5} pts={3:<8} {4:<18} mats={5:<3} "
            "tex {6}/{7}  ({8:.1f}s)  {9}").format(
        pool, name, rec.get("n_meshes", 0), rec.get("total_points", 0), dims,
        rec.get("n_materials", 0), rec.get("n_textures_found", 0),
        rec.get("n_textures_missing", 0), rec.get("wall_s", 0.0),
        rec.get("verdict", "?"))


def _write_markdown(results):
    n_kit = sum(1 for r in results if r.get("verdict", "").startswith("kit-like"))
    n_merged = sum(1 for r in results if r.get("verdict", "").startswith("merged"))
    n_err = sum(1 for r in results if "error" in r)
    n_tex_missing_assets = sum(1 for r in results if r.get("n_textures_missing", 0) > 0)
    total_tex_found = sum(r.get("n_textures_found", 0) for r in results)
    total_tex_missing = sum(r.get("n_textures_missing", 0) for r in results)
    n_has_texdir = sum(1 for r in results if r.get("textures_dir_present") is True)
    n_no_texdir = sum(1 for r in results if r.get("textures_dir_present") is False)

    abs_paths = rel_paths = 0
    for r in results:
        for m in r.get("materials", []):
            for t in m.get("textures", []):
                raw = t.get("raw", "") or ""
                if not raw:
                    continue
                if "://" in raw or raw.startswith("/"):
                    abs_paths += 1
                else:
                    rel_paths += 1

    lines = []
    lines.append("# standalone/buildings/intact — kit-vs-monolith + texture survey")
    lines.append("")
    lines.append(
        "{0} of {1} probed assets ({2} error(s)) are **kit-like** (mesh "
        "prims carry per-part identity: named walls/windows/floors/etc.); "
        "**{3} are merged monoliths** (one mesh, or several meshes with no "
        "authored part identity).".format(n_kit, len(results), n_err, n_merged))
    lines.append(
        "Textures: {0} resolved, {1} missing across {2} texture input(s) "
        "total; {3} of {4} assets have at least one missing texture. "
        "{5} asset(s) have their own `<name>/textures/` sibling folder next "
        "to `<name>.usdc`, {6} do not.".format(
            total_tex_found, total_tex_missing,
            total_tex_found + total_tex_missing,
            n_tex_missing_assets, len(results), n_has_texdir, n_no_texdir))
    lines.append(
        "Path style: {0} texture input(s) authored as a path RELATIVE to "
        "the asset's own `.usdc` layer (e.g. `textures/foo.png`), {1} "
        "authored as an absolute/scheme path; `Sdf.AssetPath.resolvedPath` "
        "anchors either case to an `omniverse://airlab-nucleus...` URL with "
        "no manual path-joining needed.".format(rel_paths, abs_paths))
    lines.append(
        "Full per-asset detail (top-level children, first 12 mesh paths, "
        "per-material shader id + every texture input with its resolved "
        "path) is in `standalone_intact_probe.json`.")
    lines.append("")
    lines.append("| pool | name | meshes | points | W x D x H (m) | parts have identity? | materials | textures found/missing | verdict |")
    lines.append("|---|---|---:|---:|---|---|---:|---|---|")
    for r in sorted(results, key=lambda r: (r.get("pool", ""), r.get("name", ""))):
        if "error" in r:
            lines.append("| {0} | {1} | - | - | - | - | - | - | ERROR: {2} |".format(
                r.get("pool", "?"), r.get("name", "?"), r["error"]))
            continue
        bbox = r.get("bbox_m") or {}
        dims = ("{0} x {1} x {2}".format(bbox.get("W"), bbox.get("D"), bbox.get("H"))
                if bbox else "?")
        identity = ("yes — {0}".format(r["identity_evidence"]) if r["parts_have_identity"]
                    else "no — {0}".format(r["identity_evidence"]))
        tex = "{0}/{1}".format(r.get("n_textures_found", 0), r.get("n_textures_missing", 0))
        lines.append("| {0} | {1} | {2} | {3} | {4} | {5} | {6} | {7} | {8} |".format(
            r["pool"], r["name"], r["n_meshes"], r["total_points"], dims,
            identity, r["n_materials"], tex, r["verdict"]))

    with open(MD_OUT, "w") as fh:
        fh.write("\n".join(lines) + "\n")


def main():
    assets = _discover()
    print("discovered {0} assets across pools {1}".format(len(assets), POOLS))
    sys.stdout.flush()
    results = []
    for pool, name, url in assets:
        t0 = time.time()
        try:
            rec = probe(pool, name, url)
        except Exception as exc:
            rec = {"pool": pool, "name": name, "url": url,
                   "error": "{0}: {1}".format(type(exc).__name__, exc),
                   "traceback": traceback.format_exc()}
        rec["wall_s"] = round(time.time() - t0, 1)
        results.append(rec)
        print(_fmt_row(rec))
        sys.stdout.flush()

    os.makedirs(os.path.dirname(JSON_OUT), exist_ok=True)
    with open(JSON_OUT, "w") as fh:
        json.dump(results, fh, indent=1)
    print("\n-> {0}".format(JSON_OUT))

    _write_markdown(results)
    print("-> {0}".format(MD_OUT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
