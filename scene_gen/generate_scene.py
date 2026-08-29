"""
generate_scene.py — the detailed-city entry point.

Composes the existing generator with the three new passes. `scene_generator` is
imported, never modified: `build_city`, `apply_placements` and
`apply_ground_planes` do exactly what they do today, and the new work is
interleaved around them.

    build_city                 roads, blocks, buildings, parks, cars   (unchanged)
      + districts.remap        swap building art per ring              (NEW)
      + city_detail.build      zoned street furniture                  (NEW)
    apply_placements                                                   (unchanged)
    apply_ground_planes        asphalt, grass, lane dashes             (unchanged)
      + road_markings.apply    crosswalks, stop bars, parking bays     (NEW)

Drop-in for `scene_generator.generate_scene_on_stage`, so a launch script only
has to change which function it calls.

The built-in street-furniture passes are switched off by the scene config
(spacings set to 0 in `presets/urban_v2.yaml`), not by code — so pointing this
at an ordinary config still works, it just has nothing extra to add.
"""

import glob
import os
import random

import scene_generator as sg

from detail import city_detail
from layout import city_layout
from detail import districts
from detail import road_markings


def check_duplicate_yaml_keys(paths=None) -> int:
    """Fail loudly on duplicate mapping keys in the scene configs.

    PyYAML silently keeps the LAST of any duplicated key, which is how a stale
    `dumpsters: []` left behind by an edit wiped out a populated pool with no
    error, no warning, and no visible diff in the loaded config — the category
    just quietly stopped placing anything. Nothing in the load path catches it,
    so it is checked here before generation starts.

    Returns the number of duplicates found; raises if any.
    """
    import yaml

    if paths is None:
        cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config")
        paths = (glob.glob(os.path.join(cfg, "asset_sets", "*.yaml"))
                 + glob.glob(os.path.join(cfg, "presets", "*.yaml"))
                 + glob.glob(os.path.join(cfg, "low_level", "*.yaml")))

    class _Strict(yaml.SafeLoader):
        pass

    def _no_dupes(loader, node, deep=False):
        seen, dupes = set(), []
        for key_node, _ in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if key in seen:
                dupes.append(key)
            seen.add(key)
        if dupes:
            raise yaml.constructor.ConstructorError(
                None, None,
                f"duplicate key(s) {dupes} at line {node.start_mark.line + 1}",
                node.start_mark)
        return yaml.SafeLoader.construct_mapping(loader, node, deep)

    _Strict.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _no_dupes)

    bad = []
    for p in sorted(paths):
        try:
            with open(p) as fh:
                yaml.load(fh, Loader=_Strict)
        except yaml.constructor.ConstructorError as exc:
            bad.append(f"  {os.path.relpath(p)}: {exc.problem}")
    if bad:
        raise ValueError("duplicate YAML keys — PyYAML would silently keep "
                         "only the last, so this is fatal:\n" + "\n".join(bad))
    return 0


def prune_prims(stage, config: dict, placements: list) -> int:
    """Deactivate named sub-prims inside placed assets.

    Some assets carry geometry that shouldn't be in the scene but can't be
    removed at the asset level: a street-name sign that also has a stop sign
    welded to it, or an exporter leaving its editor scaffolding
    (`CINEMA_4D_Editor`) in the file. The asset is otherwise wanted, so the
    choice is to drop it entirely or to prune the offending prim.

    Deactivating rather than removing: the prim lives in a referenced layer, so
    `RemovePrim` from the session layer doesn't take, whereas deactivation
    composes and is reversible in the viewport.

    Config, matched by substring against each placement's USD path::

        prune_prims:
          - usd_contains: "599a74c1c8fd4f19bf6754b8128da6b8"
            prims: ["pCube4", "pCube44"]      # matched by prim NAME, any depth
    """
    from pxr import Usd

    rules = config.get("prune_prims") or []
    if not rules:
        return 0

    n_pruned = 0
    per_rule = {}
    for rule in rules:
        needle = str(rule.get("usd_contains", ""))
        wanted = {str(p) for p in (rule.get("prims") or [])}
        if not needle or not wanted:
            continue
        hits = 0
        for pl in placements:
            if needle not in str(pl.get("usd", "")):
                continue
            path = pl.get("prim_path")
            if not path:
                continue
            root = stage.GetPrimAtPath(path)
            if not root.IsValid():
                continue
            # Instance proxies aren't editable, so traverse the prototypes too.
            for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
                if p.GetName() in wanted and p.IsActive():
                    try:
                        p.SetActive(False)
                        hits += 1
                    except Exception:
                        pass
        per_rule[needle[:12]] = hits
        n_pruned += hits

    if n_pruned:
        detail = "  ".join(f"{k}={v}" for k, v in sorted(per_rule.items()))
        print(f"[scene_gen] pruned {n_pruned} sub-prims  {detail}")
    return n_pruned


def _tally(placements):
    out = {}
    for p in placements:
        c = p.get("category", "?")
        out[c] = out.get(c, 0) + 1
    return out


def stamp_asset_provenance(stage, placements, manifest_path: str = "") -> int:
    """Record which asset every placed prim came from, on the prim and on disk.

    Identifying an asset from a prim path by regenerating the scene and reading
    the Nth placement of a category is unreliable: the index is a function of
    the pools, so adding or removing ONE asset renumbers everything after it and
    the same path denotes a different asset than it did last run. That is a good
    way to blacklist the wrong model.

    So each prim carries its own answer. `customData` is authored on the prim,
    which means selecting it in the viewport and reading the property panel is
    enough — no regeneration, no index arithmetic, and it stays correct even
    when the pools change underneath.

    A TSV of the same mapping is written beside the scene for grepping.
    """
    rows = []
    for pl in placements:
        path = pl.get("prim_path")
        if not path:
            continue
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        usd = str(pl.get("usd", ""))
        cat = str(pl.get("category", ""))
        try:
            prim.SetCustomDataByKey("sourceAsset", usd)
            prim.SetCustomDataByKey("assetCategory", cat)
            # The pack is what gets blacklisted in practice, so surface it
            # directly rather than making it a mental substring operation.
            prim.SetCustomDataByKey("sourcePack", _pack_of(usd))
        except Exception:
            pass
        rows.append((str(path), cat, _pack_of(usd), usd))

    if manifest_path:
        try:
            with open(manifest_path, "w") as fh:
                fh.write("prim_path\tcategory\tpack\tusd\n")
                for r in sorted(rows):
                    fh.write("\t".join(r) + "\n")
            print(f"[scene_gen] asset manifest -> {manifest_path} ({len(rows)} prims)")
        except Exception as exc:
            print(f"[scene_gen] could not write manifest: {exc}")
    return len(rows)


def apply_surface_overrides(stage, config: dict, placements: list) -> int:
    """Force roughness / metallic on the materials a category binds.

    Some library assets are authored for an archviz still and come back far too
    glossy in a daylit scene — the park path tile reads as wet stone. The
    material lives in a referenced layer and is shared by every instance, so the
    fix is authored once per MATERIAL rather than per prim.

    Config::

        surface_overrides:
          - categories: [trail]
            roughness: 0.85
            metallic: 0.0
    """
    from pxr import Sdf, Usd, UsdShade

    rules = config.get("surface_overrides") or []
    if not rules:
        return 0

    by_cat = {}
    for pl in placements:
        by_cat.setdefault(str(pl.get("category", "")), []).append(pl)

    seen, n = set(), 0
    for rule in rules:
        cats = [str(c) for c in (rule.get("categories") or [])]
        for cat in cats:
            for pl in by_cat.get(cat, ()):
                path = pl.get("prim_path")
                if not path:
                    continue
                root = stage.GetPrimAtPath(path)
                if not root.IsValid():
                    continue
                for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
                    if prim.GetTypeName() != "Mesh":
                        continue
                    mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
                    if not mat or not mat.GetPrim().IsValid():
                        continue
                    key = str(mat.GetPrim().GetPath())
                    if key in seen:
                        continue
                    seen.add(key)
                    shader = mat.ComputeSurfaceSource()[0]
                    if not shader:
                        continue
                    for name, val in (("roughness", rule.get("roughness")),
                                      ("metallic", rule.get("metallic"))):
                        if val is None:
                            continue
                        try:
                            inp = shader.GetInput(name)
                            if not inp:
                                inp = shader.CreateInput(
                                    name, Sdf.ValueTypeNames.Float)
                            inp.Set(float(val))
                            n += 1
                        except Exception:
                            pass
    if n:
        print(f"[scene_gen] surface overrides: {n} input(s) on "
              f"{len(seen)} material(s)")
    return n


def _pack_of(usd: str) -> str:
    """The asset pack a USD belongs to — the unit assets get rejected by."""
    u = str(usd)
    if "/assets/aec/" in u:
        tail = u.split("/assets/aec/", 1)[1]
        return "AEC/" + tail.split("/", 1)[0]
    if "/assets/objaverse/" in u or u.startswith("objaverse://"):
        return "objaverse"
    if "/Library/Stages/" in u:
        rest = u.split("/Library/Stages/", 1)[1].split("/")
        # Dmytro nests one more level: Dmytro/Assets/Game/<PACK>/...
        if rest[0] == "Dmytro" and len(rest) > 3:
            return "Dmytro/" + rest[3]
        return "/".join(rest[:2]) if len(rest) > 1 else rest[0]
    return "?"


def generate_scene_on_stage(stage,
                              config,
                              parent_path: str = "/World/stage/generated",
                              scene_scale_factor: float = 1.0,
                              snap_to_ground: bool = False) -> list:
    """Build the detailed city onto a live stage. Returns the placement list."""
    check_duplicate_yaml_keys()

    if isinstance(config, str):
        config = sg.load_config(config)

    resolver = sg._make_resolver(config)
    rng = random.Random(int(config.get("seed", 0)) + 7717)

    # Scoped for the build_city call only; restored on exit, so nothing else in
    # the process sees a patched generator.
    with city_layout.patched(config):
        placements, layout = sg.build_city(config, resolver)
    base_counts = _tally(placements)

    district_at, rings = districts.assign(config, layout)
    if rings:
        districts.remap_buildings(config, layout, placements, resolver, rng,
                                  district_at)

    # District zoning can promote a reserved superblock to a park after the
    # initial build pass recorded its paving rectangle. Keep the procedural
    # ground optimization, but remove those rectangles so the park's grass
    # plane remains visible beneath its trail and park furniture.
    park_rects = districts.park_blocks(layout, placements)
    if park_rects and layout.get("paved_blocks"):
        layout["paved_blocks"] = [
            r for r in layout["paved_blocks"]
            if not any(r[0] >= b[0] and r[1] >= b[1]
                       and r[2] <= b[2] and r[3] <= b[3]
                       for b in park_rects)]

    detail = city_detail.build(
        config, layout, resolver, rng,
        district_of=districts.block_lookup(district_at) if rings else None)
    detail += city_detail.build_road_surface(config, layout, resolver, rng)

    placements = placements + detail

    ground_snap = sg._make_physx_ground_snap() if snap_to_ground else None
    sg.apply_placements(stage, placements, parent_path, scene_scale_factor,
                        ground_snap, resolver=resolver)
    # After apply_placements, which is what assigns each placement its prim_path.
    prune_prims(stage, config, placements)
    stamp_asset_provenance(
        stage, placements,
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "_scene_assets.tsv"))
    sg.apply_ground_planes(stage, config, layout, parent_path,
                           scene_scale_factor)
    # Hydrant clear zones are signed no-parking and bus zones are kerb that is
    # neither parked on nor hatched, so the markings pass needs the positions
    # city_detail chose; it runs earlier, hence passing them through.
    _at = lambda cat: [(p["x_m"], p["y_m"]) for p in placements
                       if p.get("category") == cat]
    road_markings.apply(stage, config, layout, parent_path, scene_scale_factor,
                        hydrants=_at("fire_hydrant"),
                        bus_stops=_at("bus_stop"))

    _report(base_counts, _tally(placements))
    return placements


def reload_scene_on_stage(stage,
                            config,
                            parent_path: str = "/World/stage/generated",
                            scene_scale_factor: float = 1.0,
                            snap_to_ground: bool = False,
                            add_colliders_fn=None) -> list:
    """Clear and rebuild in place, for iterating without restarting Isaac Sim.

    Mirrors `scene_generator.reload_scene_on_stage` so the Script Editor loop
    documented in scene_gen/README.md works the same way here — reload this
    module too, since edits to the new passes live in it.
    """
    from pxr import Sdf

    if stage.GetPrimAtPath(parent_path).IsValid():
        stage.RemovePrim(Sdf.Path(parent_path))
        print(f"[scene_gen] cleared existing '{parent_path}'")

    placements = generate_scene_on_stage(
        stage, config, parent_path, scene_scale_factor, snap_to_ground)

    if add_colliders_fn is not None:
        gen = stage.GetPrimAtPath(parent_path)
        if gen.IsValid():
            add_colliders_fn(gen)
    return placements


def _report(before: dict, after: dict):
    """Per-category tally, so v1 and v2 can be compared numerically rather than
    only by eye."""
    keys = sorted(set(before) | set(after))
    width = max((len(k) for k in keys), default=8)
    print("\n" + "=" * (width + 26))
    print(f"{'category':<{width}}  {'build_city':>10}  {'total':>10}")
    print("-" * (width + 26))
    for k in keys:
        b, a = before.get(k, 0), after.get(k, 0)
        mark = "  +" if a > b else ""
        print(f"{k:<{width}}  {b:>10}  {a:>10}{mark}")
    print("-" * (width + 26))
    print(f"{'TOTAL':<{width}}  {sum(before.values()):>10}  "
          f"{sum(after.values()):>10}")
    print("=" * (width + 26) + "\n")


# ---------------------------------------------------------------------------
# Offline CLI: bake to USD without Isaac Sim, same shape as scene_generator's.
# ---------------------------------------------------------------------------

def main():
    import argparse

    from pxr import Usd, UsdGeom

    ap = argparse.ArgumentParser(
        description="Bake the detailed procedural city to a USD file.")
    ap.add_argument("--config", required=True,
                    help="scene config (preset name, high- or low-level path)")
    ap.add_argument("--output", required=True, help="output .usd path")
    ap.add_argument("--scale-factor", type=float, default=1.0,
                    help="1 / meters_per_unit for the target stage")
    args = ap.parse_args()

    from compile_disaster import load_scene_config
    config = load_scene_config(args.config)

    stage = Usd.Stage.CreateNew(args.output)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0 / float(args.scale_factor))
    UsdGeom.Xform.Define(stage, "/World")
    generate_scene_on_stage(stage, config, "/World/generated",
                              float(args.scale_factor))
    stage.GetRootLayer().Save()
    print(f"[scene_gen] wrote {args.output}")


if __name__ == "__main__":
    main()
