"""plan — enumerate the (type, level) grid Stage A must bake.

Pure Python, no `pxr`: the plan is worth being able to compute, print and test
on the host in milliseconds, because it is what tells you whether a bake is a
twenty-minute job or an overnight one BEFORE you start it.

WHAT COUNTS AS DISPLACEABLE
---------------------------
SPEC: "Only assets the disaster DISPLACES (e.g. buildings, trees) are baked;
surface-only damage is deferred to Stage B, step 6." So a bench that merely
gets scorched is not here — a material does that at assembly time, for free.
What is here is anything whose GEOMETRY changes: buildings and vegetation.

WHERE THE TYPES COME FROM
-------------------------
Three sources, because "build one clean instance of that type" means three
different things:

    modular    a house assembled from the kit (`detail.modular_house.STYLES`).
               The type is the style name; building it means running the kit.
    library    a USD referenced whole (every `buildings.*` pool in the asset
               pack, and the tree species). The type is a slug of its path;
               building it means adding a reference.
    (skipped)  pools that are already damaged art — `buildings.damaged` and
               `buildings.destroyed` are somebody's authored ruin, and baking
               a ruin's ruin is meaningless.

The damage MODEL is then dispatched per kind, which is where the two surviving
damage stacks both find a home: modular kits go through `damage_flow`
(it knows the kit's wall/roof/gable structure), library USDs through
`mesh_damage`'s asset-generic operators, vegetation through `vegetation`.
"""

from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from archetypes import library as lib                          # noqa: E402
from disaster import levels as L                               # noqa: E402

#: Asset-pack pools that hold INTACT structures worth damaging, in the LEGACY
#: layout — `damaged` and `destroyed` are pre-authored ruins, while
#: `rowhouse`/`midrise`/`tower` are urban's typology pools, all intact art.
#: Kept for the tests that assert a ruin is never planned; the planner itself
#: asks `scene_generator.every_building(config, "intact")`, which answers the
#: same question in either layout (in the current one the condition is a TAG on
#: the entry, so the pool names carry no condition at all).
STRUCTURE_POOLS = ("intact", "townhouse", "lowrise", "midrise",
                   "highrise", "rowhouse", "tower")

#: Where vegetation lives in a pack. TOP-LEVEL lists, not a `vegetation:`
#: group — `shared.yaml` keeps `trees:` and `street_trees:` beside the props.
#: Trees only: a shrub is not displaceable at the scale a drone sees.
VEGETATION_POOLS = ("trees", "street_trees", "park_trees")


class Item:
    """One thing to bake: a type, its kind, and how to build a clean instance.

    `scale` and `axis_up` are carried because an archetype must be built at the
    SAME size the scene would have placed it at. The Nucleus building packs are
    centimetre-authored (`asset_scale: 0.01`), and referencing one without that
    produced archetypes 8 km across — which then got referenced into scenes as
    buildings a hundred times bigger than their block.
    """

    __slots__ = ("type", "kind", "source", "build", "levels", "scale",
                 "axis_up")

    def __init__(self, type_, kind, source, build, levels, scale=1.0,
                 axis_up="Z"):
        self.type = type_        # slugged identifier, the filename component
        self.kind = kind         # L.STRUCTURE | L.VEGETATION
        self.source = source     # usd path, or the modular style name
        self.build = build       # "modular" | "library"
        self.levels = list(levels)
        self.scale = float(scale)
        self.axis_up = str(axis_up or "Z").upper()

    def __repr__(self):
        return (f"Item({self.type!r}, {self.kind}, {self.build}, "
                f"{len(self.levels)} levels)")

    @property
    def combos(self) -> list:
        return [(self.type, lv) for lv in self.levels]


def _paths(entries, default_scale: float = 1.0) -> list:
    """``[(path, scale, axis_up), ...]`` out of an asset-pack entry list.

    The per-entry `scale` and `axis-up` overrides are the point: dropping them
    is how the baker built 8 km buildings out of a centimetre-authored pack.
    """
    from scene_generator import _normalize_usd_list
    if not entries:
        return []
    paths, sc, au, _yaw, _tags = _normalize_usd_list(
        entries, default_scale, "")
    return [(p, float((sc or {}).get(p, default_scale)),
             str((au or {}).get(p, "Z")).upper()) for p in paths]


def grouped_pools(usds: dict, group: str, pools: tuple,
                  default_scale: float = 1.0) -> list:
    """Every USD under ``usds[group][pool]`` — how buildings are organised."""
    node = (usds or {}).get(group) or {}
    if not isinstance(node, dict):
        return []
    return [t for pool in pools
            for t in _paths(node.get(pool), default_scale)]


def flat_pools(usds: dict, pools: tuple, default_scale: float = 1.0) -> list:
    """Every USD under the top-level ``usds[pool]`` — how trees are organised."""
    return [t for pool in pools
            for t in _paths((usds or {}).get(pool), default_scale)]


def modular_styles() -> list:
    """The kit house styles, or [] if the kit is unavailable."""
    try:
        from detail import modular_house as mh
        return sorted(mh.STYLES.keys())
    except Exception:                                          # noqa: BLE001
        return []


def build_plan(config: dict, disaster: str = "", include_modular: bool = True,
               include_library: bool = True) -> list:
    """The full Stage A grid for *config*. Returns ``[Item, ...]``.

    *config* is a compiled low-level config (it needs `usds`, which
    `resolve_asset_pack` fills in). *disaster* overrides the config's own type,
    which is how one config bakes libraries for several disasters.
    """
    dtype = str(disaster or (config.get("disaster") or {}).get("type")
                or "none").lower()
    usds = config.get("usds") or {}

    struct_levels = L.bake_levels(dtype, L.STRUCTURE)
    veg_levels = L.bake_levels(dtype, L.VEGETATION)

    # The pack's default scale. Nucleus packs are centimetre-authored, so this
    # is usually 0.01 and is what a per-entry `scale:` overrides.
    default_scale = float(config.get("asset_scale", 1.0))

    items, seen, by_slug = [], set(), {}

    def _add(type_, kind, source, build, levels, scale=1.0, axis_up="Z"):
        # Dedupe by SOURCE, not by slug: the same USD listed in two pools is
        # one type, but two different USDs must never collapse into one
        # archetype just because their basenames match. `Trees/Oak.usd` and
        # `Vegetation/Oak.usd` both slug to `Oak`, and silently baking one and
        # placing it for both is the kind of bug that shows up as "why is every
        # tree on this street identical".
        if (source, kind) in seen:
            return
        seen.add((source, kind))
        slug = lib.type_slug(type_)
        prev = by_slug.get((slug, kind))
        if prev is not None and prev != source:
            n = 2
            while (f"{slug}_{n}", kind) in by_slug:
                n += 1
            slug = f"{slug}_{n}"
        by_slug[(slug, kind)] = source
        items.append(Item(slug, kind, source, build, levels, scale, axis_up))

    if include_modular:
        for style in modular_styles():
            _add(style, L.STRUCTURE, style, "modular", struct_levels)

    if include_library:
        # EVERY intact building, whatever pool it sits in — see
        # `scene_generator.every_building`. Authored ruins (tagged `damaged` /
        # `destroyed`, or sitting in those pools) are skipped: baking a ruin's
        # ruin is meaningless and doubles the library for nothing.
        from scene_generator import every_building
        for usd, sc, au in _paths(every_building(config, "intact"),
                                  default_scale):
            _add(usd, L.STRUCTURE, usd, "library", struct_levels, sc, au)
        for usd, sc, au in flat_pools(usds, VEGETATION_POOLS, default_scale):
            _add(usd, L.VEGETATION, usd, "library", veg_levels, sc, au)

    return items


def used_by_scene(config: dict, items: list) -> list:
    """Only the items a scene actually PLACES. Needs no Isaac Sim.

    A small scene draws a handful of assets from a pack holding dozens, and
    Stage A is priced per archetype (~40 s each for a library asset). Baking a
    198-archetype library to look at six buildings is most of a working day
    spent on assets that never appear.

    Runs the real layout and detail stages — the same `build_scene` the sim
    calls — and keeps the types whose USD came back in the placement list. That
    is exact rather than heuristic: if the scene places it, it is here.

    NOT a substitute for a full bake, and it is narrower than it looks:

    * It is a function of the SEED. Reseed or resize and holes appear.
    * It is a function of the ENVIRONMENT. Packing keys off measured
      footprints, and a plain `python3` cannot open a Nucleus asset — it falls
      back to `fallback_sizes`, packs differently, and places DIFFERENT
      buildings. Measured on `urban_quake_tiny`: plain python3 reports 5
      structure types, the same call inside Kit reports 2.

    So a list produced on the host must not be pasted into a bake that runs
    under Kit. `bake_cli.py --used-only` is safe because it resolves the plan
    *after* Isaac Sim has started, in the same environment that will build the
    scene; `plan.py --used-only` on the host is an estimate, useful for
    pricing a bake and not for pinning one.
    """
    import scene_generator as sg
    import generate_scene

    resolver = sg._make_resolver(config)
    placements, _layout, _counts = generate_scene.build_scene(
        config, resolver, stop_after="detail")
    # MATCH ON THE SLUG, not on the raw string. A placement's `usd` has been
    # joined to `asset_root` (`omniverse://.../Foo.usd`) while a plan item's
    # source is the bare relative path the pack listed (`Foo.usd`), so a
    # string compare silently matches only the assets that were already
    # absolute — which is how this found every tree and not one building.
    placed = {lib.type_slug(p.get("usd", ""))
              for p in placements if p.get("usd")}
    keep, seen = [], set()
    for it in items:
        if lib.type_slug(it.source) in placed and id(it) not in seen:
            seen.add(id(it))
            keep.append(it)
    # Modular styles carry no USD — the kit builds them — so they are kept by
    # whether the scene placed a house of that style at all.
    styles = {str(p.get("style")) for p in placements if p.get("style")}
    for it in items:
        if it.build == "modular" and it.source in styles and id(it) not in seen:
            seen.add(id(it))
            keep.append(it)
    return keep


def summarise(items: list, dtype: str) -> str:
    """A human-readable size estimate — read this before starting a bake."""
    n_struct = sum(1 for i in items if i.kind == L.STRUCTURE)
    n_veg = sum(1 for i in items if i.kind == L.VEGETATION)
    combos = sum(len(i.levels) for i in items)
    mod = sum(1 for i in items if i.build == "modular")
    lines = [
        f"Stage A plan — disaster '{dtype}'",
        f"  structures : {n_struct:4d} types ({mod} modular, "
        f"{n_struct - mod} library) x {len(L.bake_levels(dtype, L.STRUCTURE))} levels",
        f"  vegetation : {n_veg:4d} types x "
        f"{len(L.bake_levels(dtype, L.VEGETATION))} levels",
        f"  TOTAL      : {combos} archetypes to bake",
    ]
    return "\n".join(lines)


def main():
    import argparse

    ap = argparse.ArgumentParser(
        description="Print the Stage A bake plan for a config. Costs nothing "
                    "— run it before committing to a bake.")
    ap.add_argument("--config", required=True,
                    help="scene config (preset name, high- or low-level path)")
    ap.add_argument("--disaster", default="",
                    help="override the config's disaster type")
    ap.add_argument("--list", action="store_true",
                    help="list every (type, level) rather than just counting")
    ap.add_argument("--used-only", action="store_true",
                    help="only the types this scene actually places. Much "
                         "smaller, but seed-specific — see `used_by_scene`.")
    ap.add_argument("--arch-only", action="store_true",
                    help="print an ARCH_ONLY= line ready to paste")
    args = ap.parse_args()

    from compile_disaster import load_scene_config
    config = load_scene_config(args.config)
    dtype = (args.disaster
             or (config.get("disaster") or {}).get("type") or "none").lower()
    items = build_plan(config, dtype)
    if args.used_only:
        full = len(items)
        items = used_by_scene(config, items)
        print(f"(--used-only: {len(items)} of {full} types are actually placed)")
    print(summarise(items, dtype))
    if args.arch_only:
        print("\nARCH_ONLY=" + ",".join(sorted(i.type for i in items)))
    if args.list:
        for it in items:
            for _t, lv in it.combos:
                print(f"  {it.kind:10} {it.build:8} {it.type}_{lv}")


if __name__ == "__main__":
    main()
