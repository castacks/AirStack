"""
scene_generator.py — Procedural city generator for Isaac Sim / USD.

Builds a dense urban neighborhood by referencing a library of USD props
(houses, trees, streetlights, cars). Blocks are packed with buildings out to
the sidewalk; leftover gaps are filled with placeholder prisms:

    Step 1 — Roads: metric BSP splits the region into irregular blocks separated
             by road corridors. Corridor widths are N × lane_width_m (main or
             secondary). Ground geometry is procedural UsdGeom.Mesh planes:
             one asphalt base (whole region, z=0), one grass plane per block
             (z=0.01, covers asphalt; road gaps reveal asphalt underneath),
             and yellow/white lane-marking dashes per corridor (z=0.02).
    Step 2 — Ground: brick sidewalk tiles ring each block; packed blocks get
             wall-to-wall concrete pavement.
    Step 3 — Buildings: each block is *packed* with buildings out to the
             sidewalk via greedy guillotine packing (packing.*); gaps no
             library building fits get mild-colored placeholder box prisms;
             a fraction of blocks stay as grassy parks — each park gets a
             meandering Catmull-Rom trail with park-tagged benches (facing
             the trail), lamps and trash cans along it, a playground
             cluster off the trail, plus rocks and plants on the grass and
             evenly-spread trees (parks.*). Packing draws from the intact /
             damaged / destroyed pools by fate weight (disaster.damaged_fraction
             / destroyed_fraction), each building packed with its own
             footprint; destroyed ruins get a debris field — piles hugging
             the base, fragments physics-settled nearby — and sometimes
             lean (slight tilt + sink) with rubble on the caved-in side.

All disaster knobs live under ``disaster.*`` and are *maxima*: each is scaled
per-position by ``disaster.field`` (see :func:`make_damage_field`), so an
earthquake can attenuate from its epicenter while a tornado carves a narrow
track across an otherwise untouched city. This module consumes **low-level**
configs only; author them by hand or compile one from a high-level disaster
spec with ``compile_disaster.py``.
    Step 4 — Detail: humans on sidewalks/trails/grass; bus stops,
             streetlights, benches, planters (street trees + plant boxes),
             fire hydrants and trash cans along block frontages — every
             street prop inset off the curb so it sits fully on the
             sidewalk (shared occupancy grid — categories never stack);
             traffic lights at NS×EW corridor intersections.
    Step 5 — Cars + disaster: cars parked along road corridors (right-hand
             traffic). Disaster pass (tornado aftermath) applied inline.

Relative USD paths in the config are prefixed with ``asset_root`` (URLs and
absolute paths pass through unchanged).

Asset footprints are **measured at generation time** from each USD's bounding
box (``UsdGeom.BBoxCache``) so the layout tiles correctly regardless of prop
size; if a USD can't be opened (e.g. offline with no Nucleus access) it falls
back to per-category ``fallback_sizes`` constants from the config.

The layout math is pure Python (depends only on ``math`` / ``random``); only
measurement and USD writing touch ``pxr`` / ``omni``.

Public API:
    load_config              — Parse the city spec YAML into a dict
    resolve_sky              — Resolve the config's ``sky`` entry against asset_root
    build_city               — Compute placements for a config (+ size resolver)
    apply_placements         — Write placements onto a USD stage as references
    generate_scene_usd       — Bake a standalone .usd from a config (offline)
    generate_scene_on_stage  — Compose onto a live stage (runtime, Isaac Sim)
    reload_scene_on_stage    — Clear + regenerate in place (no sim restart)

Config schema — see config/low_level/default.yaml for a
worked example, and GENERATION.md for how the
high-level (disaster) and low-level (scene) config layers fit together.
"""

import math
import os
import random
import re

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, UsdSkel, Vt


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Stages
#
# A low-level config is organised by the stage that consumes it:
#
#   layout:    where blocks, roads and buildings go — the city plan
#   detail:    what is placed on that plan — vegetation, street furniture,
#              vehicles, people, park furnishing
#   disaster:  what the event does to the finished scene
#
# plus the scene-wide metaparameters (`seed`, `asset_pack`, `exclusions`,
# `measure_usds`, `usds`, …) which belong to no single stage.
#
# `_stage` is how every module reaches into one. It tolerates a missing stage
# so a partial config still loads, and it is the single place to change if the
# grouping is ever revised.
# ---------------------------------------------------------------------------

STAGES = ("layout", "detail", "disaster")


def _stage(config: dict, name: str) -> dict:
    """The *name* stage of *config*, or an empty dict if absent."""
    return (config or {}).get(name) or {}


def stage(config: dict, name: str) -> dict:
    """Public alias — other modules read their settings through this."""
    return _stage(config, name)


# Which stage owns each settings block. Anything not listed is a scene-wide
# metaparameter (`seed`, `asset_pack`, `usds`, `exclusions`, …) and stays at the
# top level.
STAGE_OF = {
    # layout — where blocks, roads and buildings go
    "packing": "layout", "frontage": "layout", "roads": "layout",
    "districts": "layout",
    # detail — what is placed on that plan
    "trees": "detail", "plants": "detail", "planters": "detail",
    "streetlights": "detail", "benches": "detail", "trash_cans": "detail",
    "bus_stops": "detail", "fire_hydrants": "detail",
    "traffic_lights": "detail", "humans": "detail", "cars": "detail",
    "driveways": "detail", "parks": "detail", "city_detail": "detail",
}

# Keys that are already stages, or belong to the layout stage under their own
# name (`layout.region_m` and friends were nested before this grouping existed).
_STAGE_KEYS = set(STAGES)


def restage(settings: dict) -> dict:
    """Nest a flat settings dict under its stages.

    Locale compilers and a preset's `overrides:` are far more readable written
    flat — one line per knob — than with every entry buried two levels deep.
    They stay that way and are normalised here, which also means a config
    written before the grouping existed still loads unchanged.

    Idempotent: anything already under a stage key is left alone.
    """
    if not isinstance(settings, dict):
        return settings
    out: dict = {}
    for key, val in settings.items():
        if key in _STAGE_KEYS and isinstance(val, dict):
            # MERGE, never assign. A settings dict routinely holds both an
            # already-staged block (`detail:` loaded from a locale YAML) and
            # loose keys that belong in it (`trees:`, `plants:`). Assigning
            # meant whichever came last silently erased the other.
            _deep_merge(out.setdefault(key, {}), val)
            continue
        target = STAGE_OF.get(key)
        if target is None:
            out[key] = val
        else:
            _deep_merge(out.setdefault(target, {}), {key: val})
    return out


from config_merge import deep_merge as _deep_merge      # noqa: E402,F401


def _merge_asset_pack(base: dict, child: dict) -> dict:
    """Merge *child* asset pack over *base* in place.

    Like :func:`_deep_merge`, except a key written ``<name>+`` **appends** to
    the inherited list instead of replacing it — for a category the child
    wants to extend rather than own outright ::

        trees+:                     # keep the base's trees, add these
          - "…/SM_PalmTree.usd"

    Writing ``trees:`` instead replaces the inherited list outright, which is
    the right move when a set wants nothing the base offered — as every locale
    set does for ``buildings`` and ``debris``: a wrecked mid-rise and a
    wrecked house are different materials, not one extended by the other, so
    those categories are never shared and always fully replaced.
    """
    for k, v in child.items():
        if isinstance(k, str) and k.endswith("+"):
            key = k[:-1]
            inherited = base.get(key)
            base[key] = ((list(inherited) if isinstance(inherited, list) else [])
                         + (list(v) if isinstance(v, list) else [v]))
        elif isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge_asset_pack(base[k], v)
        else:
            base[k] = v
    return base


def _find_asset_pack(name: str, dirs: list) -> str:
    candidates = [str(name)]
    for d in dirs:
        candidates += [os.path.join(d, str(name)),
                       os.path.join(d, f"{name}.yaml"),
                       os.path.join(d, f"{name}.yml")]
    path = next((c for c in candidates if os.path.isfile(c)), None)
    if path is None:
        searched = "\n".join(f"    {os.path.normpath(d)}" for d in dirs)
        raise ValueError(f"asset_pack {name!r} not found. Searched:\n{searched}")
    return path


def _load_asset_pack(name: str, dirs: list, chain: list = None) -> tuple:
    """Load an asset pack and everything it ``extends``, base-first.

    Returns ``(merged_dict, [paths in resolution order])``.
    """
    import yaml

    chain = list(chain or [])
    if name in chain:
        raise ValueError("asset_pack extends cycle: "
                         + " -> ".join(chain + [str(name)]))
    chain.append(name)

    path = _find_asset_pack(name, dirs)
    with open(path) as f:
        doc = yaml.safe_load(f)
    if not isinstance(doc, dict):
        raise ValueError(f"asset pack {path!r} did not parse to a mapping")

    parent = doc.pop("extends", None)
    if not parent:
        return doc, [path]
    # A LIST of parents is allowed, merged left to right, so a set can join two
    # libraries without duplicating either — the suburb needs the suburban pools
    # AND the park's recreation pools, and neither has any reason to carry the
    # other's. `prepare_assets._with_extends` already accepted a list; this side
    # did not, so a list resolved on the host and then died at scene build with
    # `asset_pack "[...]" not found`. The two now agree.
    parents = [parent] if isinstance(parent, str) else list(parent)
    base, paths = {}, []
    for parent_name in parents:
        b, ps = _load_asset_pack(str(parent_name), dirs, chain)
        base = _merge_asset_pack(base, b) if base else b
        paths += ps
    return _merge_asset_pack(base, doc), paths + [path]


def resolve_asset_pack(config: dict, config_path: str = None) -> dict:
    """Merge the asset pack named by ``config['asset_pack']`` into *config*.

    An asset pack (``config/asset_packs/<name>.yaml``) holds
    *what the assets are* — ``asset_root``, ``asset_scale``, ``sky``,
    ``orientation``, ``fallback_sizes`` and the whole ``usds`` library — so a
    scene can be re-skinned by naming a different set. The config wins on
    conflicts, so it can override anything the set defines.

    A set may ``extends:`` another, which is how the locale sets avoid
    restating the props every locale shares (see ``shared.yaml``): the base is
    loaded first and the child merged over it, with ``<key>+`` appending to an
    inherited list rather than replacing it.

    ``asset_pack`` may be a bare name or a path. No-op when absent (a config
    carrying its own ``usds`` is still valid).
    """
    name = config.get("asset_pack")
    if not name:
        return config

    here = os.path.dirname(os.path.abspath(__file__))
    dirs = [os.path.join(here, "config", "asset_packs")]
    if config_path:
        cfg_dir = os.path.dirname(os.path.abspath(config_path))
        # low_level/<cfg>.yaml and low_level/compiled/<cfg>.yaml both sit
        # under config/, so look up as well as alongside.
        dirs[:0] = [os.path.join(cfg_dir, "asset_packs"),
                    os.path.join(cfg_dir, "..", "asset_packs"),
                    os.path.join(cfg_dir, "..", "..", "asset_packs")]

    assets, paths = _load_asset_pack(str(name), dirs)

    merged = _deep_merge(assets, config)      # config overrides the set
    merged["asset_pack"] = name
    lineage = " <- ".join(os.path.basename(p) for p in reversed(paths))
    print(f"[scene_gen] asset pack: {name} ({lineage})")
    return merged


def validate_config(config: dict, source: str = "<config>") -> dict:
    """Raise ValueError if *config* is missing anything the generator needs.

    Split out from :func:`load_config` so configs built in memory (e.g.
    compiled from a high-level disaster spec) get the same checks. *source*
    only labels the error messages.
    """
    if not isinstance(config, dict):
        raise ValueError(f"City config {source!r} did not parse to a mapping")

    if config.get("disaster-type") or config.get("disaster_type"):
        raise ValueError(
            f"{source!r} looks like a HIGH-level disaster spec, not a "
            "low-level scene config. Compile it first:\n"
            "    python3 utils/compile_disaster.py <spec>\n"
            "or load it via compile_disaster.load_scene_config(), which "
            "compiles high-level specs on the fly.")

    if "usds" not in config:
        raise ValueError(
            f"City config {source!r} has no 'usds' library — name an asset "
            "set with `asset_pack: <name>` (see config/asset_packs/), "
            "or inline a usds section.")
    if "layout" not in config or "region_m" not in config["layout"]:
        raise ValueError(f"City config {source!r} has no 'layout.region_m' "
                         "(city extent in meters, e.g. [200, 200])")

    usds = config["usds"]
    # "buildings" is the current section name; "houses" accepted for
    # backward compatibility.
    if not building_entries(config, condition=DEFAULT_CONDITION):
        raise ValueError(
            "usds.buildings must list at least one INTACT building USD — "
            "either under a `buildings.intact` pool (legacy layout) or as a "
            "typology pool entry tagged `tags: [intact]` (or untagged, which "
            "means intact). See `building_entries`.")

    absent = missing_objaverse_assets(config)
    if absent:
        print(f"[scene_gen] WARNING: {len(absent)} Objaverse asset(s) are not "
              f"cached and will render as placeholder prisms:")
        for uid in absent[:10]:
            print(f"[scene_gen]   {uid}")
        if len(absent) > 10:
            print(f"[scene_gen]   … and {len(absent) - 10} more")
        print("[scene_gen] Fix: run this on the host, then relaunch:")
        print("[scene_gen]   python3 scene_gen/prepare_assets.py "
              f"{config.get('asset_pack') or ''}".rstrip())

    return config


def load_config(config_path: str) -> dict:
    """Load and lightly validate a **low-level** city-generator YAML spec.

    A low-level spec holds the generator settings and names an asset pack
    (``asset_pack: urban``) for the asset library itself, which is merged in
    here — see :func:`resolve_asset_pack`. Author them by hand
    (``low_level/default.yaml``) or compile them from a high-level disaster
    spec with ``compile_disaster.py``, which writes into ``low_level/compiled/``.

    To accept *either* level, use ``compile_disaster.load_scene_config()``.

    Raises ValueError on missing required keys so a typo fails loudly at startup
    rather than producing an empty scene.
    """
    import yaml  # lazy: keeps the import optional for callers passing dicts

    with open(config_path) as f:
        config = yaml.safe_load(f)

    if isinstance(config, dict):
        config = resolve_asset_pack(config, config_path)
    return validate_config(config, config_path)


# ---------------------------------------------------------------------------
# USD footprint measurement (with graceful fallback to constants)
# ---------------------------------------------------------------------------

def _scaled_footprint(fp: dict, scale: float) -> dict:
    """*fp* measured at unit scale, resized to *scale*.

    Every field `_measure_footprint_raw` returns is a length read off the same
    bbox, so the whole dict is linear in scale and this is exact, not an
    approximation. That is what lets the measurement be cached independently of
    the scale it is asked for — see `SizeResolver.get`.
    """
    s = float(scale)
    return {k: v * s for k, v in fp.items()}


def _measure_footprint_raw(usd_path: str, axis_up: str = "Z"):
    """Open *usd_path* and return its footprint at unit scale, or None.

    Returns ``{"sx","sy","sz","base","cx","cy","cz"}`` in Z-up world space
    (after any axis correction). For Z-up assets cx/cy are the XY bbox center
    offsets from the prim origin; cz is always 0. For Y-up assets
    (``axis_up="Y"``) the measurements are remapped so that sx/sy reflect the
    world-XY footprint, base reflects the local Y_min, and cz carries the local-Z
    centroid that apply_placements folds into the centroid-correction offset vector
    (local Z maps to world -Y after the +90° roll correction).

    Multiply by the placement's scale with `_scaled_footprint`. Opening the
    stage is the expensive part — a Nucleus round trip — so callers should cache
    on ``(usd_path, axis_up)`` and never on the scale.
    """
    # Local schemes are expanded here too: `disaster_stage` injects
    # `airstack://` archetype URLs after the pools were normalised, and USD
    # cannot open the scheme itself — the archetype then measured as the
    # 30 x 20 m category fallback and its footprint check was meaningless.
    try:
        stage = Usd.Stage.Open(_expand_scheme(usd_path) or usd_path)
    except Exception as e:
        print(f"[scene_gen] measure: could not open {usd_path}: {e}")
        return None
    if stage is None:
        return None

    prim = stage.GetDefaultPrim()
    if not (prim and prim.IsValid()):
        children = list(stage.GetPseudoRoot().GetChildren())
        prim = children[0] if children else None
    if not (prim and prim.IsValid()):
        return None

    try:
        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
            useExtentsHint=True,
        )
        rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    except Exception as e:
        print(f"[scene_gen] measure: bbox failed for {usd_path}: {e}")
        return None
    if rng.IsEmpty():
        return None

    mn, sz = rng.GetMin(), rng.GetSize()
    if axis_up == "Y":
        # Y-up asset: a +90° roll in apply_placements converts to Z-up.
        # After rotateX(+90°): local X→world X, local Y→world Z, local Z→world -Y.
        # sx/sy are the world XY footprint; base = -local_Y_min; cy=0 (height
        # centroid doesn't affect XY layout); cz = local Z centroid (fed into the
        # 3-component centroid offset so apply_placements rotates it to world -Y).
        return {
            "sx": sz[0],                    # local X  → world X footprint
            "sy": sz[2],                    # local Z  → world Y footprint
            "sz": sz[1],                    # local Y  → world height
            "base": -mn[1],                 # local Y_min → world Z = 0 after roll
            "cx": mn[0] + sz[0] / 2,        # local X centroid (stays world X)
            "cy": 0.0,                      # height centroid; no XY correction needed
            "cz": mn[2] + sz[2] / 2,        # local Z centroid → world -Y via roll
        }
    return {"sx": sz[0], "sy": sz[1],
            "sz": sz[2], "base": -mn[2],
            # XY offset of the bbox center from the asset's local origin.
            # Non-zero when the pivot isn't at the visual centroid; apply_placements
            # subtracts this so the *visual* center lands at the requested position.
            "cx": mn[0] + sz[0] / 2,
            "cy": mn[1] + sz[1] / 2,
            "cz": 0.0}


def _measure_footprint(usd_path: str, scale: float, axis_up: str = "Z"):
    """`_measure_footprint_raw` at *scale*. Measures every call — no cache.

    Kept for callers outside the generator; `SizeResolver` deliberately does not
    use it, because measuring is what has to happen once per asset rather than
    once per placement.
    """
    fp = _measure_footprint_raw(usd_path, axis_up)
    return None if fp is None else _scaled_footprint(fp, scale)


# Local asset roots, addressed by a pseudo-scheme so a config never has to
# name a machine-specific path. The repo is bind-mounted at a different place
# in the Isaac Sim container (/isaac-sim/AirStack) than on the host, so these
# resolve off this file's location and stay correct in both.
#
#   airstack://<path>    -> <repo root>/<path>
#   objaverse://<path>   -> <repo>/scene_gen/assets/objaverse/<path>
#
# Anything else containing "://" is a real URL (omniverse://, file://, …) and
# passes through untouched.
SCENE_GEN_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(SCENE_GEN_DIR)

LOCAL_ASSET_ROOTS = {
    "airstack": _REPO_ROOT,
    "objaverse": os.path.join(SCENE_GEN_DIR, "assets", "objaverse"),
}


_OBJAVERSE_UID_RE = re.compile(r"^[0-9a-fA-F]{32}$")


def _expand_scheme(path: str):
    """Resolve a ``<name>://`` local asset root to an absolute path, or return
    None if *path* carries no such scheme.

    ``objaverse://<uid>`` is the normal form: an Objaverse asset is identified
    by its dataset uid and nothing else, and the cache slot is derived from it
    (``<uid>/<uid>.usdc``). An explicit sub-path is still honored, so an
    unconverted or hand-placed file under the cache root can be named directly.
    """
    scheme, sep, rest = str(path or "").partition("://")
    if not sep or scheme not in LOCAL_ASSET_ROOTS:
        return None
    rest = rest.lstrip("/")
    if scheme == "objaverse" and _OBJAVERSE_UID_RE.match(rest):
        rest = os.path.join(rest, rest + ".usdc")
    return os.path.join(LOCAL_ASSET_ROOTS[scheme], rest)


def missing_local_assets(config: dict) -> list:
    """`airstack://` paths the config references that are not on disk.

    The sibling of `missing_objaverse_assets`, for the repo-local packs. Those
    are vendor content extracted by hand (see `assets/aec/README.md`), so a
    half-extracted pack is easy to end up with and produces no error at all —
    the geometry silently does not load, or loads without its materials.

    Two real instances, both reported as generator bugs and neither one:
      * the brownstone pack extracted without `Materials/`, so nineteen of its
        twenty MDL bindings dangled and the buildings rendered untextured;
      * the `tower` pack not extracted at all, which is where the park
        vegetation lives — so the parks came out sparse.
    """
    import os

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    seen, missing = set(), []
    scale = float(config.get("asset_scale", 1.0))
    asset_root = str(config.get("asset_root", "") or "")

    def walk(node):
        if isinstance(node, dict):
            for v in node.values():
                walk(v)
        elif isinstance(node, list):
            for entry in node:
                if isinstance(entry, dict) and "usd" in entry:
                    _check(str(entry["usd"]))
                elif isinstance(entry, str):
                    _check(entry)
                elif isinstance(entry, (dict, list)):
                    walk(entry)

    def _check(usd):
        if not usd.startswith("airstack://") or usd in seen:
            return
        seen.add(usd)
        rel = usd[len("airstack://"):]
        if not os.path.exists(os.path.join(root, rel)):
            missing.append(rel)

    walk(config.get("usds") or {})
    return sorted(missing)


def missing_objaverse_assets(config: dict) -> list:
    """Objaverse uids the config references that aren't in the local cache.

    The cache is derived state that ``airstack up`` fills in, so an empty one
    is a fixable condition rather than a broken config — but it would otherwise
    surface as a silent wall of placeholder prisms, so callers report it.
    """
    found, missing = set(), []

    def walk(node):
        if isinstance(node, dict):
            for v in node.values():
                walk(v)
        elif isinstance(node, list):
            for v in node:
                walk(v)
        elif isinstance(node, str):
            m = re.search(r"objaverse://([0-9a-fA-F]{32})", node)
            if m and m.group(1) not in found:
                found.add(m.group(1))
                if not os.path.exists(_expand_scheme(f"objaverse://{m.group(1)}")):
                    missing.append(m.group(1))

    walk(config.get("usds", {}))
    return missing


def _join_asset_root(path: str, asset_root: str) -> str:
    """Prefix *asset_root* onto relative asset paths. Paths with a URL scheme
    (``omniverse://``, ``file://``, …) or a leading ``/`` / ``~`` are already
    absolute and pass through unchanged; a local-root scheme (see
    :data:`LOCAL_ASSET_ROOTS`) expands to an absolute path.
    """
    path = str(path)
    expanded = _expand_scheme(path)
    if expanded is not None:
        return expanded
    if path.startswith(("/", "~")) or "://" in path:
        return path
    # The root itself may be a local-root scheme (a set whose assets are all
    # local can write `asset_root: "objaverse://"`).
    root = _expand_scheme(asset_root) or str(asset_root or "")
    if not root:
        return path
    return root.rstrip("/") + "/" + path


def resolve_sky(config: dict) -> str:
    """Resolve the top-level ``sky`` config entry against ``asset_root``.
    Returns "" if no ``sky`` is configured.

    The entry is either an equirect HDRI (``.exr``/``.hdr``) or a stage USD
    whose root sky/environment prims should be borrowed. Callers pass the
    result to ``scene_prep.add_sky(stage, ...)``, which dispatches on the
    extension — this module stays sim-agnostic, so it only resolves the path
    rather than touching the stage.
    """
    sky = str(config.get("sky", "") or "").strip()
    if not sky:
        return ""
    return _join_asset_root(sky, str(config.get("asset_root", "") or ""))


def _parse_usd_entry(entry, default_scale: float, asset_root: str = ""):
    """Normalize a USD entry to ``(path, scale, axis_up, yaw_offset, tags)``.

    Entries can be plain strings (use *default_scale*, Z-up) or dicts::

        # plain — uses global asset_scale, Z-up
        - "omniverse://host/Props/SM_Grass.usd"

        # per-asset overrides
        - usd: "omniverse://host/Props/SM_Building.usd"
          scale: 1.0      # already in meters; global asset_scale is e.g. 0.01
          axis-up: "Y"    # authored Y-up (e.g. Unity export); corrected via +90° roll
          yaw-offset: 90  # deg — art doesn't face +X; added to every computed yaw
          tags: ["park"]  # placement context: "park" props go on park trails,
                          # untagged ones on street frontage
          solid: true     # the model already has material in its walls, so
                          # mesh damage must NOT thicken it. Omitted means
                          # false: everything is treated as a shell and gets
                          # solidified, which is what almost all of this
                          # library is. See `solid_assets`.

    Relative paths get *asset_root* prefixed (see :func:`_join_asset_root`).
    """
    if isinstance(entry, dict):
        return (_join_asset_root(str(entry["usd"]), asset_root),
                float(entry.get("scale", default_scale)),
                str(entry.get("axis-up", "Z")).upper(),
                float(entry.get("yaw-offset", 0.0)),
                frozenset(str(t) for t in entry.get("tags", ())))
    return (_join_asset_root(str(entry), asset_root), float(default_scale),
            "Z", 0.0, frozenset())


def solid_assets(config: dict) -> set:
    """USD paths the asset pack declares as already solid, so `solidify` skips them.

    **Declared, not detected.** This used to be decided at runtime — a ray probe
    asking how much material sat behind each surface, and before that, wrongly,
    the enclosed volume. Both are guesses about art, remade every run, on
    geometry whose author already knows the answer. A flag in the asset pack says
    it once:

        buildings:
          damaged:
            - usd: "omniverse://.../SM_SolidTower.usd"
              solid: true

    The default is FALSE — an unmarked asset is treated as a shell and gets
    thickened. That is the right default because it is what nearly all of this
    library is, and because the two failure modes are not symmetric: thickening
    a solid model costs points, while failing to thicken a shell leaves
    zero-thickness walls at every cut, which is the artifact the operator exists
    to remove.

    Walks the whole `usds:` tree, so the flag works in any pool — buildings,
    debris, props — not only the ones mesh damage reads today.
    """
    root = str(config.get("asset_root", "") or "")
    out: set = set()

    def walk(node):
        if isinstance(node, dict):
            if isinstance(node.get("usd"), str):
                if bool(node.get("solid", False)):
                    # BOTH forms. `airstack://x` resolves to an absolute path,
                    # so a caller comparing the string a human wrote would miss
                    # — and a miss here is silent, showing up only as a
                    # building that was or was not thickened.
                    raw = str(node["usd"])
                    out.add(raw)
                    out.add(_join_asset_root(raw, root))
                return
            for v in node.values():
                walk(v)
        elif isinstance(node, (list, tuple)):
            for v in node:
                walk(v)

    walk(config.get("usds") or {})
    return out


# ---------------------------------------------------------------------------
# buildings: condition x typology
# ---------------------------------------------------------------------------

#: What CONDITION a building asset is in. Carried as a TAG on the entry, so one
#: typology pool holds the standing tower and the ruined one side by side::
#:
#:     buildings:
#:       tower:
#:         - {usd: ".../BG_Building_A.usd", material: concrete, tags: [intact]}
#:         - {usd: ".../ruin_tower_01.usdc", material: steel,   tags: [destroyed]}
#:
#: The older layout put the condition in the KEY instead — `buildings.intact`,
#: `buildings.damaged`, `buildings.destroyed`, each optionally split by
#: typology — which duplicated the whole typology tree once per condition and
#: left a building's typology unstated in the flat `damaged` pool. Both layouts
#: are read (see `building_entries`); packs are converted one at a time.
BUILDING_CONDITIONS = ("intact", "damaged", "destroyed")

#: An entry with no condition tag is intact art: that is what nearly every pool
#: is made of, so only the ruins have to say so.
DEFAULT_CONDITION = "intact"


def _building_section(config: dict) -> dict:
    """`usds.buildings` (or the legacy `usds.houses`)."""
    usds = config.get("usds") or {}
    return usds.get("buildings") or usds.get("houses") or {}


def _is_tagged_layout(bld: dict) -> bool:
    """True when the section is keyed by TYPOLOGY and conditions are tags.

    Decided by the keys rather than by a version field: a section holding any
    of `intact` / `damaged` / `destroyed` as a key is the old layout, and one
    that holds none of them is the new one. A pack cannot be half-converted
    without the mixture being obvious here.
    """
    return not any(k in BUILDING_CONDITIONS for k in (bld or {}))


def condition_of(entry, default: str = DEFAULT_CONDITION) -> str:
    """The condition tag on one `usds.buildings` entry, or *default*."""
    if isinstance(entry, dict):
        for t in entry.get("tags") or ():
            if str(t) in BUILDING_CONDITIONS:
                return str(t)
    return default


def _flatten_pool(node) -> list:
    """A pool as a flat entry list, whether it is one or a dict of sub-pools."""
    if isinstance(node, dict) and "usd" not in node:
        return [e for v in node.values() for e in _flatten_pool(v)]
    if isinstance(node, (list, tuple)):
        return [e for v in node for e in _flatten_pool(v)]
    return [node] if node else []


def building_entries(config: dict, condition: str = None,
                     typology: str = None) -> list:
    """Raw `usds.buildings` entries for ONE pool, from either layout.

    *condition* is `intact` / `damaged` / `destroyed`; *typology* is a
    `districts.typologies` name (`tower`, `midrise`, `rowhouse`). Either may be
    None for "any". Entries come back as the pack wrote them, so a caller still
    runs them through `_normalize_usd_list`.

    THE LEGACY BRANCH REPRODUCES THE OLD LOOKUPS EXACTLY, and deliberately so:
    an old pack keeps the same pools it had, because those pools decide the
    LAYOUT (`build_city` packs from `buildings.intact`) and a wider pool would
    silently rebuild every scene that pack has ever produced. In that layout a
    typology pool lives at the TOP level and holds intact art, which is why a
    typology query looks there first.
    """
    bld = _building_section(config)
    if _is_tagged_layout(bld):
        out = []
        for typ, pool in bld.items():
            if typology is not None and typ != typology:
                continue
            for e in _flatten_pool(pool):
                if condition is None or condition_of(e) == condition:
                    out.append(e)
        return out

    if typology is None:
        if condition is None:
            return [e for k in bld for e in _flatten_pool(bld[k])]
        return _flatten_pool(bld.get(condition))
    if condition in (None, DEFAULT_CONDITION) \
            and isinstance(bld.get(typology), (list, tuple)):
        return list(bld[typology])
    node = bld.get(condition or DEFAULT_CONDITION)
    return _flatten_pool(node.get(typology)) if isinstance(node, dict) else []


def every_building(config: dict, condition: str = DEFAULT_CONDITION) -> list:
    """Every DISTINCT entry of *condition*, whichever pool it sits in.

    Differs from `building_entries` only in the legacy layout, where the same
    intact building is listed twice — once in the flat `buildings.intact` pool
    that `build_city` packs from, once in the typology pool that `districts`
    rezones from. A generator draws from one pool at a time; the Stage A baker
    has to enumerate the UNION or it leaves half the library unbaked. In the
    tagged layout there is only one list and the two agree.
    """
    bld = _building_section(config)
    if _is_tagged_layout(bld):
        return building_entries(config, condition=condition)
    keys = [condition]
    if condition == DEFAULT_CONDITION:
        keys += [k for k in bld if k not in BUILDING_CONDITIONS]
    out, seen = [], set()
    for k in keys:
        for e in _flatten_pool(bld.get(k)):
            usd = e.get("usd") if isinstance(e, dict) else e
            if usd in seen:
                continue
            seen.add(usd)
            out.append(e)
    return out


def asset_materials(config: dict) -> dict:
    """USD path -> the construction the asset pack declares for it.

    The sibling of `solid_assets`, and declared for the same reason: what a
    building is made of is a fact about the art, known to whoever authored it,
    and not something to re-derive every run.

        buildings:
          tower:
            - {usd: ".../BG_Building_F.usd", material: glass}

    Read by `mesh_damage.apply_to_stage`, which turns the FACADE named here
    into the STRUCTURE behind it (`mesh_damage.STRUCTURE_OF`) — a curtain-wall
    tower is glass on the outside and a steel frame at every break. Unlisted
    assets fall back to the scene-wide `mesh_damage.material`, which is a
    property of the locale rather than of the building.
    """
    root = str(config.get("asset_root", "") or "")
    out: dict = {}

    def walk(node):
        if isinstance(node, dict):
            if isinstance(node.get("usd"), str):
                kind = node.get("material")
                if kind:
                    raw = str(node["usd"])
                    out[raw] = str(kind)
                    out[_join_asset_root(raw, root)] = str(kind)
                return
            for v in node.values():
                walk(v)
        elif isinstance(node, (list, tuple)):
            for v in node:
                walk(v)

    walk(config.get("usds") or {})
    return out


def _normalize_usd_list(lst, default_scale: float, asset_root: str = ""):
    """Return ``(path_list, scale_overrides, axis_up_overrides, yaw_overrides,
    tag_overrides)`` from a raw YAML USD list.

    *path_list* is a plain list of USD paths suitable for ``rng.choice()``.
    *scale_overrides* maps path → per-asset scale for entries with an explicit ``scale``.
    *axis_up_overrides* maps path → axis_up string ("Y") for non-Z-up entries.
    *yaw_overrides* maps path → yaw-offset degrees for entries that declare one.
    *tag_overrides* maps path → frozenset of tags for entries that declare any.
    """
    if isinstance(lst, dict) and "usd" not in lst:
        # A pool split into named sub-pools — `intact: {tower: [...],
        # midrise: [...]}` — reads flat here; `districts` picks the sub-pools
        # apart by name.
        lst = [e for v in lst.values()
               for e in (v if isinstance(v, (list, tuple)) else [v])]
    if isinstance(lst, (str, dict)):
        lst = [lst]
    paths, scale_ovr, axisup_ovr, yaw_ovr, tag_ovr = [], {}, {}, {}, {}
    for entry in (lst or []):
        path, sc, au, yo, tags = _parse_usd_entry(entry, default_scale, asset_root)
        paths.append(path)
        if sc != default_scale:
            scale_ovr[path] = sc
        if au != "Z":
            axisup_ovr[path] = au
        if yo != 0.0:
            yaw_ovr[path] = yo
        if tags:
            tag_ovr[path] = tags
    return paths, scale_ovr, axisup_ovr, yaw_ovr, tag_ovr


class SizeResolver:
    """Resolves and caches each USD's metric footprint, measuring when possible
    and falling back to per-category constants otherwise.

    MEASURE ONCE PER ASSET, NOT ONCE PER PLACEMENT
    ----------------------------------------------
    A measurement is a `Usd.Stage.Open` plus a bbox compute — for a Nucleus
    asset, a network round trip. The cache used to be keyed on
    ``(path, scale, axis_up)``, and `disaster_stage` gives every debris prop an
    individually randomised scale (``_sc(du) * rng.uniform(0.7, 1.2)``), so
    **every debris placement missed the cache and re-opened the asset**. On a
    measured `earthquake` run that was 3,971 measurements of 150 distinct
    assets — 97% of them redundant, all inside `apply_placements`, which took
    10m37s of a 78-minute launch.

    Footprint is exactly linear in scale (see `_scaled_footprint`), so the
    measurement is cached on ``(path, axis_up)`` alone and the scale is applied
    afterwards. `_by_scale` still memoises the scaled dicts so a repeated
    lookup is a dict hit rather than a rebuild, but a miss there costs a
    multiply, not a stage open.

    A *failed* measurement is cached too, as ``None``. It used to be retried
    per placement, so one unresolvable asset paid the full open-and-fail
    thousands of times and printed a fallback line for each.
    """

    def __init__(self, asset_scale: float, fallback_sizes: dict, measure: bool,
                 cache=None):
        self.scale = float(asset_scale)
        self.fallback = fallback_sizes or {}
        self.measure = bool(measure)
        self._raw: dict = {}            # (path, axis_up) -> unit-scale fp or None
        self._by_scale: dict = {}       # (path, scale, axis_up) -> scaled fp
        # Assets whose footprint is a per-category GUESS rather than a
        # measurement. A guessed footprint is not a cosmetic downgrade: block
        # sizing is driven by how big the buildings are, so a run that guesses
        # produces a DIFFERENT LAYOUT from one that measures — 638 placements
        # and 6 buildings against 784 and 4, on the same config and seed. That
        # is only discoverable by comparing two runs, so callers that persist a
        # scene (`bake_scene`, `scene_cache`) read this and refuse.
        self.fallbacks: set = set()
        # Persistent across PROCESSES, where `_raw` is only across placements.
        # SPEC names "measure assets / use cached" as a pipeline step; this is
        # the "cached" half. None disables it (the tests want a cold resolver).
        self._cache = cache

    def _raw_footprint(self, usd_path: str, category: str, axis_up: str):
        """Unit-scale measurement for *usd_path*, or None. Opens the USD once."""
        key = (usd_path, axis_up)
        if key in self._raw:
            return self._raw[key]

        if self._cache is not None and self.measure:
            hit, fp = self._cache.get(usd_path, axis_up)
            if hit:
                self._raw[key] = fp
                return fp

        fp = _measure_footprint_raw(usd_path, axis_up) if self.measure else None
        self._raw[key] = fp
        if self._cache is not None and self.measure:
            self._cache.put(usd_path, axis_up, fp)
        if fp is not None:
            print(f"[scene_gen] measured {category}: {os.path.basename(usd_path)} "
                  f"-> {fp['sx']:.2f} x {fp['sy']:.2f} m "
                  f"(unit scale, up={axis_up})")
        else:
            self.fallbacks.add(usd_path)
            fb = self.fallback.get(category, [4.0, 4.0])
            print(f"[scene_gen] fallback {category}: {os.path.basename(usd_path)} "
                  f"-> {float(fb[0]):.2f} x {float(fb[1]):.2f} m")
        return fp

    def get(self, usd_path: str, category: str, scale: float = None,
            axis_up: str = "Z") -> dict:
        """Return the metric footprint for *usd_path* in Z-up world space.

        *scale* overrides the resolver's default ``asset_scale``.
        *axis_up* is ``"Y"`` for assets authored in Y-up coordinates (e.g. Unity
        exports); the returned dict has corrected sx/sy/base and includes a ``cz``
        field so apply_placements can handle the centroid offset correctly.
        """
        effective_scale = float(scale) if scale is not None else self.scale
        effective_axis_up = str(axis_up).upper() if axis_up else "Z"
        cache_key = (usd_path, effective_scale, effective_axis_up)
        if cache_key in self._by_scale:
            return self._by_scale[cache_key]

        raw = self._raw_footprint(usd_path, category, effective_axis_up)
        if raw is not None:
            fp = _scaled_footprint(raw, effective_scale)
        else:
            # Fallback sizes are already metric, so the placement scale does NOT
            # apply to them — that was true before this change too.
            fb = self.fallback.get(category, [4.0, 4.0])
            fp = {"sx": float(fb[0]), "sy": float(fb[1]),
                  "sz": float(fb[2]) if len(fb) > 2 else 3.0, "base": 0.0,
                  "cx": 0.0, "cy": 0.0, "cz": 0.0}

        self._by_scale[cache_key] = fp
        return fp


def placement_footprint(resolver, p: dict, category: str = None) -> dict:
    """Footprint of a placement that already exists, measured as it was placed.

    **Scale and axis belong to the placement.** Every pass that re-derives them
    from an asset-pack pool gets them wrong for anything the pass did not
    normalise itself: `disaster_stage` looked up standing buildings through the
    per-asset overrides of its *ruin* pools, missed, and fell back to the
    config-wide `asset_scale` (1.0) while the AEC packs carry `scale: 0.01`.
    The same run then measured the same asset twice —

        Reference_Brownstone12Row     21.11 x   80.06 m   (scale=0.01)
        Reference_Brownstone12Row   2111.27 x 8005.56 m   (scale=1.0)

    — and the inflated one is what the debris ring walked outward from, which
    put rubble kilometres outside an 800 m region. `tools/plan_png` had the
    identical bug for the same reason.

    So there is one function for it, and every caller that holds a placement
    uses this rather than restating the rule.
    """
    return resolver.get(p.get("usd", ""),
                        category or p.get("category", "asset"),
                        scale=float(p.get("scale", 1.0) or 1.0),
                        axis_up=str(p.get("axis_up", "Z") or "Z"))


# ---------------------------------------------------------------------------
# Exclusion zones — keep spawn points / corridors clear of *clutter*
# (ground + roads ignore these; houses / trees / streetlights honor them)
# ---------------------------------------------------------------------------

def _in_exclusion(x: float, y: float, exclusions: list) -> bool:
    for ex in exclusions:
        if "radius" in ex:
            cx, cy = ex["center"]
            if (x - cx) ** 2 + (y - cy) ** 2 <= float(ex["radius"]) ** 2:
                return True
        elif "bounds" in ex:
            (xmin, ymin), (xmax, ymax) = ex["bounds"]
            if xmin <= x <= xmax and ymin <= y <= ymax:
                return True
    return False


def _in_rect(x, y, rect) -> bool:
    """rect = (xmin, ymin, xmax, ymax)."""
    return rect[0] <= x <= rect[2] and rect[1] <= y <= rect[3]


# ---------------------------------------------------------------------------
# Damage field — where the disaster actually hit
#
# The model itself lives in `disaster/field.py`; this file is the USD writer
# and had no business owning it. Re-exported here because `build_city` and a
# dozen callers import these names from `scene_generator`.
# ---------------------------------------------------------------------------

from disaster.field import (                        # noqa: E402,F401
    make_damage_field, make_scour_density,
    path_segments as _path_segments,
    point_segment_dist as _point_segment_dist,
    smoothstep as _smoothstep,
)


# ---------------------------------------------------------------------------
# Procedural layout — recursive subdivision (roads) + lot parcelling
#
# Pipeline follows the standard procedural-city literature (Parish-Müller 2001;
# Vanegas 2012 "Procedural Generation of Parcels"): a road network divides the
# region into blocks, each block is recursively subdivided into lots, and a
# building is fit into each lot. Specialized here to axis-aligned rectangles so
# everything stays tileable with the square road/ground assets (no diagonals).
# ---------------------------------------------------------------------------

def _jitter_posf(lo: float, hi: float, jitter: float, rng) -> float:
    """Float version of :func:`_jitter_pos` (for metric lot splits)."""
    if hi <= lo:
        return lo
    mid = (lo + hi) / 2.0
    half = (hi - lo) / 2.0
    return min(hi, max(lo, mid + (rng.random() * 2.0 - 1.0) * jitter * half))


def _subdivide_region_metric(w_m: float, h_m: float,
                              min_block_m: float, max_block_m: float,
                              jitter: float, rng, roads_cfg: dict):
    """Recursively subdivide a ``w_m × h_m`` metric region using BSP.

    Returns ``(blocks, road_corridors)`` in meter coordinates centered on the
    origin. Each block is ``(x0, y0, x1, y1)``; each corridor is a dict
    ``{"x0", "y0", "x1", "y1", "n_lanes", "dir": "ns"|"ew"}``.

    A border ring of roads forms the outer frame; the BSP then recursively
    splits the interior. Each split is randomly promoted to a main (wide) road
    based on *roads_cfg.main_road_chance*.
    """
    lane_w      = float(roads_cfg.get("lane_width_m", 3.5))
    main_lanes  = int(roads_cfg.get("main_road_lanes", 4))
    sec_lanes   = int(roads_cfg.get("secondary_road_lanes", 2))
    brd_lanes   = int(roads_cfg.get("border_lanes", 2))
    main_chance = float(roads_cfg.get("main_road_chance", 0.25))

    border_w = brd_lanes * lane_w
    half_w, half_h = w_m / 2.0, h_m / 2.0

    road_corridors: list = []

    # Border ring: 4 outer strips
    road_corridors.append({"x0": -half_w, "y0": -half_h,
                            "x1":  half_w, "y1": -half_h + border_w,
                            "n_lanes": brd_lanes, "dir": "ew"})
    road_corridors.append({"x0": -half_w, "y0":  half_h - border_w,
                            "x1":  half_w, "y1":  half_h,
                            "n_lanes": brd_lanes, "dir": "ew"})
    road_corridors.append({"x0": -half_w,            "y0": -half_h + border_w,
                            "x1": -half_w + border_w, "y1":  half_h - border_w,
                            "n_lanes": brd_lanes, "dir": "ns"})
    road_corridors.append({"x0":  half_w - border_w, "y0": -half_h + border_w,
                            "x1":  half_w,            "y1":  half_h - border_w,
                            "n_lanes": brd_lanes, "dir": "ns"})

    blocks: list = []

    def _recurse(x0, y0, x1, y1):
        w, h = x1 - x0, y1 - y0
        if w <= 1e-3 or h <= 1e-3:
            return
        min_road = sec_lanes * lane_w
        can_x = w >= 2.0 * min_block_m + min_road
        can_y = h >= 2.0 * min_block_m + min_road
        if not (can_x or can_y):
            blocks.append((x0, y0, x1, y1))
            return
        too_big = w > max_block_m or h > max_block_m
        if not too_big and rng.random() < 0.5:
            blocks.append((x0, y0, x1, y1))
            return
        if can_x and (w >= h or not can_y):
            is_main = rng.random() < main_chance
            n_lanes = main_lanes if is_main else sec_lanes
            road_w = n_lanes * lane_w
            lo = x0 + min_block_m + road_w / 2.0
            hi = x1 - min_block_m - road_w / 2.0
            if lo >= hi:
                blocks.append((x0, y0, x1, y1))
                return
            sp = _jitter_posf(lo, hi, jitter, rng)
            road_corridors.append({"x0": sp - road_w / 2.0, "y0": y0,
                                    "x1": sp + road_w / 2.0, "y1": y1,
                                    "n_lanes": n_lanes, "dir": "ns"})
            _recurse(x0, y0, sp - road_w / 2.0, y1)
            _recurse(sp + road_w / 2.0, y0, x1, y1)
        else:
            is_main = rng.random() < main_chance
            n_lanes = main_lanes if is_main else sec_lanes
            road_w = n_lanes * lane_w
            lo = y0 + min_block_m + road_w / 2.0
            hi = y1 - min_block_m - road_w / 2.0
            if lo >= hi:
                blocks.append((x0, y0, x1, y1))
                return
            sp = _jitter_posf(lo, hi, jitter, rng)
            road_corridors.append({"x0": x0, "y0": sp - road_w / 2.0,
                                    "x1": x1, "y1": sp + road_w / 2.0,
                                    "n_lanes": n_lanes, "dir": "ew"})
            _recurse(x0, y0, x1, sp - road_w / 2.0)
            _recurse(x0, sp + road_w / 2.0, x1, y1)

    _recurse(-half_w + border_w, -half_h + border_w,
              half_w - border_w,  half_h - border_w)
    return blocks, road_corridors


def _catmull_rom_points(way, samples_per_seg: int = 8):
    """Sample a Catmull-Rom spline through waypoints ``[(x, y), ...]``.

    Endpoints are duplicated so the curve passes through the first and last
    waypoint. Returns a dense polyline ``[(x, y), ...]``.
    """
    if len(way) < 3:
        return list(way)
    pts = [way[0]] + list(way) + [way[-1]]
    out = []
    for i in range(len(pts) - 3):
        p0, p1, p2, p3 = pts[i], pts[i + 1], pts[i + 2], pts[i + 3]
        for k in range(samples_per_seg):
            t = k / samples_per_seg
            t2, t3 = t * t, t * t * t
            out.append(tuple(
                0.5 * ((2.0 * p1[j]) + (-p0[j] + p2[j]) * t
                       + (2.0 * p0[j] - 5.0 * p1[j] + 4.0 * p2[j] - p3[j]) * t2
                       + (-p0[j] + 3.0 * p1[j] - 3.0 * p2[j] + p3[j]) * t3)
                for j in (0, 1)))
    out.append(way[-1])
    return out


def _walk_polyline(pts, step: float, phase: float = 0.0):
    """Yield ``(x, y, yaw_deg)`` every *step* meters of arc length along the
    polyline *pts*, starting *phase* × *step* in. Yaw is the local travel
    direction (deg)."""
    if len(pts) < 2 or step <= 1e-9:
        return
    next_s = phase * step
    walked = 0.0
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        seg = math.hypot(bx - ax, by - ay)
        if seg < 1e-9:
            continue
        yaw = math.degrees(math.atan2(by - ay, bx - ax))
        while next_s <= walked + seg:
            t = (next_s - walked) / seg
            yield ax + (bx - ax) * t, ay + (by - ay) * t, yaw
            next_s += step
        walked += seg


# Muted facade tones for placeholder buildings (filler prisms).
_PLACEHOLDER_COLORS = (
    (0.76, 0.72, 0.65),   # warm beige
    (0.72, 0.73, 0.75),   # light gray
    (0.70, 0.62, 0.55),   # muted clay
    (0.64, 0.69, 0.72),   # blue-gray
    (0.73, 0.70, 0.58),   # sandy tan
    (0.67, 0.71, 0.64),   # sage
)


def _pack_block_with_buildings(rect, candidates, gap: float,
                               min_leftover: float, rng, first=None,
                               pick_fn=None):
    """Greedily pack building *candidates* into *rect* (guillotine packing).

    Each step places a randomly chosen fitting building in the rect's min
    corner, then splits the remaining L-shape into two rects (cut along the
    axis with the larger leftover, so the bigger remainder stays whole) and
    recurses. Buildings touch the rect edges — a block packed this way is
    built out to its sidewalk — separated from each other by *gap*.

    *candidates* is ``[(key, sx, sy), ...]`` footprints, reusable any number of
    times. *first*, if given, is a key that must be placed before random
    picking starts (used to guarantee big buildings their reserved block).
    *pick_fn*, if given, replaces the uniform random pick: it receives the
    fitting ``(key, w, h, yaw)`` options plus the slot's anchor corner
    ``(x, y)`` and returns one option — used to weight the choice between
    intact/damaged/destroyed pools by the local damage intensity.

    Returns ``(placed, leftovers)``: *placed* is
    ``[(key, cx, cy, fit_yaw, (bx0, by0, bx1, by1)), ...]``; *leftovers* are
    rects (>= *min_leftover* on both axes) where no candidate fits — filler
    placeholders go there.
    """
    placed, leftovers = [], []
    stack = [tuple(rect)]
    forced = first
    while stack:
        x0, y0, x1, y1 = stack.pop()
        w, h = x1 - x0, y1 - y0
        if w < min_leftover or h < min_leftover:
            continue                              # sliver → stays pavement
        fits = []
        for key, sx, sy in candidates:
            if sx <= w and sy <= h:
                fits.append((key, sx, sy, 0.0))
            if sy <= w and sx <= h and abs(sx - sy) > 1e-6:
                fits.append((key, sy, sx, 90.0))
        if not fits:
            leftovers.append((x0, y0, x1, y1))
            continue
        pick_from = [c for c in fits if c[0] == forced] if forced else []
        if pick_from:
            forced = None
            key, bw, bh, yaw = rng.choice(pick_from)
        else:
            key, bw, bh, yaw = (pick_fn(fits, x0, y0) if pick_fn
                                else rng.choice(fits))
        bx1, by1 = x0 + bw, y0 + bh
        placed.append((key, x0 + bw / 2.0, y0 + bh / 2.0, yaw,
                       (x0, y0, bx1, by1)))
        right_w, top_h = x1 - bx1 - gap, y1 - by1 - gap
        if right_w >= top_h:                      # vertical cut: right strip whole
            if right_w > 0:
                stack.append((bx1 + gap, y0, x1, y1))
            if top_h > 0:
                stack.append((x0, by1 + gap, bx1, y1))
        else:                                     # horizontal cut: top strip whole
            if top_h > 0:
                stack.append((x0, by1 + gap, x1, y1))
            if right_w > 0:
                stack.append((bx1 + gap, y0, x1, by1))
    return placed, leftovers


def _make_plane_mesh(stage, prim_path: str,
                     x0: float, y0: float, x1: float, y1: float,
                     z_m: float, uv_scale_m: float, ssf: float,
                     display_color=None, mat_prim_path: str = ""):
    """Define a quad UsdGeom.Mesh spanning (x0,y0)-(x1,y1) at height z_m.

    UV coordinates tile at 1 UV unit per *uv_scale_m* meters so tiling
    materials repeat at a consistent real-world density. *ssf* converts
    meters to stage units. *display_color* is an (r,g,b) float triple shown
    in the viewport when no material is bound.
    """
    w, h = x1 - x0, y1 - y0
    s = ssf
    pts = Vt.Vec3fArray([
        Gf.Vec3f(x0 * s, y0 * s, z_m * s),
        Gf.Vec3f(x1 * s, y0 * s, z_m * s),
        Gf.Vec3f(x1 * s, y1 * s, z_m * s),
        Gf.Vec3f(x0 * s, y1 * s, z_m * s),
    ])
    u_max = w / uv_scale_m if uv_scale_m > 1e-9 else 1.0
    v_max = h / uv_scale_m if uv_scale_m > 1e-9 else 1.0
    uvs = Vt.Vec2fArray([
        Gf.Vec2f(0.0, 0.0), Gf.Vec2f(u_max, 0.0),
        Gf.Vec2f(u_max, v_max), Gf.Vec2f(0.0, v_max),
    ])

    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(prim_path))
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * 4))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateSubdivisionSchemeAttr("none")

    pvapi = UsdGeom.PrimvarsAPI(mesh)
    pv = pvapi.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                              UsdGeom.Tokens.vertex)
    pv.Set(uvs)

    if display_color:
        r, g, b = display_color
        mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(r, g, b)]))

    if mat_prim_path:
        mat = stage.GetPrimAtPath(mat_prim_path)
        if mat.IsValid():
            UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(
                UsdShade.Material(mat))

    return mesh


def _make_dash_mesh(stage, prim_path: str,
                    cx: float, cy: float, z_m: float,
                    length: float, width: float, yaw_deg: float,
                    ssf: float, display_color=None):
    """Create a single lane-marking dash as a thin UsdGeom.Mesh quad.

    The dash is centred at *(cx, cy)* at height *z_m* with its long axis at
    *yaw_deg* (0° = parallel to +X, 90° = parallel to +Y).
    """
    cos_y = math.cos(math.radians(yaw_deg))
    sin_y = math.sin(math.radians(yaw_deg))
    hl, hw = length / 2.0, width / 2.0

    def _rot(lx, ly):
        return (cx + cos_y * lx - sin_y * ly,
                cy + sin_y * lx + cos_y * ly)

    corners = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
    s = ssf
    pts = Vt.Vec3fArray([
        Gf.Vec3f(_rot(lx, ly)[0] * s, _rot(lx, ly)[1] * s, z_m * s)
        for lx, ly in corners
    ])

    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(prim_path))
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * 4))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateSubdivisionSchemeAttr("none")

    if display_color:
        r, g, b = display_color
        mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(r, g, b)]))

    return mesh


def _make_box_mesh(stage, prim_path: str,
                   x0: float, y0: float, x1: float, y1: float,
                   z0_m: float, height_m: float, ssf: float,
                   display_color=None):
    """Define a closed rectangular prism UsdGeom.Mesh from (x0,y0,z0) up to
    z0+height. Used for placeholder filler buildings. Being a Mesh (not a
    Cube) keeps it visible to scene_prep.add_colliders, so drones can sense it.
    """
    s = ssf
    xa, xb = x0 * s, x1 * s
    ya, yb = y0 * s, y1 * s
    za, zb = z0_m * s, (z0_m + height_m) * s
    P = Gf.Vec3f
    pts = Vt.Vec3fArray([
        P(xa, ya, za), P(xb, ya, za), P(xb, yb, za), P(xa, yb, za),
        P(xa, ya, zb), P(xb, ya, zb), P(xb, yb, zb), P(xa, yb, zb),
    ])
    # CCW-from-outside quads (right-handed): -Z, +Z, -Y, +X, +Y, -X
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    face_normals = [(0, 0, -1), (0, 0, 1), (0, -1, 0),
                    (1, 0, 0), (0, 1, 0), (-1, 0, 0)]

    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(prim_path))
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 6))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
    mesh.CreateNormalsAttr(Vt.Vec3fArray(
        [Gf.Vec3f(*n) for n in face_normals for _ in range(4)]))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.CreateSubdivisionSchemeAttr("none")

    if display_color:
        r, g, b = display_color
        mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(r, g, b)]))

    return mesh


def apply_ground_planes(stage, config: dict, layout: dict,
                         parent_path: str, ssf: float):
    """Create procedural ground geometry (asphalt, grass, lane markings).

    Three layers are written under *parent_path/ground*:
    1. One asphalt UsdGeom.Mesh spanning the whole region at z=0.
    2. One grass UsdGeom.Mesh per block at z=0.01 (sits above asphalt so
       exposed road gaps between blocks show asphalt through).
    3. Lane-marking dashes at z=0.02 per road corridor: a yellow double
       centre-line (opposing-direction boundary) and white dashes at each
       same-direction lane boundary.
    4. Placeholder filler buildings (*parent_path/placeholders*): mild-colored
       box meshes filling block gaps no library building fits
       (``layout["placeholder_buildings"]``, computed by :func:`build_city`).

    *layout* must contain ``region``, ``blocks``, and ``road_corridors`` as
    returned by :func:`build_city`.
    """
    rx0, ry0, rx1, ry1 = layout["region"]
    blocks         = layout["blocks"]
    road_corridors = layout["road_corridors"]

    roads_cfg  = _stage(config, "layout").get("roads", {})
    lane_w_m   = float(roads_cfg.get("lane_width_m", 3.5))
    uv_asphalt = float(roads_cfg.get("asphalt_uv_scale_m", 4.0))
    uv_grass   = float(roads_cfg.get("grass_uv_scale_m", 3.0))
    ll_cfg     = roads_cfg.get("lane_lines", {})
    dash_len   = float(ll_cfg.get("dash_length_m", 3.0))
    dash_gap   = float(ll_cfg.get("dash_gap_m", 3.0))
    dash_w     = float(ll_cfg.get("dash_width_m", 0.15))
    cline_off  = float(ll_cfg.get("center_line_width_m", 0.20)) / 2.0

    usds_cfg   = config.get("usds", {})
    mat_cfg    = usds_cfg.get("materials", {})
    asset_root = str(config.get("asset_root", "") or "").rstrip("/")

    gnd = parent_path + "/ground"
    UsdGeom.Scope.Define(stage, Sdf.Path(gnd))

    # Reference material USDs into the stage so they can be bound to mesh prims.
    # _load_mat() defines a prim at a known stage path and adds the material USD
    # as a reference; the material's default prim composes at that path, making
    # it a valid UsdShade.Material target for MaterialBindingAPI.
    mat_scope = gnd + "/materials"
    UsdGeom.Scope.Define(stage, Sdf.Path(mat_scope))

    def _load_mat(key):
        url = mat_cfg.get(key, "")
        if not url:
            return ""
        url = _join_asset_root(url, asset_root)
        prim_path = mat_scope + "/" + key
        prim = stage.DefinePrim(Sdf.Path(prim_path))
        prim.GetReferences().AddReference(url)
        prim.Load()  # force-load any nested payload (see apply_placements)
        return prim_path

    asphalt_mat = _load_mat("asphalt")
    grass_mat   = _load_mat("grass")

    # 1) Asphalt base (whole region, z=0)
    _make_plane_mesh(stage, gnd + "/asphalt_base",
                     rx0, ry0, rx1, ry1, 0.0, uv_asphalt, ssf,
                     display_color=(0.15, 0.15, 0.15),
                     mat_prim_path=asphalt_mat)

    # 2) Grass per block (z=0.01 — covers asphalt; road corridors show through)
    for idx, (bx0, by0, bx1, by1) in enumerate(blocks):
        _make_plane_mesh(stage, f"{gnd}/grass_{idx}",
                         bx0, by0, bx1, by1, 0.01, uv_grass, ssf,
                         display_color=(0.2, 0.5, 0.1),
                         mat_prim_path=grass_mat)

    # 3) Lane markings (z=0.02)
    period = dash_len + dash_gap
    dash_i = 0
    for corr in road_corridors:
        n_lanes = corr["n_lanes"]
        dir_    = corr["dir"]

        if dir_ == "ns":
            # Vertical corridor: markings run N-S (yaw=90°)
            road_len = corr["y1"] - corr["y0"]
            mid_x    = (corr["x0"] + corr["x1"]) / 2.0
            n_dashes = max(1, int(road_len / period))
            step_y   = road_len / n_dashes
            ys = [corr["y0"] + (k + 0.5) * step_y for k in range(n_dashes)]

            for y in ys:
                for off in (-cline_off, cline_off):
                    _make_dash_mesh(stage, f"{gnd}/dash_{dash_i}",
                                    mid_x + off, y, 0.02,
                                    dash_len, dash_w, 90.0, ssf,
                                    display_color=(1.0, 1.0, 0.0))
                    dash_i += 1

            for sign in (-1.0, 1.0):
                for div in range(1, n_lanes // 2):
                    wx = mid_x + sign * div * lane_w_m
                    for y in ys:
                        _make_dash_mesh(stage, f"{gnd}/dash_{dash_i}",
                                        wx, y, 0.02,
                                        dash_len, dash_w, 90.0, ssf,
                                        display_color=(1.0, 1.0, 1.0))
                        dash_i += 1

        else:  # "ew"
            # Horizontal corridor: markings run E-W (yaw=0°)
            road_len = corr["x1"] - corr["x0"]
            mid_y    = (corr["y0"] + corr["y1"]) / 2.0
            n_dashes = max(1, int(road_len / period))
            step_x   = road_len / n_dashes
            xs = [corr["x0"] + (k + 0.5) * step_x for k in range(n_dashes)]

            for x in xs:
                for off in (-cline_off, cline_off):
                    _make_dash_mesh(stage, f"{gnd}/dash_{dash_i}",
                                    x, mid_y + off, 0.02,
                                    dash_len, dash_w, 0.0, ssf,
                                    display_color=(1.0, 1.0, 0.0))
                    dash_i += 1

            for sign in (-1.0, 1.0):
                for div in range(1, n_lanes // 2):
                    wy = mid_y + sign * div * lane_w_m
                    for x in xs:
                        _make_dash_mesh(stage, f"{gnd}/dash_{dash_i}",
                                        x, wy, 0.02,
                                        dash_len, dash_w, 0.0, ssf,
                                        display_color=(1.0, 1.0, 1.0))
                        dash_i += 1

    # 4) Placeholder filler buildings (box meshes in block gaps)
    placeholders = layout.get("placeholder_buildings") or []
    if placeholders:
        ph_scope = parent_path + "/placeholders"
        UsdGeom.Scope.Define(stage, Sdf.Path(ph_scope))
        for i, b in enumerate(placeholders):
            _make_box_mesh(stage, f"{ph_scope}/building_{i}",
                           b["x0"], b["y0"], b["x1"], b["y1"],
                           0.0, b["height"], ssf, display_color=b["color"])

    print(f"[scene_gen] Ground: 1 asphalt base, {len(blocks)} grass blocks, "
          f"{dash_i} lane dashes across {len(road_corridors)} corridors, "
          f"{len(placeholders)} placeholder buildings")


# ---------------------------------------------------------------------------
# City builder
# ---------------------------------------------------------------------------

def build_city(config: dict, resolver: SizeResolver):
    """Compute all placements for the city described by *config*.

    Returns ``(placements, layout)`` where *placements* is a flat list of
    placement dicts::

        {"usd", "x_m", "y_m", "z_m", "yaw_deg",
         "roll_deg", "pitch_deg", "scale", "category"}

    and *layout* = ``{"region": (x0,y0,x1,y1), "blocks": [...],
    "road_corridors": [...], "placeholder_buildings": [...]}`` consumed by
    :func:`apply_ground_planes`.
    Deterministic for a given ``config["seed"]``.
    """
    rng = random.Random(int(config.get("seed", 0)))
    # Damage draws come from their own stream so that raising severity cannot
    # move the pristine city. Sharing one RNG is what made a severity sweep
    # relayout the scene: every damage-conditional draw shifted the sequence,
    # and 2,609 of 4,063 non-damage placements moved between severity 0 and 0.8
    # — 1,212 of them plants, which no building-fate rule can explain.
    # Requirement 4a: a locale and a seed fully specify the layout.
    rng_dmg = random.Random(int(config.get("seed", 0)) + 5501)
    asset_scale = float(config.get("asset_scale", 1.0))
    asset_root = str(config.get("asset_root", "") or "")
    exclusions = config.get("exclusions", [])
    orient = config.get("orientation", {})

    usds = config["usds"]
    tiles = usds.get("tiles", {})

    # Normalize every USD list/dict to plain paths + per-asset scale/axis_up lookups.
    # Entries can be plain strings or dicts:
    #   {usd: "omni://...", scale: 0.01, axis-up: "Y"}
    _scale_overrides: dict = {}
    _axis_up_overrides: dict = {}
    _yaw_offset_overrides: dict = {}
    _tag_overrides: dict = {}

    def _nl(lst):
        paths, sc_ov, au_ov, yo_ov, tag_ov = _normalize_usd_list(
            lst, asset_scale, asset_root)
        _scale_overrides.update(sc_ov)
        _axis_up_overrides.update(au_ov)
        _yaw_offset_overrides.update(yo_ov)
        _tag_overrides.update(tag_ov)
        return paths

    def _pool(paths, tag):
        """Paths carrying *tag*; falls back to the whole list if none do."""
        tagged = [p for p in paths if tag in _tag_overrides.get(p, ())]
        return tagged or paths

    def _pool_not(paths, tag):
        """Paths NOT carrying *tag*; falls back to the whole list if all do."""
        untagged = [p for p in paths if tag not in _tag_overrides.get(p, ())]
        return untagged or paths

    def _sc(path):
        """Per-asset scale, falling back to global asset_scale."""
        return _scale_overrides.get(path, asset_scale)

    def _au(path):
        """Per-asset up-axis ("Y" or "Z"), falling back to "Z"."""
        return _axis_up_overrides.get(path, "Z")

    def _yo(path):
        """Per-asset yaw offset (deg) for art not facing +X, default 0."""
        return _yaw_offset_overrides.get(path, 0.0)

    concrete_usds = _nl(tiles.get("concrete") or [])
    sidewalk_usds = _nl(tiles.get("sidewalk") or tiles.get("brick") or [])
    trail_usds    = _nl(tiles.get("trail") or [])
    # By CONDITION, not by pool name: `building_entries` reads the condition
    # off the entry's tag in the current layout and off the pool key in the
    # legacy one, so this line means the same thing in both.
    house_intact    = _nl(building_entries(config, condition="intact"))
    house_damaged   = _nl(building_entries(config, condition="damaged"))
    house_destroyed = _nl(building_entries(config, condition="destroyed"))
    debris_usds_cfg   = usds.get("debris") or {}
    debris_pile_usds  = _nl(debris_usds_cfg.get("piles") or [])
    debris_piece_usds = _nl(debris_usds_cfg.get("pieces") or [])
    tree_usds     = _nl(usds.get("trees") or [])
    plant_usds    = _nl(usds.get("plants") or [])
    rock_usds     = _nl(usds.get("rocks") or [])
    light_usds    = _nl(usds.get("streetlights") or [])
    bench_usds    = _nl(usds.get("benches") or [])
    car_usds      = _nl(usds.get("cars") or [])
    trash_usds    = _nl(usds.get("trash_cans") or [])
    tlight_usds   = _nl(usds.get("traffic_lights") or [])
    hydrant_usds  = _nl(usds.get("fire_hydrants") or [])
    human_usds    = _nl(usds.get("humans") or [])
    planter_usds  = _nl(usds.get("planters") or [])
    busstop_usds  = _nl(usds.get("bus_stops") or [])
    play_usds     = _nl(usds.get("play_structures") or [])

    # Street vs park furniture pools (tags in the usds entry): "park" marks
    # park-only furniture, "sidewalk" marks street-frontage benches, "tree"
    # marks planters big enough for a street tree, "stump" marks tree stumps
    # (park scatter only — never potted in a sidewalk planter).
    street_light_usds = _pool_not(light_usds, "park")
    park_light_usds   = _pool(light_usds, "park")
    street_bench_usds = _pool(bench_usds, "sidewalk")
    park_bench_usds   = _pool(bench_usds, "park")
    street_trash_usds = _pool_not(trash_usds, "park")
    park_trash_usds   = _pool(trash_usds, "park")
    street_tree_usds  = _pool_not(tree_usds, "stump")
    tree_planter_usds  = _pool(planter_usds, "tree") if planter_usds else []
    plant_planter_usds = _pool_not(planter_usds, "tree") if planter_usds else []

    layout_cfg = config.get("layout", {})
    region_m = layout_cfg.get("region_m", [200.0, 200.0])
    region_w_m, region_h_m = float(region_m[0]), float(region_m[1])
    min_block_m = float(layout_cfg.get("min_block_m", 24.0))
    max_block_m = float(layout_cfg.get("max_block_m", 60.0))
    split_jitter = float(layout_cfg.get("split_jitter", 0.2))
    tile_overlap = float(layout_cfg.get("tile_overlap", 1.02))

    pack_cfg    = _stage(config, "layout").get("packing", {})
    bld_gap_m   = float(pack_cfg.get("building_gap_m", 2.5))
    park_chance = float(pack_cfg.get("park_block_chance", 0.12))
    # Locale knobs: urban paves a block wall-to-wall and builds out to the
    # sidewalk; a suburb leaves the block as lawn and sets houses back from it.
    pave_blocks = bool(pack_cfg.get("pave_blocks", True))
    setback_m   = float(pack_cfg.get("setback_m", 0.0))
    verge_m     = float(_stage(config, "layout").get("frontage", {}).get("verge_m", 0.0))
    drive_cfg   = _stage(config, "detail").get("driveways", {})

    # (block, inset, building_rect) per placed building — the lot context the
    # driveway pass needs to run a drive from the house out to the street.
    block_of_building: list = []
    ph_cfg      = pack_cfg.get("placeholders", {})
    ph_enabled  = bool(ph_cfg.get("enabled", True))
    ph_min_m    = float(ph_cfg.get("min_footprint_m", 5.0))
    ph_max_m    = float(ph_cfg.get("max_footprint_m", 20.0))
    ph_h_range  = ph_cfg.get("height_m", [8.0, 30.0])

    parks_cfg          = _stage(config, "detail").get("parks", {})
    trail_margin_m     = float(parks_cfg.get("trail_margin_m", 4.0))
    trail_waypoints    = parks_cfg.get("trail_waypoints", [2, 4])
    park_bench_sp      = float(parks_cfg.get("bench_spacing_m", 14.0))
    park_light_sp      = float(parks_cfg.get("streetlight_spacing_m", 22.0))
    park_trash_sp      = float(parks_cfg.get("trash_can_spacing_m", 30.0))
    furniture_offset_m = float(parks_cfg.get("furniture_offset_m", 0.8))
    rocks_per_park     = parks_cfg.get("rocks_per_park", [3, 8])
    rock_min_sep_m     = float(parks_cfg.get("rock_min_separation_m", 2.5))
    pg_chance          = float(parks_cfg.get("playground_chance", 0.6))
    pg_count           = parks_cfg.get("playground_structures", [2, 3])
    pg_spacing_m       = float(parks_cfg.get("playground_spacing_m", 5.0))
    pg_margin_m        = float(parks_cfg.get("playground_margin_m", 2.0))

    roads_cfg = _stage(config, "layout").get("roads", {})
    lane_w_m = float(roads_cfg.get("lane_width_m", 3.5))

    # --- Resolve footprints (no grass/road tiles; only houses, concrete, sidewalk) ---
    house_fps = {u: resolver.get(u, "house", scale=_sc(u), axis_up=_au(u))
                 for u in house_intact + house_damaged + house_destroyed}
    concrete_fp = (resolver.get(concrete_usds[0], "concrete", scale=_sc(concrete_usds[0]))
                   if concrete_usds else None)
    sidewalk_fp = (resolver.get(sidewalk_usds[0], "sidewalk", scale=_sc(sidewalk_usds[0]))
                   if sidewalk_usds else None)

    sw_w = max(sidewalk_fp["sx"], sidewalk_fp["sy"]) if sidewalk_fp is not None else 0.0

    # Walking-surface heights, so props sit ON the ground layer they stand on
    # rather than poking out of it from z=0. Sidewalk tiles are placed at
    # z=0.015 and their slab top sits (sz - base) above the pivot; the grass
    # plane is at z=0.01 (see apply_ground_planes).
    sidewalk_top = ((0.015 + sidewalk_fp["sz"] - sidewalk_fp["base"])
                    if sidewalk_fp is not None else 0.0)
    grass_top = 0.01

    placements: list = []

    def add(usd, x, y, z, yaw, category, scale, roll=0.0, pitch=0.0, axis_up="Z",
            stretch=None, pose=None, settle=False):
        # Per-asset yaw-offset is baked in here so every category gets it —
        # callers pass the *intended facing* and the offset corrects for art
        # that isn't authored facing +X. *stretch* is an optional (fx, fy)
        # extra local-XY scale on top of *scale* — used by tiling to stretch
        # each tile to exactly cover its grid step. *pose* names a
        # _HUMAN_POSES entry to bind onto a rigged human. *settle=True* marks
        # props placed at approximated fallen orientations (toppled poles,
        # flipped cars) for a post-placement physics settle
        # (scene_prep.settle_rigid_props) so they rest naturally.
        p = {
            "usd": usd, "x_m": x, "y_m": y, "z_m": z,
            "yaw_deg": yaw + _yo(usd), "roll_deg": roll, "pitch_deg": pitch,
            "scale": scale, "category": category, "axis_up": axis_up,
        }
        if stretch is not None:
            p["stretch"] = stretch
        if pose is not None:
            p["pose"] = pose
        if settle:
            p["settle"] = True
        placements.append(p)

    def _axis_roll(usd):
        """+90° roll stands Y-up-authored assets upright in the Z-up world."""
        return 90.0 if _au(usd) == "Y" else 0.0

    # Real-extent overlap rejection for furniture (street frontage AND park
    # trails). Point grids space placements out, but props have footprints
    # (a 4 m shelter, a 2 m bench) — placement passes reserve their world-XY
    # bbox here and skip the spot when it intersects one already reserved.
    # Deliberate composites (a tree inside its planter, plants in a box)
    # simply don't reserve their contents.
    _occ_cells: dict = {}
    _OCC_CELL = 4.0

    def _occ_reserve(rect, pad=0.1):
        """Reserve XY bbox *rect*; False (nothing reserved) on overlap."""
        r = (rect[0] - pad, rect[1] - pad, rect[2] + pad, rect[3] + pad)
        keys = [(gx, gy)
                for gx in range(int(math.floor(r[0] / _OCC_CELL)),
                                int(math.floor(r[2] / _OCC_CELL)) + 1)
                for gy in range(int(math.floor(r[1] / _OCC_CELL)),
                                int(math.floor(r[3] / _OCC_CELL)) + 1)]
        for k in keys:
            for o in _occ_cells.get(k, ()):
                if not (r[2] <= o[0] or r[0] >= o[2]
                        or r[3] <= o[1] or r[1] >= o[3]):
                    return False
        for k in keys:
            _occ_cells.setdefault(k, []).append(r)
        return True

    def _occ_reserve_square(x, y, fp):
        """Reserve a conservative square (long-axis half — safe under any
        yaw) around (x, y) for *fp*. For props at arbitrary headings, e.g.
        park furniture along a curved trail."""
        half = max(fp["sx"], fp["sy"]) / 2.0
        return _occ_reserve((x - half, y - half, x + half, y + half))

    def _prop_curb_rect(x, y, street_yaw, fp, pole_only=False):
        """World-XY bbox of a prop centered at (x, y) with its long axis
        along the curb. *pole_only* uses the short axis on both sides — for
        streetlights / traffic lights whose bbox is mostly overhanging arm
        (the arm may overlap freely; only the pole occupies the sidewalk)."""
        half_long = max(fp["sx"], fp["sy"]) / 2.0
        half_short = min(fp["sx"], fp["sy"]) / 2.0
        if pole_only:
            half_long = half_short
        if abs(math.cos(math.radians(street_yaw))) < 0.5:
            hx, hy = half_long, half_short     # edge runs E-W
        else:
            hx, hy = half_short, half_long     # edge runs N-S
        return (x - hx, y - hy, x + hx, y + hy)

    def _tile_rect(usd_choices, rect, tsx, tsy, z, category, skip=None):
        """Fill an axis-aligned *rect* (xmin,ymin,xmax,ymax) with tiles sized
        ~(tsx, tsy), snapping the count so the fill is gap-free. Tiles whose
        center falls inside *skip* (a rect) are omitted.

        The center-to-center spacing (``stepx``/``stepy``) is chosen so the
        tile *count* divides the rect evenly, and each tile is then stretched
        per-axis to exactly cover its step — so the fill is gap-free no matter
        how the rect relates to the authored tile size (the texture stretches
        slightly instead). ``tile_overlap`` (>1) additionally inflates each
        tile a hair so neighbors overlap rather than meet at a float-exact
        seam; slight z-fighting at the seams is the accepted tradeoff over
        visible cracks.
        """
        x0, y0, x1, y1 = rect
        w, h = x1 - x0, y1 - y0
        if w <= 1e-6 or h <= 1e-6:
            return
        ntx = max(1, round(w / tsx)) if tsx > 1e-6 else 1
        nty = max(1, round(h / tsy)) if tsy > 1e-6 else 1
        stepx, stepy = w / ntx, h / nty
        stx = stepx / tsx * tile_overlap if tsx > 1e-6 else 1.0
        sty = stepy / tsy * tile_overlap if tsy > 1e-6 else 1.0
        for ix in range(ntx):
            for iy in range(nty):
                px = x0 + (ix + 0.5) * stepx
                py = y0 + (iy + 0.5) * stepy
                if skip is not None and _in_rect(px, py, skip):
                    continue
                u = usd_choices if isinstance(usd_choices, str) else rng.choice(usd_choices)
                add(u, px, py, z, 0.0, category, _sc(u), stretch=(stx, sty))

    # Where the disaster hit. Every disaster.* knob is a maximum, reached
    # only where this field reads 1.0 — see make_damage_field.
    dis_cfg = config.get("disaster", {})
    damage_at = make_damage_field(
        dis_cfg.get("field"),
        (-region_w_m / 2.0, -region_h_m / 2.0,
          region_w_m / 2.0,  region_h_m / 2.0))

    def _hit(x, y, fraction):
        """True with probability *fraction* scaled by the local damage.

        Draws from `rng_dmg`, never `rng` — see the note at the top of this
        function. Every damage decision must leave the layout stream alone.
        """
        return fraction > 0.0 and rng_dmg.random() < fraction * damage_at(x, y)

    def _hit_count(x, y, count_range):
        """Random count from [lo, hi], scaled down by the local damage."""
        k = damage_at(x, y)
        if k <= 0.0:
            return 0
        return int(round(rng_dmg.randint(int(count_range[0]),
                                         int(count_range[1])) * k))

    # Building pools with a nonzero fate weight *somewhere in the region* are
    # the only ones packing may draw from — an "untouched" city (both
    # fractions 0) must never place a ruin even when a slot fits nothing
    # else, and a "total" one (weights summing to 1 at full intensity) must
    # never place an intact building. The field's bounds decide this: a
    # tornado hits 1.0 on its track but 0.0 elsewhere, so all three pools
    # stay eligible even at destroyed_fraction 1.0.
    damaged_fraction   = float(dis_cfg.get("damaged_fraction", 0.0))
    destroyed_fraction = float(dis_cfg.get("destroyed_fraction", 0.0))

    # PACKING IS PRISTINE. Buildings are laid out from the intact pool alone,
    # and fate is assigned afterwards by the disaster stage, which swaps a ruin
    # only when one is sized to stand in the same footprint. Packing used to
    # draw fate-weighted from all three pools, which made the *layout* a
    # function of severity twice over: a ruin has a different footprint than
    # the intact building it replaced, so it packs differently, and the fate
    # roll itself consumed the layout RNG. Both are requirement 4a violations.
    eligible_houses = list(house_intact)

    # Auto-raise the block size bound so the BSP can produce a block whose
    # sidewalk inset fits the largest placeable building.
    h_max = max((max(house_fps[u]["sx"], house_fps[u]["sy"])
                 for u in eligible_houses), default=0.0)
    need_block = h_max + 2.0 * sw_w + 1e-3
    if need_block > max_block_m:
        print(f"[scene_gen] WARNING: largest building needs a {need_block:.1f}m block but "
              f"layout.max_block_m={max_block_m:.1f}m — auto-raising so it can still be placed.")
        max_block_m = need_block

    # Buildings that might not fit an arbitrary (min-sized) block get a
    # dedicated anchor block reserved below. Packing picks randomly among
    # fitting candidates, so without a forced anchor the largest buildings
    # could go entirely unplaced.
    big_buildings = [u for u in eligible_houses
                     if max(house_fps[u]["sx"], house_fps[u]["sy"]) + 2.0 * sw_w
                     > min_block_m]

    # Pool membership by usd — needed both for the anchor pass below (fate
    # must match a big building's forced location) and for the per-slot fate
    # roll during packing.
    damaged_set   = set(house_damaged)
    destroyed_set = set(house_destroyed)

    # =======================================================================
    # ROADS — metric BSP subdivision into blocks + road corridors.
    # Ground geometry (asphalt, grass planes, lane dashes) is written by
    # apply_ground_planes(); build_city only places asset references.
    # =======================================================================
    blocks, road_corridors = _subdivide_region_metric(
        region_w_m, region_h_m, min_block_m, max_block_m,
        split_jitter, rng, roads_cfg)

    city_layout = {
        "region": (-region_w_m / 2.0, -region_h_m / 2.0,
                    region_w_m / 2.0,  region_h_m / 2.0),
        "blocks": blocks,
        "road_corridors": road_corridors,
    }

    def _fit(fp, aw, ah):
        """Yaw (0 or 90) at which the building fits the available lot, else None."""
        if fp["sx"] <= aw and fp["sy"] <= ah:
            return 0.0
        if fp["sy"] <= aw and fp["sx"] <= ah:
            return 90.0
        return None

    def _inset_of(block):
        x0, y0, x1, y1 = block
        return ((x0 + sw_w, y0 + sw_w, x1 - sw_w, y1 - sw_w)
                if (sidewalk_fp is not None and sw_w > 1e-6) else block)

    # Reserve an anchor block per oversized building: largest building first,
    # each taking the tightest-fitting free block, so every library building
    # is guaranteed at least one placement (packing's per-slot fate roll,
    # below, can't be trusted to reach a rarely-fitting big building on its
    # own — random picking among fits could go a whole run without landing on
    # it, or never even offer it if a smaller candidate happens to fill every
    # oversized lot first).
    #
    # Anchoring used to be constrained by the local damage intensity
    # (`_anchor_ok`): an oversized *intact* building could not anchor into a
    # block sitting in a tornado's track, because packing chose the asset —
    # and therefore its fate — before any fate was rolled. That was the
    # explicit layout->damage feedback requirement 4a forbids: raising
    # severity moved buildings.
    #
    # It is no longer needed. Packing draws from the intact pool only, so
    # every anchor candidate has the same fate (none), and the disaster
    # stage decides afterwards — a building anchored in the track gets ruined
    # there, wherever it was anchored.

    anchor_for: dict = {}                            # block -> forced first usd
    unanchored = 0
    for u in sorted(big_buildings,
                    key=lambda u: -(house_fps[u]["sx"] * house_fps[u]["sy"])):
        best = None
        for blk in blocks:
            if blk in anchor_for:
                continue
            ix0, iy0, ix1, iy1 = _inset_of(blk)
            iw, ih = ix1 - ix0, iy1 - iy0
            if iw <= 0 or ih <= 0 or _fit(house_fps[u], iw, ih) is None:
                continue
            if best is None or iw * ih < best[0]:
                best = (iw * ih, blk)
        if best is not None:
            anchor_for[best[1]] = u
        else:
            unanchored += 1
    if unanchored:
        print(f"[scene_gen] WARNING: {unanchored}/{len(big_buildings)} large "
              "buildings have no block that both fits them and matches their "
              "fate to the local damage — increase layout.region_m, raise "
              "layout.max_block_m, or accept that building goes unplaced "
              "this run rather than forcing it somewhere implausible.")

    # =======================================================================
    # BLOCKS -> PACKED BUILDINGS, plus per-block sidewalk
    # (grass and road asphalt are procedural planes from apply_ground_planes)
    # =======================================================================
    house_front_yaw    = float(orient.get("house_front", 0.0))
    # (dis_cfg / fate fractions / eligible_houses were read above, before the
    # anchor pass, so anchors and packing share the same eligibility.)
    # (No debris here. It was read from `disaster.debris` at this point and
    # never used again once the emission moved to `disaster_stage`, and from
    # there into `disaster/debris.py`. `build_city` lays out a PRISTINE city;
    # the event happens to it afterwards, which is the invariant this whole
    # generator is built around.)

    house_rects: list = []      # building footprint rects (for tree avoidance)
    placeholder_bldgs: list = []
    city_layout["placeholder_buildings"] = placeholder_bldgs
    park_blocks: list = []      # (block, inset) rects of blocks kept as parks
    residential_blocks: list = []  # (block, inset) of packed blocks — lawn when
                                   # pave_blocks is off (suburban/rural)
    n_buildings = n_parks = 0

    # Eligible pools pack directly with their own measured footprints; the
    # fate weights (destroyed_fraction / damaged_fraction) drive the *choice*
    # of pool at each packing pick via _pick_building below. Zero-weight
    # pools are excluded from the candidates entirely, so e.g. an untouched
    # city can never fall back onto a ruin (nor "total" onto an intact one).
    pack_candidates = [(u, house_fps[u]["sx"], house_fps[u]["sy"])
                       for u in eligible_houses]

    def _pick_building(fits, sx_, sy_):
        """Pick among fitting candidates. Pristine — fate is not consulted.

        This used to be fate-weighted, which coupled layout to severity. The
        fate roll now happens in `disaster_stage`, after districts.
        """
        return rng.choice(fits)

    def _split_leftover(r, out):
        """Break an oversized leftover rect into a few varied prisms so a
        big gap reads as several adjoining buildings, not one giant slab."""
        rx0, ry0, rx1, ry1 = r
        w, h = rx1 - rx0, ry1 - ry0
        if w <= ph_max_m and h <= ph_max_m:
            out.append(r)
            return
        if w >= h:
            lo, hi = rx0 + ph_min_m, rx1 - ph_min_m - bld_gap_m
            if hi <= lo:
                out.append(r)
                return
            p = rng.uniform(lo, hi)
            _split_leftover((rx0, ry0, p, ry1), out)
            _split_leftover((p + bld_gap_m, ry0, rx1, ry1), out)
        else:
            lo, hi = ry0 + ph_min_m, ry1 - ph_min_m - bld_gap_m
            if hi <= lo:
                out.append(r)
                return
            q = rng.uniform(lo, hi)
            _split_leftover((rx0, ry0, rx1, q), out)
            _split_leftover((rx0, q + bld_gap_m, rx1, ry1), out)

    # Park selection, up front so the count can be guaranteed: per-block
    # chance roll first, then promote random extra blocks until
    # packing.min_parks is met (or demote down to packing.max_parks).
    # Anchored blocks are exempt — they must host their reserved building.
    min_parks = int(pack_cfg.get("min_parks", 0))
    max_parks = pack_cfg.get("max_parks")
    park_candidates = [blk for blk in blocks if blk not in anchor_for]
    park_set = {blk for blk in park_candidates if rng.random() < park_chance}
    if len(park_set) < min_parks:
        pool = [b for b in park_candidates if b not in park_set]
        rng.shuffle(pool)
        park_set.update(pool[:min_parks - len(park_set)])
        if len(park_set) < min_parks:
            print(f"[scene_gen] WARNING: min_parks={min_parks} but only "
                  f"{len(park_set)} unanchored blocks available.")
    if max_parks is not None and len(park_set) > int(max_parks):
        chosen = sorted(park_set)       # deterministic order before shuffle
        rng.shuffle(chosen)
        park_set = set(chosen[:int(max_parks)])

    for blk in blocks:
        x0, y0, x1, y1 = blk

        # Sidewalk frontage ring (brick tiles between road edge and block
        # interior). `verge_m` pushes the walk inward off the kerb; the strip
        # left behind exposes the block's grass plane — the planting strip
        # that reads as residential.
        inset = blk
        if sidewalk_fp is not None and sw_w > 1e-6:
            vx0, vy0 = x0 + verge_m, y0 + verge_m
            vx1, vy1 = x1 - verge_m, y1 - verge_m
            if vx1 - vx0 > 2 * sw_w and vy1 - vy0 > 2 * sw_w:
                _tile_rect(sidewalk_usds, (vx0, vy0, vx1, vy0 + sw_w), sw_w, sw_w, 0.015, "sidewalk")
                _tile_rect(sidewalk_usds, (vx0, vy1 - sw_w, vx1, vy1), sw_w, sw_w, 0.015, "sidewalk")
                _tile_rect(sidewalk_usds, (vx0, vy0 + sw_w, vx0 + sw_w, vy1 - sw_w), sw_w, sw_w, 0.015, "sidewalk")
                _tile_rect(sidewalk_usds, (vx1 - sw_w, vy0 + sw_w, vx1, vy1 - sw_w), sw_w, sw_w, 0.015, "sidewalk")
                inset = (vx0 + sw_w, vy0 + sw_w, vx1 - sw_w, vy1 - sw_w)

        # Park blocks stay grassy — this is where trees, plants, rocks and
        # humans end up. Trails + park furniture are laid in a dedicated pass
        # afterwards. Selection happens up front (park_set) so
        # min_parks/max_parks can be honored.
        if blk in park_set:
            n_parks += 1
            park_blocks.append((blk, inset))
            continue

        # Urban: pave the block interior wall-to-wall, so buildings and
        # filler prisms sit on pavement built out to the sidewalk. Suburb
        # (pave_blocks false): the block stays lawn, and only the ground each
        # building and driveway actually covers is hard surface.
        if concrete_fp is not None and pave_blocks:
            _tile_rect(concrete_usds[0], inset, concrete_fp["sx"], concrete_fp["sy"],
                       0.02, "concrete")

        # Front setback: hold buildings off the sidewalk so the strip between
        # them and the street stays lawn. Skipped if it would leave no room.
        pack_rect = inset
        if setback_m > 1e-6:
            sx0, sy0, sx1, sy1 = inset
            cand = (sx0 + setback_m, sy0 + setback_m,
                    sx1 - setback_m, sy1 - setback_m)
            if cand[2] - cand[0] > ph_min_m and cand[3] - cand[1] > ph_min_m:
                pack_rect = cand

        residential_blocks.append((blk, inset))
        placed, leftovers = _pack_block_with_buildings(
            pack_rect, pack_candidates, bld_gap_m, ph_min_m, rng,
            first=anchor_for.get(blk), pick_fn=_pick_building)
        block_of_building.extend((blk, inset, br) for (_u, _cx, _cy, _r, br) in placed)

        for (usd, cx, cy, rot, brect) in placed:
            # PRISTINE. Fate, tilt/sink and debris all belong to the disaster
            # stage, which runs after `districts` has finished rezoning — see
            # disaster_stage.apply_to_buildings for why the ordering matters.
            fp = house_fps[usd]
            # Y-up assets (e.g. Unity exports) need a +90° roll to stand
            # upright in Isaac Sim's Z-up world.
            add(usd, cx, cy, fp["base"], house_front_yaw + rot, "house",
                _sc(usd), roll=(90.0 if _au(usd) == "Y" else 0.0),
                axis_up=_au(usd))
            # The rubble a ruin drops belongs to the FOOTPRINT it was packed
            # into, not to whichever asset ends up standing on it, so the
            # packed extent travels with the placement for the disaster stage.
            placements[-1]["_footprint_m"] = (brect[2] - brect[0],
                                              brect[3] - brect[1])
            house_rects.append(brect)
            n_buildings += 1

        # Gaps nothing in the library fits: fill with placeholder prisms.
        if ph_enabled:
            pieces: list = []
            for r in leftovers:
                _split_leftover(r, pieces)
            for (rx0, ry0, rx1, ry1) in pieces:
                base_col = rng.choice(_PLACEHOLDER_COLORS)
                col = tuple(min(1.0, max(0.0, c + rng.uniform(-0.05, 0.05)))
                            for c in base_col)
                placeholder_bldgs.append({
                    "x0": rx0, "y0": ry0, "x1": rx1, "y1": ry1,
                    "height": rng.uniform(float(ph_h_range[0]), float(ph_h_range[1])),
                    "color": col,
                })
                house_rects.append((rx0, ry0, rx1, ry1))

    # =======================================================================
    # DRIVEWAYS — a paved strip from each house out to the nearest street,
    # with a car on some of them. Suburban cars belong beside the house, not
    # at the kerb; urban leaves this off (driveways.chance 0).
    # =======================================================================
    drive_chance = float(drive_cfg.get("chance", 0.0))
    if drive_chance > 0.0 and concrete_fp is not None and block_of_building:
        drive_w = float(drive_cfg.get("width_m", 3.0))
        drive_car_chance = float(drive_cfg.get("car_chance", 0.0))
        n_drives = n_drive_cars = 0

        for (blk, inset, brect) in block_of_building:
            if rng.random() >= drive_chance:
                continue
            bx0, by0, bx1, by1 = brect
            ix0, iy0, ix1, iy1 = inset
            bcx, bcy = (bx0 + bx1) / 2.0, (by0 + by1) / 2.0

            # Run the drive out of whichever block edge the house sits nearest.
            gaps = {"S": by0 - iy0, "N": iy1 - by1,
                    "W": bx0 - ix0, "E": ix1 - bx1}
            side = min(gaps, key=gaps.get)
            if gaps[side] <= 0.5:          # already hard against the sidewalk
                continue

            half = drive_w / 2.0
            if side in ("S", "N"):
                # Offset along the house front so the drive runs beside it,
                # not through the front door.
                off = rng.uniform(-1.0, 1.0) * max(0.0, (bx1 - bx0) / 2.0 - half)
                cx_ = min(max(bcx + off, ix0 + half), ix1 - half)
                rect = ((cx_ - half, iy0, cx_ + half, by0) if side == "S"
                        else (cx_ - half, by1, cx_ + half, iy1))
                car_yaw = 90.0
            else:
                off = rng.uniform(-1.0, 1.0) * max(0.0, (by1 - by0) / 2.0 - half)
                cy_ = min(max(bcy + off, iy0 + half), iy1 - half)
                rect = ((ix0, cy_ - half, bx0, cy_ + half) if side == "W"
                        else (bx1, cy_ - half, ix1, cy_ + half))
                car_yaw = 0.0

            if rect[2] - rect[0] < 0.5 or rect[3] - rect[1] < 0.5:
                continue
            _tile_rect(concrete_usds[0], rect, concrete_fp["sx"],
                       concrete_fp["sy"], 0.02, "concrete")
            house_rects.append(rect)       # keep scatter off the drive
            n_drives += 1

            if car_usds and rng.random() < drive_car_chance:
                dcx = (rect[0] + rect[2]) / 2.0
                dcy = (rect[1] + rect[3]) / 2.0
                if not (exclusions and _in_exclusion(dcx, dcy, exclusions)):
                    cu = rng.choice(car_usds)
                    cfp = resolver.get(cu, "car", scale=_sc(cu), axis_up=_au(cu))
                    add(cu, dcx, dcy, cfp["base"],
                        car_yaw + float(orient.get("car_front", 0.0))
                        + rng.uniform(-3.0, 3.0),
                        "car", _sc(cu), roll=_axis_roll(cu), axis_up=_au(cu))
                    n_drive_cars += 1

        if n_drives:
            print(f"[scene_gen] {n_drives} driveways ({n_drive_cars} with a car)")

    # =======================================================================
    # PARKS — meandering trail through each park block, park furniture
    # (benches facing the trail, trash cans, lamps) along its sides.
    # Trail tile rects join house_rects so later scatter stays off the path.
    # =======================================================================
    trail_paths: list = []      # dense (x, y) polylines, one per park trail
    trail_top = 0.0             # z of the trail tiles' walking surface
    if park_blocks and trail_usds:
        tu = trail_usds[0]
        tfp = resolver.get(tu, "trail", scale=_sc(tu))
        t_len, t_w = tfp["sx"], tfp["sy"]     # art X runs along the path
        trail_top = 0.015 + tfp["sz"] - tfp["base"]
        wp_lo, wp_hi = int(trail_waypoints[0]), int(trail_waypoints[1])

        for (pblk, pinset) in park_blocks:
            ix0, iy0, ix1, iy1 = pinset
            m = min(trail_margin_m, (ix1 - ix0) / 4.0, (iy1 - iy0) / 4.0)
            # Meander crossing the park's longer axis, edge to edge.
            horizontal = (ix1 - ix0) >= (iy1 - iy0)
            n_way = rng.randint(wp_lo, wp_hi)
            if horizontal:
                lat = lambda: rng.uniform(iy0 + m, iy1 - m)
                xs = [ix0 + (i + 1) / (n_way + 1) * (ix1 - ix0)
                      + rng.uniform(-m, m) for i in range(n_way)]
                way = ([(ix0, lat())] + [(min(max(x, ix0 + m), ix1 - m), lat())
                                         for x in sorted(xs)] + [(ix1, lat())])
            else:
                lat = lambda: rng.uniform(ix0 + m, ix1 - m)
                ys = [iy0 + (i + 1) / (n_way + 1) * (iy1 - iy0)
                      + rng.uniform(-m, m) for i in range(n_way)]
                way = ([(lat(), iy0)] + [(lat(), min(max(y, iy0 + m), iy1 - m))
                                         for y in sorted(ys)] + [(lat(), iy1)])
            dense = _catmull_rom_points(way, samples_per_seg=10)
            trail_paths.append(dense)

            # Trail tiles: slight step overlap keeps curves gap-free.
            for tx, ty, tyaw in _walk_polyline(dense, t_len * 0.8):
                add(tu, tx, ty, 0.015, tyaw, "trail", _sc(tu) * tile_overlap)
                hl = max(t_len, t_w) / 2.0
                house_rects.append((tx - hl, ty - hl, tx + hl, ty + hl))

            # Furniture along the trail sides. Offsets/phases differ per
            # category so they interleave rather than stack.
            side_d = t_w / 2.0 + furniture_offset_m
            for pool, cat, spacing, phase in (
                    (park_bench_usds, "bench", park_bench_sp, 0.5),
                    (park_light_usds, "streetlight", park_light_sp, 0.25),
                    (park_trash_usds, "trash_can", park_trash_sp, 0.75)):
                if not pool:
                    continue
                for fx, fy, fyaw in _walk_polyline(dense, spacing, phase=phase):
                    side = rng.choice((-1.0, 1.0))
                    rad = math.radians(fyaw)
                    nx_, ny_ = -math.sin(rad) * side, math.cos(rad) * side
                    px_, py_ = fx + nx_ * side_d, fy + ny_ * side_d
                    if not _in_rect(px_, py_, pinset):
                        continue
                    if exclusions and _in_exclusion(px_, py_, exclusions):
                        continue
                    u = rng.choice(pool)
                    fp = resolver.get(u, cat, scale=_sc(u), axis_up=_au(u))
                    if not _occ_reserve_square(px_, py_, fp):
                        continue
                    # Face back toward the trail (benches especially).
                    face_yaw = math.degrees(math.atan2(-ny_, -nx_))
                    add(u, px_, py_, grass_top + fp["base"], face_yaw, cat,
                        _sc(u), roll=_axis_roll(u), axis_up=_au(u))

            # ---- Playground: a cluster of play structures (slide, swings)
            # on a clear patch of grass away from the trail. Structures ring
            # a common center facing inward; their footprints join
            # house_rects so trees/rocks/humans keep clear.
            if play_usds and rng.random() < pg_chance:
                clear_r = pg_spacing_m + 3.0
                spot = None
                if (ix1 - ix0) > 2 * clear_r and (iy1 - iy0) > 2 * clear_r:
                    for _try in range(30):
                        cx_ = rng.uniform(ix0 + clear_r, ix1 - clear_r)
                        cy_ = rng.uniform(iy0 + clear_r, iy1 - clear_r)
                        if exclusions and _in_exclusion(cx_, cy_, exclusions):
                            continue
                        if min((cx_ - dx_) ** 2 + (cy_ - dy_) ** 2
                               for dx_, dy_ in dense) < (clear_r + t_w) ** 2:
                            continue
                        spot = (cx_, cy_)
                        break
                if spot is not None:
                    n_st = rng.randint(int(pg_count[0]), int(pg_count[1]))
                    picks = play_usds[:]
                    rng.shuffle(picks)
                    picks = [picks[i % len(picks)] for i in range(n_st)]
                    base_ang = rng.uniform(0.0, 360.0)
                    for i, su in enumerate(picks):
                        ang = base_ang + i * 360.0 / n_st
                        rad_ = math.radians(ang)
                        r_ = pg_spacing_m if n_st > 1 else 0.0
                        sx_ = spot[0] + math.cos(rad_) * r_
                        sy_ = spot[1] + math.sin(rad_) * r_
                        sfp = resolver.get(su, "play_structure",
                                           scale=_sc(su), axis_up=_au(su))
                        add(su, sx_, sy_, grass_top + sfp["base"],
                            ang + 180.0 + rng.uniform(-15.0, 15.0),
                            "play_structure", _sc(su),
                            roll=_axis_roll(su), axis_up=_au(su))
                        hl_ = max(sfp["sx"], sfp["sy"]) / 2.0 + pg_margin_m
                        house_rects.append((sx_ - hl_, sy_ - hl_,
                                            sx_ + hl_, sy_ + hl_))

    # =======================================================================
    # STEP 3 — DETAIL: trees + humans (open-ground scatter); rocks (parks);
    # streetlights + fire hydrants + trash cans (block frontage); traffic
    # lights (intersection corners)
    # =======================================================================

    # Disaster aftermath parameters — consumed inline through steps 3-4
    # (per-category toppling, leaning, scattering, strewn wrecks). Every
    # fraction below is a maximum, scaled per-position by damage_at() via
    # _hit() / _hit_count(), so the aftermath follows the disaster's field.
    dcfg = dis_cfg

    def _scatter_in_blocks(count_range, min_sep, margin, place_fn, in_blocks=None,
                           density_per_100m2=None, even=False):
        """Rejection-sample points on each block's open ground — off building
        lots by *margin*, off exclusions, *min_sep* apart — and call
        ``place_fn(x, y)`` for each accepted point. *in_blocks* restricts the
        pass to a subset of blocks (default: all).

        *density_per_100m2*, if given, sizes each block's count from its area
        instead of *count_range*. *even=True* switches to best-candidate
        sampling — each point is the candidate farthest from those already
        placed — for an even, blue-noise-like spread across the block.
        """
        for (x0, y0, x1, y1) in (blocks if in_blocks is None else in_blocks):
            local = [hr for hr in house_rects
                     if hr[0] >= x0 - 5 and hr[2] <= x1 + 5
                     and hr[1] >= y0 - 5 and hr[3] <= y1 + 5]
            placed: list = []
            if density_per_100m2 is not None:
                want = max(1, round((x1 - x0) * (y1 - y0) / 100.0
                                    * float(density_per_100m2)))
            else:
                want = rng.randint(int(count_range[0]), int(count_range[1]))

            def _ok(tx, ty):
                if any(_in_rect(tx, ty, (hr[0] - margin, hr[1] - margin,
                                         hr[2] + margin, hr[3] + margin))
                       for hr in local):
                    return False
                if exclusions and _in_exclusion(tx, ty, exclusions):
                    return False
                return not any((tx - px) ** 2 + (ty - py) ** 2 < min_sep ** 2
                               for px, py in placed)

            if even:
                for _ in range(want):
                    best = None
                    for _c in range(10):
                        tx, ty = rng.uniform(x0, x1), rng.uniform(y0, y1)
                        if not _ok(tx, ty):
                            continue
                        d = min(((tx - px) ** 2 + (ty - py) ** 2
                                 for px, py in placed), default=float("inf"))
                        if best is None or d > best[0]:
                            best = (d, tx, ty)
                    if best is not None:
                        place_fn(best[1], best[2])
                        placed.append((best[1], best[2]))
            else:
                for _ in range(want * 12):
                    if len(placed) >= want:
                        break
                    tx, ty = rng.uniform(x0, x1), rng.uniform(y0, y1)
                    if not _ok(tx, ty):
                        continue
                    place_fn(tx, ty)
                    placed.append((tx, ty))

    # Frontage props (streetlights, fire hydrants, trash cans) share one 2 m
    # occupancy grid so different categories never stack on the same spot.
    frontage_seen: set = set()

    def _frontage_positions(spacing, phase=0.0, jitter_frac=0.0):
        """Yield (x, y, street_yaw) along every block's frontage ring at
        *spacing*, skipping already-occupied slots and exclusion zones.
        *street_yaw* is the outward heading (deg) from the block toward the
        adjacent road — pass it as yaw to make a prop face its street.

        *phase* shifts every point along its edge by that fraction of a step
        (phase 0 includes both corners; phase 0.5 emits step midpoints only).
        Categories placed after the first would otherwise land exactly on the
        corners/steps the first category already claimed in the occupancy
        grid and get dropped wholesale. *jitter_frac* adds a random per-point
        slide (fraction of a step) so sparse categories look hand-placed.

        A *spacing* of 0 (or less) means "this category doesn't belong here"
        and yields nothing — how a locale switches a whole street-furniture
        category off (a suburb has no public bins or bus shelters).
        """
        if spacing <= 0.0:
            return
        for (x0, y0, x1, y1) in blocks:
            # (edge endpoints, outward yaw): south -90, north +90,
            # west 180, east 0 — art convention: front faces +X at yaw 0.
            for ax, ay, bx, by, out_yaw in (
                    (x0, y0, x1, y0, -90.0), (x0, y1, x1, y1, 90.0),
                    (x0, y0, x0, y1, 180.0), (x1, y0, x1, y1, 0.0)):
                length = math.hypot(bx - ax, by - ay)
                if spacing > length:
                    # Sparser than one per edge: emit a single point with
                    # probability length/spacing so the average density still
                    # matches the requested spacing (instead of every edge
                    # getting one regardless).
                    if rng.random() >= length / spacing:
                        continue
                    n = 1
                else:
                    n = int(length / spacing)
                step = length / n
                for k in range(n + 1 if phase == 0.0 else n):
                    s = (k + phase) * step
                    if jitter_frac:
                        s = min(length, max(0.0, s + rng.uniform(-1.0, 1.0)
                                            * jitter_frac * step))
                    t = s / length
                    px, py = ax + (bx - ax) * t, ay + (by - ay) * t
                    key = (round(px / 2.0), round(py / 2.0))
                    if key in frontage_seen:
                        continue
                    frontage_seen.add(key)
                    if exclusions and _in_exclusion(px, py, exclusions):
                        continue
                    yield px, py, out_yaw

    # Frontage points sit exactly ON the curb line, so a prop centered there
    # protrudes halfway into the road. Every street-furniture pass steps its
    # prop inward (opposite the outward street yaw) by half the prop's short
    # footprint axis plus this margin, so meshes sit fully on the sidewalk
    # while staying close to the curb.
    curb_margin = float(_stage(config, "layout").get("frontage", {}).get("curb_margin_m", 0.3))

    def _inset_pos(px, py, street_yaw, dist):
        """Step (px, py) *dist* meters from the curb onto the sidewalk."""
        rad = math.radians(street_yaw)
        return px - math.cos(rad) * dist, py - math.sin(rad) * dist

    def _curb_inset(fp):
        """Inset distance putting *fp*'s short axis fully on the sidewalk.
        The short axis is the prop's depth toward the road (the long axis
        runs along the curb — bench seats, shelter fronts, lamp arms)."""
        return min(fp["sx"], fp["sy"]) / 2.0 + curb_margin

    def _topple_flat(usd, fp, x, y, category, z0=0.0, pose=None):
        """Prop knocked flat: ~90° roll about its base pivot with a random
        heading; z lifts the pivot by half the footprint width so the lying
        prop roughly rests on the ground surface at height *z0* — marked for
        the physics settle pass to find its true resting orientation."""
        add(usd, x, y, z0 + max(fp["sx"], fp["sy"]) / 2.0,
            rng_dmg.uniform(0.0, 360.0),
            category, _sc(usd),
            roll=_axis_roll(usd)
                 + rng_dmg.choice([-1.0, 1.0]) * rng_dmg.uniform(80.0, 100.0),
            axis_up=_au(usd), pose=pose, settle=True)

    park_block_set = {b for (b, _i) in park_blocks}
    park_insets = [inset for (_b, inset) in park_blocks]

    # Residential lawns: the un-paved part of a packed block. Urban paves
    # its blocks (pave_blocks), so there is no lawn and these stay empty;
    # a suburb grows trees and shrubs here, which is what makes it read as
    # a suburb rather than an urban scene of small buildings.
    lawn_insets = ([inset for (_b, inset) in residential_blocks]
                   if not pave_blocks else [])

    # ---- Trees: parks get bare-ground trees, densely and evenly spread;
    # suburban lawns get them at their own density. Urban street trees are
    # potted in tree-tagged planters along the frontage instead — see the
    # planter passes in the street-furniture section below.
    if tree_usds and (park_insets or lawn_insets):
        tcfg = _stage(config, "detail").get("trees", {})
        # Wind takes trees down with the buildings — in tornado reference
        # imagery the treeline is the clearest marker of the track, snapped
        # bare inside it and untouched a few metres outside. Stumps are already
        # down, so they're excluded from the pool that can topple.
        trees_toppled = float(dcfg.get("trees_toppled_fraction", 0.0))
        stump_usds = set(_pool(tree_usds, "stump")) if tree_usds else set()

        def _place_tree(tx, ty):
            usd = rng.choice(tree_usds)
            jitter = rng.uniform(0.85, 1.2)
            # Drawn before the damage branch, and unconditionally: taking the
            # toppled path used to skip this draw, so one felled tree shifted
            # every subsequent layout draw in the scene. That single early
            # `return` is most of why a severity sweep relaid 1,211 plants.
            # The layout stream must consume the same draws either way.
            yaw = rng.uniform(0, 360)
            tfp = resolver.get(usd, "tree", scale=_sc(usd))
            if usd not in stump_usds and _hit(tx, ty, trees_toppled):
                _topple_flat(usd, tfp, tx, ty, "tree", z0=grass_top)
                return
            add(usd, tx, ty, grass_top + tfp["base"],
                yaw, "tree", _sc(usd) * jitter)

        if park_insets:
            _scatter_in_blocks(None,
                               float(parks_cfg.get("tree_min_separation_m", 4.0)),
                               float(tcfg.get("house_margin_m", 1.5)),
                               _place_tree,
                               in_blocks=park_insets,
                               density_per_100m2=float(
                                   parks_cfg.get("tree_density_per_100m2", 0.8)),
                               even=True)

        lawn_tree_density = float(tcfg.get("lawn_density_per_100m2", 0.0))
        if lawn_insets and lawn_tree_density > 0.0:
            _scatter_in_blocks(None,
                               float(tcfg.get("lawn_min_separation_m", 6.0)),
                               float(tcfg.get("house_margin_m", 1.5)),
                               _place_tree,
                               in_blocks=lawn_insets,
                               density_per_100m2=lawn_tree_density,
                               even=True)

    # ---- Plants (grass clumps, flowers, bushes): park grass, and suburban
    # lawns/foundation planting.
    if plant_usds and (park_insets or lawn_insets):
        pcfg = _stage(config, "detail").get("plants", {})

        def _place_plant(tx, ty):
            usd = rng.choice(plant_usds)
            jitter = rng.uniform(0.8, 1.15)
            add(usd, tx, ty,
                grass_top + resolver.get(usd, "plant", scale=_sc(usd))["base"],
                rng.uniform(0, 360), "plant", _sc(usd) * jitter,
                roll=_axis_roll(usd), axis_up=_au(usd))

        if park_insets:
            _scatter_in_blocks(pcfg.get("per_block", [4, 12]),
                               float(pcfg.get("min_separation_m", 1.0)),
                               float(pcfg.get("house_margin_m", 1.5)),
                               _place_plant,
                               in_blocks=park_insets)

        lawn_plant_density = float(pcfg.get("lawn_density_per_100m2", 0.0))
        if lawn_insets and lawn_plant_density > 0.0:
            _scatter_in_blocks(None,
                               float(pcfg.get("min_separation_m", 1.0)),
                               float(pcfg.get("lawn_house_margin_m", 0.6)),
                               _place_plant,
                               in_blocks=lawn_insets,
                               density_per_100m2=lawn_plant_density)

    # ---- Rocks: scattered on park grass only (off the trail — trail tiles
    # are in house_rects).
    if rock_usds and park_insets:

        def _place_rock(tx, ty):
            usd = rng.choice(rock_usds)
            jitter = rng.uniform(0.7, 1.3)
            add(usd, tx, ty,
                grass_top + resolver.get(usd, "rock", scale=_sc(usd))["base"],
                rng.uniform(0, 360), "rock", _sc(usd) * jitter,
                roll=_axis_roll(usd), axis_up=_au(usd))

        _scatter_in_blocks(rocks_per_park, rock_min_sep_m, 0.5, _place_rock,
                           in_blocks=park_insets)

    # ---- The built-in frontage passes lived here: bus stops, streetlights,
    # benches, planters, fire hydrants, trash cans and traffic lights, each
    # walking a block's frontage ring and placing one category on a single
    # line at the kerb.
    #
    # They are gone. `detail/city_detail.py` places all of it now, against the
    # NACTO sidewalk section, for every locale — urban since Krrish's work,
    # suburban since locales/suburban.yaml. Keeping both meant a category
    # placed twice if a spacing was ever un-zeroed, and the built-in pass had a
    # structural flaw the zoned one does not: every category shared one 2 m
    # occupancy grid, so whichever ran last found the kerb full and was
    # silently dropped. That is what the `phase` argument existed to work
    # around.
    #
    # `_frontage_positions` survives — the human pass still walks the frontage.

    # ---- Humans. Pedestrians walk the sidewalks (heading along the street),
    # walkers follow the park trails, idlers stand on open grass. All are
    # posed via their UsdSkel rigging (walk / idle — see _HUMAN_POSES) so
    # they don't render in their authored T-pose.
    # Disaster: a prone fraction lies flat where they stood (face-down/up,
    # posed arms-down first so they read as casualties, not crosses), and
    # humans_strewn extra casualties are thrown onto the road asphalt.
    if human_usds:
        hcfg = _stage(config, "detail").get("humans", {})
        prone = float(dcfg.get("humans_prone_fraction", 0.0))

        def _human_down(u, fp, x, y, z0):
            """Casualty flat on the ground: face-down/up (±90° pitch about
            the body's facing axis via roll on the pre-yaw frame), lifted by
            half the body depth (sy — arms-down pose, not the T-pose span)."""
            add(u, x, y, z0 + fp["sy"] / 2.0, rng_dmg.uniform(0.0, 360.0),
                "human", _sc(u),
                roll=_axis_roll(u) + rng_dmg.choice([-1.0, 1.0]) * rng_dmg.uniform(82.0, 98.0),
                axis_up=_au(u), pose="idle")

        def _add_human(tx, ty, z0, yaw, pose):
            u = rng.choice(human_usds)
            fp = resolver.get(u, "human", scale=_sc(u), axis_up=_au(u))
            if _hit(tx, ty, prone):
                _human_down(u, fp, tx, ty, z0)
            else:
                add(u, tx, ty, z0 + fp["base"], yaw, "human", _sc(u),
                    roll=_axis_roll(u), axis_up=_au(u), pose=pose)

        # Sidewalk pedestrians: half a sidewalk in from the curb line,
        # heading along the street (both directions).
        sw_spacing = float(hcfg.get("sidewalk_spacing_m", 0.0))
        if sw_spacing > 0.0:
            inset_d = max(sw_w / 2.0, 0.4)
            for px, py, street_yaw in _frontage_positions(sw_spacing, phase=0.4,
                                                          jitter_frac=0.4):
                rad = math.radians(street_yaw)
                hx = px - math.cos(rad) * inset_d
                hy = py - math.sin(rad) * inset_d
                # Don't spawn a pedestrian intersecting street furniture.
                if not _occ_reserve((hx - 0.25, hy - 0.25,
                                     hx + 0.25, hy + 0.25)):
                    continue
                _add_human(hx, hy, sidewalk_top,
                           street_yaw + rng.choice((90.0, -90.0)), "walk")

        # Park trail walkers: on the trail itself, heading along it.
        tr_spacing = float(hcfg.get("trail_spacing_m", 0.0))
        if tr_spacing > 0.0:
            for dense in trail_paths:
                for hx, hy, hyaw in _walk_polyline(dense, tr_spacing, phase=0.6):
                    if exclusions and _in_exclusion(hx, hy, exclusions):
                        continue
                    _add_human(hx, hy, trail_top,
                               hyaw + rng.choice((0.0, 180.0)), "walk")

        # Idlers on open grass (parks mostly — packed blocks have little
        # open ground), random headings.
        _scatter_in_blocks(hcfg.get("per_block", [0, 2]),
                           float(hcfg.get("min_separation_m", 2.0)),
                           float(hcfg.get("house_margin_m", 1.0)),
                           lambda tx, ty: _add_human(
                               tx, ty, grass_top, rng.uniform(0.0, 360.0), "idle"))

        # Disaster: extra casualties strewn on the road asphalt.
        strewn_range = dcfg.get("humans_strewn", [0, 0])
        want = rng_dmg.randint(int(strewn_range[0]), int(strewn_range[1]))
        n_strewn = 0
        for _ in range(want * 10):
            if n_strewn >= want or not road_corridors:
                break
            corr = rng_dmg.choice(road_corridors)
            hx = rng_dmg.uniform(corr["x0"], corr["x1"])
            hy = rng_dmg.uniform(corr["y0"], corr["y1"])
            if exclusions and _in_exclusion(hx, hy, exclusions):
                continue
            # Keep casualties inside the damage zone: accept the sampled
            # spot in proportion to how hard it was hit.
            if rng_dmg.random() >= damage_at(hx, hy):
                continue
            u = rng_dmg.choice(human_usds)
            fp = resolver.get(u, "human", scale=_sc(u), axis_up=_au(u))
            _human_down(u, fp, hx, hy, 0.0)
            n_strewn += 1

    # =======================================================================
    # STEP 4 — CARS (right-hand traffic along road corridors) + STREWN WRECKS
    # =======================================================================
    if car_usds:
        ccfg = _stage(config, "detail").get("cars", {})
        car_density  = float(ccfg.get("density", 0.0))
        lane_frac    = float(ccfg.get("lane_offset_frac", 0.3))
        car_yaw_jit  = float(ccfg.get("yaw_jitter_deg", 4.0))
        car_front    = float(orient.get("car_front", 0.0))
        toppled_frac = float(dcfg.get("cars_toppled_fraction", 0.0))

        def _car_fp(usd):
            return resolver.get(usd, "car", scale=_sc(usd), axis_up=_au(usd))

        def _add_toppled_car(usd, fp, x, y):
            """Car flipped on its side (~±90° roll) or roof (~180°), random
            heading. z lifts the base pivot so the flipped body rests on the
            ground: on-side height = body width (sy), on-roof height = sz.
            """
            # rng_dmg throughout: flipping a car is damage. These six draws on
            # the layout stream were the last severity leak — a flipped car
            # shifted every prop placed after it.
            if rng_dmg.random() < 0.5:
                roll, z = (rng_dmg.choice([-1.0, 1.0])
                           * rng_dmg.uniform(80.0, 100.0), fp["sy"] / 2.0)
            else:
                roll, z = 180.0 + rng_dmg.uniform(-12.0, 12.0), fp["sz"]
            add(usd, x, y, z, rng_dmg.uniform(0.0, 360.0), "car", _sc(usd),
                roll=_axis_roll(usd) + roll, pitch=rng_dmg.uniform(-8.0, 8.0),
                axis_up=_au(usd), settle=True)

        # ---- Lane cars: distributed along road corridors, right-hand traffic.
        for corr in road_corridors:
            n_lanes  = corr["n_lanes"]
            road_w   = n_lanes * lane_w_m
            # Offset from corridor centreline to right-hand lane centre.
            # lane_frac 0=centre, 1=road edge; typical lane centre = 0.5*lane_w.
            lane_off = lane_frac * road_w / 2.0

            if corr["dir"] == "ns":
                road_len = corr["y1"] - corr["y0"]
                mid_x    = (corr["x0"] + corr["x1"]) / 2.0
                n_spots  = max(1, int(road_len / 8.0))
                for _ in range(n_spots):
                    if rng.random() >= car_density:
                        continue
                    heading = rng.choice((1, -1))   # +1=northbound, -1=southbound
                    cx = mid_x - heading * lane_off
                    cy = rng.uniform(corr["y0"], corr["y1"])
                    if exclusions and _in_exclusion(cx, cy, exclusions):
                        continue
                    yaw = 90.0 if heading > 0 else 270.0
                    usd = rng.choice(car_usds)
                    fp  = _car_fp(usd)
                    # Drawn unconditionally: the flipped branch does not use
                    # it, and skipping it shifted every later placement.
                    yaw_jit = rng.uniform(-car_yaw_jit, car_yaw_jit)
                    if _hit(cx, cy, toppled_frac):
                        _add_toppled_car(usd, fp, cx, cy)
                    else:
                        add(usd, cx, cy, fp["base"], yaw + car_front + yaw_jit,
                            "car", _sc(usd), roll=_axis_roll(usd), axis_up=_au(usd))
            else:  # "ew"
                road_len = corr["x1"] - corr["x0"]
                mid_y    = (corr["y0"] + corr["y1"]) / 2.0
                n_spots  = max(1, int(road_len / 8.0))
                for _ in range(n_spots):
                    if rng.random() >= car_density:
                        continue
                    heading = rng.choice((1, -1))   # +1=eastbound, -1=westbound
                    cx = rng.uniform(corr["x0"], corr["x1"])
                    cy = mid_y - heading * lane_off
                    if exclusions and _in_exclusion(cx, cy, exclusions):
                        continue
                    yaw = 0.0 if heading > 0 else 180.0
                    usd = rng.choice(car_usds)
                    fp  = _car_fp(usd)
                    # Drawn unconditionally: the flipped branch does not use
                    # it, and skipping it shifted every later placement.
                    yaw_jit = rng.uniform(-car_yaw_jit, car_yaw_jit)
                    if _hit(cx, cy, toppled_frac):
                        _add_toppled_car(usd, fp, cx, cy)
                    else:
                        add(usd, cx, cy, fp["base"], yaw + car_front + yaw_jit,
                            "car", _sc(usd), roll=_axis_roll(usd), axis_up=_au(usd))

        # ---- Strewn wrecks: tornado-tossed cars anywhere in the region.
        strewn_range = dcfg.get("cars_strewn", [0, 0])
        strewn_topple = float(dcfg.get("strewn_topple_fraction", 0.7))
        strewn_margin = float(dcfg.get("strewn_margin_m", 1.5))
        want = rng_dmg.randint(int(strewn_range[0]), int(strewn_range[1]))
        half_w, half_h = region_w_m / 2.0, region_h_m / 2.0
        strewn_pts: list = []
        for _ in range(want * 15):
            if len(strewn_pts) >= want:
                break
            x, y = rng_dmg.uniform(-half_w, half_w), rng_dmg.uniform(-half_h, half_h)
            if exclusions and _in_exclusion(x, y, exclusions):
                continue
            if any(_in_rect(x, y, (hr[0] - strewn_margin, hr[1] - strewn_margin,
                                   hr[2] + strewn_margin, hr[3] + strewn_margin))
                   for hr in house_rects):
                continue
            if any((x - px) ** 2 + (y - py) ** 2 < 5.0 ** 2 for px, py in strewn_pts):
                continue                              # don't stack wrecks
            # Wrecks land where the disaster hit: accept the sampled spot in
            # proportion to the local intensity (a tornado strews along its
            # track, not across the whole city).
            if rng_dmg.random() >= damage_at(x, y):
                continue
            usd = rng_dmg.choice(car_usds)
            fp = _car_fp(usd)
            if rng_dmg.random() < strewn_topple:
                _add_toppled_car(usd, fp, x, y)
            else:
                # Upright but tossed onto uneven ground — settle it too.
                add(usd, x, y, fp["base"], rng_dmg.uniform(0.0, 360.0), "car",
                    _sc(usd), roll=_axis_roll(usd), axis_up=_au(usd),
                    settle=True)
            strewn_pts.append((x, y))

    # --- Path scour ---------------------------------------------------------
    # MOVED OUT, to `disaster/disaster_stage.py:apply_path_scour`.
    #
    # It emitted debris across the whole region from here — inside the LAYOUT
    # stage — and `districts` then re-packs every block treating debris as an
    # immovable obstacle. So the amount of scour, which is a function of
    # severity, decided how many buildings the city ended up with: 919 at
    # severity 0 against 869 at 0.6 for the same seed, with houses moved all
    # over the plan. That is the exact coupling the three-stage split exists to
    # forbid, and it is why `test_structure_invariant_to_severity` failed for
    # every preset that has `debris.path_*` set and passed for the two that
    # do not.
    #
    # Scour is disaster output, so it belongs in the disaster stage, which runs
    # last and cannot perturb anything.

    # --- Summary -----------------------------------------------------------
    counts: dict = {}
    for p in placements:
        counts[p["category"]] = counts.get(p["category"], 0) + 1
    print(f"[scene_gen] City region {region_w_m:.0f}×{region_h_m:.0f} m "
          f"({len(blocks)} blocks, {n_parks} parks, {len(road_corridors)} road corridors)")
    print(f"[scene_gen] Placements by category: {counts} "
          f"(total {len(placements)}, {n_buildings} buildings, "
          f"{len(placeholder_bldgs)} placeholder prisms)")
    return placements, city_layout


# ---------------------------------------------------------------------------
# Human posing (UsdSkel)
# ---------------------------------------------------------------------------
# The RenderPeople "*_rigged_*_ue4" humans are SkelRoots with a UE4-mannequin-
# compatible Skeleton (pelvis / spine_01… / upperarm_l / thigh_r…, 95 joints)
# carrying rest + bind transforms but *no* animation clip — composed as-is
# they stand in a T-pose. Poses are authored procedurally: a static
# UsdSkel.Animation is computed from the character's own rest transforms with
# world-space rotation deltas applied to a few joints, then bound to the
# instance's Skeleton via skel:animationSource. Joint names are shared across
# the whole pack, so one pose spec works for every character.
#
# Spec format: {joint_basename: (world_axis_xyz, degrees)} — the delta rotates
# the joint's rest frame about its own origin in world space; descendants
# follow. Axis/sign constants were verified geometrically (joint world
# positions after posing): the characters are authored facing -Y (hence the
# yaw-offset: 90 in the config) in an A-pose — arms ~45° down along the X
# axis — so lowering an arm to the side takes ~+45° more about world Y.

_HUMAN_POSES = {
    # Natural stand: arms lowered to the sides, slight elbow relaxation.
    "idle": {
        "upperarm_l": ((0.0, 1.0, 0.0), 45.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -45.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 8.0),
        "lowerarm_r": ((0.0, 1.0, 0.0), -8.0),
    },
    # Mid-stride walk: arms down (opposite swing), legs scissored.
    # Facing is -Y, so "forward" deltas move joints toward -Y.
    "walk": {
        "upperarm_l": ((0.0, 1.0, 0.0), 42.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -42.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 6.0),
        "lowerarm_r": ((0.0, 1.0, 0.0), -6.0),
        "upperarm_l+": ((1.0, 0.0, 0.0), 12.0),   # swing: left arm back
        "upperarm_r+": ((1.0, 0.0, 0.0), -12.0),  # right arm forward
        "thigh_l": ((1.0, 0.0, 0.0), -20.0),      # left leg forward
        "thigh_r": ((1.0, 0.0, 0.0), 14.0),       # right leg back
        "calf_r": ((1.0, 0.0, 0.0), 18.0),        # trailing knee bent
    },

    # ---- Stage C victim poses (see targets.py) ----------------------------
    # Same convention as the two above, extended rather than re-derived: +Y on
    # a LEFT joint lowers that arm (so -Y raises it), -X on a thigh swings the
    # leg forward (the characters face -Y). The three LYING poses are authored
    # UPRIGHT here and laid flat by the placer's roll — `targets.to_placements`
    # picks the roll sign from the pose name, so `prone` is face-down and
    # `supine` face-up rather than a coin flip.
    #
    # Poses are drawn from what aerial SAR datasets actually contain (SARD:
    # standing, sitting, lying, and "exhausted or injured" positions), because
    # the point of them is to be recognisable from above.

    # On the back, limbs relaxed and slightly splayed — the posture a casualty
    # is found in, not the stiff cross the T-pose gives.
    "supine": {
        "upperarm_l": ((0.0, 1.0, 0.0), 30.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -30.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 12.0),
        "lowerarm_r": ((0.0, 1.0, 0.0), -12.0),
        "thigh_l": ((0.0, 1.0, 0.0), 8.0),
        "thigh_r": ((0.0, 1.0, 0.0), -8.0),
        "calf_l": ((1.0, 0.0, 0.0), 10.0),
    },
    # Face-down, one arm thrown up past the head: the "fell while running"
    # silhouette, and the one that reads least like a mannequin from the air.
    "prone": {
        "upperarm_l": ((0.0, 1.0, 0.0), -70.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), -20.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -35.0),
        "lowerarm_r": ((0.0, 1.0, 0.0), -15.0),
        "thigh_r": ((1.0, 0.0, 0.0), -12.0),
        "calf_r": ((1.0, 0.0, 0.0), 25.0),
    },
    # Curled on one side. The void-space posture: knees to chest, arms tucked,
    # spine rounded — small enough to fit the gap that kept them alive.
    "fetal": {
        "spine_02": ((1.0, 0.0, 0.0), -18.0),
        "thigh_l": ((1.0, 0.0, 0.0), -75.0),
        "thigh_r": ((1.0, 0.0, 0.0), -70.0),
        "calf_l": ((1.0, 0.0, 0.0), 85.0),
        "calf_r": ((1.0, 0.0, 0.0), 80.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 55.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -55.0),
        "lowerarm_l": ((1.0, 0.0, 0.0), -70.0),
        "lowerarm_r": ((1.0, 0.0, 0.0), -70.0),
    },
    # Drop, cover, hold on — crouched under whatever was overhead. Upright, so
    # it stands on its feet; used for the trapped cohort in a standing void.
    "crouch": {
        "spine_02": ((1.0, 0.0, 0.0), -28.0),
        "thigh_l": ((1.0, 0.0, 0.0), -80.0),
        "thigh_r": ((1.0, 0.0, 0.0), -80.0),
        "calf_l": ((1.0, 0.0, 0.0), 95.0),
        "calf_r": ((1.0, 0.0, 0.0), 95.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 25.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -25.0),
        "lowerarm_l": ((1.0, 0.0, 0.0), -80.0),   # arms up over the head
        "lowerarm_r": ((1.0, 0.0, 0.0), -80.0),
    },
    # Sitting on the ground, legs out front — the walking wounded, waiting.
    "seated": {
        "spine_02": ((1.0, 0.0, 0.0), 10.0),
        "thigh_l": ((1.0, 0.0, 0.0), -88.0),
        "thigh_r": ((1.0, 0.0, 0.0), -88.0),
        "calf_l": ((1.0, 0.0, 0.0), 12.0),
        "calf_r": ((1.0, 0.0, 0.0), 12.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
    },
    # Kneeling beside someone — what a bystander at a collapse is doing.
    "kneeling": {
        "spine_02": ((1.0, 0.0, 0.0), -20.0),
        "thigh_l": ((1.0, 0.0, 0.0), -25.0),
        "thigh_r": ((1.0, 0.0, 0.0), -20.0),
        "calf_l": ((1.0, 0.0, 0.0), 105.0),
        "calf_r": ((1.0, 0.0, 0.0), 100.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 35.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -35.0),
        "lowerarm_l": ((1.0, 0.0, 0.0), -45.0),
        "lowerarm_r": ((1.0, 0.0, 0.0), -45.0),
    },
    # Standing, left arm straight overhead: the signal-to-aircraft pose, and
    # the one an aerial searcher has the best chance of catching.
    "wave_up": {
        "upperarm_l": ((0.0, 1.0, 0.0), -135.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), -15.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -45.0),
        "lowerarm_r": ((0.0, 1.0, 0.0), -8.0),
    },
    # Down but signalling: one arm reaching out past the head, the other at
    # the side. Laid flat by the placer like the other lying poses.
    "wave_down": {
        "upperarm_l": ((0.0, 1.0, 0.0), -100.0),
        "lowerarm_l": ((1.0, 0.0, 0.0), -30.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -30.0),
        "lowerarm_r": ((0.0, 1.0, 0.0), -10.0),
        "thigh_l": ((0.0, 1.0, 0.0), 10.0),
    },
}


def _pose_joint_transforms(joints, rest, deltas):
    """Compute posed joint transforms from a skeleton's rest pose.

    *joints* is the UsdSkel joint-path list (parent-first order), *rest* the
    matching local rest transforms (Gf.Matrix4d), *deltas* a pose spec dict
    (see ``_HUMAN_POSES``; a trailing ``+`` on the joint name stacks a second
    delta on the same joint). Returns ``(new_local, new_world)`` matrix lists.
    """
    idx = {str(p): i for i, p in enumerate(joints)}
    n = len(joints)
    new_local = [Gf.Matrix4d(m) for m in rest]
    new_world: list = [None] * n
    by_joint: dict = {}
    for name, (axis, deg) in deltas.items():
        by_joint.setdefault(name.rstrip("+"), []).append(
            Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(Gf.Vec3d(*axis), float(deg))))
    for i in range(n):
        path = str(joints[i])
        pi = idx.get(path.rsplit("/", 1)[0], -1) if "/" in path else -1
        pw = new_world[pi] if pi >= 0 else Gf.Matrix4d(1.0)
        w = new_local[i] * pw
        for rot in by_joint.get(path.rsplit("/", 1)[-1], []):
            t = w.ExtractTranslation()
            # Rotate the frame about its own origin by a world-space delta.
            w = (w * Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(t) * -1.0)
                 * rot * Gf.Matrix4d(1.0).SetTranslate(t))
            new_local[i] = w * pw.GetInverse()
        new_world[i] = w
    return new_local, new_world


def _read_skeleton(asset_url: str):
    """Open *asset_url* and return its (joints, rest_transforms), or None."""
    try:
        src = Usd.Stage.Open(asset_url)
    except Exception:
        src = None
    if src is None:
        return None
    prim = next((p for p in src.Traverse() if p.GetTypeName() == "Skeleton"), None)
    if prim is None:
        return None
    skel = UsdSkel.Skeleton(prim)
    joints = skel.GetJointsAttr().Get()
    rest = skel.GetRestTransformsAttr().Get()
    if not joints or not rest or len(rest) != len(joints):
        return None
    return list(joints), list(rest)


def _bind_human_pose(stage, prim, usd: str, pose: str, pose_cache: dict):
    """Author a static pose animation under the placed human *prim* (a
    reference to *usd*) and bind it to the composed Skeleton. Posed local
    transforms are computed once per (asset, pose) and cached in
    *pose_cache*; assets without a readable skeleton cache ``None`` and stay
    in their authored pose.
    """
    key = (usd, pose)
    if key not in pose_cache:
        deltas = _HUMAN_POSES.get(pose)
        skel_data = _read_skeleton(usd) if deltas else None
        if skel_data is None:
            pose_cache[key] = None
        else:
            joints, rest = skel_data
            new_local, _ = _pose_joint_transforms(joints, rest, deltas)
            pose_cache[key] = (Vt.TokenArray([str(j) for j in joints]),
                               Vt.Matrix4dArray(new_local))
    cached = pose_cache[key]
    if cached is None:
        return
    skel_prim = next((c for c in Usd.PrimRange(prim)
                      if c.GetTypeName() == "Skeleton"), None)
    if skel_prim is None:
        return
    anim = UsdSkel.Animation.Define(stage, prim.GetPath().AppendChild("Pose"))
    anim.GetJointsAttr().Set(cached[0])
    anim.SetTransforms(cached[1], Usd.TimeCode.Default())
    binding = UsdSkel.BindingAPI.Apply(skel_prim)
    binding.CreateAnimationSourceRel().SetTargets([anim.GetPrim().GetPath()])


# ---------------------------------------------------------------------------
# USD composition
# ---------------------------------------------------------------------------

def _sanitize(name: str) -> str:
    out = "".join(c if c.isalnum() else "_" for c in name)
    return out if out and not out[0].isdigit() else f"_{out}"


#: USDs already reported as composing nothing, so the warning is one line
#: per broken asset rather than one per placement. See the note inside
#: `apply_placements`.
_warned_unresolved: set = set()


#: Collider-ready overlays written by `tools/collider_cache.py`. An INSTANCED
#: placement must reference one of these rather than the raw asset: an
#: instanceable prim has no traversable children, so `scene_prep.add_colliders`
#: cannot reach inside it and the placement would silently lose its collision.
_COLLIDER_CACHE_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "assets", "_collider_cache")
_collider_ready_index = {}


def _collider_ready(url: str) -> str:
    """The collider-ready variant of *url*, or "" if it has not been cached."""
    if url in _collider_ready_index:
        return _collider_ready_index[url]
    got = ""
    try:
        from tools.collider_cache import cache_path
        cand = cache_path(url, _COLLIDER_CACHE_DIR)
        got = cand if os.path.isfile(cand) else ""
    except Exception:                                            # noqa: BLE001
        got = ""
    _collider_ready_index[url] = got
    return got


#: A placement is never instanced while one of these is set, whatever its
#: category says: each marks a pass that authors INSIDE the asset, and USD
#: forbids that on an instance. `settle` is `settle_rigid_props`, `pose` is
#: `_bind_human_pose`, `_mesh_damage` is the cutter.
_NO_INSTANCE_KEYS = ("settle", "pose", "_mesh_damage")


def apply_placements(stage,
                     placements: list,
                     parent_path: str = "/World/stage/generated",
                     scene_scale_factor: float = 1.0,
                     ground_snap=None,
                     resolver: "SizeResolver | None" = None,
                     instance_categories: "set | None" = None) -> str:
    """Write *placements* onto *stage* as USD references under *parent_path*.

    **Not instanceable by default, deliberately.** `SetInstanceable(True)` was
    unconditional here originally and was removed in 8187043e, because
    `scene_prep.add_colliders` recursively applies `UsdPhysics.CollisionAPI` to
    every gprim beneath each placement — and an instanceable prim has no
    traversable children, so `GetChildren()` returns nothing and no collider is
    ever applied. The drone would fly through buildings and LiDAR would see
    nothing. `settle_rigid_props`, `prune_prims`, `apply_surface_overrides` and
    `_bind_human_pose` all edit inside referenced assets too, so they have the
    same conflict.

    The cost is real — the urban scene OOM-killed at 89.1M points, and the
    graph suburb plants 3,384 trees at ~55k points each = ~186M — so
    *instance_categories* opts INDIVIDUAL CATEGORIES back in. Opt-in, per
    category, because the constraint above is per-pass, not global: a category
    may be instanced only if NOTHING that runs later authors inside it. In
    practice that means it must be touched by none of `add_colliders`,
    `prune_prims`, `apply_surface_overrides`, `settle_rigid_props`,
    `_bind_human_pose` — nor by the disaster passes, since `disaster/` scorches
    and fractures real placed geometry and USD forbids editing inside an
    instance. `suburb_preview` can instance freely (it calls none of them);
    `suburb_mini_wildfire` therefore lists only what does not burn, and the
    urban path cannot use this at all without auditing its prune list first.

    The general fix is still the one 8187043e pointed at: pre-author
    `CollisionAPI` into cached per-asset variants so prototypes arrive
    collider-ready, then instance those, which fits the `prepare_assets.py`
    cache pattern. This parameter is the narrow version that works today.

    **STAGE A ARCHETYPES ALREADY MEET THAT BAR**, which is why a placement may
    also opt itself in with `_instanceable`. `disaster/bake.export_object`
    authors `UsdPhysics.CollisionAPI` on every merged mesh it writes, and
    `scene_prep.add_colliders` skips a prim that already has the API — so an
    instanced archetype loses no collider, and none of the other passes reach
    inside one either: mesh damage skips it (`disaster_stage` pops
    `_mesh_damage`), the settle skips it (not marked `settle`), the debris it
    sheds is authored as a SIBLING scope, and `targets.mark_cut_geometry` reads
    the survey's `_archetype` flag rather than traversing for a `fragments`
    scope. That matters because a reference is otherwise resident in full per
    placement: eighty buildings drawn from thirty archetypes cost eighty
    copies, which is the whole of why a 500 m map runs the host out of RAM.

    Metric coordinates are multiplied by *scene_scale_factor* (= 1 /
    meters_per_unit) to land in stage units. Each prim gets translate /
    rotateXYZ (roll, pitch, yaw) / scale ops. *ground_snap*, if given, is a
    callable ``(x_m, y_m) -> z_m | None`` overriding the placement Z.

    *resolver*, if given, is used to look up each USD's bbox center offset
    (``cx``, ``cy``) so the *visual* centroid of the asset lands at the
    requested metric position — necessary when a prop's pivot isn't at its
    geometric center. The offset is rotated with the placement, so rotated
    tiles (e.g. road tees/dead-ends) stay correctly positioned.
    """
    UsdGeom.Scope.Define(stage, Sdf.Path(parent_path))
    proto_index: dict = {}
    pose_cache: dict = {}
    ssf = float(scene_scale_factor)
    n_instanced = 0
    #: Assets a category asked to instance that have no collider-ready
    #: variant. Reported once at the end rather than per placement: one
    #: uncached tile would otherwise print 57,943 identical lines.
    _uncached_instanceables: set = set()

    missing_refs: set = set()

    for i, p in enumerate(placements):
        usd = p["usd"]
        group = proto_index.setdefault(usd, len(proto_index))
        prim_path = f"{parent_path}/{p.get('category', 'asset')}_{group}_{i}"
        # LOCAL SCHEMES MUST BE EXPANDED HERE, and the failure is silent.
        # USD has never heard of `airstack://` (same note as `damage._TEX_DIR`),
        # and `AddReference` returns True for an asset it cannot open — the
        # prim simply composes empty, so a dangling reference looks exactly
        # like a building that was never placed.
        #
        # Pack-declared assets are expanded by `_normalize_usd_list` when the
        # pool is built, but anything injected into a placement AFTERWARDS is
        # not. `disaster_stage` writes an `airstack://` archetype URL (on
        # purpose — placements stay relocatable, and the Stage B tests assert
        # that form), which meant every archetype-backed building came up
        # invisible and the scene showed only its debris.
        ref_url = _expand_scheme(usd) or usd

        # DECIDED BEFORE THE REFERENCE IS ADDED, because an instanced
        # placement has to reference the COLLIDER-READY variant -- see
        # `_collider_ready`. A placement that opted itself in
        # (`_instanceable`, i.e. a Stage A archetype) already carries its
        # colliders and references the asset directly.
        self_opted = bool(p.get("_instanceable"))
        cat_opted = bool(instance_categories
                         and p.get("category") in instance_categories)
        if any(p.get(k) for k in _NO_INSTANCE_KEYS):
            cat_opted = False
        want_instance = self_opted or cat_opted
        if cat_opted and not self_opted:
            ready = _collider_ready(ref_url)
            if ready:
                ref_url = ready
            else:
                # No cached variant: instancing here would drop the collider,
                # so DON'T. Placing it uninstanced is the safe failure.
                want_instance = False
                if usd not in _uncached_instanceables:
                    _uncached_instanceables.add(usd)

        if ref_url != usd and ref_url not in missing_refs \
                and not os.path.exists(ref_url):
            missing_refs.add(ref_url)
            print(f"[scene_gen] WARN: {usd} -> {ref_url} is not on disk; "
                  f"that prim will compose empty")
        # No local typeName: some assets (e.g. Dmytro/Unreal exports) have a
        # Mesh as their root prim, and a local "Xform" opinion would override
        # the referenced "Mesh" type — attributes still compose (bbox looks
        # right) but the prim stops being a gprim and renders nothing. A
        # typeless def lets the referenced asset's own type win.
        prim = stage.DefinePrim(prim_path)
        p["prim_path"] = prim_path      # so callers can post-process (settling)
        if not prim.GetReferences().AddReference(ref_url):
            print(f"[scene_gen] WARN: failed to reference {ref_url} at {prim_path}")
            continue
        # A SCOPE-ROOTED ASSET GETS PROMOTED. Stage A libraries baked before
        # 2026-08-25 root every archetype at a `Scope`, which composes but is
        # not Xformable, so the transform below could not be authored and the
        # building was silently skipped. Scope -> Xform is lossless (both are
        # plain containers), so a local Xform opinion is the right repair;
        # a Mesh root is left alone for the reason given above.
        if prim.GetTypeName() == "Scope":
            prim.SetTypeName("Xform")
        # Some asset packs (e.g. Dmytro) wrap their geometry in an internal
        # payload arc for streaming. Prims composed into an already-running
        # stage don't auto-load nested payloads the way Usd.Stage.Open() does,
        # so the reference lands with a correct bbox/transform but no visible
        # geometry unless we force-load it here.
        prim.Load()

        # Rigged humans: bind a procedurally authored static pose so they
        # don't compose in their T-shaped bind pose (see _HUMAN_POSES).
        if p.get("pose"):
            _bind_human_pose(stage, prim, usd, p["pose"], pose_cache)

        z_m = p["z_m"]
        if ground_snap is not None:
            snapped = ground_snap(p["x_m"], p["y_m"])
            if snapped is not None:
                z_m = snapped + p["z_m"]

        # The asset's prim anchor sits at the corner of its bbox, not the visual
        # centroid. We keep ops as translate -> rotateXYZ -> scale (so rotation
        # pivots about the anchor), but pre-rotate the anchor->centroid offset by
        # that same rotation and fold it into the translation. The mesh then ends
        # up exactly where a rotation-about-centroid would put it. With no
        # rotation this reduces to a plain XY recenter.
        roll = float(p.get("roll_deg", 0.0))
        pitch = float(p.get("pitch_deg", 0.0))
        yaw = float(p["yaw_deg"])

        # Optional per-axis local-XY stretch on top of the uniform scale
        # (tiles stretched to exactly cover their grid step).
        st = p.get("stretch")
        stx, sty = (float(st[0]), float(st[1])) if st else (1.0, 1.0)

        cx_off, cy_off, cz_off = 0.0, 0.0, 0.0
        if resolver is not None:
            axis_up_p = p.get("axis_up", "Z")
            fp = resolver.get(usd, p.get("category", "asset"),
                              scale=float(p["scale"]), axis_up=axis_up_p)
            cx_off = fp.get("cx", 0.0) * stx
            cy_off = fp.get("cy", 0.0) * sty
            cz_off = fp.get("cz", 0.0)

        # Rotate the anchor->centroid offset by the same XYZ rotation USD applies
        # (X, then Y, then Z). TransformDir keeps us off Gf's row-vector layout.
        # For Y-up assets: roll includes +90° correction so local Z (cz_off)
        # rotates to world -Y, and local Y (cy_off=0) doesn't corrupt Z placement.
        offset = Gf.Vec3d(cx_off, cy_off, cz_off)
        offset = Gf.Rotation(Gf.Vec3d(1, 0, 0), roll).TransformDir(offset)
        offset = Gf.Rotation(Gf.Vec3d(0, 1, 0), pitch).TransformDir(offset)
        offset = Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw).TransformDir(offset)

        xform = UsdGeom.Xformable(prim)
        if not xform:
            # Reference composed nothing typed (missing/broken asset) — the
            # holder prim is typeless and can't carry transform ops.
            #
            # ONCE PER ASSET, not per placement. This warned every time and a
            # single bad ground tile emitted 30,420 identical lines, so the one
            # line that mattered — a building asset that resolves to nothing —
            # was unfindable in the scrollback. That is why "some buildings are
            # missing" went undiagnosed: the diagnostic was already firing.
            # `generate_scene.report_empty_placements` prints the grouped tally.
            if usd not in _warned_unresolved:
                _warned_unresolved.add(usd)
                print(f"[scene_gen] WARN: {usd} composed no Xformable prim "
                      f"(first at {prim_path}; further copies not logged)")
            continue
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(
            (p["x_m"] - offset[0]) * ssf,
            (p["y_m"] - offset[1]) * ssf,
            (z_m - offset[2]) * ssf))
        xform.AddRotateXYZOp().Set(Gf.Vec3f(roll, pitch, yaw))
        s = float(p["scale"])
        xform.AddScaleOp().Set(Gf.Vec3d(s * stx, s * sty, s))
        # PER PLACEMENT, not only per category. The category rule cannot
        # express the case that matters most: on a damaged map the `house`
        # category holds intact models, authored ruins, buildings the cutter is
        # about to edit in place, and pre-baked archetypes — and only the last
        # of those is safe to instance. `_instanceable` lets whoever KNOWS say
        # so; see `disaster_stage`'s archetype branch for why an archetype is
        # the one asset in the scene that qualifies.
        if want_instance:
            prim.SetInstanceable(True)
            n_instanced += 1

    if _uncached_instanceables:
        print(f"[scene_gen] {len(_uncached_instanceables)} asset(s) in an "
              f"instanced category have no collider-ready variant and were "
              f"placed UNINSTANCED (run tools/collider_cache.py): "
              f"{sorted(_uncached_instanceables)[:4]}")
    print(f"[scene_gen] Applied {len(placements)} placements under '{parent_path}' "
          f"({len(proto_index)} unique USDs, scale_factor={ssf}"
          + (f", {n_instanced} instanced" if n_instanced else "") + ")")
    return parent_path


def _make_resolver(config: dict, cache=None) -> SizeResolver:
    """The resolver for *config*, backed by the on-disk measurement cache.

    Pass ``cache=False`` for a cold resolver (the tests want one, so a stale
    cache can never make a failing measurement look like a passing one).
    """
    # `is None` / `is False`, never truthiness: `MeasureCache` defines
    # `__len__`, so an EMPTY cache is falsy and `cache or None` silently
    # discarded the very cache a cold run had just been handed.
    if cache is None:
        from measure_cache import MeasureCache
        cache = MeasureCache()
    elif cache is False:
        cache = None
    return SizeResolver(
        asset_scale=float(config.get("asset_scale", 1.0)),
        fallback_sizes=config.get("fallback_sizes", {}),
        measure=bool(config.get("measure_usds", True)),
        cache=cache,
    )


# ---------------------------------------------------------------------------
# Offline bake — no Isaac Sim required (usd-core pip package is enough)
# ---------------------------------------------------------------------------

def generate_scene_usd(config, output_path: str, scene_scale_factor: float = 1.0) -> str:
    """Bake a standalone ``.usd`` city from *config* (path or dict). Offline.

    Footprint measurement needs to open the referenced USDs, so set
    ``measure_usds: false`` (use fallback sizes) when baking without Nucleus
    access, or run the bake where the assets are reachable.
    """
    if isinstance(config, str):
        config = load_config(config)

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0 / float(scene_scale_factor))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))

    terrain = config.get("terrain_usd")
    if terrain:
        tprim = stage.DefinePrim("/World/terrain")  # typeless — see apply_placements
        tprim.GetReferences().AddReference(terrain)

    resolver = _make_resolver(config)
    placements, layout = build_city(config, resolver)
    apply_placements(stage, placements, "/World/stage/generated", scene_scale_factor,
                     resolver=resolver)
    apply_ground_planes(stage, config, layout, "/World/stage/generated", scene_scale_factor)

    stage.GetRootLayer().Save()
    print(f"[scene_gen] Wrote city to {output_path}")
    return output_path


# ---------------------------------------------------------------------------
# Runtime compose — onto a live Isaac Sim stage
# ---------------------------------------------------------------------------

def _make_physx_ground_snap(max_height_m: float = 500.0):
    """Ground-snap callable that raycasts straight down onto terrain colliders,
    or None if PhysX scene-query isn't available (e.g. offline).
    """
    try:
        import carb
        from omni.physx import get_physx_scene_query_interface
    except Exception:
        print("[scene_gen] PhysX scene query unavailable — ground snap disabled")
        return None

    sq = get_physx_scene_query_interface()

    def snap(x_m: float, y_m: float):
        origin = carb.Float3(x_m, y_m, max_height_m)
        direction = carb.Float3(0.0, 0.0, -1.0)
        hit = sq.raycast_closest(origin, direction, max_height_m * 2.0)
        if hit and hit.get("hit"):
            return float(hit["position"][2])
        return None

    return snap


def generate_scene_on_stage(stage,
                            config,
                            parent_path: str = "/World/stage/generated",
                            scene_scale_factor: float = 1.0,
                            snap_to_ground: bool = False) -> list:
    """Build the city directly onto a live Isaac Sim *stage*.

    Kept as the name four launch scripts already call; it now **delegates to
    `generate_scene.generate_scene_on_stage`**, which is the one pipeline.

    It used to call `build_city` and `apply_placements` itself, and that stopped
    being correct the moment damage moved out of `build_city` into the disaster
    stage: this function would happily build a *pristine* city for a preset
    asking for a tornado, with no error anywhere. `preset_report.py` had the
    same bug for the same reason.

    Delegating is safe for a v1 config as well as a v2 one. Every detailed pass
    is config-gated and no-ops when its settings are absent — verified: a v1
    config through the v2 pipeline yields a byte-identical placement digest
    (`patched=False`, `rings=False`, `detail_added=0`).
    """
    # Lazy: generate_scene imports this module, so a top-level import here
    # would be circular.
    import generate_scene

    return generate_scene.generate_scene_on_stage(
        stage, config, parent_path, scene_scale_factor, snap_to_ground)


# `reload_scene_on_stage` is `generate_scene.reload_scene_on_stage`. It used to
# be duplicated here, and the two copies drifted: this one never learned about
# the mesh-damage fragments the other appends, so a reload in the Script Editor
# produced a subtly different scene from a fresh launch. One pipeline, one
# reload — see `generate_scene_on_stage` just above, which delegates the same way.
def reload_scene_on_stage(stage, config,
                          parent_path: str = "/World/stage/generated",
                          scene_scale_factor: float = 1.0,
                          snap_to_ground: bool = False,
                          add_colliders_fn=None) -> list:
    """Clear a generated subtree and rebuild it in place, without restarting
    Isaac Sim. Delegates to `generate_scene`, which is the one pipeline."""
    import generate_scene
    return generate_scene.reload_scene_on_stage(
        stage, config, parent_path, scene_scale_factor, snap_to_ground,
        add_colliders_fn)


# ---------------------------------------------------------------------------
# Offline CLI: python scene_generator.py --config <yaml> --output <usd>
# ---------------------------------------------------------------------------

def main():
    import argparse

    parser = argparse.ArgumentParser(description="Bake a procedural USD city from a YAML spec.")
    parser.add_argument("--config", required=True, help="Path to city-generator YAML spec")
    parser.add_argument("--output", required=True, help="Output .usd path")
    parser.add_argument("--scale-factor", type=float, default=1.0,
                        help="Metric→stage-unit factor (1/meters_per_unit). Default 1.0 (meters).")
    args = parser.parse_args()

    generate_scene_usd(args.config, args.output, args.scale_factor)


if __name__ == "__main__":
    main()
