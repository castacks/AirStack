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

def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge *override* into *base* in place: nested dicts merge
    key-by-key, everything else (scalars, lists) is replaced outright."""
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v
    return base


def _merge_asset_set(base: dict, child: dict) -> dict:
    """Merge *child* asset set over *base* in place.

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
            _merge_asset_set(base[k], v)
        else:
            base[k] = v
    return base


def _find_asset_set(name: str, dirs: list) -> str:
    candidates = [str(name)]
    for d in dirs:
        candidates += [os.path.join(d, str(name)),
                       os.path.join(d, f"{name}.yaml"),
                       os.path.join(d, f"{name}.yml")]
    path = next((c for c in candidates if os.path.isfile(c)), None)
    if path is None:
        searched = "\n".join(f"    {os.path.normpath(d)}" for d in dirs)
        raise ValueError(f"asset_set {name!r} not found. Searched:\n{searched}")
    return path


def _load_asset_set(name: str, dirs: list, chain: list = None) -> tuple:
    """Load an asset set and everything it ``extends``, base-first.

    Returns ``(merged_dict, [paths in resolution order])``.
    """
    import yaml

    chain = list(chain or [])
    if name in chain:
        raise ValueError("asset_set extends cycle: "
                         + " -> ".join(chain + [str(name)]))
    chain.append(name)

    path = _find_asset_set(name, dirs)
    with open(path) as f:
        doc = yaml.safe_load(f)
    if not isinstance(doc, dict):
        raise ValueError(f"asset set {path!r} did not parse to a mapping")

    parent = doc.pop("extends", None)
    if not parent:
        return doc, [path]
    # A LIST of parents is allowed, merged left to right, so a set can join two
    # libraries without duplicating either -- the suburb needs the suburban
    # pools AND the park's recreation pools, and neither has any reason to carry
    # the other's. `prepare_assets._with_extends` already accepted a list; this
    # side did not, so a list resolved on the host and then died at scene build
    # with `asset_set "[...]" not found`. The two now agree.
    parents = [parent] if isinstance(parent, str) else list(parent)
    base, paths = {}, []
    for name in parents:
        b, ps = _load_asset_set(str(name), dirs, chain)
        base = _merge_asset_set(base, b) if base else b
        paths += ps
    return _merge_asset_set(base, doc), paths + [path]


def _filter_asset_prefixes(value, prefixes):
    """Remove USD entries whose path begins with a blacklisted prefix."""
    if isinstance(value, list):
        kept = []
        for item in value:
            usd = item.get("usd", "") if isinstance(item, dict) else item
            if isinstance(usd, str) and any(usd.startswith(p) for p in prefixes):
                continue
            kept.append(_filter_asset_prefixes(item, prefixes))
        return kept
    if isinstance(value, dict):
        return {k: _filter_asset_prefixes(v, prefixes)
                for k, v in value.items()}
    return value


def resolve_asset_set(config: dict, config_path: str = None) -> dict:
    """Merge the asset set named by ``config['asset_set']`` into *config*.

    An asset set (``config/asset_sets/<name>.yaml``) holds
    *what the assets are* — ``asset_root``, ``asset_scale``, ``sky``,
    ``orientation``, ``fallback_sizes`` and the whole ``usds`` library — so a
    scene can be re-skinned by naming a different set. The config wins on
    conflicts, so it can override anything the set defines.

    A set may ``extends:`` another, which is how the locale sets avoid
    restating the props every locale shares (see ``shared.yaml``): the base is
    loaded first and the child merged over it, with ``<key>+`` appending to an
    inherited list rather than replacing it.

    ``asset_set`` may be a bare name or a path. No-op when absent (a config
    carrying its own ``usds`` is still valid).
    """
    name = config.get("asset_set")
    if not name:
        return config

    here = os.path.dirname(os.path.abspath(__file__))
    dirs = [os.path.join(here, "config", "asset_sets")]
    if config_path:
        cfg_dir = os.path.dirname(os.path.abspath(config_path))
        # low_level/<cfg>.yaml and low_level/compiled/<cfg>.yaml both sit
        # under config/, so look up as well as alongside.
        dirs[:0] = [os.path.join(cfg_dir, "asset_sets"),
                    os.path.join(cfg_dir, "..", "asset_sets"),
                    os.path.join(cfg_dir, "..", "..", "asset_sets")]

    assets, paths = _load_asset_set(str(name), dirs)
    prefixes = assets.pop("asset_blacklist_prefixes", []) or []
    if prefixes:
        assets = _filter_asset_prefixes(assets, tuple(str(p) for p in prefixes))

    merged = _deep_merge(assets, config)      # config overrides the set
    merged["asset_set"] = name
    lineage = " <- ".join(os.path.basename(p) for p in reversed(paths))
    print(f"[scene_gen] asset set: {name} ({lineage})")
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
            "set with `asset_set: <name>` (see config/asset_sets/), "
            "or inline a usds section.")
    if "layout" not in config or "region_m" not in config["layout"]:
        raise ValueError(f"City config {source!r} has no 'layout.region_m' "
                         "(city extent in meters, e.g. [200, 200])")

    usds = config["usds"]
    # "buildings" is the current section name; "houses" accepted for
    # backward compatibility.
    bld = usds.get("buildings") or usds.get("houses") or {}
    if not bld.get("intact"):
        raise ValueError("usds.buildings.intact must list at least one building USD")

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
              f"{config.get('asset_set') or ''}".rstrip())

    return config


def load_config(config_path: str) -> dict:
    """Load and lightly validate a **low-level** city-generator YAML spec.

    A low-level spec holds the generator settings and names an asset set
    (``asset_set: urban``) for the asset library itself, which is merged in
    here — see :func:`resolve_asset_set`. Author them by hand
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
        config = resolve_asset_set(config, config_path)
    return validate_config(config, config_path)


# ---------------------------------------------------------------------------
# USD footprint measurement (with graceful fallback to constants)
# ---------------------------------------------------------------------------

def _measure_footprint(usd_path: str, scale: float, axis_up: str = "Z"):
    """Open *usd_path* and return its metric footprint as a dict, or None.

    Returns ``{"sx","sy","sz","base","cx","cy","cz"}`` in Z-up world space
    (after *scale* and any axis correction). For Z-up assets cx/cy are the XY
    bbox center offsets from the prim origin; cz is always 0. For Y-up assets
    (``axis_up="Y"``) the measurements are remapped so that sx/sy reflect the
    world-XY footprint, base reflects the local Y_min, and cz carries the local-Z
    centroid that apply_placements folds into the centroid-correction offset vector
    (local Z maps to world -Y after the +90° roll correction).
    """
    try:
        stage = Usd.Stage.Open(usd_path)
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
            "sx": sz[0] * scale,                       # local X  → world X footprint
            "sy": sz[2] * scale,                       # local Z  → world Y footprint
            "sz": sz[1] * scale,                       # local Y  → world height
            "base": -mn[1] * scale,                    # local Y_min → world Z = 0 after roll
            "cx": (mn[0] + sz[0] / 2) * scale,        # local X centroid (stays world X)
            "cy": 0.0,                                 # height centroid; no XY correction needed
            "cz": (mn[2] + sz[2] / 2) * scale,        # local Z centroid → world -Y via roll
        }
    return {"sx": sz[0] * scale, "sy": sz[1] * scale,
            "sz": sz[2] * scale, "base": -mn[2] * scale,
            # XY offset of the bbox center from the asset's local origin.
            # Non-zero when the pivot isn't at the visual centroid; apply_placements
            # subtracts this so the *visual* center lands at the requested position.
            "cx": (mn[0] + sz[0] / 2) * scale,
            "cy": (mn[1] + sz[1] / 2) * scale,
            "cz": 0.0}


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

# AIRSTACK_ASSET_ROOT repoints `airstack://` off the local repo and onto a
# Nucleus URL (or any other root) — e.g.
#   AIRSTACK_ASSET_ROOT=omniverse://host:443/Projects/SEI-COA
# makes `airstack://scene_gen/assets/x` resolve to
#   omniverse://host:443/Projects/SEI-COA/scene_gen/assets/x
# so a scene can run entirely off uploaded assets instead of the local mount.
_AIRSTACK_ROOT = os.environ.get("AIRSTACK_ASSET_ROOT", "").strip() or _REPO_ROOT

LOCAL_ASSET_ROOTS = {
    "airstack": _AIRSTACK_ROOT,
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
    root = LOCAL_ASSET_ROOTS[scheme]
    # A URL root (omniverse://, s3://, …) must be JOINED as a URL — os.path.join
    # would leave the "://" but is otherwise fine on posix; be explicit anyway.
    if "://" in str(root):
        return str(root).rstrip("/") + "/" + rest
    return os.path.join(root, rest)


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


def _normalize_usd_list(lst, default_scale: float, asset_root: str = ""):
    """Return ``(path_list, scale_overrides, axis_up_overrides, yaw_overrides,
    tag_overrides)`` from a raw YAML USD list.

    *path_list* is a plain list of USD paths suitable for ``rng.choice()``.
    *scale_overrides* maps path → per-asset scale for entries with an explicit ``scale``.
    *axis_up_overrides* maps path → axis_up string ("Y") for non-Z-up entries.
    *yaw_overrides* maps path → yaw-offset degrees for entries that declare one.
    *tag_overrides* maps path → frozenset of tags for entries that declare any.
    """
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
    """

    def __init__(self, asset_scale: float, fallback_sizes: dict, measure: bool):
        self.scale = float(asset_scale)
        self.fallback = fallback_sizes or {}
        self.measure = bool(measure)
        self._cache: dict = {}

    def get(self, usd_path: str, category: str, scale: float = None,
            axis_up: str = "Z") -> dict:
        """Return the metric footprint for *usd_path* in Z-up world space.

        *scale* overrides the resolver's default ``asset_scale``.
        *axis_up* is ``"Y"`` for assets authored in Y-up coordinates (e.g. Unity
        exports); the returned dict has corrected sx/sy/base and includes a ``cz``
        field so apply_placements can handle the centroid offset correctly. The
        cache key is ``(path, effective_scale, axis_up)`` so different corrections
        don't collide.
        """
        effective_scale = float(scale) if scale is not None else self.scale
        effective_axis_up = str(axis_up).upper() if axis_up else "Z"
        cache_key = (usd_path, effective_scale, effective_axis_up)
        if cache_key in self._cache:
            return self._cache[cache_key]

        # MEASURE ONCE PER (path, axis), AT SCALE 1.0, AND MULTIPLY. The
        # footprint is linear in scale (a bbox times a uniform scale; the
        # Y-up remap is a permutation and sign, independent of scale), but
        # per-instance scale JITTER made every placement a fresh cache key —
        # the same plant asset was opened and measured thousands of times
        # (32,587 log lines, one Usd.Stage.Open each) and the 500 m city
        # build ballooned to 38.8 GB RSS and was OOM-killed (2026-08-31).
        raw_key = (usd_path, effective_axis_up)
        if not hasattr(self, "_raw"):
            self._raw = {}
        if raw_key not in self._raw:
            self._raw[raw_key] = (
                _measure_footprint(usd_path, 1.0, effective_axis_up)
                if self.measure else None)
            if self._raw[raw_key] is not None:
                r = self._raw[raw_key]
                print(f"[scene_gen] measured {category}: "
                      f"{os.path.basename(usd_path)} -> "
                      f"{r['sx']:.2f} x {r['sy']:.2f} (at scale 1, "
                      f"up={effective_axis_up}; per-instance scales derived)")
        raw = self._raw[raw_key]
        fp = (None if raw is None else
              {k: raw[k] * effective_scale
               for k in ("sx", "sy", "sz", "base", "cx", "cy", "cz")})
        if fp is None:
            fb = self.fallback.get(category, [4.0, 4.0])
            fp = {"sx": float(fb[0]), "sy": float(fb[1]),
                  "sz": float(fb[2]) if len(fb) > 2 else 3.0, "base": 0.0,
                  "cx": 0.0, "cy": 0.0, "cz": 0.0}
            print(f"[scene_gen] fallback {category}: {os.path.basename(usd_path)} "
                  f"-> {fp['sx']:.2f} x {fp['sy']:.2f} m")

        self._cache[cache_key] = fp
        return fp


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
# Every disaster knob under ``disaster.*`` is a *maximum*, reached only where
# the field reads 1.0. ``disaster.field`` shapes that intensity over the
# region, which is what separates disaster types from each other: an
# earthquake attenuates radially from its epicenter, a tornado carves a
# narrow path and leaves the rest of the city untouched, a blast falls off
# sharply from ground zero. Absent a field, intensity is a flat 1.0
# everywhere (the historical behavior).
# ---------------------------------------------------------------------------

def _smoothstep(t: float) -> float:
    """Hermite ease on [0, 1] — softer edges than a linear ramp."""
    t = min(1.0, max(0.0, t))
    return t * t * (3.0 - 2.0 * t)


def _point_segment_dist(px, py, ax, ay, bx, by) -> float:
    vx, vy = bx - ax, by - ay
    seg2 = vx * vx + vy * vy
    if seg2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / seg2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def _path_segments(cfg: dict, region: tuple) -> list:
    """Segments for a ``field.kind: path`` track, building the default
    straight sweep across *region* on ``heading_deg`` when no explicit
    ``points`` are given. Shared by :func:`make_damage_field` and
    :func:`make_scour_density` so both agree on where the track actually is.
    """
    pts = cfg.get("points")
    if not pts:
        rx0, ry0, rx1, ry1 = region
        mx, my = (rx0 + rx1) / 2.0, (ry0 + ry1) / 2.0
        ang = math.radians(float(cfg.get("heading_deg", 45.0)))
        reach = math.hypot(rx1 - rx0, ry1 - ry0)
        pts = [[mx - math.cos(ang) * reach, my - math.sin(ang) * reach],
               [mx + math.cos(ang) * reach, my + math.sin(ang) * reach]]
    return [(float(pts[i][0]), float(pts[i][1]),
             float(pts[i + 1][0]), float(pts[i + 1][1]))
            for i in range(len(pts) - 1)] or [
        (float(pts[0][0]), float(pts[0][1]),
         float(pts[0][0]), float(pts[0][1]))]


def make_damage_field(field_cfg: dict, region: tuple):
    """Build ``f(x, y) -> intensity`` (0..1) from a ``disaster.field`` spec.

    ``kind``:
      * ``uniform`` — ``inside`` everywhere (default when no field is given).
      * ``radial``  — ``inside`` within ``radius_m`` of ``center``, easing to
        ``outside`` over the next ``falloff_m``. Earthquakes, blasts.
      * ``path``    — ``inside`` within ``width_m`` of the ``points``
        polyline, easing to ``outside`` over ``falloff_m``. Tornado tracks.

    *region* is ``(x0, y0, x1, y1)`` in meters; a ``path`` with no explicit
    ``points`` gets a default track crossing the region at ``heading_deg``.

    The returned callable carries ``.lo`` / ``.hi`` — the intensity bounds
    over the whole region. Callers use them to decide what is *possible*
    anywhere (e.g. whether any building can stay intact) before sampling.
    """
    cfg = field_cfg or {}
    kind = str(cfg.get("kind", "uniform")).lower()
    inside = float(cfg.get("inside", 1.0))
    outside = float(cfg.get("outside", 0.0))

    def _tag(fn, lo, hi):
        fn.lo, fn.hi = lo, hi
        return fn

    if kind == "uniform":
        return _tag(lambda x, y: inside, inside, inside)

    def _ease(dist, full, fall):
        """inside within *full*, easing to outside across *fall* beyond it."""
        if dist <= full:
            return inside
        return inside + (outside - inside) * _smoothstep(
            (dist - full) / max(fall, 1e-6))

    if kind == "radial":
        cx, cy = cfg.get("center", [0.0, 0.0])
        cx, cy = float(cx), float(cy)
        radius = float(cfg.get("radius_m", 80.0))
        falloff = float(cfg.get("falloff_m", 120.0))
        return _tag(lambda x, y: _ease(math.hypot(x - cx, y - cy),
                                       radius, falloff),
                    min(inside, outside), max(inside, outside))

    if kind == "path":
        half_w = float(cfg.get("width_m", 60.0)) / 2.0
        falloff = float(cfg.get("falloff_m", 40.0))
        segs = _path_segments(cfg, region)

        def f(x, y):
            d = min(_point_segment_dist(x, y, *s) for s in segs)
            return _ease(d, half_w, falloff)

        return _tag(f, min(inside, outside), max(inside, outside))

    raise ValueError(f"Unknown disaster.field.kind {kind!r} "
                     "(expected uniform, radial or path)")


def make_scour_density(field_cfg: dict, region: tuple, shape: float = 1.6):
    """Build ``g(x, y) -> [0, 1]``: a density *gradient* peaked at the
    disaster's core (track centerline / epicenter), falling smoothly to 0 at
    the same distance where :func:`make_damage_field`'s own ease finishes.

    Distinct from ``make_damage_field``'s intensity, which is a *plateau* —
    flat ``inside`` through the whole width or radius, easing only at the
    very edge. That shape is right for deciding whether a building in the
    corridor survives (a tornado hits everything in its path about equally
    hard), but wrong for ground scour: in real tornado aftermath the debris
    is visibly densest right under the vortex track and thins out well
    before the edge of the damage zone, not uniform across the whole width.

    Reuses the field's own geometry (its track / epicenter, width/radius and
    falloff) so the *extent* debris can appear in is exactly what
    ``disaster.field`` already configures — only the density gradient inside
    that extent changes. ``shape`` > 1 concentrates density nearer the core
    than a plain smoothstep falloff would; 1.0 is just the smoothstep.
    """
    cfg = field_cfg or {}
    kind = str(cfg.get("kind", "uniform")).lower()

    def _peak(d, reach):
        t = min(1.0, max(0.0, d / max(reach, 1e-6)))
        return (1.0 - _smoothstep(t)) ** shape

    if kind == "path":
        half_w = float(cfg.get("width_m", 60.0)) / 2.0
        falloff = float(cfg.get("falloff_m", 40.0))
        reach = half_w + falloff
        segs = _path_segments(cfg, region)
        return lambda x, y: _peak(min(_point_segment_dist(x, y, *s)
                                      for s in segs), reach)

    if kind == "radial":
        cx, cy = cfg.get("center", [0.0, 0.0])
        cx, cy = float(cx), float(cy)
        radius = float(cfg.get("radius_m", 80.0))
        falloff = float(cfg.get("falloff_m", 120.0))
        reach = radius + falloff
        return lambda x, y: _peak(math.hypot(x - cx, y - cy), reach)

    return lambda x, y: 1.0   # uniform field: no core to concentrate around


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

    Five layers are written under *parent_path/ground*:
    1. One asphalt UsdGeom.Mesh spanning the whole region at z=0.
    2. One grass UsdGeom.Mesh per block at z=0.01 (sits above asphalt so
       exposed road gaps between blocks show asphalt through).
    3. One material-bound pavement mesh per downtown block interior at z=0.02.
       This replaces tens of thousands of referenced concrete tile assets.
    4. Four material-bound sidewalk strip meshes per block, at the original
       sidewalk tile's measured walking-surface height.
    5. Lane-marking dashes at z=0.02 per road corridor: a yellow double
       centre-line (opposing-direction boundary) and white dashes at each
       same-direction lane boundary.
    6. Placeholder filler buildings (*parent_path/placeholders*): mild-colored
       box meshes filling block gaps no library building fits
       (``layout["placeholder_buildings"]``, computed by :func:`build_city`).

    *layout* must contain ``region``, ``blocks``, and ``road_corridors`` as
    returned by :func:`build_city`.
    """
    rx0, ry0, rx1, ry1 = layout["region"]
    blocks         = layout["blocks"]
    road_corridors = layout["road_corridors"]

    roads_cfg  = config.get("roads", {})
    lane_w_m   = float(roads_cfg.get("lane_width_m", 3.5))
    uv_asphalt = float(roads_cfg.get("asphalt_uv_scale_m", 4.0))
    uv_grass   = float(roads_cfg.get("grass_uv_scale_m", 3.0))
    uv_concrete = float(roads_cfg.get("concrete_uv_scale_m", 4.0))
    uv_sidewalk = float(roads_cfg.get("sidewalk_uv_scale_m", 4.0))
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
    concrete_mat = _load_mat("concrete")
    sidewalk_mat = _load_mat("sidewalk") or concrete_mat

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

    # 3) Downtown block paving.  The grass remains as a harmless underlay;
    # parks simply have no entry in paved_blocks and therefore stay grass.
    paved_blocks = layout.get("paved_blocks") or []
    for idx, (px0, py0, px1, py1) in enumerate(paved_blocks):
        _make_plane_mesh(stage, f"{gnd}/concrete_{idx}",
                         px0, py0, px1, py1, 0.02, uv_concrete, ssf,
                         display_color=(0.42, 0.42, 0.40),
                         mat_prim_path=concrete_mat)

    # 4) Sidewalk rings.  These retain the old slab's measured top elevation,
    # which is also the surface height used by the street-furniture placer.
    sidewalk_rects = layout.get("sidewalk_rects") or []
    sidewalk_z = float(layout.get("sidewalk_top_m", 0.02))
    for idx, (sx0, sy0, sx1, sy1) in enumerate(sidewalk_rects):
        _make_plane_mesh(stage, f"{gnd}/sidewalk_{idx}",
                         sx0, sy0, sx1, sy1, sidewalk_z, uv_sidewalk, ssf,
                         display_color=(0.48, 0.48, 0.46),
                         mat_prim_path=sidewalk_mat)

    # 5) Lane markings (z=0.02)
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
          f"{len(paved_blocks)} paved block meshes, "
          f"{len(sidewalk_rects)} sidewalk strip meshes, "
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
    bld_cfg = usds.get("buildings") or usds.get("houses") or {}
    house_intact    = _nl(bld_cfg["intact"])
    house_damaged   = _nl(bld_cfg.get("damaged") or [])
    house_destroyed = _nl(bld_cfg.get("destroyed") or [])
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

    pack_cfg    = config.get("packing", {})
    bld_gap_m   = float(pack_cfg.get("building_gap_m", 2.5))
    park_chance = float(pack_cfg.get("park_block_chance", 0.12))
    # Locale knobs: downtown paves a block wall-to-wall and builds out to the
    # sidewalk; a suburb leaves the block as lawn and sets houses back from it.
    pave_blocks = bool(pack_cfg.get("pave_blocks", True))
    setback_m   = float(pack_cfg.get("setback_m", 0.0))
    verge_m     = float(config.get("frontage", {}).get("verge_m", 0.0))
    drive_cfg   = config.get("driveways", {})

    # (block, inset, building_rect) per placed building — the lot context the
    # driveway pass needs to run a drive from the house out to the street.
    block_of_building: list = []
    ph_cfg      = pack_cfg.get("placeholders", {})
    ph_enabled  = bool(ph_cfg.get("enabled", True))
    ph_min_m    = float(ph_cfg.get("min_footprint_m", 5.0))
    ph_max_m    = float(ph_cfg.get("max_footprint_m", 20.0))
    ph_h_range  = ph_cfg.get("height_m", [8.0, 30.0])

    parks_cfg          = config.get("parks", {})
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

    roads_cfg = config.get("roads", {})
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
        """True with probability *fraction* scaled by the local damage."""
        return fraction > 0.0 and rng.random() < fraction * damage_at(x, y)

    def _hit_count(x, y, count_range):
        """Random count from [lo, hi], scaled down by the local damage."""
        k = damage_at(x, y)
        if k <= 0.0:
            return 0
        return int(round(rng.randint(int(count_range[0]),
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
    intact_weight = max(0.0, 1.0 - (damaged_fraction + destroyed_fraction)
                        * damage_at.lo)
    eligible_houses = (
        (house_intact if intact_weight > 0.0 else [])
        + (house_damaged if damaged_fraction * damage_at.hi > 0.0 else [])
        + (house_destroyed if destroyed_fraction * damage_at.hi > 0.0 else []))

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
    # That guarantee must not override the disaster, though: an oversized
    # *intact* building anchored into a block sitting dead-center in a
    # tornado's track — where destroyed_fraction*damage_at is ~1 — would
    # stand there untouched no matter how severe the disaster, because
    # anchoring picks the building before any fate is ever rolled. So a
    # candidate block is only eligible for a given building if that block's
    # local damage intensity would plausibly produce *that* fate: an intact
    # building needs a block where destruction/damage isn't the likely
    # outcome, a destroyed one needs a block where destruction plausibly is.
    # Damaged sits in between and is left unconstrained — the least jarring
    # of the three to anchor somewhere its fate wasn't the most likely roll.
    def _anchor_ok(u, cx_, cy_):
        k = damage_at(cx_, cy_)
        d_here, m_here = destroyed_fraction * k, damaged_fraction * k
        if u in destroyed_set:
            return d_here >= 0.5
        if u in damaged_set:
            return True
        return (d_here + m_here) < 0.5   # intact

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
            if not _anchor_ok(u, (ix0 + ix1) / 2.0, (iy0 + iy1) / 2.0):
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
    debris_rules   = dis_cfg.get("debris", {})
    piles_per      = debris_rules.get("piles_per_building", [2, 4])
    pile_max_off   = float(debris_rules.get("pile_max_offset_m", 3.0))
    pieces_per     = debris_rules.get("pieces_per_building", [10, 20])
    pieces_scatter = float(debris_rules.get("pieces_scatter_m", 6.0))
    tilt_chance    = float(debris_rules.get("tilt_chance", 0.35))
    tilt_deg       = debris_rules.get("tilt_deg", [2.0, 6.0])
    sink_m         = debris_rules.get("sink_m", [0.4, 1.2])
    lean_piles     = debris_rules.get("lean_piles", [2, 3])

    house_rects: list = []      # building footprint rects (for tree avoidance)
    placeholder_bldgs: list = []
    city_layout["placeholder_buildings"] = placeholder_bldgs
    park_blocks: list = []      # (block, inset) rects of blocks kept as parks
    paved_blocks: list = []     # block interiors rendered as procedural paving
    sidewalk_rects: list = []   # four procedural strips around each block
    residential_blocks: list = []  # (block, inset) of packed blocks — lawn when
                                   # pave_blocks is off (suburban/rural)
    n_buildings = n_damaged = n_destroyed = n_parks = 0

    # Eligible pools pack directly with their own measured footprints; the
    # fate weights (destroyed_fraction / damaged_fraction) drive the *choice*
    # of pool at each packing pick via _pick_building below. Zero-weight
    # pools are excluded from the candidates entirely, so e.g. an untouched
    # city can never fall back onto a ruin (nor "total" onto an intact one).
    pack_candidates = [(u, house_fps[u]["sx"], house_fps[u]["sy"])
                       for u in eligible_houses]

    def _pick_building(fits, sx_, sy_):
        """Fate-weighted pick among fitting candidates: destroyed_fraction /
        damaged_fraction / rest-intact, each scaled by the damage intensity
        at the slot anchor (*sx_*, *sy_*) so ruins cluster where the disaster
        actually hit.

        When the rolled pool has no candidate sized for this slot, falls back
        to the *next closest* fate rather than jumping straight to whatever
        fits — a destroyed roll degrades to damaged before ever landing on
        intact, so a severe disaster can't produce an untouched building just
        because no ruin asset happens to be sized for that particular lot.
        """
        k = damage_at(sx_, sy_)
        d_frac = destroyed_fraction * k
        m_frac = damaged_fraction * k
        destroyed_fits = [f for f in fits if f[0] in destroyed_set]
        damaged_fits   = [f for f in fits if f[0] in damaged_set]
        intact_fits    = [f for f in fits
                          if f[0] not in destroyed_set and f[0] not in damaged_set]
        r_fate = rng.random()
        if r_fate < d_frac:
            order = (destroyed_fits, damaged_fits, intact_fits)
        elif r_fate < d_frac + m_frac:
            order = (damaged_fits, destroyed_fits, intact_fits)
        else:
            order = (intact_fits, damaged_fits, destroyed_fits)
        for pool_fits in order:
            if pool_fits:
                return rng.choice(pool_fits)
        return rng.choice(fits)   # unreachable: fits is never empty here

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
                sidewalk_rects.extend([
                    (vx0, vy0, vx1, vy0 + sw_w),
                    (vx0, vy1 - sw_w, vx1, vy1),
                    (vx0, vy0 + sw_w, vx0 + sw_w, vy1 - sw_w),
                    (vx1 - sw_w, vy0 + sw_w, vx1, vy1 - sw_w),
                ])
                inset = (vx0 + sw_w, vy0 + sw_w, vx1 - sw_w, vy1 - sw_w)

        # Park blocks stay grassy — this is where trees, plants, rocks and
        # humans end up. Trails + park furniture are laid in a dedicated pass
        # afterwards. Selection happens up front (park_set) so
        # min_parks/max_parks can be honored.
        if blk in park_set:
            n_parks += 1
            park_blocks.append((blk, inset))
            continue

        # Downtown: pave the block interior wall-to-wall, so buildings and
        # filler prisms sit on pavement built out to the sidewalk. Suburb
        # (pave_blocks false): the block stays lawn, and only the ground each
        # building and driveway actually covers is hard surface.
        if concrete_fp is not None and pave_blocks:
            paved_blocks.append(inset)

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
            bw, bh = brect[2] - brect[0], brect[3] - brect[1]
            is_destroyed = usd in destroyed_set
            # No pre-damaged pool: fall back to tilting + sinking an intact
            # building as a damaged stand-in with probability damaged_fraction.
            tilt_standin = (not house_damaged and not is_destroyed
                            and _hit(cx, cy, damaged_fraction))
            is_damaged = usd in damaged_set or tilt_standin

            fp = house_fps[usd]
            # Y-up assets (e.g. Unity exports) need a +90° roll to stand upright
            # in Isaac Sim's Z-up world. This is combined with any damage tilt.
            axis_roll = 90.0 if _au(usd) == "Y" else 0.0
            roll = axis_roll
            pitch = 0.0
            z = fp["base"]
            yaw = house_front_yaw + rot
            if tilt_standin:
                roll = axis_roll + rng.uniform(-18.0, 18.0)
                pitch = rng.uniform(-18.0, 18.0)
                z -= rng.uniform(0.3, 0.9)

            # Some destroyed ruins additionally lean: a slight tilt + sink,
            # with rubble piled at the base of the side now overhanging.
            lean_dir = None
            if is_destroyed and _hit(cx, cy, tilt_chance):
                t_roll = rng.choice([-1.0, 1.0]) * rng.uniform(
                    float(tilt_deg[0]), float(tilt_deg[1]))
                t_pitch = rng.choice([-1.0, 1.0]) * rng.uniform(
                    float(tilt_deg[0]), float(tilt_deg[1]))
                roll += t_roll
                pitch += t_pitch
                z -= rng.uniform(float(sink_m[0]), float(sink_m[1]))
                # World-XY direction the top now overhangs: +pitch (about Y)
                # tips the top toward +X, +roll (about X) toward -Y — both in
                # the pre-yaw frame, so rotate by the building's yaw.
                lx_ = math.sin(math.radians(t_pitch))
                ly_ = -math.sin(math.radians(t_roll))
                yr_ = math.radians(yaw)
                wx_ = lx_ * math.cos(yr_) - ly_ * math.sin(yr_)
                wy_ = lx_ * math.sin(yr_) + ly_ * math.cos(yr_)
                n_ = math.hypot(wx_, wy_)
                if n_ > 1e-9:
                    lean_dir = (wx_ / n_, wy_ / n_)

            if is_damaged:
                n_damaged += 1
            add(usd, cx, cy, z, yaw, "house", _sc(usd),
                roll=roll, pitch=pitch, axis_up=_au(usd))
            house_rects.append(brect)
            n_buildings += 1

            # Destroyed building -> debris field. Rubble piles hug the ruin
            # (overlap with the building, sidewalk or road is fine); smaller
            # pieces land a bit farther and are physics-settled, so they come
            # to rest against whatever they hit.
            if is_destroyed:
                n_destroyed += 1
                hw_, hh_ = bw / 2.0, bh / 2.0

                def _edge_r(ca, sa):
                    """Center-to-footprint-edge distance along (ca, sa)."""
                    return min(hw_ / max(abs(ca), 1e-6),
                               hh_ / max(abs(sa), 1e-6))

                def _ring_pos(extra_lo, extra_hi, ang=None):
                    """Random point offset [extra_lo, extra_hi] m beyond the
                    footprint edge — any direction, or around *ang*."""
                    if ang is None:
                        ang = rng.uniform(0.0, 2.0 * math.pi)
                    ca, sa = math.cos(ang), math.sin(ang)
                    r_ = _edge_r(ca, sa) + rng.uniform(extra_lo, extra_hi)
                    return cx + ca * r_, cy + sa * r_

                def _add_pile(dx_, dy_):
                    if exclusions and _in_exclusion(dx_, dy_, exclusions):
                        return
                    du = rng.choice(debris_pile_usds)
                    dfp = resolver.get(du, "debris_pile", scale=_sc(du),
                                       axis_up=_au(du))
                    add(du, dx_, dy_, dfp["base"] + 0.02,
                        rng.uniform(0.0, 360.0), "debris_pile",
                        _sc(du) * rng.uniform(0.8, 1.2),
                        roll=_axis_roll(du), axis_up=_au(du))

                if debris_pile_usds:
                    for _ in range(_hit_count(cx, cy, piles_per)):
                        _add_pile(*_ring_pos(-1.0, pile_max_off))
                    # Leaning ruin: rubble hugging the base of the caved /
                    # overhanging side, spread along that edge.
                    if lean_dir is not None:
                        base_ang = math.atan2(lean_dir[1], lean_dir[0])
                        for _ in range(rng.randint(int(lean_piles[0]),
                                                   int(lean_piles[1]))):
                            ang = base_ang + rng.uniform(-0.5, 0.5)
                            _add_pile(*_ring_pos(-1.5, 1.0, ang=ang))

                if debris_piece_usds:
                    for _ in range(_hit_count(cx, cy, pieces_per)):
                        dx_, dy_ = _ring_pos(0.3, pieces_scatter)
                        if exclusions and _in_exclusion(dx_, dy_, exclusions):
                            continue
                        du = rng.choice(debris_piece_usds)
                        dfp = resolver.get(du, "debris", scale=_sc(du),
                                           axis_up=_au(du))
                        # Dropped from slightly above with a random heading;
                        # the settle pass finds the resting pose.
                        add(du, dx_, dy_, dfp["base"] + 0.4,
                            rng.uniform(0.0, 360.0), "debris",
                            _sc(du) * rng.uniform(0.7, 1.2),
                            roll=_axis_roll(du), axis_up=_au(du), settle=True)

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
    # at the kerb; downtown leaves this off (driveways.chance 0).
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
    curb_margin = float(config.get("frontage", {}).get("curb_margin_m", 0.3))

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
        add(usd, x, y, z0 + max(fp["sx"], fp["sy"]) / 2.0, rng.uniform(0.0, 360.0),
            category, _sc(usd),
            roll=_axis_roll(usd) + rng.choice([-1.0, 1.0]) * rng.uniform(80.0, 100.0),
            axis_up=_au(usd), pose=pose, settle=True)

    park_block_set = {b for (b, _i) in park_blocks}
    park_insets = [inset for (_b, inset) in park_blocks]

    # Residential lawns: the un-paved part of a packed block. Downtown paves
    # its blocks (pave_blocks), so there is no lawn and these stay empty;
    # a suburb grows trees and shrubs here, which is what makes it read as
    # a suburb rather than a downtown of small buildings.
    lawn_insets = ([inset for (_b, inset) in residential_blocks]
                   if not pave_blocks else [])

    # ---- Trees: parks get bare-ground trees, densely and evenly spread;
    # suburban lawns get them at their own density. Downtown street trees are
    # potted in tree-tagged planters along the frontage instead — see the
    # planter passes in the street-furniture section below.
    if tree_usds and (park_insets or lawn_insets):
        tcfg = config.get("trees", {})
        # Wind takes trees down with the buildings — in tornado reference
        # imagery the treeline is the clearest marker of the track, snapped
        # bare inside it and untouched a few metres outside. Stumps are already
        # down, so they're excluded from the pool that can topple.
        trees_toppled = float(dcfg.get("trees_toppled_fraction", 0.0))
        stump_usds = set(_pool(tree_usds, "stump")) if tree_usds else set()

        def _place_tree(tx, ty):
            usd = rng.choice(tree_usds)
            jitter = rng.uniform(0.85, 1.2)
            tfp = resolver.get(usd, "tree", scale=_sc(usd))
            if usd not in stump_usds and _hit(tx, ty, trees_toppled):
                _topple_flat(usd, tfp, tx, ty, "tree", z0=grass_top)
                return
            add(usd, tx, ty, grass_top + tfp["base"],
                rng.uniform(0, 360), "tree", _sc(usd) * jitter)

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
        pcfg = config.get("plants", {})

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

    # ---- Bus stops: sparse along frontage, shelter opening toward the
    # street. Placed before the other street furniture so the shelters claim
    # their slots first — including the extra occupancy cells their length
    # covers, so lamps/cans don't spawn inside them.
    if busstop_usds:
        spacing = float(config.get("bus_stops", {}).get("spacing_m", 130.0))
        for px, py, street_yaw in _frontage_positions(spacing, phase=0.85,
                                                      jitter_frac=0.3):
            u = rng.choice(busstop_usds)
            fp = resolver.get(u, "bus_stop", scale=_sc(u), axis_up=_au(u))
            bx, by = _inset_pos(px, py, street_yaw, _curb_inset(fp))
            if not _occ_reserve(_prop_curb_rect(bx, by, street_yaw, fp)):
                continue
            add(u, bx, by, sidewalk_top + fp["base"], street_yaw, "bus_stop",
                _sc(u), roll=_axis_roll(u), axis_up=_au(u))
            # Claim the curb cells the shelter spans in the occupancy grid.
            trad = math.radians(street_yaw + 90.0)
            half_len = max(fp["sx"], fp["sy"]) / 2.0
            for k in range(-int(half_len / 2.0) - 1, int(half_len / 2.0) + 2):
                cx_ = px + math.cos(trad) * k * 2.0
                cy_ = py + math.sin(trad) * k * 2.0
                frontage_seen.add((round(cx_ / 2.0), round(cy_ / 2.0)))

    # ---- Streetlights: uniform spacing along frontage, lamp arm reaching
    # over the roadway (street rule: light the street, not the block).
    if light_usds:
        spacing = float(config.get("streetlights", {}).get("spacing_m", 20.0))
        lights_toppled = float(dcfg.get("streetlights_toppled_fraction", 0.0))
        for px, py, street_yaw in _frontage_positions(spacing):
            lu = rng.choice(street_light_usds)
            lfp = resolver.get(lu, "streetlight", scale=_sc(lu))
            lx, ly = _inset_pos(px, py, street_yaw, _curb_inset(lfp))
            if not _occ_reserve(_prop_curb_rect(lx, ly, street_yaw, lfp,
                                                pole_only=True)):
                continue
            if _hit(lx, ly, lights_toppled):
                _topple_flat(lu, lfp, lx, ly, "streetlight", z0=sidewalk_top)
            else:
                add(lu, lx, ly, sidewalk_top + lfp["base"], street_yaw,
                    "streetlight", _sc(lu))

    # ---- Benches: along frontage, seat facing the street.
    if bench_usds:
        spacing = float(config.get("benches", {}).get("spacing_m", 30.0))
        for px, py, street_yaw in _frontage_positions(spacing, phase=0.25,
                                                      jitter_frac=0.3):
            u = rng.choice(street_bench_usds)
            fp = resolver.get(u, "bench", scale=_sc(u), axis_up=_au(u))
            bx, by = _inset_pos(px, py, street_yaw, _curb_inset(fp))
            if not _occ_reserve(_prop_curb_rect(bx, by, street_yaw, fp)):
                continue
            add(u, bx, by, sidewalk_top + fp["base"], street_yaw, "bench", _sc(u),
                roll=_axis_roll(u), axis_up=_au(u))

    # ---- Planters: fully on the sidewalk, close to the curb, long axis
    # along the street. Tall planters carry their greenery at soil level —
    # a fraction of the planter's height — never at the sidewalk itself.
    planters_cfg = config.get("planters", {})
    soil_frac = float(planters_cfg.get("soil_level_frac", 0.75))

    def _add_planter(px, py, street_yaw, pu):
        """Place planter *pu* inset from the curb; returns (x, y, soil_z,
        along_x, along_y, usable_len) for whatever goes inside it, or None
        when the spot's bbox is already occupied. The greenery placed inside
        deliberately skips occupancy — it shares the planter's footprint."""
        pfp = resolver.get(pu, "planter", scale=_sc(pu), axis_up=_au(pu))
        qx, qy = _inset_pos(px, py, street_yaw, _curb_inset(pfp))
        if not _occ_reserve(_prop_curb_rect(qx, qy, street_yaw, pfp)):
            return None
        # Long axis parallel to the curb (art long axis assumed X).
        pyaw = street_yaw + (90.0 if pfp["sx"] >= pfp["sy"] else 0.0)
        add(pu, qx, qy, sidewalk_top + pfp["base"], pyaw, "planter", _sc(pu),
            roll=_axis_roll(pu), axis_up=_au(pu))
        trad = math.radians(street_yaw + 90.0)
        return (qx, qy, sidewalk_top + pfp["sz"] * soil_frac,
                math.cos(trad), math.sin(trad), max(pfp["sx"], pfp["sy"]))

    # Street trees: one per tree-tagged planter.
    if tree_planter_usds and street_tree_usds:
        spacing = float(planters_cfg.get("tree_spacing_m", 30.0))
        for px, py, street_yaw in _frontage_positions(spacing, phase=0.7,
                                                      jitter_frac=0.2):
            planted = _add_planter(px, py, street_yaw,
                                   rng.choice(tree_planter_usds))
            if planted is None:
                continue
            qx, qy, soil_z, _ax, _ay, _ln = planted
            tu_ = rng.choice(street_tree_usds)
            tfp_ = resolver.get(tu_, "tree", scale=_sc(tu_))
            add(tu_, qx, qy, soil_z + tfp_["base"], rng.uniform(0.0, 360.0),
                "tree", _sc(tu_) * rng.uniform(0.9, 1.1))

    # Plant boxes: 1-3 plant clumps per planter, spread along its length.
    if plant_planter_usds and plant_usds:
        spacing = float(planters_cfg.get("plant_spacing_m", 40.0))
        slot_m = float(planters_cfg.get("plant_slot_m", 0.7))
        for px, py, street_yaw in _frontage_positions(spacing, phase=0.15,
                                                      jitter_frac=0.25):
            planted = _add_planter(px, py, street_yaw,
                                   rng.choice(plant_planter_usds))
            if planted is None:
                continue
            qx, qy, soil_z, ax_, ay_, ln_ = planted
            n = max(1, min(3, int(ln_ / slot_m)))
            for k in range(n):
                off = (k - (n - 1) / 2.0) * (ln_ * 0.7 / max(n, 1))
                pl = rng.choice(plant_usds)
                plfp = resolver.get(pl, "plant", scale=_sc(pl), axis_up=_au(pl))
                add(pl, qx + ax_ * off, qy + ay_ * off, soil_z + plfp["base"],
                    rng.uniform(0.0, 360.0), "plant",
                    _sc(pl) * rng.uniform(0.8, 1.0),
                    roll=_axis_roll(pl), axis_up=_au(pl))

    # ---- Fire hydrants: sparse along frontage, outlet toward the street.
    # Bolted to the water main, so a tornado rarely moves them — only a
    # small sheared-off-and-flat fraction.
    if hydrant_usds:
        spacing = float(config.get("fire_hydrants", {}).get("spacing_m", 55.0))
        sheared = float(dcfg.get("fire_hydrants_toppled_fraction", 0.0))
        for px, py, street_yaw in _frontage_positions(spacing, phase=0.5,
                                                      jitter_frac=0.2):
            u = rng.choice(hydrant_usds)
            fp = resolver.get(u, "fire_hydrant", scale=_sc(u), axis_up=_au(u))
            hx, hy = _inset_pos(px, py, street_yaw, _curb_inset(fp))
            if not _occ_reserve(_prop_curb_rect(hx, hy, street_yaw, fp)):
                continue
            if _hit(hx, hy, sheared):
                _topple_flat(u, fp, hx, hy, "fire_hydrant", z0=sidewalk_top)
            else:
                add(u, hx, hy, sidewalk_top + fp["base"], street_yaw,
                    "fire_hydrant", _sc(u), roll=_axis_roll(u), axis_up=_au(u))

    # ---- Trash cans: along frontage. Light and unanchored, so tipped ones
    # are also wind-blown up to trash_cans_scatter_m from where they stood.
    if trash_usds:
        spacing = float(config.get("trash_cans", {}).get("spacing_m", 25.0))
        tipped = float(dcfg.get("trash_cans_toppled_fraction", 0.0))
        scatter = float(dcfg.get("trash_cans_scatter_m", 0.0))
        for px, py, street_yaw in _frontage_positions(spacing, phase=0.5,
                                                      jitter_frac=0.35):
            u = rng.choice(street_trash_usds)
            fp = resolver.get(u, "trash_can", scale=_sc(u), axis_up=_au(u))
            tx_, ty_ = _inset_pos(px, py, street_yaw, _curb_inset(fp))
            if not _occ_reserve(_prop_curb_rect(tx_, ty_, street_yaw, fp)):
                continue
            if _hit(tx_, ty_, tipped):
                # Wind displacement also scales with the local intensity —
                # cans on the fringe tip over, cans in the core fly.
                ang = rng.uniform(0.0, 2.0 * math.pi)
                d = rng.uniform(0.0, scatter * damage_at(tx_, ty_))
                bx_, by_ = tx_ + d * math.cos(ang), ty_ + d * math.sin(ang)
                if exclusions and _in_exclusion(bx_, by_, exclusions):
                    bx_, by_ = tx_, ty_     # don't blow into a keep-out zone
                _topple_flat(u, fp, bx_, by_, "trash_can", z0=sidewalk_top)
            else:
                add(u, tx_, ty_, sidewalk_top + fp["base"], rng.uniform(0.0, 360.0),
                    "trash_can", _sc(u), roll=_axis_roll(u), axis_up=_au(u))

    # ---- Traffic lights: one per road intersection (probabilistic), on a
    # random sidewalk corner, yawed to face the intersection center (street
    # rule: signal head hangs over/toward the junction it controls).
    # Disaster: snapped flat at the base, or left standing but bent/leaning.
    if tlight_usds:
        tlcfg = config.get("traffic_lights", {})
        tl_chance = float(tlcfg.get("intersection_chance", 1.0))
        corner_margin = float(tlcfg.get("corner_margin_m", 1.0))
        tl_front = float(orient.get("traffic_light_front", 0.0))
        tl_toppled = float(dcfg.get("traffic_lights_toppled_fraction", 0.0))
        tl_leaning = float(dcfg.get("traffic_lights_leaning_fraction", 0.0))
        lean_lo, lean_hi = dcfg.get("traffic_lights_lean_deg", [8.0, 30.0])
        # Find intersections. BSP corridors *abut* at T-junctions (a child
        # road ends exactly where the crossing road begins) so a pure
        # rect-overlap test finds nothing — treat touching-or-crossing
        # corridor pairs as intersections instead. The junction zone is the
        # NS road's x-band × the EW road's y-band.
        eps = 0.25
        ns_corridors = [c for c in road_corridors if c["dir"] == "ns"]
        ew_corridors = [c for c in road_corridors if c["dir"] == "ew"]
        for nsc in ns_corridors:
            for ewc in ew_corridors:
                if (ewc["x0"] > nsc["x1"] + eps or ewc["x1"] < nsc["x0"] - eps or
                        nsc["y0"] > ewc["y1"] + eps or nsc["y1"] < ewc["y0"] - eps):
                    continue
                ox0, ox1 = nsc["x0"], nsc["x1"]
                oy0, oy1 = ewc["y0"], ewc["y1"]
                if rng.random() >= tl_chance:
                    continue
                icx, icy = (ox0 + ox1) / 2.0, (oy0 + oy1) / 2.0
                u = rng.choice(tlight_usds)
                fp = resolver.get(u, "traffic_light", scale=_sc(u), axis_up=_au(u))
                # Pole on the sidewalk, corner_margin past the road corner.
                # At a T-junction some corners fall inside the through road —
                # try corners in random order and take the first on land whose
                # pole spot isn't already occupied by street furniture.
                hw = (ox1 - ox0) / 2.0 + corner_margin
                hh = (oy1 - oy0) / 2.0 + corner_margin
                corners = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
                rng.shuffle(corners)
                x = y = None
                for dx_, dy_ in corners:
                    cx_, cy_ = icx + dx_ * hw, icy + dy_ * hh
                    if any(c["x0"] < cx_ < c["x1"] and c["y0"] < cy_ < c["y1"]
                           for c in road_corridors):
                        continue
                    if not _occ_reserve(_prop_curb_rect(cx_, cy_, 0.0, fp,
                                                        pole_only=True)):
                        continue
                    x, y = cx_, cy_
                    break
                if x is None:
                    continue
                if exclusions and _in_exclusion(x, y, exclusions):
                    continue
                yaw = math.degrees(math.atan2(icy - y, icx - x)) + tl_front
                if _hit(x, y, tl_toppled):
                    _topple_flat(u, fp, x, y, "traffic_light", z0=sidewalk_top)
                elif _hit(x, y, tl_leaning):
                    tilt = rng.uniform(float(lean_lo), float(lean_hi))
                    add(u, x, y, sidewalk_top + fp["base"], yaw, "traffic_light",
                        _sc(u),
                        roll=_axis_roll(u) + rng.choice([-1.0, 1.0]) * tilt,
                        pitch=rng.choice([-1.0, 1.0]) * tilt * rng.uniform(0.0, 0.5),
                        axis_up=_au(u))
                else:
                    add(u, x, y, sidewalk_top + fp["base"], yaw, "traffic_light",
                        _sc(u), roll=_axis_roll(u), axis_up=_au(u))

    # ---- Humans. Pedestrians walk the sidewalks (heading along the street),
    # walkers follow the park trails, idlers stand on open grass. All are
    # posed via their UsdSkel rigging (walk / idle — see _HUMAN_POSES) so
    # they don't render in their authored T-pose.
    # Disaster: a prone fraction lies flat where they stood (face-down/up,
    # posed arms-down first so they read as casualties, not crosses), and
    # humans_strewn extra casualties are thrown onto the road asphalt.
    if human_usds:
        hcfg = config.get("humans", {})
        prone = float(dcfg.get("humans_prone_fraction", 0.0))

        def _human_down(u, fp, x, y, z0):
            """Casualty flat on the ground: face-down/up (±90° pitch about
            the body's facing axis via roll on the pre-yaw frame), lifted by
            half the body depth (sy — arms-down pose, not the T-pose span)."""
            add(u, x, y, z0 + fp["sy"] / 2.0, rng.uniform(0.0, 360.0),
                "human", _sc(u),
                roll=_axis_roll(u) + rng.choice([-1.0, 1.0]) * rng.uniform(82.0, 98.0),
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
        want = rng.randint(int(strewn_range[0]), int(strewn_range[1]))
        n_strewn = 0
        for _ in range(want * 10):
            if n_strewn >= want or not road_corridors:
                break
            corr = rng.choice(road_corridors)
            hx = rng.uniform(corr["x0"], corr["x1"])
            hy = rng.uniform(corr["y0"], corr["y1"])
            if exclusions and _in_exclusion(hx, hy, exclusions):
                continue
            # Keep casualties inside the damage zone: accept the sampled
            # spot in proportion to how hard it was hit.
            if rng.random() >= damage_at(hx, hy):
                continue
            u = rng.choice(human_usds)
            fp = resolver.get(u, "human", scale=_sc(u), axis_up=_au(u))
            _human_down(u, fp, hx, hy, 0.0)
            n_strewn += 1

    # =======================================================================
    # STEP 4 — CARS (right-hand traffic along road corridors) + STREWN WRECKS
    # =======================================================================
    if car_usds:
        ccfg = config.get("cars", {})
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
            if rng.random() < 0.5:
                roll, z = rng.choice([-1.0, 1.0]) * rng.uniform(80.0, 100.0), fp["sy"] / 2.0
            else:
                roll, z = 180.0 + rng.uniform(-12.0, 12.0), fp["sz"]
            add(usd, x, y, z, rng.uniform(0.0, 360.0), "car", _sc(usd),
                roll=_axis_roll(usd) + roll, pitch=rng.uniform(-8.0, 8.0),
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
                    if _hit(cx, cy, toppled_frac):
                        _add_toppled_car(usd, fp, cx, cy)
                    else:
                        add(usd, cx, cy, fp["base"],
                            yaw + car_front + rng.uniform(-car_yaw_jit, car_yaw_jit),
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
                    if _hit(cx, cy, toppled_frac):
                        _add_toppled_car(usd, fp, cx, cy)
                    else:
                        add(usd, cx, cy, fp["base"],
                            yaw + car_front + rng.uniform(-car_yaw_jit, car_yaw_jit),
                            "car", _sc(usd), roll=_axis_roll(usd), axis_up=_au(usd))

        # ---- Strewn wrecks: tornado-tossed cars anywhere in the region.
        strewn_range = dcfg.get("cars_strewn", [0, 0])
        strewn_topple = float(dcfg.get("strewn_topple_fraction", 0.7))
        strewn_margin = float(dcfg.get("strewn_margin_m", 1.5))
        want = rng.randint(int(strewn_range[0]), int(strewn_range[1]))
        half_w, half_h = region_w_m / 2.0, region_h_m / 2.0
        strewn_pts: list = []
        for _ in range(want * 15):
            if len(strewn_pts) >= want:
                break
            x, y = rng.uniform(-half_w, half_w), rng.uniform(-half_h, half_h)
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
            if rng.random() >= damage_at(x, y):
                continue
            usd = rng.choice(car_usds)
            fp = _car_fp(usd)
            if rng.random() < strewn_topple:
                _add_toppled_car(usd, fp, x, y)
            else:
                # Upright but tossed onto uneven ground — settle it too.
                add(usd, x, y, fp["base"], rng.uniform(0.0, 360.0), "car",
                    _sc(usd), roll=_axis_roll(usd), axis_up=_au(usd),
                    settle=True)
            strewn_pts.append((x, y))

    # --- Path scour: debris strewn along the damage corridor itself ---------
    # Building debris (above) hangs off each ruin, so it inherits the *layout* —
    # it clusters on lots and stops at the property line. A tornado track
    # doesn't: it lays a continuous band of dirt, splintered wood and scraped
    # ground straight across lawns, streets and open fields alike, and that
    # unbroken line is the single most recognisable thing about the aftermath.
    # This pass fills it in, sampling the whole region and keeping points in
    # proportion to the local damage field — so the band appears wherever the
    # disaster is intense, tapers at its edges, and is absent entirely for a
    # `field` that never gets there.
    path_pieces = float(debris_rules.get("path_pieces_per_100m2", 0.0))
    path_piles = float(debris_rules.get("path_piles_per_100m2", 0.0))
    path_min = float(debris_rules.get("path_min_intensity", 0.25))
    path_shape = float(debris_rules.get("path_density_shape", 1.6))
    if (path_pieces > 0.0 or path_piles > 0.0) and (debris_piece_usds or debris_pile_usds):
        rx0_, ry0_ = -region_w_m / 2.0, -region_h_m / 2.0
        rx1_, ry1_ = region_w_m / 2.0, region_h_m / 2.0
        # Densities are per 100 m² of *affected ground*, not of the region, so
        # the same number means the same thing whether the field is a narrow
        # tornado corridor or a hurricane's region-wide blanket. Integrating
        # the field gives the effective affected area.
        _n = 32
        _mean_k = sum(damage_at(rx0_ + (rx1_ - rx0_) * (i + 0.5) / _n,
                                ry0_ + (ry1_ - ry0_) * (j + 0.5) / _n)
                      for i in range(_n) for j in range(_n)) / (_n * _n)
        area_100 = (region_w_m * region_h_m) / 100.0 * _mean_k

        # Density gradient, peaked on the track centerline / epicenter and
        # tapering toward the edge — see make_scour_density's docstring for
        # why this has to be a separate function from damage_at (which stays
        # a flat plateau, correctly, for building/topple placement).
        scour_density = make_scour_density(
            dis_cfg.get("field"), (rx0_, ry0_, rx1_, ry1_), shape=path_shape)

        def _scour_pt():
            """A point on the track, or None. `damage_at` gates the extent
            (same width/threshold the rest of the disaster uses); acceptance
            is then weighted by the peaked density gradient, not the flat
            field value, so the strip reads as densest at its centerline."""
            for _ in range(12):
                x = rng.uniform(rx0_, rx1_)
                y = rng.uniform(ry0_, ry1_)
                if damage_at(x, y) < path_min:
                    continue
                if rng.random() > scour_density(x, y):
                    continue
                # Keep the ground clear inside buildings that are still
                # standing; everything else — road, sidewalk, lawn, field —
                # is fair game, because the real thing covers all of it.
                if any(_in_rect(x, y, r) for r in house_rects):
                    continue
                return x, y
            return None

        n_scour_pieces = n_scour_piles = 0
        if debris_piece_usds and path_pieces > 0.0:
            for _ in range(int(round(path_pieces * area_100))):
                pt = _scour_pt()
                if pt is None:
                    continue
                du = rng.choice(debris_piece_usds)
                dfp = resolver.get(du, "debris", scale=_sc(du), axis_up=_au(du))
                add(du, pt[0], pt[1], dfp["base"] + 0.4,
                    rng.uniform(0.0, 360.0), "debris",
                    _sc(du) * rng.uniform(0.7, 1.2),
                    roll=_axis_roll(du), axis_up=_au(du), settle=True)
                n_scour_pieces += 1

        if debris_pile_usds and path_piles > 0.0:
            for _ in range(int(round(path_piles * area_100))):
                pt = _scour_pt()
                if pt is None:
                    continue
                du = rng.choice(debris_pile_usds)
                dfp = resolver.get(du, "debris_pile", scale=_sc(du), axis_up=_au(du))
                add(du, pt[0], pt[1], dfp["base"] + 0.02,
                    rng.uniform(0.0, 360.0), "debris_pile",
                    _sc(du) * rng.uniform(0.8, 1.3),
                    roll=_axis_roll(du), axis_up=_au(du))
                n_scour_piles += 1

        if n_scour_pieces or n_scour_piles:
            print(f"[scene_gen] Path scour: {n_scour_pieces} pieces, "
                  f"{n_scour_piles} piles along the damage corridor")

    # --- Summary -----------------------------------------------------------
    counts: dict = {}
    for p in placements:
        counts[p["category"]] = counts.get(p["category"], 0) + 1
    print(f"[scene_gen] City region {region_w_m:.0f}×{region_h_m:.0f} m "
          f"({len(blocks)} blocks, {n_parks} parks, {len(road_corridors)} road corridors)")
    print(f"[scene_gen] Placements by category: {counts} "
          f"(total {len(placements)}, {n_buildings} buildings, {n_damaged} damaged, "
          f"{n_destroyed} destroyed, {len(placeholder_bldgs)} placeholder prisms)")
    city_layout["paved_blocks"] = paved_blocks
    city_layout["sidewalk_rects"] = sidewalk_rects
    city_layout["sidewalk_top_m"] = sidewalk_top
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
        # REST IS AN A-POSE, NOT A T-POSE, and the difference is 45 degrees of
        # shoulder. MEASURED on rp_carla: shoulder (0.158, 1.389), hand
        # (0.513, 1.035) — the arm already hangs 45 deg below horizontal, which
        # is Epic's A-posed mannequin reference. Read the hand against the
        # PELVIS instead of the shoulder and it looks like a T-pose; that
        # mistake produced a "+-78" correction that swings the hand 0.30 m
        # ACROSS the body with the wrist rising.
        # Swept 40..120 in 5s: +40 puts the hand 7 mm off plumb under the
        # shoulder, and every larger value carries it inboard.
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
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
    # ---- SURVIVOR POSES (disaster/people.py). ALL FIVE ARE VERIFY ON SIGHT:
    # they were derived from the rig's geometry, not seen. Each carries the
    # arithmetic that produced it so the next reader can correct rather than
    # re-guess.
    #
    # THE ONE RULE THE WHOLE BLOCK TURNS ON. A limb at rest points DOWN (-Z);
    # writing that direction as ``v(a) = (0, sin a, -cos a)`` makes an X-axis
    # delta of `a` degrees exactly "swing the limb through a about the hip".
    # Facing is -Y, so NEGATIVE a swings a limb FORWARD and positive a swings
    # it BACK — which is what the `walk` entry above already encodes
    # (thigh_l -20 = "left leg forward"). A delta on a CHILD stacks on top of
    # its parent's, because `_pose_joint_transforms` rebuilds each joint from
    # the parent's already-posed world frame: a calf delta of +79 on a thigh
    # at -102 leaves the calf at -23, i.e. 79 degrees of knee flexion.
    #
    # Segment lengths used below are the UE4 mannequin's at 1.80 m: thigh
    # 0.45 m, shin 0.42 m, ankle-to-sole 0.08 m, pelvis at 0.95 m. The pack is
    # 1.73-1.86 m, so `_POSE_Z_OFFSET` is applied SCALED BY MEASURED HEIGHT.

    # Sitting on the ground, legs out in front with a slight knee bend — the
    # posture people take when they have stopped and are waiting, which is
    # most of what a refuge area is. Hip 102 deg, knee 79 deg.
    #   knee  = pelvis + 0.45 * v(-102) = pelvis + (0, -0.440, +0.094)
    #   ankle = knee   + 0.42 * v(-23)  = pelvis + (0, -0.827, -0.070)
    # so the soles end 0.15 m under the pelvis and 0.83 m in front of it: the
    # seat and the heels are both on the ground, which is the check that makes
    # this pose self-consistent rather than merely plausible.
    "sit_ground": {
        # MEASURED off rp_carla (thigh 0.472, calf 0.364), not derived: -102/+79
        # was a CHAIR sit — the shin hung down, so the ankle finished 0.237 m
        # BELOW the hip and 0.60 m in front, and the z-offset that grounds the
        # seat then puts the feet 0.17 m under the asphalt. Legs-out sitting
        # needs the thigh horizontal and the knee nearly straight: the ankle
        # then lands 0.836 m forward and level with the hip, so seat and heels
        # reach the ground together, which is the check the old comment claimed
        # and the old numbers failed.
        "thigh_l": ((1.0, 0.0, 0.0), -90.0),
        "thigh_r": ((1.0, 0.0, 0.0), -90.0),
        "calf_l": ((1.0, 0.0, 0.0), 3.0),
        "calf_r": ((1.0, 0.0, 0.0), 3.0),
        "foot_l": ((1.0, 0.0, 0.0), 12.0),        # relax the ankle, toes up
        "foot_r": ((1.0, 0.0, 0.0), 12.0),
        "spine_02": ((1.0, 0.0, 0.0), 8.0),       # lean back onto the hands
        "upperarm_l": ((0.0, 1.0, 0.0), 52.0),    # arms down, slightly wider
        "upperarm_r": ((0.0, 1.0, 0.0), -52.0),   # than `idle` — propping
        "upperarm_l+": ((1.0, 0.0, 0.0), 22.0),   # ...and back behind the hip
        "upperarm_r+": ((1.0, 0.0, 0.0), 22.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 8.0),
        "lowerarm_r": ((0.0, 1.0, 0.0), -8.0),
    },
    # Sitting on an EDGE — a kerb, a pool coping, a low wall — legs hanging.
    # Hip 90 (thigh straight forward, level), knee 90 (shin straight down):
    #   knee  = pelvis + 0.45 * v(-90) = pelvis + (0, -0.45, 0)
    #   ankle = knee   + 0.42 * v(0)   = pelvis + (0, -0.45, -0.42)
    # The caller passes the SEAT SURFACE as z; `_POSE_Z_OFFSET` drops the
    # pelvis onto it and the soles then hang 0.50 m below it.
    "sit_edge": {
        "thigh_l": ((1.0, 0.0, 0.0), -90.0),
        "thigh_r": ((1.0, 0.0, 0.0), -90.0),
        "calf_l": ((1.0, 0.0, 0.0), 90.0),
        "calf_r": ((1.0, 0.0, 0.0), 90.0),
        "spine_02": ((1.0, 0.0, 0.0), 4.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 48.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -48.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 10.0),
        "lowerarm_r": ((0.0, 1.0, 0.0), -10.0),
    },
    # In a car seat. `sit_edge` with the thighs a couple of degrees ABOVE
    # level (a car seat pan is reclined, not a bench), the shins swept forward
    # to the pedals rather than plumb, and the torso leaning in toward the
    # wheel with the elbows carried forward. Knee flexion 78 rather than 90:
    # a driver's legs are extended, which is also what keeps the knees clear
    # of a low dashboard.
    "seated_car": {
        "thigh_l": ((1.0, 0.0, 0.0), -88.0),
        "thigh_r": ((1.0, 0.0, 0.0), -88.0),
        "calf_l": ((1.0, 0.0, 0.0), 78.0),
        "calf_r": ((1.0, 0.0, 0.0), 78.0),
        "spine_02": ((1.0, 0.0, 0.0), -10.0),     # slight forward lean
        "upperarm_l": ((0.0, 1.0, 0.0), 46.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -46.0),
        "upperarm_l+": ((1.0, 0.0, 0.0), -26.0),  # elbows forward: hands on
        "upperarm_r+": ((1.0, 0.0, 0.0), -26.0),  # the wheel
        "lowerarm_l": ((1.0, 0.0, 0.0), -44.0),
        "lowerarm_r": ((1.0, 0.0, 0.0), -44.0),
    },
    # A PASSENGER, rather than a driver: the same seated legs as
    # `seated_car`, with the arms hanging at the sides instead of carried
    # forward onto a wheel. Nobody in the passenger seat holds their arms out
    # in front of them, and the forward-elbow posture reads as reaching
    # through the windscreen when there is no wheel modelled to reach for —
    # which is what these assets have, since none of them models an interior
    # a hand could rest on. Arm angles are `sit_edge`'s.
    "seated_car_arms_down": {
        "thigh_l": ((1.0, 0.0, 0.0), -88.0),
        "thigh_r": ((1.0, 0.0, 0.0), -88.0),
        "calf_l": ((1.0, 0.0, 0.0), 78.0),
        "calf_r": ((1.0, 0.0, 0.0), 78.0),
        "spine_02": ((1.0, 0.0, 0.0), -6.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 48.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -48.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 10.0),
        "lowerarm_r": ((0.0, 1.0, 0.0), -10.0),
    },
    # THERE IS NO `wave`, AND THERE MUST NOT BE ONE. A raised-arm pose was
    # authored here on the argument that it is the most distinctive human
    # silhouette from altitude; in the rendered scene the arm reads as a
    # broken limb and the figure as a mannequin (reviewed 2026-08-26, cut on
    # sight). Every placement pass draws from idle / walk / the seated and
    # crouched postures below; `_bind_human_pose` REFUSES a pose name that is
    # not in this table rather than silently leaving the rig in its A-pose.
    # Deep crouch — down beside a car, or over somebody. Hip 100, knee 137:
    #   knee  = pelvis + 0.45 * v(-100) = pelvis + (0, -0.443, +0.078)
    #   ankle = knee   + 0.42 * v(+37)  = pelvis + (0, -0.190, -0.257)
    # The soles come up 0.61 m from their rest height, which is exactly the
    # `_POSE_Z_OFFSET` that puts them back on the ground; the pelvis then sits
    # 0.34 m up, a real squat.
    "crouch": {
        # -100/+137 raked the shin back ~50 deg, leaving the ankle only 0.25 m
        # forward of a knee 0.47 m forward — a squat nobody balances in. These
        # put the shin near vertical: knee 0.40 forward and 0.25 down, ankle
        # 0.08 forward and 0.42 down, so the sole is flat under the knee and
        # the hip rides at 0.57 m.
        "thigh_l": ((1.0, 0.0, 0.0), -58.0),
        "thigh_r": ((1.0, 0.0, 0.0), -58.0),
        "calf_l": ((1.0, 0.0, 0.0), 120.0),
        "calf_r": ((1.0, 0.0, 0.0), 120.0),
        # Cancel the shin's net rotation so the sole returns to plantigrade.
        "foot_l": ((1.0, 0.0, 0.0), -62.0),
        "foot_r": ((1.0, 0.0, 0.0), -62.0),
        "spine_02": ((1.0, 0.0, 0.0), -14.0),     # weight forward over the feet
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "upperarm_l+": ((1.0, 0.0, 0.0), -18.0),  # forearms forward over the
        "upperarm_r+": ((1.0, 0.0, 0.0), -18.0),  # knees
        "lowerarm_l": ((1.0, 0.0, 0.0), -50.0),
        "lowerarm_r": ((1.0, 0.0, 0.0), -50.0),
    },

    # ---- DISASTER-SURVIVOR POSES (disaster/tornado_people.py). ADDED 2026-08-27
    # after the first 100 m tornado render came back as "a pavement full of
    # commuters": the whole scene was idle / walk / crouch because those were
    # the only postures that existed. VERIFY ON SIGHT, all seven, and the note
    # on each says what to look at.
    #
    # SEGMENT LENGTHS ARE rp_carla's MEASURED ONES — thigh 0.472, calf 0.364,
    # pelvis 0.983, ankle 0.147 at a 1.731 m stature — the same numbers the
    # `sit_ground` correction above was solved on, because those are the ones
    # somebody actually read off a rig. The 1.80 m mannequin nominals in the
    # block header (0.45 / 0.42 / 0.95 / 0.08) are kept only to convert a solved
    # drop into this table's 1.80 m reference for `_POSE_Z_OFFSET`, by the ratio
    # 1.80 / 1.731 = 1.040.
    #
    # JOINTS THIS PACK IS NOT KNOWN TO HAVE. Everything above uses only joints
    # attested in the shipped poses (thigh/calf/foot, upperarm/lowerarm,
    # spine_02). The postures below also drive `spine_01`, `spine_03`,
    # `neck_01` and `head`, which are UE4-mannequin standard and so almost
    # certainly present in a 95-joint UE4-compatible skeleton — but nobody has
    # listed this pack's joints. `_pose_joint_transforms` IGNORES a delta whose
    # joint it cannot find, so a missing one degrades the pose (a flatter back,
    # a head that does not hang) instead of breaking it; `_bind_human_pose`
    # now prints one WARN per (asset, pose) naming exactly which deltas were
    # dropped, so this stops being a silent failure the moment a scene is
    # built.

    # DIGGING, STANDING — hinged at the waist over the pile with the arms
    # hanging into it. THE POSE `neighbour_dig` SHOULD ALWAYS HAVE HAD: it was
    # using `crouch`, whose forearms are carried forward over the knees at
    # -18/-50, and a squat with the forearms out in front is the silhouette of
    # somebody sitting at a steering wheel. That is the literal review comment
    # the rebuild started from.
    #
    #   trunk   spine_01 -46 + spine_02 -22 + spine_03 -10 = 78 deg of forward
    #           pitch. 78 rather than 66: at 66 the shoulder lands at 1.19 m
    #           and the fingertips (shoulder-to-tip 0.79 m on a 1.8 m figure,
    #           Drillis & Contini 1966 segment fractions) at 0.40 m — a third
    #           of a metre of clear air between the hands and the debris, which
    #           reads as bowing, not digging.
    #   legs    thigh -30, calf +62 -> shin at +32 (raked back), foot -32 to
    #           put the sole back flat. Ankle solves to 0.718 m below the hip
    #           against 0.836 m at rest, so the whole body drops 0.118 m and
    #           the fingertips finish ~0.18 m off the surface: hands IN the
    #           pile, which is the point.
    #   arms    the arms are children of the spine, so the 78 deg trunk pitch
    #           carries them 78 deg forward with it and they would point nearly
    #           horizontally ahead. +55 about X on each upper arm takes that
    #           back out so they hang under the shoulders the way loaded arms
    #           do; -20 on each forearm is the reach into the gap.
    #
    # WHAT TO LOOK AT: the hands. If they are level with the knees rather than
    # buried in the surface the figure is bowing at something, and the fix is
    # more knee (calf up from +62) rather than more back.
    "dig_bent": {
        "spine_01": ((1.0, 0.0, 0.0), -46.0),
        "spine_02": ((1.0, 0.0, 0.0), -22.0),
        "spine_03": ((1.0, 0.0, 0.0), -10.0),
        "neck_01": ((1.0, 0.0, 0.0), 12.0),       # un-tuck: the eyes on the pile
        "thigh_l": ((1.0, 0.0, 0.0), -30.0),
        "thigh_r": ((1.0, 0.0, 0.0), -26.0),      # stance a little open
        "calf_l": ((1.0, 0.0, 0.0), 62.0),
        "calf_r": ((1.0, 0.0, 0.0), 58.0),
        "foot_l": ((1.0, 0.0, 0.0), -32.0),       # cancel the shin: sole flat
        "foot_r": ((1.0, 0.0, 0.0), -32.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0),    # down out of the A-pose
        "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "upperarm_l+": ((1.0, 0.0, 0.0), 55.0),   # ...and back under the
        "upperarm_r+": ((1.0, 0.0, 0.0), 55.0),   # pitched shoulder: hanging
        "lowerarm_l": ((1.0, 0.0, 0.0), -20.0),   # into the gap
        "lowerarm_r": ((1.0, 0.0, 0.0), -24.0),
    },

    # DIGGING, HALF-KNEELING — one knee on the pile, the other foot planted.
    # The posture rubble work actually happens in, because a hinged stand
    # cannot be held and cannot pull: kneeling puts the shoulder 0.87 m up
    # instead of 1.19 and gives the hands the whole surface.
    #
    #   LEFT leg (down)     thigh +4, calf +86 -> shin at +90, i.e. flat on
    #                       the ground running back from the knee. Knee lands
    #                       0.471 m below the hip; that is the contact, and it
    #                       is why this pose needs a KNEE ground solve rather
    #                       than a sole one (see `_POSE_GROUND_CONTACT`).
    #   RIGHT leg (planted) solved rather than guessed. Want the ankle 0.380 m
    #                       below the hip (hip 0.527 = 0.471 knee + 0.056 knee
    #                       radius; ankle 0.147 above ground) and 0.30 m
    #                       forward. |hip->ankle| = 0.484; with a 0.472 thigh
    #                       and a 0.364 shin the cosine rule gives 44.7 deg
    #                       between the thigh and that line, whose own angle is
    #                       -38.3, so thigh = -83 and shin = +28: knee flexion
    #                       111 deg, front knee at hip height. That is a
    #                       half-kneel, not a lunge.
    #   trunk               42 deg forward (24 + 12 + 6). Less than `dig_bent`
    #                       needs, because the kneel has already lowered the
    #                       shoulder.
    #   arms                +34 about X to hang them back under the pitched
    #                       shoulder, -30 on the forearms to reach. Hands land
    #                       ~0.08 m above the surface: in it.
    #
    # WHAT TO LOOK AT: the down shin. If it stands proud of the debris the
    # figure is doing a lunge, and `calf_l` is short of +86.
    "dig_kneel": {
        "spine_01": ((1.0, 0.0, 0.0), -24.0),
        "spine_02": ((1.0, 0.0, 0.0), -12.0),
        "spine_03": ((1.0, 0.0, 0.0), -6.0),
        "neck_01": ((1.0, 0.0, 0.0), 10.0),
        "thigh_l": ((1.0, 0.0, 0.0), 4.0),        # kneeling leg: thigh plumb
        "calf_l": ((1.0, 0.0, 0.0), 86.0),        # shin flat on the ground
        "foot_l": ((1.0, 0.0, 0.0), 18.0),        # instep down, toes back
        "thigh_r": ((1.0, 0.0, 0.0), -83.0),      # planted leg, solved above
        "calf_r": ((1.0, 0.0, 0.0), 111.0),
        "foot_r": ((1.0, 0.0, 0.0), -28.0),       # cancel the shin: sole flat
        "thigh_r+": ((0.0, 1.0, 0.0), 14.0),      # front knee out, not crossed
        "upperarm_l": ((0.0, 1.0, 0.0), 42.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -42.0),
        "upperarm_l+": ((1.0, 0.0, 0.0), 34.0),
        "upperarm_r+": ((1.0, 0.0, 0.0), 34.0),
        "lowerarm_l": ((1.0, 0.0, 0.0), -30.0),
        "lowerarm_r": ((1.0, 0.0, 0.0), -34.0),
    },

    # SITTING ON THE GROUND, KNEES UP, HEAD DOWN — the injured survivor who
    # has stopped. Distinct from `sit_ground`, which is legs-out and propped
    # back on the hands: that is waiting, this is hurt. 89% of tornado injuries
    # are minor (ISS < 10) and 86% are discharged home (Niederkrotenthaler et
    # al. 2013, 1,398 patients / 39 hospitals) — the walking wounded are the
    # LARGEST visible group at T+45 min and until now the scene had no posture
    # for them at all.
    #
    #   THE FOLD IS DEEPER THAN IT LOOKS. Sitting on the floor with the feet
    #   flat means the hip joint and the ankle joint are BOTH about 0.10-0.15 m
    #   up, i.e. level with each other, with the knee ~0.45 m up and the foot
    #   ~0.40 m forward. Solving that against a 0.472 thigh and a 0.364 shin:
    #     knee   = hip + (0, -0.316, +0.351)   -> thigh at -138 deg
    #     ankle  = knee + (0, -0.082, -0.355)  -> shin  at  -13 deg
    #   hip flexion 138, knee flexion 125. A first pass at "hip 105 / knee 100"
    #   put the ankle 0.24 m BELOW the hip and the ground solve then sat the
    #   figure 0.35 m up, on an invisible stool.
    #   The heels-and-seat check: ankle finishes 0.004 m below the hip, so the
    #   sole solve lands the pelvis at 0.151 m — which is where a seated hip
    #   joint belongs, and 0.07 m higher than `sit_ground`'s heel solve puts it.
    #   foot +13 cancels the shin and returns the sole to flat.
    #   The right leg is 2 deg shallower and abducted 14 so the knees are not
    #   welded together; abduction is about Y and costs ~5 mm of ankle height,
    #   which is inside the debris surface's own roughness.
    #   Trunk 40 deg of ROUNDED flexion spread over three spine joints (a
    #   single -40 on spine_02 hinges like a mannequin), head hanging.
    #   Arms: elbows carried forward onto the knees (-30) with the forearms
    #   folded across the shins (-58).
    #
    # WHAT TO LOOK AT: the seat. If daylight shows under it the pelvis solve is
    # wrong, and it is `_POSE_GROUND_CONTACT`, not this table, that is wrong.
    "sit_slump": {
        "thigh_l": ((1.0, 0.0, 0.0), -138.0),
        "calf_l": ((1.0, 0.0, 0.0), 125.0),
        "foot_l": ((1.0, 0.0, 0.0), 13.0),
        "thigh_r": ((1.0, 0.0, 0.0), -136.0),
        "thigh_r+": ((0.0, 1.0, 0.0), 14.0),      # right knee falls outward
        "calf_r": ((1.0, 0.0, 0.0), 122.0),
        "foot_r": ((1.0, 0.0, 0.0), 14.0),
        "spine_01": ((1.0, 0.0, 0.0), -14.0),
        "spine_02": ((1.0, 0.0, 0.0), -16.0),
        "spine_03": ((1.0, 0.0, 0.0), -10.0),
        "neck_01": ((1.0, 0.0, 0.0), -16.0),      # head hanging, not looking up
        "head": ((1.0, 0.0, 0.0), -8.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 46.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -46.0),
        "upperarm_l+": ((1.0, 0.0, 0.0), -30.0),  # elbows forward onto the
        "upperarm_r+": ((1.0, 0.0, 0.0), -30.0),  # knees
        "lowerarm_l": ((1.0, 0.0, 0.0), -58.0),   # forearms across the shins
        "lowerarm_r": ((1.0, 0.0, 0.0), -58.0),
    },

    # STANDING, BUT NOT WALKING SOMEWHERE — head down, shoulders rolled, knees
    # soft, arms hanging dead. The casualty in an `assisted` trio (two people
    # holding a third up) and the shocked stander on a wrecked lot. `idle` is a
    # person waiting for a bus; this is a person who has just lost their house,
    # and at a hundred metres the difference is the head angle.
    #
    #   legs   thigh -10 / calf +20 / foot -10: 20 deg of knee, sole flat, and
    #          the ankle only 0.013 m above rest, so the ground solve is almost
    #          a no-op — this is a STANDER and it stays anchored at the soles.
    #   trunk  28 deg (10 + 12 + 6), then neck -18 and head -10: the chin ends
    #          on the chest, which is the whole read from above.
    #   arms   hanging at +44/-44 with 12 deg of forward drift and 18 deg of
    #          elbow, i.e. limp rather than placed.
    #
    # WHAT TO LOOK AT: from directly overhead this figure must still read as a
    # person and not as a shoulder-shaped blob — if the head has vanished
    # inside the torso outline, neck_01 has gone too far.
    "stand_slump": {
        "thigh_l": ((1.0, 0.0, 0.0), -10.0),
        "thigh_r": ((1.0, 0.0, 0.0), -8.0),
        "calf_l": ((1.0, 0.0, 0.0), 20.0),
        "calf_r": ((1.0, 0.0, 0.0), 18.0),
        "foot_l": ((1.0, 0.0, 0.0), -10.0),
        "foot_r": ((1.0, 0.0, 0.0), -10.0),
        "spine_01": ((1.0, 0.0, 0.0), -10.0),
        "spine_02": ((1.0, 0.0, 0.0), -12.0),
        "spine_03": ((1.0, 0.0, 0.0), -6.0),
        "neck_01": ((1.0, 0.0, 0.0), -18.0),
        "head": ((1.0, 0.0, 0.0), -10.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 44.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -44.0),
        "upperarm_l+": ((1.0, 0.0, 0.0), -12.0),
        "upperarm_r+": ((1.0, 0.0, 0.0), -12.0),
        "lowerarm_l": ((1.0, 0.0, 0.0), -18.0),
        "lowerarm_r": ((1.0, 0.0, 0.0), -18.0),
    },

    # ---- THE TWO LYING POSES, AND WHY THEY ARE NOT A ROTATED STANDER.
    #
    # Until now "prone" was `idle` rolled 90 deg about the facing axis
    # (`scene_generator._human_down`, and the `prone=True` branch of
    # `disaster.people._human_placement`, which hard-coded the pose to `idle`).
    # A rigid stander turned on its side has its arms pinned along its flanks
    # and its legs straight and touching, and that is not what a body on the
    # ground looks like from any altitude: what makes a casualty legible is the
    # ASYMMETRY — one limb out, one knee up, the head turned.
    #
    # THE ROLL STILL DOES THE LAYING DOWN. These are authored as UPRIGHT rigs
    # and the placement's `roll_deg` lays them over, because joint deltas
    # cannot move the root: rotating `pelvis` would swing the whole body about
    # a pelvis that stays at 0.95 m and the figure would hang in the air. So
    # the arithmetic below is in the CHARACTER's own frame (facing -Y, up +Z)
    # and the mapping into the world is:
    #
    #   roll +90  y' = -z, z' = +y   front -> DOWN.  FACE-DOWN. Head at world
    #                                +u where u = (cos yaw, sin yaw).
    #   roll -90  y' = +z, z' = -y   front -> UP.    FACE-UP.   Head at -u.
    #
    # so "forward" in a delta becomes DOWN (into the ground) for `lying_prone`
    # and UP for `lying_supine`, and the two poses therefore CANNOT share
    # limbs. `disaster.people._human_placement` now picks the roll SIGN from
    # the pose name rather than from a coin flip on (x + y), and refuses a
    # lying pose that is not laid down — a `lying_prone` standing upright is a
    # figure with an arm stretched overhead, which is precisely the silhouette
    # that got the old raised-arm pose cut on review.
    #
    # Neither appears in `_POSE_GROUND_CONTACT` or needs a `_POSE_Z_OFFSET`:
    # the prone branch never calls `pose_z_offset`, it seats the body by half
    # its measured depth (`fp["sy"] / 2`) instead.

    # FACE-UP. The attitude of somebody pulled out, or trapped on their back
    # under boards — the one that shows a face and a chest to a camera looking
    # down, which is why `trapped_partial` prefers it.
    #   right leg  hip 25 / knee 90, solved so the raised knee stands 0.20 m
    #              above the hip and the sole still reaches the ground:
    #              knee = hip + 0.472*(0,-0.423,+0.423) after the mapping,
    #              ankle = knee + 0.364*(0,-0.154,-0.553) -> 0.58 m toward the
    #              feet and 0.13 m below the hip, i.e. 0.045 m off the ground
    #              once the body is lifted by half its depth. More hip flexion
    #              than this cannot reach the ground at all: at hip 50 the foot
    #              would have to drop 0.46 m over a 0.364 m shin.
    #   left leg   straight, abducted 10 deg so the ankles are not touching.
    #   arms       left 30 deg off the body with the forearm swung 50 deg
    #              inboard (about Y, i.e. ACROSS the chest — an X delta would
    #              lift it vertically off the ground); right 40 deg off the
    #              body, elbow barely bent, palm up beside the hip.
    #   head       28 deg about the character's Z, which after the roll is the
    #              body's LONG axis: the head lolls to one side instead of
    #              staring at the sky.
    #
    # WHAT TO LOOK AT: the raised knee, from oblique. It is the single feature
    # that separates a casualty from a fallen mannequin, and it is also the
    # thing a board laid across the thighs has to clear.
    "lying_supine": {
        # SOFTENED 2026-08-27 with `lying_prone`'s heel and for the same
        # reason. -25/+90 put the knee 0.20 m up with the shin running down at
        # 62 deg; -19/+72 puts it 0.154 m up with the shin at 45, which still
        # reads as one knee drawn up and no longer reads as a leg in the air.
        # The foot still reaches: the knee is then 0.329 m above the surface
        # and the shin drops 0.257 m over its 0.364 m, landing the ankle
        # 0.072 m up.
        "thigh_r": ((1.0, 0.0, 0.0), -19.0),
        "calf_r": ((1.0, 0.0, 0.0), 72.0),
        "thigh_r+": ((0.0, 1.0, 0.0), 12.0),      # knee falls outward
        "thigh_l": ((1.0, 0.0, 0.0), -4.0),
        "thigh_l+": ((0.0, 1.0, 0.0), -10.0),     # left leg abducted
        "spine_02": ((1.0, 0.0, 0.0), 6.0),       # arched back over the debris
        "head": ((0.0, 0.0, 1.0), 28.0),          # lolled, not staring up
        "upperarm_l": ((0.0, 1.0, 0.0), 15.0),    # 30 deg off the body
        "upperarm_r": ((0.0, 1.0, 0.0), -5.0),    # 40 deg off the body
        "lowerarm_l": ((0.0, 1.0, 0.0), 50.0),    # forearm across the chest
        "lowerarm_r": ((0.0, 1.0, 0.0), -25.0),   # palm up beside the hip
    },

    # FACE-DOWN. `thrown` uses it — being picked up by a tornado is a
    # severe-injury and fatality mechanism (43% of hospitalised injuries at
    # OKC 1999 against 6% of treated-and-released, Brown et al. 2002), and
    # somebody who lands face-down does not roll over.
    #   left arm   the sprawl. Upper arm 95 deg out about Y (horizontal, out
    #              to the side) and the forearm 50 deg further about Y, so the
    #              whole L of the arm stays IN the plane that becomes the
    #              ground after roll +90. An X delta on the forearm would drive
    #              it into the ground on one sign and hold it in the air on the
    #              other, which is the trap this pose exists to document.
    #   right arm  down along the flank, 15 deg off the body.
    #   right leg  knee flexed 55 (calf positive = heel toward the buttock,
    #              which after roll +90 lifts the heel UP) with 18 deg of
    #              abduction so the knee falls out: a body face-down with one
    #              leg drawn up.
    #   head       turned 30 deg about the long axis; a face-down figure with
    #              its nose in the debris has no head at all from above.
    #
    # WHAT TO LOOK AT: the outstretched arm must lie ON the ground, not float
    # over it and not sink into it. If it does either, the upper-arm delta has
    # picked up an X component from somewhere.
    "lying_prone": {
        "upperarm_l": ((0.0, 1.0, 0.0), -95.0),   # out to the side, in-plane
        "lowerarm_l": ((0.0, 1.0, 0.0), -50.0),   # forearm angled to the head
        "upperarm_r": ((0.0, 1.0, 0.0), -25.0),   # down the flank
        "lowerarm_r": ((0.0, 1.0, 0.0), -15.0),
        "thigh_l": ((1.0, 0.0, 0.0), 2.0),
        "thigh_r": ((1.0, 0.0, 0.0), 6.0),
        "thigh_r+": ((0.0, 1.0, 0.0), 18.0),      # knee falls outward
        # HEEL UP, NOT LEG UP. 55 put the shin 57 deg off the ground — a leg
        # standing in the air, which is the one thing the second review picked
        # out of the bench render by eye. 32 lifts the heel 0.19 m over a
        # 0.364 m shin, which is a knee relaxed open on uneven ground and
        # still asymmetric enough to read as a body rather than a plank.
        "calf_r": ((1.0, 0.0, 0.0), 32.0),
        "spine_02": ((1.0, 0.0, 0.0), 4.0),
        "head": ((0.0, 0.0, 1.0), -30.0),         # cheek down, face in profile
    },

    # ---- THE LATERAL POSES, AND THE AXIS TRAP THAT COMES WITH THEM --------
    #
    # ADDED 2026-08-27 (second review). The pose table could put a body FACE-UP
    # and FACE-DOWN and nothing else, so a corridor full of casualties showed a
    # camera exactly two silhouettes. Real bodies on debris are as often on
    # their SIDE as either — it is the attitude somebody comes to rest in when
    # they are rolled rather than dropped, and it is the one that reads
    # differently from above: a side-lying figure is NARROW (0.4 m of shoulder
    # against 0.6 m of chest) and its limbs stack rather than spread.
    #
    # HOW A FIGURE GETS ONTO ITS SIDE. `apply_placements` authors rotateXYZ, so
    # the ops go X (roll) then Y (pitch) then Z (yaw). Roll +-90 lays the body
    # down and puts its long axis along the pre-yaw -+Y; the PITCH axis is then
    # world Y, which IS the body's own long axis, so `pitch_deg` spins the laid
    # body about itself:
    #
    #     roll +90, pitch   0    face-down            (`lying_prone`)
    #     roll -90, pitch   0    face-up              (`lying_supine`)
    #     roll +90, pitch +90    on the LEFT side     (`lying_side_l`)
    #     roll +90, pitch -90    on the RIGHT side    (`lying_side_r`)
    #
    # The head still points at +u = (cos yaw, sin yaw) for every roll +90 pose
    # and at -u for roll -90, because a rotation about Y does not move a vector
    # that lies along Y. `disaster.people.LYING_POSES` / `LYING_SPIN` carry the
    # pair and `tornado_people._body_axis` reads the roll alone, unchanged.
    #
    # THE TRAP, AND IT IS THE MIRROR IMAGE OF THE ONE `lying_prone` DOCUMENTS.
    # For a face-up or face-down figure the ground plane is spanned by the
    # character's X (left-right) and Z (up) axes, so a delta about **Y** keeps
    # a limb ON the ground — which is why every arm delta in the two poses
    # above is `((0, 1, 0), a)`. Spin the body 90 deg about its long axis and
    # the ground plane becomes span(Y, Z): now it is a delta about **X** that
    # keeps a limb down, and a Y delta lifts it into the air. Every limb delta
    # in the three poses below is therefore about X, and the handful of Y
    # deltas are there deliberately, to lift the TOP limb clear of the bottom
    # one.
    #
    # THE LIFT IS NOT `sy / 2` EITHER. The prone branch seats a laid-down body
    # on half its measured A-pose DEPTH (0.33-0.40 m across this pack, so
    # ~0.17 m). On its side the vertical dimension is the body's BREADTH, and
    # `sx` cannot supply it — the A-pose bbox is 1.20-1.34 m wide because the
    # arms are out. `disaster.people._lying_lift` models it from stature
    # instead: biacromial breadth is 0.245 H (Drillis & Contini 1966) and the
    # spine sits at half of it, 0.22 m at 1.80 m; the hips are narrower
    # (0.191 H) and the shoulder flesh compresses, so 0.115 H = 0.207 m is the
    # figure used. VERIFY ON SIGHT: a side-lying figure whose shoulder is
    # buried and whose hip floats means this number is too small, and the
    # reverse means it is too big.

    # ON THE LEFT SIDE — the recovery-position silhouette, and the commonest
    # attitude a body settles into on an uneven surface: hips and knees flexed
    # forward so the legs stack in front of the trunk, the underneath (left)
    # arm reaching forward along the ground, the top (right) arm draped across
    # and resting in front of the chest, the head tipped down onto the debris.
    #   legs   hip -32 / -58 (the TOP leg drawn up further, which is what makes
    #          the stagger read from above as two legs and not one) with knees
    #          at 48 and 88 of flexion.
    #   arms   both swung forward about X — the bottom one nearly straight and
    #          reaching (-78 / -34), the top one folded (-52 / -62) so the hand
    #          finishes in front of the face rather than out on the ground.
    #   the +Y deltas on the RIGHT limbs are the lift: on this side the right
    #          arm and leg are the upper ones, and without 12-16 deg of
    #          abduction they lie exactly inside the left ones and the figure
    #          reads as a single limb from directly overhead.
    "lying_side_l": {
        # legs — the thighs hang plumb at rest, so an X delta is already in
        # the ground plane and needs no correction.
        "thigh_l": ((1.0, 0.0, 0.0), -32.0),
        "calf_l": ((1.0, 0.0, 0.0), 48.0),
        "thigh_r": ((1.0, 0.0, 0.0), -58.0),
        "calf_r": ((1.0, 0.0, 0.0), 88.0),
        "thigh_r+": ((0.0, 1.0, 0.0), 14.0),      # top leg lifted clear
        # arms — KILL THE A-POSE ABDUCTION FIRST. This is the whole bug the
        # first render showed: the rest arm hangs 45 deg OUT to the side, and
        # that 45 deg is in the character's X, which after the long-axis spin
        # is straight UP. Swinging it forward about X without first bringing
        # it to plumb leaves an arm standing 45 deg into the air off every
        # side-lying figure in the scene, which is exactly what happened.
        # +-40 about Y is the plumb correction `idle` measures (hand 7 mm off
        # plumb under the shoulder); the X swing then runs along the ground.
        # The FOREARM needs no correction of its own — it inherits the upper
        # arm's posed frame, so once that is plumb the elbow is too.
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0),    # bottom arm: to plumb...
        "upperarm_l+": ((1.0, 0.0, 0.0), -74.0),  # ...then forward, flat
        "lowerarm_l": ((1.0, 0.0, 0.0), -32.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -40.0),   # top arm: to plumb...
        "upperarm_r+": ((1.0, 0.0, 0.0), -50.0),  # ...forward, folded over...
        "upperarm_r++": ((0.0, 1.0, 0.0), 18.0),  # ...and lifted clear
        "lowerarm_r": ((1.0, 0.0, 0.0), -58.0),
        "spine_02": ((1.0, 0.0, 0.0), -12.0),     # trunk curled forward
        "neck_01": ((1.0, 0.0, 0.0), -8.0),
        "head": ((0.0, 1.0, 0.0), 16.0),          # cheek down onto the debris
    },

    # ON THE RIGHT SIDE. A real mirror rather than the same dict with a
    # different spin: the flexions about X are UNCHANGED (forward is forward
    # whichever side you are on), the plumb corrections keep their own signs
    # because they are per-arm, and only the LIFT deltas swap limb and sign,
    # because the top limbs are now the left ones.
    "lying_side_r": {
        "thigh_r": ((1.0, 0.0, 0.0), -32.0),
        "calf_r": ((1.0, 0.0, 0.0), 48.0),
        "thigh_l": ((1.0, 0.0, 0.0), -58.0),
        "calf_l": ((1.0, 0.0, 0.0), 88.0),
        "thigh_l+": ((0.0, 1.0, 0.0), -14.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "upperarm_r+": ((1.0, 0.0, 0.0), -74.0),
        "lowerarm_r": ((1.0, 0.0, 0.0), -32.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0),
        "upperarm_l+": ((1.0, 0.0, 0.0), -50.0),
        "upperarm_l++": ((0.0, 1.0, 0.0), -18.0),
        "lowerarm_l": ((1.0, 0.0, 0.0), -58.0),
        "spine_02": ((1.0, 0.0, 0.0), -12.0),
        "neck_01": ((1.0, 0.0, 0.0), -8.0),
        "head": ((0.0, 1.0, 0.0), -16.0),
    },

    # CURLED ON THE LEFT SIDE. The other lateral silhouette, and the one that
    # matters most to a detector: a body drawn up is SHORT — knees to chest
    # takes a 1.8 m target down to about 1.1 m of ground length — so a model
    # trained only on extended figures has never seen the aspect ratio. It is
    # also what somebody who took cover and was buried actually looks like:
    # Hammer & Schmidlin's interior-refuge occupants were in bathrooms and
    # closets, curled, when the structure failed on them.
    #   hips 88 / 96 of flexion, knees 104 / 112, so both heels finish under
    #   the thighs; the arms fold in front of the face; the trunk flexes 24
    #   and the head tucks. Same plumb-first rule for the arms.
    "lying_curled_l": {
        "thigh_l": ((1.0, 0.0, 0.0), -88.0),
        "calf_l": ((1.0, 0.0, 0.0), 104.0),
        "thigh_r": ((1.0, 0.0, 0.0), -96.0),
        "calf_r": ((1.0, 0.0, 0.0), 112.0),
        "thigh_r+": ((0.0, 1.0, 0.0), 12.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0),
        "upperarm_l+": ((1.0, 0.0, 0.0), -62.0),
        "lowerarm_l": ((1.0, 0.0, 0.0), -98.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "upperarm_r+": ((1.0, 0.0, 0.0), -56.0),
        "upperarm_r++": ((0.0, 1.0, 0.0), 16.0),
        "lowerarm_r": ((1.0, 0.0, 0.0), -104.0),
        "spine_02": ((1.0, 0.0, 0.0), -24.0),
        "neck_01": ((1.0, 0.0, 0.0), -14.0),
        "head": ((0.0, 1.0, 0.0), 12.0),
    },

    # ---- TWO MORE FACE-UP / FACE-DOWN VARIANTS ---------------------------
    # Same laying-down mechanics as `lying_supine` / `lying_prone` (deltas
    # about Y stay in the ground plane), different limb story. They exist
    # because a dozen casualties drawn from two poses is a dozen copies of two
    # figures, which is a texture rather than a population.

    # FACE-UP, ARMS OUT, LEGS STRAIGHT. The "dropped where they stood"
    # attitude — no limb drawn up, both arms away from the trunk in the ground
    # plane. It is the WIDEST silhouette in the set (about 1.5 m across the
    # hands) and the easiest of all of them to see from directly above, which
    # is why it carries the fully-visible end of the mix.
    "lying_supine_open": {
        # THE SIGN GOES THE OTHER WAY FROM `idle`. For the LEFT arm a POSITIVE
        # Y delta brings the limb IN — 40 is plumb under the shoulder, which
        # is what `idle` measures — so taking it OUT to the side needs a
        # NEGATIVE one, and -45 is the arm exactly horizontal. The first
        # authoring had +52 / -46 here, which is 12 degrees PAST plumb and
        # across the body: the pose meant to be the widest silhouette in the
        # set came out with both arms pinned to its flanks.
        "upperarm_l": ((0.0, 1.0, 0.0), -38.0),   # out to ~85 deg from the
        "upperarm_r": ((0.0, 1.0, 0.0), 42.0),    # trunk, both in-plane
        "lowerarm_l": ((0.0, 1.0, 0.0), -18.0),
        "lowerarm_r": ((0.0, 1.0, 0.0), 16.0),
        "thigh_l": ((0.0, 1.0, 0.0), -9.0),       # legs apart, both straight
        "thigh_r": ((0.0, 1.0, 0.0), 13.0),
        "calf_l": ((1.0, 0.0, 0.0), 8.0),
        "calf_r": ((1.0, 0.0, 0.0), 5.0),
        "spine_02": ((1.0, 0.0, 0.0), 4.0),
        "head": ((0.0, 0.0, 1.0), -34.0),         # face turned to one side
    },

    # FACE-DOWN, BOTH ARMS UP BESIDE THE HEAD. The other end of the same
    # argument: a prone figure with its arms overhead is 2.1 m of ground length
    # and reads as a person at an altitude where a compact one reads as a
    # board. Both arms go out about Y (in-plane, as `lying_prone` documents),
    # one leg trails straight and the other is abducted 16 deg.
    "lying_prone_reach": {
        "upperarm_l": ((0.0, 1.0, 0.0), -128.0),  # up beside the head
        "lowerarm_l": ((0.0, 1.0, 0.0), -26.0),
        "upperarm_r": ((0.0, 1.0, 0.0), 118.0),
        "lowerarm_r": ((0.0, 1.0, 0.0), 34.0),
        "thigh_l": ((0.0, 1.0, 0.0), -6.0),
        "thigh_r": ((0.0, 1.0, 0.0), 16.0),
        "calf_l": ((1.0, 0.0, 0.0), 14.0),
        "spine_02": ((1.0, 0.0, 0.0), 3.0),
        "head": ((0.0, 0.0, 1.0), 38.0),          # cheek down, other way round
    },

    # PINNED SITTING UP. The other half of `trapped_partial`, and it is a
    # genuinely different labelling problem from a lying figure: the trunk is
    # VERTICAL, so what a board can hide is the legs and the lap, and what is
    # unavoidably visible is a head and shoulders standing above the pile.
    # Both attitudes are recorded in the ground truth as `partial` and a
    # detector that only ever sees one of them has learnt half the target.
    #   legs   `sit_ground`'s legs out in front, 2 deg less hip so the heel
    #          solve lands the pelvis at 0.127 m: thigh -88 / calf +5 puts the
    #          ankle 0.833 m forward and 0.061 m below the hip.
    #   left   the prop. Arm carried BACK behind the hip (+34 about X) and
    #          wide (+58 about Y), elbow nearly straight — the arm somebody
    #          holds themselves up on when their legs are under something.
    #   right  the free arm: upper arm 25 deg off the body and 20 deg forward,
    #          forearm folded 95 deg so it stands VERTICALLY from the elbow.
    #          A bent forearm at chest height, not a straight arm overhead:
    #          the straight-overhead silhouette is the one that read as a
    #          mannequin with a broken limb and got cut, and nothing here
    #          brings it back.
    #   trunk  6 deg of forward lean and 10 deg of turn about the vertical, so
    #          the shoulders are not square to the boards across the lap.
    #
    # WHAT TO LOOK AT: whether the head and the free forearm still clear the
    # boards `tornado_people._trapped_partial` lays across the lap. That pairing
    # is the whole scenario and it is the one thing only a render can settle.
    "trapped_sit": {
        "thigh_l": ((1.0, 0.0, 0.0), -88.0),
        "thigh_r": ((1.0, 0.0, 0.0), -88.0),
        "thigh_r+": ((0.0, 1.0, 0.0), 10.0),
        "calf_l": ((1.0, 0.0, 0.0), 5.0),
        "calf_r": ((1.0, 0.0, 0.0), 9.0),
        "foot_l": ((1.0, 0.0, 0.0), 14.0),
        "foot_r": ((1.0, 0.0, 0.0), 14.0),
        "spine_01": ((1.0, 0.0, 0.0), -6.0),
        "spine_02": ((0.0, 0.0, 1.0), 10.0),      # shoulders turned off square
        "upperarm_l": ((0.0, 1.0, 0.0), 58.0),    # propping arm, wide...
        "upperarm_l+": ((1.0, 0.0, 0.0), 34.0),   # ...and behind the hip
        "lowerarm_l": ((0.0, 1.0, 0.0), 6.0),     # nearly straight: weight on it
        "upperarm_r": ((0.0, 1.0, 0.0), 20.0),    # free arm, 25 deg off body
        "upperarm_r+": ((1.0, 0.0, 0.0), -20.0),
        "lowerarm_r": ((1.0, 0.0, 0.0), -95.0),   # forearm vertical from elbow
    },

    # ---- THE ASSIST, MIRRORED. Two poses because an assist is not symmetric:
    # the supporter has ONE arm around the person they are holding up and the
    # other hanging, and which arm depends on which side of them they are
    # standing. `_assisted` builds its trio as (casualty, one either side) at
    # +-0.62 m along the road normal, so the figure on the casualty's LEFT
    # reaches across with its RIGHT arm and vice versa; a single pose used for
    # both would put one supporter's arm through the casualty's back.
    #
    # THIS IS THE ONE PLACE A RAISED ARM IS CORRECT, and it is worth being
    # explicit about why it is not the cut raised-arm pose coming back: that
    # one was a signalling gesture, a lone figure with a straight arm over its
    # head, and it read as a broken limb. This is a bent arm carried at
    # SHOULDER HEIGHT across a body that is touching it, in a group of three.
    # The silhouette that makes it legible from altitude is the group, not the
    # limb.
    #
    # Derived from `walk`, which is what the supporters used to be:
    #   stride  halved (thigh -10 / +7, calf +9). Two people carrying a third
    #           take short steps; a full stride also drove them out of contact
    #           with the casualty inside one frame's worth of animation had
    #           anything ever animated these.
    #   inboard shoulder abducted to ~15 deg below horizontal (Y +/-30 from
    #           the A-pose's 45) and carried 22 deg forward, elbow flexed 70:
    #           the forearm crosses in front at chest height, which is where
    #           an arm around somebody's back sits.
    #   outboard arm hanging at `idle`'s +-40 with a slight swing.
    #   trunk   8 deg of lateral lean TOWARD the casualty (a Y-axis delta on
    #           spine_01 tips the trunk sideways: the trunk's up vector goes
    #           to (-sin b, 0, cos b), i.e. toward the character's right for
    #           positive b).
    # No ground-contact entry and a zero offset, exactly as `walk` has: the
    # legs are scissored, one heel is off the ground and that is what walking
    # looks like. VERIFY ON SIGHT: the reaching hand should finish over the
    # casualty's far shoulder, not inside its head.

    # For the supporter standing on the casualty's LEFT — reaches with the RIGHT.
    "support_r": {
        "thigh_l": ((1.0, 0.0, 0.0), -10.0),
        "thigh_r": ((1.0, 0.0, 0.0), 7.0),
        "calf_r": ((1.0, 0.0, 0.0), 9.0),
        "spine_01": ((0.0, 1.0, 0.0), 8.0),       # lean toward the casualty
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0),    # outboard arm hangs
        "upperarm_l+": ((1.0, 0.0, 0.0), 8.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 8.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -30.0),   # inboard arm up to the
        "upperarm_r+": ((1.0, 0.0, 0.0), -22.0),  # shoulder, carried forward
        "lowerarm_r": ((0.0, 1.0, 0.0), -70.0),   # forearm across the back
    },

    # For the supporter standing on the casualty's RIGHT — reaches with the LEFT.
    "support_l": {
        "thigh_r": ((1.0, 0.0, 0.0), -10.0),
        "thigh_l": ((1.0, 0.0, 0.0), 7.0),
        "calf_l": ((1.0, 0.0, 0.0), 9.0),
        "spine_01": ((0.0, 1.0, 0.0), -8.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "upperarm_r+": ((1.0, 0.0, 0.0), 8.0),
        "lowerarm_r": ((0.0, 1.0, 0.0), -8.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 30.0),
        "upperarm_l+": ((1.0, 0.0, 0.0), -22.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 70.0),
    },

    # ---- FIRE-VICTIM POSES (disaster/fire_people.py). ADDED 2026-08-31 after
    # the bench review: "the poses are completely wrong ... the current people
    # look like they're about to jump off the roof", "the people on the roof
    # need to just be standing. not like bent", "I need one hand raised as the
    # rescue signal", "torso poking outside, but their legs and lower body is
    # inside". None of the existing table served these four situations without
    # reading wrong, so these are new rather than reused. VERIFY ON SIGHT at
    # the bench relaunch — every number below was checked analytically against
    # the real rp_carla rig (`_pose_joint_transforms`, forward-kinematics,
    # 95-joint UE4 skeleton — same tool `tools/pose_check.py` uses) but never
    # rendered.
    #
    # MEASURED ON rp_carla (mpu 0.01): pelvis 0.9834, upperarm 0.2417,
    # lowerarm 0.2867, thigh 0.4727, calf 0.3646 — matches the header
    # constants above to 3 decimal places, so the FK tool used to solve these
    # is trustworthy against this rig.

    # UPRIGHT AND NEUTRAL. This is the "just standing" pose the roof classes
    # were missing: `idle`'s arm-plumb correction (identical 40/8 Y deltas)
    # with the stance widened by 8 deg of THIGH ABDUCTION each leg (a Y delta
    # on a leg that hangs plumb at rest, same mechanism `sit_ground`'s
    # `thigh_r+`/`lying_supine`'s `thigh_r+` use to fall a knee outward — here
    # it just widens the stance) instead of the fore-aft stride `walk` uses. No
    # spine or head delta at all: `stand_slump`'s whole silhouette is its
    # forward pitch (-10/-12/-6) and dropped head (-18/-10), which is exactly
    # the "bent" / "about to jump" read the review objected to on a roof deck.
    # Anchored at the soles like `idle` — no ground-contact solve, offset 0.
    "stand_calm": {
        "thigh_l": ((0.0, 1.0, 0.0), 8.0),
        "thigh_r": ((0.0, 1.0, 0.0), -8.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 8.0),
        "lowerarm_r": ((0.0, 1.0, 0.0), -8.0),
    },

    # THE STANDARD RESCUE SIGNAL — both arms overhead. NOT the cut `wave`: that
    # was ONE rigid straight arm on a lone figure and read as a broken limb.
    # This is symmetric (both arms), softly bent at the elbow and spread apart
    # (a V, not two parallel poles) precisely so it does not repeat that
    # silhouette, and it is only ever authored on figures in a GROUP context
    # (roof / roof_victim / window), never alone.
    #
    # Arms swing about X exactly the way a leg does once the A-pose splay is
    # corrected to plumb (`upperarm_*`: Y 40/-40, matching `idle`) — MEASURED:
    # a pure X delta of -180 on the plumbed arm puts the elbow dead vertical
    # over the shoulder (elbow-rel-to-shoulder = (-0.02, 0.00, +0.240), i.e.
    # the full 0.242 m upperarm length, 0 lateral/forward error). -155 (this
    # pose) leaves the arm short of straight overhead by design — a fully
    # vertical arm reads as a flagpole; short of it reads as a person raising
    # both arms. The `++` delta spreads each arm 18 deg further out (Y) after
    # the swing, so the two hands finish apart rather than touching, and the
    # forearms fold 25 deg at the elbow rather than staying ramrod straight.
    # MEASURED (rp_carla, 1.731 m): hand 0.124 m above `head_end`, 0.355 m
    # lateral half-spread — both hands clear the head and are visibly apart
    # from directly above, which is the read a rescue signal needs from a
    # drone. Legs widened 8 deg (as `stand_calm`) so the whole figure reads as
    # standing square, not mid-stride. Anchored at the soles, offset 0.
    "wave_help": {
        "thigh_l": ((0.0, 1.0, 0.0), 8.0),
        "thigh_r": ((0.0, 1.0, 0.0), -8.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0),
        "upperarm_l+": ((1.0, 0.0, 0.0), -155.0),
        "upperarm_l++": ((0.0, 1.0, 0.0), 18.0),
        "lowerarm_l": ((1.0, 0.0, 0.0), -25.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "upperarm_r+": ((1.0, 0.0, 0.0), -155.0),
        "upperarm_r++": ((0.0, 1.0, 0.0), -18.0),
        "lowerarm_r": ((1.0, 0.0, 0.0), -25.0),
    },

    # LEANING OUT OF A WINDOW, PELVIS-HINGED — FINALIZED 2026-09-01 from the
    # 8-variant bench pose row (`fire_people.WINDOW_POSE_VARIANTS`, now
    # retired — see that name's own history for the 7 rejected mechanisms).
    # Coordinator's verdict on the render: "`lw_pelvis_posx` — clean forward
    # lean out of the wall plane, face visible, reads as a person leaning
    # out. REJECTED: spine-chain variants hinge so deep the head passes
    # through the glazing ... mixed_posx reads as standing-looking-down."
    #
    # THIS IS A DIFFERENT JOINT FROM EVERY OTHER POSE IN THIS TABLE. Nothing
    # else here ever puts a delta on `pelvis` — every other forward-leaning
    # pose (`dig_bent`, `stand_slump`, the retired spine-only `lean_window`)
    # hinges the SPINE CHAIN with the legs left untouched. A `pelvis` delta
    # rotates the ROOT-ADJACENT joint itself, so BOTH the spine chain and
    # both legs hang from the rotated frame — a true rigid seesaw, legs
    # included. That is exactly why the spine-only mechanism read as
    # "bent the wrong way" on a real render while this one did not: this
    # pack's actual USD Skel evaluation was answering a different question
    # (which joint reads as the body's own forward hinge) than three rounds
    # of `_pose_joint_transforms` derivation had modelled it as.
    #
    # SIGN, MEASURED, NOT ASSUMED. Every OTHER trunk-forward pose in this
    # table leans in +Y (spine stands UP at rest, so `v(a)`'s NEGATIVE X
    # gives +Y — see `dig_bent`'s own account). The bench row tested BOTH
    # signs on `pelvis` and the WINNING one was `lw_pelvis_posx`: POSITIVE
    # 35 deg, leaning in -Y, the opposite of the convention every other pose
    # here follows. Do not "fix" this sign to match the others — it was
    # fixed once already (the retired spine-only version), on paper, and
    # that is the version that rendered backwards. -Y is what this specific
    # class's `_pass_window` yaw computation actually treats as outward.
    #
    # MODERATE ANGLE — the row's own magnitude (35 deg) read as right;
    # `pelvis` +30 here splits the difference with the request's own
    # "~25-35 deg" and leaves a small margin off the row's tested extreme.
    # MEASURED (rp_carla, 1.731 m): head 0.262 m of -Y off the pelvis
    # (0.152 of stature — `_LEAN_WINDOW_HEAD_FRAC_H` below), hand_l 0.612 m
    # — a clearly braced-forward arm silhouette.
    #
    # ARMS FOLLOW THE SAME -Y SIGN — the opposite of the retired version's
    # arms for the same reason the pelvis sign flipped: an arm hangs DOWN
    # like a leg, and `v(a)`'s NEGATIVE X is what sends a down-hanging limb
    # toward -Y (the leg convention: thigh -20 = forward = -Y). Plumbed
    # (Y ±40) then swung -55 about X, forearm -16 further.
    #
    # HEAD COUNTERED, KEPT SUBTLE — coordinator: "a small head/neck-up
    # counter-rotation so the face looks outward rather than at the
    # pavement ... keep it subtle." `neck_01` -16 / `head` -9 (sum -25
    # against the pelvis's own +30) leaves the head's NET rotation at only
    # +5 deg — close to level, legible as looking outward, not the dramatic
    # counter the retired version needed against its much larger 78 deg
    # spine lean.
    #
    # LEGS ARE NOT INDEPENDENTLY POSED, BUT THEY DO MOVE — a consequence of
    # hinging at `pelvis` rather than the spine, and left as-is: the row's
    # own render is what approved this, legs included, and the window
    # class's own sill-hip z placement (`fire_people._pass_window`,
    # `op["z_sill"] - _HIP_H * H`) still lands the PELVIS at the sill
    # correctly regardless — a joint's own rotation never moves its own
    # position, so the pelvis sits at exactly its rest height (0.983 m on
    # rp_carla) whatever angle it is turned through. The legs stay behind
    # the wall panel below the sill either way.
    "lean_window": {
        "pelvis": ((1.0, 0.0, 0.0), 30.0),
        "neck_01": ((1.0, 0.0, 0.0), -16.0),
        "head": ((1.0, 0.0, 0.0), -9.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0),
        "upperarm_l+": ((1.0, 0.0, 0.0), -55.0),
        "lowerarm_l": ((1.0, 0.0, 0.0), -16.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "upperarm_r+": ((1.0, 0.0, 0.0), -55.0),
        "lowerarm_r": ((1.0, 0.0, 0.0), -16.0),
    },

    # ======================================================================
    # THROWAWAY — WINDOW-POSE EMPIRICAL SEARCH, 2026-09-01. DELETE AFTER THE
    # BENCH ROW SETTLES WHICH ONE ACTUALLY LEANS OUT. User: "the people in
    # windows look completely wrong, they're just standing straight."
    # Coordinator: "empirical search instead of derivation ... one render
    # settles the UE/USD mapping question that three rounds of paper
    # derivation have not."
    #
    # Every one of these was checked with `_pose_joint_transforms` against
    # the real rig — the arithmetic composes without error for all eight —
    # which is exactly why derivation alone cannot pick a winner: `lean_
    # window`'s own numbers ALSO composed correctly on paper and still
    # rendered backwards (bench-v2). What is being searched here is not
    # "does this pose compute", it is "which axis and sign the actual
    # Isaac Sim skinning/skeleton pipeline treats as forward" — a question
    # only a render answers.
    #
    # Arms and stance are IDENTICAL across all eight (`idle`'s plumb
    # correction, feet at `stand_calm`'s 8 deg abduction) so the ONLY
    # variable between renders is the hinge mechanism itself:
    #
    #   pelvis_negx / pelvis_posx   a single hinge at the ROOT-adjacent
    #     `pelvis` joint (children: BOTH legs and the whole spine chain —
    #     the legs tip WITH the trunk, a true rigid seesaw). MEASURED:
    #     head lands 0.298 m / -0.316 m of Y off the pelvis.
    #   pelvis_negy / pelvis_posy   the CONTROL for "wrong axis": a Y delta
    #     on `pelvis` rolls the hips SIDEWAYS (one hip up, one down) with
    #     no forward/back motion at all — MEASURED head_dy = -0.011 m,
    #     i.e. no lean. Included because the coordinator asked for both
    #     axes explicitly; expected to read as "tips sideways", not a lean.
    #   spine_negx / spine_posx   the CURRENT `lean_window` mechanism
    #     (spine_01/02/03 distributed, legs untouched) at both signs.
    #     MEASURED: head 0.208 m / -0.225 m of Y.
    #   mixed_negx / mixed_posx   pelvis AND spine share the hinge (legs
    #     tip partway, trunk adds the rest) at both signs. MEASURED: head
    #     0.260 m / -0.276 m of Y.
    #
    # `fire_people._pass_window_pose_experiment` (gated on `cfg["window_
    # pose_experiment"]`, off by default) places one figure per surviving
    # name here at consecutive real openings of the bench's intact F3
    # building — see that function's own docstring.
    "lw_pelvis_negx": {
        "pelvis": ((1.0, 0.0, 0.0), -35.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0), "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 8.0), "lowerarm_r": ((0.0, 1.0, 0.0), -8.0),
        "thigh_l": ((0.0, 1.0, 0.0), 8.0), "thigh_r": ((0.0, 1.0, 0.0), -8.0),
    },
    "lw_pelvis_posx": {
        "pelvis": ((1.0, 0.0, 0.0), 35.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0), "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 8.0), "lowerarm_r": ((0.0, 1.0, 0.0), -8.0),
        "thigh_l": ((0.0, 1.0, 0.0), 8.0), "thigh_r": ((0.0, 1.0, 0.0), -8.0),
    },
    "lw_pelvis_negy": {
        "pelvis": ((0.0, 1.0, 0.0), -35.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0), "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 8.0), "lowerarm_r": ((0.0, 1.0, 0.0), -8.0),
        "thigh_l": ((0.0, 1.0, 0.0), 8.0), "thigh_r": ((0.0, 1.0, 0.0), -8.0),
    },
    "lw_pelvis_posy": {
        "pelvis": ((0.0, 1.0, 0.0), 35.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0), "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 8.0), "lowerarm_r": ((0.0, 1.0, 0.0), -8.0),
        "thigh_l": ((0.0, 1.0, 0.0), 8.0), "thigh_r": ((0.0, 1.0, 0.0), -8.0),
    },
    "lw_spine_negx": {
        "spine_01": ((1.0, 0.0, 0.0), -20.0), "spine_02": ((1.0, 0.0, 0.0), -12.0),
        "spine_03": ((1.0, 0.0, 0.0), -7.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0), "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 8.0), "lowerarm_r": ((0.0, 1.0, 0.0), -8.0),
        "thigh_l": ((0.0, 1.0, 0.0), 8.0), "thigh_r": ((0.0, 1.0, 0.0), -8.0),
    },
    "lw_spine_posx": {
        "spine_01": ((1.0, 0.0, 0.0), 20.0), "spine_02": ((1.0, 0.0, 0.0), 12.0),
        "spine_03": ((1.0, 0.0, 0.0), 7.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0), "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 8.0), "lowerarm_r": ((0.0, 1.0, 0.0), -8.0),
        "thigh_l": ((0.0, 1.0, 0.0), 8.0), "thigh_r": ((0.0, 1.0, 0.0), -8.0),
    },
    "lw_mixed_negx": {
        "pelvis": ((1.0, 0.0, 0.0), -15.0),
        "spine_01": ((1.0, 0.0, 0.0), -12.0), "spine_02": ((1.0, 0.0, 0.0), -8.0),
        "spine_03": ((1.0, 0.0, 0.0), -5.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0), "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 8.0), "lowerarm_r": ((0.0, 1.0, 0.0), -8.0),
        "thigh_l": ((0.0, 1.0, 0.0), 8.0), "thigh_r": ((0.0, 1.0, 0.0), -8.0),
    },
    "lw_mixed_posx": {
        "pelvis": ((1.0, 0.0, 0.0), 15.0),
        "spine_01": ((1.0, 0.0, 0.0), 12.0), "spine_02": ((1.0, 0.0, 0.0), 8.0),
        "spine_03": ((1.0, 0.0, 0.0), 5.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 40.0), "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 8.0), "lowerarm_r": ((0.0, 1.0, 0.0), -8.0),
        "thigh_l": ((0.0, 1.0, 0.0), 8.0), "thigh_r": ((0.0, 1.0, 0.0), -8.0),
    },
    # ====================================================================

    # SUPINE, ONE ARM EXTENDED — for a figure partially buried in collapse
    # rubble (`fire_people.casualty_apron`/`roof_debris`), reaching clear of
    # the debris that covers the rest of them. Distinct from `lying_prone_
    # reach` (BOTH arms overhead, face-down, "2.1 m of ground length"): this
    # is face-UP and ASYMMETRIC — one arm reaching, one arm pinned close to
    # the body — because the whole point is a figure whose one side is under
    # material and whose other is not.
    #
    # Authored upright like every lying pose; `disaster.people`/`disaster.
    # fire_people`'s `prone=True` branch lays it down by roll (-90, face-up —
    # see `LYING_POSES`/`LYING_ROLL`) and lifts it by half the measured body
    # DEPTH, exactly as `lying_supine` is.
    #
    # THE REACHING ARM MUST LEAVE THE GROUND PLANE, WHICH RULES OUT A Y DELTA
    # — the opposite of what `lying_prone_reach` needs. A first draft of this
    # pose copied `lying_prone_reach`'s Y-axis magnitude (118-128, "up beside
    # the head") on the argument that both are face-up/face-down arm deltas
    # and both therefore "stay in the ground plane". That argument is correct
    # for `lying_prone_reach` (an arm lying flat past the head IS the point)
    # and wrong here: a delta about Y for a face-up figure keeps the limb IN
    # the plane the body is lying in, so the hand ends up lying flat on the
    # ground beside the head — MEASURED (`_pose_joint_transforms`, the real
    # rig): after the `-90` supine roll the hand sits only 0.098 m above the
    # pelvis LINE, which is buried under exactly the same rubble surface as
    # the rest of the body. "Reaching clear of the debris" needs height, and
    # height for a face-up limb comes from an X delta instead — the trap the
    # LATERAL poses' own comment documents (X keeps a limb down once the body
    # is on its SIDE) is mirrored here: for a face-up body it is the X axis
    # that lifts a limb OUT of the ground plane. Bring the arm to plumb first
    # (`upperarm_r`: Y -40, `idle`'s own correction) and then swing it -108
    # about X — MEASURED: the hand finishes 0.688 m forward and 0.411 m PROUD
    # of the pelvis line once the body is rolled supine, a real reach up and
    # out rather than a hand resting on the ground beside the head.
    # Left arm and legs are `lying_supine`'s own proven values verbatim
    # (15/50 tucked arm, the asymmetric raised-knee legs) — reused rather than
    # re-solved because they are already reviewed and correct for this
    # attitude, and the whole point of the new arm is to be the ONE thing
    # that differs. MEASURED, left (resting) hand: 0.101 m above the pelvis
    # line — close against the body, not reaching, which is the contrast that
    # makes the right arm read as reaching.
    "buried_reach": {
        "thigh_r": ((1.0, 0.0, 0.0), -19.0),
        "calf_r": ((1.0, 0.0, 0.0), 72.0),
        "thigh_r+": ((0.0, 1.0, 0.0), 12.0),
        "thigh_l": ((1.0, 0.0, 0.0), -4.0),
        "thigh_l+": ((0.0, 1.0, 0.0), -10.0),
        "spine_02": ((1.0, 0.0, 0.0), 6.0),
        "head": ((0.0, 0.0, 1.0), 24.0),
        "upperarm_r": ((0.0, 1.0, 0.0), -40.0),
        "upperarm_r+": ((1.0, 0.0, 0.0), -108.0),
        "lowerarm_r": ((1.0, 0.0, 0.0), -16.0),
        "upperarm_l": ((0.0, 1.0, 0.0), 15.0),
        "lowerarm_l": ((0.0, 1.0, 0.0), 50.0),
    },
}

# Metres to ADD to a placement's z so the pose's own support surface lands on
# the z the caller named. Necessary because a pose is authored as joint
# rotations about a FIXED root: the pelvis does not move, so bending the legs
# lifts the feet off the ground instead of lowering the body onto it, and a
# figure posed `sit_ground` at its standing z floats with its seat at hip
# height.
#
# THE SUPPORT SURFACE DIFFERS BY POSE, and that is the whole contract:
#
#   idle / walk          the SOLES. Nothing moves below the hip, so 0.
#   sit_ground           the SEAT, on the ground. The posed soles end 0.15 m
#                        below the pelvis (see the pose), so dropping the
#                        pelvis to 0.15 puts seat and heels down together:
#                        0.15 - 0.95 = -0.80.
#   sit_edge / seated_car the SEAT, on whatever the caller is sitting them on
#                        — a kerb, a coping, a car seat. The caller passes the
#                        SEAT-TOP z; -0.85 drops the pelvis to 0.10 above it
#                        (the buttock mesh hangs ~0.10 below the pelvis JOINT,
#                        so 0.10 is contact, not float) and the legs then hang
#                        0.50 m below the seat.
#   crouch               the SOLES again, but they have risen 0.61 m (see the
#                        pose), so -0.61 puts them back.
#
# Values are for the 1.80 m mannequin the poses were derived on. Scale them by
# `measured_height / 1.80` — `disaster.people._pose_dz` does — because the pack
# runs 1.73-1.86 m and an unscaled offset buries the short characters.
POSE_REF_HEIGHT_M = 1.80

# WHERE THE HIP HAS TO LAND for the poses whose support is the GROUND, in
# metres above it. These replace a scaled `_POSE_Z_OFFSET` because that scales
# by TOTAL HEIGHT, and hip height does not track it across this pack: sophia is
# 1.784 m on a 0.899 m hip, carla 1.731 m on a 0.983 m hip — a 12% spread in
# the ratio, which is +-7 cm on a seated figure. One global number cannot serve
# all six rigs, which is what "THE MALE CHARACTERS SIT 0.15 M HIGH" in
# `disaster/people.py` was really recording. Measuring each rig's own hip
# removes the class of error rather than retuning it per pack.
# Solve GROUND poses for the contact itself rather than for a hip height.
# Targeting the hip still floats a rig whose buttock-to-hip distance differs
# (sophia sat 6 cm proud with her hip exactly on target), because that distance
# varies too. The contact point does not: put the named part on z=0 and the
# figure is right whatever its proportions.
#   sole  the whole foot is flat on the ground (a squat)
#   heel  a toes-up sitting foot rocks back onto its heel, which sits well
#         above the sole plane — measured as a fraction of the rig's own
#         rest ankle height, so it scales with the foot.
#   knee  a kneeling figure carries its weight on the KNEE, and no amount of
#         foot arithmetic finds that: `dig_kneel`'s down shin lies flat on the
#         ground with the ankle at the same height as the knee, so a sole or
#         heel solve would seat the shin and leave the knee 6 cm under it.
_POSE_GROUND_CONTACT = {
    "sit_ground": "heel",
    "crouch": "sole",
    # ---- the 2026-08-27 survivor poses. `lying_supine` / `lying_prone` are
    # deliberately absent: those are placed by the prone branch of
    # `disaster.people._human_placement`, which seats the body on half its
    # measured DEPTH and never calls `pose_z_offset` at all.
    "dig_bent": "sole",
    "dig_kneel": "knee",
    "sit_slump": "sole",
    "stand_slump": "sole",
    "trapped_sit": "heel",
}

# WHICH JOINT each contact is measured at, and how far the flesh extends BELOW
# that joint at the moment of contact, as a multiple of the rig's own rest
# ankle height. The ankle is the scale reference because it is the one
# joint-to-surface distance the rig states for itself: a character stands on
# the ground at rest, so its rest ankle height IS its ankle-to-sole distance,
# and it runs 0.090-0.148 m across this pack.
#
#   sole  1.00  the whole measured ankle-to-sole distance. Exact, not modelled.
#   heel  0.45  a toes-up sitting foot rocks back onto its heel, which stands
#               well above the sole plane.
#   knee  0.50  MODELLED, and the only number here that is. A kneeling knee
#               puts roughly 0.055-0.07 m of patella and flesh under the joint
#               centre on an adult; against a 0.09-0.148 m ankle height that is
#               almost exactly half, and scaling it off the ankle keeps it
#               per-rig rather than making it one more global constant that
#               fits the 1.73 m character and buries the 1.86 m one.
_CONTACT_JOINT = {
    "sole": ("foot_l", 1.00),
    "heel": ("foot_l", 0.45),
    "knee": ("calf_l", 0.50),
}
_HEEL_FRAC = 0.45

_RIG_HIP_CACHE: dict = {}


def rig_hip_height(usd: str):
    """Rest height of a rig's pelvis above its soles, metres, or None.

    None whenever the asset cannot be opened — the host-side planner runs with
    no Nucleus, and a pose plan there must still work off the legacy table
    rather than raise.
    """
    if usd in _RIG_HIP_CACHE:
        return _RIG_HIP_CACHE[usd]
    hip = None
    try:
        stage = Usd.Stage.Open(usd)
        if stage is not None:
            prim = next((p for p in stage.Traverse()
                         if p.GetTypeName() == "Skeleton"), None)
            if prim is not None:
                data = _read_skeleton(usd)
                if data is not None:
                    joints, rest = data
                    leaf = [str(j).rsplit("/", 1)[-1] for j in joints]
                    if "pelvis" in leaf:
                        _l, world = _pose_joint_transforms(joints, rest, {})
                        mpu = UsdGeom.GetStageMetersPerUnit(stage) or 0.01
                        hip = world[leaf.index("pelvis")].ExtractTranslation()[2] * mpu
    except Exception:                                  # a bad rig is data
        hip = None
    _RIG_HIP_CACHE[usd] = hip
    return hip


def _ground_contact_drop(usd: str, pose: str, contact: str):
    """Drop that puts *pose*'s contact point on z=0 for THIS rig, or None.

    At rest the character stands on the ground, so its rest ankle height IS
    that rig's ankle-to-sole distance — and it runs 0.090 to 0.148 m across
    this pack, which is most of why one global offset never fitted. THE ANKLE
    IS THE RULER even when the contact is somewhere else: `_CONTACT_JOINT`
    names the joint whose posed height is being solved for, and the flesh
    beneath it as a multiple of that same measured ankle height, so a kneeling
    figure is seated in the rig's own units rather than in a constant.
    """
    data = _read_skeleton(usd)
    deltas = _HUMAN_POSES.get(pose)
    joint, frac = _CONTACT_JOINT.get(contact, (None, None))
    if data is None or not deltas or joint is None:
        return None
    joints, rest = data
    leaf = [str(j).rsplit("/", 1)[-1] for j in joints]
    if "foot_l" not in leaf or joint not in leaf:
        return None
    try:
        stage = Usd.Stage.Open(usd)
        mpu = (UsdGeom.GetStageMetersPerUnit(stage) or 0.01) if stage else 0.01
        i, ia = leaf.index(joint), leaf.index("foot_l")
        _l0, rest_w = _pose_joint_transforms(joints, rest, {})
        _l1, posed_w = _pose_joint_transforms(joints, rest, deltas)
        ankle_rest = rest_w[ia].ExtractTranslation()[2] * mpu
        contact_posed = posed_w[i].ExtractTranslation()[2] * mpu
    except Exception:
        return None
    return -(contact_posed - ankle_rest * float(frac))


def pose_z_offset(usd: str, pose: str, height_m: float) -> float:
    """Metres to drop a posed figure so its support reaches the surface.

    Ground poses are solved against the RIG's own measured hip; everything else
    falls back to `_POSE_Z_OFFSET` scaled by stature, which is right for a pose
    the caller places on a seat.
    """
    if not pose:
        return 0.0
    _check_pose(pose)
    contact = _POSE_GROUND_CONTACT.get(pose)
    if contact is not None:
        drop = _ground_contact_drop(usd, pose, contact)
        if drop is not None:
            return drop
    off = float(_POSE_Z_OFFSET.get(pose, 0.0))
    if off == 0.0:
        return 0.0
    ref = POSE_REF_HEIGHT_M
    return off * (float(height_m) / ref if ref > 0 else 1.0)

_POSE_Z_OFFSET = {
    "idle": 0.0,
    "walk": 0.0,
    # Hip 0.13 m up seated / 0.57 m in a squat, measured on the 1.731 m rig and
    # scaled into this table's 1.80 m reference by _pose_dz.
    # -0.88 grounds the 1.73 m rig but buries the tall males: hip height does
    # NOT track total height here (sophia is 1.784 m on a 0.899 m hip, carla
    # 1.731 m on a 0.983 m hip), so _pose_dz's height scaling cannot serve all
    # six. -0.85 is the value that puts every rig in tools/pose_check.py's
    # band. The structural fix is to derive the drop from each rig's MEASURED
    # hip rather than from its height — see that tool's header.
    "sit_ground": -0.85,
    "sit_edge": -0.85,
    "seated_car": -0.85,
    "seated_car_arms_down": -0.85,
    "crouch": -0.43,
    # ---- THE 2026-08-27 SURVIVOR POSES, as FALLBACKS ONLY. Every one of them
    # is in `_POSE_GROUND_CONTACT`, so on a machine that can open the rig the
    # solved drop wins and none of these numbers is ever read. They are what a
    # HOST-SIDE plan gets — `tools/tornado_png.py`, the dry runs, the test
    # suite — where there is no Nucleus and `_read_skeleton` returns None, and
    # a plan whose z values were all zero would hide exactly the floating and
    # sinking these poses have to be checked for.
    #
    # Derived on rp_carla (pelvis 0.983, thigh 0.472, calf 0.364, ankle 0.147)
    # and converted into this table's 1.80 m reference by 1.80 / 1.731 = 1.040,
    # since `pose_z_offset` scales them back down by the character's stature:
    #   dig_bent      ankle 0.718 below the hip vs 0.836 at rest  -0.118 -> -0.12
    #   dig_kneel     knee 0.512 up, 0.074 of flesh under it      -0.438 -> -0.46
    #   sit_slump     ankle 0.979 up, sole solve                  -0.832 -> -0.87
    #   stand_slump   ankle 0.823 below the hip                   -0.013 -> -0.01
    #   trapped_sit   ankle 0.922 up, heel solve at 0.066         -0.856 -> -0.89
    "dig_bent": -0.12,
    "dig_kneel": -0.46,
    "sit_slump": -0.87,
    "stand_slump": -0.01,
    "trapped_sit": -0.89,
    # A LYING FIGURE IS NOT DROPPED AT ALL. The prone branch of
    # `disaster.people._human_placement` computes its own z from half the
    # measured body depth and never asks for an offset; these are here so the
    # table lists every pose rather than so anything reads them.
    "lying_supine": 0.0,
    "lying_prone": 0.0,
    # The two assist poses are `walk` with an arm and a lean, and like `walk`
    # they scissor the legs and lift a heel: anchored at the soles, no drop.
    "support_l": 0.0,
    "support_r": 0.0,
    # ---- THE 2026-08-31 FIRE-VICTIM POSES. `stand_calm` / `wave_help` /
    # `lean_window` touch NO leg joint at all (only the stance-widening thigh
    # abduction, which does not move the ankle), so they are anchored at the
    # soles exactly like `idle` — MEASURED, `foot_l` unchanged from rest — and
    # not in `_POSE_GROUND_CONTACT` for the same reason `idle`/`walk` are not:
    # a contact solve on an unmoved ankle would return 0 anyway.
    "stand_calm": 0.0,
    "wave_help": 0.0,
    "lean_window": 0.0,
    # `buried_reach` is a lying pose (see `LYING_POSES`/`fire_people.LYING_ROLL`
    # for the roll that lays it down face-up); like `lying_supine`/`lying_prone`
    # its z comes from half the measured body depth, never from this table.
    "buried_reach": 0.0,
    # THROWAWAY window-pose search poses — approximate (0.0, soles-anchored)
    # rather than solved: a pelvis hinge DOES swing the whole leg with it
    # (unlike the spine-only variants, which leave the legs untouched and
    # are genuinely exact at 0.0), so the pelvis/mixed variants carry a few
    # cm of float or sink at the feet. Acceptable for a one-off pose-
    # selection bench row; not carried forward once one variant ships.
    "lw_pelvis_negx": 0.0, "lw_pelvis_posx": 0.0,
    "lw_pelvis_negy": 0.0, "lw_pelvis_posy": 0.0,
    "lw_spine_negx": 0.0, "lw_spine_posx": 0.0,
    "lw_mixed_negx": 0.0, "lw_mixed_posx": 0.0,
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


def _check_pose(pose):
    """A pose name that is not in `_HUMAN_POSES` is a BUG, not a no-op.

    `_HUMAN_POSES.get(pose)` returning None used to mean "leave the rig
    alone", which for a rigged RenderPeople character is the A-pose bind
    pose — so a typo, or a pose that has since been removed (`wave`),
    rendered a mannequin with its arms out and nobody was told. Refuse.
    """
    if pose and pose not in _HUMAN_POSES:
        raise ValueError(
            "unknown human pose %r; allowed: %s. There is deliberately no 'wave'"
            " pose any more — see _HUMAN_POSES." % (pose, sorted(_HUMAN_POSES)))


def _bind_human_pose(stage, prim, usd: str, pose: str, pose_cache: dict):
    """Author a static pose animation under the placed human *prim* (a
    reference to *usd*) and bind it to the composed Skeleton. Posed local
    transforms are computed once per (asset, pose) and cached in
    *pose_cache*; assets without a readable skeleton cache ``None`` and stay
    in their authored pose.
    """
    _check_pose(pose)
    key = (usd, pose)
    if key not in pose_cache:
        deltas = _HUMAN_POSES.get(pose)
        skel_data = _read_skeleton(usd) if deltas else None
        if skel_data is None:
            pose_cache[key] = None
        else:
            joints, rest = skel_data
            # A DELTA ON A JOINT THE RIG HAS NOT GOT IS SILENTLY DROPPED by
            # `_pose_joint_transforms`, which is the right behaviour (a pose
            # that loses its head turn is still a pose) and the wrong silence.
            # The survivor poses added 2026-08-27 drive `spine_01`, `spine_03`,
            # `neck_01` and `head` — UE4-mannequin standard, but nobody has
            # ever listed THIS pack's 95 joints — so a rig without them would
            # quietly ship a figure with a straight back and its chin up, which
            # is most of what those poses are for. Printed once per (asset,
            # pose) because the cache is only missed once.
            leaf = {str(j).rsplit("/", 1)[-1] for j in joints}
            missing = sorted({n.rstrip("+") for n in deltas} - leaf)
            if missing:
                print("[scene_gen] WARN: pose %r on %s: %d delta(s) dropped, "
                      "joint(s) not in this rig: %s"
                      % (pose, os.path.basename(str(usd)), len(missing),
                         ", ".join(missing)))
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


def apply_placements(stage,
                     placements: list,
                     parent_path: str = "/World/stage/generated",
                     scene_scale_factor: float = 1.0,
                     ground_snap=None,
                     resolver: "SizeResolver | None" = None,
                     instance_categories: "set | None" = None) -> str:
    """Write *placements* onto *stage* as USD references under *parent_path*.

    NOT INSTANCED BY DEFAULT, and this docstring used to claim otherwise. There
    is no `SetInstanceable` call anywhere in this module: every placement
    composes as its own reference, so N copies of a tree cost N x its points.
    `suburb_yards` is the only file that had this right; the README says
    "marked instanceable so the hundreds of repeated tiles share geometry",
    which is simply not what the code does. Measured consequence: the graph
    suburb plants 3,384 trees at ~55k points each = ~186M points, against the
    89.1M that OOM-killed Isaac Sim on the urban scene.

    *instance_categories* opts specific categories in. It is opt-in rather than
    the default because `generate_scene.prune_prims` DEACTIVATES SUB-PRIMS
    INSIDE placed assets — a street-name sign with a stop sign welded on, an
    exporter's leftover `CINEMA_4D_Editor` scaffolding — and USD forbids editing
    inside an instance. Any category that pass touches must stay un-instanced.
    The suburb path does not call `prune_prims` at all, so it can instance
    freely; the urban path cannot without auditing its prune list first.

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

    for i, p in enumerate(placements):
        usd = p["usd"]
        group = proto_index.setdefault(usd, len(proto_index))
        prim_path = f"{parent_path}/{p.get('category', 'asset')}_{group}_{i}"
        # No local typeName: some assets (e.g. Dmytro/Unreal exports) have a
        # Mesh as their root prim, and a local "Xform" opinion would override
        # the referenced "Mesh" type — attributes still compose (bbox looks
        # right) but the prim stops being a gprim and renders nothing. A
        # typeless def lets the referenced asset's own type win.
        prim = stage.DefinePrim(prim_path)
        p["prim_path"] = prim_path      # so callers can post-process (settling)
        if not prim.GetReferences().AddReference(usd):
            print(f"[scene_gen] WARN: failed to reference {usd} at {prim_path}")
            continue
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
        # `raw_pivot` opts a placement OUT of the centroid correction. A kit
        # assembled from many pieces is positioned in the ART's own frame — each
        # wall, door and roof already sits where the artist put it relative to
        # the others — so re-centring every piece on its own bbox pulls the
        # building apart. The correction is right for a standalone prop whose
        # pivot is arbitrary, and wrong for anything self-assembling.
        if resolver is not None and not p.get("raw_pivot"):
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
            print(f"[scene_gen] WARN: {usd} composed no Xformable prim at {prim_path}")
            continue
        # A harvested piece carries the full 4x4 it had in its source scene
        # (row-major, translation in metres); author it verbatim rather than
        # re-deriving Euler angles from it.
        if p.get("matrix"):
            m = Gf.Matrix4d(*[float(v) for row in p["matrix"] for v in row])
            for k in range(3):
                m[3][k] = m[3][k] * ssf
            xform.ClearXformOpOrder()
            xform.AddTransformOp().Set(m)
            continue
        # Instancing goes on AFTER the transform ops: the ops live on the
        # instance root, which is still editable, while the referenced geometry
        # below it becomes shared. Setting it before would make the prim's own
        # xform attributes part of the prototype.
        xform.ClearXformOpOrder()
        _set_xform_ops(xform, prim,
                       translate=((p["x_m"] - offset[0]) * ssf,
                                  (p["y_m"] - offset[1]) * ssf,
                                  (z_m - offset[2]) * ssf),
                       rotate=(roll, pitch, yaw),
                       scale=(float(p["scale"]) * stx,
                              float(p["scale"]) * sty, float(p["scale"])))
        if instance_categories and p.get("category") in instance_categories:
            # NEVER INSTANCE A GPRIM-ROOTED PLACEMENT. An asset whose
            # referenced ROOT prim is itself a Mesh puts no Mesh inside its
            # prototype — Hydra draws the prototype and the building renders
            # as NOTHING (fire-city review 2026-08-31: 5 Mesh-rooted assets
            # = 10 of 75 buildings invisible, including the block that made
            # the brownstones look missing). Xform-rooted assets instance
            # fine; gprim roots stay un-instanced.
            if prim.IsA(UsdGeom.Gprim):
                print("[scene_gen] NOT instanced (gprim-rooted asset would "
                      "render empty as a prototype): "
                      + os.path.basename(str(p.get("usd", "?"))))
            else:
                prim.SetInstanceable(True)
                n_instanced += 1

    print(f"[scene_gen] Applied {len(placements)} placements under '{parent_path}' "
          f"({len(proto_index)} unique USDs, scale_factor={ssf}"
          + (f", {n_instanced} instanced" if n_instanced else "") + ")")
    return parent_path


def _set_xform_ops(xform, prim, translate, rotate, scale):
    """Author translate / rotateXYZ / scale, MATCHING any precision already there.

    `ClearXformOpOrder` clears the op ORDER; it does not delete the op
    attributes. So when a referenced asset authors its own transform — plenty of
    library vegetation and Nucleus props do — the attribute composes onto our
    holder prim, and `AddRotateXYZOp()` then asks for its default float
    precision against an existing `double3` and USD raises:

        XformOp <...xformOp:rotateXYZ> has typeName 'double3' which does not
        match the requested precision 'PrecisionFloat'

    which aborted the whole scene mid-build. The op is only ever added at one
    precision here, so the fix is to ask the prim what is already authored and
    match it rather than assume. Behaviour is unchanged for the overwhelmingly
    common case of an asset that authors no transform at all.
    """
    def precision(attr_name, default_double):
        a = prim.GetAttribute(attr_name)
        if a and a.HasAuthoredValue():
            tn = a.GetTypeName()
            if tn == Sdf.ValueTypeNames.Double3:
                return True
            if tn == Sdf.ValueTypeNames.Float3:
                return False
        return default_double

    if precision("xformOp:translate", True):
        xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    else:
        xform.AddTranslateOp(UsdGeom.XformOp.PrecisionFloat).Set(
            Gf.Vec3f(*translate))

    if precision("xformOp:rotateXYZ", False):
        xform.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*rotate))
    else:
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotate))

    if precision("xformOp:scale", False):
        xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*scale))
    else:
        xform.AddScaleOp().Set(Gf.Vec3f(*scale))


def _make_resolver(config: dict) -> SizeResolver:
    return SizeResolver(
        asset_scale=float(config.get("asset_scale", 1.0)),
        fallback_sizes=config.get("fallback_sizes", {}),
        measure=bool(config.get("measure_usds", True)),
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
    # In-memory only — see `generate_scene_on_stage`'s note on why this is
    # never written back to a YAML file.
    config["_city_layout"] = layout
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

    Called from a launch script after the base environment is loaded. Footprints
    are measured from the referenced USDs (resolved via the omni client), so the
    layout adapts to the real prop sizes.
    """
    if isinstance(config, str):
        config = load_config(config)

    resolver = _make_resolver(config)
    placements, layout = build_city(config, resolver)
    # Stashed on the in-memory `config` dict ONLY (never written to a YAML
    # file — `disaster.ground_class.GroundClass.from_config` is the only
    # reader, at earthquake assembly time, and a preset's own YAML round-trip
    # never touches a config object that has already been through
    # `build_city`). `city_layout` carries plain tuples/dicts/lists, so even
    # if something did try to serialise it, `yaml.safe_dump` degrades it to
    # nested lists rather than raising — checked, not assumed — but the key
    # itself never reaches a serialiser in this codebase today.
    config["_city_layout"] = layout
    ground_snap = _make_physx_ground_snap() if snap_to_ground else None
    apply_placements(stage, placements, parent_path, scene_scale_factor, ground_snap,
                     resolver=resolver)
    apply_ground_planes(stage, config, layout, parent_path, scene_scale_factor)
    return placements


def reload_scene_on_stage(stage,
                          config,
                          parent_path: str = "/World/stage/generated",
                          scene_scale_factor: float = 1.0,
                          snap_to_ground: bool = False,
                          add_colliders_fn=None) -> list:
    """Clear a previously generated subtree and regenerate it in place — for
    iterating on the config *without restarting Isaac Sim*.

    Run from the Kit Script Editor (or any code holding the live stage). Removes
    everything under *parent_path*, re-reads *config* from disk, and rebuilds.
    Stop the timeline before calling if physics is running.

    Pass ``add_colliders_fn=scene_prep.add_colliders`` to apply colliders to the
    new geometry (kept as a parameter so this module stays sim-agnostic).
    """
    if stage.GetPrimAtPath(parent_path).IsValid():
        stage.RemovePrim(Sdf.Path(parent_path))
        print(f"[scene_gen] Cleared existing '{parent_path}'")

    placements = generate_scene_on_stage(
        stage, config, parent_path, scene_scale_factor, snap_to_ground)

    if add_colliders_fn is not None:
        gen = stage.GetPrimAtPath(parent_path)
        if gen.IsValid():
            add_colliders_fn(gen)

    return placements


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
