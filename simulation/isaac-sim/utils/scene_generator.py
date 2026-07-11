"""
scene_generator.py — Procedural city generator for Isaac Sim / USD.

Builds a building-first neighborhood by referencing a library of USD props
(tiles, houses, trees, streetlights). Buildings vary wildly in size, so rather
than forcing a uniform grid it packs each building into its own lot:

    Packing — sample one building per cell, then size every row/column to the
             largest building in it (cells are non-uniform). Road gutters sized
             to fit a road + sidewalks run between and around all cells.
    Step 1 — Ground: a concrete pad under each building, a road lattice in the
             gutters (intersections + straights), sidewalks ringing each cell,
             and grass filling the remaining slack inside each lot.
    Step 2 — Buildings: one per cell, centered; a configurable fraction rendered
             damaged (tilted + sunk, preferring a damaged USD when provided).
    Step 3 — Detail: trees + humans scattered on open ground; streetlights,
             fire hydrants and trash cans along block frontages (sharing an
             occupancy grid so categories never stack); traffic lights at
             intersection corners, facing the intersection.
    Step 4 — Cars + disaster: cars parked on the right-hand lane of straight
             road segments. The disaster pass (tornado aftermath) is applied
             inline: per-category topple fractions (streetlights, traffic
             lights, trash cans, fire hydrants, cars), leaning traffic lights,
             wind-scattered trash cans, prone humans, and extra car wrecks
             strewn across open ground. See the placement sections and
             ``disaster`` in the config.

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
    build_city               — Compute placements for a config (+ size resolver)
    apply_placements         — Write placements onto a USD stage as references
    generate_scene_usd       — Bake a standalone .usd from a config (offline)
    generate_scene_on_stage  — Compose onto a live stage (runtime, Isaac Sim)
    reload_scene_on_stage    — Clear + regenerate in place (no sim restart)

Config schema — see config/scene_generator_config.yaml for a worked example.
"""

import math
import os
import random

from pxr import Gf, Sdf, Usd, UsdGeom


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

def load_config(config_path: str) -> dict:
    """Load and lightly validate a city-generator YAML spec.

    Raises ValueError on missing required keys so a typo fails loudly at startup
    rather than producing an empty scene.
    """
    import yaml  # lazy: keeps the import optional for callers passing dicts

    with open(config_path) as f:
        config = yaml.safe_load(f)

    if not isinstance(config, dict):
        raise ValueError(f"City config {config_path!r} did not parse to a mapping")
    if "usds" not in config:
        raise ValueError(f"City config {config_path!r} has no 'usds' library")
    if "layout" not in config or "region_m" not in config["layout"]:
        raise ValueError(f"City config {config_path!r} has no 'layout.region_m' "
                         "(city extent in meters, e.g. [200, 200])")

    usds = config["usds"]
    tiles = usds.get("tiles", {})
    if not tiles.get("grass"):
        raise ValueError("usds.tiles.grass must list at least one grass tile USD")
    if not tiles.get("road", {}).get("straight"):
        raise ValueError("usds.tiles.road.straight is required for the road network")
    if not usds.get("houses", {}).get("intact"):
        raise ValueError("usds.houses.intact must list at least one house USD")

    return config


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


def _join_asset_root(path: str, asset_root: str) -> str:
    """Prefix *asset_root* onto relative asset paths. Paths with a URL scheme
    (``omniverse://``, ``file://``, …) or a leading ``/`` / ``~`` are already
    absolute and pass through unchanged.
    """
    if not asset_root or "://" in path or path.startswith(("/", "~")):
        return path
    return asset_root.rstrip("/") + "/" + path


def _parse_usd_entry(entry, default_scale: float, asset_root: str = ""):
    """Normalize a USD entry to ``(path, scale, axis_up)``.

    Entries can be plain strings (use *default_scale*, Z-up) or dicts::

        # plain — uses global asset_scale, Z-up
        - "omniverse://host/Props/SM_Grass.usd"

        # per-asset overrides
        - usd: "omniverse://host/Props/SM_Building.usd"
          scale: 1.0      # already in meters; global asset_scale is e.g. 0.01
          axis-up: "Y"    # authored Y-up (e.g. Unity export); corrected via +90° roll

    Relative paths get *asset_root* prefixed (see :func:`_join_asset_root`).
    """
    if isinstance(entry, dict):
        return (_join_asset_root(str(entry["usd"]), asset_root),
                float(entry.get("scale", default_scale)),
                str(entry.get("axis-up", "Z")).upper())
    return _join_asset_root(str(entry), asset_root), float(default_scale), "Z"


def _normalize_usd_list(lst, default_scale: float, asset_root: str = ""):
    """Return ``(path_list, scale_overrides, axis_up_overrides)`` from a raw YAML USD list.

    *path_list* is a plain list of USD paths suitable for ``rng.choice()``.
    *scale_overrides* maps path → per-asset scale for entries with an explicit ``scale``.
    *axis_up_overrides* maps path → axis_up string ("Y") for non-Z-up entries.
    """
    paths, scale_ovr, axisup_ovr = [], {}, {}
    for entry in (lst or []):
        path, sc, au = _parse_usd_entry(entry, default_scale, asset_root)
        paths.append(path)
        if sc != default_scale:
            scale_ovr[path] = sc
        if au != "Z":
            axisup_ovr[path] = au
    return paths, scale_ovr, axisup_ovr


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

        fp = (_measure_footprint(usd_path, effective_scale, effective_axis_up)
              if self.measure else None)
        if fp is not None:
            print(f"[scene_gen] measured {category}: {os.path.basename(usd_path)} "
                  f"-> {fp['sx']:.2f} x {fp['sy']:.2f} m "
                  f"(scale={effective_scale}, up={effective_axis_up})")
        else:
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
# Procedural layout — recursive subdivision (roads) + lot parcelling
#
# Pipeline follows the standard procedural-city literature (Parish-Müller 2001;
# Vanegas 2012 "Procedural Generation of Parcels"): a road network divides the
# region into blocks, each block is recursively subdivided into lots, and a
# building is fit into each lot. Specialized here to axis-aligned rectangles so
# everything stays tileable with the square road/ground assets (no diagonals).
# ---------------------------------------------------------------------------

# Neighbor direction bits for road tile selection.
_N, _E, _S, _W = 1, 2, 4, 8


def _jitter_pos(lo: int, hi: int, jitter: float, rng) -> int:
    """Pick an integer split position in [lo, hi], biased toward the middle.
    ``jitter`` in [0, 1]: 0 = always center, 1 = anywhere in the range.
    """
    if hi <= lo:
        return lo
    mid = (lo + hi) / 2.0
    half = (hi - lo) / 2.0
    p = mid + (rng.random() * 2.0 - 1.0) * jitter * half
    return int(round(min(hi, max(lo, p))))


def _jitter_posf(lo: float, hi: float, jitter: float, rng) -> float:
    """Float version of :func:`_jitter_pos` (for metric lot splits)."""
    if hi <= lo:
        return lo
    mid = (lo + hi) / 2.0
    half = (hi - lo) / 2.0
    return min(hi, max(lo, mid + (rng.random() * 2.0 - 1.0) * jitter * half))


def _subdivide_region(nx: int, ny: int, min_block: int, max_block: int,
                      jitter: float, rng, anchors_cells=None):
    """Recursively subdivide an ``nx × ny`` cell region into blocks, reserving a
    one-cell-wide road at every split (plus a ring road around the border).

    Returns ``(road_cells, blocks)`` where ``road_cells`` is a set of ``(i, j)``
    cell coords and ``blocks`` is a list of half-open cell rects
    ``(x0, y0, x1, y1)`` of non-road interior. Irregular (jittered splits,
    probabilistic early stop) — not a uniform grid.

    *anchors_cells*, if given, is a list of ``(w, h)`` cell sizes that get
    **deterministically reserved** as standalone blocks in a row along the top
    of the interior, each ringed by its own road, before the rest of the
    interior is recursively subdivided as usual. The generic recursion is
    probabilistic and never guarantees a block of any particular size — for
    callers that need a block big enough for a specific (e.g. oversized)
    building, that guarantee has to come from an explicit reservation like
    this, not from hoping the RNG produces one.
    """
    road = set()
    for i in range(nx):
        road.add((i, 0)); road.add((i, ny - 1))
    for j in range(ny):
        road.add((0, j)); road.add((nx - 1, j))

    blocks = []

    def _recurse(x0, y0, x1, y1):
        stack = [(x0, y0, x1, y1)]
        while stack:
            x0, y0, x1, y1 = stack.pop()
            w, h = x1 - x0, y1 - y0
            if w <= 0 or h <= 0:
                continue
            can_x = w >= 2 * min_block + 1   # room for two blocks + a road column
            can_y = h >= 2 * min_block + 1
            if not (can_x or can_y):
                blocks.append((x0, y0, x1, y1))
                continue
            too_big = w > max_block or h > max_block
            if not too_big and rng.random() < 0.5:   # irregularity: stop early
                blocks.append((x0, y0, x1, y1))
                continue
            if can_x and (w >= h or not can_y):       # vertical road at column p
                p = _jitter_pos(x0 + min_block, x1 - min_block - 1, jitter, rng)
                for j in range(y0, y1):
                    road.add((p, j))
                stack.append((x0, y0, p, y1))
                stack.append((p + 1, y0, x1, y1))
            else:                                     # horizontal road at row q
                q = _jitter_pos(y0 + min_block, y1 - min_block - 1, jitter, rng)
                for i in range(x0, x1):
                    road.add((i, q))
                stack.append((x0, y0, x1, q))
                stack.append((x0, q + 1, x1, y1))

    ix0, iy0, ix1, iy1 = 1, 1, nx - 1, ny - 1

    if anchors_cells:
        row_h = max(h for _, h in anchors_cells)
        x_cur = ix0
        placed_right = ix0
        for (aw, ah) in anchors_cells:
            ax0, ax1 = x_cur, x_cur + aw
            ay0, ay1 = iy0, iy0 + row_h
            if ax1 > ix1 or ay1 > iy1:
                break                          # out of room; remaining anchors dropped
            blocks.append((ax0, ay0, ax1, ay1))
            for i in range(ax0 - 1, ax1 + 1):
                road.add((i, ay0 - 1)); road.add((i, ay1))
            for j in range(ay0 - 1, ay1 + 1):
                road.add((ax0 - 1, j)); road.add((ax1, j))
            placed_right = ax1
            x_cur = ax1 + 1
        # Leftover L-shape: a right strip (full height) + a bottom strip (under
        # the anchor row only) — together they exactly tile the rest.
        if placed_right < ix1:
            _recurse(placed_right + 1, iy0, ix1, iy1)
        if placed_right > ix0 and iy0 + row_h < iy1:
            # Bottom strip's right edge must include the last anchor's right
            # border column (placed_right), not stop before it — otherwise
            # that column is claimed by neither strip below the anchor row
            # (right strip starts at placed_right+1, bottom strip stopped at
            # placed_right), leaving an unclaimed dead column: no road, no
            # block, nothing tiled — a literal gap extending down from the
            # anchor's corner.
            _recurse(ix0, iy0 + row_h + 1, placed_right + 1, iy1)
        if placed_right == ix0:                # no anchor fit at all; fall back
            _recurse(ix0, iy0, ix1, iy1)
    else:
        _recurse(ix0, iy0, ix1, iy1)

    return road, blocks


def _subdivide_block_to_lots(rect, lot_min: float, lot_max: float,
                             alley: float, jitter: float, rng):
    """Recursively subdivide a metric block *rect* (xmin,ymin,xmax,ymax) into
    lots, leaving a thin *alley* gap between siblings.

    Returns ``(lots, alleys)`` as lists of metric rects. Buildings on lots that
    share a block are separated only by these alleys (no road between them) —
    the requested clustering behavior.
    """
    lots, alleys = [], []
    stack = [tuple(rect)]
    while stack:
        x0, y0, x1, y1 = stack.pop()
        w, h = x1 - x0, y1 - y0
        if w < lot_min or h < lot_min:
            continue                              # too thin → becomes grass
        can_x = w >= 2 * lot_min + alley
        can_y = h >= 2 * lot_min + alley
        if not (can_x or can_y):
            lots.append((x0, y0, x1, y1))
            continue
        too_big = w > lot_max or h > lot_max
        if not too_big and rng.random() < 0.4:
            lots.append((x0, y0, x1, y1))
            continue
        if can_x and (w >= h or not can_y):       # vertical alley
            p = _jitter_posf(x0 + lot_min, x1 - lot_min - alley, jitter, rng)
            alleys.append((p, y0, p + alley, y1))
            stack.append((x0, y0, p, y1))
            stack.append((p + alley, y0, x1, y1))
        else:                                     # horizontal alley
            q = _jitter_posf(y0 + lot_min, y1 - lot_min - alley, jitter, rng)
            alleys.append((x0, q, x1, q + alley))
            stack.append((x0, y0, x1, q))
            stack.append((x0, q + alley, x1, y1))
    return lots, alleys


def _road_tile_for_mask(mask: int):
    """Map a 4-bit N/E/S/W neighbor mask to ``(tile_key, base_yaw)``.

    Yaws within each family rotate consistently in 90° steps, so a single
    per-family ``orientation`` offset in the config fine-tunes the art.
    """
    n, e, s, w = bool(mask & _N), bool(mask & _E), bool(mask & _S), bool(mask & _W)
    count = n + e + s + w
    if count >= 4:
        return "four_way", 0.0
    if count == 3:                                # tee: yaw keyed on missing arm
        if not n:   return "tee", 0.0
        if not w:   return "tee", 90.0
        if not s:   return "tee", 180.0
        return "tee", 270.0                       # missing e
    if count == 2:
        if n and s: return "straight", 90.0
        if e and w: return "straight", 0.0
        if s and w: return "corner", 0.0         # adjacent pair → corner
        if e and s: return "corner", 90.0
        if n and e: return "corner", 180.0
        return "corner", 270.0                    # w and n
    if count == 1:                               # dead end: yaw keyed on the arm
        if n: return "dead_end", 0.0
        if w: return "dead_end", 90.0
        if s: return "dead_end", 180.0
        return "dead_end", 270.0                  # e
    return "four_way", 0.0                        # isolated (shouldn't occur)


# ---------------------------------------------------------------------------
# City builder
# ---------------------------------------------------------------------------

def build_city(config: dict, resolver: SizeResolver) -> list:
    """Compute all placements for the city described by *config*.

    Returns a flat list of placement dicts::

        {"usd", "x_m", "y_m", "z_m", "yaw_deg",
         "roll_deg", "pitch_deg", "scale", "category"}

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

    def _nl(lst):
        paths, sc_ov, au_ov = _normalize_usd_list(lst, asset_scale, asset_root)
        _scale_overrides.update(sc_ov)
        _axis_up_overrides.update(au_ov)
        return paths

    def _sc(path):
        """Per-asset scale, falling back to global asset_scale."""
        return _scale_overrides.get(path, asset_scale)

    def _au(path):
        """Per-asset up-axis ("Y" or "Z"), falling back to "Z"."""
        return _axis_up_overrides.get(path, "Z")

    grass_usds    = _nl(tiles["grass"])
    concrete_usds = _nl(tiles.get("concrete") or [])
    sidewalk_usds = _nl(tiles.get("sidewalk") or [])
    house_intact  = _nl(usds["houses"]["intact"])
    house_damaged = _nl(usds["houses"].get("damaged") or [])
    tree_usds     = _nl(usds.get("trees") or [])
    light_usds    = _nl(usds.get("streetlights") or [])
    car_usds      = _nl(usds.get("cars") or [])
    trash_usds    = _nl(usds.get("trash_cans") or [])
    tlight_usds   = _nl(usds.get("traffic_lights") or [])
    hydrant_usds  = _nl(usds.get("fire_hydrants") or [])
    human_usds    = _nl(usds.get("humans") or [])

    # Road dict: values may also be plain strings or {usd:, scale:, axis-up:} dicts.
    road: dict = {}
    for k, entry in (tiles.get("road") or {}).items():
        path, sc, au = _parse_usd_entry(entry, asset_scale, asset_root)
        road[k] = path
        if sc != asset_scale:
            _scale_overrides[path] = sc
        if au != "Z":
            _axis_up_overrides[path] = au

    layout = config.get("layout", {})
    region_m = layout.get("region_m", [200.0, 200.0])
    region_w_m, region_h_m = float(region_m[0]), float(region_m[1])
    min_block_m = float(layout.get("min_block_m", 24.0))
    max_block_m = float(layout.get("max_block_m", 60.0))
    split_jitter = float(layout.get("split_jitter", 0.2))
    lotcfg = layout.get("lot", {})
    lot_min_m  = float(lotcfg.get("min_m", 12.0))
    lot_max_m  = float(lotcfg.get("max_m", 28.0))
    alley_m    = float(lotcfg.get("alley_m", 2.0))
    lot_margin = float(lotcfg.get("margin_m", 1.5))
    single_lot_chance = float(lotcfg.get("single_lot_chance", 0.45))
    tile_overlap = float(layout.get("tile_overlap", 1.02))
    size_tie_tolerance = float(lotcfg.get("size_tie_tolerance", 0.1))

    # --- Resolve footprints we need for layout (pass per-asset scale) ------
    grass_fp = resolver.get(grass_usds[0], "grass", scale=_sc(grass_usds[0]))
    road_fp  = resolver.get(road["straight"], "road", scale=_sc(road["straight"]))
    house_fps = {u: resolver.get(u, "house", scale=_sc(u), axis_up=_au(u))
                 for u in house_intact + house_damaged}
    concrete_fp = (resolver.get(concrete_usds[0], "concrete", scale=_sc(concrete_usds[0]))
                   if concrete_usds else None)
    sidewalk_fp = (resolver.get(sidewalk_usds[0], "sidewalk", scale=_sc(sidewalk_usds[0]))
                   if sidewalk_usds else None)

    gx, gy = grass_fp["sx"], grass_fp["sy"]
    cell = max(road_fp["sx"], road_fp["sy"])   # road tiles are square; 1 cell = 1 road tile
    sw_w = max(sidewalk_fp["sx"], sidewalk_fp["sy"]) if sidewalk_fp is not None else 0.0

    placements: list = []

    def add(usd, x, y, z, yaw, category, scale, roll=0.0, pitch=0.0, axis_up="Z"):
        placements.append({
            "usd": usd, "x_m": x, "y_m": y, "z_m": z,
            "yaw_deg": yaw, "roll_deg": roll, "pitch_deg": pitch,
            "scale": scale, "category": category, "axis_up": axis_up,
        })

    def _axis_roll(usd):
        """+90° roll stands Y-up-authored assets upright in the Z-up world."""
        return 90.0 if _au(usd) == "Y" else 0.0

    def _tile_rect(usd_choices, rect, tsx, tsy, z, category, skip=None):
        """Fill an axis-aligned *rect* (xmin,ymin,xmax,ymax) with tiles sized
        ~(tsx, tsy), snapping the count so the fill is gap-free. Tiles whose
        center falls inside *skip* (a rect) are omitted.

        The center-to-center spacing (``stepx``/``stepy``) is stretched so the
        tile *count* divides the rect evenly, but each tile's own rendered size
        is a fixed per-asset scale that doesn't adjust to match — the two only
        agree when the rect happens to be an exact multiple of the tile size.
        Otherwise this leaves a hairline gap (spacing > tile size) or overlap
        (spacing < tile size). ``tile_overlap`` nudges every tile's scale up a
        touch so any gap becomes a slight overlap instead — z-fighting is the
        deliberately accepted tradeoff over visible cracks between tiles.
        """
        x0, y0, x1, y1 = rect
        w, h = x1 - x0, y1 - y0
        if w <= 1e-6 or h <= 1e-6:
            return
        ntx = max(1, round(w / tsx)) if tsx > 1e-6 else 1
        nty = max(1, round(h / tsy)) if tsy > 1e-6 else 1
        stepx, stepy = w / ntx, h / nty
        for ix in range(ntx):
            for iy in range(nty):
                px = x0 + (ix + 0.5) * stepx
                py = y0 + (iy + 0.5) * stepy
                if skip is not None and _in_rect(px, py, skip):
                    continue
                u = usd_choices if isinstance(usd_choices, str) else rng.choice(usd_choices)
                add(u, px, py, z, 0.0, category, _sc(u) * tile_overlap)

    # Guarantee the largest library building can always be placed somewhere,
    # regardless of how layout.lot.max_m / max_block_m are configured: a lot
    # subdivider that caps lot size below the biggest building would silently
    # turn every lot to grass (no building ever fits _fit()). Bump the
    # *effective* bounds up to the largest building's footprint + margins, and
    # bump max_block_m to match so blocks are large enough to contain such a
    # lot after the sidewalk inset.
    lot_max_m_configured = lot_max_m
    h_max = max((max(fp["sx"], fp["sy"]) for fp in house_fps.values()), default=0.0)
    need_lot = h_max + 2.0 * lot_margin
    if need_lot > lot_max_m:
        print(f"[scene_gen] WARNING: largest building needs a {need_lot:.1f}m lot but "
              f"layout.lot.max_m={lot_max_m:.1f}m — auto-raising so it can still be placed.")
        lot_max_m = need_lot + 1e-3
    need_block = need_lot + 2.0 * sw_w + 1e-3
    if need_block > max_block_m:
        max_block_m = need_block

    # Buildings too big for an ordinary subdivided lot (footprint+margin above
    # the *user-configured* lot_max_m) — these need a whole, unsplit block.
    big_buildings = [u for u in house_intact + house_damaged
                     if max(house_fps[u]["sx"], house_fps[u]["sy"]) + 2.0 * lot_margin
                     > lot_max_m_configured]

    # =======================================================================
    # ROADS — recursive subdivision of the region into blocks (irregular).
    # Work on a lattice of road-tile-sized cells so everything stays tileable.
    # =======================================================================
    nx = max(5, int(round(region_w_m / cell)))
    ny = max(5, int(round(region_h_m / cell)))
    min_block = max(1, int(round(min_block_m / cell)))
    max_block = max(min_block + 1, int(round(max_block_m / cell)))

    # Deterministically reserve a block for each oversized building — the
    # generic recursive subdivision below is probabilistic and can easily
    # never produce a block large enough by chance (verified: with a 48m
    # building it failed >50% of seeds relying on RNG luck alone). Square,
    # sized for the building's longer side so either 90° orientation fits.
    anchors_cells = sorted(
        ({"w": max(1, math.ceil((max(house_fps[u]["sx"], house_fps[u]["sy"])
                                  + 2.0 * lot_margin + 2.0 * sw_w) / cell))}
         for u in big_buildings), key=lambda a: -a["w"])
    anchors_cells = [(a["w"], a["w"]) for a in anchors_cells]
    if anchors_cells and sum(w for w, _ in anchors_cells) + len(anchors_cells) + 1 > nx - 2:
        print(f"[scene_gen] WARNING: layout.region_m too small to fit all "
              f"{len(anchors_cells)} oversized buildings side by side — some may "
              "be dropped. Increase layout.region_m.")

    road_cells, block_cells = _subdivide_region(
        nx, ny, min_block, max_block, split_jitter, rng, anchors_cells=anchors_cells)

    ox, oy = -nx * cell / 2.0, -ny * cell / 2.0     # center the region on origin

    def _cell_center(i, j):
        return ox + (i + 0.5) * cell, oy + (j + 0.5) * cell

    def _cells_to_rect(x0, y0, x1, y1):
        return (ox + x0 * cell, oy + y0 * cell, ox + x1 * cell, oy + y1 * cell)

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

    # Reserve enough of the *tightest-fitting-but-sufficient* blocks to
    # guarantee every big building gets a real placement, rather than leaving
    # it to a per-block coin flip (which, for a single rare large block, would
    # fail most runs — see single_lot_chance below for the general case).
    forced_single: set = set()
    if big_buildings:
        candidates = []
        for blk in block_cells:
            inset = _inset_of(_cells_to_rect(*blk))
            iw = (inset[2] - inset[0]) - 2.0 * lot_margin
            ih = (inset[3] - inset[1]) - 2.0 * lot_margin
            if iw > 0 and ih > 0 and any(_fit(house_fps[u], iw, ih) is not None
                                         for u in big_buildings):
                candidates.append((iw * ih, blk))
        candidates.sort(key=lambda t: t[0])         # tightest fit first
        forced_single = {blk for _, blk in candidates[:len(big_buildings)]}
        if len(candidates) < len(big_buildings):
            print(f"[scene_gen] WARNING: only {len(candidates)}/{len(big_buildings)} "
                  "blocks big enough for the largest buildings — increase "
                  "layout.region_m or layout.max_block_m to fit them all.")

    # =======================================================================
    # STEP 1 — GROUND
    # =======================================================================

    # ---- Roads via neighbor bitmask (handles arbitrary axis-aligned network).
    road_off = {
        "straight": float(orient.get("road_straight", 0.0)),
        "four_way": float(orient.get("road_four_way", 0.0)),
        "tee":      float(orient.get("road_tee", 0.0)),
        "corner":   float(orient.get("road_corner", 0.0)),
        "dead_end": float(orient.get("road_dead_end", 0.0)),
    }
    _fallback = {"corner": ["corner", "dead_end", "tee", "four_way", "straight"],
                 "dead_end": ["dead_end", "tee", "four_way", "straight"],
                 "tee": ["tee", "four_way", "straight"],
                 "four_way": ["four_way", "straight"],
                 "straight": ["straight"]}

    def _resolve_road(key):
        for k in _fallback[key]:
            if road.get(k):
                return road[k]
        return road["straight"]

    straight_cells: list = []      # (i, j, "EW"|"NS") — car placement sites (step 4)
    intersection_cells: list = []  # (i, j) of 4-way/tee cells — traffic light sites
    for (i, j) in road_cells:
        mask = ((_N if (i, j + 1) in road_cells else 0) |
                (_E if (i + 1, j) in road_cells else 0) |
                (_S if (i, j - 1) in road_cells else 0) |
                (_W if (i - 1, j) in road_cells else 0))
        key, base_yaw = _road_tile_for_mask(mask)
        if key == "straight":
            straight_cells.append((i, j, "NS" if base_yaw == 90.0 else "EW"))
        elif key in ("four_way", "tee"):
            intersection_cells.append((i, j))
        usd = _resolve_road(key)
        cx, cy = _cell_center(i, j)
        add(usd, cx, cy, 0.0, base_yaw + road_off[key], "road", _sc(usd))

    # =======================================================================
    # BLOCKS -> LOTS -> BUILDINGS, plus per-block ground (sidewalk/grass/alley)
    # =======================================================================
    house_front_yaw  = float(orient.get("house_front", 0.0))
    damaged_fraction = float(config.get("damaged_fraction", 0.0))

    house_rects: list = []      # building footprint rects (for tree avoidance)
    n_buildings = n_damaged = 0

    for blk in block_cells:
        bx0, by0, bx1, by1 = blk
        block = _cells_to_rect(bx0, by0, bx1, by1)
        x0, y0, x1, y1 = block

        # Sidewalk frontage ring (between block edge / road and the lot interior).
        inset = block
        if sidewalk_fp is not None and sw_w > 1e-6:
            _tile_rect(sidewalk_usds, (x0, y0, x1, y0 + sw_w), sw_w, sw_w, 0.015, "sidewalk")
            _tile_rect(sidewalk_usds, (x0, y1 - sw_w, x1, y1), sw_w, sw_w, 0.015, "sidewalk")
            _tile_rect(sidewalk_usds, (x0, y0 + sw_w, x0 + sw_w, y1 - sw_w), sw_w, sw_w, 0.015, "sidewalk")
            _tile_rect(sidewalk_usds, (x1 - sw_w, y0 + sw_w, x1, y1 - sw_w), sw_w, sw_w, 0.015, "sidewalk")
            inset = (x0 + sw_w, y0 + sw_w, x1 - sw_w, y1 - sw_w)

        # Grass carpets the whole interior; lots/alleys/pads draw over it.
        _tile_rect(grass_usds, inset, gx, gy, 0.0, "grass")

        # Subdivide the interior into lots separated by alleys. Recursive
        # splitting cuts one axis at a time, so a block that's only fittable as
        # a *single* lot (e.g. a large building needs more than lot_max on both
        # axes) can get sliced down to pieces far smaller than lot_max before
        # the recursion bottoms out — splitting always "wins" over preserving
        # one big lot. ``forced_single`` (computed above) guarantees big
        # buildings a real, unsplit block; smaller-but-still-whole-fitting
        # blocks get a probabilistic chance too, for variety (a big building
        # occupying an entire block is also realistic).
        iw, ih = (inset[2] - inset[0]) - 2.0 * lot_margin, (inset[3] - inset[1]) - 2.0 * lot_margin
        whole_fits = iw > 0 and ih > 0 and any(
            _fit(house_fps[u], iw, ih) is not None for u in house_intact + house_damaged)
        if blk in forced_single or (whole_fits and rng.random() < single_lot_chance):
            lots, alleys = [inset], []
        else:
            lots, alleys = _subdivide_block_to_lots(
                inset, lot_min_m, lot_max_m, alley_m, split_jitter, rng)

        if concrete_fp is not None:
            for al in alleys:
                _tile_rect(concrete_usds[0], al, concrete_fp["sx"], concrete_fp["sy"],
                           0.01, "concrete")

        # Fit a building into each lot (clustering: adjacent lots share a block,
        # separated only by alleys). Lots nothing fits in stay grass.
        for (lx0, ly0, lx1, ly1) in lots:
            aw = (lx1 - lx0) - 2.0 * lot_margin
            ah = (ly1 - ly0) - 2.0 * lot_margin
            if aw <= 0 or ah <= 0:
                continue
            is_damaged = rng.random() < damaged_fraction
            pool = house_damaged if (is_damaged and house_damaged) else house_intact
            cands = [(u, _fit(house_fps[u], aw, ah)) for u in pool]
            cands = [(u, r) for u, r in cands if r is not None]
            if not cands and pool is not house_intact:           # fall back to intact
                cands = [(u, _fit(house_fps[u], aw, ah)) for u in house_intact]
                cands = [(u, r) for u, r in cands if r is not None]
            if not cands:
                continue                                          # nothing fits → grass

            # Use as much of the lot as possible: prefer the largest building
            # (by footprint area) that fits, not a uniformly random pick — this
            # is what makes the dedicated big-building anchors self-correct (an
            # anchor sized for the largest building keeps choosing it over
            # smaller candidates that would also fit, while a smaller anchor
            # excludes the largest one via _fit and falls through correctly).
            # But within a size tier, still pick randomly — otherwise the
            # first-sorted building of a near-tied group (e.g. two buildings
            # only differing by measurement noise) would always win, starving
            # the others. "Near-tied" = within size_tie_tolerance of the
            # largest fitting candidate's footprint area.
            cands.sort(key=lambda t: -(house_fps[t[0]]["sx"] * house_fps[t[0]]["sy"]))
            max_area = house_fps[cands[0][0]]["sx"] * house_fps[cands[0][0]]["sy"]
            tied = [c for c in cands if house_fps[c[0]]["sx"] * house_fps[c[0]]["sy"]
                   >= max_area * (1.0 - size_tie_tolerance)]
            usd, rot = rng.choice(tied)
            fp = house_fps[usd]
            cx, cy = (lx0 + lx1) / 2.0, (ly0 + ly1) / 2.0
            # Y-up assets (e.g. Unity exports) need a +90° roll to stand upright
            # in Isaac Sim's Z-up world. This is combined with any damage tilt.
            axis_roll = 90.0 if _au(usd) == "Y" else 0.0
            roll = axis_roll
            pitch = 0.0
            z = fp["base"]
            if is_damaged:
                roll = axis_roll + rng.uniform(-18.0, 18.0)
                pitch = rng.uniform(-18.0, 18.0)
                z -= rng.uniform(0.3, 0.9)
                n_damaged += 1
            lot_rect = (lx0, ly0, lx1, ly1)
            if concrete_fp is not None:
                # Pave the whole lot, not just the building's own footprint —
                # reads as a paved building site/plaza rather than a tight
                # driveway-sized pad.
                _tile_rect(concrete_usds[0], lot_rect, concrete_fp["sx"], concrete_fp["sy"],
                           0.02, "concrete")
            add(usd, cx, cy, z, house_front_yaw + rot, "house", _sc(usd),
                roll=roll, pitch=pitch, axis_up=_au(usd))
            house_rects.append(lot_rect)
            n_buildings += 1

    # =======================================================================
    # STEP 3 — DETAIL: trees + humans (open-ground scatter); streetlights +
    # fire hydrants + trash cans (block frontage); traffic lights
    # (intersection corners)
    # =======================================================================

    # Disaster (tornado aftermath) parameters — consumed inline through
    # steps 3-4 (per-category toppling, leaning, scattering, strewn wrecks).
    dcfg = config.get("disaster", {})

    def _scatter_in_blocks(count_range, min_sep, margin, place_fn):
        """Rejection-sample points on each block's open ground — off building
        lots by *margin*, off exclusions, *min_sep* apart — and call
        ``place_fn(x, y)`` for each accepted point.
        """
        for (bx0, by0, bx1, by1) in block_cells:
            x0, y0, x1, y1 = _cells_to_rect(bx0, by0, bx1, by1)
            local = [hr for hr in house_rects
                     if hr[0] >= x0 - 5 and hr[2] <= x1 + 5
                     and hr[1] >= y0 - 5 and hr[3] <= y1 + 5]
            placed: list = []
            want = rng.randint(int(count_range[0]), int(count_range[1]))
            for _ in range(want * 12):
                if len(placed) >= want:
                    break
                tx, ty = rng.uniform(x0, x1), rng.uniform(y0, y1)
                if any(_in_rect(tx, ty, (hr[0] - margin, hr[1] - margin,
                                         hr[2] + margin, hr[3] + margin))
                       for hr in local):
                    continue
                if exclusions and _in_exclusion(tx, ty, exclusions):
                    continue
                if any((tx - px) ** 2 + (ty - py) ** 2 < min_sep ** 2
                       for px, py in placed):
                    continue
                place_fn(tx, ty)
                placed.append((tx, ty))

    # Frontage props (streetlights, fire hydrants, trash cans) share one 2 m
    # occupancy grid so different categories never stack on the same spot.
    frontage_seen: set = set()

    def _frontage_positions(spacing, phase=0.0, jitter_frac=0.0):
        """Yield (x, y) along every block's frontage ring at *spacing*,
        skipping already-occupied slots and exclusion zones.

        *phase* shifts every point along its edge by that fraction of a step
        (phase 0 includes both corners; phase 0.5 emits step midpoints only).
        Categories placed after the first would otherwise land exactly on the
        corners/steps the first category already claimed in the occupancy
        grid and get dropped wholesale. *jitter_frac* adds a random per-point
        slide (fraction of a step) so sparse categories look hand-placed.
        """
        for (bx0, by0, bx1, by1) in block_cells:
            x0, y0, x1, y1 = _cells_to_rect(bx0, by0, bx1, by1)
            for ax, ay, bx, by in ((x0, y0, x1, y0), (x0, y1, x1, y1),
                                   (x0, y0, x0, y1), (x1, y0, x1, y1)):
                length = math.hypot(bx - ax, by - ay)
                n = max(1, int(length / spacing))
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
                    yield px, py

    def _topple_flat(usd, fp, x, y, category):
        """Prop knocked flat: ~90° roll about its base pivot with a random
        heading; z lifts the pivot by half the footprint width so the lying
        prop rests on (not in) the ground."""
        add(usd, x, y, max(fp["sx"], fp["sy"]) / 2.0, rng.uniform(0.0, 360.0),
            category, _sc(usd),
            roll=_axis_roll(usd) + rng.choice([-1.0, 1.0]) * rng.uniform(80.0, 100.0),
            axis_up=_au(usd))

    # ---- Trees: scattered on open grass.
    if tree_usds:
        tcfg = config.get("trees", {})

        def _place_tree(tx, ty):
            usd = rng.choice(tree_usds)
            jitter = rng.uniform(0.85, 1.2)
            add(usd, tx, ty, resolver.get(usd, "tree", scale=_sc(usd))["base"],
                rng.uniform(0, 360), "tree", _sc(usd) * jitter)

        _scatter_in_blocks(tcfg.get("per_block", tcfg.get("per_cell", [1, 3])),
                           float(tcfg.get("min_separation_m", 2.5)),
                           float(tcfg.get("house_margin_m", 1.5)),
                           _place_tree)

    # ---- Streetlights: uniform spacing along frontage; some snapped flat.
    if light_usds:
        spacing = float(config.get("streetlights", {}).get("spacing_m", 20.0))
        lights_toppled = float(dcfg.get("streetlights_toppled_fraction", 0.0))
        for px, py in _frontage_positions(spacing):
            lu = rng.choice(light_usds)
            lfp = resolver.get(lu, "streetlight", scale=_sc(lu))
            if rng.random() < lights_toppled:
                _topple_flat(lu, lfp, px, py, "streetlight")
            else:
                add(lu, px, py, lfp["base"], 0.0, "streetlight", _sc(lu))

    # ---- Fire hydrants: sparse along frontage. Bolted to the water main, so
    # a tornado rarely moves them — only a small sheared-off-and-flat fraction.
    if hydrant_usds:
        spacing = float(config.get("fire_hydrants", {}).get("spacing_m", 55.0))
        sheared = float(dcfg.get("fire_hydrants_toppled_fraction", 0.0))
        for px, py in _frontage_positions(spacing, phase=0.5, jitter_frac=0.2):
            u = rng.choice(hydrant_usds)
            fp = resolver.get(u, "fire_hydrant", scale=_sc(u), axis_up=_au(u))
            if rng.random() < sheared:
                _topple_flat(u, fp, px, py, "fire_hydrant")
            else:
                add(u, px, py, fp["base"], rng.uniform(0.0, 360.0),
                    "fire_hydrant", _sc(u), roll=_axis_roll(u), axis_up=_au(u))

    # ---- Trash cans: along frontage. Light and unanchored, so tipped ones
    # are also wind-blown up to trash_cans_scatter_m from where they stood.
    if trash_usds:
        spacing = float(config.get("trash_cans", {}).get("spacing_m", 25.0))
        tipped = float(dcfg.get("trash_cans_toppled_fraction", 0.0))
        scatter = float(dcfg.get("trash_cans_scatter_m", 0.0))
        for px, py in _frontage_positions(spacing, phase=0.5, jitter_frac=0.35):
            u = rng.choice(trash_usds)
            fp = resolver.get(u, "trash_can", scale=_sc(u), axis_up=_au(u))
            if rng.random() < tipped:
                ang = rng.uniform(0.0, 2.0 * math.pi)
                d = rng.uniform(0.0, scatter)
                bx_, by_ = px + d * math.cos(ang), py + d * math.sin(ang)
                if exclusions and _in_exclusion(bx_, by_, exclusions):
                    bx_, by_ = px, py       # don't blow into a keep-out zone
                _topple_flat(u, fp, bx_, by_, "trash_can")
            else:
                add(u, px, py, fp["base"], rng.uniform(0.0, 360.0),
                    "trash_can", _sc(u), roll=_axis_roll(u), axis_up=_au(u))

    # ---- Traffic lights: one per intersection cell (probabilistic), offset
    # toward a random corner and yawed to face the intersection center.
    # Disaster: snapped flat at the base, or left standing but bent/leaning.
    if tlight_usds:
        tlcfg = config.get("traffic_lights", {})
        tl_chance = float(tlcfg.get("intersection_chance", 1.0))
        corner_frac = float(tlcfg.get("corner_offset_frac", 0.85))
        tl_front = float(orient.get("traffic_light_front", 0.0))
        tl_toppled = float(dcfg.get("traffic_lights_toppled_fraction", 0.0))
        tl_leaning = float(dcfg.get("traffic_lights_leaning_fraction", 0.0))
        lean_lo, lean_hi = dcfg.get("traffic_lights_lean_deg", [8.0, 30.0])
        corner_off = corner_frac * cell / 2.0
        for (i, j) in intersection_cells:
            if rng.random() >= tl_chance:
                continue
            ccx, ccy = _cell_center(i, j)
            dx_, dy_ = rng.choice(((1, 1), (1, -1), (-1, 1), (-1, -1)))
            x, y = ccx + dx_ * corner_off, ccy + dy_ * corner_off
            if exclusions and _in_exclusion(x, y, exclusions):
                continue
            u = rng.choice(tlight_usds)
            fp = resolver.get(u, "traffic_light", scale=_sc(u), axis_up=_au(u))
            yaw = math.degrees(math.atan2(ccy - y, ccx - x)) + tl_front
            if rng.random() < tl_toppled:
                _topple_flat(u, fp, x, y, "traffic_light")
            elif rng.random() < tl_leaning:
                tilt = rng.uniform(float(lean_lo), float(lean_hi))
                add(u, x, y, fp["base"], yaw, "traffic_light", _sc(u),
                    roll=_axis_roll(u) + rng.choice([-1.0, 1.0]) * tilt,
                    pitch=rng.choice([-1.0, 1.0]) * tilt * rng.uniform(0.0, 0.5),
                    axis_up=_au(u))
            else:
                add(u, x, y, fp["base"], yaw, "traffic_light", _sc(u),
                    roll=_axis_roll(u), axis_up=_au(u))

    # ---- Humans: scattered on open ground; a prone fraction lies flat
    # (casualties / taking cover), the rest stand with random headings.
    if human_usds:
        hcfg = config.get("humans", {})
        prone = float(dcfg.get("humans_prone_fraction", 0.0))

        def _place_human(tx, ty):
            u = rng.choice(human_usds)
            fp = resolver.get(u, "human", scale=_sc(u), axis_up=_au(u))
            if rng.random() < prone:
                _topple_flat(u, fp, tx, ty, "human")
            else:
                add(u, tx, ty, fp["base"], rng.uniform(0.0, 360.0),
                    "human", _sc(u), roll=_axis_roll(u), axis_up=_au(u))

        _scatter_in_blocks(hcfg.get("per_block", [0, 2]),
                           float(hcfg.get("min_separation_m", 2.0)),
                           float(hcfg.get("house_margin_m", 1.0)),
                           _place_human)

    # =======================================================================
    # STEP 4 — CARS (right-hand traffic on straight roads) + STREWN WRECKS
    # =======================================================================
    if car_usds:
        ccfg = config.get("cars", {})
        car_density  = float(ccfg.get("density", 0.0))
        lane_frac    = float(ccfg.get("lane_offset_frac", 0.5))
        car_yaw_jit  = float(ccfg.get("yaw_jitter_deg", 4.0))
        car_front    = float(orient.get("car_front", 0.0))
        toppled_frac = float(dcfg.get("cars_toppled_fraction", 0.0))
        # lane_offset_frac maps [0, 1] onto [road centerline, road edge].
        lane_off = lane_frac * (cell / 2.0)

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
                axis_up=_au(usd))

        # ---- Lane cars: at most one per straight road cell (cars.density),
        # offset to the right-hand side of a randomly chosen travel direction
        # (right-hand traffic), staggered along the segment.
        for (i, j, axis) in straight_cells:
            if rng.random() >= car_density:
                continue
            ccx, ccy = _cell_center(i, j)
            slide = rng.uniform(-0.35, 0.35) * cell
            heading = rng.choice((1, -1))            # +1 = E or N, -1 = W or S
            if axis == "EW":
                x, y = ccx + slide, ccy - heading * lane_off
                yaw = 0.0 if heading > 0 else 180.0
            else:
                x, y = ccx + heading * lane_off, ccy + slide
                yaw = 90.0 if heading > 0 else 270.0
            if exclusions and _in_exclusion(x, y, exclusions):
                continue
            usd = rng.choice(car_usds)
            fp = _car_fp(usd)
            if rng.random() < toppled_frac:
                _add_toppled_car(usd, fp, x, y)
            else:
                add(usd, x, y, fp["base"],
                    yaw + car_front + rng.uniform(-car_yaw_jit, car_yaw_jit),
                    "car", _sc(usd), roll=_axis_roll(usd), axis_up=_au(usd))

        # ---- Strewn wrecks: tornado-tossed cars anywhere in the region
        # (roads included), off building footprints and exclusion zones.
        strewn_range = dcfg.get("cars_strewn", [0, 0])
        strewn_topple = float(dcfg.get("strewn_topple_fraction", 0.7))
        strewn_margin = float(dcfg.get("strewn_margin_m", 1.5))
        want = rng.randint(int(strewn_range[0]), int(strewn_range[1]))
        half_w, half_h = nx * cell / 2.0, ny * cell / 2.0
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
            usd = rng.choice(car_usds)
            fp = _car_fp(usd)
            if rng.random() < strewn_topple:
                _add_toppled_car(usd, fp, x, y)
            else:
                add(usd, x, y, fp["base"], rng.uniform(0.0, 360.0), "car",
                    _sc(usd), roll=_axis_roll(usd), axis_up=_au(usd))
            strewn_pts.append((x, y))

    # --- Summary -----------------------------------------------------------
    counts: dict = {}
    for p in placements:
        counts[p["category"]] = counts.get(p["category"], 0) + 1
    print(f"[scene_gen] City region {region_w_m:.0f}x{region_h_m:.0f} m "
          f"({nx}x{ny} cells, {len(block_cells)} blocks, {len(road_cells)} road cells)")
    print(f"[scene_gen] Placements by category: {counts} "
          f"(total {len(placements)}, {n_buildings} buildings, {n_damaged} damaged)")
    return placements


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
                     resolver: "SizeResolver | None" = None) -> str:
    """Write *placements* onto *stage* as instanceable USD references under
    *parent_path*.

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
    ssf = float(scene_scale_factor)

    for i, p in enumerate(placements):
        usd = p["usd"]
        group = proto_index.setdefault(usd, len(proto_index))
        prim_path = f"{parent_path}/{p.get('category', 'asset')}_{group}_{i}"
        prim = stage.DefinePrim(prim_path, "Xform")
        if not prim.GetReferences().AddReference(usd):
            print(f"[scene_gen] WARN: failed to reference {usd} at {prim_path}")
            continue

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

        cx_off, cy_off, cz_off = 0.0, 0.0, 0.0
        if resolver is not None:
            axis_up_p = p.get("axis_up", "Z")
            fp = resolver.get(usd, p.get("category", "asset"),
                              scale=float(p["scale"]), axis_up=axis_up_p)
            cx_off = fp.get("cx", 0.0)
            cy_off = fp.get("cy", 0.0)
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
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(
            (p["x_m"] - offset[0]) * ssf,
            (p["y_m"] - offset[1]) * ssf,
            (z_m - offset[2]) * ssf))
        xform.AddRotateXYZOp().Set(Gf.Vec3f(roll, pitch, yaw))
        s = float(p["scale"])
        xform.AddScaleOp().Set(Gf.Vec3d(s, s, s))

    print(f"[scene_gen] Applied {len(placements)} placements under '{parent_path}' "
          f"({len(proto_index)} unique USDs, scale_factor={ssf})")
    return parent_path


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
        tprim = stage.DefinePrim("/World/terrain", "Xform")
        tprim.GetReferences().AddReference(terrain)

    resolver = _make_resolver(config)
    placements = build_city(config, resolver)
    apply_placements(stage, placements, "/World/stage/generated", scene_scale_factor,
                     resolver=resolver)

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
    placements = build_city(config, resolver)
    ground_snap = _make_physx_ground_snap() if snap_to_ground else None
    apply_placements(stage, placements, parent_path, scene_scale_factor, ground_snap,
                     resolver=resolver)
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
