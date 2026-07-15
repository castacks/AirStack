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
             the trail), lamps and trash cans along it, plus rocks and
             plants on the grass and evenly-spread trees (parks.*); a
             configurable fraction of buildings rendered damaged
             (tilted + sunk).
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

Config schema — see config/scene_generator_config.yaml for a worked example.
"""

import math
import os
import random

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, UsdSkel, Vt


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
    # "buildings" is the current section name; "houses" accepted for
    # backward compatibility.
    bld = usds.get("buildings") or usds.get("houses") or {}
    if not bld.get("intact"):
        raise ValueError("usds.buildings.intact must list at least one building USD")

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
                               min_leftover: float, rng, first=None):
    """Greedily pack building *candidates* into *rect* (guillotine packing).

    Each step places a randomly chosen fitting building in the rect's min
    corner, then splits the remaining L-shape into two rects (cut along the
    axis with the larger leftover, so the bigger remainder stays whole) and
    recurses. Buildings touch the rect edges — a block packed this way is
    built out to its sidewalk — separated from each other by *gap*.

    *candidates* is ``[(key, sx, sy), ...]`` footprints, reusable any number of
    times. *first*, if given, is a key that must be placed before random
    picking starts (used to guarantee big buildings their reserved block).

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
        else:
            pick_from = fits
        key, bw, bh, yaw = rng.choice(pick_from)
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

    roads_cfg  = config.get("roads", {})
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
        if not (url.startswith("omniverse://") or os.path.isabs(url)):
            url = asset_root + "/" + url
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
    house_intact  = _nl(bld_cfg["intact"])
    house_damaged = _nl(bld_cfg.get("damaged") or [])
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

    roads_cfg = config.get("roads", {})
    lane_w_m = float(roads_cfg.get("lane_width_m", 3.5))

    # --- Resolve footprints (no grass/road tiles; only houses, concrete, sidewalk) ---
    house_fps = {u: resolver.get(u, "house", scale=_sc(u), axis_up=_au(u))
                 for u in house_intact + house_damaged}
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

    # Auto-raise the block size bound so the BSP can produce a block whose
    # sidewalk inset fits the largest library building.
    h_max = max((max(fp["sx"], fp["sy"]) for fp in house_fps.values()), default=0.0)
    need_block = h_max + 2.0 * sw_w + 1e-3
    if need_block > max_block_m:
        print(f"[scene_gen] WARNING: largest building needs a {need_block:.1f}m block but "
              f"layout.max_block_m={max_block_m:.1f}m — auto-raising so it can still be placed.")
        max_block_m = need_block

    # Buildings that might not fit an arbitrary (min-sized) block get a
    # dedicated anchor block reserved below. Packing picks randomly among
    # fitting candidates, so without a forced anchor the largest buildings
    # could go entirely unplaced.
    big_buildings = [u for u in house_intact + house_damaged
                     if max(house_fps[u]["sx"], house_fps[u]["sy"]) + 2.0 * sw_w
                     > min_block_m]

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
    # is guaranteed at least one placement.
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
            if iw > 0 and ih > 0 and _fit(house_fps[u], iw, ih) is not None:
                if best is None or iw * ih < best[0]:
                    best = (iw * ih, blk)
        if best is not None:
            anchor_for[best[1]] = u
        else:
            unanchored += 1
    if unanchored:
        print(f"[scene_gen] WARNING: {unanchored}/{len(big_buildings)} large "
              "buildings have no block big enough — increase layout.region_m "
              "or layout.max_block_m to fit them all.")

    # =======================================================================
    # BLOCKS -> PACKED BUILDINGS, plus per-block sidewalk
    # (grass and road asphalt are procedural planes from apply_ground_planes)
    # =======================================================================
    house_front_yaw  = float(orient.get("house_front", 0.0))
    damaged_fraction = float(config.get("damaged_fraction", 0.0))

    house_rects: list = []      # building footprint rects (for tree avoidance)
    placeholder_bldgs: list = []
    city_layout["placeholder_buildings"] = placeholder_bldgs
    park_blocks: list = []      # (block, inset) rects of blocks kept as parks
    n_buildings = n_damaged = n_parks = 0

    pack_candidates = [(u, house_fps[u]["sx"], house_fps[u]["sy"])
                       for u in house_intact + house_damaged]

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

    for blk in blocks:
        x0, y0, x1, y1 = blk

        # Sidewalk frontage ring (brick tiles between road edge and block interior).
        inset = blk
        if sidewalk_fp is not None and sw_w > 1e-6:
            _tile_rect(sidewalk_usds, (x0, y0, x1, y0 + sw_w), sw_w, sw_w, 0.015, "sidewalk")
            _tile_rect(sidewalk_usds, (x0, y1 - sw_w, x1, y1), sw_w, sw_w, 0.015, "sidewalk")
            _tile_rect(sidewalk_usds, (x0, y0 + sw_w, x0 + sw_w, y1 - sw_w), sw_w, sw_w, 0.015, "sidewalk")
            _tile_rect(sidewalk_usds, (x1 - sw_w, y0 + sw_w, x1, y1 - sw_w), sw_w, sw_w, 0.015, "sidewalk")
            inset = (x0 + sw_w, y0 + sw_w, x1 - sw_w, y1 - sw_w)

        # A fraction of blocks stay as grassy parks — this is where trees,
        # plants, rocks and humans end up (packed blocks have no open ground).
        # Trails + park furniture are laid in a dedicated pass afterwards.
        if blk not in anchor_for and rng.random() < park_chance:
            n_parks += 1
            park_blocks.append((blk, inset))
            continue

        # Packed block: pave the interior wall-to-wall (buildings and filler
        # prisms sit on pavement, built out to the sidewalk).
        if concrete_fp is not None:
            _tile_rect(concrete_usds[0], inset, concrete_fp["sx"], concrete_fp["sy"],
                       0.02, "concrete")

        placed, leftovers = _pack_block_with_buildings(
            inset, pack_candidates, bld_gap_m, ph_min_m, rng,
            first=anchor_for.get(blk))

        for (usd, cx, cy, rot, brect) in placed:
            fp = house_fps[usd]
            # Y-up assets (e.g. Unity exports) need a +90° roll to stand upright
            # in Isaac Sim's Z-up world. This is combined with any damage tilt.
            axis_roll = 90.0 if _au(usd) == "Y" else 0.0
            roll = axis_roll
            pitch = 0.0
            z = fp["base"]
            # Pre-damaged art always reads damaged; otherwise tilt + sink an
            # intact building as a stand-in with probability damaged_fraction.
            is_damaged = (usd in house_damaged or
                          (not house_damaged and rng.random() < damaged_fraction))
            if is_damaged:
                roll = axis_roll + rng.uniform(-18.0, 18.0)
                pitch = rng.uniform(-18.0, 18.0)
                z -= rng.uniform(0.3, 0.9)
                n_damaged += 1
            add(usd, cx, cy, z, house_front_yaw + rot, "house", _sc(usd),
                roll=roll, pitch=pitch, axis_up=_au(usd))
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
                    # Face back toward the trail (benches especially).
                    face_yaw = math.degrees(math.atan2(-ny_, -nx_))
                    add(u, px_, py_, grass_top + fp["base"], face_yaw, cat,
                        _sc(u), roll=_axis_roll(u), axis_up=_au(u))

    # =======================================================================
    # STEP 3 — DETAIL: trees + humans (open-ground scatter); rocks (parks);
    # streetlights + fire hydrants + trash cans (block frontage); traffic
    # lights (intersection corners)
    # =======================================================================

    # Disaster (tornado aftermath) parameters — consumed inline through
    # steps 3-4 (per-category toppling, leaning, scattering, strewn wrecks).
    dcfg = config.get("disaster", {})

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
        """
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

    # ---- Trees: parks get bare-ground trees, densely and evenly spread.
    # Street trees are potted in tree-tagged planters along the frontage —
    # see the planter passes in the street-furniture section below.
    if tree_usds and park_insets:
        tcfg = config.get("trees", {})

        def _place_tree(tx, ty):
            usd = rng.choice(tree_usds)
            jitter = rng.uniform(0.85, 1.2)
            add(usd, tx, ty,
                grass_top + resolver.get(usd, "tree", scale=_sc(usd))["base"],
                rng.uniform(0, 360), "tree", _sc(usd) * jitter)

        _scatter_in_blocks(None,
                           float(parks_cfg.get("tree_min_separation_m", 4.0)),
                           float(tcfg.get("house_margin_m", 1.5)),
                           _place_tree,
                           in_blocks=park_insets,
                           density_per_100m2=float(
                               parks_cfg.get("tree_density_per_100m2", 0.8)),
                           even=True)

    # ---- Plants (grass clumps, flowers, bushes): park grass only.
    if plant_usds and park_insets:
        pcfg = config.get("plants", {})

        def _place_plant(tx, ty):
            usd = rng.choice(plant_usds)
            jitter = rng.uniform(0.8, 1.15)
            add(usd, tx, ty,
                grass_top + resolver.get(usd, "plant", scale=_sc(usd))["base"],
                rng.uniform(0, 360), "plant", _sc(usd) * jitter,
                roll=_axis_roll(usd), axis_up=_au(usd))

        _scatter_in_blocks(pcfg.get("per_block", [4, 12]),
                           float(pcfg.get("min_separation_m", 1.0)),
                           float(pcfg.get("house_margin_m", 1.5)),
                           _place_plant,
                           in_blocks=park_insets)

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
            if rng.random() < lights_toppled:
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
            add(u, bx, by, sidewalk_top + fp["base"], street_yaw, "bench", _sc(u),
                roll=_axis_roll(u), axis_up=_au(u))

    # ---- Planters: fully on the sidewalk, close to the curb, long axis
    # along the street. Tall planters carry their greenery at soil level —
    # a fraction of the planter's height — never at the sidewalk itself.
    planters_cfg = config.get("planters", {})
    soil_frac = float(planters_cfg.get("soil_level_frac", 0.75))

    def _add_planter(px, py, street_yaw, pu):
        """Place planter *pu* inset from the curb; returns (x, y, soil_z,
        along_x, along_y, usable_len) for whatever goes inside it."""
        pfp = resolver.get(pu, "planter", scale=_sc(pu), axis_up=_au(pu))
        qx, qy = _inset_pos(px, py, street_yaw, _curb_inset(pfp))
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
            qx, qy, soil_z, _ax, _ay, _ln = _add_planter(
                px, py, street_yaw, rng.choice(tree_planter_usds))
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
            qx, qy, soil_z, ax_, ay_, ln_ = _add_planter(
                px, py, street_yaw, rng.choice(plant_planter_usds))
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
            if rng.random() < sheared:
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
            if rng.random() < tipped:
                ang = rng.uniform(0.0, 2.0 * math.pi)
                d = rng.uniform(0.0, scatter)
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
                # Pole on the sidewalk, corner_margin past the road corner.
                # At a T-junction some corners fall inside the through road —
                # try corners in random order and take the first on land.
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
                    x, y = cx_, cy_
                    break
                if x is None:
                    continue
                if exclusions and _in_exclusion(x, y, exclusions):
                    continue
                u = rng.choice(tlight_usds)
                fp = resolver.get(u, "traffic_light", scale=_sc(u), axis_up=_au(u))
                yaw = math.degrees(math.atan2(icy - y, icx - x)) + tl_front
                if rng.random() < tl_toppled:
                    _topple_flat(u, fp, x, y, "traffic_light", z0=sidewalk_top)
                elif rng.random() < tl_leaning:
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
            if rng.random() < prone:
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
                    if rng.random() < toppled_frac:
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
                    if rng.random() < toppled_frac:
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

    # --- Summary -----------------------------------------------------------
    counts: dict = {}
    for p in placements:
        counts[p["category"]] = counts.get(p["category"], 0) + 1
    print(f"[scene_gen] City region {region_w_m:.0f}×{region_h_m:.0f} m "
          f"({len(blocks)} blocks, {n_parks} parks, {len(road_corridors)} road corridors)")
    print(f"[scene_gen] Placements by category: {counts} "
          f"(total {len(placements)}, {n_buildings} buildings, {n_damaged} damaged, "
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
    pose_cache: dict = {}
    ssf = float(scene_scale_factor)

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
            print(f"[scene_gen] WARN: {usd} composed no Xformable prim at {prim_path}")
            continue
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(
            (p["x_m"] - offset[0]) * ssf,
            (p["y_m"] - offset[1]) * ssf,
            (z_m - offset[2]) * ssf))
        xform.AddRotateXYZOp().Set(Gf.Vec3f(roll, pitch, yaw))
        s = float(p["scale"])
        xform.AddScaleOp().Set(Gf.Vec3d(s * stx, s * sty, s))

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

    Called from a launch script after the base environment is loaded. Footprints
    are measured from the referenced USDs (resolved via the omni client), so the
    layout adapts to the real prop sizes.
    """
    if isinstance(config, str):
        config = load_config(config)

    resolver = _make_resolver(config)
    placements, layout = build_city(config, resolver)
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
