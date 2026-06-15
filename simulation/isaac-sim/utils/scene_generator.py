"""
scene_generator.py — Procedural city generator for Isaac Sim / USD.

Builds a grid-based neighborhood by referencing a library of USD props (tiles,
houses, trees, streetlights) and laying them out in three steps:

    Step 1 — Ground: a road lattice separates an N×M grid of building cells;
             each cell is carpeted with grass, plus a concrete driveway (and an
             optional sidewalk border) around the house.
    Step 2 — Houses: one house per cell, a configurable fraction rendered as
             damaged (tilted + sunk, or swapped for a damaged USD if provided).
    Step 3 — Detail: trees scattered on open grass, streetlights at uniform
             spacing along the roads.

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
    if "grid" not in config:
        raise ValueError(f"City config {config_path!r} has no 'grid' (cols/rows)")

    usds = config["usds"]
    tiles = usds.get("tiles", {})
    if not tiles.get("grass"):
        raise ValueError("usds.tiles.grass must list at least one grass tile USD")
    if not tiles.get("road", {}).get("straight"):
        raise ValueError("usds.tiles.road.straight is required for the road lattice")
    if not usds.get("houses", {}).get("intact"):
        raise ValueError("usds.houses.intact must list at least one house USD")

    return config


# ---------------------------------------------------------------------------
# USD footprint measurement (with graceful fallback to constants)
# ---------------------------------------------------------------------------

def _measure_footprint(usd_path: str, scale: float):
    """Open *usd_path* and return its metric footprint as a dict, or None.

    Returns ``{"sx", "sy", "sz", "base"}`` where sx/sy/sz are the world-aligned
    bounding-box dimensions (meters, after *scale*) and ``base`` is the offset to
    add to Z so the asset's lowest point sits at z=0 (= -min_z * scale).
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
    return {"sx": sz[0] * scale, "sy": sz[1] * scale,
            "sz": sz[2] * scale, "base": -mn[2] * scale,
            # XY offset of the bbox center from the asset's local origin.
            # Non-zero when the pivot isn't at the visual centroid; apply_placements
            # subtracts this so the *visual* center lands at the requested position.
            "cx": (mn[0] + sz[0] / 2) * scale,
            "cy": (mn[1] + sz[1] / 2) * scale}


class SizeResolver:
    """Resolves and caches each USD's metric footprint, measuring when possible
    and falling back to per-category constants otherwise.
    """

    def __init__(self, asset_scale: float, fallback_sizes: dict, measure: bool):
        self.scale = float(asset_scale)
        self.fallback = fallback_sizes or {}
        self.measure = bool(measure)
        self._cache: dict = {}

    def get(self, usd_path: str, category: str) -> dict:
        if usd_path in self._cache:
            return self._cache[usd_path]

        fp = _measure_footprint(usd_path, self.scale) if self.measure else None
        if fp is not None:
            print(f"[scene_gen] measured {category}: {os.path.basename(usd_path)} "
                  f"-> {fp['sx']:.2f} x {fp['sy']:.2f} m")
        else:
            fb = self.fallback.get(category, [4.0, 4.0])
            fp = {"sx": float(fb[0]), "sy": float(fb[1]),
                  "sz": float(fb[2]) if len(fb) > 2 else 3.0, "base": 0.0,
                  "cx": 0.0, "cy": 0.0}  # assume centered pivot for fallbacks
            print(f"[scene_gen] fallback {category}: {os.path.basename(usd_path)} "
                  f"-> {fp['sx']:.2f} x {fp['sy']:.2f} m")

        self._cache[usd_path] = fp
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
# Grid geometry
# ---------------------------------------------------------------------------

def _axis_layout(n_cells: int, cell_m: float, road_w: float):
    """Lay out one axis: a road lane, then a cell, repeating, with a closing
    road lane. Returns (road_centers[n+1], cell_origins[n], cell_centers[n],
    total_length).
    """
    total = n_cells * cell_m + (n_cells + 1) * road_w
    x = -total / 2.0
    road_centers, cell_o, cell_c = [], [], []
    for _ in range(n_cells):
        road_centers.append(x + road_w / 2.0)
        x += road_w
        cell_o.append(x)
        cell_c.append(x + cell_m / 2.0)
        x += cell_m
    road_centers.append(x + road_w / 2.0)
    return road_centers, cell_o, cell_c, total


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
    exclusions = config.get("exclusions", [])
    orient = config.get("orientation", {})

    usds = config["usds"]
    tiles = usds.get("tiles", {})
    grass_usds = tiles["grass"]
    concrete_usds = tiles.get("concrete") or []
    sidewalk_usds = tiles.get("sidewalk") or []
    road = tiles["road"]
    house_intact = usds["houses"]["intact"]
    house_damaged = usds["houses"].get("damaged") or []
    tree_usds = usds.get("trees") or []
    light_usds = usds.get("streetlights") or []

    cols = int(config["grid"]["cols"])
    rows = int(config["grid"]["rows"])
    lot_margin = float(config["grid"].get("lot_margin_m", 2.0))

    # --- Resolve footprints we need for layout -----------------------------
    grass_fp = resolver.get(grass_usds[0], "grass")
    road_fp = resolver.get(road["straight"], "road")
    house_fps = {u: resolver.get(u, "house") for u in house_intact + house_damaged}
    concrete_fp = resolver.get(concrete_usds[0], "concrete") if concrete_usds else None
    sidewalk_fp = resolver.get(sidewalk_usds[0], "sidewalk") if sidewalk_usds else None

    gx, gy = grass_fp["sx"], grass_fp["sy"]
    road_w = max(road_fp["sx"], road_fp["sy"])
    rlen = road_fp["sx"]  # tile pitch along a straight road's run

    # Cell sized to fit the largest house + margin, snapped to a whole number
    # of grass tiles so the grass fill is gap-free.
    h_max = max(max(fp["sx"], fp["sy"]) for fp in house_fps.values())
    lot = h_max + 2.0 * lot_margin
    cell_m = math.ceil(lot / gx) * gx

    rcx, cell_ox, cell_cx, total_w = _axis_layout(cols, cell_m, road_w)
    rcy, cell_oy, cell_cy, total_h = _axis_layout(rows, cell_m, road_w)

    placements: list = []

    def add(usd, x, y, z, yaw, category, scale, roll=0.0, pitch=0.0):
        placements.append({
            "usd": usd, "x_m": x, "y_m": y, "z_m": z,
            "yaw_deg": yaw, "roll_deg": roll, "pitch_deg": pitch,
            "scale": scale, "category": category,
        })

    # =======================================================================
    # STEP 1 — GROUND
    # =======================================================================

    # ---- Roads: intersections at every lattice crossing, straights between.
    road_yaw        = float(orient.get("road_straight", 0.0))
    tee_yaw_off     = float(orient.get("road_tee",      0.0))
    corner_yaw_off  = float(orient.get("road_corner",   0.0))
    dead_yaw_off    = float(orient.get("road_dead_end", 0.0))
    four_yaw_off    = float(orient.get("road_four_way", 0.0))

    def _tee_yaw(ci, rj):
        if rj == 0:    return 0.0
        if rj == rows: return 180.0
        if ci == 0:    return 270.0
        return 90.0  # ci == cols

    def _corner_yaw(ci, rj):
        return {(0, 0): 0.0, (cols, 0): 90.0,
                (cols, rows): 180.0, (0, rows): 270.0}[(ci, rj)]

    for ci in range(cols + 1):
        for rj in range(rows + 1):
            is_corner = ci in (0, cols) and rj in (0, rows)
            edge = (ci in (0, cols)) ^ (rj in (0, rows))
            if is_corner and road.get("corner"):
                usd, yaw = road["corner"], _corner_yaw(ci, rj) + corner_yaw_off
            elif is_corner and road.get("dead_end"):
                usd, yaw = road["dead_end"], _corner_yaw(ci, rj) + dead_yaw_off
            elif (is_corner or edge) and road.get("tee"):
                usd, yaw = road["tee"], _tee_yaw(ci, rj) + tee_yaw_off
            else:
                usd, yaw = road.get("four_way", road["straight"]), four_yaw_off
            add(usd, rcx[ci], rcy[rj], 0.0, yaw + road_yaw, "road", asset_scale)

    # Straight segments between intersections (n tiles per cell span).
    n_seg = max(1, round(cell_m / rlen))
    step = cell_m / n_seg
    for ci in range(cols + 1):                       # vertical roads
        for r in range(rows):
            for k in range(n_seg):
                y = cell_oy[r] + (k + 0.5) * step
                add(road["straight"], rcx[ci], y, 0.0, road_yaw + 90.0, "road", asset_scale)
    for rj in range(rows + 1):                       # horizontal roads
        for c in range(cols):
            for k in range(n_seg):
                x = cell_ox[c] + (k + 0.5) * step
                add(road["straight"], x, rcy[rj], 0.0, road_yaw, "road", asset_scale)

    # ---- Per-cell ground: grass carpet + concrete driveway (+ sidewalk).
    house_rects: dict = {}     # (c, r) -> (xmin, ymin, xmax, ymax) of house
    drive_rects: dict = {}     # (c, r) -> driveway rect
    nx = max(1, round(cell_m / gx))
    ny = max(1, round(cell_m / gy))
    sx_step, sy_step = cell_m / nx, cell_m / ny

    for c in range(cols):
        for r in range(rows):
            x0, y0 = cell_ox[c], cell_oy[r]
            # Grass fill
            for ix in range(nx):
                for iy in range(ny):
                    add(rng.choice(grass_usds),
                        x0 + (ix + 0.5) * sx_step, y0 + (iy + 0.5) * sy_step,
                        0.0, 0.0, "grass", asset_scale)

            # Sidewalk: one-tile ring just inside the cell edge (if a USD exists)
            if sidewalk_fp is not None:
                sw = sidewalk_usds[0]
                sgx, sgy = max(0.5, sidewalk_fp["sx"]), max(0.5, sidewalk_fp["sy"])
                nsx = max(1, round(cell_m / sgx))
                inset_lo_x, inset_hi_x = x0 + sgx / 2, x0 + cell_m - sgx / 2
                inset_lo_y, inset_hi_y = y0 + sgy / 2, y0 + cell_m - sgy / 2
                for k in range(nsx):
                    fx = x0 + (k + 0.5) * (cell_m / nsx)
                    add(sw, fx, inset_lo_y, 0.01, 0.0, "sidewalk", asset_scale)
                    add(sw, fx, inset_hi_y, 0.01, 0.0, "sidewalk", asset_scale)
                    fy = y0 + (k + 0.5) * (cell_m / nsx)
                    add(sw, inset_lo_x, fy, 0.01, 0.0, "sidewalk", asset_scale)
                    add(sw, inset_hi_x, fy, 0.01, 0.0, "sidewalk", asset_scale)

    # =======================================================================
    # STEP 2 — HOUSES (one per cell, some damaged)
    # =======================================================================
    house_front_yaw = float(orient.get("house_front", 0.0))
    damaged_fraction = float(config.get("damaged_fraction", 0.0))
    all_cells = [(c, r) for c in range(cols) for r in range(rows)]
    n_damaged = int(round(damaged_fraction * len(all_cells)))
    damaged_cells = set(rng.sample(all_cells, n_damaged)) if n_damaged else set()

    for (c, r) in all_cells:
        cx, cy = cell_cx[c], cell_cy[r]
        is_damaged = (c, r) in damaged_cells

        if is_damaged and house_damaged:
            usd = rng.choice(house_damaged)
            roll = pitch = 0.0
            z = house_fps[usd]["base"]
        elif is_damaged:
            # No damaged USD provided: fake damage by tilting + sinking an
            # intact house so it reads as collapsed.
            usd = rng.choice(house_intact)
            roll = rng.uniform(-18.0, 18.0)
            pitch = rng.uniform(-18.0, 18.0)
            z = house_fps[usd]["base"] - rng.uniform(0.3, 0.9)
        else:
            usd = rng.choice(house_intact)
            roll = pitch = 0.0
            z = house_fps[usd]["base"]

        fp = house_fps[usd]
        add(usd, cx, cy, z, house_front_yaw, "house", asset_scale, roll=roll, pitch=pitch)
        house_rects[(c, r)] = (cx - fp["sx"] / 2, cy - fp["sy"] / 2,
                               cx + fp["sx"] / 2, cy + fp["sy"] / 2)

        # Concrete driveway: strip from the house's front (south) edge down to
        # the south road. Placed last so it sits over the grass.
        if concrete_fp is not None:
            ccw, cch = concrete_fp["sx"], concrete_fp["sy"]
            front_y = cy - fp["sy"] / 2
            road_edge_y = cell_oy[r]            # south road = bottom of the cell
            d = max(0.0, front_y - road_edge_y)
            nd = max(1, round(d / cch))
            dstep = d / nd if nd else 0.0
            for k in range(nd):
                add(concrete_usds[0], cx, road_edge_y + (k + 0.5) * dstep,
                    0.02, 0.0, "concrete", asset_scale)
            drive_rects[(c, r)] = (cx - ccw / 2, road_edge_y, cx + ccw / 2, front_y)

    # =======================================================================
    # STEP 3 — TREES (open grass) + STREETLIGHTS (uniform along roads)
    # =======================================================================
    if tree_usds:
        per_cell = config.get("trees", {}).get("per_cell", [1, 3])
        tree_min_sep = float(config.get("trees", {}).get("min_separation_m", 2.5))
        tree_margin = float(config.get("trees", {}).get("house_margin_m", 1.5))
        for (c, r) in all_cells:
            x0, y0 = cell_ox[c], cell_oy[r]
            hr = house_rects.get((c, r))
            dr = drive_rects.get((c, r))
            hr_exp = (hr[0] - tree_margin, hr[1] - tree_margin,
                      hr[2] + tree_margin, hr[3] + tree_margin) if hr else None
            placed: list = []
            want = rng.randint(int(per_cell[0]), int(per_cell[1]))
            for _ in range(want * 12):               # bounded rejection sampling
                if len(placed) >= want:
                    break
                tx = rng.uniform(x0 + 1.0, x0 + cell_m - 1.0)
                ty = rng.uniform(y0 + 1.0, y0 + cell_m - 1.0)
                if hr_exp and _in_rect(tx, ty, hr_exp):
                    continue
                if dr and _in_rect(tx, ty, dr):
                    continue
                if exclusions and _in_exclusion(tx, ty, exclusions):
                    continue
                if any((tx - px) ** 2 + (ty - py) ** 2 < tree_min_sep ** 2
                       for px, py in placed):
                    continue
                usd = rng.choice(tree_usds)
                jitter = rng.uniform(0.85, 1.2)
                add(usd, tx, ty, resolver.get(usd, "tree")["base"],
                    rng.uniform(0, 360), "tree", asset_scale * jitter)
                placed.append((tx, ty))

    if light_usds:
        spacing = float(config.get("streetlights", {}).get("spacing_m", 20.0))
        setback = float(config.get("streetlights", {}).get("setback_m", 1.0))
        off = road_w / 2.0 + setback
        light_base = resolver.get(light_usds[0], "streetlight")["base"]

        def _place_lights(points, x_off, y_off):
            for (lx, ly) in points:
                px, py = lx + x_off, ly + y_off
                if exclusions and _in_exclusion(px, py, exclusions):
                    continue
                add(rng.choice(light_usds), px, py, light_base, 0.0,
                    "streetlight", asset_scale)

        n_along_y = max(1, int(total_h / spacing))
        n_along_x = max(1, int(total_w / spacing))
        for ci in range(cols + 1):                   # along vertical roads
            side = off if ci < cols else -off
            pts = [(rcx[ci], -total_h / 2 + (k + 0.5) * (total_h / n_along_y))
                   for k in range(n_along_y)]
            _place_lights(pts, side, 0.0)
        for rj in range(rows + 1):                   # along horizontal roads
            side = off if rj < rows else -off
            pts = [(-total_w / 2 + (k + 0.5) * (total_w / n_along_x), rcy[rj])
                   for k in range(n_along_x)]
            _place_lights(pts, 0.0, side)

    # --- Summary -----------------------------------------------------------
    counts: dict = {}
    for p in placements:
        counts[p["category"]] = counts.get(p["category"], 0) + 1
    print(f"[scene_gen] City {cols}x{rows} cells, cell={cell_m:.1f} m, "
          f"road_w={road_w:.1f} m, total={total_w:.0f}x{total_h:.0f} m")
    print(f"[scene_gen] Placements by category: {counts} "
          f"(total {len(placements)}, {len(damaged_cells)} damaged)")
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
        prim.SetInstanceable(True)

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

        cx_off, cy_off = 0.0, 0.0
        if resolver is not None:
            fp = resolver.get(usd, p.get("category", "asset"))
            cx_off, cy_off = fp.get("cx", 0.0), fp.get("cy", 0.0)

        # Rotate the anchor->centroid offset by the same XYZ rotation USD applies
        # (X, then Y, then Z). TransformDir keeps us off Gf's row-vector layout.
        offset = Gf.Vec3d(cx_off, cy_off, 0.0)
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
