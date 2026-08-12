"""
suburb_lots.py — neighbourhoods, lots, houses, driveways, fences.

Additive, in the shape `districts.py` established: `build_city` is never
modified, its placement list is rewritten.

WHY A LOT PASS AT ALL
---------------------
The suburb currently runs the DOWNTOWN packing. `build_city` guillotine-packs
houses into a block wherever they fit, which is right for a city block full of
party-wall commercial buildings and wrong for a suburb, because a suburb is
LOTTED. A block is platted into parcels of similar width; each parcel carries
one house set back a consistent distance from the street, a driveway to the
kerb, and yards front and rear. Packing has no concept of any of that, and the
first thing it destroys is the CONSISTENT SETBACK — which is most of what makes
a street read as suburban. Houses at 6, 14, 9 and 21 m from the kerb read as a
storage yard for houses, not as a street.

So this pass throws away the packed houses and re-lots the block: rows along the
block faces, parcels of the neighbourhood's width, house front faces aligned on
one line, driveway out to the kerb, fences on the boundaries the neighbourhood
fences.

NEIGHBOURHOODS ARE CONTIGUOUS, NOT PER BLOCK
--------------------------------------------
A lot type rolled per block makes a large-lot block, a compact block and a
large-lot block along one street, which reads as noise rather than as variety —
the same lesson `districts` learned with its zoning map. The neighbourhood map
is therefore a jittered-grid Voronoi: seeds spaced `patch_m` apart, nearest
seed wins, so a neighbourhood is a contiguous multi-block patch with a ragged
boundary. Blocks change type at a boundary you can walk to, which is how a
subdivision boundary actually behaves — the plats were recorded separately, so
lot width and setback change at the line between them and nowhere else.

LOT DEPTH FLEXES, SETBACK DOES NOT
----------------------------------
The block depth a subdivider hands us is whatever the street network produced,
and it will not be twice the ordinance lot depth. Real platting resolves that
the same way every time: the front setback is fixed by ordinance and the SLACK
GOES IN THE BACK. So lot depth here flexes to half the block, bounded by
`lot_depth_min_m` and `lot_depth_max_m`, while `front_setback_m` is honoured
exactly. Two consequences worth stating: houses on one street line up
regardless of block depth, and a deep block gets deep back yards rather than a
second, street-less row of houses facing nothing.

Lot WIDTH flexes the other way — the face divides into a whole number of equal
parcels near `lot_width_m`, because that is what a plat does with a block
length that doesn't divide evenly, and because unequal widths read as an error.

YARDS ARE PUBLISHED, NOT RECOMPUTED
-----------------------------------
`layout["lot_yards"]` carries one record per lot: the parcel, the house
footprint, the driveway, and the front/rear yard rectangles. The planting pass
(`suburb_yards.py`) consumes that instead of re-deriving the lotting, so the
two passes cannot disagree about where a yard is. A neighbourhood with only a
front yard emits ``"rear_yard": None`` and vice versa — that is a neighbourhood
property, not an accident of geometry.
"""

import math
import random

from scene_generator import _in_exclusion, _normalize_usd_list, _stage

# Footprint slack when testing whether a house still fits its lot. Negative-
# biased on purpose: fractionally larger clips the neighbour or the drive.
_FIT_TOL_M = 0.05

# Geometry authored below its own origin (a porch step, a basement light well)
# would otherwise be lifted clear of the ground by the generator's `z = base`
# rule, floating the house. Same threshold and reasoning as districts.
_BELOW_GRADE_MAX_M = 1.2

# Paving heights. build_city lays block grass at 0.01 and its own paving at
# 0.02; a driveway is paving, so it matches, and the fence sits on the ground.
_DRIVE_Z = 0.02
_GROUND_Z = 0.0

# A yard rectangle shallower than this is not a yard, it is the gap between a
# wall and a fence. Emitting it invites the planting pass to put a tree in it.
_MIN_YARD_M = 2.5

# How far a front-loaded drive runs past the front building line: one parking
# space, 9 x 18 ft (Portland Title 33 §33.266.130.D.1). Shorter and the car
# parked on it overhangs the sidewalk; longer and it is a second driveway.
_STALL_M = 5.49


# ---------------------------------------------------------------------------
# asset library
# ---------------------------------------------------------------------------

class _Lib:
    """The `usds` pools this pass needs, with per-asset scale/axis/yaw/tags."""

    def __init__(self, config: dict):
        self._default = float(config.get("asset_scale", 1.0))
        self._root = str(config.get("asset_root", "") or "")
        self._usds = config.get("usds") or {}
        self._sc, self._au, self._yo, self._tags = {}, {}, {}, {}

    def pool(self, key, tag=None):
        raw = self._usds.get(key)
        if key in ("intact", "damaged", "destroyed"):
            bld = self._usds.get("buildings") or self._usds.get("houses") or {}
            raw = bld.get(key)
        paths, sc, au, yo, tags = _normalize_usd_list(raw, self._default,
                                                      self._root)
        self._sc.update(sc)
        self._au.update(au)
        self._yo.update(yo)
        self._tags.update(tags)
        if tag is not None:
            # An untagged asset is usable anywhere; tags narrow, they don't
            # partition. A pool of one fence with no tag must still fence.
            tagged = [p for p in paths if tag in self._tags.get(p, ())]
            if tagged:
                return tagged
            return [p for p in paths if not self._tags.get(p)]
        seen, out = set(), []
        for p in paths:
            if p not in seen:
                seen.add(p)
                out.append(p)
        return out

    def scale(self, u):
        return self._sc.get(u, self._default)

    def axis_up(self, u):
        return self._au.get(u, "Z")

    def yaw_offset(self, u):
        return self._yo.get(u, 0.0)


def _place_z(fp):
    """Ground height for a footprint — see `_BELOW_GRADE_MAX_M`."""
    return 0.0 if 0.0 < fp["base"] <= _BELOW_GRADE_MAX_M else fp["base"]


# ---------------------------------------------------------------------------
# neighbourhood map
# ---------------------------------------------------------------------------

def _pick(mix: dict, rnd: random.Random):
    total = sum(max(0.0, float(v)) for v in mix.values())
    if total <= 0.0:
        return None
    r = rnd.random() * total
    for name, w in mix.items():
        r -= max(0.0, float(w))
        if r <= 0.0:
            return name
    return list(mix)[-1]


class _Patches:
    """``patch_at(x, y) -> neighbourhood name``.

    A jittered grid of seeds, nearest seed wins. Chosen over a per-block roll
    for contiguity (see the module docstring) and over a plain grid because a
    plain grid puts every neighbourhood boundary on a straight line through the
    whole region, which no subdivision boundary does.
    """

    def __init__(self, cfg: dict, region, rng: random.Random):
        x0, y0, x1, y1 = region
        step = max(40.0, float(cfg.get("patch_m", 260.0)))
        jitter = float(cfg.get("patch_jitter", 0.35)) * step
        mix = cfg.get("mix") or {}
        self.seeds = []
        ny = max(1, int(round((y1 - y0) / step)))
        nx = max(1, int(round((x1 - x0) / step)))
        for i in range(nx):
            for j in range(ny):
                cx = x0 + (i + 0.5) * (x1 - x0) / nx
                cy = y0 + (j + 0.5) * (y1 - y0) / ny
                cx += rng.uniform(-jitter, jitter)
                cy += rng.uniform(-jitter, jitter)
                name = _pick(mix, rng)
                if name:
                    self.seeds.append((cx, cy, name))

    def __call__(self, x, y):
        if not self.seeds:
            return None
        return min(self.seeds,
                   key=lambda s: (s[0] - x) ** 2 + (s[1] - y) ** 2)[2]


def neighbourhood_field(config: dict, region=None):
    """``patch_at(x, y) -> name``. Returns a None-yielding callable when the
    pass is not configured, so callers need no feature test."""
    cfg = (config.get("suburb_lots") or {})
    nb = cfg.get("neighbourhoods") or {}
    if not (cfg.get("enabled", True) and nb and cfg.get("types")):
        return lambda x, y: None
    reg = region or config.get("region")
    if not reg:
        w, h = (config.get("region_m") or [800.0, 800.0])[:2]
        reg = (-w / 2.0, -h / 2.0, w / 2.0, h / 2.0)
    return _Patches(nb, reg, random.Random(int(config.get("seed", 0)) + 5501))


# ---------------------------------------------------------------------------
# lot frames
# ---------------------------------------------------------------------------
# Each row of lots gets a local (u, v) frame: u runs ALONG the street, v runs
# INTO the block from the property line. Every dimension below — setback, house
# depth, driveway length, yard depth — is then written once in (u, v) instead
# of four times, once per block face, which is where the sign errors live.

# side -> (U, V, yaw). V is the inward normal; the yaw convention is
# districts' (yaw 0 faces -X), which is the heading of V.
_FRAMES = {
    "S": ((1.0, 0.0), (0.0, 1.0), 90.0),
    "N": ((-1.0, 0.0), (0.0, -1.0), 270.0),
    "W": ((0.0, -1.0), (1.0, 0.0), 0.0),
    "E": ((0.0, 1.0), (-1.0, 0.0), 180.0),
}


def _origin(side, prop, lot_lo, lot_hi):
    """World position of the lot's (u=0, v=0) corner: on the property line, at
    the low-u end of the parcel."""
    px0, py0, px1, py1 = prop
    if side == "S":
        return (lot_lo, py0)
    if side == "N":
        return (lot_hi, py1)
    if side == "W":
        return (px0, lot_hi)
    return (px1, lot_lo)                        # E


def _uv_rect(side, org, u0, v0, u1, v1):
    """A (u, v) rectangle as a world AABB."""
    (ux, uy), (vx, vy), _yaw = _FRAMES[side]
    xs, ys = [], []
    for u, v in ((u0, v0), (u1, v1)):
        xs.append(org[0] + u * ux + v * vx)
        ys.append(org[1] + u * uy + v * vy)
    return (min(xs), min(ys), max(xs), max(ys))


def _uv_point(side, org, u, v):
    (ux, uy), (vx, vy), _yaw = _FRAMES[side]
    return (org[0] + u * ux + v * vx, org[1] + u * uy + v * vy)


def _rows(prop, depth_min, depth_max):
    """Split a block's property rect into lot ROWS.

    Returns ``[(side, run_lo, run_hi, depth)]`` — one row per built face.

    LOT DEPTH FOLLOWS THE BLOCK, up to a limit. A subdivider sets block depth
    FROM lot depth — a block is two lots deep — so a block deeper than twice
    the target depth means deeper lots, not a strip of no-man's-land down the
    middle. The cap stops that turning a 200 m block into 100 m back gardens;
    what is left over past the cap is genuinely unbuilt land, which real
    subdivisions do have (a drainage easement, a wooded strip).

    Only the two long faces are lotted. Lotting all four puts two houses in
    every corner facing each other across a 3 m gap; a real block gives its
    corners to the end lots' side yards, which is what leaving the short faces
    alone produces.
    """
    px0, py0, px1, py1 = prop
    w, h = px1 - px0, py1 - py0
    along_x = w >= h
    short = h if along_x else w
    run_lo, run_hi = (px0, px1) if along_x else (py0, py1)

    if short < depth_min:
        return []
    if short < 2.0 * depth_min:
        # Only one row fits. Half a lotted street beats a bare block, and the
        # back of the lots faces the far street rather than a second row of
        # houses standing 4 m from it.
        return [("S" if along_x else "W", run_lo, run_hi,
                 min(short, depth_max))]
    d = max(depth_min, min(depth_max, short / 2.0))
    lo, hi = ("S", "N") if along_x else ("W", "E")
    return [(lo, run_lo, run_hi, d), (hi, run_lo, run_hi, d)]


def _split_lots(run_lo, run_hi, want_w, min_w, needs=0.0):
    """Divide a face into whole parcels of near-equal width.

    Real platting divides the block length into a whole number of lots and
    absorbs the remainder by widening every lot slightly, rather than leaving a
    sliver at one end. Equal widths are also what makes the row read as
    deliberate: a 24, 24, 24, 9 m sequence reads as a mistake.

    *needs* is the width the narrowest house in the pool needs including its
    drive and side yards. A block face that divides into parcels too narrow to
    build on is re-divided into FEWER, WIDER ones rather than lotted and left
    empty — which is also what a subdivider does when the frontage doesn't
    divide: sell four lots instead of five.
    """
    face = run_hi - run_lo
    if face < min_w:
        return []
    n = max(1, int(round(face / max(1.0, want_w))))
    while n > 1 and (face / n < min_w or face / n < needs):
        n -= 1
    w = face / n
    return [(run_lo + i * w, run_lo + (i + 1) * w) for i in range(n)]


# ---------------------------------------------------------------------------
# fences
# ---------------------------------------------------------------------------

def _fence_run(p0, p1, mod_len, gap_m):
    """Module centres and yaw for a fence along the segment p0->p1.

    Returns ``[(x, y, yaw_deg)]``. Modules butt end to end — a fence is
    continuous, and the 2.5 m packing gap that reads as "detached houses" on a
    terrace reads as a broken fence here — and the run is centred on the
    segment so the two end gaps match.
    """
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dy)
    step = mod_len + gap_m
    if length < mod_len or step <= 1e-6:
        return []
    n = int((length + gap_m) // step)
    if n < 1:
        return []
    ux, uy = dx / length, dy / length
    yaw = math.degrees(math.atan2(dy, dx))
    start = (length - (n * step - gap_m)) / 2.0
    return [(p0[0] + ux * (start + i * step + mod_len / 2.0),
             p0[1] + uy * (start + i * step + mod_len / 2.0), yaw)
            for i in range(n)]


def _hits(x, y, mod, fp, yaw, rects, clear=0.35):
    """Would a fence module at (x, y) stand in one of *rects*?

    The module's own AABB, inflated by a hair, against the driveway rects. A
    geometric test rather than a per-lot skip interval because a boundary is
    SHARED: the rear line of one row is the rear line of the row behind it, and
    the drives that cross it belong to both.
    """
    depth = min(fp["sx"], fp["sy"])
    along = abs((yaw % 180.0)) < 45.0 or abs((yaw % 180.0) - 180.0) < 45.0
    hx = (mod if along else depth) / 2.0 + clear
    hy = (depth if along else mod) / 2.0 + clear
    return any(x + hx > r[0] and r[2] > x - hx
               and y + hy > r[1] and r[3] > y - hy for r in rects)


# ---------------------------------------------------------------------------
# the pass
# ---------------------------------------------------------------------------

def _park_blocks(layout, placements):
    """The blocks that are parks, which this pass must not lot.

    `suburb_layout` publishes its park superblocks as a module global, the same
    handshake `city_layout` uses; `districts.park_blocks` is the fallback for a
    run whose subdivider is the built-in one. "Has no buildings" is deliberately
    NOT the test — a block nothing fit is also empty, and it needs lotting.
    """
    blocks = layout.get("blocks") or []
    for mod in ("suburb_layout", "city_layout"):
        try:
            m = __import__(mod)
            got = [b for b in getattr(m, "PARK_RESERVES", []) if b in blocks]
            if got:
                return set(got)
        except ImportError:
            pass
    try:
        from detail import districts
        return set(districts.park_blocks(layout, placements))
    except ImportError:
        return set()


def _block_inset(config, resolver):
    """How far `build_city` holds block content off the block edge: the verge
    plus one sidewalk tile.

    This is the RIGHT-OF-WAY width, and it is what the property line sits
    behind — so it is also exactly how far the driveway has to run PAST the
    front lot line to reach the kerb. Recomputed here because the generator
    keeps the inset local to its packing loop; the sidewalk ring is one tile
    wide and that width is `max(tile.sx, tile.sy)`, there being no separate
    knob for it.
    """
    tiles = (config.get("usds") or {}).get("tiles") or {}
    raw = tiles.get("sidewalk") or tiles.get("brick") or []
    default = float(config.get("asset_scale", 1.0))
    paths, sc_ovr, _au, _yaw, _tags = _normalize_usd_list(
        raw, default, str(config.get("asset_root", "") or ""))
    walk = 0.0
    if paths:
        fp = resolver.get(paths[0], "sidewalk",
                          scale=sc_ovr.get(paths[0], default))
        walk = max(fp["sx"], fp["sy"])
    return float((_stage(config, "layout").get("frontage") or {}).get("verge_m", 0.0)) + walk


def _in_block(p, blk):
    return (blk[0] <= float(p["x_m"]) <= blk[2]
            and blk[1] <= float(p["y_m"]) <= blk[3])


def build(config: dict, layout: dict, placements: list, resolver, rng) -> dict:
    """Re-lot every non-park block. Returns the per-block neighbourhood map.

    Rebuilding rather than nudging: a packed house is in the wrong place AND
    the wrong size for its parcel, and there is no parcel to nudge it into
    until this pass has drawn one.

    Disaster art survives untouched — ruins from the damaged/destroyed pools
    and their debris stay where `build_city` put them, so re-lotting cannot
    quietly repair a wrecked suburb.

    WIRING: one call, after `build_city` and before `city_detail`, which is
    where `districts.remap_buildings` sits for the urban scene::

        suburb_lots.build(config, layout, placements, resolver, rng)

    It no-ops on a config without a `suburb_lots` block, so the call is safe on
    the urban preset too.
    """
    cfg = config.get("suburb_lots") or {}
    types = cfg.get("types") or {}
    if not (cfg.get("enabled", True) and types):
        return {}
    if layout.get("_suburb_lots_done"):
        return layout.get("_neighbourhood_of") or {}
    layout["_suburb_lots_done"] = True

    lib = _Lib(config)
    patch_at = neighbourhood_field(config, layout.get("region"))
    inset = _block_inset(config, resolver)
    exclusions = config.get("exclusions") or []
    parks = _park_blocks(layout, placements)
    orient = config.get("orientation") or {}
    face_off = float(orient.get("house_front", 0.0))
    car_off = float(orient.get("car_front", 0.0))

    def fp_of(usd, cat):
        return resolver.get(usd, cat, scale=lib.scale(usd),
                            axis_up=lib.axis_up(usd))

    def add(usd, cat, x, y, z, yaw, stretch=None):
        p = {"usd": usd, "x_m": x, "y_m": y, "z_m": z,
             "yaw_deg": yaw + lib.yaw_offset(usd),
             "roll_deg": 90.0 if lib.axis_up(usd) == "Y" else 0.0,
             "pitch_deg": 0.0, "scale": lib.scale(usd), "category": cat,
             "axis_up": lib.axis_up(usd)}
        if stretch is not None:
            p["stretch"] = stretch
        placements.append(p)
        return p

    # House pools, per typology name, measured once. `sy` is the frontage and
    # `sx` the depth for a house standing at yaw 0 — the same convention
    # districts' terrace rows use, and it holds in every lot frame because a
    # lot's u axis is always the asset's y after the frame yaw.
    pool_cache: dict = {}

    def houses_for(names):
        key = tuple(names)
        if key not in pool_cache:
            out, seen = [], set()
            for n in names:
                for u in lib.pool(n):
                    if u in seen:
                        continue
                    seen.add(u)
                    out.append((u, fp_of(u, "house")))
            out.sort(key=lambda e: -e[1]["sy"])
            pool_cache[key] = out
        return pool_cache[key]

    fence_cache: dict = {}

    def fence_for(tag):
        if tag not in fence_cache:
            pool = lib.pool("lot_fences", tag=tag)
            fence_cache[tag] = [(u, fp_of(u, "fence")) for u in pool]
        return fence_cache[tag]

    drive_pool = [(u, fp_of(u, "concrete")) for u in lib.pool("driveway_tiles")]
    car_pool = lib.pool("cars")

    intact = {u for u in lib.pool("intact")}
    clear_scatter = bool(cfg.get("clear_block_scatter", True))
    # build_city scattered these against ITS house positions; every one of them
    # is now standing in a lot, a driveway or a house. The planting pass replants
    # the yards, so they go rather than being carried as obstacles.
    scatter_cats = {"tree", "plant", "rock", "concrete", "car"}

    lot_records: list = []
    nb_of, counts, tally = {}, {}, {}
    fenced_segs: set = set()
    fence_jobs: list = []

    # PLAN FIRST, THEN CLEAR. A block this pass declines to lot has to come out
    # of the far side exactly as `build_city` left it — better a packed block
    # than a bare one — so nothing is deleted until it is known the block will
    # be rebuilt.
    plan = []
    for blk in layout.get("blocks", []):
        if blk in parks:
            continue
        prop = (blk[0] + inset, blk[1] + inset, blk[2] - inset, blk[3] - inset)
        if prop[2] - prop[0] < 8.0 or prop[3] - prop[1] < 8.0:
            continue
        bcx, bcy = (blk[0] + blk[2]) / 2.0, (blk[1] + blk[3]) / 2.0
        name = patch_at(bcx, bcy)
        typ = types.get(name)
        if not typ or not houses_for(typ.get("pools") or ["intact"]):
            continue
        d_want = float(typ.get("lot_depth_m", 34.0))
        rows = _rows(prop, float(typ.get("lot_depth_min_m", 18.0)),
                     float(typ.get("lot_depth_max_m", d_want * 1.5)))
        if not rows:
            continue
        plan.append((blk, name, typ, prop, rows))
        nb_of[blk] = name
        counts[name] = counts.get(name, 0) + 1

    if plan:
        doomed = [b for b, _n, _t, _p, _r in plan]

        def _clear(p):
            cat = p.get("category")
            if not ((cat == "house" and p.get("usd") in intact)
                    or (clear_scatter and cat in scatter_cats)):
                return False
            return any(_in_block(p, b) for b in doomed)

        placements[:] = [p for p in placements if not _clear(p)]

    for blk, name, typ, prop, rows in plan:
        pool = houses_for(typ.get("pools") or ["intact"])
        lot_w = float(typ.get("lot_width_m", 20.0))
        lot_w_min = float(typ.get("lot_width_min_m", lot_w * 0.75))
        setback = float(typ.get("front_setback_m", 7.6))
        side_min = float(typ.get("side_setback_m", 1.5))
        rear_min = float(typ.get("rear_setback_m", 6.0))
        drive_w = float(typ.get("driveway_width_m", 3.0))
        drive_rear = bool(typ.get("driveway_to_rear", False))
        yards = {str(y) for y in (typ.get("yards") or ["front", "rear"])}
        fcfg = typ.get("fence") or {}
        fence_chance = float(fcfg.get("chance", 0.0))
        fence_gap = float(fcfg.get("module_gap_m", 0.0))
        fence_pool = fence_for(fcfg.get("tag"))
        car_chance = float(typ.get("driveway_car_chance", 0.0))

        t = tally.setdefault(name, {"lots": 0, "houses": 0, "drives": 0,
                                    "fences": 0, "front": 0, "rear": 0})

        # What the narrowest house in the pool needs across the lot.
        needs = min(e[1]["sy"] for e in pool) + drive_w + 2.0 * side_min

        for side, run_lo, run_hi, row_d in rows:
            last_usd = None
            for lot_lo, lot_hi in _split_lots(run_lo, run_hi, lot_w, lot_w_min,
                                              needs):
                width = lot_hi - lot_lo
                org = _origin(side, prop, lot_lo, lot_hi)
                rec = {"block": tuple(blk), "neighbourhood": name,
                       "lot": _uv_rect(side, org, 0.0, 0.0, width, row_d),
                       "street_side": side, "facing_deg": _FRAMES[side][2] + face_off,
                       "house": None, "driveway": None,
                       "front_yard": None, "rear_yard": None,
                       "fenced": False, "fence_sides": []}
                t["lots"] += 1

                # The house has to leave room for the drive AND a side yard on
                # both sides; a house that does not is not this lot's house.
                max_wu = width - drive_w - 2.0 * side_min + _FIT_TOL_M
                max_dv = row_d - setback - rear_min + _FIT_TOL_M
                fits = [e for e in pool
                        if e[1]["sy"] <= max_wu and e[1]["sx"] <= max_dv]
                if not fits:
                    lot_records.append(rec)
                    continue
                # Two identical houses side by side is the one repeat a street
                # never survives; anything beyond that is fine and real.
                choose = [e for e in fits if e[0] != last_usd] or fits
                usd, fp = choose[rng.randrange(len(choose))]
                last_usd = usd
                hw, hd = fp["sy"], fp["sx"]

                # Drive on the low-u or high-u side, held to one side per lot
                # so two neighbouring drives sometimes pair up at the shared
                # boundary — which is what a real street does.
                drive_lo = rng.random() < 0.5
                if drive_lo:
                    du0, du1 = side_min * 0.5, side_min * 0.5 + drive_w
                    hu0 = du1 + side_min
                else:
                    du1 = width - side_min * 0.5
                    du0 = du1 - drive_w
                    hu0 = du0 - side_min - hw
                hu1 = hu0 + hw

                # THE SETBACK LINE: the house FRONT FACE sits at `setback`, not
                # its centre. Houses of different depth then still line up,
                # which is the whole point of the pass.
                hv0, hv1 = setback, setback + hd
                rec["house"] = _uv_rect(side, org, hu0, hv0, hu1, hv1)
                cx, cy = _uv_point(side, org, (hu0 + hu1) / 2.0,
                                   (hv0 + hv1) / 2.0)
                if exclusions and _in_exclusion(cx, cy, exclusions):
                    lot_records.append(rec)
                    continue
                add(usd, "house", cx, cy, _place_z(fp),
                    _FRAMES[side][2] + face_off)
                t["houses"] += 1

                # DRIVEWAY: from the kerb (the block edge, v = -inset) up the
                # side yard. build_city's own drive stops at the sidewalk's
                # inner edge and so never actually meets the road.
                dv1 = (row_d if drive_rear and "rear" in yards
                       else hv0 + min(hd, _STALL_M))
                drect = _uv_rect(side, org, du0, -inset, du1, dv1)
                if drive_pool and drect[2] - drect[0] > 0.5 \
                        and drect[3] - drect[1] > 0.5:
                    _tile(add, rng, drive_pool, drect)
                    rec["driveway"] = drect
                    t["drives"] += 1
                    if car_pool and rng.random() < car_chance:
                        ccx, ccy = _uv_point(side, org, (du0 + du1) / 2.0,
                                             (max(0.0, dv1) + hv0) / 2.0)
                        cu = car_pool[rng.randrange(len(car_pool))]
                        cfp = fp_of(cu, "car")
                        # Nose-in, along the drive: the frame yaw IS the
                        # heading of the inward normal, which is the direction
                        # a car in a driveway points.
                        add(cu, "car", ccx, ccy, cfp["base"],
                            _FRAMES[side][2] + car_off)

                # YARDS. The front yard excludes the drive, so the planting
                # pass cannot plant a shrub on the tarmac; the rear yard
                # excludes it only when the drive runs through to the back.
                fu0 = du1 if drive_lo else 0.0
                fu1 = width if drive_lo else du0
                if "front" in yards and setback >= _MIN_YARD_M \
                        and fu1 - fu0 >= _MIN_YARD_M:
                    rec["front_yard"] = _uv_rect(side, org, fu0, 0.0, fu1,
                                                 setback)
                    t["front"] += 1
                ru0, ru1 = (fu0, fu1) if drive_rear else (0.0, width)
                if "rear" in yards and row_d - hv1 >= _MIN_YARD_M \
                        and ru1 - ru0 >= _MIN_YARD_M:
                    rec["rear_yard"] = _uv_rect(side, org, ru0, hv1, ru1, row_d)
                    t["rear"] += 1

                # FENCES. Which boundary is fenced is the neighbourhood's
                # choice, and the boundaries are not interchangeable — a front
                # fence is a low run along the street, a rear fence is a tall
                # one behind the house.
                #
                # THE RUNS ARE DEFERRED. A boundary is shared — both
                # back-to-back rows own the rear line, both neighbours own the
                # side line — so it is laid ONCE, and it cannot be laid until
                # every driveway that crosses it is known. Collecting the runs
                # and laying them after all the lots is what lets a single rear
                # fence open for the drives of both rows behind it.
                if fence_pool and rng.random() < fence_chance:
                    fu, ffp = fence_pool[rng.randrange(len(fence_pool))]
                    want = []
                    if fcfg.get("front"):
                        want.append(("front", (0.0, 0.0), (width, 0.0)))
                    if fcfg.get("side"):
                        # From the FRONT BUILDING LINE back. Running it out to
                        # the street would fence the front garden off from the
                        # neighbour's, which is not what a suburb looks like.
                        want.append(("side", (0.0, hv0), (0.0, row_d)))
                        want.append(("side", (width, hv0), (width, row_d)))
                    if fcfg.get("rear"):
                        want.append(("rear", (0.0, row_d), (width, row_d)))
                    for tag, a, b in want:
                        p0 = _uv_point(side, org, *a)
                        p1 = _uv_point(side, org, *b)
                        key = _seg_key(p0, p1)
                        if key in fenced_segs:
                            continue
                        fenced_segs.add(key)
                        fence_jobs.append((p0, p1, fu, ffp, fence_gap, name,
                                           rec, tag))
                lot_records.append(rec)

    # Fences, now that every driveway is known. A module is dropped where it
    # would stand in a driveway — the gate, in effect — which is the one fence
    # artefact that is impossible to miss from the ground.
    drives = [r["driveway"] for r in lot_records if r["driveway"]]
    for p0, p1, fu, ffp, gap, nb, rec, tag in fence_jobs:
        mod = _mod_len(ffp, lib.yaw_offset(fu))
        n = 0
        for x, y, yaw in _fence_run(p0, p1, mod, gap):
            if _hits(x, y, mod, ffp, yaw, drives):
                continue
            add(fu, "lot_fence", x, y, _GROUND_Z + ffp["base"], yaw)
            n += 1
        if n:
            rec["fenced"] = True
            if tag not in rec["fence_sides"]:
                rec["fence_sides"].append(tag)
            tally[nb]["fences"] += n

    layout["lot_yards"] = lot_records
    layout["_neighbourhood_of"] = nb_of

    n_h = sum(v["houses"] for v in tally.values())
    print(f"[suburb_lots] lotted {len(nb_of)} blocks -> {len(lot_records)} "
          f"lots, {n_h} houses")
    for name in sorted(types):
        v = tally.get(name) or {}
        note = ""
        if name not in counts:
            note = "  <- no block in this neighbourhood"
        elif not v.get("houses"):
            note = "  <- lotted but nothing fit the parcel"
        print(f"[suburb_lots]   {name:12s} blocks={counts.get(name, 0):2d} "
              f"lots={v.get('lots', 0):3d} houses={v.get('houses', 0):3d} "
              f"drives={v.get('drives', 0):3d} fence_mods={v.get('fences', 0):4d} "
              f"front_yards={v.get('front', 0):3d} "
              f"rear_yards={v.get('rear', 0):3d}{note}")
    return nb_of


def _seg_key(p0, p1):
    """Quantised identity for a shared boundary, so two back-to-back rows do
    not both fence the line between them."""
    return tuple(round(v, 1) for v in (min(p0[0], p1[0]), min(p0[1], p1[1]),
                                       max(p0[0], p1[0]), max(p0[1], p1[1])))


def _mod_len(fp, yaw_offset):
    """The fence module's extent ALONG the run.

    The pass asks for a run along +X and the asset's `yaw-offset` turns its art
    to match, so which measured axis ends up along the run depends on that
    offset — the park railing is authored running along +Y with `yaw-offset:
    90`, and reading `sx` for it would give the 0.33 m rail thickness as the
    module length and lay forty times too many.
    """
    turned = abs((float(yaw_offset) % 180.0) - 90.0) < 45.0
    return max(0.3, fp["sy"] if turned else fp["sx"])


def _tile(add, rng, pool, rect):
    """Pave *rect* gap-free.

    Same trick as `build_city`'s tiler: pick a tile COUNT that divides the rect
    and stretch each tile to exactly cover its step, so the fill has no seam
    however the driveway width relates to the authored tile size. A driveway is
    3 m wide and no tile in the library is.
    """
    x0, y0, x1, y1 = rect
    w, h = x1 - x0, y1 - y0
    usd, fp = pool[rng.randrange(len(pool))]
    tsx, tsy = max(0.5, fp["sx"]), max(0.5, fp["sy"])
    nx, ny = max(1, round(w / tsx)), max(1, round(h / tsy))
    stepx, stepy = w / nx, h / ny
    st = (stepx / tsx, stepy / tsy)
    for ix in range(nx):
        for iy in range(ny):
            add(usd, "driveway", x0 + (ix + 0.5) * stepx,
                y0 + (iy + 0.5) * stepy, _DRIVE_Z + fp["base"], 0.0,
                stretch=st)
