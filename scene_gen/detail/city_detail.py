"""
city_detail.py — street furniture placed against sidewalk zones.

Additive counterpart to `scene_generator.build_city`'s built-in frontage passes.
Nothing here edits the generator; `generate_scene.py` calls `build()` after
`build_city` and appends the result to the same placement list, and the scene
config zeroes the built-in spacings so there is exactly one owner of sidewalk
props.

WHY A SEPARATE PASS
-------------------
The built-in passes put every category on one line at the kerb: each insets by
`_curb_inset(fp)` and they all share a single 2 m occupancy grid
(`frontage_seen`). Categories placed later land on cells earlier ones already
claimed and get dropped wholesale — the `phase` argument exists to work around
exactly that. With a dozen categories the line is simply full.

This module uses the standard four-zone sidewalk section instead (NACTO), from
the kerb inward:

    kerb │ curb │ furnishing │ pedestrian through │ frontage │ building

Furniture belongs in the *furnishing* zone against the kerb; things that belong
to the building (café seating, A-boards) belong in the *frontage* zone; and the
*through* zone is deliberately left empty, which is both the realism tell and
what keeps the sidewalk walkable for a ground robot. A fifth pseudo-zone,
`interior`, is not a sidewalk zone at all — it is the service strip behind the
frontage, for the props (dumpsters) that a real street keeps off its pavement.

ORIENTATION
-----------
Yaw is derived from the *measured footprint*, never from "the art faces +X".
Whichever footprint axis is shorter is the one turned across the pavement: that
is true of a bench, a bus shelter and a sign panel alike, and it is also what
keeps the mesh out of the carriageway. Only which *end* of that axis looks at
the street is art rather than geometry, and that is what the per-category
orientation mode picks. See `_prop_yaw`.

FOOTPRINTS
----------
Occupancy is a bbox reservation, not a point grid — a point grid gives a 6 m
bench the same single cell as a bollard, so the two land inside each other.
Every prop reserves the part of itself that stands on the pavement (the pole,
not the lamp arm; the trunk, not the crown) and the inset from the kerb is at
least half that depth, so nothing overhangs the road at ground level.

TRAFFIC CONTROL
---------------
This module owns junction control, because it is the only pass that can see both
sides of the decision. `build_city`'s traffic-light pass places ONE light per
intersection gated on a probability, which is wrong two ways over: MUTCD 4D.13
requires a *minimum of two signal faces per approach*, continuously visible over
the approach sight distance — not two per junction — and a junction is signal-
controlled or stop-controlled by road hierarchy, never by a coin flip and never
both at once. Set `traffic_lights.intersection_chance: 0` and let this run.

    signal   either leg is an arterial (MUTCD 4C warrants track volume, and
             lane count is the only volume proxy a corridor carries). One mast
             arm per approach, its arm spanning the lanes it governs; the asset
             carries two heads, which is the 4D.13 minimum for that approach.
    all-way  both legs the same class — MUTCD 2B.08's "volumes on the
             intersecting roads are approximately equal" criterion.
    two-way  otherwise: 2B.07 assigns right-of-way at a minor road meeting a
             major one, so only the MINOR road's approaches are signed.

SIGHTLINES
----------
Keeping props out of each other is not enough for regulatory signs: a tree or a
shelter standing between a sign and the lane it governs hides it while
overlapping nothing. Each stop sign therefore reserves the sight triangle
between its panel and the approach, and anything tall enough to occlude is
refused inside it. Geometry is AASHTO: stopping sight distance from the Green
Book's SSD relation, 1080 mm driver eye height, and the panel at its MUTCD 2A.18
urban mounting height. See `_Sightlines`.
"""

import math
import random

from scene_generator import (_in_exclusion, _normalize_usd_list)

# Zones ordered from the kerb inward. "through" is intentionally placeable —
# nothing is configured into it, but the model shouldn't forbid it outright.
_ZONES = (("curb", 0.15), ("furnishing", 1.8), ("through", 2.0),
          ("frontage", 0.6))
_ZONE_NAMES = tuple(z for z, _d in _ZONES)

# Behind the frontage zone, inside the block. Not a sidewalk zone: it is walked
# as its own ring rather than measured off the kerb line.
_INTERIOR = "interior"

# Config keys are plural (they name a pool of assets); placement categories are
# singular, because that is what `fallback_sizes` is keyed on and what the rest
# of the generator emits. Getting this wrong is quiet rather than loud — the
# footprint lookup just misses and every prop silently takes the generic 4x4 m
# default — so the mapping is explicit.
_CATEGORY = {
    "streetlights":      "streetlight",
    "benches":           "bench",
    "trash_cans":        "trash_can",
    "fire_hydrants":     "fire_hydrant",
    "bus_stops":         "bus_stop",
    "street_trees":      "street_tree",
    "planter_fences":    "planter_fence",
    "mailboxes":         "mailbox",
    "cafe_sets":         "cafe_set",
    "bollards":          "bollard",
    "bike_lane_delineators": "bike_lane_delineator",
    "parking_meters":    "parking_meter",
    "bike_racks":        "bike_rack",
    "utility_poles":     "utility_pole",
    "dumpsters":         "dumpster",
    "phone_booths":      "phone_booth",
    "traffic_cones":     "traffic_cone",
    "billboards":        "billboard",
    "manholes":          "manhole",
    "newspaper_boxes":   "newspaper_box",
    "storm_drains":      "storm_drain",
    "signs_stop":        "sign",
    "signs_street_name": "sign",
    "signs_parking":     "sign",
}

# Orientation modes. All of them put the short footprint axis across the
# pavement; they differ in what ends up pointing at the road.
_ALONG = "along"      # long axis along the kerb, front turned to the building
_FACE = "face"        # ditto, front turned to the street (shelters, meters)
_TRAFFIC = "traffic"  # panel square-on to the lane, i.e. read from a moving car
_POLE = "pole"        # arm out over the roadway; only the pole is on the paving
_CANOPY = "canopy"    # crown overhangs; only the trunk is on the paving
_FREE = "free"        # no front and no long axis worth aligning

_ORIENT = {
    "cafe_set":      _ALONG,
    "bike_rack":     _ALONG,
    "dumpster":      _ALONG,
    "bench":         _FACE,     # seat toward the street
    "bus_stop":      _FACE,     # shelter opening on the street side
    "phone_booth":   _FACE,
    "mailbox":       _FACE,
    "newspaper_box": _FACE,
    "parking_meter": _FACE,
    "fire_hydrant":  _FACE,
    "sign":          _TRAFFIC,
    "billboard":     _TRAFFIC,
    "streetlight":   _POLE,
    "utility_pole":  _POLE,
    "street_tree":   _CANOPY,
    "planter_fence": _FREE,
    "trash_can":     _FREE,
    "traffic_cone":  _FREE,
    "bollard":       _FREE,
    "manhole":       _FREE,
    "storm_drain":   _FREE,
}


def _rect(block):
    """Block corners as (xmin, ymin, xmax, ymax)."""
    x0, y0, x1, y1 = block
    return min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)


def _zone_inset(zones: dict, zone: str) -> float:
    """Metres from the kerb line to the centre of *zone*."""
    widths = [float(zones.get(f"{z}_m", d)) for z, d in _ZONES]
    try:
        i = _ZONE_NAMES.index(zone)
    except ValueError:
        raise ValueError(f"unknown sidewalk zone {zone!r}; expected one of "
                         f"{', '.join(_ZONE_NAMES)} or {_INTERIOR}")
    return sum(widths[:i]) + widths[i] / 2.0


def _zone_bounds(zones: dict, zone: str):
    """``(near, far)`` metres from the kerb spanned by *zone*."""
    widths = [float(zones.get(f"{z}_m", d)) for z, d in _ZONES]
    i = _ZONE_NAMES.index(zone)
    near = sum(widths[:i])
    return near, near + widths[i]


def _interior_inset(zones: dict) -> float:
    """Metres from the kerb line to the interior service ring.

    Defaults to just past the frontage zone: deep enough to be off the sidewalk
    entirely, shallow enough to stay in the strip between the pavement and the
    building line rather than inside a packed building.
    """
    total = sum(float(zones.get(f"{z}_m", d)) for z, d in _ZONES)
    return float(zones.get(f"{_INTERIOR}_m", total + 1.5))


class _Occupancy:
    """Footprint reservation over one shared grid.

    The point grid this replaces claimed a single cell per prop regardless of
    size, so a 6 m bench and a bollard could occupy the same metre of kerb and
    the scene read as scattered rather than laid out. Props now reserve their
    world-XY bbox; the grid is only a bucketing scheme so the overlap test stays
    local. Mirrors `scene_generator._occ_reserve`.

    One grid for all zones, not one per zone: with real footprints the zones
    already separate props spatially, and a shared grid is the only thing that
    stops a deep prop in one zone from reaching into its neighbour.
    """

    def __init__(self, cell_m: float = 4.0, pad_m: float = 0.1):
        self.cell = max(0.5, float(cell_m))
        self.pad = max(0.0, float(pad_m))
        self._cells = {}

    def _keys(self, r):
        c = self.cell
        return [(gx, gy)
                for gx in range(int(math.floor(r[0] / c)),
                                int(math.floor(r[2] / c)) + 1)
                for gy in range(int(math.floor(r[1] / c)),
                                int(math.floor(r[3] / c)) + 1)]

    def reserve(self, rect) -> bool:
        """Claim *rect*; False (and nothing claimed) if it hits a reservation."""
        r = (rect[0] - self.pad, rect[1] - self.pad,
             rect[2] + self.pad, rect[3] + self.pad)
        keys = self._keys(r)
        for k in keys:
            for o in self._cells.get(k, ()):
                if not (r[2] <= o[0] or r[0] >= o[2]
                        or r[3] <= o[1] or r[1] >= o[3]):
                    return False
        for k in keys:
            self._cells.setdefault(k, []).append(r)
        return True


def stopping_sight_distance(speed_kph: float, reaction_s: float = 2.5,
                            decel_mss: float = 3.4) -> float:
    """AASHTO Green Book stopping sight distance, metres.

    ``SSD = 0.278·V·t + 0.039·V²/a`` with the Green Book's design values —
    2.5 s brake reaction and 3.4 m/s² deceleration. Reproduces the SSD table to
    the rounding: 30 km/h -> 31 m (table 35), 40 -> 46 (50), 50 -> 63 (65).
    """
    v = max(5.0, float(speed_kph))
    return 0.278 * v * reaction_s + 0.039 * v * v / max(0.5, decel_mss)


class _Sightlines:
    """Sight triangles reserved by regulatory signs, as keep-outs for tall props.

    Footprint occupancy alone lets a street tree stand three metres upstream of
    a stop sign and hide it completely without touching it. The driver's view is
    the thing being protected, so it gets reserved like any other space.

    In the sign's frame — *a* upstream toward the approaching driver, *b* outward
    across the pavement from the sign, kerb at ``b = inset`` — the union of every
    sightline from an eye anywhere in the near lane to the panel is the triangle

        0 <= a <= L,   0 <= b <= min(inset, a·(inset + eye_off)/S)

    with ``S`` the stopping sight distance and ``L = S·inset/(inset + eye_off)``
    where the shallowest line crosses the kerb. Beyond L the sightline is over
    the carriageway, where nothing is placed anyway — which is why a 50 m sight
    distance costs only ~10-15 m of kerb.

    Height matters as much as plan: the sightline runs from the driver's eye
    (AASHTO 1080 mm) up to the panel, so it clears everything below about 1.8 m
    inside the wedge. A bin does not hide a stop sign; a tree does. Both axes are
    world-aligned here — every block edge is — so the test is exact, not sampled.
    """

    def __init__(self, cell_m: float = 8.0):
        self.cell = max(1.0, float(cell_m))
        self._cells = {}

    def _keys(self, r):
        c = self.cell
        return [(gx, gy)
                for gx in range(int(math.floor(r[0] / c)),
                                int(math.floor(r[2] / c)) + 1)
                for gy in range(int(math.floor(r[1] / c)),
                                int(math.floor(r[3] / c)) + 1)]

    def reserve(self, x, y, up_yaw, out_yaw, inset, eye_off, ssd,
                face_z, eye_z):
        ux, uy = _unit(up_yaw)
        nx, ny = _unit(out_yaw)
        k = (inset + eye_off) / max(1.0, ssd)
        length = inset / k if k > 1e-9 else 0.0
        if length <= 0.0 or inset <= 0.0:
            return
        w = (x, y, x + ux * length + nx * inset, y + uy * length + ny * inset)
        box = (min(w[0], w[2]), min(w[1], w[3]),
               max(w[0], w[2]), max(w[1], w[3]))
        wedge = (x, y, ux, uy, nx, ny, length, k, inset, ssd, face_z, eye_z)
        for key in self._keys(box):
            self._cells.setdefault(key, []).append(wedge)

    def occludes(self, rect, height) -> bool:
        """True if a prop of *height* standing on *rect* would hide a sign."""
        seen = set()
        for key in self._keys(rect):
            for wedge in self._cells.get(key, ()):
                if id(wedge) in seen:
                    continue
                seen.add(id(wedge))
                ox, oy, ux, uy, nx, ny, length, k, inset, ssd, fz, ez = wedge
                a0, a1 = _span(rect, ox, oy, ux, uy)
                b0, b1 = _span(rect, ox, oy, nx, ny)
                a0, a1 = max(a0, 0.0), min(a1, length)
                if a1 <= a0 or b1 <= 0.0 or b0 >= inset:
                    continue
                if b0 >= min(inset, a1 * k):
                    continue
                # Sightline elevation falls from the panel toward the driver, so
                # the far end of the overlap is where it is easiest to break.
                if height > fz + (a1 / ssd) * (ez - fz):
                    return True
        return False


def _unit(yaw_deg: float):
    """Unit vector for an axis-aligned yaw, snapped so it stays exact."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return (round(c) if abs(c) > 0.5 else 0.0, round(s) if abs(s) > 0.5 else 0.0)


def _span(rect, ox, oy, ax, ay):
    """``(lo, hi)`` extent of *rect* along axis (ax, ay) measured from (ox, oy)."""
    if ax:
        lo, hi = (rect[0] - ox, rect[2] - ox)
        return (lo, hi) if ax > 0 else (-hi, -lo)
    lo, hi = (rect[1] - oy, rect[3] - oy)
    return (lo, hi) if ay > 0 else (-hi, -lo)


def _class_rank(corridor: dict) -> int:
    """0 local, 1 collector, 2 arterial. Falls back to the lane count so this
    keeps working against a corridor dict that predates the hierarchy keys."""
    named = {"local": 0, "residential": 0, "collector": 1, "secondary": 1,
             "arterial": 2, "primary": 2, "avenue": 2}
    cls = str(corridor.get("road_class") or "").lower()
    if cls in named:
        return named[cls]
    n = int(corridor.get("n_lanes", 2))
    return 2 if n >= 4 else (1 if n == 3 else 0)


def _facing_corridor(px: float, py: float, out_yaw: float, corridors,
                     step_m: float = 1.0, search_m: float = 6.0):
    """The road corridor a frontage slot ``(px, py, out_yaw)`` fronts, or None.

    ``(px, py)`` is the block's own edge in the same frame `build_road_surface`
    and the marking pass already walk — the kerb line, with the corridor
    starting right where the block ends. A short step OUT of the block (the
    direction *out_yaw* already faces) therefore lands inside the corridor
    rect with no gap to cross. Falls back to the nearest corridor within
    *search_m* for a slot the direct step misses — a jittered corner, a
    corridor rect that does not quite reach the block edge — and returns None
    only for a genuinely interior slot with no road nearby, which street trees
    never see (city_detail's own frontage slots are always kerb-facing) but a
    caller reusing this against another category's slot might.
    """
    if not corridors:
        return None
    ax, ay = _unit(out_yaw)
    tx, ty = px + ax * step_m, py + ay * step_m
    for c in corridors:
        if (c["x0"] - 1e-6 <= tx <= c["x1"] + 1e-6
                and c["y0"] - 1e-6 <= ty <= c["y1"] + 1e-6):
            return c
    best, best_d = None, search_m
    for c in corridors:
        cx = min(max(tx, c["x0"]), c["x1"])
        cy = min(max(ty, c["y0"]), c["y1"])
        d = math.hypot(tx - cx, ty - cy)
        if d < best_d:
            best, best_d = c, d
    return best


def _max_crown_m(corridor: dict, sidewalk_depth_m: float, carriage_frac: float,
                 min_crown_m: float, lane_w: float) -> float:
    """How wide a street tree's crown may be at this frontage before it reads
    as swallowing the street rather than shading it.

    ``carriage_frac`` of the corridor's ``carriage`` span — the travelled
    way's own width, kerb to kerb, parking excluded, the same field
    `build_road_surface` paints the asphalt to — plus the sidewalk depth on
    the tree's own side (the NACTO zone stack this module already sums to
    place furniture, kerb line to building line). A crown overhanging part of
    the near lane is what a street tree is FOR, so the cap is deliberately
    NOT "fits the sidewalk"; it is "does not read as closing the carriageway
    over", which `carriage_frac` alone controls — see the `crown_by_road`
    config block for the value chosen and why.

    Falls back to ``n_lanes * lane_w`` when *corridor* predates the
    ``carriage`` key (e.g. the legacy corridor dicts
    `scene_generator._subdivide_region_metric` builds for a plain suburb),
    the same fallback `_class_rank` already uses for a missing `road_class`.
    """
    car = corridor.get("carriage")
    if car and len(car) == 2:
        carriage_w = abs(float(car[1]) - float(car[0]))
    else:
        carriage_w = float(corridor.get("n_lanes", 2)) * lane_w
    return max(min_crown_m, carriage_w * carriage_frac + sidewalk_depth_m)


def _junction_control(a: dict, b: dict, signal_lanes: int, stop_zones):
    """``(control, all_way, minor)`` for the junction between *a* and *b*.

    The decision table, from MUTCD Part 2B (STOP warrants) and Part 4C (signal
    warrants), with road class standing in for the volume those warrants are
    really written against — lane count is the only volume proxy a corridor
    carries:

        arterial x arterial   signal    4C-1/4C-2 volume; mast arms
        arterial x collector  signal    same; mast arms
        arterial x local      signal    4C-6, coordinated system: a dense grid
                                        signalises the whole avenue. Downgrades
                                        to two-way stop in a *stop_zone*, which
                                        is what a residential fringe does.
        collector x collector all-way   2B.08 "approximately equal volumes"
        collector x local     two-way   2B.07, minor road yields
        local x local         two-way   2B.08 explicitly warns against all-way
                                        as speed control, so the longer street
                                        stays through and the shorter one stops

    *minor* is the corridor whose approaches get signed under two-way control.
    `city_layout.junction_control` is the published signal/stop contract and is
    consulted first so both modules agree on which junctions are arterial; it is
    imported lazily so this file still loads while that one is mid-edit.
    """
    ra, rb = _class_rank(a), _class_rank(b)
    try:
        from layout.city_layout import junction_control as _published
        control = _published(a, b, signal_lanes)
    except Exception:
        control = ("signal" if max(int(a.get("n_lanes", 2)),
                                   int(b.get("n_lanes", 2))) >= signal_lanes
                   else "stop")

    # A local street meeting an avenue out at the residential edge is stop
    # controlled in practice — the 4C volume warrants aren't met out there.
    if (control == "signal" and stop_zones and min(ra, rb) == 0
            and (str(a.get("zone") or "") in stop_zones
                 or str(b.get("zone") or "") in stop_zones)):
        control = "stop"
    if control == "signal":
        return "signal", False, None

    if ra != rb:
        return "stop", False, (a if ra < rb else b)
    if ra >= 1:
        return "stop", True, None
    # Equal locals: the longer run is the through street, so the control is
    # stable rather than a coin flip that changes with the seed.
    la = float(a["y1"]) - float(a["y0"])
    lb = float(b["x1"]) - float(b["x0"])
    return "stop", False, (a if la < lb else b)


def _junctions(corridors, signal_lanes: int, stop_zones, eps: float = 0.25):
    """Every crossing or touching ns/ew corridor pair, classified.

    Touching counts: the BSP ends a child road exactly where the crossing road
    begins, so a pure overlap test misses every T-junction — the same reason
    `build_city`'s own pass uses an epsilon.
    """
    raw = []
    ns = [c for c in corridors if c.get("dir") == "ns"]
    ew = [c for c in corridors if c.get("dir") == "ew"]
    for a in ns:
        for b in ew:
            if (b["x0"] > a["x1"] + eps or b["x1"] < a["x0"] - eps
                    or a["y0"] > b["y1"] + eps or a["y1"] < b["y0"] - eps):
                continue
            control, all_way, _minor = _junction_control(
                a, b, signal_lanes, stop_zones)
            raw.append({"box": (a["x0"], b["y0"], a["x1"], b["y1"]),
                        "ns_set": {id(a)}, "ew_set": {id(b)}, "ns": a, "ew": b,
                        "control": control, "all_way": all_way,
                        "arterial": max(_class_rank(a), _class_rank(b)) >= 2})

    # The BSP can split twice at nearly the same coordinate in two subtrees, so
    # two corridor rects overlap and one physical intersection shows up as
    # several junctions. Left unmerged they disagree with each other — one
    # signalises while its twin five metres away signs — which is precisely the
    # "signal and stop sign at the same junction" contradiction. Overlapping
    # boxes are one junction, and the highest-class leg decides its control.
    merged = []
    for j in raw:
        for m in merged:
            if not (j["box"][2] <= m["box"][0] or j["box"][0] >= m["box"][2]
                    or j["box"][3] <= m["box"][1] or j["box"][1] >= m["box"][3]):
                m["box"] = (min(m["box"][0], j["box"][0]),
                            min(m["box"][1], j["box"][1]),
                            max(m["box"][2], j["box"][2]),
                            max(m["box"][3], j["box"][3]))
                m["ns_set"] |= j["ns_set"]
                m["ew_set"] |= j["ew_set"]
                if _class_rank(j["ns"]) > _class_rank(m["ns"]):
                    m["ns"] = j["ns"]
                if _class_rank(j["ew"]) > _class_rank(m["ew"]):
                    m["ew"] = j["ew"]
                m["arterial"] = m["arterial"] or j["arterial"]
                if j["control"] == "signal":
                    m["control"] = "signal"
                m["all_way"] = m["all_way"] and j["all_way"]
                break
        else:
            merged.append(j)
    for m in merged:
        if m["control"] == "signal":
            m["all_way"] = False
    return merged


def _approach_legs(j, eps: float = 0.25):
    """``[(travel_yaw, corner_xy, arm_yaw)]`` for every leg feeding junction *j*.

    A leg exists only where its road actually continues past the junction, which
    is what keeps a T-junction from being given a phantom fourth approach.

    Right-hand traffic puts the mast on the FAR-RIGHT corner of the approach
    with its arm reaching left across the road, so the two heads it carries hang
    over the lanes they govern and face back down the approach. That is the
    whole reason the arm direction is not decorative.

    Corners are returned in preference order, far side before near side. At the
    region boundary the far corners have no pavement behind them, and a
    near-side head is what a real junction uses there rather than leaving the
    approach dark, which is exactly the "one side has a signal and one doesn't"
    complaint — so all four are offered before an approach is given up on.
    """
    jx0, jy0, jx1, jy1 = j["box"]
    ns, ew = j["ns"], j["ew"]
    ne, nw, se, sw = (jx1, jy1), (jx0, jy1), (jx1, jy0), (jx0, jy0)
    legs = []
    # Each entry pairs a corner with the arm heading FROM that corner, because
    # a mast on the far-left corner has to reach the other way to span the same
    # lanes.
    # NEAR-side, RIGHT-hand corner first — the corner the stopping traffic is
    # actually beside. The signal stands level with the stop bar, on the same
    # kerb as the lanes held at it, so a driver at the line sees the head
    # without turning. The far corner is the diagonally opposite one and is
    # offered only as a fallback where the near one has no pavement behind it
    # (a T-junction leg), never as the preference.
    #
    # Northbound traffic approaches from the SOUTH, so its near-right corner is
    # SE and its arm reaches WEST across the road it governs. The other three
    # follow by rotation. The stop-line setback in `_place_signals` then pushes
    # the pole outward from the junction centre, which on a near corner moves it
    # BACK along the approach to the bar — on a far corner it would have pushed
    # it further past the junction, which is what put the heads beyond the
    # crossing.
    if float(ns["y0"]) < jy0 - eps:                     # northbound approach
        legs.append((90.0, ((se, 180.0), (sw, 0.0), (ne, 180.0),
                            (nw, 0.0)), ns))
    if float(ns["y1"]) > jy1 + eps:                     # southbound
        legs.append((-90.0, ((nw, 0.0), (ne, 180.0), (sw, 0.0),
                             (se, 180.0)), ns))
    if float(ew["x0"]) < jx0 - eps:                     # eastbound
        legs.append((0.0, ((sw, 90.0), (nw, -90.0), (se, 90.0),
                           (ne, -90.0)), ew))
    if float(ew["x1"]) > jx1 + eps:                     # westbound
        legs.append((180.0, ((ne, -90.0), (se, 90.0), (nw, -90.0),
                             (sw, 90.0)), ew))
    return legs


def _mast_yaw(arm_yaw: float, fp: dict, art_yaw: float) -> float:
    """Yaw putting a signal assembly's mast arm along *arm_yaw*.

    Which way the arm points is measured, not assumed: the arm is the long
    horizontal axis and the bbox centroid offset (`cx`/`cy`) says which end it
    reaches toward, since the pole sits at the other one. Deriving it means an
    asset swap cannot silently rotate every signal in the city — the failure the
    art-convention yaw offset produced. Only the 180° part of that offset is
    kept, for the same reason it is elsewhere: a ±90 offset restates what the
    extents already say.
    """
    if fp["sx"] >= fp["sy"]:
        local = 0.0 if fp.get("cx", 0.0) >= 0.0 else 180.0
    else:
        local = 90.0 if fp.get("cy", 0.0) >= 0.0 else -90.0
    flip = 180.0 if round(art_yaw / 180.0) % 2 else 0.0
    return arm_yaw - local + flip


def _block_at(blocks, x: float, y: float):
    """The block rect containing (x, y), or None — pavement test."""
    for b in blocks:
        if b[0] <= x <= b[2] and b[1] <= y <= b[3]:
            return b
    return None


def _nearest_block(blocks, x: float, y: float, limit: float):
    """Nearest block rect within *limit* metres, for a corner that landed in the
    roadway (T-junctions have one)."""
    best, best_d = None, limit
    for b in blocks:
        dx = max(b[0] - x, 0.0, x - b[2])
        dy = max(b[1] - y, 0.0, y - b[3])
        d = math.hypot(dx, dy)
        if d < best_d:
            best, best_d = b, d
    return best


def _occ_extent(fp: dict, mode: str, canopy_frac: float):
    """``(along_kerb_m, across_pavement_m)`` the prop occupies at ground level.

    Not the same as the bbox for every mode: a lamp arm and a tree crown
    overhang the roadway by design and must not reserve pavement, or the kerb
    line fills up with air. Anything placed at a free yaw reserves the
    conservative square instead, since its heading isn't known at reservation
    time.

    Traffic-facing signage is the one case where the long axis runs *across* the
    pavement rather than along it: a panel square-on to the lane is by
    definition perpendicular to the kerb it stands on.
    """
    lo, hi = min(fp["sx"], fp["sy"]), max(fp["sx"], fp["sy"])
    if mode == _POLE:
        return lo, lo
    if mode == _CANOPY:
        t = max(0.2, lo * canopy_frac)
        return t, t
    if mode == _FREE:
        return hi, hi
    if mode == _TRAFFIC:
        return lo, hi
    return hi, lo


def _half_extents(out_yaw: float, along: float, across: float):
    """World-XY half sizes for a prop standing on an edge with normal *out_yaw*."""
    if abs(math.cos(math.radians(out_yaw))) < 0.5:     # edge runs E-W
        return along / 2.0, across / 2.0
    return across / 2.0, along / 2.0


def _prop_yaw(out_yaw: float, fp: dict, mode: str, art_yaw: float, rng) -> float:
    """Yaw for a prop on an edge whose outward normal is *out_yaw*.

    Derived from the footprint, not from the art convention: a bench, a shelter
    and a planter all lie long-ways along the kerb, and the alternative puts
    half the prop in the road. The mode decides which end of the remaining axis
    looks at the street — and, for signage, turns the panel square-on to the
    lane instead.

    A per-asset yaw offset of ±90 says "this art's front is +Y" — which is
    exactly what the aspect test already discovers, so applying it as well would
    rotate the prop back off the kerb line. Only the 180° part of the offset
    survives, since which way round the art faces is not in its extents.
    """
    if mode in (_FREE, _CANOPY):
        return rng.uniform(0.0, 360.0)
    if mode == _POLE:
        return out_yaw + art_yaw
    # Right-hand traffic: the lane against this kerb runs at out_yaw - 90, so a
    # sign meant to be read from it looks back down the street at out_yaw + 90.
    base = out_yaw + 90.0 if mode == _TRAFFIC else out_yaw
    flip = 180.0 if round(art_yaw / 180.0) % 2 else 0.0
    if fp["sx"] < fp["sy"]:
        return base + flip                       # +X is already the short axis
    return base + (90.0 if mode == _ALONG else -90.0) + flip


def _fit_in_block(x: float, y: float, hx: float, hy: float, block, margin: float):
    """Slide (x, y) until the prop's bbox sits wholly inside *block*.

    The block edge is the kerb line — `city_layout` cuts blocks straight from
    the road edges and moves the edge itself when a face gets a kerb extension,
    so "inside the block" is the same statement as "off the carriageway", and it
    stays true whatever that module does with parking strips. (The corridor rect
    is not the test: it reserves strips that may since have been handed to the
    pavement.) It also catches the case no inset can fix — a 6 m bench centred a
    metre from a corner overruns the *cross* street however far back from its
    own kerb it stands.

    Returns None when the prop cannot fit at all.
    """
    x0, y0, x1, y1 = block
    lo_x, hi_x = x0 + margin + hx, x1 - margin - hx
    lo_y, hi_y = y0 + margin + hy, y1 - margin - hy
    if lo_x > hi_x or lo_y > hi_y:
        return None
    return min(max(x, lo_x), hi_x), min(max(y, lo_y), hi_y)


def _frontage_edges(blocks, inset_m: float = 0.0):
    """``[(ax, ay, bx, by, out_yaw, length, block)]`` for every block edge.

    Outward yaws: south -90, north +90, west 180, east 0 — passing this as a
    prop's facing direction makes it look at its own street. *inset_m* walks a
    ring that far inside the block instead of the kerb line itself, which is how
    the interior zone reuses this; the *block* carried alongside is always the
    original, because that is what the roadway test is against.
    """
    out = []
    for block in blocks:
        bx0, by0, bx1, by1 = _rect(block)
        x0, y0 = bx0 + inset_m, by0 + inset_m
        x1, y1 = bx1 - inset_m, by1 - inset_m
        if x1 - x0 <= 1e-6 or y1 - y0 <= 1e-6:
            continue
        for ax, ay, bx, by, yaw in (
                (x0, y0, x1, y0, -90.0), (x0, y1, x1, y1, 90.0),
                (x0, y0, x0, y1, 180.0), (x1, y0, x1, y1, 0.0)):
            length = math.hypot(bx - ax, by - ay)
            if length > 1e-6:
                out.append((ax, ay, bx, by, yaw, length,
                            (bx0, by0, bx1, by1)))
    return out


def _frontage_slots(blocks, spacing: float, rng, phase: float = 0.0,
                    jitter_frac: float = 0.0, inset_m: float = 0.0):
    """Yield ``(x, y, out_yaw, block, approach, corner)`` along the ring.

    Placement walks the edges as ONE concatenated run rather than per-edge.
    That matters for any category sparser than a block edge is long. The
    generator's own `_frontage_positions` — and this function's first version —
    fall back to "emit one point with probability length/spacing" once spacing
    exceeds the edge, which is right on average but wrong per run: with 50 m
    edges, a 130 m bus-stop spacing becomes an independent coin flip per edge,
    so the count swings with the seed and nothing guarantees the requested
    density anywhere. Corner-anchored props were the visible symptom (stop signs
    landed zero because that one random point rarely fell near a corner), but
    bus stops, mailboxes, dumpsters and cones all had it.

    Walking the concatenated run gives exactly ``total_length / spacing`` props,
    spread evenly over the whole ring, with no coin flips.

    A *spacing* of 0 or less yields nothing — how a category is switched off.
    """
    if spacing <= 0.0:
        return
    edges = _frontage_edges(blocks, inset_m)
    if not edges:
        return
    total = sum(e[5] for e in edges)
    if total <= 1e-6:
        return

    n = max(1, int(round(total / spacing)))
    step = total / n

    # Cumulative start distance of each edge along the concatenated run.
    starts, acc = [], 0.0
    for e in edges:
        starts.append(acc)
        acc += e[5]

    idx = 0
    for k in range(n):
        s = (k + phase) * step
        if jitter_frac:
            s += rng.uniform(-1.0, 1.0) * jitter_frac * step
        s = min(total - 1e-6, max(0.0, s))
        # s is near-monotonic, so advance the edge pointer rather than
        # searching; jitter can nudge it backwards, hence the second loop.
        while idx + 1 < len(edges) and s >= starts[idx] + edges[idx][5]:
            idx += 1
        while idx > 0 and s < starts[idx]:
            idx -= 1
        ax, ay, bx, by, yaw, length, block = edges[idx]
        t = (s - starts[idx]) / length
        t = min(1.0, max(0.0, t))
        yield ax + (bx - ax) * t, ay + (by - ay) * t, yaw, block, True, None


def _corner_slots(block, offset_m: float):
    """Yield ``(x, y, out_yaw, block, approach, corner)`` inside each corner.

    Corner-bound props (signs, bollards) can't be produced by filtering a
    spaced walk: `_frontage_slots` emits a single random point per edge once
    the spacing exceeds the edge length, and that point almost never lands near
    a corner — which silently starved stop signs to zero. Anchoring at the
    corners instead makes the count depend on the block count, which is what a
    "one per corner" prop actually does.

    Each corner yields two slots, one along each adjoining edge, so a corner
    reads as a junction rather than a single post.

    *approach* marks the slot a driver in the lane against that kerb reaches
    BEFORE the junction. Right-hand traffic puts that lane's direction of travel
    at ``out_yaw - 90``, so the upstream slot is the one displaced against it,
    and exactly one of the two slots on each edge qualifies. Regulatory signage
    keeps only those: a stop sign sited past the junction it controls is most of
    why they read as random scatter.
    """
    x0, y0, x1, y1 = _rect(block)
    dx = min(offset_m, (x1 - x0) * 0.4)
    dy = min(offset_m, (y1 - y0) * 0.4)
    for cx, cy, sx, sy in ((x0, y0, +1, +1), (x1, y0, -1, +1),
                           (x0, y1, +1, -1), (x1, y1, -1, -1)):
        south = cy == y0
        west = cx == x0
        yield (cx + sx * dx, cy, -90.0 if south else 90.0,
               (x0, y0, x1, y1), (sx > 0) == south, (cx, cy))
        yield (cx, cy + sy * dy, 180.0 if west else 0.0,
               (x0, y0, x1, y1), (sy > 0) != west, (cx, cy))


def _inset(px: float, py: float, out_yaw: float, dist: float):
    """Step from the kerb point inward onto the sidewalk by *dist* metres."""
    rad = math.radians(out_yaw)
    return px - math.cos(rad) * dist, py - math.sin(rad) * dist


def _lateral_insets(centre: float, across: float, zone_near: float,
                    zone_far: float, tries: int, is_interior: bool):
    """Inset distances to try, in order, all inside the prop's own zone.

    One line at the zone centre makes the kerb a strictly linear budget: the
    categories at the back of the queue are refused not because the pavement is
    full but because that one line is. Offering the back and front of the zone
    recovers most of them, and confining the tries to the zone is what keeps the
    guarantee that the pedestrian through zone stays empty.
    """
    yield centre
    if tries <= 1 or is_interior:
        return
    lo, hi = zone_near + across / 2.0, zone_far - across / 2.0
    if hi - lo <= 0.05:
        return                                  # prop is deeper than its zone
    for want in (hi, lo, (centre + hi) / 2.0, (centre + lo) / 2.0)[:tries - 1]:
        yield min(hi, max(lo, want))


def _junction_at(junctions, x: float, y: float, tol: float = 1.0):
    """The junction whose box holds (x, y) — a block corner IS a junction corner.

    Looked up by geometry rather than by probing for the adjoining corridors:
    where two corridor rects overlap, a point probe returns whichever of them
    comes first, and picking the wrong one silently put stop signs at signalised
    junctions. The corner's coordinates are unambiguous. A kerb extension grows
    the block into the corridor, so the corner can sit inside the box as well as
    on it, and the tolerance only has to cover float noise the other way.
    """
    for j in junctions:
        b = j["box"]
        if (b[0] - tol <= x <= b[2] + tol) and (b[1] - tol <= y <= b[3] + tol):
            return j
    return None


def _signed_here(junctions, block, corner, out_yaw, gate) -> bool:
    """Whether a regulatory sign belongs on this corner slot.

    A sign goes up only where the junction it faces is stop controlled, and then
    only on the approaches that actually stop: both roads under an all-way
    (2B.08), the minor road alone under a two-way (2B.07). A signalised junction
    gets none — signals and stop signs at the same junction is the contradiction
    the scene was showing.
    """
    j = _junction_at(junctions, corner[0], corner[1])
    if j is None or j["control"] != gate:
        return False
    if j["all_way"]:
        return True
    # The slot's own edge says which road the approach travels on: a slot on an
    # E-W edge belongs to the E-W road beside it.
    horizontal = abs(_unit(out_yaw)[1]) > 0.5
    approach, cross = ((j["ew"], j["ns"]) if horizontal else (j["ns"], j["ew"]))
    return _is_minor(approach, cross)


def _is_minor(approach: dict, cross: dict) -> bool:
    """Whether *approach* is the yielding leg under two-way stop control.

    Class first (2B.07 is about a minor road meeting a major one); on a tie the
    longer run stays through, so which street stops is stable across seeds
    rather than a coin flip that moves the signs every run.
    """
    ra, rc = _class_rank(approach), _class_rank(cross)
    if ra != rc:
        return ra < rc
    la = (float(approach["y1"]) - float(approach["y0"])
          if approach.get("dir") == "ns"
          else float(approach["x1"]) - float(approach["x0"]))
    lc = (float(cross["y1"]) - float(cross["y0"]) if cross.get("dir") == "ns"
          else float(cross["x1"]) - float(cross["x0"]))
    return la < lc


def _order_categories(categories: dict, clearances: dict, size_of):
    """Order categories so anything used as a clearance reference is placed
    first — otherwise the constraint has nothing to check against — and, among
    independent ones, biggest footprint first.

    Size order matters now that props reserve their real bbox: a bus shelter
    offered the kerb after the bollards and meters have peppered it finds
    nowhere to stand, which is the same starvation the built-in passes had.

    Regulatory signage jumps the whole queue regardless of size. Its position is
    fixed by the junction it controls rather than negotiable, and it has to be
    down before anything else so its sight triangle is reserved while the kerb
    is still empty.

    A cycle falls back to declaration order rather than raising, since a mutual
    clearance is a config mistake rather than something worth aborting a scene
    over.
    """
    ordered, seen = [], set()

    def visit(name, stack):
        if name in seen or name not in categories:
            return
        if name in stack:                      # cycle — leave it to fall through
            return
        stack.add(name)
        for dep in (clearances.get(name) or {}):
            visit(dep, stack)
        stack.discard(name)
        seen.add(name)
        ordered.append(name)

    def key(name):
        spec = categories.get(name) or {}
        return (0 if spec.get("control") else 1, -size_of(name), name)

    for n in sorted(categories, key=key):
        visit(n, set())
    return ordered


def _pool(usds: dict, key: str, default_scale: float, asset_root: str,
         tag: str = None):
    """Normalized ``(paths, scale, axis_up, yaw_offset)`` lookups for a category.

    Returns ``None`` when the pool is empty or absent, which is how a category
    with no art sourced yet is skipped without any code change.

    *tag*, when given, prefers members carrying it and falls back to the whole
    pool when none do — the same rule `parks.py`'s `_Lib.pool` uses for its
    ``tag="park"`` lookups (a plaza wants the park-register bench over the
    kerb one, and this is the one place outside `parks.py` that needs it).
    """
    raw = usds.get(key)
    if not raw:
        return None
    paths, sc_ovr, au_ovr, yaw_ovr, tag_ovr = _normalize_usd_list(
        raw, default_scale, asset_root)
    if not paths:
        return None
    if tag is not None:
        tagged = [p for p in paths if tag in tag_ovr.get(p, ())]
        paths = tagged or paths
    return {
        "paths": paths,
        "scale": lambda p: sc_ovr.get(p, default_scale),
        "axis_up": lambda p: au_ovr.get(p, "Z"),
        "yaw": lambda p: yaw_ovr.get(p, 0.0),
    }


def _sidewalk_top(config: dict, resolver, default_scale: float,
                  asset_root: str) -> float:
    """Walking-surface height, matching build_city's convention.

    Sidewalk tiles are laid at z=0.015 and their slab top sits (sz - base)
    above the pivot, so props stand at ``sidewalk_top + fp["base"]``.
    """
    tiles = (config.get("usds") or {}).get("tiles") or {}
    entry = tiles.get("sidewalk")
    if not entry:
        return 0.0
    pool = _pool({"sidewalk": entry}, "sidewalk", default_scale, asset_root)
    if pool is None:
        return 0.0
    p = pool["paths"][0]
    fp = resolver.get(p, "sidewalk", scale=pool["scale"](p),
                      axis_up=pool["axis_up"](p))
    return 0.015 + fp["sz"] - fp["base"]


def _stopline_setback(config: dict, sig_cfg: dict) -> float:
    """Distance from the junction box back to the stop line, along an approach.

    Derived from the marking geometry rather than given its own number, so the
    pole cannot drift away from the line it governs when a marking knob moves:
    kerb-to-crosswalk setback, plus the crossing band, plus the MUTCD 4 ft
    advance from the crossing back to the bar.
    """
    if "stopline_setback_m" in sig_cfg:
        return float(sig_cfg["stopline_setback_m"])
    m = ((config.get("roads") or {}).get("markings") or {})
    return (float(m.get("setback_m", 1.2))
            + float(m.get("crosswalk_depth_m", 2.4))
            + float(m.get("stop_bar_advance_m", 1.2)))


def _place_signals(junctions, blocks, pools, resolver, rng, occ, surface_z,
                   corner_margin: float, exclusions, tally, pole_pad=0.0,
                   yaw_trim: float = 0.0, stopline_m: float = 0.0):
    """One mast-arm assembly per approach at every signalised junction.

    MUTCD 4D.13 sets the minimum at two signal faces per APPROACH, continuously
    visible over the approach sight distance — not two per intersection, which
    is what one light per junction amounts to and why the scene read as "one
    side has a signal and one doesn't". The assemblies here carry two heads
    each, so one per approach meets the minimum and a four-way junction gets
    four of them.

    *pools* is ``{"mast": pool|None, "pole": pool|None}``; arterial junctions
    get the mast arm because its arm has to span the lanes it governs, and a
    minor signalised junction gets the cheap pole-mounted head, which is what a
    two-lane crossing actually carries.
    """
    out, missed = [], 0
    for j in junctions:
        if j["control"] != "signal":
            continue
        jcx = (j["box"][0] + j["box"][2]) / 2.0
        jcy = (j["box"][1] + j["box"][3]) / 2.0
        for travel_yaw, corners, road in _approach_legs(j):
            # A mast arm exists to span the lanes it governs, so it goes on
            # the approaches that have lanes to span. A two-lane side street
            # meeting a signalised avenue carries a pole-mounted head, which
            # is what it carries in reality and a sixth of the geometry.
            pool = (pools.get("mast") if _class_rank(road) >= 2
                    else pools.get("pole")) or pools.get("mast") \
                or pools.get("pole")
            if pool is None:
                continue
            u = rng.choice(pool["paths"])
            sc, au = pool["scale"](u), pool["axis_up"](u)
            fp = resolver.get(u, "traffic_light", scale=sc, axis_up=au)
            # Only the pole stands on the pavement; the arm is over the road by
            # design, which is the entire point of a mast arm.
            half = min(fp["sx"], fp["sy"]) / 2.0 + pole_pad
            spot = None
            # A signal head governs the STOP LINE, not the corner: a driver
            # halted at the line has to see it without craning, so the pole
            # stands level with the line rather than level with the kerb return.
            # That is `setback + crosswalk depth + stop-bar advance` back from
            # the junction box, along the approach — the corner position put it
            # a full crossing's depth too far forward.
            ns_leg = abs(abs(travel_yaw) - 90.0) < 1.0
            for (cx, cy), arm_yaw in corners:
                # Step off the junction box onto the corner pavement, backing
                # further into the corner if the first spot is taken — two legs
                # of a T-junction can want the same one corner that has any
                # pavement behind it.
                for extra in (0.0, 2.5, 5.0):
                    m = corner_margin + extra
                    ox = cx + math.copysign(m, cx - jcx)
                    oy = cy + math.copysign(m, cy - jcy)
                    # Only the along-approach axis gets the stop-line setback;
                    # the cross axis keeps the kerb margin so the pole stays on
                    # the pavement rather than stepping into the road.
                    if ns_leg:
                        oy += math.copysign(stopline_m, cy - jcy)
                    else:
                        ox += math.copysign(stopline_m, cx - jcx)
                    block = _block_at(blocks, ox, oy)
                    if block is None:
                        continue
                    fitted = _fit_in_block(ox, oy, half, half, block,
                                           corner_margin)
                    if fitted is None:
                        continue
                    x, y = fitted
                    if exclusions and _in_exclusion(x, y, exclusions):
                        continue
                    if occ.reserve((x - half, y - half, x + half, y + half)):
                        spot = (x, y, arm_yaw)
                        break
                if spot:
                    break
            if spot is None:
                missed += 1
                continue
            x, y, arm_yaw = spot
            # The trim is a quarter turn whose SIGN follows which side of the
            # approach the pole stands on: the arm has to reach across the
            # carriageway, so a pole on the left of travel and one on the right
            # need opposite quarter turns. A single global offset gets one side
            # right and leaves the other facing out of the junction.
            tx = math.cos(math.radians(travel_yaw))
            ty = math.sin(math.radians(travel_yaw))
            side = tx * (y - jcy) - ty * (x - jcx)
            trim = yaw_trim if side >= 0.0 else -yaw_trim
            out.append({
                "usd": u, "x_m": x, "y_m": y,
                "z_m": surface_z + fp["base"],
                "yaw_deg": _mast_yaw(arm_yaw, fp, pool["yaw"](u)) + trim,
                "roll_deg": 90.0 if au == "Y" else 0.0,
                "pitch_deg": 0.0,
                "scale": sc, "axis_up": au, "category": "traffic_light",
            })
            tally["traffic_light"] = tally.get("traffic_light", 0) + 1
    return out, missed


def _place_bike_delineators(config, layout, resolver, rng, cfg, usds,
                            default_scale, asset_root, exclusions, surface_z,
                            tally) -> list:
    """Flexible posts down the buffer of the protected bike-lane stretches.

    Positions come straight from `road_markings.plan_bike_lanes`, the same call
    the paint pass makes, so a post cannot end up on a stretch that was striped
    as an ordinary lane. Imported inside the function: `road_markings` imports
    the generator, and this module is imported by the same caller, so a
    top-level import would fix the order between them for no reason.
    """
    spec = cfg.get("bike_lane_delineators")
    if not isinstance(spec, dict) or not spec.get("enabled", True):
        return []
    pool = _pool(usds, "bike_lane_delineators", default_scale, asset_root)
    if pool is None:
        print("[city_detail] bike_lane_delineators: no assets — skipped")
        return []

    from detail import road_markings

    cat = _CATEGORY["bike_lane_delineators"]
    # A post stands on the carriageway, not on a marking, so it takes the road
    # surface rather than the proud offset the flush covers need.
    z0 = float(spec.get("surface_z", 0.0))
    out = []
    for lane in road_markings.plan_bike_lanes(config, layout):
        for x, y in lane["posts"]:
            if exclusions and _in_exclusion(x, y, exclusions):
                continue
            u = rng.choice(pool["paths"])
            sc, au = pool["scale"](u), pool["axis_up"](u)
            fp = resolver.get(u, cat, scale=sc, axis_up=au)
            out.append({
                "usd": u, "x_m": x, "y_m": y, "z_m": z0 + fp["base"],
                "yaw_deg": rng.uniform(0.0, 360.0),   # a round post has no front
                "roll_deg": 90.0 if au == "Y" else 0.0,
                "pitch_deg": 0.0,
                "scale": sc, "axis_up": au, "category": cat,
            })
            tally[cat] = tally.get(cat, 0) + 1
    return out


def _place_parked_cars(config, layout, resolver, rng, cfg, usds,
                       default_scale, asset_root, exclusions, tally):
    """Cars at the kerb, in the strips the parking policy actually parks on.

    `build_city`'s own car pass puts cars in the TRAVEL lanes, offset from the
    centreline — moving traffic, not parked traffic. Nothing parked at a kerb,
    which is most of what a real street shows. This walks the same parking
    strips `road_markings` paints bay ticks on, so paint and metal agree about
    where parking exists.

    Held back from junctions by the daylighting distance for the same reason
    the bays are: no jurisdiction lets you park across a crossing.
    """
    spec = cfg.get("parked_cars")
    if not isinstance(spec, dict):
        return []
    spacing = float(spec.get("spacing_m", 0.0))
    density = float(spec.get("density", 0.0))
    if spacing <= 0.0 or density <= 0.0:
        return []
    pool = _pool(usds, "parked_cars", default_scale, asset_root) \
        or _pool(usds, "cars", default_scale, asset_root)
    if pool is None:
        print("[city_detail] parked_cars: no assets — skipped")
        return []

    corridors = layout.get("road_corridors") or []
    keepout = float(spec.get("junction_clear_m", 7.5))
    # Junction spans per corridor, so a car never stands across a crossing.
    banned = {}
    for ns_c, ew_c, box in _corridor_junctions(corridors):
        banned.setdefault(id(ns_c), []).append((box[1] - keepout,
                                                box[3] + keepout))
        banned.setdefault(id(ew_c), []).append((box[0] - keepout,
                                                box[2] + keepout))

    out = []
    for c in corridors:
        is_ns = c.get("dir") == "ns"
        x0, x1 = sorted((float(c["x0"]), float(c["x1"])))
        y0, y1 = sorted((float(c["y0"]), float(c["y1"])))
        run_lo, run_hi = (y0, y1) if is_ns else (x0, x1)
        park_w = float(c.get("park_w", 0.0))
        if park_w <= 0.0:
            continue
        car = c.get("carriage")
        policy = str(c.get("parking", "both"))
        for side in ("lo", "hi"):
            if policy not in ("both", side):
                continue
            if car and len(car) == 2:
                edge = float(car[0]) if side == "lo" else float(car[1])
                cross = edge - park_w / 2.0 if side == "lo" \
                    else edge + park_w / 2.0
            else:
                lo, hi = (x0, x1) if is_ns else (y0, y1)
                cross = lo + park_w / 2.0 if side == "lo" else hi - park_w / 2.0
            n = max(1, int((run_hi - run_lo) / spacing))
            for k in range(n):
                s = run_lo + (k + 0.5) * (run_hi - run_lo) / n
                if any(a <= s <= b for a, b in banned.get(id(c), ())):
                    continue
                if rng.random() >= density:
                    continue
                x, y = (cross, s) if is_ns else (s, cross)
                if exclusions and _in_exclusion(x, y, exclusions):
                    continue
                u = rng.choice(pool["paths"])
                sc, au = pool["scale"](u), pool["axis_up"](u)
                fp = resolver.get(u, "car", scale=sc, axis_up=au)
                # Parked cars sit along the kerb, so the long axis runs with the
                # street; heading alternates because a kerb holds both.
                head = 90.0 if is_ns else 0.0
                if rng.random() < 0.5:
                    head += 180.0
                out.append({
                    "usd": u, "x_m": x, "y_m": y, "z_m": fp["base"],
                    "yaw_deg": head + pool["yaw"](u)
                    + rng.uniform(-1.5, 1.5),
                    "roll_deg": 90.0 if au == "Y" else 0.0, "pitch_deg": 0.0,
                    "scale": sc, "axis_up": au, "category": "car",
                })
                tally["car"] = tally.get("car", 0) + 1
    if out:
        print(f"[city_detail] parked cars: {len(out)} at the kerb")
    return out


def _corridor_junctions(corridors, eps: float = 0.25):
    """``(ns, ew, box)`` for every crossing pair — the span a car must not
    stand across. Same touching-counts rule as `_junctions`."""
    ns = [c for c in corridors if c.get("dir") == "ns"]
    ew = [c for c in corridors if c.get("dir") != "ns"]
    for a in ns:
        ax0, ax1 = sorted((float(a["x0"]), float(a["x1"])))
        ay0, ay1 = sorted((float(a["y0"]), float(a["y1"])))
        for b in ew:
            bx0, bx1 = sorted((float(b["x0"]), float(b["x1"])))
            by0, by1 = sorted((float(b["y0"]), float(b["y1"])))
            if (bx0 > ax1 + eps or bx1 < ax0 - eps
                    or ay0 > by1 + eps or ay1 < by0 - eps):
                continue
            yield a, b, (ax0, by0, ax1, by1)


def build_road_surface(config: dict, layout: dict, resolver, rng=None) -> list:
    """Props that belong on the carriageway rather than the pavement.

    Manholes and storm drains are the reason this exists: they were the one
    picked category with nowhere to go, because `build()` only walks block
    frontage rings and a manhole on a kerb is simply wrong. This walks the road
    corridors instead — along the centreline for manholes, along the kerb line
    for drains — which is where a real street puts them.

    Corridors come from `layout["road_corridors"]`, the same structure
    `road_markings` uses: ``{"x0","y0","x1","y1","n_lanes","dir"}``.
    """
    cfg = (config.get("city_detail") or {}).get("road_surface") or {}
    if not cfg:
        return []

    rng = rng or random.Random(int(config.get("seed", 0)) + 4441)
    usds = config.get("usds") or {}
    default_scale = float(config.get("asset_scale", 1.0))
    asset_root = str(config.get("asset_root", "") or "")
    exclusions = config.get("exclusions") or []
    corridors = layout.get("road_corridors") or []

    # The asphalt plane is at z=0 and lane markings at 0.02; covers sit flush
    # with the road, just clear of the markings to avoid coplanar z-fighting.
    surface_z = float(cfg.get("surface_z", 0.005))
    # Covers are laid at a free yaw, so they reserve the conservative square.
    occ = _Occupancy(float(cfg.get("occupancy_cell_m", 4.0)))
    out, tally = [], {}

    out += _place_bike_delineators(config, layout, resolver, rng, cfg, usds,
                                   default_scale, asset_root, exclusions,
                                   surface_z, tally)
    out += _place_parked_cars(config, layout, resolver, rng, cfg, usds,
                              default_scale, asset_root, exclusions, tally)

    for name, spec in cfg.items():
        if not isinstance(spec, dict) or name == "bike_lane_delineators":
            continue
        spacing = float(spec.get("spacing_m", 0.0))
        if spacing <= 0.0:
            continue
        pool = _pool(usds, name, default_scale, asset_root)
        if pool is None:
            print(f"[city_detail] road_surface {name}: no assets — skipped")
            continue
        # "centre" sits on the crown of the road, "kerb" hugs the gutter, which
        # is where drainage actually goes.
        lateral = str(spec.get("lateral", "centre")).lower()
        margin = float(spec.get("kerb_margin_m", 0.6))
        cat = _CATEGORY.get(name, name)

        for c in corridors:
            is_ns = c.get("dir") == "ns"
            x0, x1 = sorted((float(c["x0"]), float(c["x1"])))
            y0, y1 = sorted((float(c["y0"]), float(c["y1"])))
            run_lo, run_hi = (y0, y1) if is_ns else (x0, x1)
            cross_lo, cross_hi = (x0, x1) if is_ns else (y0, y1)
            run = run_hi - run_lo
            if run < spacing:
                continue
            n = max(1, int(run / spacing))
            step = run / n
            for k in range(n):
                s = run_lo + (k + 0.5) * step
                if lateral == "kerb":
                    edge = cross_lo + margin if rng.random() < 0.5 \
                        else cross_hi - margin
                    cross = edge
                else:
                    cross = (cross_lo + cross_hi) / 2.0
                x, y = (cross, s) if is_ns else (s, cross)
                if exclusions and _in_exclusion(x, y, exclusions):
                    continue
                u = rng.choice(pool["paths"])
                sc, au = pool["scale"](u), pool["axis_up"](u)
                fp = resolver.get(u, cat, scale=sc, axis_up=au)
                half = max(fp["sx"], fp["sy"]) / 2.0
                if not occ.reserve((x - half, y - half, x + half, y + half)):
                    continue
                out.append({
                    "usd": u, "x_m": x, "y_m": y,
                    "z_m": surface_z + fp["base"],
                    "yaw_deg": rng.uniform(0.0, 360.0),   # covers have no front
                    "roll_deg": 90.0 if au == "Y" else 0.0,
                    "pitch_deg": 0.0,
                    "scale": sc, "axis_up": au, "category": cat,
                })
                tally[cat] = tally.get(cat, 0) + 1

    if tally:
        detail = "  ".join(f"{k}={v}" for k, v in sorted(tally.items()))
        print(f"[city_detail] road surface: {detail}")
    return out


# ---------------------------------------------------------------------------
# plazas: composed open ground inside the highrise district
# ---------------------------------------------------------------------------
#
# WHERE THE SPACE IS. `districts.typologies.highrise` packs its blocks at
# `perimeter_depth_m: 0` with a 30 m `building_gap_m` — see that typology's
# own comment in the preset — so a highrise block holds one to four towers
# with wide paved gaps between them instead of a perimeter frontage band.
# MEASURED on `downtown_gac`'s three highrise blocks: decomposing each
# block's inset rect minus its towers (`districts.free_rects`) gives gaps up
# to 30 x 108 m (3,228 m2), alongside a scatter of slivers down to 2 m wide
# where a tower sits close to the block edge. The slivers are not a plaza;
# the wide gaps are — `min_side_m`/`min_area_m2` below are the gate, chosen
# off that measured distribution.
#
# ONE ARRANGEMENT FAMILY, SIZED TO WHAT FITS. A prior AC-unit pass read as
# "random instead of structured on an actual building" — the fix there and
# here is the same: pick ONE composition and vary its parameters, never its
# shape. Every plaza gets a centrepiece at its own centre; a ring of benches
# around it at an even pitch, seat normal toward the centre (`parks.py`'s
# attraction ring — see its module docstring — reused verbatim: the yaw IS
# the position angle plus 90, the same convention, the same asset family);
# a café row against whichever tower wall actually borders the gap; and a
# planter line walked around the plaza's own edge. What varies is which
# centrepiece fits (fountain, a planter cluster, a rock cluster, or — if
# none of those assets are sourced — an empty seating court), the ring's
# radius (bounded by the gap, not by a fixed number), and a random rotation.
#
# THE PLAZA'S OWN FOOTPRINT, NOT THE WHOLE GAP. A 30 x 108 m gap is not one
# plaza-sized thing — the ring composition only ever spans `2 * max_radius_m`
# or so. Walking the café row or the planter line along the FULL free rect's
# perimeter (its first-drafted form) put a table every 4.5 m for the entire
# 108 m run — fifty-odd tables down one wall, nothing like the sidewalk-cafe
# register this is meant to match. Both are instead walked around a small
# square keyed to `ring_r`, so their count scales with the COMPOSED plaza,
# and the rest of a long gap stays quiet paving, which is what a real
# corridor between two towers actually looks like.
#
# WHY THE CAFÉ ROW IS FOUND, NOT ASSUMED. A gap between two towers borders a
# real wall on at least one side — that is what makes it a gap rather than
# open ground. `_plaza_wall_runs` finds the side(s) of the free rect where a
# tower footprint actually sits flush against it, so the row of tables ends
# up against a building the way a real café frontage does, rather than
# floating in the middle of the paving. A free rect with no adjacent tower
# (walled only by other block edges) gets no café row, which is correct.
def _plaza_wall_runs(free_rect, obstacles, eps: float = 0.3,
                     min_run_m: float = 1.0):
    """``[(out_yaw, edge_coord, lo, hi)]`` — runs of *free_rect*'s own
    boundary that a tower footprint stands flush against, long enough to be
    worth a café row. *out_yaw* points away from the wall into the gap, the
    same convention `_frontage_edges` uses for a block edge (south -90,
    north 90, west 180, east 0), so the row can be placed with the existing
    `_ALONG`/`_prop_yaw` machinery exactly as a sidewalk café is.

    *obstacles* are already the margin-inflated tower rects `free_rects` was
    decomposed against, so a matching edge lines up to float precision — the
    tolerance only has to cover that, not a real search radius.
    """
    fx0, fy0, fx1, fy1 = free_rect
    runs = []
    for ox0, oy0, ox1, oy1 in obstacles:
        if abs(ox1 - fx0) <= eps:
            lo, hi = max(oy0, fy0), min(oy1, fy1)
            if hi - lo >= min_run_m:
                runs.append((180.0, fx0, lo, hi))
        if abs(ox0 - fx1) <= eps:
            lo, hi = max(oy0, fy0), min(oy1, fy1)
            if hi - lo >= min_run_m:
                runs.append((0.0, fx1, lo, hi))
        if abs(oy1 - fy0) <= eps:
            lo, hi = max(ox0, fx0), min(ox1, fx1)
            if hi - lo >= min_run_m:
                runs.append((-90.0, fy0, lo, hi))
        if abs(oy0 - fy1) <= eps:
            lo, hi = max(ox0, fx0), min(ox1, fx1)
            if hi - lo >= min_run_m:
                runs.append((90.0, fy1, lo, hi))
    return runs


def _place_plazas(config, layout, placements, resolver, rng, occ, surface_z,
                  usds, default_scale, asset_root, exclusions, tally) -> list:
    """Fountain-or-planter plazas in the large paved gaps of a highrise block.

    See the module section above for the arrangement itself; this is the
    free-space search and the per-plaza compose loop.

    Off unless `city_detail.plazas.enabled` is set, and — critically — a
    no-op that draws NOTHING from *rng* when it is not, so a preset that
    never sets it (every non-GAC preset today) reproduces its RNG sequence
    and its output exactly. Also a no-op if *placements* is empty: this pass
    needs to see where the towers actually stood after `districts` rezoned
    the blocks, which `layout` alone does not carry (`layout["placeholder_
    buildings"]` is build_city's PRE-rezone prisms and goes stale the moment
    a district remaps a block) — the caller has to pass the same placement
    list `districts.remap_buildings` already edited in place.
    """
    cfg = (config.get("city_detail") or {}).get("plazas") or {}
    if not cfg.get("enabled", False):
        return []
    if not placements:
        print("[city_detail] plazas: enabled but no `placements` given — "
              "skipped (pass the post-districts placement list so this pass "
              "can see where the towers are)")
        return []

    from detail import districts

    typ_of = layout.get("_typology_of") or {}
    wanted = set(cfg.get("typologies") or ("highrise",))
    houses = [p for p in placements if p.get("category") == "house"]
    if not houses:
        return []

    inset = districts.block_inset(config, resolver)
    podium_m = float(cfg.get("podium_margin_m", 1.5))
    min_side = float(cfg.get("min_side_m", 15.0))
    min_area = float(cfg.get("min_area_m2", 280.0))
    max_per_block = max(0, int(cfg.get("max_per_block", 2)))
    edge_margin = float(cfg.get("edge_margin_m", 2.5))
    max_radius = float(cfg.get("max_radius_m", 13.0))
    min_radius = float(cfg.get("min_radius_m", 5.0))
    fountain_chance = float(cfg.get("fountain_chance", 0.65))
    # Wider than parks.py's 1.0 m ring_gap_m on purpose: a bench reserved by
    # its own conservative square (see `place_free`) can reach back nearly
    # half its own depth, and there is no trail width here to absorb that
    # the way there is in a park. This is what keeps a bench from clipping
    # the fountain's own keep-out box.
    ring_gap = float(cfg.get("ring_gap_m", 2.2))
    bench_spacing = max(2.0, float(cfg.get("bench_spacing_m", 6.5)))
    trash_n = max(0, int(cfg.get("trash_cans_per_plaza", 2)))
    light_n = max(0, int(cfg.get("streetlights_per_plaza", 4)))
    planter_spacing = max(1.5, float(cfg.get("planter_spacing_m", 5.0)))
    planter_inset = float(cfg.get("planter_inset_m", 1.2))
    cafe_setback = float(cfg.get("cafe_setback_m", 3.0))
    cafe_spacing = max(2.0, float(cfg.get("cafe_spacing_m", 4.5)))
    cafe_min_run = float(cfg.get("cafe_min_run_m", 8.0))
    cluster_radius_frac = float(cfg.get("cluster_radius_frac", 0.35))

    features = (config.get("usds") or {}).get("park_features") or []
    planter_pool = _pool(usds, "planters", default_scale, asset_root)
    rock_pool = _pool(usds, "rocks", default_scale, asset_root)
    bench_pool = (_pool(usds, "benches", default_scale, asset_root, tag="park")
                 or _pool(usds, "benches", default_scale, asset_root))
    trash_pool = _pool(usds, "trash_cans", default_scale, asset_root)
    light_pool = _pool(usds, "streetlights", default_scale, asset_root)
    cafe_pool = _pool(usds, "cafe_sets", default_scale, asset_root)

    if not bench_pool or not (features or planter_pool or rock_pool):
        print("[city_detail] plazas: no bench or centrepiece assets in the "
              "set — skipped")
        return []

    def radius_of(usd, cat, pool):
        fp = resolver.get(usd, cat, scale=pool["scale"](usd),
                          axis_up=pool["axis_up"](usd))
        return math.hypot(fp["sx"], fp["sy"]) / 2.0

    def half_of(usd, cat, pool):
        fp = resolver.get(usd, cat, scale=pool["scale"](usd),
                          axis_up=pool["axis_up"](usd))
        return max(fp["sx"], fp["sy"]) / 2.0

    # Largest conservative half-extent any ring item can draw — MEASURED,
    # not assumed, because the fountain ring math below has to clear it
    # exactly. See that comment for what this is for.
    ring_item_half = max(
        [half_of(u, "bench", bench_pool) for u in bench_pool["paths"]]
        + ([half_of(u, "trash_can", trash_pool) for u in trash_pool["paths"]]
           if trash_pool else [])
        + [0.5])

    out, plaza_tally = [], {}

    def emit(usd, cat, x, y, yaw, pool):
        """Unconditional placement, for a part of a stack whose ground was
        already reserved as one keep-out (the fountain: basin plus three
        co-located water discs, one reservation for all four)."""
        sc, au = pool["scale"](usd), pool["axis_up"](usd)
        fp = resolver.get(usd, cat, scale=sc, axis_up=au)
        out.append({
            "usd": usd, "x_m": x, "y_m": y, "z_m": surface_z + fp["base"],
            "yaw_deg": yaw + pool["yaw"](usd),
            "roll_deg": 90.0 if au == "Y" else 0.0, "pitch_deg": 0.0,
            "scale": sc, "axis_up": au, "category": cat,
        })
        tally[cat] = tally.get(cat, 0) + 1
        plaza_tally[cat] = plaza_tally.get(cat, 0) + 1
        return fp

    def place_free(usd, cat, x, y, yaw, pool, pole=False):
        """Reserve the conservative square a free-yaw prop reserves elsewhere
        in this file (`_occ_extent`'s `_FREE` mode) and place, or skip on
        collision — the ring, the clusters and the edge planters all stand at
        a yaw with no edge to derive an oriented box from.

        *pole* switches to `_POLE`'s reservation instead: a streetlight's arm
        reaches out over nothing (it is meant to overhang the seating, the
        way it overhangs a sidewalk), only the pole stands on the ground, and
        reserving the FULL arm reach as if the whole assembly needed ground
        clearance is what put a light 1 m outside the bench ring's own radius
        into occupancy collision with the bench directly across the 11-45
        degree gap between them — `max(sx,sy)` counted the 2.19 m arm as
        ground footprint.
        """
        fp = resolver.get(usd, cat, scale=pool["scale"](usd),
                          axis_up=pool["axis_up"](usd))
        half = (min(fp["sx"], fp["sy"]) if pole
               else max(fp["sx"], fp["sy"])) / 2.0
        if exclusions and _in_exclusion(x, y, exclusions):
            return None
        if not occ.reserve((x - half, y - half, x + half, y + half)):
            return None
        return emit(usd, cat, x, y, yaw, pool)

    def place_along(usd, cat, x, y, out_yaw, pool):
        """Reserve the true oriented footprint against a wall — the same
        `_occ_extent`/`_half_extents` pair `build()` uses for every sidewalk
        category, reused so a café row occupies exactly what it will occupy
        in the real scene rather than the conservative square."""
        sc, au = pool["scale"](usd), pool["axis_up"](usd)
        fp = resolver.get(usd, cat, scale=sc, axis_up=au)
        along, across = _occ_extent(fp, _ALONG, 0.25)
        hx, hy = _half_extents(out_yaw, along, across)
        if exclusions and _in_exclusion(x, y, exclusions):
            return None
        if not occ.reserve((x - hx, y - hy, x + hx, y + hy)):
            return None
        yaw = _prop_yaw(out_yaw, fp, _ALONG, pool["yaw"](usd), rng)
        return emit(usd, cat, x, y, yaw, pool)

    n_plazas, n_candidates = 0, 0
    kinds = {"fountain": 0, "planters": 0, "rocks": 0, "empty": 0}

    for raw in layout.get("blocks") or ():
        t = typ_of.get(tuple(raw)) or typ_of.get(raw)
        if t not in wanted:
            continue
        bx0, by0, bx1, by1 = _rect(raw)
        rect = (bx0 + inset, by0 + inset, bx1 - inset, by1 - inset)
        obstacles = [districts._rect_of(p, resolver, margin=podium_m)
                    for p in houses
                    if bx0 <= p["x_m"] <= bx1 and by0 <= p["y_m"] <= by1]
        if not obstacles:
            continue

        # build_city's own human/car scatter runs long before this pass and
        # has no idea a plaza is coming — a pedestrian standing where the
        # fountain lands is a real defect (MEASURED on downtown_gac), and
        # nothing else in this file protects against it because `occ` here
        # is fresh, seeded only by this pass's own placements. Towers do not
        # need the same treatment: `free_rects` already excludes them (with
        # `podium_m` to spare) from the ground being searched at all.
        for p in placements:
            if (p.get("category") not in ("human", "car")
                    or not (bx0 <= p["x_m"] <= bx1 and by0 <= p["y_m"] <= by1)):
                continue
            fp = resolver.get(p.get("usd", ""), p.get("category"),
                              scale=p.get("scale"), axis_up=p.get("axis_up", "Z"))
            h = max(fp["sx"], fp["sy"]) / 2.0
            occ.reserve((p["x_m"] - h, p["y_m"] - h,
                        p["x_m"] + h, p["y_m"] + h))

        free = districts.free_rects(rect, obstacles, min_side=min_side)
        free = [f for f in free if (f[2] - f[0]) * (f[3] - f[1]) >= min_area]
        free.sort(key=lambda f: -(f[2] - f[0]) * (f[3] - f[1]))
        n_candidates += len(free)

        for fr in free[:max_per_block]:
            fx0, fy0, fx1, fy1 = fr
            cx, cy = (fx0 + fx1) / 2.0, (fy0 + fy1) / 2.0
            r_avail = min((fx1 - fx0) / 2.0, (fy1 - fy0) / 2.0) - edge_margin
            if r_avail < min_radius - 1e-6:
                continue          # belt-and-braces; min_side already implies this
            r_max = max(min_radius, min(r_avail, max_radius))

            phase = rng.uniform(0.0, 360.0)
            keep_r, kind, ring_r_floor = 0.0, "empty", 0.0

            # ---- the centrepiece: fountain first if it is rolled AND fits,
            # else a planter cluster, else a rock cluster, else nothing.
            if features and rng.random() < fountain_chance:
                feat = rng.choice(features)
                fparts, f_sc, f_au, f_yaw, _tg = _normalize_usd_list(
                    feat.get("parts") or [], default_scale, asset_root)
                if fparts:
                    fpool = {"scale": lambda p: f_sc.get(p, default_scale),
                            "axis_up": lambda p: f_au.get(p, "Z"),
                            "yaw": lambda p: f_yaw.get(p, 0.0)}
                    fr_keep = max([float(feat.get("radius_m", 6.0))]
                                 + [radius_of(p, "park_feature", fpool)
                                    for p in fparts])
                    # fr_keep is a circumscribing-CIRCLE radius (hypot(sx,sy)/2
                    # is rotation-invariant, so it bounds the mesh at every
                    # `spin`, not just axis-aligned). Reserving the matching
                    # circumscribing SQUARE (half=fr_keep) was tried first and
                    # is wrong: that square's own CORNERS sit at fr_keep*sqrt2
                    # from centre — MEASURED, it silently ate every other ring
                    # slot (an exact, phase-independent 50%, because 45 degrees
                    # is exactly half the square's 90 degree period) even
                    # though the true circular mesh never reaches past
                    # fr_keep. The inscribed square (half=fr_keep/sqrt2) never
                    # reserves ground the mesh doesn't occupy, so it can no
                    # longer refuse a ring slot the mesh was never near; ring_r
                    # is what still keeps every ring item outside the true
                    # circle, computed exactly below rather than by the
                    # occupancy grid.
                    q = fr_keep / math.sqrt(2.0)
                    diag_safe_r = fr_keep + (ring_item_half + occ.pad) \
                        * math.sqrt(2.0) * 1.05
                    if diag_safe_r <= r_max and occ.reserve(
                            (cx - q, cy - q, cx + q, cy + q)):
                        spin = rng.uniform(0.0, 360.0)
                        for p in fparts:
                            emit(p, "park_feature", cx, cy, spin, fpool)
                        keep_r, kind, ring_r_floor = fr_keep, "fountain", diag_safe_r

            if kind == "empty" and planter_pool:
                keep_r = min(2.5, max(1.5, r_max * cluster_radius_frac))
                n_p = rng.randint(3, 5)
                placed_any = False
                for i in range(n_p):
                    th = math.radians(phase + i * 360.0 / n_p)
                    if place_free(rng.choice(planter_pool["paths"]),
                                  "planter", cx + math.cos(th) * keep_r,
                                  cy + math.sin(th) * keep_r,
                                  rng.uniform(0.0, 360.0), planter_pool):
                        placed_any = True
                if placed_any:
                    keep_r, kind = keep_r + 1.0, "planters"

            if kind == "empty" and rock_pool:
                keep_r = min(2.0, max(1.0, r_max * cluster_radius_frac * 0.7))
                for _i in range(rng.randint(2, 3)):
                    th = rng.uniform(0.0, 2.0 * math.pi)
                    rr = rng.uniform(0.0, keep_r)
                    place_free(rng.choice(rock_pool["paths"]), "rock",
                              cx + math.cos(th) * rr, cy + math.sin(th) * rr,
                              rng.uniform(0.0, 360.0), rock_pool)
                keep_r, kind = max(keep_r, 1.5), "rocks"

            if kind == "empty":
                # Nothing sourced to stand at the centre — a seating court
                # around open paving, sized the way `parks.py`'s bare
                # `plaza_radius_m` attraction is, not by a keep-out that
                # doesn't exist.
                keep_r = min(4.0, max(2.0, r_max * 0.4))
            kinds[kind] += 1

            # ---- the ring: benches facing inward, evenly pitched, no jitter
            ring_r = min(max(keep_r + ring_gap, ring_r_floor), r_max)
            if ring_r > 1.0:
                n_b = max(3, round(2.0 * math.pi * ring_r / bench_spacing))
                bench_pitch = 360.0 / n_b
                for i in range(n_b):
                    th = phase + i * bench_pitch
                    rad = math.radians(th)
                    # Seat normal is placement yaw + 90 (parks.py's ring
                    # convention — same asset family — see the module note
                    # above): th + 90 points the seat at the centre.
                    place_free(rng.choice(bench_pool["paths"]), "bench",
                              cx + math.cos(rad) * ring_r,
                              cy + math.sin(rad) * ring_r,
                              th + 90.0, bench_pool)
                # Offset by a FRACTION of the bench pitch, not a fixed 45/22.5
                # degrees — MEASURED: n_b came out to exactly 8 on the
                # downtown_gac run, so a fixed +45 degree offset landed a
                # trash can EXACTLY on a bench slot (45 is 360/8) and every
                # one of them was refused by the shared occupancy grid. A
                # fraction of whatever the real pitch is can never land on a
                # bench slot for any n_b.
                for i in range(trash_n if trash_pool else 0):
                    th = math.radians(phase + bench_pitch * 0.5
                                      + i * 360.0 / max(1, trash_n))
                    place_free(rng.choice(trash_pool["paths"]), "trash_can",
                              cx + math.cos(th) * ring_r,
                              cy + math.sin(th) * ring_r,
                              math.degrees(th) + 180.0, trash_pool)
                for i in range(light_n if light_pool else 0):
                    th = math.radians(phase + bench_pitch * 0.25
                                      + i * 360.0 / max(1, light_n))
                    place_free(rng.choice(light_pool["paths"]), "streetlight",
                              cx + math.cos(th) * (ring_r + 1.0),
                              cy + math.sin(th) * (ring_r + 1.0),
                              math.degrees(th) + 180.0, light_pool, pole=True)

            # The plaza's own footprint, not the free rect's — see the module
            # note above on why the café row and the planter line are walked
            # around THIS, a square keyed to the ring, rather than the whole
            # (possibly 100 m long) gap.
            r_plaza = min(ring_r + edge_margin, r_avail + edge_margin)
            plaza_rect = (max(fx0, cx - r_plaza), max(fy0, cy - r_plaza),
                         min(fx1, cx + r_plaza), min(fy1, cy + r_plaza))

            # ---- café row against whichever tower wall borders this gap,
            # clipped to the plaza's own footprint along that wall.
            if cafe_pool:
                centre_of = lambda yaw: cy if yaw in (180.0, 0.0) else cx
                runs = []
                for out_yaw, edge, lo, hi in _plaza_wall_runs(fr, obstacles):
                    c = centre_of(out_yaw)
                    lo2, hi2 = max(lo, c - r_plaza), min(hi, c + r_plaza)
                    if hi2 - lo2 >= cafe_min_run:
                        runs.append((out_yaw, edge, lo2, hi2, hi2 - lo2))
                if runs:
                    runs.sort(key=lambda r: -r[4])
                    out_yaw, edge, lo, hi, run_len = runs[0]
                    n_c = max(1, round(run_len / cafe_spacing))
                    step = run_len / n_c
                    for k in range(n_c):
                        s = lo + (k + 0.5) * step
                        wx, wy = ((edge, s) if out_yaw in (180.0, 0.0)
                                 else (s, edge))
                        px, py = _inset(wx, wy, out_yaw, cafe_setback)
                        place_along(rng.choice(cafe_pool["paths"]),
                                   "cafe_set", px, py, out_yaw, cafe_pool)

            # ---- planters around the plaza's own edge, defining it rather
            # than sprinkled through it — the same edge-walk `build()` uses
            # for sidewalk furniture, run against the plaza's footprint
            # instead of a block. No jitter: an edge line is meant to read
            # as regular.
            if planter_pool:
                for x, y, _oy, _blk, _appr, _corner in _frontage_slots(
                        [plaza_rect], planter_spacing, rng,
                        inset_m=planter_inset):
                    place_free(rng.choice(planter_pool["paths"]), "planter",
                              x, y, rng.uniform(0.0, 360.0), planter_pool)

            n_plazas += 1

    if n_plazas:
        detail = "  ".join(f"{k}={v}" for k, v in sorted(plaza_tally.items()))
        print(f"[city_detail] plazas: {n_plazas} composed of "
              f"{n_candidates} candidate free rect(s) on "
              f"{len(wanted)} typolog{'y' if len(wanted) == 1 else 'ies'} "
              f"(fountain={kinds['fountain']} planters={kinds['planters']} "
              f"rocks={kinds['rocks']} empty={kinds['empty']})\n"
              f"[city_detail]   {detail}")
    elif n_candidates:
        print(f"[city_detail] plazas: 0 composed ({n_candidates} candidate "
              f"free rect(s) found, but 0 fit — check min_side_m/min_area_m2)")
    return out


def build(config: dict, layout: dict, resolver, rng=None,
          district_of=None, placements=None) -> list:
    """Return placement dicts for zoned street furniture.

    *layout* is build_city's second return value (``region``/``blocks``/
    ``road_corridors``). *district_of*, if given, maps a block rect to a
    district dict carrying ``furniture_scale`` — spacings are multiplied by it,
    so an outer ring thins out without a second config tree. *placements*, if
    given, is the placement list AFTER `districts.remap_buildings` has run —
    the caller's own building list, not a copy — and is read (never mutated)
    only by the `city_detail.plazas` pass, to find the towers standing in a
    highrise block; every other pass here ignores it, so a caller that has
    no use for plazas can go on omitting it.

    Emits the same placement schema `apply_placements` consumes, so the caller
    can simply extend build_city's list.
    """
    cfg = config.get("city_detail") or {}
    categories = cfg.get("categories") or {}
    if not categories:
        return []

    rng = rng or random.Random(int(config.get("seed", 0)) + 1013)
    zones_cfg = cfg.get("zones") or {}
    clearances = cfg.get("clearances") or {}
    jitter = float(cfg.get("jitter_frac", 0.1))
    occ = _Occupancy(float(cfg.get("occupancy_cell_m", 4.0)),
                     float(cfg.get("occupancy_pad_m", 0.1)))
    # Gap held between a prop's footprint and the kerb line, so nothing sits
    # flush with the carriageway. Same knob the generator's own passes read.
    curb_margin = float(cfg.get("curb_margin_m",
                                (config.get("frontage") or {})
                                .get("curb_margin_m", 0.3)))
    # Share of a tree's crown width that its trunk and pit actually occupy —
    # reserving the whole canopy would leave room for about one tree per block.
    canopy_frac = float(cfg.get("canopy_footprint_frac", 0.25))
    interior_inset = _interior_inset(zones_cfg)
    # Kerb to building line: the sum of every NACTO zone, i.e. the far edge of
    # `frontage`, the last one. Used by `crown_by_road` as "the sidewalk depth
    # on the tree's own side" — reusing `_zone_bounds` rather than re-summing
    # `_ZONES` so the two stay locked to the same zone widths a preset sets.
    sidewalk_depth_m = _zone_bounds(zones_cfg, "frontage")[1]
    # Same fallback `_class_rank`/`eye_off` already use for a corridor with no
    # measured lane width.
    default_lane_w = float((config.get("roads") or {}).get("lane_width_m", 3.5))
    # How many lateral positions inside a zone a prop may try before giving up.
    # One line at the zone centre makes the kerb a strictly linear budget, and
    # the categories at the end of the queue starve; a real street tucks a bin
    # behind the meter line instead. Tries stay inside the prop's own zone, so
    # the through zone is still guaranteed empty.
    lateral_tries = max(1, int(cfg.get("lateral_tries", 3)))
    # Poles all want the same strip — 197 streetlights, 104 sign posts and four
    # signal heads per signalised junction, every one of them in the furnishing
    # zone. Their footprints are small enough to pack shoulder to shoulder,
    # which reads as a picket fence; this holds them apart by a plausible
    # working clearance instead.
    pole_pad = float(cfg.get("pole_clearance_m", 1.0)) / 2.0
    # How far a prop may slide ALONG the kerb to find a gap. Measured over the
    # whole ring the furnishing line is only ~40% occupied, yet ~40% of the
    # later categories were refused: the loss is not capacity but phase, with
    # several categories' slots landing on the same metre while metres either
    # side stay empty. Sliding turns a collision into the next free spot, which
    # is also how a real street ends up with an evenly filled kerb.
    slide_m = float(cfg.get("slide_m", 3.0))

    usds = config.get("usds") or {}
    default_scale = float(config.get("asset_scale", 1.0))
    asset_root = str(config.get("asset_root", "") or "")
    exclusions = config.get("exclusions") or []
    blocks = [_rect(b) for b in (layout.get("blocks") or [])]
    corridors = layout.get("road_corridors") or []

    surface_z = _sidewalk_top(config, resolver, default_scale, asset_root)

    # ---- traffic control ---------------------------------------------------
    sig_cfg = cfg.get("signals") or {}
    junctions = _junctions(corridors, int(sig_cfg.get("signal_lanes", 4)),
                           set(sig_cfg.get("stop_zones") or ()))
    sight_cfg = cfg.get("sightlines") or {}
    ssd = float(sight_cfg.get("ssd_m", 0.0)) or stopping_sight_distance(
        float(sight_cfg.get("design_speed_kph", 40.0)))
    eye_z = float(sight_cfg.get("eye_height_m", 1.08))      # AASHTO
    face_z = float(sight_cfg.get("sign_face_z_m", 2.1))     # MUTCD 2A.18 urban
    min_obstruct = float(sight_cfg.get("min_obstruction_m", 1.0))
    eye_off = float(sight_cfg.get("eye_offset_m", 0.0))
    if eye_off <= 0.0:
        # Centre of the lane against the kerb: the parking strip it may have to
        # cross, plus half a lane.
        lane_w = float((config.get("roads") or {}).get("lane_width_m", 3.5))
        park = max((float(c.get("park_w", 0.0)) for c in corridors), default=0.0)
        eye_off = park + lane_w / 2.0
    sight = _Sightlines()

    # Tree guards carry their own tree as a payload, so a guarded slot gets the
    # guard *instead of* a bare tree rather than both stacked in one spot.
    guard_pool = _pool(usds, "planter_fences", default_scale, asset_root)

    placed = {}          # category -> [(x, y)], for clearance checks
    out = []
    tally = {}
    dropped = {}
    occluded = {}

    signal_pools = {
        "mast": _pool(usds, str(sig_cfg.get("mast_usds", "traffic_lights_mast")),
                      default_scale, asset_root),
        "pole": _pool(usds, str(sig_cfg.get("pole_usds", "traffic_lights")),
                      default_scale, asset_root),
    }
    if any(signal_pools.values()):
        signals, unplaced = _place_signals(
            junctions, blocks, signal_pools, resolver, rng, occ, surface_z,
            float(sig_cfg.get("corner_margin_m", 1.4)), exclusions, tally,
            pole_pad, float(sig_cfg.get("yaw_offset_deg", 0.0)),
            _stopline_setback(config, sig_cfg))
        out.extend(signals)
        n_sig = sum(1 for j in junctions if j["control"] == "signal")
        print(f"[city_detail] junctions: {len(junctions)} "
              f"({n_sig} signal, {len(junctions) - n_sig} stop) — "
              f"{len(signals)} signal heads, {unplaced} approaches unplaced")

    def _clear_of(name, x, y):
        for other, min_d in (clearances.get(name) or {}).items():
            d2 = float(min_d) ** 2
            for (ox, oy) in placed.get(other, ()):
                if (x - ox) ** 2 + (y - oy) ** 2 < d2:
                    return False
        return True

    pools = {}

    def _pool_of(name):
        if name not in pools:
            pools[name] = _pool(usds, name, default_scale, asset_root)
        return pools[name]

    def _footprint_area(name):
        """Largest ground footprint in the category's pool, for ordering.

        Switched-off categories short-circuit before touching the resolver, so
        ordering never triggers a measurement pass for art the scene won't use.
        """
        spec = categories.get(name) or {}
        if (float(spec.get("spacing_m", 0.0)) <= 0.0
                and float(spec.get("near_corner_m", 0.0)) <= 0.0):
            return 0.0
        pool = _pool_of(name)
        if pool is None:
            return 0.0
        cat = _CATEGORY.get(name, name)
        mode = str(spec.get("orient") or _ORIENT.get(cat, _FACE))
        best = 0.0
        for p in pool["paths"]:
            fp = resolver.get(p, cat, scale=pool["scale"](p),
                              axis_up=pool["axis_up"](p))
            a, b = _occ_extent(fp, mode, canopy_frac)
            best = max(best, a * b)
        return best

    # Distinct phase per category so slots don't all collapse onto the same
    # points along an edge before occupancy even gets a look in.
    order = _order_categories(categories, clearances, _footprint_area)
    phases = {name: (i * 0.37) % 1.0 for i, name in enumerate(order)}

    for name in order:
        spec = categories[name] or {}
        zone = str(spec.get("zone", "furnishing"))
        base_spacing = float(spec.get("spacing_m", 0.0))
        # Corner-anchored categories ignore spacing entirely (their count comes
        # from the block count), so a 0 there means "corner-bound", not "off".
        # Off is still spacing 0 with no near_corner_m.
        if base_spacing <= 0.0 and float(spec.get("near_corner_m", 0.0)) <= 0.0:
            continue

        pool = _pool_of(name)
        if pool is None:
            print(f"[city_detail] {name}: no assets in the set — skipped")
            continue

        cat_default = _CATEGORY.get(name, name)
        mode = str(spec.get("orient") or _ORIENT.get(cat_default, _FACE))
        is_interior = zone == _INTERIOR
        ring_inset = interior_inset if is_interior else 0.0
        inset_d = 0.0 if is_interior else _zone_inset(zones_cfg, zone)
        zone_near, zone_far = ((0.0, 0.0) if is_interior
                               else _zone_bounds(zones_cfg, zone))
        # `control: stop` marks a category as regulatory: it appears only where
        # the junction is stop controlled, and it reserves a sight triangle so
        # nothing tall later hides it. Street-name blades leave it unset and go
        # up at every junction, which is what they do in reality.
        control_gate = str(spec.get("control") or "").lower()
        keeps_sightline = bool(spec.get("sightline", bool(control_gate)))
        guard_chance = float(spec.get("guard_chance", 0.0))
        # Some categories are a corner treatment, not a continuous run.
        # Bollards are the clear case: spaced at their real 1.5 m along every
        # frontage metre they produce thousands of them and read as fencing,
        # where a real street clusters them at crossings and plaza mouths.
        near_corner = float(spec.get("near_corner_m", 0.0))
        # Per-category kerb setback. Street trees want a deeper one than the
        # street-furniture default: codes measure to the TRUNK (Seattle 3'6"
        # centreline-to-kerb, Sedro-Woolley 3 ft for a medium tree), while the
        # generic margin is measured to the footprint edge, which for a canopy
        # is metres out from the trunk.
        cat_curb = float(spec.get("curb_margin_m", curb_margin))

        # Street-tree crown sized to the road each slot fronts, so an avenue
        # canopy does not stand over a row-house street — see `_facing_corridor`
        # / `_max_crown_m`. Config-gated under this one category (a "crown"
        # is a tree concept, not a general street-furniture one) and OFF
        # unless a preset opts in, so every config that does not set
        # `crown_by_road` keeps the old uniform draw exactly. `pool_fps` is
        # measured once per category, not per slot: the pool a slot draws
        # from never changes mid-run, only which member of it fits.
        crown_cfg = (spec.get("crown_by_road") or {}) if name == "street_trees" else {}
        crown_enabled = bool(crown_cfg.get("enabled", False))
        carriage_frac = float(crown_cfg.get("carriage_frac", 0.6))
        crown_min_m = float(crown_cfg.get("min_crown_m", 0.0))
        pool_fps = None
        if crown_enabled:
            pool_fps = [(p, resolver.get(p, cat_default, scale=pool["scale"](p),
                                         axis_up=pool["axis_up"](p)))
                       for p in pool["paths"]]
        n = 0

        # District thins furniture toward the edge by scaling spacing, but the
        # walk still has to cover a whole run at a time — walking block by block
        # is what reintroduces the per-edge problem _frontage_slots exists to
        # avoid. So group blocks by their scale factor and walk each group's
        # frontage as one concatenated ring.
        groups = {}
        for block in blocks:
            scale_f = 1.0
            if district_of is not None:
                d = district_of(block)
                if d:
                    scale_f = float(d.get("furniture_scale", 1.0)) or 1.0
            groups.setdefault(round(scale_f, 3), []).append(block)

        for scale_f, grp in sorted(groups.items()):
            if near_corner > 0.0:
                slots = (s for b in grp for s in _corner_slots(b, near_corner))
            else:
                slots = _frontage_slots(grp, base_spacing * scale_f, rng,
                                        phase=phases[name],
                                        jitter_frac=jitter,
                                        inset_m=ring_inset)

            for px, py, out_yaw, block, approach, corner in slots:
                # Signage that a driver reads only makes sense on the approach
                # side of the junction; the far-side twin is what made corner
                # signs look scattered.
                if mode == _TRAFFIC and near_corner > 0.0 and not approach:
                    continue
                if control_gate and corner is not None and not _signed_here(
                        junctions, block, corner, out_yaw, control_gate):
                    continue

                use_guard = (guard_pool is not None and guard_chance > 0.0
                             and rng.random() < guard_chance)
                src = guard_pool if use_guard else pool
                cat = "planter_fence" if use_guard else cat_default
                slot_mode = _ORIENT.get(cat, _FREE) if use_guard else mode

                # Crown-filtered draw: a guarded slot carries the guard's OWN
                # bundled tree (see the tree-guard note above), not a pick from
                # this pool, so `use_guard` skips the filter same as it skips
                # everything else about the bare-tree pool.
                max_crown = None
                if crown_enabled and not use_guard and pool_fps:
                    corridor = _facing_corridor(px, py, out_yaw, corridors)
                    if corridor is not None:
                        max_crown = _max_crown_m(corridor, sidewalk_depth_m,
                                                 carriage_frac, crown_min_m,
                                                 default_lane_w)

                if max_crown is not None:
                    fits = [t for t in pool_fps
                           if max(t[1]["sx"], t[1]["sy"]) <= max_crown]
                    if not fits:
                        # Nothing this small exists — the smallest species in
                        # the pool is still the right call. A small tree on a
                        # narrow street is correct; no tree at all (dropping
                        # the slot) is a different, worse defect.
                        fits = [min(pool_fps,
                                   key=lambda t: max(t[1]["sx"], t[1]["sy"]))]
                    u, fp = rng.choice(fits)
                    sc, au = src["scale"](u), src["axis_up"](u)
                else:
                    u = rng.choice(src["paths"])
                    sc, au = src["scale"](u), src["axis_up"](u)
                    fp = resolver.get(u, cat, scale=sc, axis_up=au)

                along, across = _occ_extent(fp, slot_mode, canopy_frac)
                if slot_mode == _POLE:
                    along, across = along + pole_pad * 2.0, across + pole_pad * 2.0
                hx, hy = _half_extents(out_yaw, along, across)
                # Half the road-facing depth plus the margin, so the footprint
                # clears the kerb line even where the zone itself is narrower
                # than the prop (a 2.4 m café set in a 0.6 m frontage zone).
                floor = across / 2.0 + cat_curb
                spot = None
                # Corner-anchored props are where they are on purpose, so they
                # never slide; a stop sign 3 m further along is a stop sign in
                # the wrong place.
                slides = ((0.0,) if near_corner > 0.0 or slide_m <= 0.0
                          else (0.0, slide_m / 2.0, -slide_m / 2.0,
                                slide_m, -slide_m))
                ax, ay = _unit(out_yaw + 90.0)
                for slide, want in ((s_, w_) for s_ in slides
                                    for w_ in _lateral_insets(
                                        inset_d, across, zone_near, zone_far,
                                        lateral_tries, is_interior)):
                    x, y = _inset(px + ax * slide, py + ay * slide, out_yaw,
                                  max(want, floor))
                    fitted = _fit_in_block(x, y, hx, hy, block, cat_curb)
                    if fitted is None:
                        continue
                    x, y = fitted
                    if exclusions and _in_exclusion(x, y, exclusions):
                        continue
                    if not _clear_of(name, x, y):
                        continue
                    rect = (x - hx, y - hy, x + hx, y + hy)
                    if fp["sz"] > min_obstruct and sight.occludes(rect,
                                                                  fp["sz"]):
                        occluded[name] = occluded.get(name, 0) + 1
                        continue
                    if not occ.reserve(rect):
                        continue
                    spot = (x, y, max(want, floor))
                    break
                if spot is None:
                    dropped[name] = dropped.get(name, 0) + 1
                    continue
                x, y, used_inset = spot

                if keeps_sightline:
                    # The sign faces the traffic it governs, so its facing IS
                    # the upstream direction of the approach it has to be seen
                    # from.
                    sight.reserve(x, y, out_yaw + 90.0, out_yaw, used_inset,
                                  eye_off, ssd,
                                  min(face_z, surface_z + fp["sz"]), eye_z)

                out.append({
                    "usd": u,
                    "x_m": x, "y_m": y,
                    "z_m": surface_z + fp["base"],
                    "yaw_deg": _prop_yaw(out_yaw, fp, slot_mode,
                                         src["yaw"](u), rng),
                    "roll_deg": 90.0 if au == "Y" else 0.0,
                    "pitch_deg": 0.0,
                    "scale": sc,
                    "axis_up": au,
                    "category": cat,
                })
                # Recorded under both names: `cat` for readability, `name`
                # because `clearances` is keyed on the config's plural keys.
                placed.setdefault(name, []).append((x, y))
                if cat != name:
                    placed.setdefault(cat, []).append((x, y))
                tally[cat] = tally.get(cat, 0) + 1
                n += 1

        if n == 0:
            print(f"[city_detail] {name}: 0 placed "
                  f"(spacing {base_spacing} m, zone {zone}) — check clearances")

    # Plazas last: they read the SAME `occ` grid every sidewalk category above
    # just filled, so a plaza's furniture can never land on top of a kerb prop
    # even though the two passes never coordinate directly. Gated and RNG-free
    # when off — see `_place_plazas` — so this cannot perturb the sequence
    # every category above already drew.
    out.extend(_place_plazas(config, layout, placements, resolver, rng, occ,
                             surface_z, usds, default_scale, asset_root,
                             exclusions, tally))

    total = sum(tally.values())
    detail = "  ".join(f"{k}={v}" for k, v in sorted(tally.items()))
    print(f"[city_detail] {total} props across {len(tally)} categories "
          f"(surface z={surface_z:.3f})\n[city_detail]   {detail}")
    if dropped:
        crowded = "  ".join(f"{k}={v}" for k, v in sorted(dropped.items()))
        print(f"[city_detail]   no room: {crowded}")
    if occluded:
        blind = "  ".join(f"{k}={v}" for k, v in sorted(occluded.items()))
        print(f"[city_detail]   moved off a sign's sightline: {blind}")
    return out
