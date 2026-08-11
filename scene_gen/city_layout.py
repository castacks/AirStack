"""
city_layout.py — anisotropic block subdivision.

Additive: this does not edit `scene_generator`. It builds a drop-in replacement
for `_subdivide_region_metric` with the same signature and return shape, and
`generate_city_v2.py` swaps it in around its `build_city` call and restores it
afterwards. Configs that don't ask for it are untouched, and the v1 launch
script never imports this module at all.

WHAT'S WRONG WITH THE BUILT-IN ONE
----------------------------------
`_subdivide_region_metric` always splits the longer axis::

    if can_x and (w >= h or not can_y):

Halving whichever side is longer drives every block toward 1:1, and it does:
measured over a 400x400 m region, the built-in subdivision yields a median
long:short ratio of **1.2x** with a 38 m short side.

Real downtown grids are strongly directional. Manhattan runs ~80 m between
streets and ~280 m between avenues — roughly 1:3.5 — and the two road classes
differ as well: the sparse axis carries the wide avenues, the dense axis the
narrow streets. Square blocks and uniform road widths are the two things that
most make a generated grid read as generated.

HOW THIS ONE WORKS
------------------
Instead of "split the longer side", each axis gets its own target extent, and
the recursion splits whichever axis most exceeds *its own* target. Blocks
therefore converge on the configured aspect rather than on a square. The axis
that ends up split rarely is the one whose roads are promoted to main roads, so
wide avenues land far apart and narrow streets close together without needing a
separate pass.

VARIABLE-WIDTH CORRIDORS
------------------------
A constant-width ribbon is the third tell. A real street reserves a kerb strip
either side of the carriageway; where parking is allowed the strip is asphalt,
and where it isn't the pavement is built out over it (a kerb extension), so the
road is genuinely narrower there and the carriageway can borrow the strip for a
turn lane at junctions.

The decision is per CORRIDOR, not per block face. A street has one parking
regime end to end — rolling it per face gives a dashed patchwork that reads as
noise rather than as policy. Each corridor draws ``none`` / ``one_side`` /
``both`` from its zone's weights, so a scene contains streets with no parking at
all, streets with a single parking lane, and streets with two.

Each split reserves ``n_lanes * lane_w + 2 * parking_lane_m`` regardless of the
policy, which keeps the carriageway centred in the rect — `apply_ground_planes`
draws the yellow centre line at the rect centre, so an asymmetric rect would put
it in a parking lane. The strips a corridor's policy doesn't use are handed to
the abutting blocks instead, which get the land back as pavement.

That full width is what the corridor rect reports: everything downstream
(`apply_ground_planes`, the cars pass, `city_detail`'s frontage walk,
`road_markings`) consumes corridors as plain x0/y0/x1/y1 rects, so the rect has
to stay the MAXIMUM extent or the ground stops covering the road. The rest rides
along as optional keys:

    "carriage":   [c0, c1]  cross-axis span of the travelled way (centred)
    "park_w":     float     reserved kerb strip per side, 0 on the border ring
    "parking":    "none" | "lo" | "hi" | "both"   which strips stay asphalt
    "zone":       district ring name, or None when zoning is off
    "road_class": "arterial" | "collector" | "local"

A corridor carrying none of these behaves exactly as before, so the built-in
subdivider still works with `road_markings`.

ROAD HIERARCHY
--------------
Lane counts and parking weights come from the district ring the corridor sits
in, looked up through `districts.assign` off the region geometry alone. A core
corridor is more often promoted to an avenue and less often allows parking; a
residential edge street is narrow and parks on both sides. ``zone`` and
``road_class`` are published per corridor so `city_detail` can tell an arterial
junction (signal) from a minor one (stop sign) — see :func:`junction_control`.
"""

from scene_generator import _jitter_posf

# Blocks are cut straight from the corridor edges, so an abutting face matches
# to within float noise rather than approximately.
_TOL = 1e-4


def _rng_range(v, fallback):
    """Accept ``[lo, hi]`` or a scalar for a range-valued knob."""
    if isinstance(v, (list, tuple)) and len(v) >= 2:
        return float(v[0]), float(v[1])
    if isinstance(v, (int, float)):
        return float(v), float(v)
    return fallback


def _weighted(weights: dict, rng, fallback: str):
    """Draw a key from a ``{name: weight}`` mapping."""
    items = [(k, float(v)) for k, v in (weights or {}).items() if float(v) > 0.0]
    total = sum(w for _k, w in items)
    if total <= 0.0:
        return fallback
    r = rng.random() * total
    for k, w in items:
        r -= w
        if r <= 0.0:
            return k
    return items[-1][0]


def road_class(n_lanes: int) -> str:
    """Functional class from lane count, the only hierarchy signal a corridor
    carries that every consumer already understands."""
    if n_lanes >= 4:
        return "arterial"
    if n_lanes == 3:
        return "collector"
    return "local"


def junction_control(a: dict, b: dict, signal_lanes: int = 4) -> str:
    """``"signal"`` or ``"stop"`` for a junction between corridors *a* and *b*.

    Published here rather than left to each consumer so `city_detail`'s stop
    signs and the generator's traffic lights agree about which junctions are
    arterial.

    BOTH legs have to be arterial, not either. Signalising on ``max`` means a
    two-lane side street meeting an avenue becomes a full signalised junction,
    and since half the corridors in a downtown are arterial that swallows nearly
    every crossing: measured, 38 of 50 junctions signalised and only 10 stop
    signs in the whole city. A minor road meeting a main road is ordinarily a
    two-way stop — the side street stops, the arterial runs — and a signal is
    reserved for where two main roads actually cross.
    """
    lanes = (int(a.get("n_lanes", 2)), int(b.get("n_lanes", 2)))
    return "signal" if min(lanes) >= int(signal_lanes) else "stop"


def _faces(c, blocks, lo_edge, hi_edge):
    """Yield ``(block index, side, grow axis, s0, s1)`` for every block face
    lying on *lo_edge* / *hi_edge* of corridor *c*, with the run overlap."""
    ns = c["dir"] == "ns"
    run_lo, run_hi = (c["y0"], c["y1"]) if ns else (c["x0"], c["x1"])
    for i, (bx0, by0, bx1, by1) in enumerate(blocks):
        b_run_lo, b_run_hi = (by0, by1) if ns else (bx0, bx1)
        s0, s1 = max(run_lo, b_run_lo), min(run_hi, b_run_hi)
        if s1 - s0 <= _TOL:
            continue
        b_lo, b_hi = (bx0, bx1) if ns else (by0, by1)
        if abs(b_hi - lo_edge) <= _TOL:
            yield i, "lo", (2 if ns else 3), s0, s1
        elif abs(b_lo - hi_edge) <= _TOL:
            yield i, "hi", (0 if ns else 1), s0, s1


def _apply_kerb_extensions(blocks, corridors):
    """Give each strip the corridor's policy doesn't park on to its blocks.

    Runs once over the finished BSP because a corridor is bordered by however
    many leaves the recursion happened to produce, and that isn't known while
    splitting.

    A block face can meet two corridors — a split landing on the same coordinate
    in two subtrees, which with ``split_jitter: 0`` is not even unlikely — and a
    face is one straight edge, so it can only be built out when *every* corridor
    it meets has ceded that strip. Anything else would put pavement over a live
    parking lane.

    Growth only ever widens a block, and only into a strip reserved for it, so
    blocks still can't overlap: two faces across a corridor are separated by the
    carriageway, two along it cover disjoint runs, and corner bulbs at a junction
    are separated by the crossing road's own carriageway.

    Returns the grown block list.
    """
    grow, veto = {}, set()
    for c in corridors:
        park_w = float(c.get("park_w", 0.0))
        if park_w <= 0.0:
            continue
        parking = c.get("parking", "both")
        cross_lo, cross_hi = ((c["x0"], c["x1"]) if c["dir"] == "ns"
                              else (c["y0"], c["y1"]))
        for i, side, axis, _s0, _s1 in _faces(c, blocks, cross_lo, cross_hi):
            if parking == "both" or parking == side:
                veto.add((i, axis))          # strip is a live parking lane
            else:
                grow[(i, axis)] = park_w if side == "lo" else -park_w

    out = []
    for i, b in enumerate(blocks):
        d = [0.0, 0.0, 0.0, 0.0]
        for axis in range(4):
            if (i, axis) in grow and (i, axis) not in veto:
                d[axis] = grow[(i, axis)]
        out.append((b[0] + d[0], b[1] + d[1], b[2] + d[2], b[3] + d[3]))
    return out


# Park superblocks reserved by the last subdivision, as final block rects.
# `build_city` composes the layout dict itself and cannot be edited to carry
# them, so they are published here; one city is built per process, so a module
# global is honest rather than merely convenient. `districts.park_blocks` reads
# it and falls back to trail-tile detection when it is empty.
PARK_RESERVES: list = []


def _park_reserves(aniso, config, x0, y0, x1, y1, targets, hw, rng):
    """Choose park superblock rects inside the interior rect.

    Sized as a multiple of the LOCAL block target rather than in absolute
    metres, so a park stays proportionate to its neighbourhood's grain: a park
    in a row-house district is a garden square, one downtown is a civic park.
    """
    cfg = aniso.get("parks") or {}
    if not cfg.get("enabled", True):
        return []
    lo, hi = _rng_range(cfg.get("count"), (1.0, 2.0))
    want = rng.randint(int(lo), int(hi))
    if want <= 0:
        return []
    scale_lo, scale_hi = _rng_range(cfg.get("block_scale"), (1.3, 2.1))
    margin = float(cfg.get("margin_m", 40.0))
    min_sep = float(cfg.get("min_separation_m", 220.0))

    out = []
    for _ in range(want):
        for _try in range(60):
            cx = rng.uniform(x0 + margin, x1 - margin)
            cy = rng.uniform(y0 + margin, y1 - margin)
            t_min_x, t_max_x, t_min_y, t_max_y = targets(cx, cy)
            w = (t_min_x + t_max_x) / 2.0 * rng.uniform(scale_lo, scale_hi)
            h = (t_min_y + t_max_y) / 2.0 * rng.uniform(scale_lo, scale_hi)
            r = (cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0)
            # Leave room for the bounding street plus a FULL block either side.
            # A smaller margin leaves an unsplittable 10-20 m strip against the
            # region edge, which comes back as a sliver block and puts a second
            # corridor within a few metres of the park's own frontage — the
            # overlapping-corridor pairs seen downstream.
            pad = hw + max(t_max_x, t_max_y)
            if not (x0 + pad < r[0] and r[2] < x1 - pad
                    and y0 + pad < r[1] and r[3] < y1 - pad):
                continue
            if any(max(abs(cx - (q[0] + q[2]) / 2.0),
                       abs(cy - (q[1] + q[3]) / 2.0)) < min_sep for q in out):
                continue
            out.append(r)
            break
    return out


def make_subdivider(layout_cfg: dict, config: dict = None):
    """Return a replacement for `scene_generator._subdivide_region_metric`.

    Signature and return shape are identical — ``(blocks, road_corridors)``
    with blocks as ``(x0, y0, x1, y1)`` and corridors as
    ``{"x0", "y0", "x1", "y1", "n_lanes", "dir"}`` — because everything
    downstream (packing, frontage, markings, cars) consumes exactly that. The
    width profile and hierarchy ride along as extra keys (``carriage``,
    ``park_w``, ``parking``, ``zone``, ``road_class``).

    *config* is the whole scene config, needed only to look up district rings so
    a corridor can size itself to its zone. Without it every corridor falls back
    to the un-zoned knobs.
    """
    aniso = layout_cfg.get("anisotropic") or {}
    short_lo, short_hi = _rng_range(aniso.get("block_short_m"), (55.0, 85.0))
    long_lo, long_hi = _rng_range(aniso.get("block_long_m"), (150.0, 260.0))
    long_axis = str(aniso.get("long_axis", "x")).lower()
    stop_chance = float(aniso.get("stop_chance", 0.85))
    # 2.4 m is the standard parallel-parking lane. It is reserved on both sides
    # of every interior road whatever the policy turns out to be: an asymmetric
    # rect would move the rect centre off the carriageway centre, and that is
    # where apply_ground_planes draws the yellow centre line.
    park_w = max(0.0, float(aniso.get("parking_lane_m", 2.4)))
    zones = aniso.get("zones") or {}
    base_policy = aniso.get("parking_policy") or {"both": 1.0}

    def zone_knob(zone, key, fallback):
        z = zones.get(zone) or {}
        v = z.get(key)
        return aniso.get(key, fallback) if v is None else v

    def subdivide(w_m, h_m, min_block_m, max_block_m, jitter, rng, roads_cfg):
        lane_w = float(roads_cfg.get("lane_width_m", 3.5))
        main_lanes = int(roads_cfg.get("main_road_lanes", 4))
        sec_lanes = int(roads_cfg.get("secondary_road_lanes", 2))
        brd_lanes = int(roads_cfg.get("border_lanes", 2))

        # Zoning is a pure function of the region rectangle, so it can be
        # resolved here without waiting for build_city to hand back a layout.
        zone_at, ring_names = None, []
        if zones and config:
            try:
                import districts
                zone_at, rings = districts.assign(
                    config, {"region": (-w_m / 2.0, -h_m / 2.0,
                                        w_m / 2.0, h_m / 2.0)})
                ring_names = [str(r.get("name")) for r in rings]
                if not rings:
                    zone_at = None
            except ImportError:
                zone_at = None
        ring_of = {n: i for i, n in enumerate(ring_names)}

        def zone_of(cross, r0, r1, ns):
            """Median ring over the corridor's run.

            A single point won't do: the rings are concentric and a road that
            crosses the whole region passes through all of them, so sampling
            only its midpoint makes every long corridor a core corridor and the
            outer ring never appears. The median is what a road is *mostly* in,
            which is the character it should take.
            """
            if zone_at is None:
                return None
            idx = []
            for k in range(5):
                s = r0 + (r1 - r0) * (k / 4.0)
                d = zone_at(*((cross, s) if ns else (s, cross)))
                idx.append(ring_of.get(str((d or {}).get("name")), 0))
            idx.sort()
            return ring_names[idx[len(idx) // 2]]

        # Which world axis carries the long block dimension. With long_axis "x"
        # blocks are wide in X and shallow in Y, so the X splits are the rare
        # ones and their (north-south) roads become the avenues.
        if long_axis == "x":
            min_x, max_x = long_lo, long_hi
            min_y, max_y = short_lo, short_hi
        else:
            min_x, max_x = short_lo, short_hi
            min_y, max_y = long_lo, long_hi

        # Block size follows LAND USE, not the whole-region average. Six of the
        # ten stock buildings are 50-81 m wide and simply cannot be placed on a
        # 46 m block, so a downtown has to be cut coarser than a row-house
        # district or half the library is unplaceable. districts.zone_field
        # blends its targets across an area boundary, so the grain grades
        # rather than stepping in one street.
        area_at = None
        if config:
            try:
                import districts
                area_at = districts.zone_field(
                    config, (-w_m / 2.0, -h_m / 2.0, w_m / 2.0, h_m / 2.0))
                if not hasattr(area_at, "targets_at"):
                    area_at = None
            except ImportError:
                area_at = None

        def targets(x, y):
            """(min_x, max_x, min_y, max_y) for the cell centred at (x, y)."""
            t = area_at.targets_at(x, y) if area_at is not None else None
            if not t:
                return min_x, max_x, min_y, max_y
            s_lo, s_hi, l_lo, l_hi = t
            if long_axis == "x":
                return l_lo, l_hi, s_lo, s_hi
            return s_lo, s_hi, l_lo, l_hi

        border_w = brd_lanes * lane_w
        half_w, half_h = w_m / 2.0, h_m / 2.0
        road_corridors = []

        def border(x0, y0, x1, y1, d):
            """One region-edge road. No parking strip: the region edge is fixed,
            so widening it would move the world boundary."""
            ns = d == "ns"
            span = (x0, x1) if ns else (y0, y1)
            run = (y0, y1) if ns else (x0, x1)
            return {"x0": x0, "y0": y0, "x1": x1, "y1": y1,
                    "n_lanes": brd_lanes, "dir": d, "carriage": list(span),
                    "park_w": 0.0, "parking": "none",
                    "zone": zone_of(sum(span) / 2.0, run[0], run[1], ns),
                    "road_class": road_class(brd_lanes)}

        road_corridors.append(border(-half_w, -half_h,
                                     half_w, -half_h + border_w, "ew"))
        road_corridors.append(border(-half_w, half_h - border_w,
                                     half_w, half_h, "ew"))
        road_corridors.append(border(-half_w, -half_h + border_w,
                                     -half_w + border_w, half_h - border_w, "ns"))
        road_corridors.append(border(half_w - border_w, -half_h + border_w,
                                     half_w, half_h - border_w, "ns"))

        blocks = []
        # The narrowest road a split can produce, kerb strips included, since
        # that is the whole width a cell has to give up to be splittable.
        min_road = sec_lanes * lane_w + 2.0 * park_w

        def road_at(axis, sp, x0, y0, x1, y1, zone=None, lanes=None,
                    width=None):
            """Emit one corridor centred on *sp*; return the two child rects.

            Split out so a forced cut (isolating a park superblock) reuses the
            same width profile, zoning and parking policy as a random one.

            ``width`` overrides the whole reservation, kerb strips included, for
            a mews lane between terrace rows -- see ``split``. Such a lane is
            all carriageway: a 6 m back street does not also give up 4.8 m to
            parking, and pretending it does is what makes the block too big.
            """
            ns = axis == "x"
            if zone is None:
                # Zoned off the CELL, not the finished corridor: the corridor's
                # position depends on its width, which depends on the zone. The
                # run is exact either way — a split spans its cell end to end.
                zone = (zone_of((x0 + x1) / 2.0, y0, y1, True) if ns
                        else zone_of((y0 + y1) / 2.0, x0, x1, False))
            if lanes is None:
                is_long = (axis == long_axis)
                chance = float(zone_knob(
                    zone, "main_road_chance_long" if is_long
                    else "main_road_chance_short", 0.85 if is_long else 0.05))
                lanes = int(zone_knob(zone, "lanes_main", main_lanes)
                            if rng.random() < chance
                            else zone_knob(zone, "lanes_secondary", sec_lanes))
            policy = _weighted(zone_knob(zone, "parking_policy", base_policy),
                               rng, "both")
            if policy == "one_side":
                policy = "lo" if rng.random() < 0.5 else "hi"
            if park_w <= 0.0:
                policy = "none"

            kerb = park_w
            if width is not None:
                policy, kerb = "none", 0.0
                carriage_w = width
                lanes = max(1, int(round(width / lane_w)))
                half_road = half_car = width / 2.0
            else:
                carriage_w = lanes * lane_w
                half_road = (carriage_w + 2.0 * park_w) / 2.0
                half_car = carriage_w / 2.0
            common = {"n_lanes": lanes, "park_w": kerb, "parking": policy,
                      "zone": zone, "road_class": road_class(lanes)}
            if ns:
                road_corridors.append(dict(
                    common, x0=sp - half_road, y0=y0, x1=sp + half_road, y1=y1,
                    dir="ns", carriage=[sp - half_car, sp + half_car]))
                return ((x0, y0, sp - half_road, y1),
                        (sp + half_road, y0, x1, y1))
            road_corridors.append(dict(
                common, x0=x0, y0=sp - half_road, x1=x1, y1=sp + half_road,
                dir="ew", carriage=[sp - half_car, sp + half_car]))
            return ((x0, y0, x1, sp - half_road),
                    (x0, sp + half_road, x1, y1))

        def split(axis, x0, y0, x1, y1):
            """Cut one or more roads along *axis* and recurse into the children.

            Bisection alone quantises the reachable block sizes, and coarsely.
            A cell is splittable only at ``2 * t_min + min_road``, so one just
            under that is stuck at full width while one just over halves to far
            below target. Measured: row-house blocks asked for 53 m came out
            88 m, and 88 is a dead end -- it would need ``t_min <= 38`` to split
            at all, and then yields 38. Nothing between 38 and 88 is reachable
            at any setting, so no config could have fixed it.

            A cell three or more targets wide is therefore cut into k even
            pieces in one pass instead of halved, at a width known up front so
            every child lands exactly on target.

            The width of those interior cuts is the only real control over how
            big the children come out, because ``k`` is forced: a strip holds
            ``(extent - (k-1)*road) / k``, and dropping to k+1 has to clear the
            typology's floor or it is refused. Measured, the 212 m row-house
            strip at the 11.4 m default gives k=3 and 63 m blocks -- 10 m more
            than a terrace pair uses, and k=4 would give 44 m, under the 52.7 m
            a pair physically needs. At a 6 m mews width the same strip takes
            k=4 at 48.5 m, which the pair fills exactly. Hence
            ``even_split_road_m``: a back lane between terrace rows is the one
            street that should be narrow, and widening arterials to buy the same
            10 m would need a 35 m carriageway.
            """
            tx0, tx1, ty0, ty1 = targets((x0 + x1) / 2.0, (y0 + y1) / 2.0)
            if axis == "x":
                t_lo, t_hi, extent, base = tx0, tx1, x1 - x0, x0
            else:
                t_lo, t_hi, extent, base = ty0, ty1, y1 - y0, y0
            t_mid = (t_lo + t_hi) / 2.0
            if t_mid > 0.0:
                ns = axis == "x"
                z = (zone_of((x0 + x1) / 2.0, y0, y1, True) if ns
                     else zone_of((y0 + y1) / 2.0, x0, x1, False))
                mews = float(zone_knob(z, "even_split_road_m", min_road))
                k = int(round((extent + mews) / (t_mid + mews)))
                child = (extent - (k - 1) * mews) / k if k > 0 else extent
                # k>=3 is the "bisection cannot reach the target" case. k==2
                # also needs rescuing when the ordinary cut is blocked: a cell
                # a shade under 2*t_lo + min_road cannot bisect at all and
                # freezes at full width -- MEASURED, 102.6 m against a 103.4 m
                # threshold, missing by 0.8 m and leaving a block twice its
                # target. The narrower mews clears it and halves to 48.3.
                # Anything that CAN bisect still does, keeping its jitter.
                stuck = k == 2 and extent < 2.0 * t_lo + min_road
                if (k >= 3 or stuck) and child >= t_lo:
                    rect = (x0, y0, x1, y1)
                    for i in range(k - 1):
                        sp = base + i * (child + mews) + child + mews / 2.0
                        near, rect = road_at(axis, sp, *rect, zone=z,
                                             width=mews)
                        _recurse(*near)
                    _recurse(*rect)
                    return

            lead = t_lo
            # Widest road this cut could turn into, so the margin holds
            # whichever width road_at picks.
            worst = (max(int(main_lanes), int(sec_lanes)) * lane_w
                     + 2.0 * park_w) / 2.0
            lo = (x0 if axis == "x" else y0) + lead + worst
            hi = (x1 if axis == "x" else y1) - lead - worst
            if lo >= hi:
                blocks.append((x0, y0, x1, y1))
                return
            sp = _jitter_posf(lo, hi, jitter, rng)
            for child in road_at(axis, sp, x0, y0, x1, y1):
                _recurse(*child)

        def _recurse(x0, y0, x1, y1):
            w, h = x1 - x0, y1 - y0
            if w <= 1e-3 or h <= 1e-3:
                return
            t_min_x, t_max_x, t_min_y, t_max_y = targets((x0 + x1) / 2.0,
                                                         (y0 + y1) / 2.0)
            # Splittable against the NARROWEST road this cell could actually be
            # cut with, which is the mews if the zone defines one. Testing the
            # full width here rejected cells that `split` would have cut
            # happily: a 102.6 m cell failed 2*46 + 11.4 = 103.4 by 0.8 m and
            # was emitted whole at twice its target, when the 6 m mews needs
            # only 98.0 and halves it to 48.3.
            zc = zone_of((x0 + x1) / 2.0, y0, y1, True)
            road_floor = min(min_road,
                             float(zone_knob(zc, "even_split_road_m",
                                             min_road)))
            can_x = w >= 2.0 * t_min_x + road_floor
            can_y = h >= 2.0 * t_min_y + road_floor
            if not (can_x or can_y):
                blocks.append((x0, y0, x1, y1))
                return

            # Excess relative to each axis's OWN target — this is the whole
            # difference from the built-in, which compares w against h and so
            # always drives toward a square.
            over_x = w / t_max_x
            over_y = h / t_max_y
            if over_x <= 1.0 and over_y <= 1.0:
                if rng.random() < stop_chance:
                    blocks.append((x0, y0, x1, y1))
                    return

            want_x = over_x >= over_y
            if want_x and can_x:
                split("x", x0, y0, x1, y1)
            elif can_y:
                split("y", x0, y0, x1, y1)
            elif can_x:
                split("x", x0, y0, x1, y1)
            else:
                blocks.append((x0, y0, x1, y1))

        park_lanes = int(zone_knob("park_edge", "lanes_secondary", sec_lanes))
        park_hw = (park_lanes * lane_w + 2.0 * park_w) / 2.0

        def carve(rect, reserves):
            """Isolate each park reserve as its own leaf, then subdivide around it.

            A park cannot just be "a block the packer skips": the BSP puts a
            road wherever it splits, so two adjacent park blocks come out with a
            street between them and read as two half-parks rather than one park.
            Cutting the reserve out FIRST and emitting it whole is the only way
            to get a park with no road through it — and the four cuts become the
            streets that bound it, which is what a real park has.
            """
            x0, y0, x1, y1 = rect
            here = [r for r in reserves
                    if r[0] >= x0 - 1e-6 and r[2] <= x1 + 1e-6
                    and r[1] >= y0 - 1e-6 and r[3] <= y1 + 1e-6]
            if not here:
                _recurse(x0, y0, x1, y1)
                return
            r, rest = here[0], [q for q in reserves if q is not here[0]]
            cell = rect
            # Four guillotine cuts, one street per side. A cut is skipped when
            # the park already abuts the region's border road on that side.
            #
            # The frontages take their ring's own hierarchy rather than being
            # forced to secondary — a real urban park sits on at least one
            # arterial (Central Park has avenues on both long edges, Bryant
            # Park fronts Sixth Avenue), and forcing four local streets both
            # misreads the city and flattens the road hierarchy. One long
            # frontage is promoted outright so there is always an avenue on it.
            first = True
            for axis, sp, park_is_hi in (("x", r[0] - park_hw, True),
                                         ("x", r[2] + park_hw, False),
                                         ("y", r[1] - park_hw, True),
                                         ("y", r[3] + park_hw, False)):
                lo_b, hi_b = (cell[0], cell[2]) if axis == "x" else (cell[1],
                                                                     cell[3])
                if not (lo_b + 1.0 < sp - park_hw and sp + park_hw < hi_b - 1.0):
                    continue
                lo_rect, hi_rect = road_at(
                    axis, sp, *cell,
                    lanes=(int(zone_knob(None, "lanes_main", main_lanes))
                           if first else None))
                first = False
                outside, cell = ((lo_rect, hi_rect) if park_is_hi
                                 else (hi_rect, lo_rect))
                carve(outside, rest)
            blocks.append(cell)
            # Index, not the rect: _apply_kerb_extensions grows blocks into the
            # parking strips their corridor doesn't use, so the rect recorded
            # here is stale by the time the subdivision returns.
            park_leaves.append(len(blocks) - 1)

        park_leaves: list = []
        reserves = _park_reserves(aniso, config,
                                  -half_w + border_w, -half_h + border_w,
                                  half_w - border_w, half_h - border_w,
                                  targets, park_hw, rng)
        carve((-half_w + border_w, -half_h + border_w,
               half_w - border_w, half_h - border_w), reserves)

        blocks = _apply_kerb_extensions(blocks, road_corridors)
        PARK_RESERVES[:] = [blocks[i] for i in park_leaves]
        tally = {}
        for c in road_corridors:
            key = (c.get("zone") or "-", c["n_lanes"], c.get("parking"))
            tally[key] = tally.get(key, 0) + 1
        detail = "  ".join(f"{z}/{n}L/{p}={k}"
                           for (z, n, p), k in sorted(tally.items()))
        print(f"[city_layout] {len(blocks)} blocks, {len(road_corridors)} "
              f"corridors, {len(PARK_RESERVES)} park superblock(s) "
              f"(parking strip {park_w:.1f} m/side)\n"
              f"[city_layout]   {detail}")
        return blocks, road_corridors

    return subdivide


class patched:
    """Context manager swapping the generator's subdivider for the run.

    Monkey-patching rather than a parameter because `build_city` calls
    `_subdivide_region_metric` internally and this work may not edit that file.
    The swap is scoped and restored in `finally`, and only `generate_city_v2`
    ever enters it — the original launch path never imports this module, so v1
    scenes are bit-identical.
    """

    def __init__(self, config: dict):
        self._full = config
        self._cfg = (config.get("layout") or {})
        self._enabled = bool((self._cfg.get("anisotropic") or {}).get("enabled"))
        self._orig = None

    def __enter__(self):
        if not self._enabled:
            return False
        import scene_generator as sg
        self._orig = sg._subdivide_region_metric
        sg._subdivide_region_metric = make_subdivider(self._cfg, self._full)
        a = self._cfg.get("anisotropic") or {}
        print(f"[city_layout] anisotropic subdivision: "
              f"short={a.get('block_short_m')} long={a.get('block_long_m')} "
              f"long_axis={a.get('long_axis', 'x')}")
        return True

    def __exit__(self, *exc):
        if self._orig is not None:
            import scene_generator as sg
            sg._subdivide_region_metric = self._orig
        return False
