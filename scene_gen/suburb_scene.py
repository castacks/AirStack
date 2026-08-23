"""
suburb_scene.py — build a `suburb_net` layout onto a live USD stage.

WHY NOT `apply_ground_planes`: it consumes blocks and corridors as AXIS-ALIGNED
RECTS, which is exactly what this layout does not have — streets are polyline
centrelines and blocks are polygons. Paving a curved street's bounding box is
the envelope trick the rewrite exists to avoid. So the ground is written here:

    asphalt     one ribbon mesh per street swept along its centreline, plus a
                disc at each cul-de-sac turnaround
    grass       one triangulated mesh per block polygon
    markings    drawn, not placed — see `apply_ground`

Ribbon vertices are offset along the MITRE of their two segments, not along a
single segment normal, which would leave an unpaved wedge outside every joint.
"""

import math
import os
import random

from pxr import Gf, Sdf, UsdGeom, Vt

import scene_generator as sg
from layout import suburb_net as sn
from detail import suburb_parcel as sp
from detail import suburb_yardplan as yp
from detail import suburb_park as spk

# GROUND STACK, low to high. Two rules, both learned the hard way.
#
# ROADS SIT ABOVE GRASS. With asphalt below, a block polygon that overlaps a
# carriageway covers the road while the paint still draws on top, and the road
# renders as a grass strip with lane markings on it.
#
# THE GAPS ARE LARGE ON PURPOSE. Five coplanar sheets over 1600 x 1200 m: at
# 25 mm total the per-pixel depth step exceeds the gap past a few hundred
# metres and the sheets interpenetrate into flickering edges. Paint does not
# really sit 14 cm above a road, but at this capture altitude that is far below
# a pixel and the z-fighting is not. Keep the order and keep the gaps.
_Z_GRASS = 0.02
_Z_ASPHALT = 0.10
_Z_DRIVE = 0.16
_Z_CROSSWALK = 0.20
_Z_DASH = 0.24
# STOP BARS SIT LOWER THAN THE DASHES. The ladder above was derived for a
# 1600 x 1200 m plate seen from capture altitude, where 14 cm is far below a
# pixel. A stop bar is looked at from the street, and at that range 14 cm of air
# under the paint is plainly visible — the urban markings pass says as much
# about 2 cm ("at 0.021 the paint visibly floats at close range",
# `detail/road_markings.py`). Put it just over the crosswalk it sits beside.
_Z_STOPBAR = 0.205
# The front walk sits just over the drive: the two meet at the kerb, and
# coplanar ribbons z-fight precisely where a viewer is looking.
_Z_WALK = 0.17
WALK_W_M = 1.2      # a US front walk; 1.1-1.4 m is the whole range
UV_DRIVE_M = 2.5    # Driveway_Brick_Old_01: a brick course, not a road
UV_PATH_M = 2.0     # Concrete_02: slab joints at walk scale
# Park surfaces sit just above the block grass they are laid on, with their
# paint above that, on the same "surface then markings" convention as a street.
_Z_PARK_SURF = 0.06
_Z_PARK_PATH = 0.08
_Z_PARK_LINE = 0.12

# PAINT, the two colours the urban scene uses. `_make_dash_mesh` binds no
# material, so displayColor renders unlit and full-bright — a saturated colour
# here reads as a glowing streak, not as road paint.
_WHITE = (0.92, 0.92, 0.92)
_YELLOW = (0.75, 0.63, 0.18)
# Crossing and stop-bar geometry, MUTCD-ish and matching the urban defaults.
_BAR_W_M = 0.45                # ladder bar width, across the direction of travel
_BAR_GAP_M = 0.55              # gap between bars
_STOP_BAR_W_M = 0.40           # stop bar depth, along the direction of travel

# THE JUNCTION IS A ZONE, NOT A POINT. Sidewalk tiles stop before the corner,
# dashes stop before the crossing, and the crossing is drawn where they stopped.
# All three measure from the same asphalt blob, so they agree by construction.
_JUNCTION_CLEAR_M = 2.5        # asphalt blob -> near edge of the crossing
_CROSSWALK_DEPTH_M = 3.0       # nominal along-street depth of a crossing
_SIDEWALK_CORNER_M = 6.0       # extra radius where the block rings converge

def _junction_radius(net, node):
    """Radius of the asphalt blob at *node* — half the widest street on it."""
    hw = [net.edges[eid].half_w for eid in node.edges
          if net.edges[eid].road_class != "boundary"]
    return max(hw) if hw else 0.0


def _junction_stop_m(net, node):
    """Distance from *node* at which paint may start again: past the blob, past
    the clearance, past the crossing."""
    return _junction_radius(net, node) + _JUNCTION_CLEAR_M + _CROSSWALK_DEPTH_M


def _junction_zones(net, extra=0.0):
    """``[(point, radius), ...]`` for every node where streets actually meet."""
    out = []
    for n in net.nodes.values():
        if n.road_degree(net) >= 3:
            out.append((n.p, _junction_radius(net, n) + extra))
    return out


def _in_zone(zones, p):
    for (q, r) in zones:
        if (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 <= r * r:
            return True
    return False


# ---------------------------------------------------------------------------
# which approaches get paint — one definition, shared by the sign and the bar
# ---------------------------------------------------------------------------
# Deciding "does this arm stop" in two places eventually disagrees, and a stop
# bar with no sign over it is worse than neither.

def _arm_pts(net, node, edge):
    """The arm's centreline oriented AWAY from *node*, so s=0 is the junction."""
    return edge.pts if edge.a == node.id else list(reversed(edge.pts))


def _stop_bar_s(net, node):
    """Arclength along an approach at which the STOP LINE sits.

    One number, because the bar and the sign mark the same line. Computed apart,
    the sign sat a measured 5.72 m up-road of its own paint.
    """
    return _junction_stop_m(net, node) + 0.6 + _STOP_BAR_W_M * 0.5


def _stop_approaches(net):
    """``(node, edge)`` for every stop-controlled approach.

    The minor arm stops; a collector is the through road by definition.
    """
    for n in net.nodes.values():
        if n.road_degree(net) < 3:
            continue
        for eid in n.edges:
            e = net.edges[eid]
            if e.road_class in ("boundary", "collector"):
                continue
            yield n, e


def _crossing_approaches(net):
    """``(node, edge)`` for every arm that gets a painted crossing.

    NOT EVERY JUNCTION — a residential four-way in a US subdivision carries no
    paint at all. A crossing needs a collector in the junction, and then every
    arm of it gets one: a crossing on one leg only reads as an omission.
    """
    for n in net.nodes.values():
        if n.road_degree(net) < 3:
            continue
        arms = [net.edges[eid] for eid in n.edges
                if net.edges[eid].road_class != "boundary"]
        if not any(a.road_class == "collector" for a in arms):
            continue
        for e in arms:
            if e.street_type == "lollipop":
                continue
            yield n, e


def _paint_ladder(stage, path_base, c, t, half_w, ssf, z):
    """One continental ("ladder") crossing, drawn as geometry.

    Bars run PARALLEL to travel and repeat ACROSS the carriageway, and the count
    comes from the street's own width so the ladder lands kerb to kerb. Same
    construction as `road_markings._ladder`, against a polyline instead of a
    rect.
    """
    across = sn._perp(t)
    yaw = math.degrees(math.atan2(t[1], t[0]))
    pitch = _BAR_W_M + _BAR_GAP_M
    span = 2.0 * half_w
    n = max(1, int(span / pitch))
    step = span / n
    for k in range(n):
        q = sn._add(c, sn._mul(across, -half_w + (k + 0.5) * step))
        sg._make_dash_mesh(stage, f"{path_base}_{k}", q[0], q[1], z,
                           _CROSSWALK_DEPTH_M, _BAR_W_M, yaw, ssf, _WHITE)
    return n


def _paint_stop_bar(stage, path, c, t, half_w, ssf, z):
    """A white stop bar across the APPROACHING half of the carriageway.

    `t` points away from the junction, so a driver approaching travels along -t
    and their half is `_perp(t)`. Kerb to kerb would govern the departing lane
    too.
    """
    across = sn._perp(t)
    q = sn._add(c, sn._mul(across, half_w * 0.5))
    sg._make_dash_mesh(stage, path, q[0], q[1], z,
                       half_w, _STOP_BAR_W_M,
                       math.degrees(math.atan2(across[1], across[0])),
                       ssf, _WHITE)


class _RoadIndex:
    """Grid hash of street centreline segments: "is this point on the road?".

    WHY A POSITIVE TEST IS NEEDED. `build_frontage` offsets props inward from
    the block polygon and trusts that polygon to be the kerb. It usually is —
    `blocks_from_faces` insets each face by half the road width on that side —
    but not always: where `offset_polygon` hits its mitre limit it falls back to
    the un-mitred offset point, so the block boundary can bulge OVER the
    carriageway, and a prop placed relative to it lands on asphalt. Trusting the
    inset cannot detect that; measuring the distance to the centrelines can.

    A prop is checked against every segment, so this has to be cheap. Each
    segment is registered in every cell its bounding box, grown by its own half
    width plus `_PAD`, touches — which means a query only ever has to look in
    the single cell containing the point: anything closer than `half_w + margin`
    for any margin up to `_PAD` necessarily registered itself there.
    """

    _PAD = 6.0

    def __init__(self, net, cell=40.0):
        self.cell = float(cell)
        self.cells = {}
        for e in net.edges.values():
            if e.road_class == "boundary":
                continue
            hw = e.half_w
            grow = hw + self._PAD
            for i in range(len(e.pts) - 1):
                a, b = e.pts[i], e.pts[i + 1]
                seg = (a, b, hw)
                cx0 = int(math.floor((min(a[0], b[0]) - grow) / self.cell))
                cx1 = int(math.floor((max(a[0], b[0]) + grow) / self.cell))
                cy0 = int(math.floor((min(a[1], b[1]) - grow) / self.cell))
                cy1 = int(math.floor((max(a[1], b[1]) + grow) / self.cell))
                for cx in range(cx0, cx1 + 1):
                    for cy in range(cy0, cy1 + 1):
                        self.cells.setdefault((cx, cy), []).append(seg)

    def on_road(self, p, margin=0.0):
        """True when *p* is within half a carriageway (+ *margin*) of a centreline."""
        margin = min(float(margin), self._PAD)
        key = (int(math.floor(p[0] / self.cell)), int(math.floor(p[1] / self.cell)))
        for (a, b, hw) in self.cells.get(key, ()):
            if sn.seg_seg_dist(p, p, a, b) < hw + margin:
                return True
        return False


# ---------------------------------------------------------------------------
# planting: how big a tree, and where there is room for one
# ---------------------------------------------------------------------------
# Two questions every planting pass in this file asks, answered once here so
# `build_placements` and `build_open_planting` cannot drift apart on either.

class _ObbIndex:
    """Distance from a point to the nearest of a set of ORIENTED boxes.

    HOUSES AND LOTS ARE BOXES WITH A FRAME, so use the frame. `suburb_parcel`
    hands every house its along-street unit `u` and its measured `w` x `d`, and
    every lot the same about its own rectangle (`lot_corners` is
    front-left/front-right/rear-right/rear-left of a true rectangle: `u` across
    the frontage, the inward normal down the depth). That makes the EXACT
    point-to-box distance three dot products — no polygon crossing test, no
    bounding circle that a 20 m L-plan would make useless.

    DISTANCE TO THE WALL, NOT TO THE CENTRE, and that is the whole point of
    using the box. A kit house is 10-20 m across, so a centre distance calls a
    tree tucked against the back wall of a 20 m house "10 m away" and the same
    gap beside a 10 m house "5 m" — the size ramp would be reading house size
    rather than the clearance it is meant to read.

    Each box registers in every cell it touches once grown by `reach`, so ONE
    cell lookup is the whole search: anything within `reach` of a query point
    necessarily registered itself in that point's cell. Same trick, same
    reason, as `_RoadIndex` above. Answers are capped at `reach` because every
    consumer's ramp has already saturated by then.
    """

    def __init__(self, boxes, reach=60.0):
        self.reach = float(reach)
        self.cell = max(20.0, self.reach)
        self.cells = {}
        for box in boxes:
            cx, cy, _ux, _uy, hx, hy = box
            grow = math.hypot(hx, hy) + self.reach
            gx0 = int(math.floor((cx - grow) / self.cell))
            gx1 = int(math.floor((cx + grow) / self.cell))
            gy0 = int(math.floor((cy - grow) / self.cell))
            gy1 = int(math.floor((cy + grow) / self.cell))
            for gx in range(gx0, gx1 + 1):
                for gy in range(gy0, gy1 + 1):
                    self.cells.setdefault((gx, gy), []).append(box)

    def nearest(self, p):
        """Metres from *p* to the nearest box wall: 0.0 inside, `reach` if far."""
        key = (int(math.floor(p[0] / self.cell)),
               int(math.floor(p[1] / self.cell)))
        best = self.reach
        for (cx, cy, ux, uy, hx, hy) in self.cells.get(key, ()):
            vx, vy = p[0] - cx, p[1] - cy
            dx = abs(vx * ux + vy * uy) - hx
            dy = abs(-vx * uy + vy * ux) - hy
            d = math.hypot(max(dx, 0.0), max(dy, 0.0))
            if d < best:
                best = d
                if best <= 0.0:
                    break
        return best


def _house_box(h):
    return (h["c"][0], h["c"][1], h["u"][0], h["u"][1],
            float(h["w"]) * 0.5, float(h["d"]) * 0.5)


def _rect_box(corners):
    """A four-corner rectangle (in ring order) as an `_ObbIndex` box, or None.

    Serves lots and pool holes alike. Rebuilt from the CORNERS rather than from
    a centre and a nominal size, because neither is trustworthy: a lot's front
    half-width is chord-clamped on a curving frontage, and a pool hole is the
    water rectangle already inset by its coping. The corners are what was
    actually granted; the nominal numbers are not.
    """
    if not corners or len(corners) < 4:
        return None
    p0, p1, _p2, p3 = corners[0], corners[1], corners[2], corners[3]
    ax, ay = p1[0] - p0[0], p1[1] - p0[1]
    bx, by = p3[0] - p0[0], p3[1] - p0[1]
    la = math.hypot(ax, ay)
    lb = math.hypot(bx, by)
    if la < 1e-6 or lb < 1e-6:
        return None
    return (p0[0] + ax * 0.5 + bx * 0.5, p0[1] + ay * 0.5 + by * 0.5,
            ax / la, ay / la, la * 0.5, lb * 0.5)


def _in_rect(p, rect, pad=0.0):
    """Is *p* inside the AXIS-ALIGNED ``(x0, y0, x1, y1)`` grown by *pad*?"""
    return (rect[0] - pad <= p[0] <= rect[2] + pad
            and rect[1] - pad <= p[1] <= rect[3] + pad)



def _sample_polyline(pts, step_m, half_w):
    """Points along a polyline, each carrying the ribbon's half-width."""
    out = []
    for i in range(len(pts) - 1):
        ax, ay = float(pts[i][0]), float(pts[i][1])
        bx, by = float(pts[i + 1][0]), float(pts[i + 1][1])
        L = math.hypot(bx - ax, by - ay)
        n = max(1, int(L / max(0.25, float(step_m))))
        for k in range(n + 1):
            t = k / float(n)
            out.append((ax + (bx - ax) * t, ay + (by - ay) * t, float(half_w)))
    return out


def paving_keepout(parcels, step_m=1.0):
    """Every driveway and front walk, as sampled points with a half-width.

    A TREE ON A DRIVEWAY IS NOT A LOT-KEEPOUT FAILURE. The lot rectangle stops
    open planting from landing inside somebody's garden, and it works — but a
    DRIVE crosses the verge to reach the kerb, and the frontage trees are
    placed out there on purpose, in exactly the strip the drive has to cross.
    Neither pass knew where the paving was, because drives and walks are drawn
    later, in `apply_ground`, from the same parcel data.

    So this derives them from that same data, before anything is planted.
    Ribbons rather than rectangles: a drive to a garage bends, and its
    bounding box would take out half the frontage with it.
    """
    out = []
    for p in (parcels or ()):
        p_houses = p.get("houses") or []
        for di, d in enumerate(p.get("drives") or ()):
            plan = p_houses[di].get("plan") if di < len(p_houses) else None
            run = (plan["drive"] if (plan and plan.get("drive"))
                   else (d["a"], d["b"]))
            out += _sample_polyline(list(run), step_m,
                                    float(d.get("w", 3.0)) * 0.5)
            if plan and plan.get("path"):
                out += _sample_polyline(list(plan["path"]), step_m,
                                        WALK_W_M * 0.5)
    return out


class _PavingIndex:
    """"Is this point on a drive or a walk?", on a grid."""

    def __init__(self, samples, clear_m=1.2):
        self.clear = float(clear_m)
        self.r_max = max([s[2] for s in samples], default=0.0) + self.clear
        self.cell = max(1.0, self.r_max)
        self.cells = {}
        for x, y, hw in samples:
            key = (int(math.floor(x / self.cell)), int(math.floor(y / self.cell)))
            self.cells.setdefault(key, []).append((x, y, hw))

    def on_paving(self, q):
        if not self.cells:
            return False
        cx = int(math.floor(q[0] / self.cell))
        cy = int(math.floor(q[1] / self.cell))
        for i in (cx - 1, cx, cx + 1):
            for j in (cy - 1, cy, cy + 1):
                for x, y, hw in self.cells.get((i, j), ()):
                    if math.hypot(q[0] - x, q[1] - y) <= hw + self.clear:
                        return True
        return False


class _Occupancy:
    """What is already standing on the ground, as points on a grid.

    Seeded from the placements made so far, so a planting pass keeps clear of
    the yard trees, streetlights, hydrants and park furniture that are already
    there rather than only of its own darts. Cell is the query radius, so a
    3 x 3 neighbourhood is the whole search; asking for a radius bigger than
    the cell would silently miss, so it clamps and says so by construction.
    """

    def __init__(self, cell=8.0):
        self.cell = max(1.0, float(cell))
        self.cells = {}

    def add(self, p):
        key = (int(math.floor(p[0] / self.cell)),
               int(math.floor(p[1] / self.cell)))
        self.cells.setdefault(key, []).append((p[0], p[1]))

    def near(self, p, r):
        r = min(float(r), self.cell)
        gx = int(math.floor(p[0] / self.cell))
        gy = int(math.floor(p[1] / self.cell))
        rr = r * r
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for (qx, qy) in self.cells.get((gx + dx, gy + dy), ()):
                    if (p[0] - qx) ** 2 + (p[1] - qy) ** 2 < rr:
                        return True
        return False


class _CanopyPool:
    """A tree pool RANKED BY MEASURED CANOPY, so a draw can ask for a size.

    WHY RANK A POOL RATHER THAN ADD A THIRD ONE. The asset set already splits
    trees by ROLE — `street_trees` for the verge, `trees` for open ground — and
    that split is right: it is what keeps a 25.4 m Black_Oak crown off a 10.7 m
    carriageway. What it cannot express is that `trees` itself spans 4.1 m
    (American_Beech) to 25.4 m (Black_Oak) and that WHERE in a block a tree
    stands should decide which end of that range it comes from. Ranking is the
    missing half: same pools, same roles, but a draw now carries a size.

    MEASURED, NEVER NAMED. `resolver.get` opens the asset and returns its Z-up
    footprint; the key is `max(sx, sy)` — the crown WIDTH, which is what has to
    clear a wall, not the height. On the shipped suburban set that ranks
    `trees` 4.09 / 5.09 / 5.86 / 10.29 / 25.42 m and `street_trees`
    3.02 / 10.29 m, which is exactly what the asset set records beside each
    entry from tools/measure_assets.py.

    DEGENERATE WHEN NOTHING WAS MEASURED, and it says so rather than pretending.
    Under `measure_usds: false`, or where the assets are unreachable, every
    entry falls back to the single `fallback_sizes.tree` and the ranking sorts a
    pool of identical numbers — i.e. by path. `graded` is that test, and a pool
    that fails it degrades to the uniform draw these passes did before.
    """

    def __init__(self, resolver, pools, paths, category="tree", band=0.5):
        self.band = min(1.0, max(0.05, float(band)))
        ranked = []
        for u in paths:
            fp = resolver.get(u, category, scale=pools.scale_of(u),
                              axis_up=pools.axis_of(u))
            ranked.append((max(float(fp.get("sx", 0.0) or 0.0),
                               float(fp.get("sy", 0.0) or 0.0)), u))
        ranked.sort(key=lambda r: (r[0], r[1]))
        self.sizes = [r[0] for r in ranked]
        self.usds = [r[1] for r in ranked]

    def __len__(self):
        return len(self.usds)

    @property
    def graded(self):
        """True when the measurements actually separate the pool.

        Half a metre of spread over the whole pool is nothing next to the 21 m
        this library really spans, so below it the numbers are a fallback
        constant and not a measurement.
        """
        return len(self.usds) > 1 and (self.sizes[-1] - self.sizes[0]) > 0.5

    def draw(self, rng, t):
        """One asset from the `band` of the pool nearest size *t* (0 small, 1 big).

        A SLIDING WINDOW RATHER THAN A HARD CUT, deliberately. A suburb where
        every tree within a dozen metres of a wall is the same species reads as
        one nursery order; the overlap between the near window and the far one
        is what keeps it a single neighbourhood with a size gradient across it
        rather than two plantations meeting at a contour line.
        """
        n = len(self.usds)
        if n == 0:
            return None
        if n == 1 or not self.graded:
            return self.usds[rng.randrange(n)]
        k = max(1, int(self.band * n + 0.5))
        lo = int(round(min(1.0, max(0.0, float(t))) * (n - k)))
        return self.usds[lo + rng.randrange(k)]


def _size_t(dist_m, near_m, far_m):
    """Where a tree at *dist_m* from the nearest house sits on the size ramp.

    0 at or inside `near_m` — small end of the pool; 1 at or beyond `far_m` —
    big end; linear between. Two knobs rather than one threshold because a step
    would draw a visible ring of species change round every house, and the
    thing being modelled is a gradient: what a householder plants next to a
    wall, what stands out in the middle of the block, and everything between.
    """
    if far_m <= near_m:
        return 1.0 if dist_m >= far_m else 0.0
    return min(1.0, max(0.0, (float(dist_m) - near_m) / (far_m - near_m)))


def _planting_cfg(config):
    """The planting knobs, read once so every pass agrees on them.

    They live under `suburb_parcel` beside `open_trees_per_100m2`, which is
    where the two that already existed were.
    """
    cfg = config.get("suburb_parcel") or {}
    return {
        # Distance from the nearest HOUSE WALL at which a tree may still come
        # from the big end of the pool. 12 m is a back-garden depth: `lot_depth`
        # runs 26-38 m and the house eats setback + depth, so a rear-garden tree
        # is typically 5-12 m off the wall and gets the small end, which is what
        # a householder actually plants next to a house.
        "near_m": float(cfg.get("tree_small_near_m", 12.0)),
        # ...and the distance beyond which it is open ground and takes the big
        # end unreserved. 40 m clears the deepest lot (38 m) plus its fence, so
        # anything scoring 1.0 is genuinely land nobody's garden reaches.
        "far_m": float(cfg.get("tree_big_far_m", 40.0)),
        # Share of the ranked pool one draw may reach. 0.5 over the 5-entry
        # `trees` pool is a 3-species window: enough overlap that near and far
        # planting share species, narrow enough that the 25.4 m Black_Oak is
        # unreachable at t=0 and the 4.1 m Beech unreachable at t=1.
        "band": float(cfg.get("tree_size_band", 0.5)),
        # Trees per 100 m2 on land nobody built on — undeveloped parcels and
        # the park surround. Unchanged default: a ~10.8 m planting grid.
        "open_rate": float(cfg.get("open_trees_per_100m2", 0.85) or 0.0),
        # ...and on the leftover inside a DEVELOPED block, which is thinner
        # ground: it is the back of somebody's block and `suburb_yardplan` has
        # already planted the gardens in front of it, so the same rate as
        # unplatted woodland would be wrong. Swept on the shipped preset at
        # seed 3, open-land trees over the whole suburb: 0.85 -> 6,273,
        # 0.45 -> 4,726, 0.25 -> 3,751. 0.45 is a ~14.9 m planting grid — a
        # treed common behind the houses, not a forest.
        "fill_rate": float(cfg.get("infill_trees_per_100m2", 0.45) or 0.0),
        # Trunk to trunk, between trees this pass plants.
        "gap_m": float(cfg.get("open_tree_gap_m", 6.0) or 6.0),
        # ...and from anything already standing: a fence line, a lot boundary,
        # a streetlight, a park bench. 3.0 m is a crown radius at the small end
        # of the pool.
        "clear_m": float(cfg.get("open_tree_clear_m", 3.0) or 0.0),
        # Darts per planting station before the station is given up. The grid
        # below is stratified rather than uniform, so a station that lands on a
        # lot usually has free ground within its own cell — one dart threw that
        # away and left holes exactly where the ground was tightest.
        "darts": max(1, int(cfg.get("open_tree_darts", 4))),
    }


# ---------------------------------------------------------------------------
# geometry -> mesh
# ---------------------------------------------------------------------------

def _mitre_offsets(pts, half_w):
    """Left/right edge points for a ribbon of half-width *half_w* along *pts*.

    The offset at an interior vertex is along the MITRE of its two segments,
    scaled by 1/cos(theta/2), so the two segment edges actually meet there. A
    plain per-segment normal leaves an unpaved wedge on the outside of every
    bend — at 6 m sampling and a 10.7 m road that is a visible notch at each
    joint all the way round a curve. The scale is capped so a hairpin cannot
    throw the edge off to infinity.
    """
    n = len(pts)
    left, right = [], []
    for i in range(n):
        if i == 0:
            d = sn._unit(sn._sub(pts[1], pts[0]))
            m, scale = sn._perp(d), 1.0
        elif i == n - 1:
            d = sn._unit(sn._sub(pts[-1], pts[-2]))
            m, scale = sn._perp(d), 1.0
        else:
            d0 = sn._unit(sn._sub(pts[i], pts[i - 1]))
            d1 = sn._unit(sn._sub(pts[i + 1], pts[i]))
            m = sn._unit(sn._add(sn._perp(d0), sn._perp(d1)))
            cos_half = max(0.35, abs(sn._dot(m, sn._perp(d0))))
            scale = 1.0 / cos_half
        off = sn._mul(m, half_w * scale)
        left.append(sn._add(pts[i], off))
        right.append(sn._sub(pts[i], off))
    return left, right


def _make_ribbon(stage, path, pts, half_w, z, ssf, uv_scale, color, mat="",
                 uv_along_m=0.0, v_span=0.0):
    """A quad strip swept along *pts*. One mesh, not one quad per segment.

    TWO UV CONVENTIONS, because the two road materials disagree about which
    axis is which:

    * default (`uv_along_m == 0`) — the original: `u` ACROSS the ribbon and `v`
      ALONG it, both off the single `uv_scale`. Right for a tileable swatch
      like `MI_Asphalt`.
    * trim-sheet (`uv_along_m > 0`) — `u` ALONG the ribbon at one repeat per
      `uv_along_m`, `v` ACROSS it as a FRACTION of the half width, scaled to
      `v_span`. `Road_01_Inst` is a cross-section sheet (asphalt 0-0.53, kerb
      0.53-0.63, verge 0.63-0.83, sidewalk 0.83-1.0), so its `v` is distance
      from the crown, not distance travelled. Mapped the default way it sweeps
      the kerb and sidewalk bands DOWN the street, striping grass across the
      carriageway every few metres. `v_span` under 0.53 keeps every vertex
      inside the asphalt band whatever the road's width.
    """
    if len(pts) < 2 or half_w <= 0.0:
        return None
    left, right = _mitre_offsets(pts, half_w)
    s = ssf
    verts, counts, idx = [], [], []
    for i in range(len(pts)):
        verts.append(Gf.Vec3f(left[i][0] * s, left[i][1] * s, z * s))
        verts.append(Gf.Vec3f(right[i][0] * s, right[i][1] * s, z * s))
    for i in range(len(pts) - 1):
        a = 2 * i
        counts.append(4)
        # WINDING MATTERS. Vertices alternate left, right, left, right along the
        # strip, so the obvious [a, a+2, a+3, a+1] traverses up the left side and
        # back down the right — which is CLOCKWISE seen from +Z, i.e. the face
        # points at the ground and the renderer culls it. Explicit +Z normals do
        # not save it: culling reads the winding, not the normal attribute. The
        # roads were invisible for exactly this reason.
        idx.extend([a, a + 1, a + 3, a + 2])
    cum = sn._cumulative(pts)
    uvs = []
    if uv_along_m > 0.0:                       # trim sheet: u along, v across
        for i in range(len(pts)):
            u = cum[i] / uv_along_m
            uvs.append(Gf.Vec2f(u, v_span))    # left edge
            uvs.append(Gf.Vec2f(u, 0.0))       # crown side
    else:                                      # tileable swatch: u across
        for i in range(len(pts)):
            v = cum[i] / max(uv_scale, 1e-6)
            uvs.append(Gf.Vec2f(0.0, v))
            uvs.append(Gf.Vec2f(2.0 * half_w / max(uv_scale, 1e-6), v))
    return _define_mesh(stage, path, verts, counts, idx, uvs, color, mat)


def _clip_halfplane(poly, a, b):
    """Sutherland-Hodgman: the part of *poly* left of the directed line a->b."""
    if not poly:
        return []
    def side(p):
        return ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]))
    out = []
    n_pts = len(poly)
    for i in range(n_pts):
        cur, nxt = poly[i], poly[(i + 1) % n_pts]
        sc, sx = side(cur), side(nxt)
        if sc >= 0.0:
            out.append(cur)
        # The intersection has to be emitted on EITHER crossing direction.
        # Testing only one of them drops half the cut edges and silently
        # shrinks the result — it measured 2875 m2 against an expected 5950.
        if (sc >= 0.0) != (sx >= 0.0):
            t = sc / (sc - sx) if abs(sc - sx) > 1e-12 else 0.0
            out.append((cur[0] + t * (nxt[0] - cur[0]),
                        cur[1] + t * (nxt[1] - cur[1])))
    return out


def polygon_minus_convex(poly, hole):
    """*poly* with a CONVEX *hole* removed, as a list of polygons.

    Cutting a pool into the lawn needs a real subtraction: the water sits below
    grade, so without a hole the grass simply draws over it and the pool is
    invisible from above. Raising the water instead would make it a puddle
    sitting ON the lawn.

    The partition is the standard one for a convex hole — for each hole edge i,
    take the part of the polygon OUTSIDE edge i but INSIDE edges 0..i-1. Those
    pieces tile the difference exactly, with no overlap and no T-junctions,
    which a naive "clip to each edge and union" does not.
    """
    hole = list(hole)
    if len(hole) < 3:
        return [poly]
    # orient the hole counter-clockwise so "inside" is consistently left
    area = sum(hole[i][0] * hole[(i + 1) % len(hole)][1]
               - hole[(i + 1) % len(hole)][0] * hole[i][1]
               for i in range(len(hole)))
    if area < 0:
        hole = hole[::-1]
    pieces, n = [], len(hole)
    for i in range(n):
        a, b = hole[i], hole[(i + 1) % n]
        piece = _clip_halfplane(poly, b, a)          # OUTSIDE edge i
        for j in range(i):
            c, d = hole[j], hole[(j + 1) % n]
            piece = _clip_halfplane(piece, c, d)     # inside the earlier ones
            if not piece:
                break
        if len(piece) >= 3:
            pieces.append(piece)
    return pieces or [poly]


def _make_polygon(stage, path, poly, z, ssf, uv_scale, color, mat=""):
    """A block polygon as one fan-triangulated mesh.

    Fan from the centroid rather than from vertex 0: block polygons are convex
    or mildly concave, and a centroid fan degrades gracefully on the concave
    ones where a corner fan folds over itself.
    """
    if len(poly) < 3:
        return None
    s = ssf
    cx, cy = sn.polygon_centroid(poly)
    verts = [Gf.Vec3f(cx * s, cy * s, z * s)]
    uvs = [Gf.Vec2f(cx / uv_scale, cy / uv_scale)]
    for (x, y) in poly:
        verts.append(Gf.Vec3f(x * s, y * s, z * s))
        uvs.append(Gf.Vec2f(x / uv_scale, y / uv_scale))
    counts, idx = [], []
    n = len(poly)
    for i in range(n):
        counts.append(3)
        idx.extend([0, 1 + i, 1 + (i + 1) % n])
    return _define_mesh(stage, path, verts, counts, idx, uvs, color, mat)


def _make_disc(stage, path, c, r, z, ssf, uv_scale, color, mat="", seg=24):
    poly = [(c[0] + r * math.cos(2 * math.pi * k / seg),
             c[1] + r * math.sin(2 * math.pi * k / seg)) for k in range(seg)]
    return _make_polygon(stage, path, poly, z, ssf, uv_scale, color, mat)


def _define_mesh(stage, path, verts, counts, idx, uvs, color, mat):
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray(verts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * len(verts)))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateSubdivisionSchemeAttr("none")
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    pv.Set(Vt.Vec2fArray(uvs))
    if color:
        mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    if mat:
        try:
            from pxr import UsdShade
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
                UsdShade.Material(stage.GetPrimAtPath(Sdf.Path(mat))))
        except Exception:
            pass
    return mesh


# ---------------------------------------------------------------------------
# ground
# ---------------------------------------------------------------------------

def apply_park_ground(stage, config, park, gnd, ssf, mats):
    """The park's SURFACES: court slabs, pitch, sand, paths and their markings.

    Without this the park had its props and nothing under them -- hoops standing
    on grass, a tennis block with no court, no paths at all. The props were
    placed because they are assets; the surfaces were not, because they are
    GEOMETRY, and nothing was generating it. suburb_park already produces the
    polygons and the regulation line work (`basketball_markings`,
    `tennis_markings`, `soccer_markings`); this writes them.

    Z ordering matches the street convention -- surface, then paint on top of
    it -- and sits above the block grass so a court reads as laid on the park
    rather than buried in it.
    """
    asphalt_mat, grass_mat, park_grass_mat, dirt_mat = mats
    # PARK TURF IS NOT LAWN TURF. Municipal park grass is mown coarser and
    # watered less than a front garden, so the park reads as its own ground
    # rather than as an extension of the blocks around it — which is the whole
    # point of giving it a separate key. Falls back to the street grass when the
    # asset set declares no `grass_park`, so older sets are unaffected.
    turf = park_grass_mat or grass_mat
    n = {"slab": 0, "mark": 0, "path": 0}

    def xf(pts, cx, cy, yaw):
        a = math.radians(yaw)
        ux, uy = math.cos(a), math.sin(a)
        return [(cx + ux * x - uy * y, cy + uy * x + ux * y) for (x, y) in pts]

    SURF = {                       # kind -> (colour, material, z)
        "soccer":              ((0.26, 0.44, 0.18), turf,        _Z_PARK_SURF),
        "basketball_compound": ((0.24, 0.28, 0.33), asphalt_mat, _Z_PARK_SURF),
        "tennis_block":        ((0.18, 0.42, 0.32), asphalt_mat, _Z_PARK_SURF),
        # Sand, not grass and not asphalt: `dirt` is the closest thing the AEC
        # pack ships, and a bare-earth playground floor is what it looks like.
        "playground":          ((0.85, 0.76, 0.60), dirt_mat,    _Z_PARK_SURF),
        "picnic":              ((0.42, 0.50, 0.31), turf,        _Z_PARK_SURF),
    }
    for i, z in enumerate(park["zones"]):
        spec = SURF.get(z["kind"])
        if not spec:
            continue
        col, mat, zz = spec
        if _make_polygon(stage, f"{gnd}/park_{z['kind']}_{i}", z["corners"],
                         zz, ssf, 4.0, col, mat) is not None:
            n["slab"] += 1

    # Line work, at the regulation dimensions suburb_park already solves for.
    for i, z in enumerate(park["zones"]):
        lines = []
        if z["kind"] == "soccer":
            lines = [(spk.soccer_markings(), z["centre"], z["yaw"])]
        elif z["kind"] == "basketball_compound":
            lines = [(spk.basketball_markings(), c["centre"], c["yaw"])
                     for c in z.get("courts", [])]
        # NO TENNIS LINE WORK. The tennis court is a WHOLE ASSET and its paint
        # comes with it — `park.yaml` says so where it declares the pool ("its
        # markings come with it, so suburb_park places the enclosure and fence
        # and leaves the paint alone"), and drawing a second set on top of the
        # asset's own only stipples the two together. Basketball is the other
        # way round: only the hoop is an asset, so the court is drawn.
        # The court index is in the prim path, not just the line index. Without
        # it, court 2's markings overwrite court 1's at the same path and three
        # of the four basketball courts come out unpainted -- 131 meshes were
        # authored and 78 survived.
        for k, (group, (cx, cy), yaw) in enumerate(lines):
            for j, ln in enumerate(group):
                w = xf(ln, cx, cy, yaw)
                if _make_ribbon(stage, f"{gnd}/park_line_{i}_{k}_{j}", w, 0.06,
                                _Z_PARK_LINE, ssf, 4.0,
                                (0.93, 0.93, 0.90)) is not None:
                    n["mark"] += 1

    for i, pa in enumerate(park["paths"]):
        w = float(pa.get("width_m", 2.6)) / 2.0
        if _make_ribbon(stage, f"{gnd}/park_path_{i}", pa["pts"], w,
                        _Z_PARK_PATH, ssf, 4.0, (0.72, 0.66, 0.55)) is not None:
            n["path"] += 1

    print(f"[suburb_scene] park ground: {n['slab']} surfaces, {n['mark']} "
          f"markings, {n['path']} paths")


def apply_ground(stage, config, net, blocks, parcels, region, parent_path, ssf,
                 pool_rects=None,
                 park=None):
    """Asphalt ribbons, block grass, driveways and centreline dashes."""
    roads_cfg = config.get("roads", {}) or {}

    # THE HEIGHT LADDER SCALES WITH THE PLATE, AUTOMATICALLY.
    #
    # The constants above are metres, and they were chosen for a 1600 x 1200 m
    # plate seen from capture altitude — where 14 cm of air under the paint is
    # far below one pixel, and where the depth buffer at that range needs a gap
    # that large to not z-fight. Neither of those is a property of the ROAD;
    # both follow from how far away the camera is, which in practice tracks how
    # big the plate is. Hard-coding metres therefore bakes one plate size into
    # the geometry, and every other size is wrong in one direction or the
    # other: floating paint on a small block, z-fighting on a huge one.
    #
    # So the ladder is derived from the region rather than declared. 1600 m
    # reproduces the tuned values exactly, so existing scenes are unchanged;
    # a 250 m block gets ~1.6 cm instead of 14 cm. The floor stops it
    # collapsing to zero on a tiny plate, where depth precision still needs
    # something to work with. `roads.z_scale` remains as an explicit override.
    _span = max(float(region[0]), float(region[1])) if region else 1600.0
    _zs = float(roads_cfg.get("z_scale",
                              max(0.08, min(1.0, _span / 1600.0))))
    z_grass = _Z_GRASS * _zs
    z_asphalt = _Z_ASPHALT * _zs
    z_drive = _Z_DRIVE * _zs
    z_crosswalk = _Z_CROSSWALK * _zs
    z_dash = _Z_DASH * _zs
    z_stopbar = _Z_STOPBAR * _zs
    z_walk = _Z_WALK * _zs
    z_park_surf = _Z_PARK_SURF * _zs
    z_park_path = _Z_PARK_PATH * _zs
    uv_asphalt = float(roads_cfg.get("asphalt_uv_scale_m", 4.0))
    uv_grass = float(roads_cfg.get("grass_uv_scale_m", 3.0))
    ll = roads_cfg.get("lane_lines", {}) or {}
    dash_len = float(ll.get("dash_length_m", 3.0))
    dash_gap = float(ll.get("dash_gap_m", 3.0))
    dash_w = float(ll.get("dash_width_m", 0.15))

    gnd = parent_path + "/ground"
    UsdGeom.Scope.Define(stage, Sdf.Path(gnd))

    mat_scope = gnd + "/materials"
    UsdGeom.Scope.Define(stage, Sdf.Path(mat_scope))
    mat_cfg = (config.get("usds", {}) or {}).get("materials", {}) or {}
    asset_root = str(config.get("asset_root", "") or "").rstrip("/")

    def _load_mat(key):
        url = mat_cfg.get(key, "")
        if not url:
            return ""
        url = sg._join_asset_root(url, asset_root)
        p = mat_scope + "/" + key
        prim = stage.DefinePrim(Sdf.Path(p))
        prim.GetReferences().AddReference(url)
        prim.Load()
        return p

    asphalt_mat, grass_mat = _load_mat("asphalt"), _load_mat("grass")
    # A SECOND road surface, mixed with the first. `asphalt_road_tile` is the
    # ModularNeighborhood road material — a cross-section trim sheet, so it
    # needs the trim-sheet UVs above; the original is a plain tileable swatch.
    # Mixing them per street is what stops a whole suburb reading as one
    # resurfacing job. Falls back to the original if the set doesn't name it.
    asphalt_alt = _load_mat("asphalt_road_tile")
    ROAD_TRIM_REPEAT_M = 8.0    # the tile is 16 m long over u = 0..2
    ROAD_TRIM_V = 0.50          # asphalt band ends at 0.53; stay inside it
    alt_share = float(roads_cfg.get("road_tile_share", 0.5)) if asphalt_alt else 0.0
    # THREE GROUNDS, NOT ONE. A single grass made undeveloped land render as
    # mown lawn that nobody had built on, which is the opposite of what the
    # sparseness is for: unplatted ground has to LOOK unplatted or leaving it
    # empty reads as a missing row of houses. `grass_rough` is the AEC pack's
    # Grass_Countryside, `grass` its Grass_Cut. Both fall back to `grass`, so an
    # asset set that declares neither behaves exactly as before.
    rough_mat = _load_mat("grass_rough") or grass_mat
    park_grass_mat = _load_mat("grass_park") or grass_mat
    dirt_mat = _load_mat("dirt")

    # 1) Grass first, as one sheet over the whole region: blocks do not tile it
    #    exactly, so painting only the blocks leaves gaps where one was
    #    rejected as too small.
    rx0, ry0, rx1, ry1 = region
    # The base sheet is everything the blocks do not cover — verges, leftovers,
    # the ground under the whole plat. That is rough ground, not lawn.
    # THE BASE PLANE HAS TO LOSE THE POOLS TOO. The block meshes above already
    # subtract `pool_rects`, but this backdrop did not — so a pool hole cut
    # cleanly through the grass only revealed the opaque plane underneath it,
    # and the pool never became visible. Cut it as a polygon for the same
    # reason, or skip straight to a plane when there are no pools.
    _base_poly = [(rx0, ry0), (rx1, ry0), (rx1, ry1), (rx0, ry1)]
    _base_parts = [_base_poly]
    for rect in (pool_rects or ()):
        nxt = []
        for q in _base_parts:
            nxt += polygon_minus_convex(q, rect)
        _base_parts = nxt
    if len(_base_parts) == 1 and _base_parts[0] is _base_poly:
        sg._make_plane_mesh(stage, gnd + "/ground_base", rx0, ry0, rx1, ry1,
                            -0.005, uv_grass, ssf,
                            display_color=(0.24, 0.36, 0.17),
                            mat_prim_path=rough_mat)
    else:
        for k, q in enumerate(_base_parts):
            _make_polygon(stage, "{0}/ground_base_{1}".format(gnd, k), q,
                          -0.005, ssf, uv_grass, (0.24, 0.36, 0.17), rough_mat)

    n_road = n_road_alt = 0
    # Per STREET, not per segment: a road that changes surface halfway along
    # reads as a patch job. `e.id` keeps the choice stable across rebuilds.
    for e in net.edges.values():
        if e.road_class == "boundary":
            continue
        use_alt = alt_share > 0.0 and (hash(("road_surf", e.id)) % 1000) / 1000.0 < alt_share
        if use_alt:
            ok = _make_ribbon(stage, f"{gnd}/road_{e.id}", e.pts, e.half_w,
                              z_asphalt, ssf, uv_asphalt, (0.15, 0.15, 0.15),
                              asphalt_alt, uv_along_m=ROAD_TRIM_REPEAT_M,
                              v_span=ROAD_TRIM_V)
        else:
            ok = _make_ribbon(stage, f"{gnd}/road_{e.id}", e.pts, e.half_w,
                              z_asphalt, ssf, uv_asphalt, (0.15, 0.15, 0.15),
                              asphalt_mat)
        if ok is not None:
            n_road += 1
            n_road_alt += 1 if use_alt else 0

    bulb_r = float(sn.DEFAULTS["bulb_radius_m"])
    n_bulb = 0
    for e in net.edges.values():
        if e.street_type != "lollipop":
            continue
        # ALWAYS the tileable swatch. `_make_disc` maps UVs planar off world
        # x/y, so a cross-section trim sheet would sweep verge and sidewalk
        # across the turnaround. Mixing surfaces is fine; mixing UV conventions
        # on a shape that cannot express the second one is not.
        _make_disc(stage, f"{gnd}/bulb_{e.id}", e.pts[-1], bulb_r,
                   z_asphalt, ssf, uv_asphalt, (0.15, 0.15, 0.15), asphalt_mat)
        n_bulb += 1

    n_grass = n_rough = 0
    n_cut = 0
    for i, b in enumerate(blocks):
        wild = bool(b.get("undeveloped"))
        col = (0.26, 0.34, 0.16) if wild else (0.2, 0.5, 0.1)
        # Subtract any pool whose rectangle lands on this block, so the lawn has
        # a hole to see the water through rather than a sheet over the top.
        parts = [b["poly"]]
        for rect in (pool_rects or ()):
            nxt = []
            for q in parts:
                cut = polygon_minus_convex(q, rect)
                if len(cut) > 1 or (len(cut) == 1 and cut[0] is not q):
                    n_cut += 1 if len(cut) > 1 else 0
                nxt += cut
            parts = nxt
        ok = False
        for k, q in enumerate(parts):
            suffix = "" if len(parts) == 1 else f"_{k}"
            if _make_polygon(stage, f"{gnd}/grass_{i}{suffix}", q, z_grass,
                             ssf, uv_grass, col,
                             rough_mat if wild else grass_mat) is not None:
                ok = True
        if ok:
            n_grass += 1
            n_rough += 1 if wild else 0

    # DRIVES AND WALKS, aimed at the actual openings.
    #
    # Both were drawn with NO material — a flat grey ribbon — and the drive was
    # aimed by the plat's guess of `house_w * 0.30` to one side, made before
    # anything knew which house would be built there. Where `build_placements`
    # left a `plan` on the house, that guess is replaced by the kit's own
    # answer: brick to the garage door, concrete to the front step. Houses with
    # no garage keep the platted run, which is a parking pad beside the house
    # and is where a drive on such a lot really goes.
    drive_mat, path_mat = _load_mat("driveway"), _load_mat("path")
    n_drive = n_walk = 0
    for pi, p in enumerate(parcels):
        p_houses = p.get("houses") or []
        for di, d in enumerate(p["drives"]):
            plan = p_houses[di].get("plan") if di < len(p_houses) else None
            run = plan["drive"] if (plan and plan.get("drive")) else (d["a"], d["b"])
            if _make_ribbon(stage, f"{gnd}/drive_{pi}_{di}", list(run),
                            d["w"] / 2.0, z_drive, ssf, UV_DRIVE_M,
                            (0.45, 0.45, 0.43), drive_mat) is not None:
                n_drive += 1
            # The walk is the half-metre-higher of the two: they share a kerb
            # end, and coplanar ribbons z-fight exactly where both are visible.
            if plan and plan.get("path") and _make_ribbon(
                    stage, f"{gnd}/walk_{pi}_{di}", list(plan["path"]),
                    WALK_W_M / 2.0, z_walk, ssf, UV_PATH_M,
                    (0.62, 0.61, 0.58), path_mat) is not None:
                n_walk += 1

    # 2) MARKINGS, the vocabulary the urban scene paints.
    #
    #    A LOCAL STREET CARRIES NO CENTRELINE — MUTCD 3B.01 warrants one by
    #    width and volume and a residential street meets neither, so a post-war
    #    plat has bare asphalt between its junctions. Collectors keep theirs,
    #    which is most of what makes a collector legible as the through road.
    #
    #    Paint stops at `_junction_stop_m`, the same figure the crossing and the
    #    stop bar are placed from, so the three agree by construction.
    period = dash_len + dash_gap
    n_dash, n_dash_cut = 0, 0
    for e in net.edges.values():
        if e.road_class != "collector":
            continue
        L = e.length
        stop_a = (_junction_stop_m(net, net.nodes[e.a])
                  if net.nodes[e.a].road_degree(net) >= 3 else 0.0)
        stop_b = (_junction_stop_m(net, net.nodes[e.b])
                  if net.nodes[e.b].road_degree(net) >= 3 else 0.0)
        k = int(L / period)
        for j in range(k):
            s = (j + 0.5) * period
            if s < stop_a or s > L - stop_b:
                n_dash_cut += 1
                continue
            c = sn.point_at(e.pts, s)
            t = sn.tangent_at(e.pts, s)
            yaw = math.degrees(math.atan2(t[1], t[0]))
            sg._make_dash_mesh(stage, f"{gnd}/dash_{e.id}_{j}",
                               c[0], c[1], z_dash, dash_len, dash_w, yaw, ssf,
                               display_color=_YELLOW)
            n_dash += 1

    # 3) Crossings, drawn rather than placed: the band is masked in asphalt
    #    first so lane paint does not run through the bars, which a placed tile
    #    could not do.
    n_x, n_xbar = 0, 0
    for node, e in _crossing_approaches(net):
        pts = _arm_pts(net, node, e)
        s = _junction_stop_m(net, node) - _CROSSWALK_DEPTH_M * 0.5
        if sn.polyline_length(pts) < s + _CROSSWALK_DEPTH_M:
            continue                       # arm too short to hold a crossing
        c = sn.point_at(pts, s)
        t = sn.tangent_at(pts, s)
        _make_ribbon(stage, f"{gnd}/xwalk_mask_{node.id}_{e.id}",
                     [sn._sub(c, sn._mul(t, _CROSSWALK_DEPTH_M * 0.6)),
                      sn._add(c, sn._mul(t, _CROSSWALK_DEPTH_M * 0.6))],
                     e.half_w, z_crosswalk - 0.02, ssf, uv_asphalt,
                     (0.15, 0.15, 0.15), asphalt_mat)
        n_xbar += _paint_ladder(stage, f"{gnd}/xwalk_{node.id}_{e.id}",
                                c, t, e.half_w, ssf, z_crosswalk)
        n_x += 1

    # 4) Stop bars, on exactly the arms `build_signs` puts a sign on.
    n_stop = 0
    for node, e in _stop_approaches(net):
        pts = _arm_pts(net, node, e)
        s = _stop_bar_s(net, node)
        if sn.polyline_length(pts) < s + 1.0:
            continue
        _paint_stop_bar(stage, f"{gnd}/stopbar_{node.id}_{e.id}",
                        sn.point_at(pts, s), sn.tangent_at(pts, s),
                        e.half_w, ssf, z_stopbar)
        n_stop += 1

    if park is not None:
        apply_park_ground(stage, config, park, gnd, ssf,
                          (asphalt_mat, grass_mat, park_grass_mat, dirt_mat))

    print(f"[suburb_scene] ground: {n_road} road ribbons "
          f"({n_road_alt} on the road tile), {n_bulb} turnarounds, "
          f"{n_grass} block meshes ({n_rough} rough/undeveloped), "
          f"{n_drive} driveways, {n_walk} front walks")
    print(f"[suburb_scene] markings: {n_dash} centreline dashes on collectors "
          f"({n_dash_cut} suppressed at junctions), {n_x} crossings "
          f"({n_xbar} bars), {n_stop} stop bars")


# ---------------------------------------------------------------------------
# placements
# ---------------------------------------------------------------------------

class AssetPools:
    """Asset lists with the per-asset corrections `build_city` applies.

    THE FOUR THINGS A BARE REFERENCE GETS WRONG, all of which this scene had:

      z      A prop must sit at ``fp["base"]``, the measured distance from the
             asset's origin to the bottom of its bounding box. Placing at z=0
             buries or floats everything by whatever that offset happens to be.
      scale  Entries carry ``scale:`` (0.01 for cm-authored art). Defaulting to
             1.0 renders a centimetre asset 100x too big.
      axis_up Y-up art needs a +90 roll to stand up in a Z-up world, and its
             sx/sy swap — which is why `resolver.get` takes axis_up too.
      yaw    ``yaw-offset`` corrects art not authored facing +X.

    `_normalize_usd_list` is the same module-level parser `build_city` uses, so
    these come out identical rather than merely similar.
    """

    def __init__(self, config):
        self.asset_scale = float(config.get("asset_scale", 1.0) or 1.0)
        self.asset_root = str(config.get("asset_root", "") or "").rstrip("/")
        self._scale, self._axis, self._yaw = {}, {}, {}

    def load(self, raw):
        paths, sc, au, yo, _tags = sg._normalize_usd_list(
            raw, self.asset_scale, self.asset_root)
        self._scale.update(sc)
        self._axis.update(au)
        self._yaw.update(yo)
        return paths

    def load_tagged(self, raw, tag):
        """Only the entries carrying *tag*.

        A tag is a CAPABILITY, not a sub-pool. In `lot_fences`, "privacy" means
        an entry may fence a lot boundary at all and "low" additionally means it
        is short enough for a front yard — US codes cap a front fence at 3.5 ft
        against 6-8 ft rear — so most entries carry both and the pools overlap
        on purpose. Reading them as two disjoint halves is what gave one house
        a picket down its side and a railing across its front.
        """
        paths, sc, au, yo, tags = sg._normalize_usd_list(
            raw, self.asset_scale, self.asset_root)
        self._scale.update(sc)
        self._axis.update(au)
        self._yaw.update(yo)
        return [p for p in paths if tag in (tags.get(p) or frozenset())]

    def scale_of(self, p):
        return self._scale.get(p, self.asset_scale)

    def axis_of(self, p):
        return self._axis.get(p, "Z")

    def yaw_of(self, p):
        return self._yaw.get(p, 0.0)

    def roll_of(self, p):
        return 90.0 if self.axis_of(p) == "Y" else 0.0

    def place(self, resolver, usd, category, x, y, yaw, rng=None,
              z_extra=0.0, scale_mul=1.0):
        """One placement dict with every correction applied."""
        sc = self.scale_of(usd) * scale_mul
        au = self.axis_of(usd)
        fp = resolver.get(usd, category, scale=sc, axis_up=au)
        return {
            "usd": usd, "x_m": x, "y_m": y,
            "z_m": fp.get("base", 0.0) + z_extra,
            "yaw_deg": yaw + self.yaw_of(usd),
            "roll_deg": self.roll_of(usd), "pitch_deg": 0.0,
            "scale": sc, "category": category, "axis_up": au,
        }


def _raw_pool(config, *path):
    node = config.get("usds", {}) or {}
    for k in path:
        node = (node or {}).get(k) or {}
    return node if isinstance(node, list) else []


def _shift_park(park, dx, dy):
    """Translate a park built at the origin into its reserved rectangle."""
    def mv(p):
        return (p[0] + dx, p[1] + dy)
    out = dict(park)
    out["region"] = (park["region"][0] + dx, park["region"][1] + dy,
                     park["region"][2] + dx, park["region"][3] + dy)
    out["zones"] = [dict(z, centre=mv(z["centre"]),
                         corners=[mv(q) for q in z["corners"]],
                         courts=[dict(cc, centre=mv(cc["centre"]))
                                 for cc in z.get("courts", [])])
                    for z in park["zones"]]
    out["paths"] = [dict(pa, pts=[mv(q) for q in pa["pts"]])
                    for pa in park["paths"]]
    out["fences"] = [dict(f, c=mv(f["c"])) for f in park["fences"]]
    out["props"] = [dict(pr, c=mv(pr["c"])) for pr in park["props"]]
    return out


# Park prop kind -> (asset pool key, placement category). The pools live in
# config/asset_sets/park.yaml; a kind with no pool is skipped and logged rather
# than silently dropped.
_PARK_POOLS = {
    "hoop":           ("park_hoop", "park_feature"),
    "soccer_goal":    ("park_soccer_goal", "park_feature"),
    "gazebo":         ("park_gazebo", "park_feature"),
    "fountain":       ("park_fountain", "park_feature"),
    "park_sign":      ("park_sign", "sign"),
    "picnic_table":   ("park_table", "bench"),
    "bench":          ("benches", "bench"),
    "trash_can":      ("trash_cans", "trash_can"),
    "bike_rack":      ("bike_racks", "bike_rack"),
    "swing_set":      ("park_play_swing", "play_structure"),
    "play_structure": ("park_play_structure", "play_structure"),
    "seesaw":         ("park_play_seesaw", "play_structure"),
    "tree":           ("trees", "tree"),
}


def _park_placements(config, resolver, park, rng, pools):
    """Place the park's props, its fence runs and its tennis courts."""
    out = []
    missing = set()
    cache = {}

    def pool_for(key):
        if key not in cache:
            cache[key] = pools.load(_raw_pool(config, key))
        return cache[key]

    # BENCH YAW. `shared.yaml` carries `yaw-offset: -90` on the bench art, and
    # the urban pass DISCARDS the +-90 part of any such offset on purpose —
    # `city_detail._prop_yaw` derives that quarter turn from the measured
    # footprint instead, since a bench's long axis is not a matter of opinion.
    # This pass applies the offset verbatim, so every bench came out a quarter
    # turn from where urban puts it.
    #
    # A BLANKET +90 CANCELLED THAT FOR EXACTLY HALF THE POOL. It was written
    # when `benches` was only the three `SM_Bench0*` entries, all declaring
    # -90. The park-tagged art added since does not: `Muyang/DownTown/Bench`
    # declares 0 and the two `CityPark/SM_Log_shop*` declare +90, so adding 90
    # to all six left them 90 and 180 degrees out from each other. The park
    # draws from the whole pool (`pools.load`, not `load_tagged`), so which
    # way a park bench faced came down to which entry the seed picked.
    #
    # Cancel the entry's OWN declared offset instead — the same
    # `fix - pools.yaw_of(usd)` idiom the fence pass uses — so every bench
    # lands on the raw computed yaw whatever art backs it, and the quarter
    # turn the park wants is one number in one place.
    _BENCH_QUARTER_TURN = 90.0
    _BENCH_FIX = {"bench": 90.0}     # picnic tables, unchanged

    for pr in park["props"]:
        spec = _PARK_POOLS.get(pr["kind"])
        if spec is None:
            missing.add(pr["kind"])
            continue
        pool = pool_for(spec[0])
        if not pool:
            missing.add(pr["kind"])
            continue
        # ONE BENCH TYPE AND ONE BIN TYPE PER PARK — furnished by one
        # procurement order, not a salvage yard. `suburb_park` stamps a per-kind
        # `variant` once per seed and this consumes it, so the pools keep every
        # entry and another seed furnishes differently.
        #
        # Not doable by narrowing the pool: `benches` and `trash_cans` come from
        # `shared.yaml`, so cutting them would change the street furniture too.
        u = (pool[pr["variant"] % len(pool)] if "variant" in pr
             else pool[rng.randrange(len(pool))])
        if pr["kind"] == "bench":
            fix = _BENCH_QUARTER_TURN - pools.yaw_of(u)
        else:
            fix = _BENCH_FIX.get(spec[1], 0.0)
        out.append(pools.place(resolver, u, spec[1], pr["c"][0], pr["c"][1],
                               pr.get("yaw", 0.0) + fix, rng))

    # Fence panels: one asset per panel, yawed along its run.
    fence = pool_for("park_fence")
    for f in park["fences"]:
        if not fence:
            break
        u = fence[rng.randrange(len(fence))]
        out.append(pools.place(resolver, u, "fence", f["c"][0], f["c"][1],
                               f["yaw"], rng))

    # The tennis COURT is a whole asset, unlike basketball where only the hoop
    # is sourced and the slab is drawn.
    tennis = pool_for("park_tennis_court")
    for z in park["zones"]:
        if z["kind"] != "tennis_block" or not tennis:
            continue
        for court in z.get("courts", []):
            u = tennis[rng.randrange(len(tennis))]
            out.append(pools.place(resolver, u, "park_feature",
                                   court["centre"][0], court["centre"][1],
                                   court["yaw"], rng))
    if missing:
        print("[suburb_scene] park: no asset pool for %s — skipped"
              % ", ".join(sorted(missing)))
    return out


def build_frontage(config, resolver, net, blocks, rng, pools):
    """Sidewalk tiles, streetlights and hydrants along every block frontage.

    `build_city` and `city_detail` do this for the urban scene but take blocks
    and corridors as RECTS, so neither can be called here. Walking a polygon
    frontage is if anything simpler: a tile every `step` along the ring's
    arclength, yawed to the local tangent, so the pavement follows the curve.

    Two things the bare ring walk gets wrong, both handled:

    CORNERS PILE UP — three or four blocks each lay their own ring into the
    same junction. Tiles inside the junction zone are dropped, leaving the
    corner open for the crossing.

    PROPS LAND ON THE ROAD — the inward offset assumes the block polygon is the
    kerb, which fails wherever `offset_polygon` hits its mitre limit. Every prop
    is checked positively against the centrelines (:class:`_RoadIndex`).
    """
    det = (config.get("city_detail") or {})
    cats = (det.get("categories") or {})
    zones_cfg = (det.get("zones") or {})
    verge = float(zones_cfg.get("furnishing_inset_m", 1.6) or 1.6)

    tiles = (config.get("usds", {}) or {}).get("tiles", {}) or {}
    walk_raw = tiles.get("sidewalk") or tiles.get("brick") or []
    walk = pools.load(walk_raw)
    lamps = pools.load(_raw_pool(config, "streetlights"))
    hyd = pools.load(_raw_pool(config, "fire_hydrants"))
    # KERBSIDE bins only. The modular kit has a bin of its own, but one parked
    # on every driveway reads as bin day, not as a street; these are the
    # scene's own `trash_cans` and they go on the verge rhythm with the lamps.
    bins = pools.load(_raw_pool(config, "trash_cans"))

    def spacing(key, default):
        return float((cats.get(key) or {}).get("spacing_m", default) or 0.0)

    lamp_sp = spacing("streetlights", 120.0)
    hyd_sp = spacing("fire_hydrants", 240.0)
    bin_sp = spacing("trash_cans", 190.0)

    out = []
    if walk:
        # Tile step from the measured tile, so the ring is continuous rather
        # than dotted or overlapping.
        fp0 = resolver.get(walk[0], "sidewalk", scale=pools.scale_of(walk[0]),
                           axis_up=pools.axis_of(walk[0]))
        step = max(1.0, min(fp0["sx"], fp0["sy"]) * 0.98)
    else:
        step = 4.0

    corners = _junction_zones(net, extra=_SIDEWALK_CORNER_M)
    road = _RoadIndex(net)
    n_corner = n_walk_road = n_prop_road = 0

    for blk in blocks:
        if blk.get("undeveloped"):
            continue            # no kerb furniture along unbuilt land
        poly = blk["poly"]
        front = blk.get("frontage")
        ring = list(poly) + [poly[0]]
        cum = sp._ring_cum(ring)
        perim = sn.polyline_length(ring)
        if perim < 20.0:
            continue

        def on_street(s):
            if not front:
                return True
            return bool(front[sp._side_at(cum, s) % len(front)])

        # -- sidewalk ring -------------------------------------------------
        s = 0.0
        while walk and s < perim:
            if on_street(s):
                p = sn.point_at(ring, s)
                t = sn.tangent_at(ring, s)
                n = sp._inward(poly, p, t)
                q = sn._add(p, sn._mul(n, verge * 0.5))
                if _in_zone(corners, q):
                    n_corner += 1
                elif road.on_road(q):
                    n_walk_road += 1
                else:
                    u = walk[rng.randrange(len(walk))]
                    yaw = math.degrees(math.atan2(t[1], t[0]))
                    out.append(pools.place(resolver, u, "sidewalk",
                                           q[0], q[1], yaw, rng))
            s += step

        # -- lamps and hydrants, on their own rhythm along the verge -------
        for pool, sp_m, cat in ((lamps, lamp_sp, "streetlight"),
                                (hyd, hyd_sp, "fire_hydrant"),
                                (bins, bin_sp, "trash_can")):
            if not pool or sp_m <= 0.0:
                continue
            s = rng.uniform(0.0, sp_m)
            while s < perim:
                if on_street(s):
                    p = sn.point_at(ring, s)
                    t = sn.tangent_at(ring, s)
                    n = sp._inward(poly, p, t)
                    q = sn._add(p, sn._mul(n, verge))
                    # A lamp or hydrant standing in the carriageway is worse
                    # than a missing one, so the margin here is wider than for
                    # a flat paving slab: it must clear the kerb, not touch it.
                    if road.on_road(q, margin=0.3):
                        n_prop_road += 1
                    else:
                        u = pool[rng.randrange(len(pool))]
                        # Face the street: inward normal points into the block,
                        # so the kerb is at -n.
                        yaw = math.degrees(math.atan2(-n[1], -n[0]))
                        out.append(pools.place(resolver, u, cat, q[0], q[1],
                                               yaw, rng))
                s += sp_m
    print(f"[suburb_scene] frontage: {len(out)} props, {n_corner} tiles dropped "
          f"at junction corners, {n_walk_road} tiles and {n_prop_road} "
          f"lamps/hydrants dropped for landing on the road")
    return out


def build_signs(config, resolver, net, rng, pools):
    """A stop sign on every minor approach to a junction.

    `city_detail` decides this from corridor lane counts; the same rule applies
    here off the graph, which actually knows the junction degree directly. A
    suburb is stop-controlled throughout, which is what the suburban preset's
    `traffic_lights.intersection_chance: 0` already says.
    """
    signs = pools.load(_raw_pool(config, "signs_stop")
                       or _raw_pool(config, "traffic_signs"))
    if not signs:
        return []
    out = []
    for n, e in _stop_approaches(net):
        pts = _arm_pts(net, n, e)
        # AT THE STOP LINE, not at `near_corner_m`. A stop sign stands beside
        # the bar a driver stops at; `city_detail`'s 6.0 m corner setback is a
        # rect-corridor figure and put the sign several metres up the road from
        # its own paint.
        near = _stop_bar_s(net, n)
        if sn.polyline_length(pts) < near + 2.0:
            continue
        p = sn.point_at(pts, near)
        t = sn.tangent_at(pts, near)
        # `_perp` is the LEFT normal of t, and t points away from the junction,
        # so this is the right hand of a driver approaching it — the kerb a US
        # stop sign stands on.
        q = sn._add(p, sn._mul(sn._perp(t), e.half_w + 1.2))
        u = signs[rng.randrange(len(signs))]
        # Sign faces back down its own approach, at the driver.
        yaw = math.degrees(math.atan2(-t[1], -t[0]))
        out.append(pools.place(resolver, u, "sign", q[0], q[1], yaw, rng))
    # ---- street-name blades ------------------------------------------------
    # There was NO placement pass for these at all: the pool resolves fine (two
    # entries on the suburban set) and `city_detail` puts one on every junction
    # corner downtown, but nothing in the graph suburb ever read it, so the
    # street names were simply absent. One blade per junction of degree >= 3,
    # on the far kerb of the first arm so it does not share a post with the
    # stop sign already standing on the near one.
    blades = pools.load(_raw_pool(config, "signs_street_name"))
    n_blade = 0
    if blades:
        seen = set()
        for node in net.nodes.values():
            if node.road_degree(net) < 3 or node.id in seen:
                continue
            arms = [net.edges[eid] for eid in node.edges
                    if net.edges[eid].road_class != "boundary"]
            if not arms:
                continue
            seen.add(node.id)
            e = arms[0]
            pts = _arm_pts(net, node, e)
            if sn.polyline_length(pts) < 6.0:
                continue
            at = min(5.0, sn.polyline_length(pts) - 1.0)
            p = sn.point_at(pts, at)
            t = sn.tangent_at(pts, at)
            # Opposite kerb from the stop sign: negate the left normal.
            q = sn._add(p, sn._mul(sn._perp(t), -(e.half_w + 1.2)))
            u = blades[rng.randrange(len(blades))]
            # A blade is read from the cross street, so it stands square to this
            # arm — the same quarter turn `city_detail`'s _TRAFFIC mode applies.
            yaw = math.degrees(math.atan2(-t[1], -t[0])) + 90.0
            out.append(pools.place(resolver, u, "sign", q[0], q[1], yaw, rng))
            n_blade += 1
    print(f"[suburb_scene] signs: {len(out) - n_blade} stop signs, "
          f"{n_blade} street-name blades")
    return out


# ---------------------------------------------------------------------------
# lot fences
# ---------------------------------------------------------------------------
# FRONT-YARD HEIGHT CAP. Every US fence ordinance that caps height at all caps
# the front yard at 3.5-4 ft and the side/rear at 6-8 ft; 1.22 m is 4 ft, the
# permissive end of that. A house draws ONE asset for its whole perimeter (see
# `_fence_pick` below), so this is the rule that decides whether that asset may
# also close the front: a 1.9 m close-board panel along the kerb reads as a
# compound, not a suburb. Houses whose asset is over the cap simply get an open
# front yard, which is the defining feature of post-war American suburbia and
# not a compromise.
_FRONT_FENCE_MAX_H_M = 1.22

# HOW FAR A FENCE MODULE MUST CLEAR THE CARRIAGEWAY. `build_frontage` uses
# 0.3 m for an upright prop — "it must clear the kerb, not touch it" — and a
# module is placed by its CENTRE, so half its own thickness hangs past the point
# being tested. The thickest module in the shipped pool is the 0.33 m park
# railing, so 0.3 + 0.17 = 0.5 m is that same rule applied to a thing with width.
_FENCE_ROAD_MARGIN_M = 0.5


def _fence_module(resolver, pools, usd):
    """``(length, thickness, height, yaw_fix)`` of one fence module, in metres.

    THE LONG PLAN AXIS IS THE FENCE, AND IT IS MEASURED. Which authored axis
    runs along the run used to be read off the entry's `yaw-offset` — off a
    hand-written declaration — and a wrong one is silent and catastrophic:
    taking a 7.6 cm board THICKNESS as the module length lays forty-six panels
    per metre, each turned across the boundary instead of along it. That is
    exactly the failure this pass keeps coming back with.

    A fence module is longer than it is thick by construction. That is a fact
    about the geometry, not about anyone's authoring convention, so it is what
    decides here. `yaw_fix` is what to add to the RUN DIRECTION to land the long
    axis on it, and for fences it REPLACES the entry's `yaw-offset` rather than
    compounding with it.
    """
    fp = resolver.get(usd, "fence", scale=pools.scale_of(usd),
                      axis_up=pools.axis_of(usd))
    sx, sy = float(fp["sx"]), float(fp["sy"])
    if sy > sx:                                   # authored running +Y
        return (max(0.3, sy), max(0.02, sx), float(fp.get("sz") or 0.0), 90.0)
    return (max(0.3, sx), max(0.02, sy), float(fp.get("sz") or 0.0), 0.0)


def _fence_box(x, y, yaw_deg, length, thick):
    """One module as its true oriented rectangle, for the overlap test."""
    a = math.radians(yaw_deg)
    return sp._corners(x, y, length, thick, math.cos(a), math.sin(a))


class _FenceGrid:
    """Fence modules already standing, hashed by cell: "is this ground taken?".

    TWO WAYS A FENCE CAN BE IN ANOTHER FENCE, and they need different tests.

    CROSSING. Tested against a HAIRLINE CORE of each module rather than its full
    box, and that is the whole trick. Two runs that meet at a lot corner
    necessarily interpenetrate by half a panel thickness — that is a mitre, and
    the render wants it. Two runs that CROSS interpenetrate along their length.
    Shrinking both boxes to a 2 cm ribbon that is also 2 cm short at each end
    separates the first case (they touch at a point) from the second (the
    ribbons still cross) with no threshold to tune.

    DOUBLING — two fences down one boundary, a metre apart. The cores never
    touch, so the crossing test is blind to it, and it is most of what the eye
    reads as "the fences overlap": measured on seed 3, 106 modules. Its cause is
    two neighbours platting the shared side line from their own frontage
    stations, so it is caught upstream by `suburb_parcel._line_dupe` — but that
    is a tolerance on a heuristic, and this is the hard guarantee underneath it.
    Near-parallel, closer than a side yard could ever be, and actually
    overlapping along the line, all three.
    """

    _CORE_M = 0.02
    # Lateral clearance below which two parallel fences are one fence drawn
    # twice. The narrowest gap between two genuinely different lot boundaries is
    # a side yard, and `house_gap_m` is 6 m; 1.2 m is `_line_dupe`'s own
    # tolerance, so the two passes agree on what "the same boundary" means.
    # RAISED 1.2 -> 3.0. 1.2 was `_line_dupe`'s own tolerance, which agrees
    # with it on what "the same boundary" means but is far tighter than what
    # the eye calls a doubled fence: two neighbours platting the shared line
    # from their own frontage stations land 1.5-2.5 m apart, clear the test,
    # and stand as two parallel fences down one boundary. The nearest two
    # genuinely DIFFERENT lot lines can be is a side yard, and `house_gap_m`
    # is 6 m, so 3.0 has room underneath it and nothing real to lose.
    _DOUBLE_M = 3.0
    _DOUBLE_COS = 0.985                    # 10 degrees
    _DOUBLE_OVERLAP_M = 0.5

    def __init__(self, cell=8.0):
        self.cell = float(cell)
        self.cells = {}

    def _core(self, x, y, yaw_deg, length):
        return _fence_box(x, y, yaw_deg, max(0.1, length - 2 * self._CORE_M),
                          self._CORE_M)

    def _keys(self, box):
        x0 = min(q[0] for q in box); x1 = max(q[0] for q in box)
        y0 = min(q[1] for q in box); y1 = max(q[1] for q in box)
        for gx in range(int(math.floor(x0 / self.cell)),
                        int(math.floor(x1 / self.cell)) + 1):
            for gy in range(int(math.floor(y0 / self.cell)),
                            int(math.floor(y1 / self.cell)) + 1):
                yield (gx, gy)

    def _entry(self, x, y, yaw_deg, length):
        a = math.radians(yaw_deg)
        ux, uy = math.cos(a), math.sin(a)
        h = length / 2.0
        return (self._core(x, y, yaw_deg, length), x, y, ux, uy, h,
                (x - ux * h, y - uy * h), (x + ux * h, y + uy * h))

    def _reach_box(self, x, y, yaw_deg, length):
        """The module grown by the doubling reach on every side.

        BOTH the query and the registration use this, and they have to: the
        doubling test looks sideways, so a module registered only under the
        cells its 2 cm core crosses is invisible to a query from the next cell
        over. Measured, that asymmetry let 2 doubled modules a seed through.
        """
        return _fence_box(x, y, yaw_deg, length + 2 * self._DOUBLE_M,
                          2 * self._DOUBLE_M)

    def free(self, x, y, yaw_deg, length):
        me = self._entry(x, y, yaw_deg, length)
        seen = set()
        for k in self._keys(self._reach_box(x, y, yaw_deg, length)):
            for other in self.cells.get(k, ()):
                if id(other) in seen:
                    continue
                seen.add(id(other))
                if sp._obb_overlap(me[0], other[0]):
                    return False
                if abs(me[3] * other[3] + me[4] * other[4]) < self._DOUBLE_COS:
                    continue
                # BOTH TESTS ARE SYMMETRIC, and they have to be: this runs once
                # per pair, from whichever module was laid second, so anything
                # frame-relative accepts or rejects the same pair depending on
                # the order — 2 doubled modules a seed slipped through on
                # exactly that. Two panels 8 degrees apart overlap by 0.62 m
                # projected on one and 0.45 m on the other, and sit 1.41 m apart
                # measured from one and 0.99 m from the other. So the overlap
                # takes the SHORTER projected separation (the longer overlap),
                # and the offset is the distance between the two centre lines —
                # which, once they overlap along the line, IS the offset.
                dx, dy = other[1] - me[1], other[2] - me[2]
                along = min(abs(dx * me[3] + dy * me[4]),
                            abs(dx * other[3] + dy * other[4]))
                if me[5] + other[5] - along <= self._DOUBLE_OVERLAP_M:
                    continue
                if sn.seg_seg_dist(me[6], me[7], other[6],
                                   other[7]) < self._DOUBLE_M:
                    return False
        return True

    def add(self, x, y, yaw_deg, length):
        e = self._entry(x, y, yaw_deg, length)
        for k in self._keys(self._reach_box(x, y, yaw_deg, length)):
            self.cells.setdefault(k, []).append(e)


# WHICH FENCE A HOUSE GETS, weighted toward a full-height panel. A fenced US
# back yard is a 6 ft wood or chain-link fence; a 3 ft picket or rail right
# round a lot is real but is the minority case, and 3:1 is what keeps it one.
_FENCE_TALL_WEIGHT = 3.0


def _fence_pick(pool, mods, segs, rng):
    """The ONE asset this house fences its whole perimeter with.

    DRAWN, NOT SCORED, and that is the correction worth recording. Scoring the
    candidates on how exactly they tile the boundaries picks the SHORTEST module
    essentially every time — a 2 m picket divides an arbitrary length better
    than a 5.5 m panel — which put 8,872 of 9,225 modules on one asset: a
    monotonous suburb rather than a fixed one. So tiling only decides whether an
    asset CAN serve this house at all, and the choice among the survivors is a
    weighted draw.

    "Can serve" is counted over the runs the asset would actually DRAW: a panel
    over the front-yard height cap skips the front runs by design, and must not
    be marked down for failing to tile something it was never going to lay.
    """
    if not pool:
        return None
    best, cands = None, []
    for cand in pool:
        mod_len, _th, mod_h, _fix = mods[cand]
        todo = [s for s in segs
                if not (s[2] == "low" and mod_h > _FRONT_FENCE_MAX_H_M)]
        miss = sum(1 for (a, b, _t) in todo if not _fence_run(a, b, mod_len))
        if best is None or miss < best:
            best, cands = miss, [cand]
        elif miss == best:
            cands.append(cand)
    w = [(_FENCE_TALL_WEIGHT if mods[c][2] > _FRONT_FENCE_MAX_H_M else 1.0)
         for c in cands]
    t = rng.random() * sum(w)
    for c, wi in zip(cands, w):
        t -= wi
        if t <= 0.0:
            return c
    return cands[-1]


def _trim_offroad(p0, p1, road, margin, step=0.75):
    """The longest stretch of p0->p1 that clears the carriageway, or ``None``.

    A POSITIVE test against the centrelines, for the reason :class:`_RoadIndex`
    gives: the block polygon is nominally the kerb but bulges over it wherever
    `offset_polygon` hit its mitre limit, and a lot line is a straight chord
    across a face that curves — at an outside corner it leaves the block
    entirely. Measured on seed 3, 1,056 of 4,877 fence modules (22%) stood
    inside a carriageway with the front lot line taken at face value.

    LONGEST STRETCH, not "trim both ends": if the middle of a run is on asphalt
    then the run crosses a road, and trimming the ends would leave it crossing.
    Sampled at *step* and then shrunk by half a step on each cut end, because
    the real kerb crossing lies between two samples and the conservative guess
    is the one that keeps the fence off the road.
    """
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    ln = math.hypot(dx, dy)
    if ln < 1.0:
        return None
    ux, uy = dx / ln, dy / ln
    n = max(2, int(math.ceil(ln / step)) + 1)
    ok = [not road.on_road((p0[0] + ux * ln * i / (n - 1),
                            p0[1] + uy * ln * i / (n - 1)), margin)
          for i in range(n)]
    best = run = None
    for i, good in enumerate(ok + [False]):
        if good:
            run = (i, i) if run is None else (run[0], i)
        elif run is not None:
            if best is None or run[1] - run[0] > best[1] - best[0]:
                best = run
            run = None
    if best is None:
        return None
    half = 0.5 * ln / (n - 1)
    t0 = ln * best[0] / (n - 1) + (half if best[0] > 0 else 0.0)
    t1 = ln * best[1] / (n - 1) - (half if best[1] < n - 1 else 0.0)
    if t1 - t0 < 1.0:
        return None
    return ((p0[0] + ux * t0, p0[1] + uy * t0),
            (p0[0] + ux * t1, p0[1] + uy * t1))


def _fence_run(p0, p1, mod_len, min_fit=0.60, max_fit=1.15):
    """Module centres, yaw and fit-scale for a fence along p0->p1.

    THE RUN IS COVERED END TO END. Laying whole modules and dropping the
    remainder left a 10 m boundary with one 5.95 m panel and 2 m bare at each
    end, and gave nothing at all to a boundary shorter than one module. So the
    count is chosen to SPAN the run and each module carries the residual as a
    scale — the trick `apply_ground` uses to land crossing bars kerb to kerb.
    Both candidate counts tile exactly and differ only in squeeze vs stretch,
    so the one closer to the authored size wins.

    The scale is UNIFORM (a placement carries one scale), so a squeezed panel is
    also shorter. Hence the [min_fit, max_fit] band: missing 1.5 m of fence
    reads better than a 2.7 m one towering over a bungalow.
    """
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dy)
    if length < 1.0 or mod_len <= 1e-6:
        return []
    n_hi = max(1, int(math.ceil(length / mod_len - 1e-9)))   # squeeze to fit
    n_lo = max(1, n_hi - 1)                                  # stretch to fit
    best = None
    for n in {n_hi, n_lo}:
        fit = length / (n * mod_len)
        if fit < min_fit or fit > max_fit:
            continue
        if best is None or abs(math.log(fit)) < abs(math.log(best[1])):
            best = (n, fit)
    if best is None:
        return []
    n, fit = best
    step = length / n
    ux, uy = dx / length, dy / length
    yaw = math.degrees(math.atan2(dy, dx))
    return [(p0[0] + ux * (i + 0.5) * step,
             p0[1] + uy * (i + 0.5) * step, yaw, fit)
            for i in range(n)]


# Placement categories that OCCUPY GROUND, i.e. that the infill has to keep
# clear of. Houses and fences are absent on purpose: both stand inside a lot
# rectangle, which is a keep-out in its own right, and one modular house is ~28
# placements — seeding them would treble the grid to re-answer a question the
# lot test has already answered.
_GROUND_OCCUPANTS = frozenset((
    "tree", "plant", "streetlight", "fire_hydrant", "sign", "bench",
    "trash_can", "bike_rack", "play_structure", "park_feature", "sidewalk",
))


def build_open_planting(config, resolver, net, blocks, rng, pools,
                        parcels=(), info=None, existing=(), pool_rects=(),
                        park_rect=None):
    """Trees on every piece of ground the plat left bare.

    THREE KINDS OF BARE GROUND, and this pass used to see only the first:

      undeveloped blocks   land never platted — drainage reserve, woodland, a
                           phase not sold. `suburb_net` flags them and
                           `apply_ground` gives them rough grass; without trees
                           that reads as a block somebody forgot to build on
                           rather than as land nobody ever built on.

      block interiors      the leftover inside a DEVELOPED block. Lots run
                           `lot_depth` (26-38 m) in from their frontage while
                           the median block is 2.8 ha, so on the big ones the
                           middle is out of every lot's reach: measured on the
                           shipped preset at seed 3, 125.1 ha of developed
                           block holding 77.2 ha of platted lot — 47.9 ha of
                           mown grass behind the back fences, with nothing
                           standing on it at all.

      the park surround    THE ONE THE EYE GOES TO, and the one that was
                           invisible to every pass in the file. `suburb_park`
                           plans its content into the park RECT, but the park's
                           real extent is `info["park"]["poly"]`: the rect,
                           plus the `park_pad_m` verge and the frame's ring
                           offset, plus every undeveloped parcel
                           `_merge_open_land` folded into it. And a merged
                           parcel STOPS BEING A BLOCK — `blocks_from_faces`
                           declines to emit it because its face now covers the
                           reserve — so it carries no `undeveloped` flag, is in
                           no `blocks` list, and nothing could find it. That is
                           the empty area adjoining the park. At seed 3 it is
                           one parcel of 4.4 ha, against a 12.6 ha park — a
                           third as much again of the park's own ground,
                           bare.

    A STRATIFIED GRID, NOT A DART BOARD. The old pass threw `want * 12` uniform
    darts at a bounding box and stopped when it had enough, which fills a
    polygon on average and not in fact: the accepted points clump, and the
    holes it leaves are exactly the "empty spaces with nothing in them". One
    dart per cell of a `sqrt(100 / rate)` grid, jittered anywhere inside its own
    cell, covers the region by construction while still reading as planting
    rather than as an orchard — and where the first dart lands on a lot or a
    road there are `darts` more tries within the same cell, so tight ground
    still gets its tree instead of a gap.

    KEEP-OUTS, all of them already computed by somebody else:
      roads      `_RoadIndex` — the centrelines, because a block polygon can
                 bulge over the carriageway (see its docstring)
      turnarounds the lollipop bulb discs, which the centreline test cannot see
      lots       every `lot_corners` rectangle: house, drive, garage, pool and
                 yard planting all live inside one, and `suburb_yardplan` owns
                 that ground
      pools      the rectangles `build_placements` cut out of the lawn
      park       the park rect, when its content was built into it
      standing   everything already placed, via `_GROUND_OCCUPANTS`

    Size comes from the same `_size_t` ramp `build_placements` uses, so a tree
    just over a back fence is a 4 m Beech and one in the middle of the reserve
    is a 25 m Black_Oak. Measured over the shipped preset at seed 3: 2,109
    trees before, 4,726 after — 2,037 on undeveloped blocks, 2,300 in block
    interiors, 389 around the park — and the share of open ground (sampled on a
    4 m grid, off the road and out of the houses) within 15 m of a tree goes
    75.4% -> 96.1%. Nothing lands on a road, a lot, a house, a pool or the park
    rect at seeds 1, 3, 7 and 12.
    """
    pc = _planting_cfg(config)
    cpool = _CanopyPool(resolver, pools, pools.load(_raw_pool(config, "trees")),
                        band=pc["band"])
    if not len(cpool):
        return []

    # -- what to plant, and how thickly ------------------------------------
    regions = []                     # (poly, trees per 100 m2, tag)
    for b in blocks:
        if b.get("undeveloped"):
            regions.append((b["poly"], pc["open_rate"], "undeveloped"))
    for p in parcels:
        regions.append((p["block"], pc["fill_rate"], "interior"))
    pinfo = (info or {}).get("park") or {}
    if pinfo.get("poly") and len(pinfo["poly"]) >= 3:
        regions.append((pinfo["poly"], pc["open_rate"], "park"))
    if not regions:
        return []

    # -- what not to plant on ----------------------------------------------
    road = _RoadIndex(net)
    bulb_r = float((config.get("suburb_net") or {}).get(
        "bulb_radius_m", sn.DEFAULTS["bulb_radius_m"]))
    bulbs = [e.pts[-1] for e in net.edges.values()
             if e.street_type == "lollipop"]
    bulb_idx = _Occupancy(cell=max(bulb_r + pc["clear_m"], 1.0))
    for q in bulbs:
        bulb_idx.add(q)
    houses = [h for p in parcels for h in p["houses"]]
    house_idx = _ObbIndex([_house_box(h) for h in houses],
                          reach=max(pc["far_m"], pc["near_m"]) + 1.0)
    # LOTS AND POOL HOLES IN ONE INDEX. A pool always sits in the rear yard of
    # the lot that ordered it, so the lot rectangle covers it already — but the
    # index costs nothing extra and a caller that hands over holes from
    # somewhere else still gets them honoured.
    keepout = [b for b in (_rect_box(h.get("lot_corners")) for h in houses)
               if b is not None]
    keepout += [b for b in (_rect_box(r) for r in (pool_rects or ()))
                if b is not None]
    lot_idx = _ObbIndex(keepout, reach=max(20.0, pc["clear_m"] + 1.0))
    # TWO GRIDS, TWO RADII. What is already standing only has to be cleared —
    # `clear_m`, a small crown radius — while two trees from THIS pass owe each
    # other the full `gap_m`. One grid at the larger radius would hold new
    # planting six metres off every sidewalk tile on an undeveloped frontage,
    # and would not be able to say which rule refused a candidate.
    standing = _Occupancy(cell=max(pc["clear_m"], 1.0))
    for q in (existing or ()):
        if q.get("category") in _GROUND_OCCUPANTS:
            standing.add((q["x_m"], q["y_m"]))
    mine = _Occupancy(cell=max(pc["gap_m"], 1.0))

    pave = _PavingIndex(paving_keepout(parcels),
                        clear_m=float(pc["clear_m"]) * 0.5)

    def blocked(q):
        """Why *q* cannot take a tree, or None."""
        if road.on_road(q, margin=min(1.5 + pc["clear_m"], 6.0)):
            return "road"
        # DRIVES AND FRONT WALKS ARE ROAD TOO, as far as a tree is concerned.
        # They are drawn later, in `apply_ground`, so nothing here knew they
        # existed — which is how trees ended up standing on the path to a
        # front door.
        if pave.on_paving(q):
            return "paving"
        if bulb_idx.near(q, bulb_r + pc["clear_m"]):
            return "road"
        if lot_idx.nearest(q) <= pc["clear_m"]:
            return "lot"
        # THE HOUSE TOO, not just its lot. A garage wing is allowed to sit on or
        # over the side lot line, and a lot's front half-width is chord-clamped
        # on a curving frontage — so a footprint can stick out past the
        # rectangle it was granted. Measured before this test went in: 22 trees
        # a seed standing inside a house that no lot rectangle covered.
        if house_idx.nearest(q) <= pc["clear_m"]:
            return "lot"
        if park_rect is not None and _in_rect(q, park_rect, pc["clear_m"]):
            return "park"
        if standing.near(q, pc["clear_m"]):
            return "standing"
        if mine.near(q, pc["gap_m"]):
            return "spacing"
        return None

    out = []
    tally = {}
    why = {}
    for (poly, rate, tag) in regions:
        if rate <= 0.0:
            continue
        # Cell side that yields `rate` trees per 100 m2 when every cell takes
        # one. The grid is anchored on the WORLD, not on the region's bounding
        # box, so two regions meeting at a street do not double up along it.
        step = math.sqrt(100.0 / rate)
        xs = [q[0] for q in poly]
        ys = [q[1] for q in poly]
        ix0 = int(math.floor(min(xs) / step))
        ix1 = int(math.floor(max(xs) / step))
        iy0 = int(math.floor(min(ys) / step))
        iy1 = int(math.floor(max(ys) / step))
        n_here = 0
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                for _ in range(pc["darts"]):
                    q = ((ix + rng.random()) * step, (iy + rng.random()) * step)
                    if not sn.point_in_polygon(poly, q):
                        continue
                    bad = blocked(q)
                    if bad is not None:
                        why[bad] = why.get(bad, 0) + 1
                        continue
                    mine.add(q)
                    u = cpool.draw(rng, _size_t(house_idx.nearest(q),
                                                pc["near_m"], pc["far_m"]))
                    out.append(pools.place(resolver, u, "tree", q[0], q[1],
                                           rng.uniform(0.0, 360.0), rng))
                    n_here += 1
                    break
        tally[tag] = tally.get(tag, 0) + n_here
    print("[suburb_scene] open planting: %d trees (%s); rejected %s"
          % (len(out),
             ", ".join(f"{v} {k}" for k, v in sorted(tally.items())) or "none",
             ", ".join(f"{v} {k}" for k, v in sorted(why.items())) or "none"))
    return out


def modular_catalogue(config):
    """The modular-kit styles as catalogue entries, or [] when switched off.

    Same `(w, d)` contract `suburb_parcel` packs against, so the existing lot
    floor logic sizes the plat around the kit with no special case: the widest
    style is what sets the minimum lot, exactly as a measured USD would.
    """
    if not (config.get("suburb_parcel") or {}).get("modular_houses"):
        return []
    from detail import modular_house as mh
    out = []
    for style in mh.ORDER:
        cs = mh.footprint(mh.STYLES[style])
        xs = [i for i, _ in cs]
        ys = [j for _, j in cs]
        out.append({"style": style,
                    "w": (max(xs) - min(xs) + 1) * mh.CELL_M,
                    "d": (max(ys) - min(ys) + 1) * mh.CELL_M})
    return out


def house_catalogue(config, resolver, pools, yaw_off=-90.0):
    """Measured house footprints WITH THE GARAGE WING FOLDED IN.

    THE BUG THIS EXISTS TO CLOSE. A garage lot was rendered as house PLUS its
    `SM_House_NN_Extension` at the identical transform — correct, because the
    wing is authored in its parent's frame and composes only there. But
    `suburb_parcel` reserved its garage box on a RANDOMLY CHOSEN side at a
    separation of its own invention, so the space the wing actually occupies had
    never been tested against anything. ~35% of houses carried a wing no overlap
    test in the pipeline had ever seen, and that is the remaining source of
    houses appearing to overlap.

    Neither box can move to meet the other: house and wing are one rigid asset
    pair, so the LOT has to be sized to what the art really is. That was blocked
    on measurement — the pack is on Nucleus and reading it from a shell needs
    interactive auth — but it was never blocked here: `SizeResolver` measures at
    scene-build time, inside Isaac Sim, where Nucleus is already authenticated.
    It reports `cx`/`cy` as well as `sx`/`sy`, so the union of house and wing is
    a real bounding box and not a sum of extents.

    Returns one entry per house asset, in a STABLE ORDER, because
    `suburb_parcel` stamps the index it sited into `size_index` and the caller
    must place that asset and no other.

    THE ANCHOR IS THE VISUAL CENTROID, NOT THE PRIM ORIGIN. `apply_placements`
    already rotates each asset's own bbox-centre offset out of its placement, so
    the point it is handed is where the asset's CENTROID lands. Two consequences
    this exists to serve, each of which cost a bug to learn:

      * Subtracting the union offset from the requested point DOUBLE-CORRECTS,
        because the centroid offset is taken out again downstream. Every house
        lands displaced by its own pivot offset -- large for this pack, whose
        pivot is nowhere near the middle of the house.
      * Emitting a house and its wing at the SAME point CO-CENTRES them, so the
        wing sits on the house rather than beside it. "Composes at the identical
        transform" is true of a raw reference and false the moment a centroid
        correction is applied.

    So each entry carries three points in the asset's own frame: the house
    centroid `hc`, the wing centroid `ac` (None without a wing), and the union
    centre `(ox, oy)`. The caller asks for `lot_centre + R * (hc - o)` and
    `lot_centre + R * (ac - o)`; their difference is exactly `ac - hc`, the
    authored relationship, so the wing lands where the artist put it while the
    UNION lands centred on the rectangle `suburb_parcel` reserved.
    """
    houses = pools.load(_raw_pool(config, "buildings", "intact"))
    addons = pools.load(_raw_pool(config, "buildings", "house_addons"))
    addon_for = {}
    for a in addons:
        stem = os.path.basename(a).split("_Extension")[0]
        for h in houses:
            if os.path.basename(h).split(".")[0] == stem:
                addon_for[h] = a
    # Which measured axis lies ALONG the frontage depends on the yaw offset the
    # preset applies, so it is derived rather than assumed: at -90 the asset's
    # local +Y runs along the street, at 0 its +X does.
    turned = abs((float(yaw_off) % 180.0) - 90.0) < 45.0

    out = []
    for u in houses:
        boxes = [resolver.get(u, "house", scale=pools.scale_of(u),
                              axis_up=pools.axis_of(u))]
        a = addon_for.get(u)
        if a:
            boxes.append(resolver.get(a, "house", scale=pools.scale_of(a),
                                      axis_up=pools.axis_of(a)))
        x0 = min(float(f.get("cx", 0.0)) - float(f["sx"]) / 2.0 for f in boxes)
        x1 = max(float(f.get("cx", 0.0)) + float(f["sx"]) / 2.0 for f in boxes)
        y0 = min(float(f.get("cy", 0.0)) - float(f["sy"]) / 2.0 for f in boxes)
        y1 = max(float(f.get("cy", 0.0)) + float(f["sy"]) / 2.0 for f in boxes)
        sx, sy = x1 - x0, y1 - y0
        hc = (float(boxes[0].get("cx", 0.0)), float(boxes[0].get("cy", 0.0)))
        ac = ((float(boxes[1].get("cx", 0.0)), float(boxes[1].get("cy", 0.0)))
              if a else None)
        out.append({"usd": u, "addon": a,
                    "w": sy if turned else sx,      # across the frontage
                    "d": sx if turned else sy,      # into the lot
                    # Each part's own centroid, which is the point
                    # `apply_placements` will anchor it by.
                    "hc": hc, "ac": ac,
                    "ox": 0.5 * (x0 + x1), "oy": 0.5 * (y0 + y1)})
    n_wing = sum(1 for e in out if e["addon"])
    if out:
        print(f"[suburb_scene] house catalogue: {len(out)} measured "
              f"({n_wing} with a garage wing folded into the footprint), "
              f"w {min(e['w'] for e in out):.1f}-{max(e['w'] for e in out):.1f} m, "
              f"d {min(e['d'] for e in out):.1f}-{max(e['d'] for e in out):.1f} m")
    return out


def build_placements(config, resolver, parcels, rng, pools, yaw_off=-90.0,
                     catalogue=None, pool_holes_out=None, net=None):
    """Houses, lot furniture and parcel trees, with per-asset corrections.

    TWO TREE POOLS, picked on the `kind` stamp `suburb_parcel` already emits.
    One pool for both put a 25.4 m crown (Black_Oak, 19.7 m tall) on the verge,
    overhanging the whole carriageway.

        street / front  ->  `street_trees`, modest crowns only
        back            ->  `trees`, open space, big specimens fine

    `street_trees` falls back to `trees` for sets older than the suburban set.

    AND WITHIN EITHER POOL, BY DISTANCE TO THE NEAREST HOUSE WALL. The role
    split says which pool; it cannot say which end of it. A verge tree beside a
    house and a verge tree on the frontage of a lot nothing would fit are both
    "street", and they should not be the same size — so both are drawn through
    `_CanopyPool` on a `_size_t` ramp off `_ObbIndex`. On the shipped set that
    is the 3.02 m Douglas_Fir against the 10.29 m Shumard_Oak on the verge, and
    the 4.09 m Beech against the 25.42 m Black_Oak behind the house.
    """
    houses = pools.load(_raw_pool(config, "buildings", "intact"))
    n_mod = 0
    pool_holes = pool_holes_out if pool_holes_out is not None else []
    mod_share = float((config.get("suburb_parcel") or {})
                      .get("modular_share", 1.0))
    pc = _planting_cfg(config)
    open_trees = pools.load(_raw_pool(config, "trees"))
    street_trees = pools.load(_raw_pool(config, "street_trees")) or open_trees
    open_pool = _CanopyPool(resolver, pools, open_trees, band=pc["band"])
    street_pool = _CanopyPool(resolver, pools, street_trees, band=pc["band"])
    # Every house in the SUBURB, not in the block: a tree on the last lot of one
    # block is metres from the first house of the next one across the street,
    # and a per-block index would call that open ground.
    house_idx = _ObbIndex([_house_box(h) for p in parcels for h in p["houses"]],
                          reach=max(pc["far_m"], pc["near_m"]) + 1.0)

    # ONE POOL, NOT TWO. `lot_fences` used to be split into a "low" list for
    # the front and a "privacy" list for the sides, and a house drew from both —
    # which is exactly the "some houses have two different fences" defect: on
    # seed 3, 116 of 224 fenced houses showed two assets. The tags stay, but
    # they now describe WHAT A MODULE MAY DO rather than which pool it lives in:
    # every entry that can serve a lot perimeter is tagged "privacy" and goes in
    # here, and "low" additionally means "short enough for the front yard",
    # which `_FRONT_FENCE_MAX_H_M` re-derives from the measured height anyway.
    fence_pool = pools.load_tagged(_raw_pool(config, "lot_fences"), "privacy")
    fence_mod = {u: _fence_module(resolver, pools, u) for u in fence_pool}
    # SAY WHAT WAS MEASURED. A fence laid across its own boundary at forty
    # panels a metre is obvious in a render and invisible in a log, and it has
    # now happened twice. One line per asset makes the next one a five-second
    # read: `len` should be metres, not centimetres.
    for u in fence_pool:
        ml, mt, mh, mf = fence_mod[u]
        print(f"[suburb_scene] fence module {os.path.basename(u):<44} "
              f"len {ml:5.2f} m  thick {mt:4.2f} m  high {mh:4.2f} m  "
              f"turned {mf:+.0f} deg (declared {pools.yaw_of(u):+.0f})")
    fence_taken = _FenceGrid()
    fence_road = _RoadIndex(net) if net is not None else None
    n_fence_road = n_fence_clash = 0

    out = []
    n_gar = n_fence = 0
    # Parcel trees are deferred to a second pass — see the
    # paving keep-out below.
    tree_jobs = []
    if not houses:
        print("[suburb_scene] WARNING: no buildings.intact pool in asset set")
    for p in parcels:
        for h in p["houses"]:
            if not houses:
                break
            # h["yaw_deg"] is the FRONTAGE TANGENT -- along the street. The house
            # faces ACROSS it toward the kerb, which is the tangent rotated by
            # `yaw_off`. That assumes the art faces +X; see the preset comment.
            yaw = h["yaw_deg"] + yaw_off
            ent = None
            if catalogue and h.get("size_index") is not None:
                ent = catalogue[int(h["size_index"]) % len(catalogue)]

            # A KIT HOUSE COSTS ~28 PRIMS AGAINST A WHOLE-HOUSE USD'S ONE.
            # The pieces are low-poly, so the point budget is fine (~5M over a
            # full plat against the 89M that OOM-killed the urban scene), but
            # 24k un-instanced prims is real composition cost — and they CANNOT
            # be instanced, because `apply_palette` rebinds subsets inside them
            # and USD forbids editing inside an instance. `modular_share` is
            # the dial: 1.0 all kit, 0.0 all whole-house, anything between mixes
            # them per house on a stable hash.
            use_mod = (ent is not None and "style" in ent
                       and (mod_share >= 1.0
                            or (hash(("mod", p.get("id", 0), h["c"])) % 1000)
                            / 1000.0 < mod_share))
            if use_mod:
                # MODULAR KIT. `build_building` centres the footprint on the
                # point it is given and faces its front at local -Y, whereas the
                # whole-house art faces +X — hence the +90. Getting that wrong
                # turns every house sideways to its own street.
                from detail import modular_house as mh
                parts = mh.build_building(ent["style"], h["c"][0], h["c"][1],
                                          yaw + 90.0, rng, category="house")
                pal = mh.STYLES[ent["style"]].get("palette")
                for q in parts:
                    q["palette"] = pal      # shell only; dressing is the
                out += parts                # suburb's own job, not the kit's
                n_mod += 1
                # THE SEAM, closed. `plan_lot` is the one place that knows
                # where this style's garage door and front step land, and it
                # measures the rear from the same lot record the plat platted.
                # It is kept ON THE HOUSE so `apply_ground` draws the drive and
                # the walk to those exact points instead of re-guessing them —
                # that guess is why drives arrived beside the garage and paths
                # led to blank wall.
                plan = mh.plan_lot(h, ent["style"], rng)
                h["plan"] = plan
                # The PLOT decides whether there is a pool: `has_pool` comes
                # from the archetype package, so they land on the big lots.
                pool, hole = mh.pool_at(ent["style"], h["c"][0], h["c"][1],
                                        yaw + 90.0,
                                        force=bool(h.get("has_pool")),
                                        rear_m=plan["rear_m"])
                if pool:
                    out += pool
                    pool_holes.append(hole)
            elif ent is None or "style" in ent:
                # No measurement, so no wing: without the two centroids there is
                # no way to place it beside its parent rather than inside it.
                u = houses[rng.randrange(len(houses))]
                out.append(pools.place(resolver, u, "house",
                                       h["c"][0], h["c"][1], yaw, rng))
            else:
                # PLACE THE ASSET THE LOT WAS SIZED FOR. `size_index` is the
                # entry `suburb_parcel` fitted and overlap-tested; drawing any
                # other one puts the module straight back to guessing, which is
                # what its own comment warns about.
                u = ent["usd"]
                # The angle the asset is ACTUALLY placed at -- `yaw` plus any
                # per-asset `yaw-offset` the set declares -- because that is the
                # rotation `apply_placements` applies to its own offset too.
                a = math.radians(yaw + pools.yaw_of(u))
                ca, sa = math.cos(a), math.sin(a)

                def anchor(pt, _ca=ca, _sa=sa, _e=ent, _c=h["c"]):
                    """Where to ASK for a part so the UNION lands on the lot box.

                    `apply_placements` puts the asset's CENTROID at the point it
                    is given, so what to ask for is the lot centre plus that
                    part's own displacement from the union centre. Asking for
                    the lot centre minus the union offset instead double-counts
                    the correction and throws every house off its lot by its own
                    pivot offset.
                    """
                    dx, dy = pt[0] - _e["ox"], pt[1] - _e["oy"]
                    return (_c[0] + dx * _ca - dy * _sa,
                            _c[1] + dx * _sa + dy * _ca)

                hx, hy = anchor(ent["hc"])
                out.append(pools.place(resolver, u, "house", hx, hy, yaw, rng))
                if ent["addon"]:
                    # A DIFFERENT POINT from the house, necessarily. Both at the
                    # same point co-centres them and the wing lands inside the
                    # house; the gap between these two anchors is exactly the
                    # authored offset between the two centroids.
                    ax, ay = anchor(ent["ac"])
                    out.append(pools.place(resolver, ent["addon"], "house",
                                           ax, ay, yaw, rng))
                    n_gar += 1
            segs = list(h.get("fence_segs") or ())
            if segs and fence_pool:
                # ONE ASSET PER HOUSE, ACROSS EVERY BOUNDARY. The pick used to
                # be cached per TAG, which still let a lot show a picket down
                # one side and a railing across the front — a repair, not a
                # fence. `_fence_pick` chooses it once for the whole perimeter,
                # having seen all of this house's runs at once.
                uf = h.get("_fence_pick")
                if uf not in fence_mod:
                    uf = _fence_pick(fence_pool, fence_mod, segs, rng)
                    h["_fence_pick"] = uf
                mod_len, _mod_th, mod_h, mod_fix = (fence_mod[uf] if uf
                                                   else (0.0, 0.0, 0.0, 0.0))
                # `place` adds the entry's yaw-offset; the measured fix is the
                # authority for fences, so cancel the declaration out.
                mod_yaw = mod_fix - pools.yaw_of(uf) if uf else 0.0
                for (a, b, tag) in (segs if uf else ()):
                    # A "low" run is the FRONT boundary. This house has one
                    # asset for its whole perimeter, so the front is either
                    # fenced with that asset or left open — see
                    # `_FRONT_FENCE_MAX_H_M`.
                    if tag == "low" and mod_h > _FRONT_FENCE_MAX_H_M:
                        continue
                    span = (a, b)
                    if fence_road is not None:
                        span = _trim_offroad(a, b, fence_road,
                                             _FENCE_ROAD_MARGIN_M)
                        if span is None:
                            n_fence_road += 1
                            continue
                    run = _fence_run(span[0], span[1], mod_len)
                    if not run:
                        continue
                    # LAST GUARD: ground another fence already stands on. The
                    # parcel pass cuts a boundary where it crosses a standing
                    # one, but it only knows about its own block, and `fit`
                    # stretches a module past the length the cut was made for.
                    # Modules inside one run abut exactly, so the survivors are
                    # taken as the LONGEST CONTIGUOUS stretch of free ones —
                    # a run with a hole punched in its middle is the defect
                    # this whole pass exists to remove.
                    free = [fence_taken.free(fx, fy, fyaw, mod_len * fit)
                            for (fx, fy, fyaw, fit) in run]
                    if not all(free):
                        n_fence_clash += 1
                        lo = hi = None
                        cur = None
                        for i, ok in enumerate(free + [False]):
                            if ok:
                                cur = (i, i) if cur is None else (cur[0], i)
                            elif cur is not None:
                                if lo is None or cur[1] - cur[0] > hi - lo:
                                    lo, hi = cur
                                cur = None
                        if lo is None:
                            continue
                        run = run[lo:hi + 1]
                    for (fx, fy, fyaw, fit) in run:
                        fence_taken.add(fx, fy, fyaw, mod_len * fit)
                        out.append(pools.place(resolver, uf, "fence", fx, fy,
                                               fyaw + mod_yaw, rng,
                                               scale_mul=fit))
                        n_fence += 1
        for t in p["trees"]:
            cp = (street_pool if t.get("kind") in ("street", "front")
                  else open_pool)
            if not len(cp):
                continue
            u = cp.draw(rng, _size_t(house_idx.nearest(t["c"]),
                                     pc["near_m"], pc["far_m"]))
            # DEFERRED. The paving keep-out needs every parcel's drives and
            # walks, and a parcel's `plan` — which is what decides where its
            # drive actually runs — is only stamped as its houses are placed.
            # Filtering here would test against a half-built map.
            tree_jobs.append((u, t["c"][0], t["c"][1]))
    # PAVING KEEP-OUT, now that every parcel has its houses and their plans.
    # A frontage tree is placed out on the verge on purpose, and a driveway
    # crosses that verge to reach the kerb — so the two collide by
    # construction unless one of them knows about the other.
    pave = _PavingIndex(paving_keepout(parcels),
                        clear_m=float(pc.get("clear_m", 3.0)) * 0.5)
    # AND THE POOLS. `build_open_planting` has taken `pool_rects` as a keep-out
    # from the start; the parcel pass never did, so a frontage or yard tree
    # could be stationed on top of one. `pool_holes` is filled by this same
    # function as the houses are placed, so by here it is complete.
    pool_boxes = [b for b in (_rect_box(r) for r in pool_holes) if b is not None]
    pool_idx = _ObbIndex(pool_boxes, reach=max(20.0, float(pc.get("clear_m", 3.0)) + 1.0))
    n_tree = n_tree_paved = n_tree_pool = 0
    for u, tx, ty in tree_jobs:
        if pave.on_paving((tx, ty)):
            n_tree_paved += 1
            continue
        if pool_boxes and pool_idx.nearest((tx, ty)) <= float(pc.get("clear_m", 3.0)):
            n_tree_pool += 1
            continue
        out.append(pools.place(resolver, u, "tree", tx, ty,
                               rng.uniform(0.0, 360.0), rng))
        n_tree += 1
    print(f"[suburb_scene] parcel trees: {n_tree} planted, "
          f"{n_tree_paved} dropped for landing on a drive or a front walk, "
          f"{n_tree_pool} dropped for landing in a pool")

    print(f"[suburb_scene] lot furniture: {n_gar} garages, "
          f"{n_fence} fence modules "
          f"({n_fence_road} runs dropped off the carriageway, "
          f"{n_fence_clash} shortened off a standing fence)")
    if fence_pool and fence_road is None:
        print("[suburb_scene] WARNING: no street net handed to build_placements"
              " — fences are not being checked against the carriageway")
    return out


# ---------------------------------------------------------------------------
# entry point
# ---------------------------------------------------------------------------

def generate_suburb_on_stage(stage, config,
                             parent_path: str = "/World/stage/generated",
                             scene_scale_factor: float = 1.0,
                             snap_to_ground: bool = False) -> list:
    """Build the graph-based suburb onto a live stage. Returns placements.

    Same shape as `generate_scene.generate_scene_on_stage` so the launch
    scripts are interchangeable.
    """
    if isinstance(config, str):
        config = sg.load_config(config)

    seed = int(config.get("seed", 0))
    rng = random.Random(seed + 7717)

    layout_cfg = dict(config.get("suburb_net") or {})
    region = config.get("layout", {}).get("region_m") or [1600.0, 1200.0]
    w_m, h_m = float(region[0]), float(region[1])

    net, blocks, info = sn.generate(w_m, h_m, rng, layout_cfg)
    stats = sn.stats(net, blocks, info["region"])
    print(f"[suburb_scene] {w_m:.0f} x {h_m:.0f} m  seed {seed}")
    print(sn.format_stats(stats))

    # Undeveloped parcels are land the plat has not built on -- drainage
    # reserve, woodland, or simply not sold yet. suburb_net marks them; skipping
    # them here is what makes that mark mean anything, otherwise the sparseness
    # is invisible because every parcel still gets its row of houses.
    buildable = [b for b in blocks if not b.get("undeveloped")]
    n_open = len(blocks) - len(buildable)

    # THE TURNAROUND IS PAVEMENT THE BLOCK POLYGON DOES NOT KNOW ABOUT.
    # `apply_ground` discs every lollipop end at `bulb_radius_m` while the block
    # boundary runs straight past it, so lots there were sited on the
    # carriageway — 25-43 houses a seed standing in the road. The bulb is the
    # authority on where the road is, so it goes to the parcel pass as a
    # keep-out. The margin is a front yard on top of the paving.
    pcfg = dict(config.get("suburb_parcel") or {})
    bulb_r = float(sn.DEFAULTS["bulb_radius_m"])
    margin = float(pcfg.get("bulb_margin_m", 3.0))
    pcfg.setdefault("keepout_discs",
                    [(e.pts[-1], bulb_r + margin) for e in net.edges.values()
                     if e.street_type == "lollipop"])

    # MEASURE BEFORE SITING. The resolver must exist before the parcel pass:
    # that pass can only stop guessing footprints if it is handed real ones, and
    # the measurement is what folds each garage wing into its parent's.
    resolver = sg._make_resolver(config)
    pools = AssetPools(config)
    yaw_off = float((config.get("suburb_parcel") or {})
                    .get("house_yaw_offset_deg", -90.0))
    # The kit takes precedence when the preset asks for it. Both catalogues
    # publish `(w, d)`, so everything downstream — the lot floor, the packer's
    # overlap test, `house_sizes` — is identical either way.
    catalogue = modular_catalogue(config)
    if catalogue:
        # Before a single prim is written: does every style's roof cover its
        # footprint, and does every style have a front door? Both have been
        # wrong in ways only a render revealed.
        from detail import modular_house as _mh
        _mh.check()
        print(f"[suburb_scene] modular kit: {len(catalogue)} styles, "
              f"{min(e['w'] for e in catalogue):.0f}-"
              f"{max(e['w'] for e in catalogue):.0f} m wide")
    else:
        catalogue = house_catalogue(config, resolver, pools, yaw_off)
    if catalogue and pcfg.get("house_sizes") is None:
        pcfg["house_sizes"] = [(e["w"], e["d"]) for e in catalogue]
        # A LOT NARROWER THAN THE SMALLEST HOUSE CAN NEVER BE BUILT ON. The
        # preset's range is tuned against the pack it ships with, but the pack
        # is measured at run time and nothing stops a future one being wider --
        # and the failure is silent, because a lot that fits no entry is simply
        # refused and the street quietly thins. So the floor is derived from
        # what was actually measured rather than trusted to stay in step.
        #
        # The narrowest lot is the base minimum scaled by the TIGHTEST density
        # class, and it has to clear the smallest house plus the side yard the
        # overlap test enforces (half `house_gap_m`, which is what the fit check
        # uses). Raised, never lowered: a preset asking for wider lots than the
        # art needs is a legitimate choice.
        tight = min(d["lot"] for d in sp.DENSITY.values())
        need = (min(e["w"] for e in catalogue)
                + float(pcfg.get("house_gap_m", 4.0)) * 0.5)
        lw = sp._rng_pair(pcfg.get("lot_width_m", [21.0, 30.0]), (21.0, 30.0))
        floor = need / max(tight, 1e-6)
        if lw[0] < floor:
            pcfg["lot_width_m"] = [floor, max(lw[1], floor * 1.35)]
            print(f"[suburb_scene] lot width floor raised "
                  f"{lw[0]:.1f} -> {floor:.1f} m: the narrowest density class "
                  f"would not fit the smallest measured house "
                  f"({min(e['w'] for e in catalogue):.1f} m)")
    if catalogue and catalogue[0].get("style"):
        # THE PLAT ASKS THE KIT WHERE THE OPENINGS ARE. `suburb_parcel` breaks
        # the front fence for whatever crosses the front lot line; only this
        # module knows which style lands on which lot, so it answers rather
        # than letting the plat guess a position and a side.
        from detail import modular_house as _mh
        _dw = float(pcfg.get("driveway_w_m", 3.0))

        def _front_openings(size_index, house_w):
            spec = _mh.STYLES[catalogue[size_index % len(catalogue)]["style"]]
            door_x, garage_x, _ = _mh.front_anchors(spec)
            gaps = [(door_x, WALK_W_M / 2.0 + 0.6, "door")]
            if garage_x is not None:
                # "drive" also tells the plat this house's garage is IN THE ART,
                # so it must not reserve a detached box beside it.
                gaps.append((garage_x, _dw / 2.0 + 0.6, "drive"))
            return gaps

        pcfg["front_openings"] = _front_openings
    parcels = sp.parcel_blocks(buildable, rng, pcfg)
    pstats = sp.stats(parcels)
    print(f"[suburb_scene] {pstats['houses']} houses, {pstats['trees']} trees "
          f"on {pstats['blocks_built']}/{pstats['blocks']} blocks "
          f"({n_open} left undeveloped)")
    # WHAT THE MEASUREMENT COST. A measured house is bigger than the nominal box
    # that stood in for it, so some lots can no longer take one -- and if the
    # art turns out to be ~16 m across, a 17-26 m lot genuinely cannot fit it.
    # That is a real answer about the plat, not a failure, but it has to be
    # visible rather than silently thinning the streets.
    if pstats.get("size_rejected") or pstats.get("keepout_rejected"):
        print(f"[suburb_scene]   {pstats.get('size_rejected', 0)} lots refused "
              f"for house size, {pstats.get('keepout_rejected', 0)} for "
              f"standing on a cul-de-sac turnaround")

    _pool_holes = []
    # `net` is for the fences and nothing else: a lot line is a straight chord
    # across a block face that curves, so the only authority on where the road
    # actually is happens to be the centrelines. See `_trim_offroad`.
    placements = build_placements(config, resolver, parcels, rng, pools,
                                  yaw_off=yaw_off, catalogue=catalogue,
                                  pool_holes_out=_pool_holes, net=net)
    # -- the park ------------------------------------------------------------
    # suburb_net reserves the ground and frames it with a street; the park's own
    # content is generated here, into that reserve. Generating it separately and
    # hoping the two agree would be the same mistake as the old envelope trick:
    # the reserve is the authority on where the park is, so the park is built to
    # fit it rather than the other way round.
    park = None
    pinfo = info.get("park")
    if pinfo and bool(config.get("park_content", True)):
        px0, py0, px1, py1 = pinfo["rect"]
        pcfg = dict(config.get("suburb_park") or {})
        pcfg["region_m"] = [px1 - px0, py1 - py0]
        park = spk.plan(rng, pcfg)
        # spk.plan works in a region centred on the origin; shift it into place.
        dx = (px0 + px1) / 2.0
        dy = (py0 + py1) / 2.0
        park = _shift_park(park, dx, dy)
        placements += _park_placements(config, resolver, park, rng, pools)
        ps = spk.stats(park)
        print(f"[suburb_scene] park: {pinfo['size'][0]:.0f} x "
              f"{pinfo['size'][1]:.0f} m, {len(pinfo['entrances'])} entrances, "
              f"zones {ps['zones']}")

    # Same discs the parcel pass used, so the two agree on where the
    # pavement is — the yard pass plats inside `lot_corners`, and a lot's
    # corners can overhang a turnaround.
    yard, ystats = yp.plan(config, parcels, rng, resolver=resolver,
                           keepout_discs=pcfg.get("keepout_discs"))
    placements += yard
    yp.report(ystats)
    placements += build_frontage(config, resolver, net, blocks, rng, pools)
    # LAST OF THE PLANTING PASSES, and that ordering is what makes it work: it
    # fills what is left, so it has to be able to see everything that took a
    # piece of ground before it — the lots, the yard planting, the park's own
    # content and the frontage props. `park["rect"]` goes in only when the park
    # was actually built into it; with `park_content: false` that ground is as
    # bare as any other and gets planted like it.
    placements += build_open_planting(
        config, resolver, net, blocks, rng, pools,
        parcels=parcels, info=info, existing=placements,
        pool_rects=_pool_holes,
        park_rect=(pinfo["rect"] if (park is not None and pinfo) else None))
    placements += build_signs(config, resolver, net, rng, pools)
    import collections as _c
    print("[suburb_scene] placements by category: %s"
          % dict(_c.Counter(p["category"] for p in placements)))

    ground_snap = sg._make_physx_ground_snap() if snap_to_ground else None
    # INSTANCE THE REPEATED CATEGORIES. apply_placements does not instance by
    # default -- its docstring and the README both claimed it did, and neither
    # was true, so N copies of a tree cost N x its points. At ~55k points a
    # tree that is ~96M for the street planting alone, against the 89.1M that
    # OOM-killed Isaac Sim on the urban scene.
    #
    # Safe here specifically: instancing forbids editing INSIDE a prim, and the
    # pass that does that (generate_scene.prune_prims) is never called on this
    # path. Houses are left un-instanced so damage variants and per-building
    # edits stay possible.
    sg.apply_placements(stage, placements, parent_path, scene_scale_factor,
                        ground_snap, resolver=resolver,
                        instance_categories=set(config.get(
                            "instance_categories",
                            ["tree", "plant", "sidewalk", "streetlight",
                             "fire_hydrant", "sign", "crosswalk", "fence",
                             "play_structure"])))
    # AFTER placement, because it needs each prim_path. The kit ships exactly
    # ONE wall texture, so without this every modular house is the same cream.
    # A no-op when the kit is off: nothing carries a `palette`.
    if any(q.get("palette") for q in placements):
        from detail import modular_house as mh
        n_pal = mh.apply_palette(stage, placements, parent_path)
        print(f"[suburb_scene] palette: {n_pal} subsets rebound")

    apply_ground(stage, config, net, blocks, parcels, info["region"],
                 parent_path, scene_scale_factor, pool_rects=_pool_holes,
                 park=park)
    return placements
