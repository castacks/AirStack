"""baseline_captures.py -- the FINAL urban-fire baseline review pass: every
photo the user asked for, for ONE scene, written local so they can be
reviewed without opening Kit.

USER DIRECTIVE (quoted): "take a lot of photos. Various districts on fires,
diff angles of it, various buildings on fire, diff angles, a whole overview
of the city top down and also like a perspective one. I also want a photo of
every single human placement from 2 angles at least. This is for every
scene. I want all these photos local after the generator is done so i can
review it. I also want photos of 'groups of people' Which is a group on a
roof or on the face of a building people that are in the windows, etc."

Five families, five subdirs under `snap_dir`:

  overviews/  the whole 1 km plate: one true top-down, two high obliques
              from opposite corners.
  districts/  the burning CLUSTERS (connected components of the manifest's
              own footprints, <=25 m apart -- the same adjacency graph
              `tools/fire_city_union.py`'s `concentration_metrics` already
              runs), 2-3 orbit angles each, framed to the cluster's own
              extent.
  buildings/  per burning building, the existing top+oblique pair
              (`fire_assembly_lib.fire_view_params` + `clear_oblique` --
              already built, REUSED word for word) plus a SECOND oblique at
              azimuth+120 for "diff angles".
  people/     every placed figure, >=2 angles -- the bench closeup framing
              (`fire_people_bench_launch_script.py`'s `_people_closeups`,
              2026-09-01) ported here: a tight portrait at the figure's own
              facing yaw plus a second at yaw+90, except `window` (street-
              level, BOTH angles) and the burial classes
              (`casualty_apron`/`roof_debris`, elevated, BOTH angles) which
              keep the bench's own safer framing on every angle they get.
  groups/     figure GROUPS -- a roof deck, a building's facade windows, an
              apron windrow -- one framed shot per group, camera fitted to
              the group's own AABB + margin.

PURE PLAN, SEPARATE EXECUTION -- BY DESIGN, AND WHY IT MATTERS FOR TESTING.
Every `plan_*` function in this file returns `Shot` namedtuples (`name`,
`eye`, `target`, `focal_mm`, all in real METRES, `ssf` applied only at
execution) computed with nothing but `math` -- no `pxr`, no `carb`, no
`omni`. That is what makes the clustering / framing / naming math in here
unit-testable on a bare host (`scene_gen/tests/test_baseline_captures.py`),
the same discipline `disaster/fire_people.py`'s own docstring describes
("HOST-SIDE, no pxr... the planner touches no stage"). The one exception is
`plan_building_shots`, which needs `fire_assembly_lib.fire_view_params`/
`clear_oblique` (REUSED, not reimplemented, per the work order) -- that
import is LAZY, inside the function, so importing THIS module never touches
`pxr` even though `fire_assembly_lib` itself does. `run_capture_plan` (and
everything below it) is the execution half: it needs a live `stage` and the
Isaac `snapshots` module (`carb`/`omni.kit.app`), so it only ever runs
inside Kit.

WIRING (the launcher's own job, not this module's):

    from disaster import baseline_captures as bc
    obs = getattr(self, "street_positions", None) or {}
    bc.capture_baseline(self.stage, self._snaps(), self.placed,
                        getattr(self, "people_records_final", []) or [],
                        SNAP_DIR, max(self.region), obstacles=obs,
                        ssf=self.ssf)

`self.placed` is `urban_fire_city_launch_script.py`'s own per-building list
(`i`, `x`, `y`, `bbox`, `doc`, `masses`, `stem`, `rec`); `people_records_final`
is whatever `place_people()` actually authored (see that method's own
docstring for why it has to be the AUTHORED list, not the raw dry-run file --
a record the placement pass refused was never put on the stage and a photo
of it is a photo of nothing); `street_positions` is the SAME `{i: {"x", "y",
"W", "D", "H", "yaw"}}` dict the existing per-building oblique already uses
to clear itself of a neighbour.
"""

import math
import os
from collections import namedtuple

from . import urban_fire_spread as ufs

# ---------------------------------------------------------------------------
# The plan unit
# ---------------------------------------------------------------------------
#: `name` is a RELATIVE PATH (no extension) already carrying its subdir --
#: "overviews/city_top", "people/window_014_a090" -- so the execution loop
#: only ever does `os.path.join(snap_dir, shot.name + ".png")`. `eye`/
#: `target` are `(x, y, z)` metre tuples, the same shape `snapshots.
#: place_camera` takes; `focal_mm` is always a concrete float (never `None`)
#: so the execution loop never has to special-case a missing lens.
Shot = namedtuple("Shot", ["name", "eye", "target", "focal_mm"])

#: how long one capture costs, wall clock -- `snapshots.snapshot`'s own
#: `frames=40` app-update spin plus the render's own convergence time,
#: measured loosely across every bench/row/city capture pass so far. Used
#: only to print an ETA range before the (possibly hundreds-of-shots) loop
#: runs; never fed back into anything that changes the plan.
SEC_PER_SHOT_LO, SEC_PER_SHOT_HI = 2.0, 4.0

DEFAULT_FRAMES = 40


# ===========================================================================
# 0) shared geometry helpers
# ===========================================================================
def connected_components(items, gap_fn, radius_m):
    """Union-find over `items` (any indexable sequence); `gap_fn(items[i],
    items[j])` is the clear distance between two items (metres, 0 if they
    touch/overlap). Two items whose gap is `<= radius_m` end up in the same
    component. Returns a list of INDEX lists into `items`, biggest first --
    the exact shape `tools/fire_city_union.py`'s `concentration_metrics`
    already builds for its own "how concentrated is this union" report (that
    tool's own docstring: "connected components of that same <=25 m
    adjacency graph"); this is the same algorithm, lifted out so a second
    caller (districts, here) does not reimplement it.
    """
    n = len(items)
    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for i in range(n):
        for j in range(i + 1, n):
            if gap_fn(items[i], items[j]) <= radius_m:
                union(i, j)
    groups = {}
    for k in range(n):
        groups.setdefault(find(k), []).append(k)
    return sorted(groups.values(), key=len, reverse=True)


def _mean_bearing_deg(yaws_deg, fallback=225.0):
    """Circular mean of a list of bearings (degrees) -- `fallback` (the same
    south-west default `snapshots.views_around` uses) if the list is empty
    or every bearing cancels out (e.g. two figures facing exactly opposite
    ways)."""
    if not yaws_deg:
        return fallback
    sx = sum(math.cos(math.radians(float(y))) for y in yaws_deg)
    sy = sum(math.sin(math.radians(float(y))) for y in yaws_deg)
    if abs(sx) < 1e-9 and abs(sy) < 1e-9:
        return fallback
    return math.degrees(math.atan2(sy, sx))


def assert_unique_names(shots):
    """Raise `ValueError` naming the first collision if two `shots` share a
    `.name` -- a silent overwrite is the one failure mode a capture pass can
    have that leaves no trace in the log (the second write just clobbers the
    first PNG, and nothing prints). Returns `True` otherwise."""
    seen = set()
    for s in shots:
        if s.name in seen:
            raise ValueError("duplicate capture name: {0}".format(s.name))
        seen.add(s.name)
    return True


# ===========================================================================
# 1) OVERVIEWS -- the whole plate, top-down + two corner obliques
# ===========================================================================
#: same 0.95 standoff `snapshots.overview` already uses to fit a whole
#: `span_m` square through an 18 mm lens straight down.
OVERVIEW_TOP_FRAC = 0.95
#: opposite corners -- 225 is `views_around`'s own default south-west
#: bearing (keeps the sun on the subject under the default sky); 45 is its
#: exact opposite, north-east.
OVERVIEW_CORNER_AZIMUTHS_DEG = (45.0, 225.0)
#: camera stands just outside the plate's own footprint along the diagonal.
OVERVIEW_OBLIQUE_DIST_FRAC = 0.95
#: high enough that the far corner clears any roofline between it and the
#: camera without climbing all the way to a second top-down.
OVERVIEW_OBLIQUE_H_FRAC = 0.62
OVERVIEW_AIM_H_FRAC = 0.02
OVERVIEW_AIM_H_MIN_M = 2.0
OVERVIEW_FOCAL_MM = 18.0


#: Capture aspect (w, h). The viewport path renders 1280x720 -- MEASURED on
#: the shipped cells -- and `place_camera` authors only the HORIZONTAL
#: aperture, so the vertical field is narrower by 9/16. The 0.95 standoff
#: above was solved for the horizontal axis alone, which is why a SQUARE
#: plate does not fit: at h = 0.95*span the horizontal ground coverage is
#: 1.106*span but the vertical is ~0.62*span. Framing has to be solved
#: against the SMALLER of the two half-angles.
OVERVIEW_ASPECT = (1280.0, 720.0)
#: 35 mm horizontal aperture `snapshots.place_camera` sets.
OVERVIEW_APERTURE_MM = 20.955
#: Margin so the plate does not touch the frame edge.
OVERVIEW_MARGIN = 1.06


def overview_height_m(span_m, focal_mm=OVERVIEW_FOCAL_MM,
                      aspect=OVERVIEW_ASPECT,
                      aperture_mm=OVERVIEW_APERTURE_MM,
                      margin=OVERVIEW_MARGIN):
    """Camera height that fits a *span_m* SQUARE in frame, both axes.

    `half_fov = atan(aperture / (2 * focal))` horizontally; the vertical
    half-angle is that scaled by the frame aspect. The plate is square, so
    the binding constraint is whichever half-angle is smaller -- the
    vertical one on any wider-than-tall capture. Returns the height at which
    the span subtends that angle, times a margin.

    At 1280x720, 18 mm: horizontal half-FOV 30.2 deg, vertical 17.8 deg, so
    this returns ~1.65*span where the old rule returned 0.95*span. That is
    the whole bug -- the shipped `overviews/city_top` was framing roughly
    the middle 60 % of the cell and calling it the city.
    """
    half_h = math.atan(float(aperture_mm) / (2.0 * float(focal_mm)))
    half_v = math.atan(math.tan(half_h) * float(aspect[1]) / float(aspect[0]))
    half_min = min(half_h, half_v)
    return float(margin) * (float(span_m) / 2.0) / math.tan(half_min)


def plan_overviews(region_span_m, centre=(0.0, 0.0)):
    """3 shots: `overviews/city_top` (true top-down) and `overviews/
    city_corner_ne` / `overviews/city_corner_sw` (high obliques from
    opposite corners) -- the "whole overview of the city top down and also
    like a perspective one" from the user directive, doubled to opposite
    corners so a fire in either half of the plate is never the far, hazy
    side of the one oblique shot."""
    span = float(region_span_m)
    cx, cy = float(centre[0]), float(centre[1])
    # CENTRE, not (0, 0). A cell whose content is not centred on the stage
    # origin -- any cropped cell, and any cell whose corridor sits off to one
    # side -- gets an overview of the wrong patch of ground otherwise. The
    # legacy shot in `urban_fire_city_launch_script` already did this; the
    # `capture_baseline` path dropped the centre and kept only the span.
    top_h = overview_height_m(span)
    shots = [Shot("overviews/city_top", (cx, cy, top_h),
                  (cx, cy, 0.0), OVERVIEW_FOCAL_MM)]
    dist = span * OVERVIEW_OBLIQUE_DIST_FRAC
    h = span * OVERVIEW_OBLIQUE_H_FRAC
    aim_h = max(OVERVIEW_AIM_H_MIN_M, span * OVERVIEW_AIM_H_FRAC)
    for tag, az in zip(("corner_ne", "corner_sw"), OVERVIEW_CORNER_AZIMUTHS_DEG):
        a = math.radians(az)
        eye = (cx + dist * math.cos(a), cy + dist * math.sin(a), h)
        shots.append(Shot("overviews/city_{0}".format(tag), eye,
                          (cx, cy, aim_h), OVERVIEW_FOCAL_MM))
    return shots


# ===========================================================================
# 1b) BLOCKS -- one true top-down per CITY BLOCK
# ===========================================================================
#: The `districts/` family below shoots BURNING CLUSTERS, not city blocks --
#: connected components over the composed bakes, which on a one-corridor fire
#: is a single component and therefore THREE frames for the whole cell. That
#: leaves every unburnt block, and every block the fire only clipped,
#: unphotographed. This family is the missing half: every block in the
#: layout gets its own plumb top-down, so a reviewer can check zoning,
#: density, frontage and damage block by block instead of inferring it from
#: three obliques.
BLOCK_FOCAL_MM = 24.0
#: Framed on the block's LONG side plus a margin, so the surrounding streets
#: are visible and the block reads in context rather than as a floating slab.
BLOCK_MARGIN = 1.35
#: Never closer than this, or a small block gives a camera inside a building.
BLOCK_MIN_H_M = 90.0


def plan_block_shots(blocks, focal_mm=BLOCK_FOCAL_MM):
    """One plumb top-down per block.

    *blocks* is `[(x0, y0, x1, y1, name), ...]` or `[(rect, name), ...]` or
    plain rects -- whatever the caller has. The typology name, when present,
    goes into the filename so a contact sheet sorts by district.

    Height is solved with the same `overview_height_m` rule the overview
    uses, against the block's long side, so a block fills the frame the same
    way whatever its size -- the thing that makes a contact sheet
    comparable.
    """
    shots = []
    for i, entry in enumerate(blocks or []):
        name = None
        if isinstance(entry, (list, tuple)) and len(entry) == 2 \
                and isinstance(entry[0], (list, tuple)):
            rect, name = entry
        elif isinstance(entry, (list, tuple)) and len(entry) >= 5:
            rect, name = entry[:4], entry[4]
        else:
            rect = entry
        x0, y0, x1, y1 = (float(v) for v in rect[:4])
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        span = max(x1 - x0, y1 - y0) * BLOCK_MARGIN
        h = max(BLOCK_MIN_H_M, overview_height_m(span, focal_mm=focal_mm))
        tag = "%03d" % i
        if name:
            tag += "_%s" % str(name)
        shots.append(Shot("blocks/b%s" % tag, (cx, cy, h), (cx, cy, 0.0),
                          focal_mm))
    return shots


# ===========================================================================
# 2) DISTRICTS -- burning clusters, 2-3 orbit angles each
# ===========================================================================
#: same default `tools/fire_city_union.py`'s `concentration_metrics` uses
#: for "is this building part of the same burning district as that one".
DISTRICT_ADJACENCY_M = 25.0
DISTRICT_FOCAL_MM = 24.0
DISTRICT_MIN_DIST_M = 90.0
DISTRICT_DIST_SPAN_MULT = 1.15
DISTRICT_DIST_HEIGHT_MULT = 1.5
DISTRICT_MIN_OBL_H_M = 35.0
DISTRICT_OBL_H_SPAN_MULT = 0.22
DISTRICT_OBL_H_HEIGHT_MULT = 0.5
DISTRICT_AIM_H_FRAC = 0.35
DISTRICT_AIM_H_MIN_M = 3.0
#: base azimuth for cluster #1's orbit; every later cluster's own base is
#: rotated off this by `17 * cluster_index` deg so a whole city's worth of
#: district shots do not all stare from the identical relative bearing --
#: deterministic (seeded off the cluster's own rank), never random.
DISTRICT_BASE_AZIMUTH_DEG = 40.0
DISTRICT_BASE_AZIMUTH_STEP_DEG = 17.0


def _footprint(rec):
    """Normalize one "placed building" entry into the flat `{"x", "y", "W",
    "D", "yaw", "H"}` `urban_fire_spread.gap_m`/`_corners` need. Accepts
    either the launcher's own `self.placed` shape (`rec["x"]`/`rec["y"]`
    already resolved, `rec["rec"]` the manifest record carrying `W`/`D`/
    `yaw_deg`, `rec["bbox"]` the composed world bbox) or a flat synthetic
    dict (`x`, `y`, `W`, `D`, `yaw`/`yaw_deg`, `H`) a test can build with no
    launcher context at all."""
    geo = rec.get("rec") if isinstance(rec.get("rec"), dict) else rec
    x = float(rec.get("x", geo.get("x", 0.0)))
    y = float(rec.get("y", geo.get("y", 0.0)))
    W = float(geo.get("W") or rec.get("W") or 1.0)
    D = float(geo.get("D") or rec.get("D") or 1.0)
    yaw = float(rec.get("yaw", geo.get("yaw_deg", rec.get("yaw_deg", 0.0))) or 0.0)
    b = rec.get("bbox")
    if b:
        H = float(b[5]) - float(b[2])
    else:
        H = float(geo.get("H") or rec.get("H") or 0.0)
    return {"x": x, "y": y, "W": W, "D": D, "yaw": yaw, "H": H}


def cluster_buildings(placed, adjacency_m=DISTRICT_ADJACENCY_M):
    """`[[placed-entry, ...], ...]` -- connected components of `placed` at
    `<= adjacency_m` footprint-to-footprint gap (`urban_fire_spread.gap_m`,
    the real edge-to-edge distance, not centre-to-centre -- two 40 m blocks
    2 m apart read as adjacent, exactly `gap_m`'s own rationale), biggest
    first. Every burning building lands in SOME component, including a
    lone one far from every other fire (a component of size 1) -- an
    isolated fire is still a district, just a small one."""
    foots = [_footprint(r) for r in placed]
    comps = connected_components(foots, ufs.gap_m, adjacency_m)
    return [[placed[i] for i in comp] for comp in comps]


def district_orbit_azimuths(n_buildings, base_deg=DISTRICT_BASE_AZIMUTH_DEG):
    """2 angles for a 1-2 building cluster, 3 for anything bigger -- "2-3
    angles (orbit azimuths)" per the work order -- evenly spaced starting
    at `base_deg`."""
    n = 2 if n_buildings <= 2 else 3
    step = 360.0 / n
    return [round((base_deg + k * step) % 360.0, 1) for k in range(n)]


def cluster_footprint_aabb(foots):
    """`(x0, y0, x1, y1)` enclosing every building's own ROTATED footprint
    corners in the cluster (`urban_fire_spread._corners` -- the same
    rotation `gap_m` itself measures against), not an axis-aligned W/D box
    that would under- or over-state a yawed building's true extent."""
    xs, ys = [], []
    for f in foots:
        for cx, cy in ufs._corners(f):
            xs.append(cx)
            ys.append(cy)
    return (min(xs), min(ys), max(xs), max(ys))


def district_view_params(aabb, max_height_m):
    """`{"cx", "cy", "dist", "obl_h", "aim_h"}` for one cluster -- in the
    SPIRIT of `fire_assembly_lib.fire_view_params`'s own `max(floor, ...)`
    standoff arithmetic, scaled for a multi-building footprint rather than
    one building's sidecar (this is a new formula, not a literal reuse --
    `fire_view_params` needs a bake's `doc`/`masses`, which a cluster does
    not have)."""
    x0, y0, x1, y1 = aabb
    span = max(x1 - x0, y1 - y0, 10.0)
    H = max(0.0, float(max_height_m))
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    dist = max(DISTRICT_MIN_DIST_M, DISTRICT_DIST_SPAN_MULT * span,
               DISTRICT_DIST_HEIGHT_MULT * H)
    obl_h = max(DISTRICT_MIN_OBL_H_M, DISTRICT_OBL_H_SPAN_MULT * span,
                DISTRICT_OBL_H_HEIGHT_MULT * H)
    aim_h = max(DISTRICT_AIM_H_MIN_M, DISTRICT_AIM_H_FRAC * H)
    return {"cx": cx, "cy": cy, "dist": dist, "obl_h": obl_h, "aim_h": aim_h}


def plan_district_shots(placed, adjacency_m=DISTRICT_ADJACENCY_M,
                        base_deg=DISTRICT_BASE_AZIMUTH_DEG):
    """Every burning-building cluster, 2-3 occlusion-clearable orbit shots
    each, named `districts/d{NN}_n{building count}_a{azimuth}`."""
    clusters = cluster_buildings(placed, adjacency_m)
    shots = []
    for idx, cluster in enumerate(clusters, start=1):
        foots = [_footprint(r) for r in cluster]
        aabb = cluster_footprint_aabb(foots)
        max_h = max((f["H"] for f in foots), default=0.0)
        vp = district_view_params(aabb, max_h)
        azs = district_orbit_azimuths(
            len(cluster), base_deg=base_deg + DISTRICT_BASE_AZIMUTH_STEP_DEG * idx)
        for az in azs:
            a = math.radians(az)
            eye = (vp["cx"] + vp["dist"] * math.cos(a),
                   vp["cy"] + vp["dist"] * math.sin(a), vp["obl_h"])
            target = (vp["cx"], vp["cy"], vp["aim_h"])
            name = "districts/d{0:02d}_n{1}_a{2:03.0f}".format(
                idx, len(cluster), az)
            shots.append(Shot(name, eye, target, DISTRICT_FOCAL_MM))
    return shots


# ===========================================================================
# 3) BUILDINGS -- per-building top+oblique pair (existing), + azimuth+120
# ===========================================================================
def building_tag(i, stem, level):
    """`d{i}_{stem}_{level}` -- the exact tag
    `urban_fire_city_launch_script.capture()`'s own per-building loop
    already builds (`"d{0}_{1}_{2}".format(r["i"], ..., level)`), pulled out
    so it is testable on its own and shared with the second oblique."""
    parts = ["d{0}".format(i), str(stem or "bldg"), str(level or "")]
    return "_".join(p for p in parts if p)


def second_oblique_azimuth(az_deg):
    """The "diff angle" oblique's bearing: the fire-facing azimuth rotated
    +120 deg, wrapped to `[0, 360)`."""
    return (float(az_deg) + 120.0) % 360.0


def plan_building_shots(placed, obstacles=None):
    """Per burning building: the top+oblique pair `fire_assembly_lib.
    fire_view_params` + `clear_oblique` already produce (REUSED, word for
    word -- this is the "existing views_around top+oblique pair with
    clear_oblique" the work order calls "already satisfied"), plus a SECOND
    oblique at `second_oblique_azimuth`, independently pushed clear of the
    same obstacles.

    `obstacles`, if given, is `{i: {"x","y","W","D","H","yaw"}}` -- the same
    shape `fire_assembly_lib.load_dump_positions` / the launcher's own
    `street_positions` already carry -- keyed by manifest index so a
    building's own footprint never blocks its own camera (this function
    excludes it by `i`, exactly like the existing `capture()` loop's `obs =
    [... if i2 != r["rec"].get("i")]`).

    LAZILY imports `fire_assembly_lib` (which imports `pxr`) -- see the
    module docstring for why nothing above this function does. Every OTHER
    plan function in this file is pure and needs no import at all.
    """
    from . import fire_assembly_lib as fal

    shots = []
    for r in placed:
        b = r.get("bbox")
        if not b:
            continue
        i = r.get("i")
        doc = r.get("doc") or {}
        stem = r.get("stem") or doc.get("name")
        tag = building_tag(i, stem, doc.get("level", ""))
        x, y = float(r["x"]), float(r["y"])
        vp = fal.fire_view_params(doc, r.get("masses"), b)
        obs = [o for i2, o in (obstacles or {}).items() if i2 != i]
        shots.append(Shot("buildings/{0}_top".format(tag),
                          (x, y, vp["top_h"]), (x, y, 0.0), OVERVIEW_FOCAL_MM))
        vp1 = vp
        if obs:
            vp1, _ = fal.clear_oblique(vp1, x, y, obs)
        shots.append(_building_oblique_shot(tag, "obl000", x, y, vp1))
        vp2 = dict(vp)
        vp2["azimuth_deg"] = second_oblique_azimuth(vp["azimuth_deg"])
        if obs:
            vp2, _ = fal.clear_oblique(vp2, x, y, obs)
        shots.append(_building_oblique_shot(tag, "obl120", x, y, vp2))
    return shots


def _building_oblique_shot(tag, suffix, x, y, vp):
    a = math.radians(vp["azimuth_deg"])
    eye = (x + vp["obl_dist"] * math.cos(a), y + vp["obl_dist"] * math.sin(a),
           vp["obl_h"])
    return Shot("buildings/{0}_{1}".format(tag, suffix), eye,
               (x, y, vp["aim_h"]), OVERVIEW_FOCAL_MM)


# ===========================================================================
# 4) PEOPLE -- every figure, >=2 angles
# ===========================================================================
# Ported from `fire_people_bench_launch_script.py`'s `_people_closeups`
# (2026-09-01, bench-v4/v5 feedback) -- see that function's own comments for
# the failure each branch fixes: a level ground-height shot put the burial
# classes' camera eye INSIDE the covering debris about half the time
# (`yaw_deg` on those records is a random lie-down direction, not a facing
# side to stand clear of), and a tight portrait crops the wall out of a
# window closeup, which is exactly the context an occlusion/recession audit
# needs. Values match the bench file's own tuned constants.
_STAND_AIM_M = 1.15
_PRONE_AIM_M = 0.30
_TIGHT_DIST_M = 5.0
_TIGHT_FOCAL_MM = 50.0
_WINDOW_DIST_M = 12.0
_WINDOW_EYE_M = 1.6
_WINDOW_FOCAL_MM = 35.0
_BURIAL_CLASSES = ("casualty_apron", "roof_debris")
_BURIAL_ELEV_DEG = 35.0
_BURIAL_DIST_M = 7.0
#: "a photo of every single human placement from 2 angles at least": the
#: figure's own facing yaw, and yaw+90.
PEOPLE_ANGLE_OFFSETS_DEG = (0.0, 90.0)


def person_shot_params(rec, offset_deg):
    """`(eye, target, focal_mm)` for one person record, framed at
    `offset_deg` off its own recorded `yaw_deg` -- `window` always gets the
    STREET-LEVEL framing (task: "window class uses the street-level framing
    for BOTH" angles), the two burial classes always get the ELEVATED
    framing (safe on any bearing, unlike a level shot which is only safe
    facing away from the windrow -- a random `yaw_deg` on those records
    means one of the two default angles would otherwise be a coin flip on
    planting the camera in the debris), and every other class gets the
    bench's plain tight portrait.
    """
    x, y, z = float(rec["x"]), float(rec["y"]), float(rec["z"])
    yaw = math.radians(float(rec.get("yaw_deg", 0.0)) + float(offset_deg))
    cls = str(rec.get("cls", "unknown"))
    aim = _PRONE_AIM_M if rec.get("prone") else _STAND_AIM_M
    tz = z + aim
    if cls == "window":
        ex = x + _WINDOW_DIST_M * math.cos(yaw)
        ey = y + _WINDOW_DIST_M * math.sin(yaw)
        return (ex, ey, _WINDOW_EYE_M), (x, y, tz), _WINDOW_FOCAL_MM
    if cls in _BURIAL_CLASSES:
        ce = math.radians(_BURIAL_ELEV_DEG)
        horiz, rise = _BURIAL_DIST_M * math.cos(ce), _BURIAL_DIST_M * math.sin(ce)
        ex, ey = x + horiz * math.cos(yaw), y + horiz * math.sin(yaw)
        return (ex, ey, tz + rise), (x, y, tz), _TIGHT_FOCAL_MM
    ex, ey = x + _TIGHT_DIST_M * math.cos(yaw), y + _TIGHT_DIST_M * math.sin(yaw)
    return (ex, ey, tz), (x, y, tz), _TIGHT_FOCAL_MM


def person_shot_name(rec, fallback_id):
    """`people/<cls>_<id>` -- `<angle>` is appended by `plan_people_shots`.
    `fallback_id` (the record's position in whatever list the caller is
    iterating) is used only when the record itself carries no `id`, so two
    id-less records never collide."""
    cls = str(rec.get("cls", "unknown"))
    rid = rec.get("id")
    if rid is None:
        rid = fallback_id
    return "people/{0}_{1}".format(cls, rid)


def plan_people_shots(records, offsets_deg=PEOPLE_ANGLE_OFFSETS_DEG):
    """`Shot` list, `len(offsets_deg)` (2 by default) per record, named
    `<base>_a<offset:03.0f>`."""
    shots = []
    for idx, rec in enumerate(records):
        base = person_shot_name(rec, idx)
        for off in offsets_deg:
            eye, target, focal = person_shot_params(rec, off)
            name = "{0}_a{1:03.0f}".format(base, off % 360.0)
            shots.append(Shot(name, eye, target, focal))
    return shots


# ===========================================================================
# 5) GROUPS -- roof decks, facade window runs, apron windrows
# ===========================================================================
# `fire_people.py` already clusters most of its own classes at GENERATION
# time: `plan.next_group()` is called ONCE per roof deck / street knot /
# apron windrow and the returned `gid` is shared by every member
# (`fire_people.py`'s own `_street_group`/roof/`_burial_record` call sites),
# so `group` alone is already the right key for every class EXCEPT
# `window` -- a window figure gets its OWN `group` id every time
# (`plan.next_group()` called per-record inside the window loops), because
# window figures are spread across separate openings/storeys, not seeded as
# a cluster. "The face of a building people that are in the windows" is
# still a real spatial group, just not one `fire_people.py` labelled as
# one -- `(building_i, side)` is exactly that facade, and every window
# record already carries both fields.
GROUP_MIN_SIZE = 2
GROUP_AABB_MARGIN_M = 2.5
#: vertical pad above the tallest figure's own z in the group -- head room,
#: not a real measurement (people records only ever carry a support-surface
#: z, see `fire_people.py`'s own account of what `z` means on every class).
GROUP_HEAD_ROOM_M = 2.0
GROUP_FOCAL_MM = 35.0
GROUP_ELEV_DEG = 25.0
GROUP_DIST_MULT = 1.6
GROUP_MIN_DIST_M = 9.0


def group_key(rec):
    """The clustering key for one people record -- see the section
    docstring above. `None` for a record that cannot be keyed at all (no
    `group` and, for a window, no `side`/`building_i`) -- such a record
    joins no group rather than crashing the pass."""
    cls = str(rec.get("cls", ""))
    bi = rec.get("building_i")
    if cls == "window":
        side = rec.get("side")
        if bi is None or side is None:
            return None
        return ("window", bi, side)
    gid = rec.get("group")
    if gid is None:
        return None
    return (cls, gid)


def cluster_people(records):
    """`{key: [record, ...]}` via `group_key`, kept only for keys with
    `>= GROUP_MIN_SIZE` members -- a lone figure is not a group shot, the
    same "no singleton" rule `fire_people.py`'s own roof/roof_victim
    classes already enforce at generation time (a group of one is withdrawn
    there and its budget returned to the pool)."""
    by_key = {}
    for r in records:
        k = group_key(r)
        if k is None:
            continue
        by_key.setdefault(k, []).append(r)
    return {k: v for k, v in by_key.items() if len(v) >= GROUP_MIN_SIZE}


def group_aabb(records):
    """`(x0, y0, z0, x1, y1, z1)` over a group's own recorded positions."""
    xs = [float(r["x"]) for r in records]
    ys = [float(r["y"]) for r in records]
    zs = [float(r["z"]) for r in records]
    return (min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))


def group_azimuth_deg(records, fallback=225.0):
    """The camera stands where the group itself is looking -- the same
    convention the bench closeup uses for one figure
    (`fire_people_bench_launch_script._people_closeups`: "STANDS WHERE THE
    FIGURE ITSELF IS LOOKING"), applied to the group's own circular MEAN
    facing bearing so the shot reads faces, not backs of heads."""
    yaws = [r.get("yaw_deg") for r in records if r.get("yaw_deg") is not None]
    return _mean_bearing_deg([float(y) for y in yaws], fallback=fallback)


def group_eye_target(aabb, azimuth_deg):
    """`(eye, target)` for one group: an oblique at `GROUP_ELEV_DEG` above
    the horizontal, standing off far enough (`GROUP_DIST_MULT` x the
    padded footprint span) to hold the whole `aabb` + margin in frame."""
    x0, y0, z0, x1, y1, z1 = aabb
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    W = (x1 - x0) + 2.0 * GROUP_AABB_MARGIN_M
    D = (y1 - y0) + 2.0 * GROUP_AABB_MARGIN_M
    span = max(W, D, 3.0)
    aim_z = 0.5 * (z0 + z1) + 0.3 * GROUP_HEAD_ROOM_M
    dist = max(GROUP_MIN_DIST_M, GROUP_DIST_MULT * span)
    elev = math.radians(GROUP_ELEV_DEG)
    horiz, rise = dist * math.cos(elev), dist * math.sin(elev)
    a = math.radians(azimuth_deg)
    eye = (cx + horiz * math.cos(a), cy + horiz * math.sin(a), aim_z + rise)
    return eye, (cx, cy, aim_z)


def _group_sort_key(key):
    """A homogeneous sort key over the mixed `("window", bi, side)` /
    `(cls, gid)` shapes `group_key` produces -- every field coerced to a
    single type before comparison so two keys with different arities/types
    at the same position never raise."""
    return (str(key[0]), key[1] if isinstance(key[1], int) else 0,
           str(key[2]) if len(key) > 2 else "")


def plan_group_shots(records, fallback_azimuth=225.0):
    """One `Shot` per kept cluster (`cluster_people`). `window` clusters are
    named by their own stable facade tag (`group_windows_b<building>_
    <side>`, matching the work order's own example); every other class gets
    a per-`(building_i, cls)` SEQUENCE NUMBER (`group_<cls>_b<building>_
    01`, `_02`, ... -- matching `group_roof_<building>_01`), assigned in a
    deterministic (sorted-key) order so re-running the same records always
    numbers the same way.
    """
    clusters = cluster_people(records)
    shots = []
    seq = {}
    for key in sorted(clusters.keys(), key=_group_sort_key):
        recs = clusters[key]
        if key[0] == "window":
            _cls, bi, side = key
            name = "groups/group_windows_b{0}_{1}".format(bi, side)
        else:
            cls = key[0]
            bi = recs[0].get("building_i", "x")
            seq_key = (bi, cls)
            seq[seq_key] = seq.get(seq_key, 0) + 1
            name = "groups/group_{0}_b{1}_{2:02d}".format(cls, bi, seq[seq_key])
        aabb = group_aabb(recs)
        az = group_azimuth_deg(recs, fallback_azimuth)
        eye, target = group_eye_target(aabb, az)
        shots.append(Shot(name, eye, target, GROUP_FOCAL_MM))
    return shots


# ===========================================================================
# 6) sightline clearance for the macro families (overviews/districts/groups)
# ===========================================================================
def clear_shot_sightline(shot, obstacles):
    """Raise `shot`'s eye height, along its own eye-to-target line, until
    the sightline clears every obstacle footprint it crosses --
    `fire_assembly_lib.raise_over_sightline`'s exact algorithm (REUSED,
    lazily imported -- see the module docstring), applied to a `Shot`
    rather than to `fire_view_params`' own `vp` dict. `obstacles` empty/
    falsy: `shot` returned UNCHANGED and nothing is imported at all -- the
    same short-circuit `urban_fire_city_launch_script.capture()`'s own `if
    obs: ...` already uses.
    """
    if not obstacles:
        return shot
    from . import fire_assembly_lib as fal

    ex, ey, eh = shot.eye
    tx, ty, tz = shot.target
    dist = math.hypot(ex - tx, ey - ty)
    if dist < 1e-6:
        return shot
    az = math.degrees(math.atan2(ey - ty, ex - tx))
    new_h = fal.raise_over_sightline(tx, ty, tz, dist, az, eh, obstacles)
    if new_h <= eh + 1e-6:
        return shot
    return shot._replace(eye=(ex, ey, new_h))


def apply_sightline_clearance(shots, obstacles):
    """`clear_shot_sightline` over a whole shot list."""
    return [clear_shot_sightline(s, obstacles) for s in shots]


# ===========================================================================
# 7) the plan, assembled; the estimate; the execution loop
# ===========================================================================
def build_capture_plan(region_span_m, placed, people, obstacles=None,
                       adjacency_m=DISTRICT_ADJACENCY_M, centre=(0.0, 0.0),
                       blocks=None):
    """The whole plan: all 5 families, sightline-cleared where a helper
    exists for it, plus `"_all"` (every family concatenated, in the order
    the work order lists them, name-uniqueness already checked -- raises if
    not).

    `obstacles`, if given, is `{i: {"x","y","W","D","H","yaw"}}` (see
    `plan_building_shots`); a flat `list(obstacles.values())` clears the
    overview/district/group families, the dict itself (so a building can
    exclude ITSELF by index) goes to `plan_building_shots`.

    Needs `fire_assembly_lib` (hence `pxr`) for the BUILDINGS family only --
    every other family is computed first, with plain math.
    """
    flat_obs = list((obstacles or {}).values())
    families = {
        "overviews": plan_overviews(region_span_m, centre=centre),
        "blocks": plan_block_shots(blocks),
        "districts": plan_district_shots(placed, adjacency_m),
        "people": plan_people_shots(people),
        "groups": plan_group_shots(people),
    }
    if flat_obs:
        for fam in ("overviews", "districts", "groups"):
            families[fam] = apply_sightline_clearance(families[fam], flat_obs)
    families["buildings"] = plan_building_shots(placed, obstacles)
    all_shots = []
    for fam in ("overviews", "blocks", "districts", "buildings", "people",
                "groups"):
        all_shots.extend(families[fam])
    assert_unique_names(all_shots)
    families["_all"] = all_shots
    return families


def plan_stats(families):
    """Per-family counts + `"total"` + a `SEC_PER_SHOT_LO`/`_HI` wall-clock
    range, in seconds."""
    counts = {k: len(v) for k, v in families.items() if not k.startswith("_")}
    total = sum(counts.values())
    counts["total"] = total
    counts["est_wall_clock_s_lo"] = total * SEC_PER_SHOT_LO
    counts["est_wall_clock_s_hi"] = total * SEC_PER_SHOT_HI
    return counts


def print_plan_summary(families, prefix="bc"):
    """Print the per-family/total shot count and an ETA range BEFORE the
    (possibly hundreds-of-shots) capture loop runs -- "the wall-clock at
    ~2-4 s/shot matters" per the work order. Returns `plan_stats`'s own
    dict."""
    counts = plan_stats(families)
    order = ("overviews", "districts", "buildings", "people", "groups")
    print("[{0}] capture plan: {1} shot(s) total ({2})".format(
        prefix, counts["total"],
        ", ".join("{0}={1}".format(k, counts[k]) for k in order if k in counts)))
    lo_m = counts["est_wall_clock_s_lo"] / 60.0
    hi_m = counts["est_wall_clock_s_hi"] / 60.0
    print("[{0}] estimated wall clock at {1:.0f}-{2:.0f} s/shot: "
          "{3:.0f}-{4:.0f} min".format(prefix, SEC_PER_SHOT_LO,
                                       SEC_PER_SHOT_HI, lo_m, hi_m))
    return counts


def run_capture_plan(stage, snaps, families, snap_dir, ssf=1.0,
                     frames=DEFAULT_FRAMES, prefix="bc"):
    """Execute every `Shot` in `families["_all"]` -- `snaps.place_camera` +
    `snaps.snapshot`, `ssf` applied HERE (every plan function above works in
    real metres, see the module docstring, so one plan is identical
    regardless of which scene's scale factor executes it). Prints the
    up-front estimate, then one line per family as it starts so a long run
    is legible in the pane.
    """
    os.makedirs(snap_dir, exist_ok=True)
    print_plan_summary(families, prefix=prefix)
    order = ("overviews", "districts", "buildings", "people", "groups")
    n_done = 0
    for fam in order:
        shots = families.get(fam) or []
        if not shots:
            continue
        print("[{0}] {1}: {2} shot(s)".format(prefix, fam, len(shots)))
        for s in shots:
            eye = tuple(v * ssf for v in s.eye)
            target = tuple(v * ssf for v in s.target)
            snaps.place_camera(stage, eye, target, focal_mm=s.focal_mm)
            snaps.snapshot(os.path.join(snap_dir, s.name + ".png"), frames)
            n_done += 1
    print("[{0}] {1} capture(s) written under {2}".format(
        prefix, n_done, snap_dir))
    return n_done


def capture_baseline(stage, snaps, placed, people, snap_dir, region_span_m,
                     obstacles=None, ssf=1.0, frames=DEFAULT_FRAMES,
                     adjacency_m=DISTRICT_ADJACENCY_M, prefix="bc",
                     only_families=None, centre=(0.0, 0.0), blocks=None):
    """THE call a launcher makes: build the plan, then run it. See the
    module docstring's WIRING section for the 3-line snippet.

    `only_families` (or env `FC_CAPTURE_FAMILIES`, comma-separated:
    overviews,districts,buildings,people,groups) restricts which shot
    families are rendered — the dataset's people-placement variants k>1
    share bit-identical geometry with k=1, so re-shooting the city families
    five times is pure waste; they shoot `people,groups` only.
    """
    import os as _os
    if only_families is None:
        raw = (_os.environ.get("FC_CAPTURE_FAMILIES") or "").strip()
        only_families = ([f.strip() for f in raw.split(",") if f.strip()]
                         or None)
    families = build_capture_plan(region_span_m, placed, people,
                                  obstacles=obstacles, adjacency_m=adjacency_m,
                                  centre=centre, blocks=blocks)
    if only_families:
        families = {k: v for k, v in families.items()
                    if k in only_families and k != "_all"}
        families["_all"] = [s for v in families.values() for s in v]
        print("[bc] capture families restricted to: {0}".format(
            sorted(k for k in families if k != "_all")))
    run_capture_plan(stage, snaps, families, snap_dir, ssf=ssf, frames=frames,
                     prefix=prefix)
    return families
