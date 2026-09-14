"""site_plan.py — a layout TRACED FROM A REAL PLACE instead of generated.

`city_layout` and `suburb_net` invent a plausible street fabric. This module
does not invent anything: it reads a *site plan* — road centrelines and block
roles measured off an aerial image — and hands back the same
``(net, blocks, info)`` triple `suburb_net.generate` does, so every pass
downstream (parcelling, ground, markings, the disaster stage) runs unchanged.

    photos/<site>.png                          the aerial
        |  tools/trace_site_plan.py            segment -> skeleton -> graph
        v
    config/site_plans/<site>.yaml              THE SOURCE OF TRUTH, hand-editable
        |  layout/site_plan.generate()         this module
        v
    (net, blocks, info)                        what suburb_net.generate returns

WHY THE YAML IS THE SOURCE OF TRUTH AND NOT THE IMAGE. Segmentation is a guess
that is right about most of a site and wrong about some of it — a shadow read
as asphalt, a rubble field read as a lane. Re-running the tracer at scene build
time would make the scene a function of that guess, and re-tuning a threshold
would silently move streets. So the trace is run ONCE, its output is committed,
and a wrong street is fixed by editing four numbers rather than by chasing a
threshold that also moves twelve other streets.

COORDINATES. The spec is in metres, +x east, +y north, origin at the centre of
the region — the same frame the generator uses everywhere else. The tracer does
the flip out of image pixels (+y down) once, on the way in.

WIDTHS ARE MEASURED, CLASSES ARE DERIVED. Each street carries the width the
image actually shows; `road_class` is only the nearest of `suburb_net.CLASSES`
and exists because downstream code switches on it (frontage, min radius,
markings). A traced 6.4 m lane stays 6.4 m wide, not the 10.7 m the `local`
class would impose.

BLOCK ROLES. A block here is not a housing parcel — it is a rubble field, a
concrete pad, a stand of trees, a parking lot. The spec tags each block with an
interior point and a role, and roles are matched to the recovered faces by
containment, so editing a street moves the block without orphaning its role.
`undeveloped` is set from the role, which is what tells the parcelling pass to
leave the land alone.

FEATURES ARE WHAT IS *ON* A BLOCK, and they are hand-authored. A role says the
block is a concrete pad; a feature says there is a 17 x 6 m container on it at
this position and this bearing. Detection gets buildings and rubble fields
roughly right and everything else wrong — canopy scores as rough as broken
concrete, a shipping container and a shed are the same bright rectangle — so
the spec carries measured footprints read off the aerial rather than a
classifier's guess. Each one is attached to the block that contains it, so a
pass that dresses a block can ask what is already standing there.
"""

import math
import os

import yaml

from layout import suburb_net as sn

_HERE = os.path.dirname(os.path.abspath(__file__))
SPEC_DIR = os.path.join(os.path.dirname(_HERE), "config", "site_plans")

# Roles that carry no built parcels. A block flagged `undeveloped` is emitted
# (its ground still has to be drawn) but the passes that build houses skip it —
# the same contract `suburb_net` gives its own left-whole parcels.
OPEN_ROLES = ("wooded", "grass", "rubble", "collapsed", "staging", "pad",
              "parking", "water")

# Feature kinds the spec may use. Not a closed set for the reader — a consumer
# should treat an unknown kind as "something is here, do not build over it" —
# but every kind used by a committed plan is listed so the preview can colour it
# and a typo shows up as a missing colour rather than as silence.
FEATURE_KINDS = ("building", "rubble", "wreck", "pit", "vehicle", "container",
                 "mast", "debris", "tower", "tank", "vessel", "pipe_rig",
                 "fence", "sign", "tree", "person")


def spec_path(name: str) -> str:
    """Resolve a site-plan name or path to a file."""
    if os.path.isabs(name) or os.sep in name:
        return name
    if not name.endswith((".yaml", ".yml")):
        name += ".yaml"
    return os.path.join(SPEC_DIR, name)


def load_spec(name: str) -> dict:
    path = spec_path(name)
    with open(path) as fh:
        spec = yaml.safe_load(fh) or {}
    spec.setdefault("name", os.path.splitext(os.path.basename(path))[0])
    spec["_path"] = path
    return spec


def region_of(spec: dict):
    """``(w_m, h_m)`` of the traced crop."""
    r = spec.get("region_m") or [0.0, 0.0]
    return float(r[0]), float(r[1])


# ---------------------------------------------------------------------------
# clipping — a traced street runs off the edge of the photo
# ---------------------------------------------------------------------------

def _clip_polyline(pts, rect):
    """Split *pts* into the runs that lie inside *rect*, cut at the boundary.

    A road that leaves and re-enters the crop comes back as two runs, which is
    correct: inside the region they are two streets, and joining them across
    land that is not in the scene would draw pavement through it.
    """
    x0, y0, x1, y1 = rect

    def inside(p):
        return x0 - 1e-6 <= p[0] <= x1 + 1e-6 and y0 - 1e-6 <= p[1] <= y1 + 1e-6

    def cut(a, b):
        """Point where segment a->b crosses the rect boundary, a inside."""
        t_best = 1.0
        for lo, hi, i in ((x0, x1, 0), (y0, y1, 1)):
            d = b[i] - a[i]
            if abs(d) < 1e-12:
                continue
            for bound in (lo, hi):
                t = (bound - a[i]) / d
                if 1e-9 < t < t_best:
                    q = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
                    if inside(q):
                        t_best = t
        return (a[0] + t_best * (b[0] - a[0]), a[1] + t_best * (b[1] - a[1]))

    runs, cur = [], []
    for i, p in enumerate(pts):
        if inside(p):
            if not cur and i > 0:
                cur.append(cut(p, pts[i - 1]))   # re-entering
            cur.append(p)
        else:
            if cur:
                cur.append(cut(cur[-1], p))      # leaving
                runs.append(cur)
                cur = []
    if cur:
        runs.append(cur)
    return [r for r in runs if len(r) >= 2 and sn.polyline_length(r) > 1.0]


def merge_chains(roads, tol=2.5):
    """Join traced roads that meet end to end into one polyline.

    The tracer emits one entry per skeleton edge, so a street that once had a
    junction on it — since pruned — arrives as two entries meeting at a point
    where nothing else joins. That point is not a corner of anything; it is
    where the graph was cut. Left as two roads, :func:`spline` pins both ends
    and the join stays sharp, which is where every remaining kerb bulge came
    from: measured, all ten vertices that widened the carriageway past 1.1x sat
    exactly on such a node and none anywhere else.

    Merged only when the two agree on width and class — a genuine change of
    carriageway is a real edge and must keep its own geometry.

    *tol* must match the graph's snap distance: the tracer rounds every vertex
    and two entries that meet in the photo can be authored a couple of metres
    apart, which is exactly why `generate` snaps them onto one node. At the
    default 1 m only one pair in `disaster_city` was close enough to see.
    """
    items = [dict(r) for r in roads]

    def compatible(a, b):
        """Same carriageway, allowing for MEASUREMENT NOISE.

        Width is measured per skeleton edge from the distance transform, so
        two halves of one street come back 5.4 m and 5.8 m and their derived
        classes can differ. Demanding exact agreement rejected 51 of the 52
        genuine joins on `disaster_city`. A real change of carriageway is a
        step, not a fraction of a metre.
        """
        wa, wb = float(a.get("width_m", 0.0)), float(b.get("width_m", 0.0))
        return abs(wa - wb) <= max(1.5, 0.25 * max(wa, wb, 1e-6))

    def ends(r):
        return (tuple(r["pts"][0]), tuple(r["pts"][-1]))

    changed = True
    while changed:
        changed = False
        for i, a in enumerate(items):
            if a is None:
                continue
            for j, b in enumerate(items):
                if b is None or i == j or not compatible(a, b):
                    continue
                a0, a1 = ends(a)
                b0, b1 = ends(b)
                for (pa, pb, flip_a, flip_b) in ((a1, b0, False, False),
                                                 (a1, b1, False, True),
                                                 (a0, b0, True, False),
                                                 (a0, b1, True, True)):
                    if math.dist(pa, pb) > tol:
                        continue
                    # Only where NOTHING ELSE ends: three roads meeting is a
                    # junction and merging two of them through it would draw
                    # the third into the side of a continuous street.
                    touch = sum(1 for r in items if r is not None
                                for e in ends(r) if math.dist(e, pa) <= tol)
                    if touch != 2:
                        continue
                    pa_pts = list(reversed(a["pts"])) if flip_a else list(a["pts"])
                    pb_pts = list(reversed(b["pts"])) if flip_b else list(b["pts"])
                    la = sn.polyline_length(pa_pts) or 1e-6
                    lb = sn.polyline_length(pb_pts) or 1e-6
                    wa, wb = float(a["width_m"]), float(b["width_m"])
                    a["pts"] = pa_pts + pb_pts[1:]
                    # LENGTH-WEIGHTED, so a 60 m run does not take the width of
                    # the 6 m stub it absorbed.
                    a["width_m"] = round((wa * la + wb * lb) / (la + lb), 1)
                    a["class"] = class_for_width(a["width_m"])
                    items[j] = None
                    changed = True
                    break
                if changed:
                    break
            if changed:
                break
    return [r for r in items if r is not None]


def simplify(pts, tol_m):
    """Douglas-Peucker: drop vertices no further than *tol_m* off the chord.

    THE TRACER'S WOBBLE IS IN THE VERTICES, and `spline` below cannot remove
    it — that pass fillets whatever corners it is handed, so a run of ten
    two-degree jinks becomes ten tiny arcs and the kerb still reads as a
    wandering line. MEASURED on this plan: 120 interior vertices with a median
    turn of 19 degrees, which is not a road, it is skeletonisation noise.
    At 1.5 m that falls to 32 corners with a median turn of 36 degrees — the
    corners a surveyor would have set.

    The tolerance is a HARD BOUND on how far the centreline moves, so it has to
    stay well inside the carriageway: these are 5-11 m wide, and a couple of
    metres is the difference between a straight street and a kinked one, not
    between this site and a different one.
    """
    if tol_m <= 0.0 or len(pts) < 3:
        return list(pts)
    a, b = pts[0], pts[-1]
    dx, dy = b[0] - a[0], b[1] - a[1]
    ln = math.hypot(dx, dy)
    worst, wi = -1.0, 0
    for i in range(1, len(pts) - 1):
        q = pts[i]
        if ln < 1e-9:
            d = math.hypot(q[0] - a[0], q[1] - a[1])
        else:
            d = abs(dy * (q[0] - a[0]) - dx * (q[1] - a[1])) / ln
        if d > worst:
            worst, wi = d, i
    if worst <= tol_m:
        return [a, b]
    return simplify(pts[:wi + 1], tol_m)[:-1] + simplify(pts[wi:], tol_m)


def spline(pts, step_m=4.0, radius_m=6.0):
    """Round the corners of a traced centreline; leave the straights alone.

    A traced centreline is the Douglas-Peucker simplification of a pixel
    skeleton, so every vertex is a real corner and every run between two of
    them is a straight chord. Swept into a ribbon that gives a facet at each
    vertex, and worse: `_mitre_offsets` widens the carriageway by
    1/cos(theta/2) at a corner, so a 60 degree kink bulges the road to 1.6x its
    width and the kerb comes out visibly scalloped.

    AN INTERPOLATING SPLINE CANNOT FIX THAT. Catmull-Rom passes THROUGH every
    control point, so a right-angle corner stays a right-angle corner however
    finely it is sampled — measured, it still turned 38 degrees at one vertex
    and the bulge was unchanged. What a road actually has at a corner is a
    FILLET, so that is what this cuts: each corner is replaced by a quadratic
    arc that leaves the straight `radius_m` before the vertex and rejoins it
    `radius_m` after, sampled every *step_m*.

    Straights stay exactly on the traced line, and the corner never moves
    further from it than the fillet radius — so the road still lies where the
    aerial put it, and every turn is one a carriageway can be swept round.
    """
    n = len(pts)
    if n < 3 or step_m <= 0.0 or radius_m <= 0.0:
        return list(pts)
    out = [pts[0]]
    for i in range(1, n - 1):
        a, v, c = pts[i - 1], pts[i], pts[i + 1]
        d0, d1 = sn._dist(a, v), sn._dist(v, c)
        if d0 < 1e-6 or d1 < 1e-6:
            continue
        # Never eat more than 45% of a leg: two corners on a short segment must
        # not consume each other and leave the road folded back on itself.
        r0 = min(radius_m, 0.45 * d0)
        r1 = min(radius_m, 0.45 * d1)
        u0 = sn._unit(sn._sub(v, a))
        u1 = sn._unit(sn._sub(c, v))
        turn = abs(sn._cross(u0, u1))
        p_in = sn._sub(v, sn._mul(u0, r0))
        p_out = sn._add(v, sn._mul(u1, r1))
        out.append(p_in)
        if turn > 1e-3:                    # a real corner: arc through it
            k = max(2, int(round((r0 + r1) / step_m)) + 1)
            for j in range(1, k):
                t = j / k
                q0 = (p_in[0] + (v[0] - p_in[0]) * t, p_in[1] + (v[1] - p_in[1]) * t)
                q1 = (v[0] + (p_out[0] - v[0]) * t, v[1] + (p_out[1] - v[1]) * t)
                out.append((q0[0] + (q1[0] - q0[0]) * t,
                            q0[1] + (q1[1] - q0[1]) * t))
        out.append(p_out)
    out.append(pts[-1])
    # Drop points that landed on top of each other; a zero-length segment has
    # no direction and `_mitre_offsets` would take its normal from noise.
    clean = [out[0]]
    for q in out[1:]:
        if sn._dist(q, clean[-1]) > 1e-3:
            clean.append(q)
    return clean


def class_for_width(w_m: float) -> str:
    """Nearest `suburb_net` road class to a measured width."""
    named = [(n, c["width_m"]) for n, c in sn.CLASSES.items() if n != "boundary"]
    return min(named, key=lambda nc: abs(nc[1] - w_m))[0]


# ---------------------------------------------------------------------------
# build
# ---------------------------------------------------------------------------

def generate(width_m, height_m, rng=None, cfg=None):
    """Build the traced network. Signature matches `suburb_net.generate`.

    *rng* is accepted and unused: a traced plan has nothing to draw. Keeping the
    parameter is what lets a launch script swap the two generators by name.
    """
    cfg = dict(cfg or {})
    spec = cfg.get("_spec") or load_spec(cfg.get("spec") or cfg.get("site_plan"))

    sw, sh = region_of(spec)
    if sw > 0.0 and sh > 0.0 and (abs(sw - width_m) > 1.0 or abs(sh - height_m) > 1.0):
        # The spec's region is the traced extent, and it is authoritative: a
        # config asking for a different box would crop or pad the real place.
        print(f"[site_plan] region_m from the spec: {sw:.1f} x {sh:.1f} m "
              f"(config asked for {width_m:.1f} x {height_m:.1f})")
    w_m, h_m = (sw, sh) if sw > 0.0 and sh > 0.0 else (width_m, height_m)
    hw, hh = w_m / 2.0, h_m / 2.0
    region = (-hw, -hh, hw, hh)

    net = sn.Network()
    # The crop edge, exactly as `suburb_net.generate` lays it: zero width, never
    # drawn, there only to close the planar graph so faces exist.
    corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
    cids = [net.add_node(p) for p in corners]
    for i in range(4):
        net.add_edge([corners[i], corners[(i + 1) % 4]], "boundary", "boundary",
                     a=cids[i], b=cids[(i + 1) % 4])

    snap = float(cfg.get("snap_m", 2.5))
    # Resample spacing for the centreline spline; 0 keeps the raw traced
    # chords. It is applied BEFORE the route is cut into the graph, so the
    # blocks are inset from the smoothed line too and the kerb the ribbon
    # sweeps is the kerb the block stops at.
    step = float(cfg.get("smooth_step_m", spec.get("smooth_step_m", 4.0)))
    # STRAIGHTEN BEFORE FILLETING. `simplify` drops the tracer's noise so that
    # what reaches `spline` is the corners a road actually has; `fillet_m` is
    # then the radius those corners are cut to. Doing it the other way round
    # rounds the noise and keeps it.
    simp = float(cfg.get("simplify_m", spec.get("simplify_m", 0.0)))
    fillet = float(cfg.get("fillet_m", spec.get("fillet_m", 6.0)))
    n_roads = 0
    for road in merge_chains(spec.get("roads") or [], tol=snap):
        pts = [(float(p[0]), float(p[1])) for p in road.get("pts") or []]
        if len(pts) < 2:
            continue
        w = float(road.get("width_m", sn.CLASSES["local"]["width_m"]))
        cls = str(road.get("class") or class_for_width(w))
        stype = str(road.get("street_type", cls))
        for run in _clip_polyline(pts, region):
            if simp > 0.0:
                run = simplify(run, simp)
            if step > 0.0:
                run = spline(run, step, fillet)
            run = _snapped(net, run, snap)
            if sn.polyline_length(run) <= 1.0:
                continue
            if not sn._connect_route(net, run, cls, stype, min_gap=0.0):
                continue
            # `_connect_route` mints exactly one street id and splits inherit
            # theirs, so the id it just used is the network's newest. Widths are
            # set after the fact because the route builder takes a class, not a
            # width, and here the measured width is the point.
            sid = net._sid
            for e in net.edges.values():
                if e.street_id == sid:
                    e.width_m = w
            n_roads += 1

    face_list = sn.faces(net)
    # The floor lives in the SPEC, not in this call: the tracer classified its
    # blocks under one value and a preview or a scene using another would be
    # looking at a different set of blocks than the roles were written for.
    min_area = cfg.get("min_block_m2", spec.get("min_block_m2", 150.0))
    blocks = sn.blocks_from_faces(net, face_list, min_area=float(min_area))
    open_roles = set(cfg.get("open_roles") or OPEN_ROLES)
    n_tagged = _apply_roles(blocks, spec.get("blocks") or [], open_roles)
    features = _attach_features(blocks, spec.get("features") or [])

    info = {"region": region, "park": None, "reserve": None,
            "undeveloped_polys": [b["poly"] for b in blocks if b["undeveloped"]],
            "undeveloped": sum(1 for b in blocks if b["undeveloped"]),
            "site_plan": spec.get("name"), "spec": spec.get("_path"),
            "roads": n_roads, "roles_tagged": n_tagged,
            # Also hung on their host block as `block["features"]`; here as one
            # flat list for a consumer that wants the site, not a block.
            "features": features,
            # Hand-drawn bare-ground outlines, passed straight through: they
            # describe ground no role or feature footprint covers.
            "ground_patches": list(spec.get("ground_patches") or []),
            "features_homeless": sum(1 for f in features if f["block"] is None),
            # `suburb_net` reports what its own passes made; a traced plan has
            # no such passes, and zero is the honest answer rather than absent.
            "collectors": 0, "connectors": 0, "loops": 0, "lollipops": 0}
    return net, blocks, info


def _snapped(net, pts, tol):
    """Pull the ends of a run onto an existing node within *tol*.

    The tracer rounds every vertex, so two streets that meet in the photo can
    arrive 30 cm apart. `Network.add_edge` snaps at 0.5 m; anything looser has
    to be done here, before the route is cut, or the graph gains a pair of
    nodes a handspan apart and the face between them is a sliver block.
    """
    if tol <= 0.0:
        return pts
    out = list(pts)
    for i in (0, -1):
        best, bd = None, tol
        for n in net.nodes.values():
            d = sn._dist(n.p, out[i])
            if d < bd:
                best, bd = n.p, d
        if best is not None:
            out[i] = best
    return out


def feature_poly(f):
    """A feature's outline in metres.

    `poly` when the spec gives one — a rubble field is not a rectangle and
    saying it is would put debris on clean ground. Otherwise the oriented
    rectangle from `size_m` and `yaw_deg`, which is what a building is.
    """
    if f.get("poly"):
        return [(float(x), float(y)) for x, y in f["poly"]]
    cx, cy = (float(v) for v in f["at"])
    L, W = (float(v) for v in f.get("size_m", (1.0, 1.0)))
    a = math.radians(float(f.get("yaw_deg", 0.0)))
    ca, sa = math.cos(a), math.sin(a)
    return [(cx + ca * dx - sa * dy, cy + sa * dx + ca * dy)
            for dx, dy in ((-L / 2, -W / 2), (L / 2, -W / 2),
                           (L / 2, W / 2), (-L / 2, W / 2))]


def _attach_features(blocks, features):
    """Hang each feature on the block that contains it.

    Centre first, then any vertex of the OUTLINE. A block polygon is inset by
    half a carriageway from the street centreline, so a 33 m trailer parked
    along the edge of a pad has its midpoint in that inset margin and belongs to
    the pad all the same. Testing the outline is what stops the inset deciding.

    Nearest-centroid is NOT the fallback here, unlike roles: a role describes a
    block and must land somewhere, whereas a feature describes a patch of
    ground and a building assigned to the wrong block would be dressed by the
    wrong pass. One that touches no block (a trailer stopped on the kerb) keeps
    `block: None` and is still returned.
    """
    for b in blocks:
        b["features"] = []
    out = []
    for f in features:
        if not f.get("at"):
            continue
        g = dict(f)
        g["poly"] = feature_poly(f)
        p = (float(f["at"][0]), float(f["at"][1]))
        host = next((b for b in blocks if sn.point_in_polygon(b["poly"], p)), None)
        if host is None:
            host = next((b for b in blocks
                         if any(sn.point_in_polygon(b["poly"], q) for q in g["poly"])),
                        None)
        g["block"] = host
        if host is not None:
            host["features"].append(g)
        out.append(g)
    return out


def rounded_poly(poly, r, seg=7):
    """*poly* (CCW) with every corner cut back to an arc of radius *r*.

    For ground a surveyor laid out rather than a face the tracer found. A
    traced block boundary runs down the middle of the street that bounds it, so
    it inherits every wobble in the road spline -- fine for a lawn, wrong for a
    car park, whose edge in the photograph is straight runs joined by arcs.
    Nothing else about the block changes: the spec gives the shape, the tracer
    still decides where the block IS.

    The radius is per-corner and CLAMPED to half the shorter adjacent edge, so
    a shape with one short side keeps its other corners full-size instead of
    every arc collapsing to the tightest one. A corner too straight to round
    (or a degenerate one) is emitted as its own vertex.
    """
    pts = [(float(x), float(y)) for x, y in poly]
    if len(pts) < 3 or r <= 0.0:
        return pts
    n = len(pts)
    out = []
    for i in range(n):
        v = pts[i]
        p, q = pts[(i - 1) % n], pts[(i + 1) % n]
        d1 = (p[0] - v[0], p[1] - v[1])
        d2 = (q[0] - v[0], q[1] - v[1])
        l1 = math.hypot(*d1)
        l2 = math.hypot(*d2)
        if l1 < 1e-9 or l2 < 1e-9:
            out.append(v)
            continue
        u1 = (d1[0] / l1, d1[1] / l1)
        u2 = (d2[0] / l2, d2[1] / l2)
        cosang = max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1]))
        half = math.acos(cosang) / 2.0
        if half < 1e-6 or abs(half - math.pi / 2.0) < 1e-9:
            out.append(v)
            continue
        # Cut back `t` along each edge; the arc that fits has radius
        # t * tan(half), which is `r` unless a short edge forced t down.
        t = min(r / math.tan(half), l1 / 2.0, l2 / 2.0)
        rr = t * math.tan(half)
        if rr < 1e-6:
            out.append(v)
            continue
        bis = (u1[0] + u2[0], u1[1] + u2[1])
        lb = math.hypot(*bis)
        if lb < 1e-9:
            out.append(v)
            continue
        c = (v[0] + bis[0] / lb * (rr / math.sin(half)),
             v[1] + bis[1] / lb * (rr / math.sin(half)))
        a1 = math.atan2(v[1] + u1[1] * t - c[1], v[0] + u1[0] * t - c[0])
        a2 = math.atan2(v[1] + u2[1] * t - c[1], v[0] + u2[0] * t - c[0])
        d = (a2 - a1 + math.pi) % (2.0 * math.pi) - math.pi
        for k in range(seg + 1):
            a = a1 + d * k / seg
            out.append((c[0] + rr * math.cos(a), c[1] + rr * math.sin(a)))
    return out


def rounded_rect(x0, y0, x1, y1, r, seg=7):
    """An axis-aligned rectangle with its four corners rounded off by *r*."""
    return rounded_poly([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], r, seg)


def _apply_roles(blocks, tags, open_roles):
    """Attach `role` to each block from the spec's interior points.

    Containment first, nearest-centroid as the fallback, so an edited street
    that moves a block by a few metres keeps its role instead of dropping it.
    Every block ends up with the key — `_untagged` when nothing claimed it — so
    a consumer can switch on it without a default.
    """
    for b in blocks:
        b["role"] = None
    hits = 0
    for t in tags:
        at = t.get("at")
        if not at:
            continue
        p = (float(at[0]), float(at[1]))
        hit = next((b for b in blocks
                    if b["role"] is None and sn.point_in_polygon(b["poly"], p)), None)
        if hit is None:
            free = [b for b in blocks if b["role"] is None]
            if not free:
                continue
            hit = min(free, key=lambda b: math.dist(b["centroid"], p))
            if math.dist(hit["centroid"], p) > 60.0:
                continue
        hit["role"] = str(t.get("role", "grass"))
        # OPTIONAL PER-BLOCK OVERRIDES travel with the role. `plant: false`
        # turns planting off for one block regardless of what its role would
        # normally get — the aerial is the authority on whether a particular
        # yard has trees in it, and the role is only a default.
        if "plant" in t:
            hit["plant"] = bool(t["plant"])
        # `outline: {poly: [[x, y], ...], corner_r: r}` — or `rect` for the
        # axis-aligned case — REPLACES the traced boundary. The car park is
        # what it exists for: its face follows the kerb spline of the road
        # around it, so the block wobbled while the photograph shows straight
        # runs joined by arcs, and the bay lines drawn on the bounding box
        # spilled out of it wherever it curved in. `poly` rather than only
        # `rect` because the lot is a TRAPEZOID — its west kerb is plumb and
        # its east one slants in going south.
        # Centroid and area are recomputed: every downstream pass reads them.
        _out = t.get("outline") or {}
        _rect, _shape = _out.get("rect"), _out.get("poly")
        _ring = None
        if _shape and len(_shape) >= 3:
            _ring = [(float(x), float(y)) for x, y in _shape]
            if sn.polygon_area(_ring) < 0.0:
                _ring = list(reversed(_ring))
        elif _rect and len(_rect) == 4:
            _ring = [(float(_rect[0]), float(_rect[1])),
                     (float(_rect[2]), float(_rect[1])),
                     (float(_rect[2]), float(_rect[3])),
                     (float(_rect[0]), float(_rect[3]))]
        if _ring:
            hit["poly"] = rounded_poly(_ring,
                                       float(_out.get("corner_r", 0.0)))
            hit["centroid"] = sn.polygon_centroid(hit["poly"])
            hit["area"] = abs(sn.polygon_area(hit["poly"]))
        hits += 1
    for b in blocks:
        if b["role"] is None:
            b["role"] = "_untagged"
        if b["role"] in open_roles:
            b["undeveloped"] = True
    return hits
