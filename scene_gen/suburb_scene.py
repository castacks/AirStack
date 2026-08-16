"""
suburb_scene.py — build a `suburb_net` layout onto a live USD stage.

WHY THIS DOES NOT REUSE `apply_ground_planes`
---------------------------------------------
`scene_generator.apply_ground_planes` paints one asphalt rect for the region and
one grass rect per block, then draws lane dashes down each corridor's centre. It
consumes blocks and corridors as AXIS-ALIGNED RECTS, and that is exactly what
this layout does not have: streets here are polyline centrelines and blocks are
polygons. Feeding it a bounding box per curved street would pave the bounding
box, which is the envelope trick the whole rewrite exists to avoid.

So the ground is written directly instead, and the geometry is honest about what
it is:

    asphalt     ONE ribbon mesh per street, swept along its centreline at the
                street's own width, plus a disc at each cul-de-sac turnaround
    grass       ONE triangulated mesh per block polygon
    centreline  dashes stepped ALONG the polyline and yawed to the local
                tangent, so the paint follows the road round its curve

Ribbons are built by offsetting each centreline vertex along the MITRE of its
two adjacent segments rather than along a single segment normal. On a curve the
segment normal leaves a wedge of unpaved ground on the outside of every joint;
the mitre closes it, which is the difference between a road and a string of
overlapping rectangles.

Z ORDER, which matters because these are coplanar sheets:

    0.000   asphalt ribbons and bulbs
    0.010   block grass          (above asphalt, so the road shows between)
    0.020   driveways            (above grass — a drive crosses the verge)
    0.020   crossings            (ladder bars, drawn on an asphalt mask)
    0.030   centreline dashes    (above everything flat)

Everything above ground — houses, trees — goes through
`scene_generator.apply_placements` unchanged, because that pass already takes an
arbitrary yaw and is exactly what is wanted here: a house's yaw is the frontage
tangent, so houses follow a curving street round its curve.
"""

import math
import os
import random

from pxr import Gf, Sdf, UsdGeom, Vt

import scene_generator as sg
import suburb_net as sn
import suburb_parcel as sp
import suburb_yardplan as yp
import suburb_park as spk

# ROADS SIT ABOVE GRASS. The original order put asphalt at 0.0 and block grass
# at 0.010 so "the road shows between the blocks" -- which only holds if a block
# never overlaps a carriageway. Where one does, the grass covers the road while
# the dashes at 0.030 still draw on top, and the result is a road rendered as a
# GRASS STRIP WITH WHITE LANE MARKINGS ON IT. Laying the carriageway over the
# ground is also what actually happens: a road is built on the land, not cut
# into a hole in it, so this ordering cannot produce that artefact at all.
# Z-FIGHTING IS WHY THESE ARE SPREAD SO FAR APART.
#
# These are five essentially coplanar sheets covering 1600 x 1200 m. The first
# version stacked them inside 25 mm total (0.005 to 0.030), which is far below
# what a depth buffer can resolve at that extent: past a few hundred metres the
# per-pixel depth step exceeds the gap, the sheets interpenetrate, and the
# render breaks into flickering TRIANGLES AND STREAKS along the shared edges --
# yellow ones, because the centreline dashes are the topmost layer punching
# through the asphalt below them.
#
# The separation is a rendering requirement, not a physical one. Paint does not
# sit 14 cm above a road, but at the altitude this dataset is captured from that
# offset is far below a pixel, whereas the z-fighting it prevents is not. Keep
# the ORDER (each layer must occlude the one under it) and keep the gaps large.
_Z_GRASS = 0.02
_Z_ASPHALT = 0.10
_Z_DRIVE = 0.16
_Z_CROSSWALK = 0.20
_Z_DASH = 0.24
# Park surfaces sit just above the block grass they are laid on, with their
# paint above that, on the same "surface then markings" convention as a street.
_Z_PARK_SURF = 0.06
_Z_PARK_PATH = 0.08
_Z_PARK_LINE = 0.12

# PAINT, the same two colours the urban scene uses. `road_markings` paints every
# marking `_WHITE` and reserves yellow for a centreline; this module painted its
# centreline (0.85, 0.75, 0.2) — a mustard far more saturated than road paint —
# down EVERY street. `_make_dash_mesh` binds no material, so displayColor is
# what the renderer shows: unlit and full-bright, which is why 3,000-odd of them
# read as yellow streaks rather than as lane markings. The z-spread above was an
# earlier attempt at the same symptom and fixed a real but different bug.
_WHITE = (0.92, 0.92, 0.92)
_YELLOW = (0.75, 0.63, 0.18)
# Crossing and stop-bar geometry, MUTCD-ish and matching the urban defaults.
_BAR_W_M = 0.45                # ladder bar width, across the direction of travel
_BAR_GAP_M = 0.55              # gap between bars
_STOP_BAR_W_M = 0.40           # stop bar depth, along the direction of travel

# THE JUNCTION IS A ZONE, NOT A POINT, and three passes need the same one:
# sidewalk tiles must stop before the corner, lane dashes must stop before the
# crossing, and the crossing is drawn where the dashes stopped. All three
# measure from the junction blob — the disc of asphalt whose radius is the half
# width of the widest street meeting there — so they agree by construction.
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
# which approaches get paint — ONE definition, shared
# ---------------------------------------------------------------------------
# The stop SIGN and the stop BAR are the same decision, and they were made in
# two places: `build_signs` walked the arms itself and the paint pass did not
# exist at all. Anything that decides "does this arm stop" twice will eventually
# disagree, and a stop bar with no sign over it (or the reverse) is worse than
# neither. Both now consume this.

def _arm_pts(net, node, edge):
    """The arm's centreline oriented AWAY from *node*, so s=0 is the junction."""
    return edge.pts if edge.a == node.id else list(reversed(edge.pts))


def _stop_bar_s(net, node):
    """Arclength along an approach at which the STOP LINE sits.

    One number, because the bar and the sign mark the same line. They were
    computed apart -- the bar from `_junction_stop_m`, the sign from
    `city_detail`'s `near_corner_m` (6.0 m) -- which put the sign ~5.6 m nearer
    the junction than the paint it is supposed to stand over, on a 10.7 m
    street. Sharing `_stop_approaches` made them agree about WHICH arms stop;
    this makes them agree about WHERE.
    """
    return _junction_stop_m(net, node) + 0.6 + _STOP_BAR_W_M * 0.5


def _stop_approaches(net):
    """``(node, edge)`` for every stop-controlled approach.

    The minor arm stops and the through road runs — which is what the suburban
    preset already asserts by setting `traffic_lights.intersection_chance: 0`.
    A collector is the through road by definition, so it does not stop.
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

    NOT EVERY JUNCTION. A residential four-way in a US subdivision carries no
    paint at all — drive any post-war plat and the crossings are on the
    collector, at the school route and the park, and nowhere else. Laddering
    every junction is the same error as striping a centreline down every
    street: it is not detail, it is noise, and it is what made these markings
    read as wrong rather than as sparse.

    So a crossing needs a collector in the junction — and then EVERY arm of
    that junction gets one, because a crossing on one leg and not the others
    reads as an omission rather than as a choice.
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

    The bars run PARALLEL to the direction of travel and repeat ACROSS the
    carriageway — that is what makes a ladder read as a crossing rather than as
    a row of rungs — and the count is derived from the street's own width so it
    lands kerb to kerb instead of overhanging. Same construction as
    `road_markings._ladder` in the urban scene, rewritten against a polyline
    centreline and a local tangent instead of an axis-aligned rect.
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

    `t` points away from the junction, so a driver approaching travels along
    -t; in right-hand traffic their half is the one on `_perp(t)` (the left
    normal of t is the right hand of -t). A bar drawn kerb to kerb would govern
    the departing lane too, which is not what a stop line means.
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


def _make_ribbon(stage, path, pts, half_w, z, ssf, uv_scale, color, mat=""):
    """A quad strip swept along *pts*. One mesh, not one quad per segment."""
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
    for i in range(len(pts)):
        v = cum[i] / max(uv_scale, 1e-6)
        uvs.append(Gf.Vec2f(0.0, v))
        uvs.append(Gf.Vec2f(2.0 * half_w / max(uv_scale, 1e-6), v))
    return _define_mesh(stage, path, verts, counts, idx, uvs, color, mat)


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
                 park=None):
    """Asphalt ribbons, block grass, driveways and centreline dashes."""
    roads_cfg = config.get("roads", {}) or {}
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
    # THREE GROUNDS, NOT ONE. A single grass made undeveloped land render as
    # mown lawn that nobody had built on, which is the opposite of what the
    # sparseness is for: unplatted ground has to LOOK unplatted or leaving it
    # empty reads as a missing row of houses. `grass_rough` is the AEC pack's
    # Grass_Countryside, `grass` its Grass_Cut. Both fall back to `grass`, so an
    # asset set that declares neither behaves exactly as before.
    rough_mat = _load_mat("grass_rough") or grass_mat
    park_grass_mat = _load_mat("grass_park") or grass_mat
    dirt_mat = _load_mat("dirt")

    # 1) Grass first, as a single sheet over the whole region. Blocks are
    #    polygons and do NOT tile the region exactly — the streets are the gaps
    #    — so painting only the blocks would leave the ground plane showing
    #    through wherever a block was rejected as too small.
    rx0, ry0, rx1, ry1 = region
    # The base sheet is everything the blocks do not cover — verges, leftovers,
    # the ground under the whole plat. That is rough ground, not lawn.
    sg._make_plane_mesh(stage, gnd + "/ground_base", rx0, ry0, rx1, ry1,
                        -0.005, uv_grass, ssf, display_color=(0.24, 0.36, 0.17),
                        mat_prim_path=rough_mat)

    n_road = 0
    for e in net.edges.values():
        if e.road_class == "boundary":
            continue
        if _make_ribbon(stage, f"{gnd}/road_{e.id}", e.pts, e.half_w,
                        _Z_ASPHALT, ssf, uv_asphalt, (0.15, 0.15, 0.15),
                        asphalt_mat) is not None:
            n_road += 1

    bulb_r = float(sn.DEFAULTS["bulb_radius_m"])
    n_bulb = 0
    for e in net.edges.values():
        if e.street_type != "lollipop":
            continue
        _make_disc(stage, f"{gnd}/bulb_{e.id}", e.pts[-1], bulb_r,
                   _Z_ASPHALT, ssf, uv_asphalt, (0.15, 0.15, 0.15), asphalt_mat)
        n_bulb += 1

    n_grass = n_rough = 0
    for i, b in enumerate(blocks):
        wild = bool(b.get("undeveloped"))
        col = (0.26, 0.34, 0.16) if wild else (0.2, 0.5, 0.1)
        if _make_polygon(stage, f"{gnd}/grass_{i}", b["poly"], _Z_GRASS, ssf,
                         uv_grass, col,
                         rough_mat if wild else grass_mat) is not None:
            n_grass += 1
            n_rough += 1 if wild else 0

    n_drive = 0
    for pi, p in enumerate(parcels):
        for di, d in enumerate(p["drives"]):
            if _make_ribbon(stage, f"{gnd}/drive_{pi}_{di}", [d["a"], d["b"]],
                            d["w"] / 2.0, _Z_DRIVE, ssf, uv_asphalt,
                            (0.45, 0.45, 0.43)) is not None:
                n_drive += 1

    # 2) MARKINGS — the same vocabulary the urban scene paints, which is what
    #    this should have been from the start: white paint, a stop bar on every
    #    stop-controlled approach, ladder crossings drawn as geometry, and a
    #    centreline only where a centreline belongs.
    #
    #    A LOCAL STREET CARRIES NO CENTRELINE. MUTCD 3B.01 warrants one by width
    #    and volume and a residential street meets neither — a post-war
    #    subdivision has bare asphalt between the junctions. Striping every
    #    street was wrong twice over: wrong as a road, and, because
    #    `_make_dash_mesh` binds no material and the colour was a saturated
    #    (0.85, 0.75, 0.2) mustard, it rendered as thousands of unlit
    #    full-bright quads — the "random streaks of yellow" reported from the
    #    sim. Collectors keep their centreline, which is most of what makes a
    #    collector legible as the through road.
    #
    #    MARKINGS STOP AT THE JUNCTION. Paint through the middle of a crossing
    #    is something no real street has. The setback is `_junction_stop_m`, the
    #    same figure the crossing and the stop bar are placed from, so the three
    #    agree by construction rather than by tuning.
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
                               c[0], c[1], _Z_DASH, dash_len, dash_w, yaw, ssf,
                               display_color=_YELLOW)
            n_dash += 1

    # 3) Crossings, drawn rather than placed. The prop-tile version of this
    #    scattered a stretched asset across the carriageway and could not mask
    #    the paint underneath it; the urban scene has always drawn its ladders
    #    and masks the band in asphalt first, so the lane lines do not run
    #    through the bars. Same here.
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
                     e.half_w, _Z_CROSSWALK - 0.02, ssf, uv_asphalt,
                     (0.15, 0.15, 0.15), asphalt_mat)
        n_xbar += _paint_ladder(stage, f"{gnd}/xwalk_{node.id}_{e.id}",
                                c, t, e.half_w, ssf, _Z_CROSSWALK)
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
                        e.half_w, ssf, _Z_DASH)
        n_stop += 1

    if park is not None:
        apply_park_ground(stage, config, park, gnd, ssf,
                          (asphalt_mat, grass_mat, park_grass_mat, dirt_mat))

    print(f"[suburb_scene] ground: {n_road} road ribbons, {n_bulb} turnarounds, "
          f"{n_grass} block meshes ({n_rough} rough/undeveloped), "
          f"{n_drive} driveways")
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

        `lot_fences` is one pool with two incompatible halves — a 0.99 m
        ornamental rail tagged "low" for a street boundary and a 1.89 m
        close-board panel tagged "privacy" for a rear one. US codes cap a front
        fence far below a rear one (3.5 ft vs 6-8 ft), so drawing from the whole
        pool puts a privacy panel along the kerb, which is not a suburb but a
        compound. The asset set documents this where it declares the pool.
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

    for pr in park["props"]:
        spec = _PARK_POOLS.get(pr["kind"])
        if spec is None:
            missing.add(pr["kind"])
            continue
        pool = pool_for(spec[0])
        if not pool:
            missing.add(pr["kind"])
            continue
        # ONE BENCH TYPE AND ONE BIN TYPE FOR THE WHOLE PARK. A park is
        # furnished by one procurement order, so every bin matches every other
        # bin; drawing per prop made a bench a different bench each time, which
        # reads as a salvage yard rather than as a park. `suburb_park` stamps a
        # per-kind `variant` once per seed for exactly the kinds that are a
        # matched set, and this consumes it -- the pools keep all their entries,
        # so a different seed still furnishes the park differently.
        #
        # It has to be done HERE and not by narrowing the pool in the asset set:
        # `benches` and `trash_cans` come from `shared.yaml` down the extends
        # chain, so cutting them would fix the type for every seed AND change
        # the street furniture with it.
        u = (pool[pr["variant"] % len(pool)] if "variant" in pr
             else pool[rng.randrange(len(pool))])
        out.append(pools.place(resolver, u, spec[1], pr["c"][0], pr["c"][1],
                               pr.get("yaw", 0.0), rng))

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

    WHY THIS EXISTS AT ALL. `build_city` lays a sidewalk ring, concrete pads and
    street furniture by walking each block's edge, and `city_detail` adds signs
    and lamps by walking each corridor — but both take blocks and corridors as
    RECTS, so neither can be called here. Without them the scene had roads and
    houses sitting on bare ground with nothing between them, which is most of
    why it read as a diagram rather than a place: on the old suburban path the
    sidewalk tiles alone are 15,346 of the 56,091 placements.

    Walking a POLYGON frontage is if anything simpler than walking a rect: the
    block ring is a polyline, so a tile every `step` along its arclength is the
    whole loop, and the tile's yaw is the local tangent, so the pavement follows
    the street round its curve instead of stair-stepping.

    TWO THINGS THE BARE RING WALK GETS WRONG, both fixed here.

    THE CORNERS PILE UP. Three or four blocks meet at a junction and each lays
    its own ring right into the corner, so the tiles overlap and heap up — the
    junction reads as a stack of slabs rather than a crossing. Tiles inside the
    junction zone are dropped, which leaves the corner open for the crossing
    that the crossing pass draws there.

    PROPS LAND ON THE ROAD. The inward offset assumes the block polygon IS the
    kerb; where `offset_polygon` hits its mitre limit it is not, and the prop
    ends up on the carriageway. Every prop is therefore checked positively
    against the street centrelines (:class:`_RoadIndex`) and dropped if it is
    not clear of the asphalt, rather than trusted because of where it came from.
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

    def spacing(key, default):
        return float((cats.get(key) or {}).get("spacing_m", default) or 0.0)

    lamp_sp = spacing("streetlights", 120.0)
    hyd_sp = spacing("fire_hydrants", 240.0)

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
                                (hyd, hyd_sp, "fire_hydrant")):
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
    print(f"[suburb_scene] signs: {len(out)} stop signs")
    return out


def _fence_run(p0, p1, mod_len, min_fit=0.60, max_fit=1.15):
    """Module centres, yaw and fit-scale for a fence along p0->p1.

    THE RUN IS COVERED END TO END. The first version laid whole modules and
    dropped the remainder: `n = length // mod_len`, centred. A 10 m boundary
    against the 5.95 m privacy panel therefore got ONE panel with 2 m of bare
    ground at each end, and a boundary shorter than one module got nothing at
    all. That is the "fences don't seem to be complete, there's large gaps" —
    not a placement bug but an arithmetic one, and it got worse the more the
    lot widths varied, because a fixed module divides a varying run badly.

    So the module count is chosen to SPAN the run and each module is scaled by
    the residual, the same trick `apply_ground` uses to land a crossing's bars
    flush kerb to kerb. Both candidate counts tile exactly; they differ only in
    whether the panel is squeezed or stretched to do it, so the one closer to
    its authored size wins.

    The scale is UNIFORM, because a placement carries one scale — so a squeezed
    panel is also shorter. That bounds how far this can be pushed and is why a
    run that would need a panel outside [min_fit, max_fit] is declined instead:
    a missing 1.5 m of fence reads better than a 2.7 m one towering over a
    bungalow. Returning the fit is what lets the caller pass `scale_mul`.
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


def build_open_planting(config, resolver, net, blocks, rng, pools):
    """Trees on the land the plat never built on.

    THE SPARSENESS WAS INVISIBLE. `suburb_net` marks parcels undeveloped and
    `generate_suburb_on_stage` skips them, so they came out as bare mown ground
    — which reads as a block somebody forgot to put houses on, not as land that
    was never platted. Unbuilt suburban land is WOODED: drainage reserve, the
    stand left between phases, the bit too steep to build. Planting it is what
    turns "no houses here" into a reason there are no houses here, and it is the
    other half of the rough-grass material — texture alone does not do it.

    Planted from `trees` (the OPEN-SPACE pool: big specimens, no kerb overhang
    constraint) rather than `street_trees`, at woodland spacing rather than
    garden spacing. Roads may cross this land, so every candidate is tested
    against the carriageway the same way `build_frontage` tests its props.
    """
    pool = pools.load(_raw_pool(config, "trees"))
    open_blocks = [b for b in blocks if b.get("undeveloped")]
    if not pool or not open_blocks:
        return []

    cfg = config.get("suburb_parcel") or {}
    per_100 = float(cfg.get("open_trees_per_100m2", 0.85) or 0.0)
    min_gap = float(cfg.get("open_tree_gap_m", 6.0) or 6.0)
    if per_100 <= 0.0:
        return []

    road = _RoadIndex(net)
    out, n_road, n_close = [], 0, 0
    for b in open_blocks:
        poly = b["poly"]
        area = abs(sn.polygon_area(poly))
        want = int(area / 100.0 * per_100)
        if want <= 0:
            continue
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        # A grid keyed at the rejection radius makes the spacing test O(1) per
        # candidate instead of O(n): at woodland density a big reserve carries
        # thousands of trees and the naive all-pairs check is what makes a
        # scatter pass quadratic.
        cell = max(min_gap, 1.0)
        grid = {}
        placed = 0
        for _ in range(want * 12):
            if placed >= want:
                break
            q = (rng.uniform(min(xs), max(xs)), rng.uniform(min(ys), max(ys)))
            if not sn.point_in_polygon(poly, q):
                continue
            if road.on_road(q, margin=1.5):
                n_road += 1
                continue
            gx, gy = int(q[0] // cell), int(q[1] // cell)
            near = False
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for r in grid.get((gx + dx, gy + dy), ()):
                        if sn._dist(q, r) < min_gap:
                            near = True
                            break
                    if near:
                        break
                if near:
                    break
            if near:
                n_close += 1
                continue
            grid.setdefault((gx, gy), []).append(q)
            u = pool[rng.randrange(len(pool))]
            out.append(pools.place(resolver, u, "tree", q[0], q[1],
                                   rng.uniform(0.0, 360.0), rng))
            placed += 1
    print(f"[suburb_scene] open land: {len(out)} trees over "
          f"{len(open_blocks)} undeveloped parcels "
          f"({n_road} rejected on road, {n_close} for spacing)")
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
                     catalogue=None):
    """Houses and parcel trees, with every per-asset correction applied.

    TWO TREE POOLS, PICKED ON `kind`. `suburb_parcel` already stamps every tree
    it emits with where it belongs — "street" for the verge rhythm, "front" for
    the specimen in the front setback, "back" for the block interior — and a
    kerb and a back garden are not planted with the same thing. Drawing all
    three from one pool is what put a 25.4 m crown (Black_Oak, 19.7 m tall) on
    the verge, where it overhangs the whole carriageway.

        street / front  ->  `street_trees`, modest crowns only
        back            ->  `trees`, open space, big specimens fine

    `street_trees` falls back to `trees` so an asset set that never split them
    (any set older than suburban_v2) behaves exactly as it did before.
    """
    houses = pools.load(_raw_pool(config, "buildings", "intact"))
    open_trees = pools.load(_raw_pool(config, "trees"))
    street_trees = pools.load(_raw_pool(config, "street_trees")) or open_trees

    # THE PACKAGE, not just the house. `suburb_parcel` has always stamped every
    # lot with an archetype and whether it carries a garage and a fence, and
    # NOTHING read those three keys -- the pass that placed lot furniture
    # (`suburb_lots`) belongs to the older rect pipeline and was never wired
    # into this one. So every archetype rendered as the same bare house and the
    # variation existed only in the data. This is where it becomes visible.
    addons = pools.load(_raw_pool(config, "buildings", "house_addons"))
    # An addon is authored to sit AGAINST its own parent, in the parent's frame,
    # so it composes correctly at the identical transform -- and only there.
    # Pairing is by the parent's stem, which is what the two names share.
    addon_for = {}
    for a in addons:
        stem = os.path.basename(a).split("_Extension")[0]
        for h in houses:
            if os.path.basename(h).split(".")[0] == stem:
                addon_for[h] = a
    garage_houses = [h for h in houses if h in addon_for]
    fence_low = pools.load_tagged(_raw_pool(config, "lot_fences"), "low")
    fence_priv = pools.load_tagged(_raw_pool(config, "lot_fences"), "privacy")

    out = []
    n_gar = n_fence = 0
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

            if ent is None:
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
            for (a, b, tag) in (h.get("fence_segs") or ()):
                pool_f = fence_priv if tag == "privacy" else fence_low
                if not pool_f:
                    continue
                # PICK THE MODULE THAT DIVIDES THIS RUN, not a random one. The
                # "low" pool holds a 5.28 m railing and a 2.0 m picket, and
                # which of them tiles a given boundary without being squeezed
                # depends entirely on the boundary's length -- which now varies
                # from a 16 m tight lot to a 48 m estate one. Drawing at random
                # threw that away and forced the fit-scale to absorb it.
                best = None
                for uf in pool_f:
                    fp = resolver.get(uf, "fence", scale=pools.scale_of(uf),
                                      axis_up=pools.axis_of(uf))
                    # Which measured axis runs ALONG the fence depends on the
                    # entry's yaw-offset: the park railing is authored running
                    # +Y with `yaw-offset: 90`, and reading sx for it would
                    # take the 0.33 m rail THICKNESS as the module length and
                    # lay it forty times over.
                    turned = abs((pools.yaw_of(uf) % 180.0) - 90.0) < 45.0
                    mod = max(0.3, float(fp["sy"] if turned else fp["sx"]))
                    run = _fence_run(a, b, mod)
                    if not run:
                        continue
                    if best is None or abs(math.log(run[0][3])) < best[0]:
                        best = (abs(math.log(run[0][3])), uf, run)
                if best is None:
                    continue
                _, uf, run = best
                for (fx, fy, fyaw, fit) in run:
                    out.append(pools.place(resolver, uf, "fence", fx, fy,
                                           fyaw, rng, scale_mul=fit))
                    n_fence += 1
        for t in p["trees"]:
            pool = (street_trees if t.get("kind") in ("street", "front")
                    else open_trees)
            if not pool:
                continue
            u = pool[rng.randrange(len(pool))]
            out.append(pools.place(resolver, u, "tree", t["c"][0], t["c"][1],
                                   rng.uniform(0.0, 360.0), rng))
    print(f"[suburb_scene] lot furniture: {n_gar} garages, "
          f"{n_fence} fence modules")
    return out


# ---------------------------------------------------------------------------
# entry point
# ---------------------------------------------------------------------------

def generate_suburb_on_stage(stage, config,
                             parent_path: str = "/World/stage/generated",
                             scene_scale_factor: float = 1.0,
                             snap_to_ground: bool = False) -> list:
    """Build the graph-based suburb onto a live stage. Returns placements.

    Same shape as `generate_city_v2.generate_city_v2_on_stage` so the launch
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

    # THE CUL-DE-SAC TURNAROUND IS PAVEMENT THE BLOCK POLYGON DOES NOT KNOW
    # ABOUT. `apply_ground` lays a disc of `bulb_radius_m` at the end of every
    # lollipop; the block boundary runs straight past it, so a lot hung off the
    # frontage there was sited on the carriageway and the house rendered
    # standing in the road. Measured 25-31 houses a seed, 2.3-2.7%, plus their
    # garages, fences and verge trees.
    #
    # The bulb is the authority on where the road is, so it is handed to the
    # parcel pass as a keep-out rather than worked around afterwards. The margin
    # is a front yard's worth on top of the paving: tangent to the kerb is not
    # somewhere a house stands either.
    pcfg = dict(config.get("suburb_parcel") or {})
    bulb_r = float(sn.DEFAULTS["bulb_radius_m"])
    margin = float(pcfg.get("bulb_margin_m", 3.0))
    pcfg.setdefault("keepout_discs",
                    [(e.pts[-1], bulb_r + margin) for e in net.edges.values()
                     if e.street_type == "lollipop"])

    # MEASURE THE HOUSES BEFORE SITING THEM. The resolver has to exist before
    # the parcel pass, not after it, because the parcel pass can only stop
    # guessing footprints if someone hands it real ones -- and the measurement
    # is what folds each garage wing into the footprint it is actually part of.
    resolver = sg._make_resolver(config)
    pools = AssetPools(config)
    yaw_off = float((config.get("suburb_parcel") or {})
                    .get("house_yaw_offset_deg", -90.0))
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

    placements = build_placements(config, resolver, parcels, rng, pools,
                                  yaw_off=yaw_off, catalogue=catalogue)
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

    yard, ystats = yp.plan(config, parcels, rng, resolver=resolver)
    placements += yard
    yp.report(ystats)
    placements += build_frontage(config, resolver, net, blocks, rng, pools)
    placements += build_open_planting(config, resolver, net, blocks, rng, pools)
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
    # pass that does that (generate_city_v2.prune_prims) is never called on this
    # path. Houses are left un-instanced so damage variants and per-building
    # edits stay possible.
    sg.apply_placements(stage, placements, parent_path, scene_scale_factor,
                        ground_snap, resolver=resolver,
                        instance_categories=set(config.get(
                            "instance_categories",
                            ["tree", "plant", "sidewalk", "streetlight",
                             "fire_hydrant", "sign", "crosswalk", "fence",
                             "play_structure"])))
    apply_ground(stage, config, net, blocks, parcels, info["region"],
                 parent_path, scene_scale_factor, park=park)
    return placements
