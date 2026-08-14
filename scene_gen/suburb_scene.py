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
    0.026   crosswalks           (paint on the carriageway, placed not drawn)
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

# THE JUNCTION IS A ZONE, NOT A POINT, and three passes need the same one:
# sidewalk tiles must stop before the corner, lane dashes must stop before the
# crossing, and the crossing is drawn where the dashes stopped. All three
# measure from the junction blob — the disc of asphalt whose radius is the half
# width of the widest street meeting there — so they agree by construction.
_JUNCTION_CLEAR_M = 2.5        # asphalt blob -> near edge of the crossing
_CROSSWALK_DEPTH_M = 3.0       # nominal along-street depth of a crossing
_SIDEWALK_CORNER_M = 6.0       # extra radius where the block rings converge

# The RetroNeighborhood crossing prop. This scene's houses, streetlight,
# hydrant, driveway and trees all come from that pack, so the crossing matches
# them; it lives on Nucleus, so nothing has to be downloaded. Read from the
# asset set's `usds.crosswalks` pool when one is declared and from here
# otherwise, so wiring the pool later needs no code change.
_CROSSWALK_USD = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/"
                  "Stages/RetroNeighborhood/Props/SM_Crosswalk.prop.usd")


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
    asphalt_mat, grass_mat = mats
    n = {"slab": 0, "mark": 0, "path": 0}

    def xf(pts, cx, cy, yaw):
        a = math.radians(yaw)
        ux, uy = math.cos(a), math.sin(a)
        return [(cx + ux * x - uy * y, cy + uy * x + ux * y) for (x, y) in pts]

    SURF = {                       # kind -> (colour, material, z)
        "soccer":              ((0.26, 0.44, 0.18), grass_mat,   _Z_PARK_SURF),
        "basketball_compound": ((0.24, 0.28, 0.33), asphalt_mat, _Z_PARK_SURF),
        "tennis_block":        ((0.18, 0.42, 0.32), asphalt_mat, _Z_PARK_SURF),
        "playground":          ((0.85, 0.76, 0.60), "",          _Z_PARK_SURF),
        "picnic":              ((0.42, 0.50, 0.31), grass_mat,   _Z_PARK_SURF),
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
        elif z["kind"] == "tennis_block":
            lines = [(spk.tennis_markings(), c["centre"], c["yaw"])
                     for c in z.get("courts", [])]
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

    # 1) Grass first, as a single sheet over the whole region. Blocks are
    #    polygons and do NOT tile the region exactly — the streets are the gaps
    #    — so painting only the blocks would leave the ground plane showing
    #    through wherever a block was rejected as too small.
    rx0, ry0, rx1, ry1 = region
    sg._make_plane_mesh(stage, gnd + "/ground_base", rx0, ry0, rx1, ry1,
                        -0.005, uv_grass, ssf, display_color=(0.22, 0.42, 0.16),
                        mat_prim_path=grass_mat)

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

    n_grass = 0
    for i, b in enumerate(blocks):
        if _make_polygon(stage, f"{gnd}/grass_{i}", b["poly"], _Z_GRASS, ssf,
                         uv_grass, (0.2, 0.5, 0.1), grass_mat) is not None:
            n_grass += 1

    n_drive = 0
    for pi, p in enumerate(parcels):
        for di, d in enumerate(p["drives"]):
            if _make_ribbon(stage, f"{gnd}/drive_{pi}_{di}", [d["a"], d["b"]],
                            d["w"] / 2.0, _Z_DRIVE, ssf, uv_asphalt,
                            (0.45, 0.45, 0.43)) is not None:
                n_drive += 1

    # 2) Centreline dashes, stepped along the polyline and yawed to the local
    #    tangent. This is the one thing the rect pass structurally could not do:
    #    it drew a straight line down the corridor centre, which on a curved
    #    street runs off the carriageway.
    #    MARKINGS STOP AT THE JUNCTION. Stepping dashes along the whole edge
    #    runs paint straight through the middle of every crossing, which no real
    #    street has: the centreline stops at the stop line and the junction box
    #    is left clear. The setback is `_junction_stop_m`, the same zone the
    #    crossing is placed in — so the dashes stop exactly where the crosswalk
    #    starts, rather than the two being tuned against each other.
    period = dash_len + dash_gap
    n_dash, n_dash_cut = 0, 0
    for e in net.edges.values():
        if e.road_class in ("boundary", "cul_de_sac"):
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
                               display_color=(0.85, 0.75, 0.2))
            n_dash += 1

    if park is not None:
        apply_park_ground(stage, config, park, gnd, ssf, (asphalt_mat, grass_mat))

    print(f"[suburb_scene] ground: {n_road} road ribbons, {n_bulb} turnarounds, "
          f"{n_grass} block meshes, {n_drive} driveways, {n_dash} dashes "
          f"({n_dash_cut} suppressed at junctions)")


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
        u = pool[rng.randrange(len(pool))]
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
    that `build_crosswalks` puts there.

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


def build_crosswalks(config, resolver, net, rng, pools):
    """A pedestrian crossing across one approach of every street junction.

    WHY NOT `road_markings`. That module paints crossings by consuming
    AXIS-ALIGNED RECT corridors — the representation this layout does not have.
    A crossing here is sited the way everything else in this module is sited: at
    an arclength along a polyline centreline, yawed to the local tangent there,
    exactly like the centreline dashes in :func:`apply_ground`.

    WHERE IT GOES. At nodes with `road_degree >= 3`, one street meeting another.
    Cul-de-sac stubs are excluded as approaches AND as the reason a node counts
    as a junction: nobody stripes a crossing over a dead end, and a node whose
    only extra arm is a stub is a stub junction, not a crossing. The crossing
    sits at `_junction_stop_m` from the node — clear of the asphalt blob where
    the carriageways overlap, and exactly where the lane dashes stop.

    HOW IT IS SIZED. From the asset, not from an assumption: `resolver.get`
    reports the prop's real sx/sy, its LONG axis is taken as the bar span
    (a crossing is wider across the road than it is deep), and it is repeated
    across the carriageway as many times as the street's own width needs, with
    a residual scale so the ladder lands flush kerb to kerb instead of
    overhanging or leaving a gap.
    """
    raw = _raw_pool(config, "crosswalks") or [_CROSSWALK_USD]
    pool = pools.load(raw)
    if not pool:
        return []

    out = []
    for n in net.nodes.values():
        if n.road_degree(net) < 3:
            continue
        arms = [net.edges[eid] for eid in n.edges
                if net.edges[eid].road_class not in ("boundary", "cul_de_sac")
                and net.edges[eid].street_type != "lollipop"]
        if len(arms) < 2:
            continue                    # a stub junction, not a crossing
        e = arms[rng.randrange(len(arms))]
        pts = e.pts if e.a == n.id else list(reversed(e.pts))
        s = _junction_stop_m(net, n) - _CROSSWALK_DEPTH_M * 0.5
        if sn.polyline_length(pts) < s + _CROSSWALK_DEPTH_M:
            continue                    # arm too short to hold a crossing
        c = sn.point_at(pts, s)
        t = sn.tangent_at(pts, s)
        across = sn._perp(t)            # the crossing runs kerb to kerb

        u = pool[rng.randrange(len(pool))]
        base_scale = pools.scale_of(u)
        fp = resolver.get(u, "crosswalk", scale=base_scale,
                          axis_up=pools.axis_of(u))
        span = max(float(fp["sx"]), float(fp["sy"]))
        if span <= 1e-3:
            continue
        # Repeat rather than stretch one copy: a stretched crossing has stretched
        # bars, and the number of bars is what makes it read as a crossing.
        reps = max(1, int(round(e.width_m / span)))
        fit = e.width_m / (reps * span)
        yaw = math.degrees(math.atan2(across[1], across[0]))
        if float(fp["sy"]) > float(fp["sx"]):
            yaw -= 90.0                 # long axis is local +Y, not +X
        for i in range(reps):
            off = (i - (reps - 1) * 0.5) * span * fit
            q = sn._add(c, sn._mul(across, off))
            out.append(pools.place(resolver, u, "crosswalk", q[0], q[1], yaw,
                                   rng, z_extra=_Z_CROSSWALK, scale_mul=fit))
    print(f"[suburb_scene] crosswalks: {len(out)} tiles")
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
    det = (config.get("city_detail") or {})
    near = float(((det.get("categories") or {}).get("signs_stop")
                  or {}).get("near_corner_m", 6.0) or 6.0)
    out = []
    for n in net.nodes.values():
        if n.road_degree(net) < 3:
            continue
        for eid in n.edges:
            e = net.edges[eid]
            if e.road_class in ("boundary", "collector"):
                continue          # the through road runs; the side street stops
            pts = e.pts if e.a == n.id else list(reversed(e.pts))
            if sn.polyline_length(pts) < near + 2.0:
                continue
            p = sn.point_at(pts, near)
            t = sn.tangent_at(pts, near)
            side = sn._perp(t)
            q = sn._add(p, sn._mul(side, e.half_w + 1.2))
            u = signs[rng.randrange(len(signs))]
            # Sign faces back down its own approach, at the driver.
            yaw = math.degrees(math.atan2(-t[1], -t[0]))
            out.append(pools.place(resolver, u, "sign", q[0], q[1], yaw, rng))
    return out


def build_placements(config, resolver, parcels, rng, pools, yaw_off=-90.0):
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
    out = []
    if not houses:
        print("[suburb_scene] WARNING: no buildings.intact pool in asset set")
    for p in parcels:
        for h in p["houses"]:
            if not houses:
                break
            u = houses[rng.randrange(len(houses))]
            # h["yaw_deg"] is the FRONTAGE TANGENT -- along the street. The house
            # faces ACROSS it toward the kerb, which is the tangent rotated by
            # `yaw_off`. That assumes the art faces +X; see the preset comment.
            out.append(pools.place(resolver, u, "house", h["c"][0], h["c"][1],
                                   h["yaw_deg"] + yaw_off, rng))
        for t in p["trees"]:
            pool = (street_trees if t.get("kind") in ("street", "front")
                    else open_trees)
            if not pool:
                continue
            u = pool[rng.randrange(len(pool))]
            out.append(pools.place(resolver, u, "tree", t["c"][0], t["c"][1],
                                   rng.uniform(0.0, 360.0), rng))
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
    parcels = sp.parcel_blocks(buildable, rng, config.get("suburb_parcel") or {})
    pstats = sp.stats(parcels)
    print(f"[suburb_scene] {pstats['houses']} houses, {pstats['trees']} trees "
          f"on {pstats['blocks_built']}/{pstats['blocks']} blocks "
          f"({n_open} left undeveloped)")

    resolver = sg._make_resolver(config)
    pools = AssetPools(config)
    placements = build_placements(
        config, resolver, parcels, rng, pools,
        yaw_off=float((config.get("suburb_parcel") or {})
                      .get("house_yaw_offset_deg", -90.0)))
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
    placements += build_crosswalks(config, resolver, net, rng, pools)
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
                             "fire_hydrant", "sign", "crosswalk"])))
    apply_ground(stage, config, net, blocks, parcels, info["region"],
                 parent_path, scene_scale_factor, park=park)
    return placements
