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

# ROADS SIT ABOVE GRASS. The original order put asphalt at 0.0 and block grass
# at 0.010 so "the road shows between the blocks" -- which only holds if a block
# never overlaps a carriageway. Where one does, the grass covers the road while
# the dashes at 0.030 still draw on top, and the result is a road rendered as a
# GRASS STRIP WITH WHITE LANE MARKINGS ON IT. Laying the carriageway over the
# ground is also what actually happens: a road is built on the land, not cut
# into a hole in it, so this ordering cannot produce that artefact at all.
_Z_GRASS = 0.005
_Z_ASPHALT = 0.015
_Z_DRIVE = 0.022
_Z_DASH = 0.030


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

def apply_ground(stage, config, net, blocks, parcels, region, parent_path, ssf):
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
    period = dash_len + dash_gap
    n_dash = 0
    for e in net.edges.values():
        if e.road_class in ("boundary", "cul_de_sac"):
            continue
        L = e.length
        k = int(L / period)
        for j in range(k):
            s = (j + 0.5) * period
            c = sn.point_at(e.pts, s)
            t = sn.tangent_at(e.pts, s)
            yaw = math.degrees(math.atan2(t[1], t[0]))
            sg._make_dash_mesh(stage, f"{gnd}/dash_{e.id}_{j}",
                               c[0], c[1], _Z_DASH, dash_len, dash_w, yaw, ssf,
                               display_color=(0.85, 0.75, 0.2))
            n_dash += 1

    print(f"[suburb_scene] ground: {n_road} road ribbons, {n_bulb} turnarounds, "
          f"{n_grass} block meshes, {n_drive} driveways, {n_dash} dashes")


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

    for blk in blocks:
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
                    u = pool[rng.randrange(len(pool))]
                    # Face the street: inward normal points into the block, so
                    # the kerb is at -n.
                    yaw = math.degrees(math.atan2(-n[1], -n[0]))
                    out.append(pools.place(resolver, u, cat, q[0], q[1], yaw,
                                           rng))
                s += sp_m
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
    """Houses and yard trees, with every per-asset correction applied."""
    houses = pools.load(_raw_pool(config, "buildings", "intact"))
    trees = pools.load(_raw_pool(config, "trees"))
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
            if not trees:
                break
            u = trees[rng.randrange(len(trees))]
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

    parcels = sp.parcel_blocks(blocks, rng, config.get("suburb_parcel") or {})
    pstats = sp.stats(parcels)
    print(f"[suburb_scene] {pstats['houses']} houses, {pstats['trees']} trees "
          f"on {pstats['blocks_built']}/{pstats['blocks']} blocks")

    resolver = sg._make_resolver(config)
    pools = AssetPools(config)
    placements = build_placements(
        config, resolver, parcels, rng, pools,
        yaw_off=float((config.get("suburb_parcel") or {})
                      .get("house_yaw_offset_deg", -90.0)))
    yard, ystats = yp.plan(config, parcels, rng, resolver=resolver)
    placements += yard
    yp.report(ystats)
    placements += build_frontage(config, resolver, net, blocks, rng, pools)
    placements += build_signs(config, resolver, net, rng, pools)
    import collections as _c
    print("[suburb_scene] placements by category: %s"
          % dict(_c.Counter(p["category"] for p in placements)))

    ground_snap = sg._make_physx_ground_snap() if snap_to_ground else None
    sg.apply_placements(stage, placements, parent_path, scene_scale_factor,
                        ground_snap, resolver=resolver)
    apply_ground(stage, config, net, blocks, parcels, info["region"],
                 parent_path, scene_scale_factor)
    return placements
