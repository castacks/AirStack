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

_Z_ASPHALT = 0.0
_Z_GRASS = 0.010
_Z_DRIVE = 0.020
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
        idx.extend([a, a + 2, a + 3, a + 1])
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

def _pool(config, *path):
    node = config.get("usds", {}) or {}
    for k in path:
        node = (node or {}).get(k) or {}
    return node if isinstance(node, list) else []


def _entry_usd(e):
    return e.get("usd") if isinstance(e, dict) else e


def build_placements(config, parcels, rng, yaw_off=-90.0):
    """Houses and trees as `scene_generator` placement dicts.

    The house yaw is the frontage tangent `suburb_parcel` already solved, so a
    house on a curving street turns with the street. That is the payoff of
    carrying real geometry through the layout: on the rect generator every
    building was locked to 0/90/180/270.
    """
    houses = _pool(config, "buildings", "intact")
    trees = _pool(config, "trees")
    out = []
    if not houses:
        print("[suburb_scene] WARNING: no buildings.intact pool in asset set")
    for p in parcels:
        for h in p["houses"]:
            if not houses:
                break
            e = houses[rng.randrange(len(houses))]
            out.append({
                "usd": _entry_usd(e), "x_m": h["c"][0], "y_m": h["c"][1],
                "z_m": 0.0,
                # h["yaw_deg"] is the FRONTAGE TANGENT — along the street. The
                # house has to face ACROSS it, toward the kerb. The lot's inward
                # normal points into the block, so the street is at -n, which is
                # the tangent rotated by -90.
                #
                # This assumes the asset is authored facing +X. That is a
                # property of the art, not of the layout, and these bungalows
                # are Objaverse conversions whose facing is not recorded
                # anywhere — so it is a knob rather than a constant. If the
                # houses come up backs-to-the-street, set this to +90.
                "yaw_deg": h["yaw_deg"] + yaw_off,
                "roll_deg": 0.0, "pitch_deg": 0.0,
                "scale": float(e.get("scale", 1.0)) if isinstance(e, dict) else 1.0,
                "category": "house", "axis_up": "Z",
            })
        for t in p["trees"]:
            if not trees:
                break
            e = trees[rng.randrange(len(trees))]
            out.append({
                "usd": _entry_usd(e), "x_m": t["c"][0], "y_m": t["c"][1],
                "z_m": 0.0, "yaw_deg": rng.uniform(0.0, 360.0),
                "roll_deg": 0.0, "pitch_deg": 0.0,
                "scale": float(e.get("scale", 1.0)) if isinstance(e, dict) else 1.0,
                "category": "tree", "axis_up": "Z",
            })
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
    placements = build_placements(
        config, parcels, rng,
        yaw_off=float((config.get("suburb_parcel") or {})
                      .get("house_yaw_offset_deg", -90.0)))

    ground_snap = sg._make_physx_ground_snap() if snap_to_ground else None
    sg.apply_placements(stage, placements, parent_path, scene_scale_factor,
                        ground_snap, resolver=resolver)
    apply_ground(stage, config, net, blocks, parcels, info["region"],
                 parent_path, scene_scale_factor)
    return placements
