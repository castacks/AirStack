"""site_features.py — the traced site's features as geometry.

A site plan (`layout/site_plan.py`) says a 21 x 17 m building stands at this
point on this bearing. This pass turns each such record into a **rectangular
prism** on the stage: a stand-in, correct in footprint, position and bearing,
and honest about being a stand-in. Swapping a prism for a real asset later is a
change of one function, because everything that decides WHERE it goes has
already happened in the layout.

WHY PRISMS AND NOT ASSETS, for now: the footprints are measured off an aerial
and the heights are read off an oblique, so plan is solid and elevation is an
estimate. A box says exactly that. Dropping a library building on each point
would say something the source photograph does not support — that this is a
brick warehouse with these windows — and would also silently mis-size it,
because a pack asset comes with its own footprint and would have to be scaled
to fit, which is how a 7 m drill tower becomes a 12 m house.

HEIGHT IS THE ONE DIMENSION A NADIR IMAGE CANNOT GIVE YOU. It comes from
`height_m` in the spec, hand-read off the oblique view by counting storeys;
`DEFAULT_H` covers a feature that has none, per kind, and is a guess by
construction. Nothing here measures a height.
"""

import json
import math
import os
import random

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

import scene_generator as sg
from layout import suburb_net as sn

# Fallback heights, metres, by kind. Deliberately modest — a stand-in that is
# too short reads as unfinished, one that is too tall reads as wrong.
# Kinds whose feature is a FIELD, not an object. `rubble` traces ~950 m2 of
# broken ground; one 8 x 8 m pile dropped at its centroid covers 6% of that and
# leaves the rest bare ground with a lump in the middle. These kinds get TILED
# with many instances of the asset instead, at a scale solved from the area.
SCATTER_KINDS = ("rubble", "debris", "collapsed")
# Pieces the nominal scale is solved for. Higher means smaller, more numerous
# chunks; a heap is more convincing as a dozen pieces than as one boulder.
SCATTER_TARGET = 9
# Grid pitch as a fraction of a piece's footprint. Below 1.0 the pieces
# OVERLAP, which is the point: a debris field has no gaps, and a jittered grid
# at pitch == footprint leaves a visible lattice of ground between them.
SCATTER_PACK = 0.62
# Fraction of its own height each piece is sunk. A rubble mesh is modelled
# sitting on a plane, so a ring of pieces at exactly z=0 reads as props placed
# on a floor; dropping them slightly buries the base into the ground.
SCATTER_SINK = 0.08
# Pieces whose footprint is within this factor of the size the grid wants are
# eligible at a given cell. Wider and a 0.2 m flake gets blown up to 3 m, which
# is how a debris field turns into a boulder field.
SCATTER_FIT_BAND = 2.0

DEFAULT_H = {"person": 1.8, "building": 5.0, "rubble": 2.5, "wreck": 3.0, "pit": 0.4,
             "vehicle": 3.2, "container": 2.8, "mast": 12.0, "debris": 1.2,
             "tower": 12.0, "tank": 5.0, "vessel": 4.0, "pipe_rig": 3.0,
             "fence": 1.8, "sign": 2.2, "tree": 8.0}

# Unlit display colour, so a stage with no lights still reads. These are
# stand-in colours for stand-in geometry: flat, distinguishable, not an
# attempt at what the material would be.
COLOUR = {"person": (0.90, 0.30, 0.30), "building": (0.78, 0.76, 0.72), "rubble": (0.46, 0.44, 0.42),
          "wreck": (0.40, 0.35, 0.33), "pit": (0.72, 0.63, 0.47),
          "vehicle": (0.30, 0.38, 0.52), "container": (0.55, 0.42, 0.28),
          "mast": (0.50, 0.50, 0.55), "debris": (0.50, 0.48, 0.45),
          "tower": (0.62, 0.62, 0.66), "tank": (0.22, 0.22, 0.26),
          "vessel": (0.70, 0.70, 0.74), "pipe_rig": (0.58, 0.58, 0.60),
          "fence": (0.55, 0.52, 0.46), "sign": (0.80, 0.78, 0.40),
          "tree": (0.22, 0.34, 0.16)}


def height_of(f: dict) -> float:
    return float(f.get("height_m") or DEFAULT_H.get(f.get("kind"), 3.0))


def _layer_info(stage, url: str):
    """`(unit_scale, default_prim_is_geometry)` for a referenced layer.

    THE SECOND HALF IS NOT COSMETIC. A reference targets the layer's
    `defaultPrim`, and some packs make that a bare `Mesh` rather than an Xform
    with the geometry beneath it. Referencing a Mesh onto an Xform composes the
    mesh's ATTRIBUTES but leaves the prim an Xform, so it has no children, an
    empty bound, and renders nothing at all — silently. DebrisConcrete is
    authored that way and 210 scattered pieces came out invisible; rubble_hd
    roots at an Xform, which is why the same code worked the day before.
    """
    try:
        sub = Usd.Stage.Open(url, load=Usd.Stage.LoadNone)
        if sub is None:
            return 1.0, False
        unit = 1.0
        mpu = UsdGeom.GetStageMetersPerUnit(sub) or 1.0
        stage_mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
        if abs(mpu - stage_mpu) > 1e-9 and stage_mpu > 1e-12:
            unit = mpu / stage_mpu
        dp = sub.GetDefaultPrim()
        # `LoadNone` is the cheap open, and for an asset whose geometry sits
        # behind a PAYLOAD it reports the defaultPrim with NO TYPE AT ALL --
        # not `Mesh`, empty. Every FactoryDistrict building is a 3 KB stub
        # over a payload, so the test below said "not a Gprim", the mesh was
        # composed onto the Xform, and the building rendered as nothing.
        # Re-open composed only in that case: the answer is unknowable
        # without it, and it is a handful of assets, not the library.
        if dp and not dp.GetTypeName():
            sub = Usd.Stage.Open(url)
            dp = sub.GetDefaultPrim() if sub is not None else dp
        return unit, bool(dp and dp.IsA(UsdGeom.Gprim))
    except Exception:
        return 1.0, False


_CATALOGUE = {}


def _load_catalogue(rel):
    """Load and cache a debris catalogue: `[{name, url, size, kind, material}]`.

    The catalogue is a LOCAL sidecar describing assets that live on Nucleus —
    it is how a piece's real footprint is known without opening 840 layers over
    the network to measure them.

    SEVERAL CATALOGUES MAY BE NAMED, as a list or a comma-separated string, and
    are concatenated. One pack alone gives a field one pack's worth of shapes:
    DebrisConcrete brings 18 large warm slabs and rubble_hd brings 840 small
    chunks, and a field drawn from both has a grain the size band picks from at
    every scale instead of the same dozen silhouettes repeating.
    """
    if not rel:
        return None
    if isinstance(rel, (list, tuple)):
        parts = [str(x).strip() for x in rel if str(x).strip()]
    else:
        parts = [x.strip() for x in str(rel).split(",") if x.strip()]
    if len(parts) != 1:
        key = "|".join(parts)
        if key in _CATALOGUE:
            return _CATALOGUE[key]
        merged = []
        for one in parts:
            merged.extend(_load_catalogue(one) or [])
        merged.sort(key=lambda q: q["_d"])
        _CATALOGUE[key] = merged
        return merged
    rel = parts[0]
    if rel in _CATALOGUE:
        return _CATALOGUE[rel]
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(here, "assets", rel)
    try:
        with open(path) as fh:
            doc = json.load(fh)
        pieces = doc.get("pieces") or []
        # `prefix` is prepended to every url. rubble_hd's urls are relative to
        # `scene_gen/assets/`; a pack catalogue's are relative to the assets
        # root itself, so the catalogue says which rather than the caller
        # guessing from the path.
        pre = doc.get("prefix", "scene_gen/assets/")
        for q in pieces:
            q["url"] = pre + q["url"]
    except Exception:
        pieces = []
    # Sorted by footprint so `_pick_piece` can bisect rather than scan.
    for p in pieces:
        p["_d"] = max(float(p["size"][0]), float(p["size"][1]))
    pieces = [p for p in pieces if p["_d"] > 0.02]
    pieces.sort(key=lambda p: p["_d"])
    _CATALOGUE[rel] = pieces
    return pieces


def _pick_piece(pieces, want, rng):
    """A random piece whose footprint is within `SCATTER_FIT_BAND` of *want*.

    Falls back to the nearest in size when the band is empty, so a field whose
    cells are larger or smaller than anything in the catalogue still fills.
    """
    lo, hi = want / SCATTER_FIT_BAND, want * SCATTER_FIT_BAND
    import bisect
    ds = [p["_d"] for p in pieces]
    i, j = bisect.bisect_left(ds, lo), bisect.bisect_right(ds, hi)
    if j > i:
        return pieces[rng.randrange(i, j)]
    if not pieces:
        return None
    k = min(bisect.bisect_left(ds, want), len(pieces) - 1)
    return pieces[k]


def _point_in_poly(pt, poly) -> bool:
    """Even-odd ray cast. Boundary cases are not special-cased: a scatter point
    landing exactly on an edge either way is indistinguishable in the result."""
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        if (y0 > y) != (y1 > y):
            xc = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
            if x < xc:
                inside = not inside
    return inside


def _scatter_asset(stage, xf, f, asset_root, ssf) -> int:
    """Tile the feature's polygon with instances of its asset. Count placed.

    WHY NOT ONE BIG ASSET. The alternative to this is scaling a single pile up
    until its bounding box covers the field, and it fails twice: the traced
    fields are concave, so a box-covering piece spills onto the road and the
    grass either side, and a rubble mesh scaled 4x has 4x-sized chunks — a
    "brick" the size of a car, which reads as a rock formation rather than as a
    collapsed building. Many pieces near their modelled size keep the grain
    right and follow the outline.

    The scale is SOLVED, not configured: given the field's area and the asset's
    own footprint, `SCATTER_TARGET` pieces would each need to be `s_nom` times
    its modelled size, clamped to the feature's `asset_scale_band`. The grid
    pitch then comes from that, so a small asset yields many pieces and a large
    one yields few, and neither leaves gaps.

    Positions are laid on a JITTERED grid rather than drawn uniformly at
    random: uniform sampling clumps, and a clump plus a bare patch is exactly
    the artefact this pass exists to remove.
    """
    poly = [(float(x), float(y)) for x, y in (f.get("poly") or [])]
    if len(poly) < 3:
        return 0
    rel = str(f.get("asset") or "").strip()
    if not rel or not asset_root:
        return 0
    url = sg._join_asset_root(rel, asset_root)

    have = f.get("asset_size_m") or [4.0, 4.0, 1.0]
    L0, W0 = float(have[0]), float(have[1])
    H0 = float(have[2]) if len(have) > 2 else 1.0
    a0 = max(L0 * W0, 1e-6)
    lo, hi = f.get("asset_scale_band") or (0.70, 1.40)
    lo, hi = float(lo), float(hi)
    target = max(int(f.get("scatter_target", SCATTER_TARGET)), 1)
    area = abs(sn.polygon_area(poly))

    # A CATALOGUE OF CHUNKS BEATS ONE PILE REPEATED. `scatter_catalogue` names
    # a JSON of individually-measured debris pieces; with it, each cell draws a
    # DIFFERENT piece sized to the cell, so the field has a grain and a colour
    # range instead of being one photogrammetry scan stamped 50 times. That
    # repetition is what made the first version read as a single white mass
    # from above — the pile's albedo is pale concrete, and 50 identical copies
    # average to exactly that pale with no variation to break it up.
    cat = _load_catalogue(f.get("scatter_catalogue"))
    if cat:
        dmax = math.sqrt(area / target)
        # TIGHTER THAN A WHOLE PILE NEEDS. `SCATTER_PACK` is calibrated for a
        # roughly convex heap that fills its own footprint; a catalogue chunk
        # is an irregular flake or slab covering perhaps half of its bounding
        # box, so the same pitch leaves the ground showing between pieces.
        pitch = max(dmax * SCATTER_PACK * 0.8, 0.4)
    else:
        s_nom = min(max(math.sqrt(area / max(target * a0, 1e-6)), lo), hi)
        pitch = max(max(L0, W0) * s_nom * SCATTER_PACK, 0.5)

    # Local frame: the parent Xform already carries the translate and the yaw,
    # so a world point has to come back through both to be placed under it.
    cx, cy = (float(v) for v in f["at"])
    yaw = math.radians(float(f.get("yaw_deg", 0.0)))
    ca, sa = math.cos(-yaw), math.sin(-yaw)

    units = {}                              # per distinct asset, not per piece
    mesh_rooted = {}
    if not cat:
        units[url], mesh_rooted[url] = _layer_info(stage, url)
    rng = random.Random(int(abs(cx) * 1000) ^ int(abs(cy) * 1000) ^ len(poly))

    xs = [q[0] for q in poly]
    ys = [q[1] for q in poly]
    n = 0
    gy = min(ys) + pitch * 0.5
    while gy < max(ys) + pitch * 0.5:
        gx = min(xs) + pitch * 0.5
        while gx < max(xs) + pitch * 0.5:
            px = gx + rng.uniform(-0.35, 0.35) * pitch
            py = gy + rng.uniform(-0.35, 0.35) * pitch
            gx += pitch
            if not _point_in_poly((px, py), poly):
                continue
            if cat:
                want = dmax * rng.uniform(0.75, 1.30)
                pick = _pick_piece(cat, want, rng)
                if pick is None:
                    continue
                purl = sg._join_asset_root(pick["url"], asset_root)
                pd = max(pick["size"][0], pick["size"][1]) or 1.0
                H0 = float(pick["size"][2])
                if purl not in units:
                    units[purl], mesh_rooted[purl] = _layer_info(stage, purl)
                s = (want / pd) * units[purl]
            else:
                purl = url
                s = min(max(s_nom * rng.uniform(0.78, 1.28), lo), hi) * units[url]
            dx, dy = px - cx, py - cy
            lx, ly = dx * ca - dy * sa, dx * sa + dy * ca
            piece = UsdGeom.Xform.Define(
                stage, Sdf.Path(f"{xf.GetPath()}/piece_{n:03d}"))
            piece.AddTranslateOp().Set(
                Gf.Vec3d(lx * ssf, ly * ssf, -H0 * s * SCATTER_SINK * ssf))
            piece.AddRotateZOp().Set(rng.uniform(0.0, 360.0))
            piece.AddScaleOp().Set(Gf.Vec3f(s, s, s))
            # A MESH-ROOTED ASSET NEEDS A TYPELESS HOST. Referenced onto this
            # Xform it would compose to nothing; referenced onto a child
            # defined with no type, the Mesh type comes through the arc and it
            # renders. Xform-rooted assets are unaffected either way, so the
            # child is used for both and the shape of the tree stays uniform.
            host = stage.DefinePrim(Sdf.Path(f"{piece.GetPath()}/geo"))
            host.GetReferences().AddReference(purl)
            # ONE PROTOTYPE PER DISTINCT ASSET — BUT NOT FOR A MESH-ROOTED ONE.
            # An instance root renders its PROTOTYPE's descendants, and when the
            # reference target is a bare Mesh the prototype has none: the point
            # data sits on the instance root itself and is never drawn. The
            # prims still report a valid extent, so this reads as 210 healthy
            # invisible pieces. Only container-rooted assets are instanced.
            host.SetInstanceable(not mesh_rooted.get(purl, False))
            n += 1
        gy += pitch
    if n:
        prim = xf.GetPrim()
        # customData takes scalars, not lists — several catalogues record as one
        # comma-separated string.
        _src = f.get("scatter_catalogue") or url
        if isinstance(_src, (list, tuple)):
            _src = ", ".join(str(x) for x in _src)
        prim.SetCustomDataByKey("sourceAsset", str(_src))
        prim.SetCustomDataByKey("scatterCount", n)
        prim.SetCustomDataByKey("scatterDistinct", len(units))
        prim.SetCustomDataByKey("standIn", False)
    return n


# Characters with NO SKELETON — posed statics. `_bind_human_pose` would find
# no Skeleton and silently leave them in their authored pose, so asking for one
# is a lie in the ground truth rather than an error. They stand, and only stand.
PERSON_NO_SKELETON = ("rp_dennis_posed", "rp_fabienne_percy_posed",
                      "rp_mei_posed")
_PERSON_DIMS = {}


def _person_dims(url):
    """`(height_m, depth_m)` of a character asset, measured once.

    Height feeds `pose_z_offset`'s stature scaling and depth is half the lift
    for a lying figure, so both have to be MEASURED — the pack runs 1.73-1.86 m
    and an unscaled drop buries the short ones.
    """
    if url in _PERSON_DIMS:
        return _PERSON_DIMS[url]
    hd = (1.80, 0.30)
    try:
        st = Usd.Stage.Open(url)
        if st is not None:
            st.Load()
            mpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
            bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                   [UsdGeom.Tokens.default_])
            r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
            if not r.IsEmpty():
                mn, mx = r.GetMin(), r.GetMax()
                hd = ((mx[2] - mn[2]) * mpu,
                      min((mx[0] - mn[0]), (mx[1] - mn[1])) * mpu)
    except Exception:
        pass
    _PERSON_DIMS[url] = hd
    return hd


def _place_person(stage, xf, f, asset_root, ssf, pose_cache):
    """Author one survivor: reference, pose, and put their support on the ground.

    EVERY FAILURE MODE HERE IS SILENT — a figure at the wrong z is authored
    without error and simply floats or sinks — so this defers to the repo's own
    measured machinery rather than guessing:

      `scene_generator.pose_z_offset`  solves a ground pose against the rig's
                                       OWN hip, because hip height does not
                                       track stature across this pack.
      `people._seated_asset_dz`        the male rigs sit 0.15 m high on a SEAT
                                       pose; it does not fire on ground poses.
      `people.LYING_POSES`             a lying pose is authored UPRIGHT and has
                                       to be rolled +-90 and lifted by half the
                                       body depth, or it stands up like a plank.

    An unknown pose name raises inside `pose_z_offset`, which is wanted: the
    alternative is a figure silently left in its A-pose.
    """
    import scene_generator as sgen
    from disaster import people as ppl

    rel = str(f.get("asset") or "").strip()
    if not rel or not asset_root:
        return False
    url = sgen._join_asset_root(rel, asset_root)
    pose = str(f.get("pose") or "").strip()
    base = os.path.basename(url).lower()
    if pose and base.startswith(PERSON_NO_SKELETON):
        print(f"[site_features] {os.path.basename(url)} has no skeleton; "
              f"pose {pose!r} dropped, placed standing")
        pose = ""
    if pose in ppl.BANNED_POSES:
        raise ValueError(f"pose {pose!r} is banned (people.BANNED_POSES)")

    height_m, depth_m = _person_dims(url)
    roll = float(ppl.LYING_POSES.get(pose, 0.0)) if pose else 0.0
    # THE SIDE POSES NEED A SPIN AS WELL AS A ROLL. `LYING_SPIN` is authored as
    # PITCH — it turns a laid-out figure over about its own long axis without
    # moving the head — so `lying_side_l/r` and `lying_curled_l` land on their
    # backs rather than their sides without it.
    pitch = float(ppl.LYING_SPIN.get(pose, 0.0)) if pose else 0.0
    if pose and pose in ppl.LYING_POSES:
        dz = ppl._lying_lift(pose, depth_m, height_m)
    else:
        dz = sgen.pose_z_offset(url, pose, height_m) if pose else 0.0
        dz += ppl._seated_asset_dz(url, pose)

    yaw = float(f.get("yaw_deg", 0.0))
    z0 = float(f.get("z_m", 0.0)) + dz
    # `build` has already authored translate+rotateZ for the generic case; a
    # person needs its own z drop and an X roll, so take the ops over rather
    # than append to them (AddXformOp raises on a duplicate).
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(f["at"][0]) * ssf,
                                     float(f["at"][1]) * ssf, z0 * ssf))
    # Roll on X, spin on Y, yaw on Z — the (roll, pitch, yaw) triple the
    # survivor planner authors, so a lying pose here matches one it places.
    xf.AddRotateXYZOp().Set(Gf.Vec3f(roll, pitch, yaw))
    unit, mesh_rooted = _layer_info(stage, url)
    host = xf.GetPrim()
    if mesh_rooted:
        host = stage.DefinePrim(Sdf.Path(f"{xf.GetPath()}/geo"))
    host.GetReferences().AddReference(url)
    if abs(unit - 1.0) > 1e-6:
        xf.AddScaleOp().Set(Gf.Vec3f(unit, unit, unit))
    if pose:
        sgen._bind_human_pose(stage, host, url, pose, pose_cache)
    prim = xf.GetPrim()
    prim.SetCustomDataByKey("sourceAsset", url)
    prim.SetCustomDataByKey("pose", pose or "idle(authored)")
    prim.SetCustomDataByKey("poseDropM", float(dz))
    prim.SetCustomDataByKey("heightM", float(height_m))
    return True


def _place_asset(stage, xf, f, asset_root, ssf):
    """Reference a library USD onto the feature's Xform. True if it took.

    UNIFORM SCALE ONLY, and only within a band. The matcher chose this asset
    because its bounding box was close to the measured footprint, so a small
    correction is honest — but scaling a 30 m building down to 12 m to make it
    "fit" produces a doll's house with 3 m doors, and non-uniform scale would
    shear it. Outside the band the asset is placed at its NATURAL SIZE and the
    prim records the mismatch, because a visible wrong-sized building is a
    better bug report than a silently squashed one.
    """
    rel = str(f.get("asset") or "").strip()
    if not rel or not asset_root:
        return False
    # `_join_asset_root` handles the LOCAL-ROOT SCHEMES as well as the plain
    # join: `airstack://scene_gen/assets/...` expands against the repo, an
    # absolute URL passes through, and anything else hangs off `asset_root`.
    # The damage archetypes have to come from the repo copy — the Nucleus copy
    # under URBAN_V3_BAKE/complete_1/ ships `archetypes/` and nothing else, so
    # its `../../materials/burn/*.png` can never resolve there.
    url = sg._join_asset_root(rel, asset_root)
    unit, mesh_rooted = _layer_info(stage, url)
    # See `_layer_info`: an asset whose defaultPrim is a bare Mesh composes to
    # nothing on an Xform. Only such an asset gets the typeless child, so the
    # assets already placing correctly keep the exact tree they had.
    if mesh_rooted:
        host = stage.DefinePrim(Sdf.Path(f"{xf.GetPath()}/geo"))
        host.GetReferences().AddReference(url)
    else:
        xf.GetPrim().GetReferences().AddReference(url)
    # UNITS FIRST, FIT SECOND. A reference does NOT rescale: `metersPerUnit` is
    # metadata the consumer is expected to honour, and USD composition does not.
    # So a RetroNeighborhood house authored in centimetres arrives in a metre
    # stage a hundred times too big — measured, `SM_House_02` came in 753 m
    # tall against its true 7.8 m. The extents pass already recorded the asset's
    # size in REAL metres, so comparing that against what the reference actually
    # composes to recovers the conversion without opening the layer again.
    want = f.get("size_m")
    have = f.get("asset_size_m")
    scale = unit
    # A QUARTER TURN IS PART OF THE MATCH. `match_assets.fit_score` scores an
    # asset against a footprint ALLOWING a 90 degree turn — a 19 x 4 m trailer
    # and a 4 x 19 m one are the same model — but placing it never applied that
    # turn, so every asset whose long axis ran the other way was dropped in
    # crosswise. That is how `vehicle_02`, an 18.7 x 3.9 m box trailer measured
    # along the kerb, ended up lying north-south through the house behind it.
    if want and have:
        wl, ww = float(want[0]), float(want[1])
        al, aw = float(have[0]), float(have[1])
        if (wl >= ww) != (al >= aw):
            xf.AddRotateZOp(opSuffix="fit").Set(90.0)
    if want and have:
        wl, hl = max(float(want[0]), float(want[1])), max(float(have[0]), float(have[1]))
        if hl > 1e-6:
            fit = wl / hl
            # A HEAP MAY BE STRETCHED, A BUILDING MAY NOT. `asset_scale_band`
            # lets a rubble model cover its measured field — a debris mesh has
            # no true size, it is a texture of broken concrete — while a house
            # stays within a few percent of the size it was modelled at.
            lo, hi = f.get("asset_scale_band") or (0.70, 1.40)
            if float(lo) <= fit <= float(hi):
                scale = unit * fit
    if abs(scale - 1.0) > 1e-6:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    prim = xf.GetPrim()
    prim.SetCustomDataByKey("sourceAsset", url)
    prim.SetCustomDataByKey("assetScale", float(scale))
    prim.SetCustomDataByKey("standIn", False)
    return True


def build(stage, features, parent_path: str, ssf: float = 1.0,
          z0: float = 0.0, asset_root: str = "",
          kind_materials=None, pose_cache=None) -> list:
    """Write one prism per feature. Returns the placement records.

    Each prism is its own Xform carrying the translate and the Z rotation, with
    an axis-aligned box beneath it in local space. Rotating the box's POINTS
    instead would work equally well for the geometry and lose the bearing —
    and the bearing is a measurement, so it should survive on the prim where a
    later pass (or a person in the viewport) can read it back.
    """
    out = []
    for i, f in enumerate(features or []):
        kind = str(f.get("kind", "debris"))
        L, W = (float(v) for v in (f.get("size_m") or _rect_of(f)))
        h = height_of(f)
        cx, cy = (float(v) for v in f["at"])
        yaw = float(f.get("yaw_deg", 0.0))

        name = f"{_sanitize(kind)}_{i:02d}"
        xf = UsdGeom.Xform.Define(stage, Sdf.Path(f"{parent_path}/{name}"))
        xf.AddTranslateOp().Set(Gf.Vec3d(cx * ssf, cy * ssf, z0 * ssf))
        xf.AddRotateZOp().Set(yaw)
        # NAMED AFTER THE FEATURE, not "box". A USD importer that collapses a
        # single-child Xform keeps the MESH's name and drops the parent's, and
        # then every prism in the scene is called `box` and nothing downstream —
        # a preview that colours by kind, a person in the viewport — can tell a
        # container from a drill tower.
        # A CHOSEN LIBRARY ASSET WINS OVER THE PRISM. The prism was always a
        # stand-in for "we know where and how big, not what"; once a feature
        # names an asset, that question is answered.
        # A PERSON IS NOT AN OBJECT. The rig is posed, dropped onto its own
        # support and possibly laid down; none of the footprint fitting or the
        # stand-in prism below means anything for one, so it returns early.
        if kind == "person":
            if _place_person(stage, xf, f, asset_root, ssf,
                             pose_cache if pose_cache is not None else {}):
                out.append({"prim_path": str(xf.GetPath()), "category": kind,
                            "size_m": (L, W, h), "at": (cx, cy),
                            "yaw_deg": yaw, "pose": f.get("pose", ""),
                            "usd": sg._join_asset_root(str(f["asset"]), asset_root),
                            "note": f.get("note", "")})
                continue

        # A FIELD IS TILED, AN OBJECT IS PLACED. A traced rubble polygon is an
        # area of broken ground, so its asset is repeated across it; a building
        # or a tanker is one thing at one place and gets exactly one instance.
        if (kind in SCATTER_KINDS and f.get("poly") and f.get("asset")
                and f.get("scatter", True)):
            npc = _scatter_asset(stage, xf, f, asset_root, ssf)
            if npc:
                out.append({"prim_path": str(xf.GetPath()), "category": kind,
                            "size_m": (L, W, h), "at": (cx, cy),
                            "yaw_deg": yaw, "count": npc,
                            "usd": sg._join_asset_root(str(f["asset"]), asset_root),
                            "note": f.get("note", "")})
                continue
        if _place_asset(stage, xf, f, asset_root, ssf):
            out.append({"prim_path": str(xf.GetPath()), "category": kind,
                        "size_m": (L, W, h), "at": (cx, cy), "yaw_deg": yaw,
                        "usd": sg._join_asset_root(str(f["asset"]), asset_root),
                        "note": f.get("note", "")})
            continue
        mesh_path = f"{parent_path}/{name}/{name}_box"
        if f.get("poly"):
            # AN OUTLINE GETS EXTRUDED, NOT BOXED. A rubble field is concave,
            # so its bounding box is half again its area and stands as a
            # plateau over ground that is not debris. The traced outline is the
            # measurement; a box would throw it away at the last step.
            _extrude(stage, mesh_path, f["poly"], (cx, cy), h, ssf,
                     COLOUR.get(kind))
        else:
            sg._make_box_mesh(stage, mesh_path,
                              -L / 2.0, -W / 2.0, L / 2.0, W / 2.0,
                              0.0, h, ssf, display_color=COLOUR.get(kind))
        # A STAND-IN CAN STILL HAVE THE RIGHT SURFACE. `kind_materials` binds a
        # real ground material to a prism whose SHAPE is a guess but whose
        # material is known — the sand pit is measured off the aerial and is
        # unambiguously sand, so rendering it as a flat tan box threw away
        # something the source image actually says.
        mp = (kind_materials or {}).get(kind)
        if mp:
            mat = UsdShade.Material.Get(stage, Sdf.Path(mp))
            if mat:
                mprim = stage.GetPrimAtPath(Sdf.Path(mesh_path))
                if mprim and mprim.IsValid():
                    UsdShade.MaterialBindingAPI.Apply(mprim)
                    UsdShade.MaterialBindingAPI(mprim).Bind(mat)
        # The record, on the prim: a viewport selection answers "what is this
        # and where did it come from" without regenerating anything. Same
        # contract as `stamp_asset_provenance` keeps for placed assets.
        prim = xf.GetPrim()
        prim.SetCustomDataByKey("featureKind", kind)
        prim.SetCustomDataByKey("featureNote", str(f.get("note", "")))
        prim.SetCustomDataByKey("featureSizeM", Gf.Vec3f(L, W, h))
        prim.SetCustomDataByKey("standIn", True)
        out.append({"prim_path": str(xf.GetPath()), "category": kind,
                    "size_m": (L, W, h), "at": (cx, cy), "yaw_deg": yaw,
                    "note": f.get("note", "")})
    if out:
        tally = {}
        for p in out:
            tally[p["category"]] = tally.get(p["category"], 0) + 1
        n_asset = sum(1 for p in out if p.get("usd"))
        n_scat = sum(p.get("count", 0) for p in out if p.get("count"))
        extra = f", {n_scat} scattered pieces" if n_scat else ""
        print(f"[site_features] {len(out)} features "
              f"({n_asset} library assets, {len(out) - n_asset} stand-in prisms"
              f"{extra}): "
              + "  ".join(f"{k}={v}" for k, v in sorted(tally.items())))
    return out


def _extrude(stage, path, poly, origin, h, ssf, colour):
    """A closed prism over *poly*, in the parent Xform's local frame.

    Cap and floor are ear-clipped (`suburb_net.earclip`); the walls are a quad
    per edge. Points are written relative to *origin* because the parent Xform
    already carries the translate — the outline in the spec is in world metres.
    """
    ox, oy = origin
    rings = sn.simple_rings([(x - ox, y - oy) for x, y in poly])
    if not rings:
        return None
    pts, counts, idx = [], [], []
    for ring in rings:
        base = len(pts)
        n = len(ring)
        for (x, y) in ring:
            pts.append(Gf.Vec3f(x * ssf, y * ssf, 0.0))
        for (x, y) in ring:
            pts.append(Gf.Vec3f(x * ssf, y * ssf, h * ssf))
        for (a, b, c) in sn.earclip(ring):
            counts.append(3)
            idx.extend([base + n + a, base + n + b, base + n + c])   # cap, up
            counts.append(3)
            idx.extend([base + c, base + b, base + a])               # floor
        for k in range(n):
            j = (k + 1) % n
            counts.append(4)
            idx.extend([base + k, base + j, base + n + j, base + n + k])
    if not counts:
        return None
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    mesh.CreateSubdivisionSchemeAttr("none")
    if colour:
        mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*colour)]))
    return mesh


def _rect_of(f):
    """Extent of a feature given only an outline — its axis-aligned bounds.

    A `rubble` field carries a traced polygon and no `size_m`. Its box is the
    bounds of that polygon, which is the honest prism for it; the outline
    itself stays in the spec for the debris pass that will eventually consume
    it instead of a box.
    """
    xs = [p[0] for p in f["poly"]]
    ys = [p[1] for p in f["poly"]]
    return (max(xs) - min(xs), max(ys) - min(ys))


def _sanitize(s):
    return "".join(c if c.isalnum() else "_" for c in str(s)) or "feature"
