"""source — one asset, resolved and read into plain arrays.

Front end for the damage pipeline: turns an asset-pack string
(``objaverse://``, ``omniverse://``, ``airstack://`` or a path) into triangles,
UVs, per-triangle material ids and the ``Material`` prims they refer to.

Nothing here needs Isaac Sim except ``omniverse://`` resolution, which Kit does
for free once a stage exists."""

from __future__ import annotations

import numpy as np
import trimesh

# ---------------------------------------------------------------------------
# Source loading — USD, and Kit for `omniverse://`
# ---------------------------------------------------------------------------


class Source:
    """Triangles of the source asset, in world space, with appearance."""

    def __init__(self, vertices, faces, tri_uv, tri_mat, mat_paths):
        self.vertices, self.faces = vertices, faces
        self.tri_uv, self.tri_mat, self.mat_paths = tri_uv, tri_mat, mat_paths
        self.parts = None   # closed solid parts, set by `_load` after thickening


def resolve_asset(spec: str, target_size: float) -> str:
    """An asset-pack asset string to something `Usd.Stage.Open` accepts.

    Delegates to the generator's own resolver so `objaverse://` etc. mean here
    exactly what they mean in a config, and triggers the same download-and-
    convert path the scene generator relies on.
    """
    import re

    from scene_generator import _expand_scheme

    uid = re.match(r"^objaverse://([0-9a-fA-F]{32})$", spec.strip())
    if uid:
        import objaverse_assets
        objaverse_assets.ensure(uid.group(1), target_size_m=target_size)
    return _expand_scheme(spec) or spec


def _triangulate(counts, indices):
    """Fan-triangulate USD face arrays; returns triangles and their source face."""
    tris, owner, base = [], [], 0
    for f, c in enumerate(counts):
        for j in range(1, c - 1):
            tris.append((indices[base], indices[base + j], indices[base + j + 1]))
            owner.append((base, base + j, base + j + 1))
        base += c
    return np.array(tris, dtype=np.int64), np.array(owner, dtype=np.int64)


def load_source(stage, root_path: str, asset: str, target_size: float,
                up_axis: str = "z", quiet=False, scale: float = 0.0) -> Source:
    """Reference *asset* under *root_path* (invisible) and extract its triangles.

    Referencing rather than copying is what preserves appearance: the asset's
    `Material` prims land on the stage with their shader networks and texture
    paths intact, and the chunks simply bind to them.
    """
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

    src = UsdGeom.Xform.Define(stage, root_path)
    src.GetPrim().GetReferences().AddReference(asset)

    if up_axis.lower() == "y":
        # Y-up asset onto a Z-up stage.
        src.AddRotateXOp().Set(90.0)

    verts, faces, uvs, mats, mat_paths = [], [], [], [], []
    mat_index = {}
    for prim in Usd.PrimRange(src.GetPrim()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts:
            continue

        xf = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()))
        p = np.array(pts, dtype=np.float64)
        p = p @ xf[:3, :3] + xf[3, :3]

        tris, corner = _triangulate(np.array(counts), np.array(idx))
        if not len(tris):
            continue

        st = _read_st(prim, len(p), np.array(idx))
        # `corner` indexes face-varying slots; for vertex/constant `st` the
        # reader already expanded to one value per slot, so this is uniform.
        tri_uv = st[corner] if st is not None else np.zeros((len(tris), 3, 2))

        bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        key = str(bound.GetPath()) if bound else ""
        if key not in mat_index:
            mat_index[key] = len(mat_paths)
            mat_paths.append(Sdf.Path(key) if key else None)

        faces.append(tris + sum(len(v) for v in verts))
        verts.append(p)
        uvs.append(tri_uv)
        mats.append(np.full(len(tris), mat_index[key]))

    if not verts:
        raise RuntimeError(f"no meshes found in {asset}")

    # Measured off the extracted geometry rather than a UsdGeom.BBoxCache:
    # the cache reported an empty range for a reference added moments earlier
    # and every asset silently came through at scale 1.0. The vertices are
    # already in hand and cannot disagree with what actually gets fractured.
    points = np.vstack(verts)
    plan = np.ptp(points, axis=0)[:2].max()
    if scale and scale > 0:
        # An explicit multiplier wins over normalising to a target size. Asset
        # sets state size that way — `scale: 0.01` for a centimetre-authored
        # Nucleus asset — and it has to be honoured rather than overridden, or
        # every building in a pack comes out the same size as every other and
        # a tower reads as a bungalow.
        scale = float(scale)
        points = points * scale
    elif target_size > 0 and plan > 0:
        scale = target_size / float(plan)
        points = points * scale
    else:
        scale = 1.0

    # THE STAGE HAS TO AGREE WITH THE POINTS. Everything above rescales the
    # numpy array only, but the prim under `root_path` is still the reference
    # at its AUTHORED size — and `earthquake.shake` hands that PRIM PATH to
    # `solids.thicken`, which re-reads the geometry off the stage. So the two
    # halves of one pipeline disagreed by exactly the scale factor.
    #
    # On an objaverse asset authored near 1 m that is a small error nobody
    # noticed. On a centimetre-authored Nucleus building (`scale: 0.01`) it is
    # 100x: `solids.visible_faces` sizes its ray grid as radius/spacing, so a
    # 96 m tower read as 9,612 units asked for a (596624, 596624) meshgrid and
    # died trying to allocate 2.59 TiB. It only stays hidden on assets where
    # `close_directly` succeeds and `thicken` never runs.
    if scale != 1.0:
        src.AddScaleOp().Set(Gf.Vec3f(float(scale), float(scale), float(scale)))

    UsdGeom.Imageable(src).MakeInvisible()
    if not quiet:
        print(f"[fracture] {asset}: {len(verts)} meshes, "
              f"{sum(len(f) for f in faces)} tris, {len(mat_paths)} materials, "
              f"scale x{scale:.4g}", flush=True)
    return Source(points, np.vstack(faces), np.vstack(uvs),
                  np.concatenate(mats), mat_paths)


def _read_st(prim, n_points, indices):
    """The mesh's texture coordinates, expanded to one per face-varying slot."""
    from pxr import UsdGeom

    for pv in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
        if "TexCoord2f" not in str(pv.GetTypeName()):
            continue
        vals = pv.Get()
        if not vals:
            continue
        vals = np.array(vals, dtype=np.float64)
        interp = pv.GetInterpolation()
        pv_idx = pv.GetIndices() if pv.IsIndexed() else None
        if pv_idx:
            vals = vals[np.array(pv_idx)]
        if interp == UsdGeom.Tokens.faceVarying:
            return vals
        if interp == UsdGeom.Tokens.vertex or interp == UsdGeom.Tokens.varying:
            return vals[indices] if len(vals) == n_points else None
    return None


def cylinder_source(radius=2.0, height=6.0, sections=64) -> Source:
    """The default subject: a plain closed cylinder, no textures."""
    m = trimesh.creation.cylinder(radius=radius, height=height,
                                  sections=sections)
    m.apply_translation((0.0, 0.0, height * 0.5))
    return Source(np.array(m.vertices), np.array(m.faces),
                  np.zeros((len(m.faces), 3, 2)),
                  np.zeros(len(m.faces), dtype=np.int64), [None])


def _mesh_soup(stage, root_path):
    """Every mesh under *root_prim* as one welded triangle soup, world space.

    Geometry only — no UVs. The appearance reference is the *pristine* source
    read before thickening; this is only the thing that gets cut.
    """
    from pxr import Usd, UsdGeom

    root = stage.GetPrimAtPath(root_path) if isinstance(root_path, str) \
        else root_path
    verts, faces, off = [], [], 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts:
            continue
        xf = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()))
        p = np.array(pts, dtype=np.float64) @ xf[:3, :3] + xf[3, :3]
        tris, _ = _triangulate(np.array(counts), np.array(idx))
        if not len(tris):
            continue
        faces.append(tris + off)
        verts.append(p)
        off += len(p)
    if not verts:
        return None
    soup = trimesh.Trimesh(np.vstack(verts), np.vstack(faces), process=False)
    # Welding is not optional: assets arrive with vertices split along every UV
    # seam, so without it the "same" wall is thousands of disjoint scraps and
    # nothing is closed (measured: 4.8% of components watertight unwelded,
    # 47% welded).
    soup.merge_vertices()
    return soup
