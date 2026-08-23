"""authoring — chunks, rubble and materials onto a USD stage.

Pure ``pxr``; nothing here imports Isaac Sim, so a stage can be built and
inspected on the host. Physics APIs are authored but not stepped — that is the
caller's business."""

from __future__ import annotations

import numpy as np
import trimesh


# Interior (fracture-face) look. The folder holds texture maps only — no .mdl
# and no .usd — so the material is built here against the JPGs in place.
# Kit resolves `omniverse://` at render time; nothing is downloaded.
INTERIOR_TEXTURE = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/"
                    "SEI-COA/StandaloneMaterials/OSB/osb_board_tjzjedni_2k/"
                    "OSB_Board_tjzjedni_2K_BaseColor.jpg")


INTERIOR_ROUGHNESS = INTERIOR_TEXTURE.replace("BaseColor", "Roughness")


def add_lighting(stage, path="/World/Lights"):
    """A dome plus a key light.

    `World` brings no lights of its own, so an unlit stage renders the object
    as a black silhouette against the ground grid — which reads as broken
    geometry rather than as missing lighting.
    """
    from pxr import Gf, UsdGeom, UsdLux

    UsdGeom.Xform.Define(stage, path)
    dome = UsdLux.DomeLight.Define(stage, path + "/Dome")
    dome.CreateIntensityAttr(900.0)
    dome.CreateColorAttr(Gf.Vec3f(0.85, 0.9, 1.0))

    key = UsdLux.DistantLight.Define(stage, path + "/Key")
    key.CreateIntensityAttr(2500.0)
    key.CreateAngleAttr(1.5)
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 35.0))
    return dome


def _textured_material(stage, path, colour_url, rough_url=""):
    """UsdPreviewSurface reading `st`, for the freshly-cut interior faces."""
    from pxr import Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")

    reader = UsdShade.Shader.Define(stage, path + "/stReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    uv = reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    def sample(name, url, out_type, channel):
        tex = UsdShade.Shader.Define(stage, f"{path}/{name}")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(url)
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(uv)
        for wrap in ("wrapS", "wrapT"):
            tex.CreateInput(wrap, Sdf.ValueTypeNames.Token).Set("repeat")
        return tex.CreateOutput(channel, out_type)

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f) \
        .ConnectToSource(sample("diffuse", colour_url,
                                Sdf.ValueTypeNames.Float3, "rgb"))
    if rough_url:
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float) \
            .ConnectToSource(sample("rough", rough_url,
                                    Sdf.ValueTypeNames.Float, "r"))
    else:
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(
        shader.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    return mat


def _flat_material(stage, path, color):
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _resolve_material(stage, src, cache, default_mat):
    """`UsdShade.Material.Get(stage, src)`, memoized, falling back to *default*.

    Called once per (chunk, material-id) pair, and a pack typically reuses a
    handful of materials across every chunk of every asset — without the
    cache this re-resolved the same handful of stage paths hundreds of times
    over one ruin.
    """
    from pxr import UsdShade

    if src not in cache:
        target = UsdShade.Material.Get(stage, src) if src else None
        cache[src] = target if (target and target.GetPrim().IsValid()) \
            else default_mat
    return cache[src]


def _bind_subsets(stage, mesh, face_mat, mat_paths, inner_mat, default_mat,
                  cache):
    """Bind cut faces to the interior material and the rest to their source."""
    from pxr import UsdGeom, UsdShade

    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
    for value in np.unique(face_mat):
        idx = np.nonzero(face_mat == value)[0]
        if value < 0:
            target = inner_mat
            name = "interior"
        else:
            src = mat_paths[value] if value < len(mat_paths) else None
            target = _resolve_material(stage, src, cache, default_mat)
            name = f"mat_{value:02d}"
        subset = UsdGeom.Subset.CreateGeomSubset(
            mesh, name, UsdGeom.Tokens.face, idx.tolist(),
            familyName="materialBind")
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(target)


# ---------------------------------------------------------------------------
# USD authoring
# ---------------------------------------------------------------------------


def author_chunks(stage, root_path, chunks, mat_paths, density=2400.0,
                  interior_texture=INTERIOR_TEXTURE):
    """One rigid-body Mesh prim per chunk, split into per-material subsets."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    UsdGeom.Xform.Define(stage, root_path)
    if interior_texture:
        inner_mat = _textured_material(
            stage, root_path + "/Interior", interior_texture,
            interior_texture.replace("BaseColor", "Roughness"))
    else:
        inner_mat = _flat_material(stage, root_path + "/Interior",
                                   (0.93, 0.93, 0.93))
    default_mat = _flat_material(stage, root_path + "/Concrete", (0.62, 0.60, 0.57))
    mat_cache = {}

    for i, c in enumerate(chunks):
        path = f"{root_path}/chunk_{i:03d}"
        mesh = UsdGeom.Mesh.Define(stage, path)
        faces = c["faces"]
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in c["points"]])
        mesh.CreateFaceVertexCountsAttr([3] * len(faces))
        mesh.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateExtentAttr(
            [Gf.Vec3f(*c["points"].min(0)), Gf.Vec3f(*c["points"].max(0))])
        # Translate AND orient, even though the orient starts as identity:
        # PhysX will rotate these bodies, and an op has to exist before a
        # settled pose can be written back for export.
        xf = UsdGeom.Xformable(mesh)
        xf.AddTranslateOp().Set(Gf.Vec3d(*c["centroid"]))
        xf.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        st.Set([Gf.Vec2f(*v) for v in c["uv"].reshape(-1, 2)])

        _bind_subsets(stage, mesh, c["face_mat"], mat_paths, inner_mat,
                      default_mat, mat_cache)

        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        # Chunks of a non-convex object are themselves non-convex, so the
        # collider cannot be a single hull the way it could for the cylinder.
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()) \
            .CreateApproximationAttr().Set(UsdPhysics.Tokens.convexDecomposition)
        body = UsdPhysics.RigidBodyAPI.Apply(mesh.GetPrim())
        body.CreateKinematicEnabledAttr(True)   # held — see `set_kinematic`
        UsdPhysics.MassAPI.Apply(mesh.GetPrim()).CreateDensityAttr(density)


def set_kinematic(stage, root_path, n, on: bool):
    """Flip every chunk between kinematic (held) and dynamic (falling).

    PhysX rebuilds the body on the transition, which is why the caller steps
    once before assigning velocities.
    """
    from pxr import UsdPhysics

    for i in range(n):
        prim = stage.GetPrimAtPath(f"{root_path}/chunk_{i:03d}")
        if prim:
            UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(on)


# ---------------------------------------------------------------------------
# Authoring
# ---------------------------------------------------------------------------


def author_ruin(stage, root, chunks, mat_paths, damage, mats, dirt, rng,
                density=2000.0):
    """Chunks as rigid bodies, cut faces coloured by material plus noise.

    Cut faces carry `primvars:displayColor` and are deliberately left with no
    material bound, so the renderer falls back to that colour — which is how
    the per-face noise shows at all. Exterior faces keep the source materials
    over the top of it.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    UsdGeom.Xform.Define(stage, root)
    default_mat = _flat_material(stage, root + "/Fallback", (0.6, 0.6, 0.6))
    mat_cache = {}

    for i, c in enumerate(chunks):
        path = f"{root}/chunk_{i:04d}"
        mesh = UsdGeom.Mesh.Define(stage, path)
        faces = c["faces"]
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in c["points"]])
        mesh.CreateFaceVertexCountsAttr([3] * len(faces))
        mesh.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateExtentAttr(
            [Gf.Vec3f(*c["points"].min(0)), Gf.Vec3f(*c["points"].max(0))])
        # Translate AND orient, even though the orient starts as identity:
        # PhysX will rotate these bodies, and an op has to exist before a
        # settled pose can be written back for export.
        xf = UsdGeom.Xformable(mesh)
        xf.AddTranslateOp().Set(Gf.Vec3d(*c["centroid"]))
        xf.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        st.Set([Gf.Vec2f(*v) for v in c["uv"].reshape(-1, 2)])

        # Stage 4: the grime layer. Damage darkens, noise breaks it up.
        #
        # The noise is mostly on BRIGHTNESS, with only a trace of colour.
        # Perturbing R, G and B independently moves the hue rather than the
        # shade, and on a saturated colour like brick that is violent: at one
        # sigma of 0.05 the fragments came out magenta and olive. Grime varies
        # how light a surface is, not what colour it is.
        base = mats[i].colour * (1.0 - 0.35 * damage[i])
        shade = 1.0 + rng.normal(0.0, 2.0 * dirt, (len(faces), 1))
        tint = rng.normal(0.0, 0.2 * dirt, (len(faces), 3))
        cols = np.clip(base[None, :] * shade + tint, 0.0, 1.0)
        dc = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "displayColor", Sdf.ValueTypeNames.Color3fArray,
            UsdGeom.Tokens.uniform)
        dc.Set([Gf.Vec3f(*v) for v in cols])

        # Only the surviving original faces get a material; cut faces are left
        # bare so displayColor is what renders on them.
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
        for value in np.unique(c["face_mat"]):
            if value < 0:
                continue
            src = mat_paths[value] if value < len(mat_paths) else None
            target = _resolve_material(stage, src, mat_cache, default_mat)
            idx = np.nonzero(c["face_mat"] == value)[0]
            subset = UsdGeom.Subset.CreateGeomSubset(
                mesh, f"mat_{value:02d}", UsdGeom.Tokens.face, idx.tolist(),
                familyName="materialBind")
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(target)

        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()) \
            .CreateApproximationAttr().Set(UsdPhysics.Tokens.convexDecomposition)
        UsdPhysics.RigidBodyAPI.Apply(mesh.GetPrim()) \
            .CreateKinematicEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(mesh.GetPrim()).CreateDensityAttr(density)


def author_rubble(stage, root, field, mats, severity, fallen, lo, hi, rng):
    """Solid-colour blocks around the base, more where more came down.

    Count follows the volume that actually detached rather than severity alone,
    so a building that held together does not spill a debris field it never
    produced. Placement is weighted by the damage projected down to the ground,
    which puts the pile under the part that collapsed.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    if fallen <= 0.0:
        return 0
    UsdGeom.Xform.Define(stage, root)

    # `fallen` is the fraction of fragments that came down, so the debris
    # field tracks what actually collapsed rather than the severity dial.
    n = int(np.clip(round(fallen * 150.0 * (0.3 + severity)), 0, 400))
    ground = field.grid.max(axis=2)                      # damage seen from above
    flat = ground.ravel()
    if flat.sum() <= 0:
        return 0
    cells = rng.choice(len(flat), size=n, p=flat / flat.sum())
    gy, gx = np.divmod(cells, field.res)
    step = (hi - lo)[:2] / max(field.res - 1, 1)

    count = 0
    for k in range(n):
        mat = mats[rng.integers(len(mats))]
        size, aspect = mat.block
        size *= rng.uniform(0.6, 1.5)
        dims = np.array([size * aspect, size, size * rng.uniform(0.5, 1.0)])
        # Spread outward from the footprint: debris lands beside a building,
        # not inside its footprint.
        centre = lo[:2] + np.array([gy[k], gx[k]]) * step
        centre = centre + rng.normal(0.0, 0.35 + 0.9 * severity, 2)
        box = trimesh.creation.box(dims)
        box.apply_transform(trimesh.transformations.random_rotation_matrix(
            rng.random(3)))
        drop = box.bounds[1][2] - box.bounds[0][2]
        pos = np.array([centre[0], centre[1],
                        lo[2] + drop * 0.5 + rng.uniform(0.0, 1.2 * severity)])

        path = f"{root}/rubble_{k:04d}"
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in box.vertices])
        mesh.CreateFaceVertexCountsAttr([3] * len(box.faces))
        mesh.CreateFaceVertexIndicesAttr(box.faces.ravel().tolist())
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateExtentAttr([Gf.Vec3f(*box.bounds[0]), Gf.Vec3f(*box.bounds[1])])
        xf = UsdGeom.Xformable(mesh)
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        xf.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        col = np.clip(mat.colour * (1.0 + rng.normal(0.0, 0.12)), 0.0, 1.0)
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "displayColor", Sdf.ValueTypeNames.Color3fArray,
            UsdGeom.Tokens.constant).Set([Gf.Vec3f(*col)])

        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()) \
            .CreateApproximationAttr().Set(UsdPhysics.Tokens.convexHull)
        UsdPhysics.RigidBodyAPI.Apply(mesh.GetPrim())
        UsdPhysics.MassAPI.Apply(mesh.GetPrim()).CreateDensityAttr(1800.0)
        count += 1
    return count


def bake_poses(stage, paths, positions, orientations):
    """Write settled world poses back onto the prims.

    PhysX keeps its results in Fabric, and whether they reach the USD layer
    depends on runtime settings — so an exported file can silently contain the
    poses the bodies had BEFORE the simulation. Writing them explicitly makes
    the export say what the viewer showed.
    """
    from pxr import Gf, UsdGeom

    for path, pos, quat in zip(paths, positions, orientations):
        prim = stage.GetPrimAtPath(path)
        if not prim:
            continue
        ops = {op.GetOpName(): op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}
        for name, op in ops.items():
            if "translate" in name:
                op.Set(Gf.Vec3d(*[float(v) for v in pos]))
            elif "orient" in name:
                w, x, y, z = [float(v) for v in quat]
                op.Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))


def content_bounds(stage, root="/World"):
    """World-space `(lo, hi)` of everything visible under *root*, or None.

    For framing a camera. Sizing a view from a nominal grid pitch instead goes
    wrong the moment one item is an outlier — a single 30 m sign among 8 m
    houses pushes the pitch out until every house is a few pixels.
    """
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(root)
    if not prim:
        return None
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        return None
    return np.array(rng.GetMin()), np.array(rng.GetMax())
