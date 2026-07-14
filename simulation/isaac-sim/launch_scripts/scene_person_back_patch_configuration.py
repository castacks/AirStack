#!/usr/bin/env python
"""
Config-driven person-tracking scene loader with an albedo patch on the PERSON.

This is a sibling of ``scene_drone_person_configuration.py``. That script paints
a textured USD patch onto a cube / board in the *scene*. This variant instead
targets the adversarial patch that is baked (skinned) onto the *person* -- e.g.
the plane mesh that ships inside
``assets/characters/F_Business_02/F_Business_02_with_skinned_patch.usd`` at (per
the Blender export) ``/root/Plane/Plane`` -- and binds an OmniPBR albedo image
onto that existing mesh so the patch is worn on the person's back and moves /
deforms with them.

It reuses everything from ``scene_drone_person_configuration`` (scene loading,
drone spawn, person controller, collision helpers, texture transforms). Importing
that module also boots the shared ``SimulationApp`` with the correct headless /
streaming settings, so this file must import it before touching any omni / pxr API.

What this script adds over the base script:
  * A ``person_patch`` config section (see below). When enabled, after the person
    is spawned the script locates the patch Mesh inside the spawned person prim
    and binds an OmniPBR material carrying ``person_patch.albedo_path`` to it.
  * Locating the patch mesh is robust to naming: give an explicit path, or let the
    script search the person's subtree for a Mesh named ``Plane`` / ``BackPatch``
    (configurable). Every Mesh found under the person is logged so an unexpected
    layout is easy to diagnose.

The base ``patch`` (scene-cube) section still works and is orthogonal: leave
``patch.enabled: false`` for a pure person-worn-patch run, or enable both to place
a scene patch *and* a person patch in the same launch.

Config file:
    Path is taken from SCENE_CONFIG_PATH (default:
    .../launch_scripts/configs/person_back/person_back.yaml). Use the
    in-container path (the repo is mounted at /isaac-sim/AirStack).

Env overrides (optional, win over the config file when set):
    PERSON_PATCH_ALBEDO_PATH         overrides person_patch.albedo_path
    (plus every override honoured by scene_drone_person_configuration:
     SCENE_USD_PATH / ISAAC_SIM_GUI, MESH_PLANE_SIZE_SEED,
     MESH_PLANE_POSITION_SEED, MESH_PLANE_ALBEDO_PATH, PLAY_SIM_ON_START)

Streaming:
    Set ISAAC_SIM_HEADLESS=true to run headless with WebRTC streaming on port 49100.
"""

import datetime
import json
import os

# Importing the base module instantiates the shared SimulationApp (honouring
# ISAAC_SIM_HEADLESS) and imports all omni / pxr APIs. Do this before using any
# of those APIs below. main() is guarded there, so importing does not launch it.
import scene_drone_person_configuration as base

# Pull the already-imported heavy dependencies off the base module rather than
# re-importing them, so this script uses the exact same instances the app booted.
carb = base.carb
np = base.np
Gf = base.Gf
Sdf = base.Sdf
Usd = base.Usd
UsdGeom = base.UsdGeom
UsdShade = base.UsdShade
OmniPBR = base.OmniPBR
simulation_app = base.simulation_app


_DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "configs",
    "person_back",
    "person_back.yaml",
)


def _texture_transform_is_identity(texture_transform):
    """True when the transform leaves texture coordinates unchanged."""
    return (
        not texture_transform["flip_horizontal"]
        and not texture_transform["flip_vertical"]
        and float(texture_transform["rotation_deg"]) == 0.0
    )


def list_meshes_under(stage, root_path):
    """Return every Mesh prim (including instance proxies) below root_path."""
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return []
    return [
        prim
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
        if prim.IsA(UsdGeom.Mesh)
    ]


def find_person_patch_mesh(stage, person_root_path, patch_cfg):
    """Locate the patch Mesh inside the spawned person prim.

    Resolution order:
      1. person_patch.abs_path  -- an absolute stage path to the Mesh.
      2. person_patch.patch_path -- a path relative to the person root (e.g.
         "Plane/Plane" or "BackPatch").
      3. person_patch.patch_names -- search the person subtree for the first
         Mesh whose leaf name matches one of these (default: Plane, BackPatch).

    Every Mesh found under the person is logged to make an unexpected character
    layout easy to diagnose.
    """
    abs_path = base._get(patch_cfg, "abs_path")
    if abs_path:
        prim = stage.GetPrimAtPath(str(abs_path))
        if prim and prim.IsValid() and prim.IsA(UsdGeom.Mesh):
            return prim
        raise RuntimeError(
            f"person_patch.abs_path is not a Mesh on the stage: {abs_path}"
        )

    meshes = list_meshes_under(stage, person_root_path)
    mesh_paths = [prim.GetPath().pathString for prim in meshes]
    carb.log_warn(
        f"Person '{person_root_path}' contains {len(meshes)} Mesh prim(s): {mesh_paths}"
    )

    rel_path = base._get(patch_cfg, "patch_path")
    if rel_path:
        full_path = person_root_path.rstrip("/") + "/" + str(rel_path).lstrip("/")
        prim = stage.GetPrimAtPath(full_path)
        if prim and prim.IsValid() and prim.IsA(UsdGeom.Mesh):
            return prim
        raise RuntimeError(
            f"person_patch.patch_path did not resolve to a Mesh: {full_path}. "
            f"Meshes under the person: {mesh_paths}"
        )

    name_candidates = base._get(patch_cfg, "patch_names", ["Plane", "BackPatch"])
    if not isinstance(name_candidates, (list, tuple)) or not name_candidates:
        raise RuntimeError("person_patch.patch_names must be a non-empty list.")
    for candidate in name_candidates:
        for prim in meshes:
            if prim.GetName() == str(candidate):
                return prim
    raise RuntimeError(
        f"Could not find a person patch Mesh named any of {list(name_candidates)} "
        f"under {person_root_path}. Set person_patch.patch_path or .abs_path. "
        f"Meshes under the person: {mesh_paths}"
    )


def author_full_quad_uvs(mesh_prim, texture_transform):
    """Author full-span UVs on a single-quad patch, applying texture_transform.

    Returns True if the mesh is a single quad and UVs were authored, else False
    (the caller falls back to transforming whatever UVs the mesh already has).
    """
    mesh = UsdGeom.Mesh(mesh_prim)
    counts = mesh.GetFaceVertexCountsAttr().Get()
    if not counts or len(counts) != 1 or int(counts[0]) != 4:
        return False
    base_uvs = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
    st_values = base.transform_texture_uvs(base_uvs, texture_transform)
    primvars = UsdGeom.PrimvarsAPI(mesh_prim)
    st = primvars.GetPrimvar("st")
    if not st:
        st = primvars.CreatePrimvar(
            "st",
            Sdf.ValueTypeNames.TexCoord2fArray,
            UsdGeom.Tokens.faceVarying,
        )
    else:
        st.SetInterpolation(UsdGeom.Tokens.faceVarying)
    st.Set(st_values)
    st.SetIndices([0, 1, 2, 3])
    return True


def transform_existing_uvs(mesh_prim, texture_transform):
    """Apply texture_transform in place to the mesh's existing 'st' primvar.

    Returns True if a value-carrying 'st' primvar was found and rewritten.
    """
    primvars = UsdGeom.PrimvarsAPI(mesh_prim)
    st = primvars.GetPrimvar("st")
    if not st or not st.HasValue():
        return False
    values = st.Get()
    uvs = [[float(value[0]), float(value[1])] for value in values]
    st.Set(base.transform_texture_uvs(uvs, texture_transform))
    return True


def bind_albedo_material(stage, mesh_prim, material_path, albedo_path, texture_scale, color):
    """Create an OmniPBR material carrying the albedo and bind it to mesh_prim.

    Mirrors the shader setup used by the scene patch in
    scene_drone_person_configuration.create_textured_mesh_plane, but binds the
    result to an *existing* (skinned) mesh with strongerThanDescendants so it
    overrides any material the injected patch already carried.
    """
    material_kwargs = {
        "prim_path": material_path,
        "name": "person_back_patch_omnipbr",
        "color": np.array([float(component) for component in color]),
        "texture_scale": [1.0, 1.0],
        "texture_translate": [0.0, 0.0],
    }
    if albedo_path:
        material_kwargs["texture_path"] = albedo_path
    OmniPBR(**material_kwargs)

    material = UsdShade.Material.Get(stage, material_path)
    for child in material.GetPrim().GetChildren():
        if child.GetTypeName() != "Shader":
            continue
        shader = UsdShade.Shader(child)
        shader.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(False)
        shader.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(False)
        shader.CreateInput("uv_space_index", Sdf.ValueTypeNames.Int).Set(0)
        shader.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2).Set(
            Gf.Vec2f(*[float(value) for value in texture_scale])
        )
        shader.CreateInput("texture_translate", Sdf.ValueTypeNames.Float2).Set(
            Gf.Vec2f(0.0, 0.0)
        )
        shader.CreateInput("texture_rotate", Sdf.ValueTypeNames.Float).Set(0.0)

    binding = UsdShade.MaterialBindingAPI.Apply(mesh_prim)
    binding.Bind(material, UsdShade.Tokens.strongerThanDescendants)
    return material


class PersonBackPatchScene(base.ConfigurableTrackScene):
    """Base configurable scene plus an albedo patch applied to the person mesh."""

    def __init__(self, config):
        # The base __init__ loads the scene, spawns the drone, spawns the person
        # (whose USD already contains the skinned patch mesh), and optionally
        # creates the scene-cube patch. After it returns, self.person exists.
        super().__init__(config)
        self._maybe_apply_person_patch()

    def _maybe_apply_person_patch(self):
        patch_cfg = base._get(self.config, "person_patch", {})
        if not bool(base._get(patch_cfg, "enabled", False)):
            return

        # Let the person's referenced USD (and its patch mesh) finish composing
        # before we search for and edit it.
        base.wait_for_stage_updates()

        albedo_path = os.environ.get(
            "PERSON_PATCH_ALBEDO_PATH",
            base._get(patch_cfg, "albedo_path", ""),
        )
        material_path = self._rebase_generated_path(
            base._get(patch_cfg, "material_path", "/World/Looks/PersonBackPatchOmniPBR")
        )
        texture_scale = base._get(patch_cfg, "texture_scale", [1.0, 1.0])
        color = base._get(patch_cfg, "color", [2.0, 2.0, 2.0])
        texture_transform = base.resolve_texture_transform(patch_cfg)

        person_root = self.person._stage_prefix
        mesh_prim = find_person_patch_mesh(self.stage, person_root, patch_cfg)
        mesh_path = mesh_prim.GetPath().pathString

        # UV handling: for the common single-quad Blender plane we can author
        # full-span UVs so one albedo image fills the patch; otherwise we respect
        # the mesh's own UVs and only apply a requested texture_transform to them.
        set_full_uvs = bool(base._get(patch_cfg, "set_full_uvs", False))
        identity = _texture_transform_is_identity(texture_transform)
        if set_full_uvs:
            if not author_full_quad_uvs(mesh_prim, texture_transform):
                carb.log_warn(
                    f"person_patch.set_full_uvs requested but {mesh_path} is not a "
                    "single quad; keeping its authored UVs."
                )
                if not identity and not transform_existing_uvs(mesh_prim, texture_transform):
                    carb.log_warn(
                        f"{mesh_path} has no 'st' primvar to apply texture_transform to."
                    )
        elif not identity:
            if not transform_existing_uvs(mesh_prim, texture_transform):
                carb.log_warn(
                    f"person_patch.texture_transform requested but {mesh_path} has no "
                    "'st' primvar; the transform was skipped."
                )

        carb.log_warn(
            f"Binding person back patch: mesh={mesh_path}, "
            f"material={material_path}, albedo={albedo_path or '<none>'}, "
            f"texture_scale={texture_scale}, color={color}, "
            f"texture_transform={texture_transform}, set_full_uvs={set_full_uvs}"
        )
        bind_albedo_material(
            self.stage,
            mesh_prim,
            material_path,
            albedo_path,
            texture_scale,
            color,
        )
        base.add_update_semantics(mesh_prim, "patch_plane")

        # One record per launch mirroring the scene-patch bookkeeping: which
        # person mesh got which albedo, and the config that produced it.
        config_path = os.environ.get("SCENE_CONFIG_PATH", _DEFAULT_CONFIG_PATH)
        self._record_person_patch(
            base.resolve_output_dir(base._get(patch_cfg, "output_dir", "records")),
            {
                "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
                "scene": os.path.splitext(os.path.basename(self.scene_usd_path))[0]
                if self.scene_usd_path
                else None,
                "config_name": os.path.join(
                    os.path.basename(os.path.dirname(config_path)),
                    os.path.basename(config_path),
                ),
                "person": self.person_name,
                "person_patch_mesh": mesh_path,
                "material_path": material_path,
                "albedo_path": albedo_path or None,
            },
        )

    @staticmethod
    def _record_person_patch(output_dir, record):
        """Append a one-line JSON record of the person patch to output_dir."""
        os.makedirs(output_dir, exist_ok=True)
        record_path = os.path.join(output_dir, "person_patch_records.jsonl")
        with open(record_path, "a") as handle:
            handle.write(json.dumps(record) + "\n")
        carb.log_warn(f"Recorded person patch to {record_path}: {record}")
        return record_path


def load_config():
    """Load the YAML config, defaulting to the person_back example config."""
    config_path = os.path.expanduser(
        os.environ.get("SCENE_CONFIG_PATH", _DEFAULT_CONFIG_PATH)
    )
    if not os.path.isfile(config_path):
        raise RuntimeError(
            f"Scene config not found: {config_path}. Set SCENE_CONFIG_PATH to a "
            "YAML file (see launch_scripts/configs/person_back/person_back.yaml)."
        )
    with open(config_path, "r") as handle:
        config = base.yaml.safe_load(handle) or {}
    carb.log_warn(f"Loaded scene config: {config_path}")
    return config


def main():
    try:
        config = load_config()
        PersonBackPatchScene(config).run()
    except Exception as exc:
        carb.log_error(str(exc))
        simulation_app.close()
        raise


if __name__ == "__main__":
    main()
