"""
scene_props.py — borrow static props (cars, trucks, …) from one Nucleus stage
into another, and tag them with semantic labels so Replicator / the ROS 2
bridge can emit ground-truth bounding boxes for them.

Why references instead of copies
--------------------------------
The vehicles in ``ModernCityDowntown.stage.usd`` are ordinary USD prims:

    /Root/BP_MCar01                                  (Xform, positioned)
        └── SM_MCar01_Body                           → SM_MCar01_Body.usd
              ├── SM_MCar01_FL_Door … RR_Door        → SM_MCar01_*_Door.usd
              └── SM_MCar01_FL_WHL  … RR_WHL         → SM_MCar01_Wheel.usd

A *prim-level reference* (``AddReference(url, "/Root/BP_MCar01")``) pulls that
whole subtree — body, doors and wheels — into the live stage without copying
or modifying anything on Nucleus. That is what lets a scene such as
``downtown_edited_v3_818.usd``, which ships no vehicles at all (its
``prop_car_pillar*`` prims are parking-garage pillars), borrow ModernCity's.

Two details this module handles for you:

* **The donor transform comes along with the reference.** ``BP_MCar01`` sits
  at (-64.9, -60.1) in ModernCity, so a naive reference lands the car there.
  Each vehicle is therefore parented under a wrapper Xform whose op order we
  own, and the donor's ops are cleared on the referenced child.
* **Units differ per stage.** Rather than hard-coding a scale, the asset's
  bounding box is measured after referencing and scaled to a target real-world
  length (``length_m``), so placements are always given in world metres.

Functions
---------
    add_vehicles           — reference + place + label a batch of vehicles
    add_vehicle            — single-vehicle version of the above
    snap_prims_to_ground   — PhysX raycast so props rest on the road surface
    label_prim             — semantic label shim (Isaac 5.x new/old API)
    label_prims_matching   — label props already present in the scene
    list_source_prims      — discover borrowable prims in a donor stage
"""

import math
import re

import omni.kit.app
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

try:                                    # imported as a package
    from .scene_prep import add_colliders
except ImportError:                     # imported flat (launch scripts add utils/ to sys.path)
    from scene_prep import add_colliders


# ---------------------------------------------------------------------------
# Catalog of borrowable vehicles
# ---------------------------------------------------------------------------
# Prim paths verified against
# omniverse://airlab-nucleus.andrew.cmu.edu/Library/Stages/Dmytro/ModernCityDowntown.stage.usd
#
# The six BP_MCar* prims all reference the same SM_MCar01_* meshes, so they are
# the same sedan model at different donor positions — the variety is in where
# *you* place them, not in the mesh. `length_m` is the real-world length the
# asset is scaled to; tweak it to fake a bigger/smaller vehicle.
DEFAULT_VEHICLE_LIBRARY = (
    "omniverse://airlab-nucleus.andrew.cmu.edu"
    "/Library/Stages/Dmytro/ModernCityDowntown.stage.usd"
)

VEHICLE_CATALOG = {
    "car":    {"prim": "/Root/BP_MCar01", "semantic_class": "car",   "length_m": 4.6},
    "car2":   {"prim": "/Root/BP_MCar2",  "semantic_class": "car",   "length_m": 4.6},
    "car3":   {"prim": "/Root/BP_MCar3",  "semantic_class": "car",   "length_m": 4.6},
    "car4":   {"prim": "/Root/BP_MCar4",  "semantic_class": "car",   "length_m": 4.6},
    "car5":   {"prim": "/Root/BP_MCar5",  "semantic_class": "car",   "length_m": 4.6},
    "car6":   {"prim": "/Root/BP_MCar6",  "semantic_class": "car",   "length_m": 4.6},
    "truck":  {"prim": "/Root/PA_ConstructionTruck01FullyRigged_PhysicsAsset",
               "semantic_class": "truck", "length_m": 7.5},
    "truck2": {"prim": "/Root/PA_ConstructionTruck01FullyRigged_PhysicsAsset2",
               "semantic_class": "truck", "length_m": 7.5},
}

# Prim-name patterns worth looking for when scanning an unfamiliar donor stage.
VEHICLE_NAME_PATTERN = r"car|truck|vehicle|sedan|suv|van|bus|pickup|taxi|lorry|trailer"


# ---------------------------------------------------------------------------
# Semantic labelling
# ---------------------------------------------------------------------------

def label_prim(prim, class_name: str, instance_name: str = "class") -> bool:
    """Attach a semantic label to *prim* so SDG annotators pick it up.

    Isaac Sim 5.x replaced the old ``Semantics.SemanticsAPI`` with
    ``UsdSemantics.LabelsAPI``; ``isaacsim.core.utils.semantics`` exposes
    ``add_labels`` for the new schema and still ships ``add_update_semantics``
    for the old one. Try the new API first, fall back to the old.

    A label on an ancestor Xform covers its whole subtree, which is exactly
    what we want: one bounding box per vehicle rather than one per door.
    """
    try:
        from isaacsim.core.utils.semantics import add_labels
        add_labels(prim, labels=[class_name], instance_name=instance_name)
        return True
    except ImportError:
        pass
    except Exception as exc:                                    # noqa: BLE001
        print(f"[scene_props] add_labels failed on {prim.GetPath()}: {exc}", flush=True)

    try:
        from isaacsim.core.utils.semantics import add_update_semantics
        add_update_semantics(prim, semantic_label=class_name, type_label=instance_name)
        return True
    except Exception as exc:                                    # noqa: BLE001
        print(f"[scene_props] could not label {prim.GetPath()} as "
              f"'{class_name}': {exc}", flush=True)
        return False


def label_prims_matching(stage, pattern: str, class_name: str,
                         root_path: str = "/World/stage") -> list:
    """Label every prim under *root_path* whose name matches *pattern*.

    Use this to get ground truth for props the scene already contains — e.g.
    ``label_prims_matching(stage, r"^BP_Building", "building")``. Matching is
    done on the prim *name* (case-insensitive), and only the outermost match in
    any branch is labelled so nested meshes don't each produce their own box.
    """
    rx = re.compile(pattern, re.I)
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        print(f"[scene_props] label_prims_matching: {root_path} not found", flush=True)
        return []

    labelled = []
    it = iter(Usd.PrimRange(root))
    for prim in it:
        if rx.search(prim.GetName()):
            if label_prim(prim, class_name):
                labelled.append(str(prim.GetPath()))
            it.PruneChildren()          # one box for the whole vehicle/building
    print(f"[scene_props] labelled {len(labelled)} prim(s) matching "
          f"/{pattern}/ as '{class_name}'", flush=True)
    return labelled


# ---------------------------------------------------------------------------
# Referencing helpers
# ---------------------------------------------------------------------------

def _make_static(prim):
    """Disable any rigid bodies inside *prim* so a borrowed prop stays put.

    Some donor assets (the construction truck is one — its prim name ends in
    ``_PhysicsAsset``) carry RigidBodyAPI. Left enabled, PhysX would drop them
    the moment the timeline plays. Colliders are left alone; only the dynamic
    part is switched off.
    """
    disabled = 0
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
            disabled += 1
    return disabled


def _world_bbox(stage, prim):
    """Axis-aligned world-space bbox of *prim* (default + render purposes)."""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    return cache.ComputeWorldBound(prim).ComputeAlignedRange()


def _pump(frames: int = 5):
    app = omni.kit.app.get_app()
    for _ in range(frames):
        app.update()


def list_source_prims(source_url: str, pattern: str = VEHICLE_NAME_PATTERN,
                      max_depth: int = 2, limit: int = 200) -> list:
    """List prims in *source_url* whose name matches *pattern* — i.e. what you
    could borrow from that stage.

    Opens the donor stage with payloads unloaded, so this is cheap enough to
    run against an unfamiliar scene just to see what is in it. Returns a list
    of ``(prim_path, type_name)``.
    """
    rx = re.compile(pattern, re.I)
    stage = Usd.Stage.Open(source_url, load=Usd.Stage.LoadNone)
    if stage is None:
        print(f"[scene_props] could not open {source_url}", flush=True)
        return []

    root = stage.GetDefaultPrim() or stage.GetPseudoRoot()
    base_depth = root.GetPath().pathElementCount
    found = []
    for prim in Usd.PrimRange(root):
        depth = prim.GetPath().pathElementCount - base_depth
        if depth > max_depth:
            continue
        if rx.search(prim.GetName()):
            found.append((str(prim.GetPath()), str(prim.GetTypeName())))
            if len(found) >= limit:
                break
    return found


# ---------------------------------------------------------------------------
# Vehicle placement
# ---------------------------------------------------------------------------

def add_vehicle(stage, name: str, position_m, yaw_deg: float = 0.0,
                kind: str = "car",
                source_url: str = DEFAULT_VEHICLE_LIBRARY,
                source_prim: str = None,
                semantic_class: str = None,
                length_m: float = None,
                scale: float = None,
                parent_path: str = "/World/vehicles",
                scene_scale_factor: float = 1.0,
                add_collision: bool = True) -> str:
    """Reference one vehicle into the stage and place it. Returns its prim path.

    Args:
        stage:              Active USD stage.
        name:               Prim name for this instance (e.g. ``"car_0"``).
        position_m:         ``(x, y, z)`` in world **metres**; z is where the
                            vehicle's *underside* is placed, not its origin.
        yaw_deg:            Heading about +Z, degrees.
        kind:               Key into :data:`VEHICLE_CATALOG`. Ignored if
                            *source_prim* is given explicitly.
        source_url:         Donor stage to reference from.
        source_prim:        Prim path inside the donor stage. Defaults to the
                            catalog entry for *kind*.
        semantic_class:     Label for ground truth. Defaults to the catalog's.
        length_m:           Real-world length to scale the asset to. Defaults
                            to the catalog's. Ignored if *scale* is given.
        scale:              Explicit uniform scale, bypassing the auto-fit.
        parent_path:        Where the vehicles live. Deliberately a sibling of
                            ``/World/stage`` so it does not inherit the
                            ``STAGE_SCALE`` the environment prim carries.
        scene_scale_factor: ``1 / metersPerUnit`` of the live stage — pass the
                            value from ``get_stage_meters_per_unit``.
        add_collision:      Apply CollisionAPI so drones can crash into them.
    """
    entry = VEHICLE_CATALOG.get(kind, {})
    source_prim = source_prim or entry.get("prim")
    if not source_prim:
        raise ValueError(f"unknown vehicle kind {kind!r} and no source_prim given "
                         f"(known kinds: {sorted(VEHICLE_CATALOG)})")
    semantic_class = semantic_class or entry.get("semantic_class", kind)
    length_m = length_m or entry.get("length_m", 4.5)

    created = add_vehicles(
        stage,
        [{"name": name, "kind": kind, "x_m": position_m[0], "y_m": position_m[1],
          "z_m": position_m[2] if len(position_m) > 2 else 0.0, "yaw_deg": yaw_deg,
          "source_prim": source_prim, "semantic_class": semantic_class,
          "length_m": length_m, "scale": scale}],
        source_url=source_url, parent_path=parent_path,
        scene_scale_factor=scene_scale_factor, add_collision=add_collision,
    )
    return created[0] if created else None


def add_vehicles(stage, placements: list,
                 source_url: str = DEFAULT_VEHICLE_LIBRARY,
                 parent_path: str = "/World/vehicles",
                 scene_scale_factor: float = 1.0,
                 add_collision: bool = True,
                 label: bool = True) -> list:
    """Reference, place, scale and label a batch of vehicles.

    Each entry of *placements* is a dict:

        {"kind": "car",        # key into VEHICLE_CATALOG
         "x_m": 12.0,          # world metres
         "y_m": -3.5,
         "z_m": 0.0,           # ground height the underside rests on
         "yaw_deg": 90.0,
         # optional overrides:
         "name": "car_0", "source_prim": "/Root/BP_MCar3",
         "semantic_class": "car", "length_m": 4.6, "scale": 0.01}

    References for the whole batch are added first, then the app loop is pumped
    once, then every vehicle is measured and transformed — composing all the
    referenced geometry in one go is much faster than pumping per vehicle.

    Returns the list of created prim paths.
    """
    if not placements:
        return []

    UsdGeom.Xform.Define(stage, Sdf.Path(parent_path))

    # ---- pass 1: add the references -------------------------------------
    pending = []
    for i, spec in enumerate(placements):
        kind = spec.get("kind", "car")
        entry = VEHICLE_CATALOG.get(kind, {})
        source_prim = spec.get("source_prim") or entry.get("prim")
        if not source_prim:
            print(f"[scene_props] skipping placement {i}: unknown kind {kind!r} "
                  f"(known: {sorted(VEHICLE_CATALOG)})", flush=True)
            continue

        name = spec.get("name") or f"{kind}_{i}"
        wrapper_path = f"{parent_path}/{name}"
        wrapper = UsdGeom.Xform.Define(stage, Sdf.Path(wrapper_path)).GetPrim()

        asset = stage.DefinePrim(f"{wrapper_path}/asset", "Xform")
        asset.GetReferences().AddReference(source_url, source_prim)

        pending.append({
            "spec": spec, "kind": kind, "entry": entry, "name": name,
            "wrapper": wrapper, "asset": asset, "source_prim": source_prim,
        })

    if not pending:
        return []

    _pump(10)   # let the references compose so extents are readable

    # ---- pass 2: neutralise the donor transform, then measure + place ----
    created = []
    s = float(scene_scale_factor)
    for item in pending:
        # The donor's own xform ops (BP_MCar01 sits at (-64.9, -60.1) in
        # ModernCity) ride in on the reference — drop them so the wrapper's
        # transform is the only thing positioning this vehicle.
        UsdGeom.Xformable(item["asset"]).ClearXformOpOrder()
        _make_static(item["asset"])

    _pump(3)

    for item in pending:
        spec, wrapper, asset = item["spec"], item["wrapper"], item["asset"]
        entry, name = item["entry"], item["name"]

        # Measured with the wrapper still at identity, so this is the asset's
        # own extent in stage units — which is how we discover its authoring
        # units without hard-coding cm-vs-m per donor stage.
        bbox = _world_bbox(stage, asset)
        if bbox.IsEmpty():
            print(f"[scene_props] WARN {name}: empty bounding box — the "
                  f"reference to {item['source_prim']} in {source_url} may not "
                  f"have resolved; leaving it unscaled", flush=True)
            size = Gf.Vec3d(0, 0, 0)
        else:
            size = bbox.GetSize()

        k = spec.get("scale")
        if k is None:
            target_len = float(spec.get("length_m") or entry.get("length_m") or 4.5)
            measured = max(size[0], size[1])
            k = (target_len * s / measured) if measured > 1e-9 else 1.0
        k = float(k)

        x_m = float(spec.get("x_m", 0.0))
        y_m = float(spec.get("y_m", 0.0))
        z_m = float(spec.get("z_m", 0.0))
        yaw = float(spec.get("yaw_deg", 0.0))

        # Rest the underside on z_m rather than dropping the origin there:
        # the donor origins are not consistently at wheel height.
        bottom = (bbox.GetMin()[2] * k) if not bbox.IsEmpty() else 0.0

        xform = UsdGeom.Xformable(wrapper)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(x_m * s, y_m * s, z_m * s - bottom))
        xform.AddRotateZOp().Set(yaw)
        xform.AddScaleOp().Set(Gf.Vec3d(k, k, k))

        if add_collision:
            add_colliders(asset)

        if label:
            semantic_class = (spec.get("semantic_class")
                              or entry.get("semantic_class")
                              or item["kind"])
            label_prim(wrapper, semantic_class)

        path = str(wrapper.GetPath())
        created.append(path)
        print(f"[scene_props] {path} ← {item['source_prim']}  "
              f"pos=({x_m:.1f}, {y_m:.1f}, {z_m:.1f})m yaw={yaw:.0f}° "
              f"scale={k:.4g} (asset extent {size[0]:.3g}×{size[1]:.3g}×{size[2]:.3g} "
              f"stage units)", flush=True)

    _pump(3)
    print(f"[scene_props] added {len(created)} vehicle(s) under {parent_path}", flush=True)
    return created


# ---------------------------------------------------------------------------
# Ground snapping
# ---------------------------------------------------------------------------

def snap_prims_to_ground(stage, prim_paths: list, ray_start_m: float = 200.0,
                         max_drop_m: float = 400.0, scene_scale_factor: float = 1.0,
                         ignore_prefix: str = None) -> int:
    """Drop each prim straight down onto the first collider beneath it.

    Placements are authored in XY (where on the map you want a car); the road
    surface height at that XY is generally unknown, so this raycasts down from
    *ray_start_m* above the prim and shifts it so its bounding box bottom rests
    on the hit point.

    Requires PhysX to have loaded the scene's colliders — call it a few frames
    **after** ``timeline.play()``, not during stage setup. Returns the number of
    prims actually moved; prims with no hit below them are left where they are.
    """
    try:
        from omni.physx import get_physx_scene_query_interface
        query = get_physx_scene_query_interface()
    except Exception as exc:                                    # noqa: BLE001
        print(f"[scene_props] ground snap unavailable ({exc}) — "
              f"leaving props at their authored z", flush=True)
        return 0

    s = float(scene_scale_factor)
    moved = 0
    for path in prim_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue

        bbox = _world_bbox(stage, prim)
        if bbox.IsEmpty():
            continue
        centre = bbox.GetMidpoint()
        bottom_z = bbox.GetMin()[2]

        origin = (float(centre[0]), float(centre[1]), float(bottom_z + ray_start_m * s))
        hit = query.raycast_closest(origin, (0.0, 0.0, -1.0), float(max_drop_m * s))
        if not hit or not hit.get("hit"):
            print(f"[scene_props] ground snap: no surface under {path}", flush=True)
            continue

        # Ignore hits on the prop itself (its own colliders are in the scene).
        hit_path = hit.get("collision", "") or hit.get("rigidBody", "")
        if hit_path.startswith(path) or (ignore_prefix and hit_path.startswith(ignore_prefix)):
            print(f"[scene_props] ground snap: {path} hit itself ({hit_path}) — skipped",
                  flush=True)
            continue

        hit_z = float(hit["position"][2])
        delta = hit_z - bottom_z
        if abs(delta) < 1e-6:
            continue

        xform = UsdGeom.Xformable(prim)
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                t = op.Get()
                op.Set(Gf.Vec3d(t[0], t[1], t[2] + delta))
                moved += 1
                print(f"[scene_props] ground snap: {path} z {t[2]:.2f} → "
                      f"{t[2] + delta:.2f} (surface at {hit_z:.2f}, {hit_path})",
                      flush=True)
                break

    return moved


def yaw_deg_from_quat_xyzw(q) -> float:
    """Heading in degrees from an ``[x, y, z, w]`` quaternion (the convention
    ``spawn_utils.generate_spawn_configs`` emits), so drone-style spawn configs
    can be reused to lay out props."""
    return math.degrees(2.0 * math.atan2(float(q[2]), float(q[3])))
