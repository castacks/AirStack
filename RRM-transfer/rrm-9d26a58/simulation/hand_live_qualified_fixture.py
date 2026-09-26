"""Qualified tabletop fixture contract shared by action-applying live probes.

Importing this module has no Isaac dependency and starts no simulator. Isaac modules
are imported only by :func:`build_qualified_fixture` after ``SimulationApp`` exists.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from typing import Any, Callable

from simulation.hand_live_binding_probe import compare_profile


EXPECTED_ASSET_URL = (
    "omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1/Isaac/"
    "IsaacLab/Robots/KukaAllegro/kuka.usd"
)
EXPECTED_SCENE_ENTITIES = {
    "red_block": "/World/red_block",
    "blue_block": "/World/blue_block",
    "tray_1": "/World/Tray",
    "table": "/World/Table",
}
EXPECTED_JOINT_NAMES = tuple(f"iiwa7_joint_{index}" for index in range(1, 8)) + (
    "index_joint_0", "middle_joint_0", "ring_joint_0", "thumb_joint_0",
    "index_joint_1", "middle_joint_1", "ring_joint_1", "thumb_joint_1",
    "index_joint_2", "middle_joint_2", "ring_joint_2", "thumb_joint_2",
    "index_joint_3", "middle_joint_3", "ring_joint_3", "thumb_joint_3",
)


def _finite_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and \
        math.isfinite(value)


def validate_qualified_fixture_spec(probe: dict[str, Any], *,
        requested_physics_dt_s: float) -> list[str]:
    """Validate that the qualification artifact can define the live fixture."""
    errors: list[str] = []
    if probe.get("schema_version") != "rrm-hand-controller-probe/v1":
        errors.append("probe_schema_mismatch")
    if probe.get("asset_url") != EXPECTED_ASSET_URL:
        errors.append("asset_url_mismatch")
    asset_sha256 = probe.get("asset_sha256")
    if not isinstance(asset_sha256, str) or len(asset_sha256) != 64 or \
            any(character not in "0123456789abcdef" for character in asset_sha256):
        errors.append("asset_sha256_missing")
    if probe.get("scene_seed") != 0:
        errors.append("scene_seed_mismatch")
    physics_dt_s = probe.get("physics_dt_s")
    if not _finite_number(physics_dt_s) or not _finite_number(requested_physics_dt_s) or \
            not math.isclose(float(physics_dt_s), float(requested_physics_dt_s),
                             rel_tol=0.0, abs_tol=1e-12):
        errors.append("physics_dt_mismatch")
    if probe.get("scene_entities") != EXPECTED_SCENE_ENTITIES:
        errors.append("scene_entities_mismatch")
    if probe.get("articulation_root") != "/World/KukaAllegro/root_joint":
        errors.append("articulation_root_mismatch")

    limits = probe.get("joint_limits")
    if not isinstance(limits, list) or len(limits) != 23:
        errors.append("joint_profile_incomplete")
    else:
        for index, (name, item) in enumerate(zip(EXPECTED_JOINT_NAMES, limits)):
            if not isinstance(item, dict) or item.get("index") != index or \
                    item.get("name") != name or not all(_finite_number(item.get(key))
                    for key in ("lower_rad", "upper_rad", "max_velocity_rad_s",
                                "clamped_default_position_rad")) or \
                    not item["lower_rad"] <= item["clamped_default_position_rad"] <= \
                    item["upper_rad"] or item["max_velocity_rad_s"] <= 0:
                errors.append(f"joint_{index}_fixture_invalid")

    controller = probe.get("controller")
    gains = controller.get("runtime_gain_override") if isinstance(controller, dict) else None
    stiffness = gains.get("configured_stiffness") if isinstance(gains, dict) else None
    damping = gains.get("configured_damping") if isinstance(gains, dict) else None
    if not isinstance(stiffness, list) or len(stiffness) != 23 or \
            not all(_finite_number(value) and value > 0 for value in stiffness):
        errors.append("configured_stiffness_invalid")
    if not isinstance(damping, list) or len(damping) != 23 or \
            not all(_finite_number(value) and value >= 0 for value in damping):
        errors.append("configured_damping_invalid")
    if not isinstance(gains, dict) or gains.get("applied_as_configured") is not True or \
            gains.get("limits_changed") is not False:
        errors.append("qualified_gain_evidence_invalid")

    safe = probe.get("safe_state")
    if not isinstance(safe, dict) or \
            not _finite_number(safe.get("joint_velocity_threshold_rad_s")) or \
            safe["joint_velocity_threshold_rad_s"] <= 0 or \
            not _finite_number(safe.get("object_velocity_threshold_m_s")) or \
            safe["object_velocity_threshold_m_s"] <= 0 or \
            type(safe.get("consecutive_window_required")) is not int or \
            safe["consecutive_window_required"] < 1:
        errors.append("safe_state_contract_invalid")
    return errors


@dataclass
class QualifiedFixture:
    hand: object
    live_profile: list[dict[str, Any]]
    initial_positions: object
    object_speeds: Callable[[], tuple[float, ...]]
    safe_state: dict[str, Any]
    attestation: dict[str, Any]


def build_qualified_fixture(*, app: object, world: object, probe: dict[str, Any],
                            requested_physics_dt_s: float) -> QualifiedFixture:
    """Build and attest the scene recorded by the qualification probe."""
    contract_errors = validate_qualified_fixture_spec(
        probe, requested_physics_dt_s=requested_physics_dt_s)
    if contract_errors:
        raise ValueError(f"qualified fixture contract invalid: {contract_errors}")

    import numpy as np
    import omni.client
    from pxr import UsdLux, UsdPhysics
    from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage

    np.random.seed(probe["scene_seed"])
    world.get_physics_context().enable_ccd(True)
    add_reference_to_stage(usd_path=probe["asset_url"], prim_path="/World/KukaAllegro")
    app.update()
    stage = get_current_stage()
    light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    light.CreateIntensityAttr(1000.0)
    roots = [str(prim.GetPath()) for prim in stage.Traverse()
             if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
             and str(prim.GetPath()).startswith("/World/KukaAllegro")]
    if roots != [probe["articulation_root"]]:
        raise RuntimeError(f"qualified articulation root mismatch: {roots}")
    hand = world.scene.add(SingleArticulation(prim_path=roots[0], name="kuka_allegro"))
    world.scene.add(FixedCuboid(
        prim_path=EXPECTED_SCENE_ENTITIES["table"], name="table",
        position=np.array([0.68, 0.0, 0.45]), scale=np.array([0.90, 0.80, 0.10]),
        color=np.array([0.55, 0.55, 0.55])))
    world.scene.add(FixedCuboid(
        prim_path=EXPECTED_SCENE_ENTITIES["tray_1"], name="tray_1",
        position=np.array([0.84, -0.24, 0.505]), scale=np.array([0.18, 0.18, 0.01]),
        color=np.array([0.9, 0.8, 0.1])))
    blocks = {}
    for entity_id, position, color in (
            ("red_block", [0.68, 0.16, 0.535], [0.9, 0.1, 0.1]),
            ("blue_block", [0.68, -0.02, 0.535], [0.1, 0.2, 0.9])):
        block = world.scene.add(DynamicCuboid(
            prim_path=EXPECTED_SCENE_ENTITIES[entity_id], name=entity_id,
            position=np.array(position), scale=np.array([0.06] * 3),
            color=np.array(color), mass=0.05))
        block.set_default_state(position=np.array(position),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]))
        blocks[entity_id] = block

    world.reset()
    names = tuple(hand.dof_names)
    limits = np.asarray(hand._articulation_view.get_dof_limits(), dtype=float)
    if limits.ndim == 3:
        limits = limits[0]
    properties = hand.dof_properties
    live_profile = [{"index": index, "name": name,
        "lower_rad": float(limits[index, 0]), "upper_rad": float(limits[index, 1]),
        "max_velocity_rad_s": float(properties[index]["maxVelocity"])}
        for index, name in enumerate(names)]

    gain_record = probe["controller"]["runtime_gain_override"]
    configured_kps = np.asarray(gain_record["configured_stiffness"], dtype=float)
    configured_kds = np.asarray(gain_record["configured_damping"], dtype=float)
    controller = hand.get_articulation_controller()
    controller.set_gains(kps=configured_kps, kds=configured_kds)
    applied_kps, applied_kds = controller.get_gains()
    gains_match = bool(np.allclose(applied_kps, configured_kps) and
                       np.allclose(applied_kds, configured_kds))
    defaults = np.asarray([item["clamped_default_position_rad"]
                           for item in probe["joint_limits"]], dtype=float)
    hand.set_joints_default_state(positions=defaults, velocities=np.zeros(23))
    world.reset()
    for _ in range(30):
        world.step(render=False)

    def object_speeds() -> tuple[float, ...]:
        return tuple(float(np.linalg.norm(np.asarray(blocks[entity_id].get_linear_velocity(),
                                                     dtype=float)))
                     for entity_id in ("red_block", "blue_block"))

    measured_object_speeds = object_speeds()
    result, asset_content = omni.client.read_file(probe["asset_url"])
    asset_sha256 = None
    if result == omni.client.Result.OK:
        asset_sha256 = hashlib.sha256(bytes(asset_content)).hexdigest()
    profile_mismatches = compare_profile(probe, live_profile)
    entity_paths_present = all(stage.GetPrimAtPath(path).IsValid()
                               for path in EXPECTED_SCENE_ENTITIES.values())
    readback_errors = []
    if asset_sha256 != probe["asset_sha256"]:
        readback_errors.append("asset_content_hash_mismatch")
    if profile_mismatches:
        readback_errors.append("live_joint_profile_mismatch")
    if not gains_match:
        readback_errors.append("controller_gain_readback_mismatch")
    if not entity_paths_present:
        readback_errors.append("scene_entity_missing")
    if len(measured_object_speeds) != 2 or not all(
            math.isfinite(value) and value >= 0 for value in measured_object_speeds):
        readback_errors.append("object_velocity_unreadable")
    attestation = {
        "qualified_scene_equivalent": not readback_errors,
        "contract_errors": contract_errors,
        "readback_errors": readback_errors,
        "asset_url": probe["asset_url"],
        "expected_asset_sha256": probe["asset_sha256"],
        "observed_asset_sha256": asset_sha256,
        "physics_dt_s": requested_physics_dt_s,
        "scene_seed": probe["scene_seed"],
        "articulation_root": roots[0],
        "scene_entities": dict(EXPECTED_SCENE_ENTITIES),
        "entity_paths_present": entity_paths_present,
        "profile_mismatches": profile_mismatches,
        "controller_gains_match": gains_match,
        "object_velocity_source": "live_dynamic_block_linear_velocity",
        "initial_object_speeds_m_s": list(measured_object_speeds),
    }
    return QualifiedFixture(hand=hand, live_profile=live_profile,
        initial_positions=defaults, object_speeds=object_speeds,
        safe_state=dict(probe["safe_state"]), attestation=attestation)
