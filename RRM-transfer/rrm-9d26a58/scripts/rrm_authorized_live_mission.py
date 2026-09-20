#!/usr/bin/env python3
"""Run an explicitly authorized live RRM mission in AirStack simulation only.

This runner is never started by the Cosmos worker, OSMO workflow, or command console.
It uses the existing public task-action adapter only when both ``--execute`` and
``--simulator-only`` are supplied.  Without them it captures/verifies/proposes one
shadow action and exits without creating a task ActionClient.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

# Keep the explicit runner usable from a normal repository shell. Dependencies are
# prepared in the isolated .rrm-deps directory by scripts/test_rrm.sh.
SOURCE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(SOURCE_ROOT))
sys.path.insert(0, str(SOURCE_ROOT / ".rrm-deps"))

from rrm.airstack_drone import DroneOutcomeVerification, MapWaypoint
from rrm.authorized_live_mission import (AuthorizedLiveMission, VerifiedLiveStep)
from rrm.continuous_replan import MissionAuthorization
from rrm.cosmos_entity_verifier_client import CosmosEntityVerifierClient
from rrm.cosmos_worker_client import CosmosWorkerClient
from rrm.drone_decision import AirStackDroneDecisionBridge, DroneNavigationTarget
from rrm.dynamic_feasibility import DynamicFeasibilityResult
from rrm.live_replan import LiveReplanCycle
from rrm_cosmos_reason2 import load_context


class DockerVerifiedFrameSource:
    """Read-only capture plus private visual-verifier call for one live step."""

    def __init__(self, cycle, context, catalog, verifier, capture_script: Path, container: str):
        self.cycle, self.context, self.catalog = cycle, context, catalog
        self.verifier, self.capture_script, self.container = verifier, capture_script, container
        self.last_stamp = None

    def capture(self) -> VerifiedLiveStep:
        suffix = f"{self.cycle.cycle_id[:8]}-{self.cycle.state['next_step_index']}"
        remote_image = f"/tmp/rrm-live-frame-{suffix}.png"
        remote_script = f"/tmp/rrm-live-capture-{suffix}.py"
        subprocess.run(["docker", "cp", str(self.capture_script), f"{self.container}:{remote_script}"],
                       check=True, capture_output=True, timeout=10)
        command = (
            "source /root/AirStack/robot/ros_ws/install/local_setup.bash; "
            "exec timeout 12 python3 \"$1\" --topic /robot_1/sensors/front_stereo/left/image_rect "
            "--odometry-topic /robot_1/odometry_conversion/odometry --output \"$2\" --timeout-s 8"
        )
        subprocess.run(["docker", "exec", self.container, "bash", "-c", command,
                        "rrm-live-capture", remote_script, remote_image],
                       check=True, capture_output=True, timeout=16)
        image = subprocess.run(["docker", "exec", self.container, "cat", remote_image],
                               check=True, capture_output=True, timeout=5).stdout
        metadata = json.loads(subprocess.run(["docker", "exec", self.container, "cat", remote_image + ".json"],
                                             check=True, capture_output=True, timeout=5).stdout)
        metadata["source_stamp_advanced"] = (self.last_stamp is not None
                                              and metadata.get("source_stamp_ns") != self.last_stamp)
        if self.last_stamp is None:
            # A first frame cannot demonstrate clock progression. Capture one more
            # frame before using any image as mission evidence.
            self.last_stamp = metadata.get("source_stamp_ns")
            return self.capture()
        self.last_stamp = metadata.get("source_stamp_ns")
        if hashlib.sha256(image).hexdigest() != metadata.get("sha256"):
            raise RuntimeError("Read-only camera capture checksum mismatch.")
        scene, context = self.verifier.verify(
            cycle_id=self.cycle.cycle_id, step_index=self.cycle.state["next_step_index"],
            metadata=metadata, image=image, context=self.context, entity_catalog=self.catalog,
        )
        return VerifiedLiveStep(metadata, image, scene, context)


class DockerPublicTaskDispatcher:
    """Explicit public-ActionClient bridge; never direct PX4/MAVROS control."""

    def __init__(self, source_root: Path, *, enabled: bool, container: str):
        self.source_root, self.enabled, self.container = source_root, enabled, container

    def dispatch(self, proposal, evidence_dir: Path) -> DroneOutcomeVerification:
        if not self.enabled:
            raise RuntimeError("public dispatch requires --execute --simulator-only")
        dispatch_nonce = hashlib.sha256(
            f"{proposal.task_id}:{proposal.action_id}:{time.time_ns()}".encode()
        ).hexdigest()[:20]
        remote = f"/tmp/rrm-live-dispatch-{dispatch_nonce}"
        remote_source, remote_proposal, remote_outcome = remote + "/source", remote + "/proposal.json", remote + "/outcome.json"
        subprocess.run(["docker", "exec", self.container, "mkdir", "-p", remote_source],
                       check=True, capture_output=True, timeout=10)
        subprocess.run(["docker", "cp", str(self.source_root) + "/.", f"{self.container}:{remote_source}"],
                       check=True, capture_output=True, timeout=30)
        proposal_path = evidence_dir / "compiled-proposal.json"
        subprocess.run(["docker", "cp", str(proposal_path), f"{self.container}:{remote_proposal}"],
                       check=True, capture_output=True, timeout=10)
        command = (
            "source /root/AirStack/robot/ros_ws/install/local_setup.bash; "
            f"export PYTHONPATH=/tmp/rrm-canonical-deps:{remote_source}:$PYTHONPATH; "
            f"exec python3 {remote_source}/scripts/airstack_drone_dispatch.py "
            f"--proposal-json {remote_proposal} --execute --verify-observation "
            f"--observation-timeout-s 10 --max-observation-age-s 2 --action-timeout-s 120 "
            f"--outcome-json {remote_outcome}"
        )
        completed = subprocess.run(["docker", "exec", "-e", "ROS_DOMAIN_ID=1", self.container,
                                    "bash", "-lc", command], capture_output=True, text=True, timeout=150)
        (evidence_dir / "dispatcher.log").write_text(completed.stdout + completed.stderr, encoding="utf-8")
        copied = subprocess.run(["docker", "exec", self.container, "cat", remote_outcome],
                                capture_output=True, text=True, timeout=5)
        if copied.returncode:
            raise RuntimeError("public dispatcher produced no independent outcome record")
        (evidence_dir / "adapter-outcome.json").write_text(copied.stdout, encoding="utf-8")
        return DroneOutcomeVerification.model_validate_json(copied.stdout)


class CommandFeasibilityEvaluator:
    """Invoke one configured embodiment preflight provider without a shell.

    The provider is adapter-owned. It may query Isaac/AirStack physics, motion
    planning, body limits, controller readiness, resources, and the stop channel.
    Its stdout must be exactly one ``DynamicFeasibilityResult`` JSON document.
    """

    def __init__(self, executable: Path, *, timeout_s: float = 30.0):
        if not executable.is_file():
            raise ValueError("Configured feasibility provider does not exist.")
        self.executable = executable.resolve()
        self.timeout_s = timeout_s

    def evaluate(self, semantic_action, proposal, context, observation, scene_state, *, stop_generation):
        request = {
            "schema_version": "rrm-feasibility-query/v1",
            "semantic_action": semantic_action,
            "proposal": proposal.model_dump(mode="json"),
            "task": context.task.model_dump(mode="json"),
            "snapshot": context.snapshot.model_dump(mode="json"),
            "capabilities": {
                "embodiment_id": context.capabilities.embodiment_id,
                "revision": context.capabilities.revision,
                "operations": sorted(context.capabilities.operations),
                "resources": sorted(context.capabilities.resources),
                "available_resources": sorted(context.capabilities.available_resources),
                "limits_ref": context.capabilities.limits_ref,
            },
            "now_monotonic_s": context.now_monotonic_s,
            "observation": observation,
            "scene_state": scene_state,
            "stop_generation": stop_generation,
            "execution_dispatch": False,
        }
        completed = subprocess.run(
            [str(self.executable)], input=json.dumps(request), text=True,
            capture_output=True, timeout=self.timeout_s, check=True,
        )
        return DynamicFeasibilityResult.model_validate_json(completed.stdout)


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(description=__doc__)
    value.add_argument("--context", required=True, type=Path)
    value.add_argument("--scene-manifest", required=True, type=Path)
    value.add_argument("--entity-catalog", required=True, type=Path)
    value.add_argument("--authorization", required=True, type=Path)
    value.add_argument("--worker-url", required=True)
    value.add_argument("--run-dir", required=True, type=Path)
    value.add_argument("--execute", action="store_true")
    value.add_argument("--simulator-only", action="store_true")
    value.add_argument("--feasibility-provider", type=Path,
                       help="Executable C03 provider; required for simulator execution")
    value.add_argument("--container", default="airstack-robot-desktop-1")
    return value


def _authorization(path: Path) -> MissionAuthorization:
    value = json.loads(path.read_text(encoding="utf-8"))
    return MissionAuthorization(task_id=value["task_id"], task_revision=value["task_revision"],
                                allowed_verbs=frozenset(value["allowed_verbs"]),
                                allowed_targets=frozenset(value["allowed_targets"]),
                                max_actions=value["max_actions"])


def _bridge(manifest: Path):
    markers = json.loads(manifest.read_text(encoding="utf-8"))["markers"]
    targets = []
    for entity_id, marker in markers.items():
        waypoint = marker.get("map_waypoint")
        if waypoint is not None:
            targets.append(DroneNavigationTarget(entity_id=entity_id,
                waypoints=(MapWaypoint(**waypoint),), goal_tolerance_m=0.3))
    return AirStackDroneDecisionBridge(embodiment_id="aerial-eval", robot_name="robot_1",
                                       targets=tuple(targets))


def main() -> int:
    args = parser().parse_args()
    if args.execute != args.simulator_only:
        raise SystemExit("Public action dispatch requires both --execute and --simulator-only.")
    if args.execute and args.feasibility_provider is None:
        raise SystemExit("Simulator execution requires an embodiment --feasibility-provider.")
    context, authorization = load_context(args.context), _authorization(args.authorization)
    if args.run_dir.exists():
        raise SystemExit("--run-dir must be new; preserve prior mission evidence instead of overwriting it.")
    cycle = LiveReplanCycle(args.run_dir, task_id=context.task.task_id,
                            task_revision=context.task.revision, expected_camera_frame="camera_left")
    bridge = _bridge(args.scene_manifest)
    verifier = CosmosEntityVerifierClient(args.worker_url)
    source = DockerVerifiedFrameSource(cycle, context, json.loads(args.entity_catalog.read_text()), verifier,
                                       Path(__file__).with_name("airstack_capture_image.py"), args.container)
    mission = AuthorizedLiveMission(
        cycle, authorization, source, CosmosWorkerClient(args.worker_url),
        lambda plan, live: bridge.compile_plan(plan, live.task, live.snapshot, live.capabilities,
                                               now_monotonic_s=live.now_monotonic_s),
        DockerPublicTaskDispatcher(SOURCE_ROOT, enabled=args.execute, container=args.container),
        (CommandFeasibilityEvaluator(args.feasibility_provider)
         if args.feasibility_provider is not None else None),
    )
    proposal = mission.propose_initial_action()
    print(json.dumps(proposal, indent=2, sort_keys=True))
    if not args.execute:
        return 0
    mission.approve_mission()
    while cycle.phase.value == "OUTCOME_REQUIRED":
        print(json.dumps(mission.dispatch_and_replan(), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
