"""One isolated A/B/C physics diagnostic using the unchanged R4 ClockProbe."""
import json
from pathlib import Path
from threading import Lock
import time
import traceback

from clock_probe_r4 import create_graph, observed, subscribe
from recorder import append_record, checkpoint, finite_number, phase, write_json

POLL_COUNT = 15
POLL_SECONDS = 0.02


class PostStepLog:
    def __init__(self, output, physx):
        self.output = output
        self.lock = Lock()
        self.count = 0
        self.invalid_count = 0
        self.subscription = physx.subscribe_physics_on_step_events(
            self.record, pre_step=False, order=0)
        if self.subscription is None:
            raise RuntimeError("PhysX did not return a post-step subscription")

    def record(self, dt):
        with self.lock:
            self.count += 1
            valid = finite_number(dt) and dt > 0
            if not valid:
                self.invalid_count += 1
            try:
                append_record(self.output / "physics-post-steps.jsonl",
                              {"sequence": self.count, "dt_s": dt if valid else repr(dt),
                               "valid": valid, "monotonic_ns": time.monotonic_ns()})
            except Exception:
                self.invalid_count += 1
                raise

    def counters(self):
        with self.lock:
            return self.count, self.invalid_count


def count_delta(before, after, key):
    a, b = before[key], after[key]
    return b - a if type(a) is int and type(b) is int else None


def classify_controls(label, before, after, final, clock_outcome=None):
    events = count_delta(before, after, "native_count")
    legacy = count_delta(before, after, "world_count")
    manager = count_delta(before, after, "manager_count")
    if (count_delta(before, final, "invalid_count") != 0 or
            count_delta(after, final, "native_count") != 0 or
            count_delta(after, final, "world_count") != 0 or
            count_delta(after, final, "manager_count") not in (None, 0)):
        return "INCONCLUSIVE"
    if label == "A":
        return "PASS" if (events, legacy) == (0, 0) and manager in (None, 0) else "INCONCLUSIVE"
    if label == "B":
        if events != 1 or after["playing"] is not True or before["playing"] is not True:
            return "WITNESS_NOT_QUALIFIED"
        return "PASS" if legacy == 1 and manager in (None, 1) else "INCONCLUSIVE"
    if events == 0 and legacy == 0 and manager in (None, 0):
        return "NO_PHYSICS_ADVANCEMENT"
    if events is not None and events > 0 and legacy is not None and 0 <= legacy < events:
        return "COUNTER_DISAGREEMENT"
    old, new = before["native_monotonic_s"], after["native_monotonic_s"]
    graph = after["clock"]["graph"] or {}
    if (events is not None and events > 0 and events == legacy and manager in (None, events)
            and finite_number(old) and finite_number(new) and new > old
            and final["native_monotonic_s"] == new
            and graph.get("ports", {}).get("Time.outputs:simulationTime") == new
            and clock_outcome
            and clock_outcome["status"] == "PASS"
            and type(clock_outcome["accepted_epoch_ns"]) is int):
        return "PASS"
    return "INCONCLUSIVE"


def run_controls(output, probe, witness, world, manager, native_manager, timeline, render):
    def snapshot():
        errors = {}
        clock = probe.snapshot()
        native_count, invalid_count = witness.counters()
        return {"native_count": native_count, "invalid_count": invalid_count,
                "world_count": clock.get("physics_step_index"),
                "manager_count": observed(manager.get_num_physics_steps, errors, "manager_count"),
                "native_time_s": observed(manager.get_simulation_time, errors, "native_time_s"),
                "native_monotonic_s": observed(lambda: native_manager.get_simulation_time_monotonic(),
                                               errors, "native_monotonic_s"),
                "playing": observed(timeline.is_playing, errors, "playing"),
                "auto_update": observed(timeline.is_auto_updating, errors, "auto_update"),
                "clock": clock, "errors": errors, "monotonic_ns": time.monotonic_ns()}

    def record(label, boundary):
        state = snapshot()
        append_record(output / "physics-controls.jsonl",
                      {"control": label, "boundary": boundary, **state})
        return state

    def poll():
        for _ in range(POLL_COUNT):
            probe.spin(POLL_SECONDS)

    for label in ("A", "B", "C"):
        clock_outcome = None
        with phase(output, f"control_{label}"):
            before = record(label, "before")
            if label == "A":
                poll()
                after = record(label, "after_poll")
            elif label == "B":
                try:
                    world.step(render=False)
                except Exception:
                    checkpoint(output, "manual_step", "failed", traceback=traceback.format_exc())
                    write_json(output / "physics-result.json",
                               {"status": "WITNESS_NOT_QUALIFIED", "last_control": label})
                    return
                after = record(label, "after_step")
                poll()
            else:
                after = None

                def capture():
                    nonlocal after
                    render()
                    after = record(label, "after_render")

                try:
                    probe.step(capture)
                except RuntimeError as error:
                    diagnostics = (output / "clock-diagnostic.jsonl").read_text().splitlines()
                    outcome = json.loads(diagnostics[-1])
                    if outcome["event"] != "step_outcome" or str(error) != (
                            f"{outcome['status']} in {outcome['phase']} on step {outcome['step']}"):
                        raise
                    clock_outcome = outcome
                else:
                    clock_outcome = json.loads(
                        (output / "clock-diagnostic.jsonl").read_text().splitlines()[-1])
            final = record(label, "final")
            status = classify_controls(label, before, after, final, clock_outcome)
            append_record(output / "physics-gates.jsonl",
                          {"control": label, "status": status,
                           "clock_outcome": clock_outcome["status"] if clock_outcome else None,
                           "accepted_epoch_ns": clock_outcome["accepted_epoch_ns"] if clock_outcome else None})
        if status != "PASS" or label == "C":
            write_json(output / "physics-result.json", {"status": status, "last_control": label})
            return


def main(output, stage_file):
    with phase(output, "isaac_startup"):
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    try:
        with phase(output, "clock_setup"):
            import omni.physx
            import omni.replicator.core as rep
            import omni.timeline
            import omni.usd
            from isaacsim.core.api import World
            from isaacsim.core.simulation_manager import SimulationManager
            from isaacsim.core.simulation_manager.impl import acquire_simulation_manager_interface
            from isaacsim.core.utils.extensions import enable_extension

            if not omni.usd.get_context().open_stage(str(stage_file)):
                raise RuntimeError("cannot open the isolated pinned stage")
            app.update()
            world = World(physics_dt=.1, rendering_dt=.1, stage_units_in_meters=1.0)
            witness = PostStepLog(output, omni.physx.get_physx_interface())
            world.reset()
            product = rep.create.render_product("/World/ReceiptCamera", resolution=(33, 25))
            rep.orchestrator.set_capture_on_play(False)
            enable_extension("isaacsim.ros2.bridge")
            app.update()
            import omni.graph.core as og
            import rclpy
            from rosgraph_msgs.msg import Clock

            rclpy.init()
            node = rclpy.create_node("ore_r4_isolated_receipt")
        try:
            with phase(output, "physics_witness"):
                create_graph(og)
                timeline = omni.timeline.get_timeline_interface()
                probe = subscribe(output, og, timeline, rclpy, node, Clock,
                                  physics_index=lambda: world.current_time_step_index)
                run_controls(output, probe, witness, world, SimulationManager,
                             acquire_simulation_manager_interface(), timeline,
                             lambda: rep.orchestrator.step(rt_subframes=1, pause_timeline=True,
                                                           delta_time=.1, wait_for_render=True))
                checkpoint(output, "diagnostic_receipt", "persisted", render_product=str(product.path))
        finally:
            node.destroy_node()
            rclpy.shutdown()
    finally:
        with phase(output, "shutdown"):
            app.close()


if __name__ == "__main__":
    main(Path("/capture-output"), Path("/capture-input/stage.usda"))
