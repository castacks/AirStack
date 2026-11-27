"""Two explicit physics steps and zero-delta renders in an isolated clock-only run."""
from pathlib import Path
import time

from clock_probe_r4 import CLOCK_TYPE, NS, WAIT_S, create_graph, subscribe
from physics_witness_r4 import PostStepLog, count_delta
from recorder import append_record, checkpoint, finite_number, phase, write_json

COUNTERS = ("native_count", "world_count", "manager_count")
TIMES = ("native_time_s", "native_monotonic_s")


def physics_gate(before, after, expected_delta):
    manager_available = before["manager_count"] is not None or after["manager_count"] is not None
    counters = COUNTERS if manager_available else COUNTERS[:2]
    if (before["invalid_count"] != 0 or after["invalid_count"] != 0
            or any(count_delta(before, after, key) != expected_delta for key in counters)):
        return "PHYSICS_COUNTER_DISAGREEMENT"
    for key in TIMES:
        old, new = before[key], after[key]
        if not (finite_number(old) and finite_number(new) and 0 <= old <= new < 2**31):
            return "INVALID_NATIVE_TIME"
        if (new > old) != bool(expected_delta):
            return "UNEXPECTED_NATIVE_TIME_CHANGE" if expected_delta == 0 else "NATIVE_TIME_DID_NOT_ADVANCE"
    return "PASS"


def graph_gate(before, after, native_time):
    earlier = before["clock"]["graph"] or {}
    graph = after["clock"]["graph"] or {}
    ports = graph.get("ports", {})
    counts = graph.get("counts", {})
    previous_counts = earlier.get("counts", {})
    if graph.get("enabled") is not True or graph.get("exists") is not True:
        return "CLOCK_GRAPH_UNAVAILABLE"
    if not all(type(counts.get(key)) is int and type(previous_counts.get(key)) is int
               and counts[key] > previous_counts[key] for key in ("Tick", "Time", "Publish")):
        return "CLOCK_GRAPH_DID_NOT_EXECUTE"
    if any(ports.get(key) != native_time for key in ("Time.outputs:simulationTime", "Publish.inputs:timeStamp")):
        return "CLOCK_GRAPH_TIME_MISMATCH"
    return "PASS"


def callback_gate(stamps, invalid, expected, prior):
    if invalid or any(a > b for a, b in zip(stamps, stamps[1:])):
        return "INVALID_CLOCK_DATA"
    if any(stamp != expected and stamp != prior for stamp in stamps):
        return "UNEXPECTED_CLOCK_TIMESTAMP"
    return "PASS" if expected in stamps else "NO_MATCHING_CLOCK_CALLBACK"


def publisher_gate(rendered, final):
    def offered(snapshot):
        return [row for row in (snapshot["clock"]["dds"] or {}).get("publishers", [])
                if row["type"] == CLOCK_TYPE]

    publishers = offered(final)
    if len(publishers) != 1 or publishers[0]["compatibility"] != "OK":
        return "CLOCK_PUBLISHER_UNVERIFIED"
    after_render = offered(rendered)
    if after_render and (len(after_render) != 1 or after_render[0]["compatibility"] != "OK"
                         or after_render[0]["gid"] != publishers[0]["gid"]):
        return "CLOCK_PUBLISHER_AMBIGUOUS"
    return "PASS"


def capture_epochs(output, probe, snapshot, resume, step, render, *, monotonic=time.monotonic):
    accepted = []
    previous_final = None
    for bracket in (1, 2):
        probe.steps = bracket

        def record(boundary):
            value = snapshot()
            append_record(output / "explicit-brackets.jsonl",
                          {"bracket": bracket, "boundary": boundary, **value})
            return value

        def save_gate(gate, status, **details):
            append_record(output / "explicit-gates.jsonl",
                          {"bracket": bracket, "gate": gate, "status": status, **details})
            return status == "PASS"

        with phase(output, f"clock_bracket_{bracket}"):
            initial = record("before_resume")
            if previous_final is not None and not save_gate(
                    "between_brackets", physics_gate(previous_final, initial, 0)):
                return accepted
            if initial["playing"] is False:
                resume()
            before = record("before_step")
            status = physics_gate(initial, before, 0)
            if status == "PASS" and before["playing"] is not True:
                status = "TIMELINE_NOT_PLAYING"
            if not save_gate("resume_is_inert", status):
                return accepted

            step()
            stepped = record("after_step")
            if not save_gate("explicit_step", physics_gate(before, stepped, 1)):
                return accepted
            native_time = stepped["native_monotonic_s"]
            expected = int(native_time * NS)  # Ros2ClockMessageImpl::writeData truncates.
            prior = accepted[-1] if accepted else None
            if prior is not None and expected <= prior:
                save_gate("epoch_order", "CLOCK_EPOCH_DID_NOT_ADVANCE", expected_ns=expected)
                return accepted
            save_gate("epoch_order", "PASS", expected_ns=expected, native_time_s=native_time)

            first = len(probe.received)
            render()
            rendered = record("after_render")
            if not save_gate("render_is_inert", physics_gate(stepped, rendered, 0)):
                return accepted
            if not save_gate("render_graph", graph_gate(stepped, rendered, native_time)):
                return accepted

            deadline = monotonic() + WAIT_S
            status = "NO_MATCHING_CLOCK_CALLBACK"
            polls = 0
            while (remaining := deadline - monotonic()) > 0:
                probe.spin(min(0.02, remaining))
                polls += 1
                status = callback_gate(probe.received[first:], probe.invalid, expected, prior)
                if status not in ("PASS", "NO_MATCHING_CLOCK_CALLBACK"):
                    break
            final = record("after_poll")
            if not save_gate("raw_callbacks", status, poll_count=polls,
                             expected_ns=expected, stamps_ns=probe.received[first:]):
                return accepted
            if not save_gate("poll_is_inert", physics_gate(rendered, final, 0)):
                return accepted
            if not save_gate("final_graph", graph_gate(stepped, final, native_time)):
                return accepted

            if not save_gate("publisher", publisher_gate(rendered, final)):
                return accepted
            accepted.append(expected)
            save_gate("accepted_epoch", "PASS", stamp_ns=expected)
            previous_final = final
    return accepted


def main(output, stage_file):
    with phase(output, "isaac_startup"):
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    try:
        with phase(output, "clock_setup"):
            from isaacsim.core.api import World
            from isaacsim.core.simulation_manager import SimulationManager
            from isaacsim.core.simulation_manager.impl import acquire_simulation_manager_interface
            from isaacsim.core.utils.extensions import enable_extension
            import omni.physx
            import omni.replicator.core as rep
            import omni.timeline
            import omni.usd

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
            with phase(output, "explicit_clock"):
                create_graph(og)
                timeline = omni.timeline.get_timeline_interface()
                probe = subscribe(output, og, timeline, rclpy, node, Clock,
                                  physics_index=lambda: world.current_time_step_index)
                native_manager = acquire_simulation_manager_interface()

                def snapshot():
                    clock = probe.snapshot()
                    events, invalid = witness.counters()
                    return {"clock": clock, "monotonic_ns": time.monotonic_ns(),
                            "native_count": events, "invalid_count": invalid,
                            "world_count": clock.get("physics_step_index"),
                            "manager_count": SimulationManager.get_num_physics_steps(),
                            "native_time_s": SimulationManager.get_simulation_time(),
                            "native_monotonic_s": native_manager.get_simulation_time_monotonic(),
                            "playing": timeline.is_playing(),
                            "auto_update": timeline.is_auto_updating()}

                def resume():
                    timeline.play()
                    timeline.commit()

                accepted = capture_epochs(
                    output, probe, snapshot, resume, lambda: world.step(render=False),
                    lambda: rep.orchestrator.step(rt_subframes=1, pause_timeline=True,
                                                  delta_time=0.0, wait_for_render=True))
                write_json(output / "explicit-result.json",
                           {"status": "PASS" if len(accepted) == 2 else "FAIL",
                            "accepted_epochs_ns": accepted})
                checkpoint(output, "diagnostic_receipt", "persisted", render_product=str(product.path))
        finally:
            node.destroy_node()
            rclpy.shutdown()
    finally:
        with phase(output, "shutdown"):
            app.close()


if __name__ == "__main__":
    main(Path("/capture-output"), Path("/capture-input/stage.usda"))
