"""Bounded observer for the isolated R4 Isaac /clock path."""
import math
import os
from pathlib import Path
import time
import traceback

from recorder import append_record, phase, write_json

GRAPH = "/World/ReceiptClock"
TOPIC = "/clock"
NS = 1_000_000_000
WAIT_S = 3.0
CLOCK_TYPE = "rosgraph_msgs/msg/Clock"


def create_graph(og):
    """Use the same action graph as the failed R4 epoch."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {keys.CREATE_NODES: [("Tick", "omni.graph.action.OnPlaybackTick"),
                            ("Time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                            ("Publish", "isaacsim.ros2.bridge.ROS2PublishClock")],
         keys.CONNECT: [("Tick.outputs:tick", "Publish.inputs:execIn"),
                        ("Time.outputs:simulationTime", "Publish.inputs:timeStamp")]},
    )


def observed(call, faults, name):
    try:
        return call()
    except Exception as error:
        faults[name] = f"{type(error).__name__}: {error}"
        return None


def graph_snapshot(og, timeline):
    faults = {}
    graph = observed(lambda: og.get_graph_by_path(GRAPH), faults, "graph_lookup")
    exists = observed(lambda: graph is not None and graph.is_valid(), faults, "graph_valid")
    if "graph_lookup" in faults:
        exists = None
    enabled = observed(lambda: not graph.is_disabled(), faults, "graph_enabled") if exists else None
    counts = {}
    if exists:
        for name in ("Tick", "Time", "Publish"):
            counts[name] = observed(lambda: int(graph.get_node(f"{GRAPH}/{name}").get_compute_count()),
                                    faults, f"{name}_compute_count")
    values = {}
    if exists:
        for port in ("Time.outputs:simulationTime", "Publish.inputs:timeStamp", "Publish.inputs:context",
                     "Publish.inputs:nodeNamespace", "Publish.inputs:topicName", "Publish.inputs:qosProfile"):
            value = observed(lambda: og.Controller.attribute(f"{GRAPH}/{port}").get(), faults, port)
            if isinstance(value, float) and not math.isfinite(value):
                value = repr(value)
            values[port] = value
    timeline_s = observed(lambda: float(timeline.get_current_time()), faults, "timeline")
    if timeline_s is not None and not math.isfinite(timeline_s):
        faults["timeline"] = f"invalid time {timeline_s}"
        timeline_s = None
    return {"exists": exists, "enabled": enabled, "counts": counts, "ports": values,
            "timeline_s": timeline_s,
            "timeline_playing": observed(lambda: timeline.is_playing(), faults, "timeline_playing"), "errors": faults}


def qos_snapshot(profile):
    return {name: getattr(profile, name).name for name in
            ("history", "reliability", "durability", "liveliness")} | {
        "depth": profile.depth,
        "deadline_ns": profile.deadline.nanoseconds,
        "lifespan_ns": profile.lifespan.nanoseconds,
        "liveliness_lease_ns": profile.liveliness_lease_duration.nanoseconds,
    }


def discovery(node, subscription, requested_qos, qos_check):
    topic = subscription.topic_name
    publishers = []
    errors = {}
    subscriptions = observed(lambda: node.get_subscriptions_info_by_topic(topic), errors, "subscriber_endpoints")
    matches = [info for info in (subscriptions or [])
               if (info.node_name, info.node_namespace, info.topic_type)
               == (node.get_name(), node.get_namespace(), CLOCK_TYPE)]
    actual_qos = matches[0].qos_profile if len(matches) == 1 else None
    subscriber_qos = actual_qos if actual_qos is not None else requested_qos
    for info in node.get_publishers_info_by_topic(topic):
        compatibility, reason = qos_check(info.qos_profile, subscriber_qos)
        publishers.append({"name": info.node_name, "namespace": info.node_namespace, "type": info.topic_type,
                           "gid": list(info.endpoint_gid), "qos": qos_snapshot(info.qos_profile),
                           "compatibility": compatibility.name, "reason": reason})
    return {"topic": topic, "publishers": publishers, "subscriber_qos": qos_snapshot(subscriber_qos),
            "subscriber_qos_source": "discovered" if actual_qos is not None else "requested", "errors": errors}


def classify(before, after, discovery_result, spins):
    if after["exists"] is False or after["enabled"] is False:
        return "NO_GRAPH_EXECUTION"
    start, end = before["counts"], after["counts"]
    if after["enabled"] is True and all(name in start and type(start[name]) is int
            and type(end.get(name)) is int and start[name] == end[name] for name in ("Tick", "Time", "Publish")):
        return "NO_GRAPH_EXECUTION"
    pub_before, pub_after = start.get("Publish"), end.get("Publish")
    if type(pub_before) is not int or type(pub_after) is not int or pub_after <= pub_before:
        return "CLOCK_PATH_UNVERIFIED"
    if discovery_result is None:
        return "CLOCK_PATH_UNVERIFIED"
    endpoints = discovery_result["publishers"]
    if not endpoints:
        return "NO_PUBLISHER_DISCOVERY"
    clock = [e for e in endpoints if e["type"] == CLOCK_TYPE]
    if clock and all(e["compatibility"] == "ERROR" for e in clock):
        return "INCOMPATIBLE_QOS"
    if any(e["compatibility"] == "OK" for e in clock) and spins > 0:
        return "NO_SUBSCRIBER_DELIVERY"
    return "CLOCK_PATH_UNVERIFIED"


class ClockProbe:
    def __init__(self, output, graph, endpoint, spin, *, monotonic=time.monotonic):
        self.output = output
        self.graph = graph
        self.endpoint = endpoint
        self.spin = spin
        self.monotonic = monotonic
        self.samples = []
        self.invalid = []
        self.callbacks = 0
        self.steps = 0

    def record(self, event, **details):
        append_record(self.output / "clock-diagnostic.jsonl", {"event": event, "step": self.steps, **details})

    def callback(self, msg):
        self.callbacks += 1
        sec, nanosec = msg.clock.sec, msg.clock.nanosec
        append_record(self.output / "clock-raw.jsonl", {"sec": sec, "nanosec": nanosec})
        valid = type(sec) is int and type(nanosec) is int and sec >= 0 and 0 <= nanosec < NS
        stamp = sec * NS + nanosec if valid else None
        if stamp is None or (self.samples and stamp <= self.samples[-1]):
            self.invalid.append(stamp)
            status = "INVALID_CLOCK_DATA"
        else:
            self.samples.append(stamp)
            status = "valid"
        self.record("callback", status=status, callback_count=self.callbacks, stamp_ns=stamp)

    def snapshot(self):
        errors = {}
        graph = observed(self.graph, errors, "graph")
        endpoint = observed(self.endpoint, errors, "dds")
        return {"graph": graph, "dds": endpoint, "callback_count": self.callbacks, "errors": errors}

    def step(self, render):
        self.steps += 1
        before = self.snapshot()
        count = self.callbacks
        stage = "render_step"
        spins = 0
        self.record("step_started", before=before)
        try:
            rendered = render()
            after_step = self.snapshot()
            self.record("render_returned", after=after_step)
            stage = "subscriber_wait"
            deadline = self.monotonic() + WAIT_S
            remaining = deadline - self.monotonic()
            while self.callbacks == count and remaining > 0:
                self.spin(min(.02, remaining))
                spins += 1
                remaining = deadline - self.monotonic()
            if self.callbacks > count:
                self.spin(0.0)
                spins += 1
        except BaseException:
            self.record("step_interrupted", phase=stage, spin_calls=spins, traceback=traceback.format_exc())
            raise

        final = self.snapshot()
        status = "PASS"
        if self.invalid or self.callbacks > count + 1:
            status = "INVALID_CLOCK_DATA"
        elif self.callbacks == count:
            status = (classify(before["graph"], final["graph"], final["dds"], spins)
                      if before["graph"] is not None and final["graph"] is not None else "CLOCK_PATH_UNVERIFIED")
            if status == "NO_PUBLISHER_DISCOVERY" and any(
                    sample["dds"] and sample["dds"]["publishers"] for sample in (before, after_step)):
                status = "CLOCK_PATH_UNVERIFIED"
        self.record("step_outcome", status=status, phase=stage, spin_calls=spins, deadline_s=WAIT_S,
                    received_callbacks=self.callbacks - count, before=before, final=final)
        if status != "PASS":
            raise RuntimeError(f"{status} in {stage} on step {self.steps}")
        return rendered


def subscribe(output, og, timeline, rclpy, node, clock_type):
    from rclpy.qos import QoSProfile, qos_check_compatible

    qos = QoSProfile(depth=10)  # Same QoS request as the former integer argument.
    probe = ClockProbe(output, lambda: graph_snapshot(og, timeline),
                       lambda: discovery(node, subscription, qos, qos_check_compatible),
                       lambda timeout: rclpy.spin_once(node, timeout_sec=timeout))
    subscription = node.create_subscription(clock_type, TOPIC, probe.callback, qos)
    errors = {}
    domain = observed(lambda: node.context.get_domain_id(), errors, "rclpy_domain")
    write_json(output / "clock-configuration.json",
               {"graph": GRAPH, "namespace": node.get_namespace(), "subscription_topic": subscription.topic_name,
                "node": node.get_name(), "rclpy_domain": domain, "executor": "rclpy.spin_once",
                "ros_environment": {key: os.environ.get(key) for key in
                                    ("ROS_DOMAIN_ID", "ROS_DISTRO", "RMW_IMPLEMENTATION",
                                     "ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_LOCALHOST_ONLY", "ROS_NAMESPACE")},
                "initial": probe.snapshot(), "errors": errors})
    return probe


def clock_only(output, stage_file):
    """One future isolated check. Advance two render steps without frame acquisition."""
    with phase(output, "isaac_startup"):
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    try:
        with phase(output, "clock_setup"):
            import omni.usd
            import omni.timeline
            import omni.replicator.core as rep
            from isaacsim.core.api import World
            from isaacsim.core.utils.extensions import enable_extension

            context = omni.usd.get_context()
            if not context.open_stage(str(stage_file)):
                raise RuntimeError("cannot open the isolated pinned stage")
            app.update()
            world = World(physics_dt=.1, rendering_dt=.1, stage_units_in_meters=1.0)
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
            with phase(output, "clock_transport"):
                create_graph(og)
                probe = subscribe(output, og, omni.timeline.get_timeline_interface(), rclpy, node, Clock)
                native_times = []
                with phase(output, "clock_steps"):
                    for _ in range(2):
                        probe.step(lambda: rep.orchestrator.step(rt_subframes=1, pause_timeline=True,
                                                                  delta_time=.1, wait_for_render=True))
                        snapshot = probe.snapshot()
                        native_times.append(snapshot["graph"]["ports"]["Time.outputs:simulationTime"])
                if (not all(type(n) in (int, float) and math.isfinite(n) for n in native_times)
                        or native_times[1] <= native_times[0]):
                    raise RuntimeError("Isaac simulation-time graph output did not demonstrably advance")
                endpoint = probe.snapshot()["dds"]
                if endpoint is None or not any(row["type"] == CLOCK_TYPE and row["compatibility"] == "OK"
                                               for row in endpoint["publishers"]):
                    raise RuntimeError("compatible /clock publisher discovery was not witnessed")
                write_json(output / "clock-check.json", {"status": "PASS", "native_simulation_times_s": native_times,
                           "ros_clock_ns": probe.samples, "publisher_discovery": endpoint,
                           "render_product": str(product.path)})
        finally:
            node.destroy_node()
            rclpy.shutdown()
    finally:
        with phase(output, "shutdown"):
            app.close()


if __name__ == "__main__":
    clock_only(Path("/capture-output"), Path("/capture-input/stage.usda"))
