from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

from explicit_clock_r4 import capture_epochs


class FakeRuntime:
    """Separate physics, graph execution and delivery like the isolated worker."""
    def __init__(self, problem):
        self.problem = problem
        self.state = {"native_count": 3, "invalid_count": 0,
                      "world_count": 3, "manager_count": 3, "playing": True,
                      "native_time_s": .3, "native_monotonic_s": .3}
        self.wire = SimpleNamespace(received=[], invalid=[], spin=self.spin)
        self.pending = []
        self.graph_time = 0.0
        self.work = []
        self.tick = 0
        self.now = 0.0

    def snapshot(self):
        counts = dict.fromkeys(("Tick", "Time", "Publish"), self.tick)
        ports = dict.fromkeys(("Time.outputs:simulationTime", "Publish.inputs:timeStamp"), self.graph_time)
        publisher = {"type": "rosgraph_msgs/msg/Clock", "gid": [1], "compatibility": "OK"}
        endpoints = [] if self.problem == "no_publisher" else [publisher]
        return deepcopy(self.state) | {
            "clock": {"graph": {"exists": True, "enabled": True, "counts": counts, "ports": ports},
                      "dds": {"publishers": endpoints}}}

    def resume(self):
        self.work.append("resume")
        self.state["playing"] = True
        if self.problem == "resume_step":
            self.state["native_count"] += 1

    def step(self):
        self.work.append("step")
        for key in ("native_count", "world_count", "manager_count"):
            if key != "manager_count" or self.problem != "counter":
                self.state[key] += 1
        if self.problem != "stalled":
            for key in ("native_time_s", "native_monotonic_s"):
                self.state[key] = self.state["native_count"] / 10

    def render(self):
        self.work.append("render")
        prior = int(self.graph_time * 10**9)
        self.state["playing"] = False
        if self.problem != "no_graph":
            self.tick += 1
        if self.problem != "wrong_graph":
            self.graph_time = self.state["native_monotonic_s"]
        if self.problem == "extra_step":
            self.state["native_count"] += 1
        if self.problem == "extra_time":
            self.state["native_monotonic_s"] += .1
        stamp = int(self.graph_time * 10**9)
        if self.problem != "no_callback":
            self.pending = ([prior] if prior else []) + [stamp, stamp]
        if self.problem == "backward":
            self.pending.extend([stamp - 1])
        if self.problem == "unexpected":
            self.pending.extend([stamp + 1])

    def spin(self, timeout):
        self.now += 1.0
        if self.pending:
            self.wire.received.append(self.pending.pop(0))
        if self.problem == "poll_advances":
            self.state["native_count"] += 1


class ExplicitClockTests(unittest.TestCase):
    def run_protocol(self, problem=None):
        runtime = FakeRuntime(problem)
        with TemporaryDirectory() as directory:
            output = Path(directory)
            epochs = capture_epochs(output, runtime.wire, runtime.snapshot, runtime.resume,
                                    runtime.step, runtime.render, monotonic=lambda: runtime.now)
            journal = [json.loads(line) for line in
                       (output / "explicit-gates.jsonl").read_text().splitlines()]
        return epochs, journal, runtime.work

    def test_two_brackets_accept_duplicates_and_prior_epoch(self):
        stamps, journal, work = self.run_protocol()
        self.assertEqual(stamps, [400000000, 500000000])
        self.assertEqual(work, ["step", "render", "resume", "step", "render"])
        callbacks = [row["stamps_ns"] for row in journal if row["gate"] == "raw_callbacks"]
        self.assertEqual(callbacks, [[400000000, 400000000], [400000000, 500000000, 500000000]])

    def test_failed_physics_step_prevents_render(self):
        for problem, verdict in (("counter", "PHYSICS_COUNTER_DISAGREEMENT"),
                                 ("stalled", "NATIVE_TIME_DID_NOT_ADVANCE")):
            with self.subTest(problem=problem):
                stamps, journal, work = self.run_protocol(problem)
                self.assertEqual(stamps, [])
                self.assertEqual(journal[-1]["status"], verdict)
                self.assertEqual(work, ["step"])

    def test_unexpected_render_and_poll_advancement_stops_second_bracket(self):
        for problem, gate in (("extra_step", "render_is_inert"), ("extra_time", "render_is_inert"),
                              ("poll_advances", "poll_is_inert")):
            with self.subTest(problem=problem):
                stamps, journal, work = self.run_protocol(problem)
                self.assertEqual(stamps, [])
                self.assertEqual(journal[-1]["gate"], gate)
                self.assertEqual(work, ["step", "render"])

    def test_unverified_clock_delivery_is_rejected(self):
        cases = {"no_graph": "CLOCK_GRAPH_DID_NOT_EXECUTE", "wrong_graph": "CLOCK_GRAPH_TIME_MISMATCH",
                 "no_callback": "NO_MATCHING_CLOCK_CALLBACK", "unexpected": "UNEXPECTED_CLOCK_TIMESTAMP",
                 "backward": "INVALID_CLOCK_DATA", "no_publisher": "CLOCK_PUBLISHER_UNVERIFIED"}
        for problem, verdict in cases.items():
            with self.subTest(problem=problem):
                stamps, journal, work = self.run_protocol(problem)
                self.assertEqual(stamps, [])
                self.assertEqual(journal[-1]["status"], verdict)
                self.assertEqual(work, ["step", "render"])

    def test_timeline_resume_may_not_advance_physics(self):
        stamps, journal, work = self.run_protocol("resume_step")
        self.assertEqual(stamps, [400000000])
        self.assertEqual(journal[-1]["gate"], "resume_is_inert")
        self.assertEqual(work, ["step", "render", "resume"])
