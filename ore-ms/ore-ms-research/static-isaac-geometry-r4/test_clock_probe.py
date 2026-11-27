"""CPU-only checks for clock-path boundaries and durable diagnostics."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as Obj
import unittest

from clock_probe_r4 import CLOCK_TYPE, ClockProbe, discovery, graph_snapshot


class ClockProbeTests(unittest.TestCase):
    def setUp(self):
        tmp = TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.output = Path(tmp.name)
        self.now = 0.0
        self.counts = {"Tick": 0, "Time": 0, "Publish": 0}
        self.exists = True
        self.enabled = True
        self.endpoints = []
        self.pending = []
        self.probe = ClockProbe(self.output, self.graph, lambda: {"publishers": self.endpoints.copy()},
                                self.spin, monotonic=lambda: self.now)

    def graph(self):
        return {"exists": self.exists, "enabled": self.enabled, "counts": dict(self.counts)}

    def spin(self, timeout):
        self.now += timeout
        if self.pending:
            sec, ns = self.pending.pop(0)
            self.probe.callback(Obj(clock=Obj(sec=sec, nanosec=ns)))

    def render(self):
        self.counts = {name: count + 1 for name, count in self.counts.items()}
        return "native render result"

    def journal(self):
        path = self.output / "clock-diagnostic.jsonl"
        self.assertTrue(path.read_bytes().endswith(b"\n"))
        return [json.loads(line) for line in path.read_text().splitlines()]

    def assert_failed(self, expected, render=None):
        with self.assertRaisesRegex(RuntimeError, expected):
            self.probe.step(render or self.render)
        last = self.journal()[-1]
        self.assertEqual(last["status"], expected)
        self.assertEqual(last["phase"], "subscriber_wait")
        return last

    def test_graph_missing_or_disabled(self):
        self.exists = False
        self.assert_failed("NO_GRAPH_EXECUTION")
        self.exists, self.enabled = True, False
        self.assert_failed("NO_GRAPH_EXECUTION")

    def test_graph_present_but_not_evaluated(self):
        last = self.assert_failed("NO_GRAPH_EXECUTION", lambda: None)
        self.assertEqual(last["before"]["graph"]["counts"], last["final"]["graph"]["counts"])

    def test_publisher_absent_after_observed_evaluation(self):
        last = self.assert_failed("NO_PUBLISHER_DISCOVERY")
        self.assertEqual(last["final"]["graph"]["counts"]["Publish"], 1)
        self.assertGreaterEqual(self.now, 3.0)

    def test_publisher_incompatible(self):
        self.endpoints = [{"type": CLOCK_TYPE, "compatibility": "ERROR"}]
        last = self.assert_failed("INCOMPATIBLE_QOS")
        self.assertEqual(last["received_callbacks"], 0)

    def test_publisher_compatible_but_no_callback(self):
        self.endpoints = [{"type": CLOCK_TYPE, "compatibility": "OK"}]
        last = self.assert_failed("NO_SUBSCRIBER_DELIVERY")
        self.assertGreater(last["spin_calls"], 0)

    def test_unavailable_counters_do_not_claim_missing_graph_or_publisher(self):
        self.counts = {"Publish": None}
        self.assert_failed("CLOCK_PATH_UNVERIFIED", lambda: None)

    def test_disappearing_publisher_does_not_imply_no_discovery(self):
        self.endpoints = [{"type": CLOCK_TYPE, "compatibility": "OK"}]

        def render():
            self.endpoints = []
            return self.render()

        self.assert_failed("CLOCK_PATH_UNVERIFIED", render)

    def test_valid_advancing_samples_persist_and_keep_render_result(self):
        self.pending = [(0, 100_000_000)]
        self.assertEqual(self.probe.step(self.render), "native render result")
        self.pending = [(0, 200_000_000)]
        self.probe.step(self.render)
        self.assertEqual(self.probe.samples, [100_000_000, 200_000_000])
        self.assertEqual(self.probe.callbacks, 2)
        raw = (self.output / "clock-raw.jsonl").read_text().splitlines()
        self.assertEqual([json.loads(line)["nanosec"] for line in raw], [100_000_000, 200_000_000])
        self.assertEqual(self.journal()[-1]["status"], "PASS")

    def test_repeated_nonadvancing_and_malformed_timestamps(self):
        self.pending = [(0, 100_000_000)]
        self.probe.step(self.render)
        for stamp in ((0, 100_000_000), (0, 99_000_000), (0, 1_000_000_000)):
            with self.subTest(stamp=stamp):
                self.pending = [stamp]
                last = self.assert_failed("INVALID_CLOCK_DATA")
                self.assertEqual(last["received_callbacks"], 1)
        self.assertEqual(self.probe.samples, [100_000_000])

    def test_ambiguous_step_with_multiple_callbacks_fails_closed(self):
        self.pending = [(0, 100_000_000), (0, 200_000_000)]
        last = self.assert_failed("INVALID_CLOCK_DATA")
        self.assertEqual(last["received_callbacks"], 2)

    def test_interruption_during_render_and_wait_is_persisted(self):
        def stop(*_):
            raise KeyboardInterrupt("fixture interruption")

        with self.assertRaises(KeyboardInterrupt):
            self.probe.step(stop)
        self.assertEqual(self.journal()[-1]["phase"], "render_step")
        self.probe.spin = stop
        with self.assertRaises(KeyboardInterrupt):
            self.probe.step(self.render)
        self.assertEqual(self.journal()[-1]["phase"], "subscriber_wait")
        self.assertIn("KeyboardInterrupt: fixture interruption", self.journal()[-1]["traceback"])


class ApiObservationTests(unittest.TestCase):
    def test_graph_snapshot_samples_counters_ports_and_timeline(self):
        graph = Obj(is_valid=lambda: True, is_disabled=lambda: False,
                    get_node=lambda _path: Obj(get_compute_count=lambda: 3))
        og = Obj(get_graph_by_path=lambda _path: graph,
                 Controller=Obj(attribute=lambda _path: Obj(get=lambda: .25)))
        timeline = Obj(get_current_time=lambda: .25, is_playing=lambda: False)
        view = graph_snapshot(og, timeline)
        self.assertTrue(view["enabled"])
        self.assertEqual(view["counts"], {"Tick": 3, "Time": 3, "Publish": 3})
        self.assertEqual(view["ports"]["Time.outputs:simulationTime"], .25)
        self.assertEqual(view["timeline_s"], .25)

    def test_discovery_uses_actual_qos_and_preserves_diagnostics(self):
        duration = Obj(nanoseconds=0)
        policy = Obj(name="policy")
        actual = Obj(history=policy, reliability=policy, durability=policy, liveliness=policy,
                     depth=10, deadline=duration, lifespan=duration, liveliness_lease_duration=duration)
        pub = Obj(node_name="writer", node_namespace="/", topic_type=CLOCK_TYPE,
                  endpoint_gid=bytes(range(4)), qos_profile=actual)
        sub = Obj(node_name="reader", node_namespace="/", topic_type=CLOCK_TYPE, qos_profile=actual)
        node = Obj(get_subscriptions_info_by_topic=lambda _topic: [sub],
                   get_publishers_info_by_topic=lambda _topic: [pub],
                   get_name=lambda: "reader", get_namespace=lambda: "/")

        def check(offered, requested):
            self.assertIs(offered, actual)
            self.assertIs(requested, actual)
            return Obj(name="OK"), ""

        view = discovery(node, Obj(topic_name="/clock"), None, check)
        self.assertEqual(view["subscriber_qos_source"], "discovered")
        self.assertEqual(view["publishers"][0]["gid"], [0, 1, 2, 3])
        self.assertEqual(view["publishers"][0]["compatibility"], "OK")


if __name__ == "__main__":
    unittest.main()
