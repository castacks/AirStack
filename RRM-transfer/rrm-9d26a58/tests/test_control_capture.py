"""CPU-only evidence tests; no ROS or simulator imports."""
import argparse
import importlib.util
import io
import json
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    "control_capture", Path(__file__).parents[1] / "scripts/airstack_control_capture.py")
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


class ControlCaptureTests(unittest.TestCase):
    def test_incremental_events_and_missing_coverage(self):
        stream = io.StringIO()
        recorder = capture.Capture(stream, {"odom": "/odom", "state": "/state"}, 10)
        recorder.record("odom", {"header": {"stamp": {"sec": 2, "nanosec": 3}}}, 11, 9)
        self.assertEqual(json.loads(stream.getvalue())["source_stamp_ns"], 2_000_000_003)
        recorder.record("odom", {}, 13, 10)
        summary = recorder.summary(14)
        self.assertEqual(summary["missing_channels"], ["state"])
        self.assertEqual(summary["channels"]["odom"]["max_receipt_gap_s"], 2)
        self.assertEqual(summary["channels"]["odom"]["final_receipt_age_s"], 1)
        self.assertIsNone(json.loads(stream.getvalue().splitlines()[1])["source_stamp_ns"])

    def test_reason_counts_and_sequence_gap_reset(self):
        recorder = capture.Capture(io.StringIO(), {"admission": "/admission"}, 0)
        for sequence in (2, 3, 7, 1):
            recorder.record("admission", {"data": json.dumps({
                "schema": "pid-admission/v1", "sequence": sequence, "reason_mask": 3})}, 1, 1)
        summary = recorder.summary(2)
        self.assertEqual(summary["admission_reason_mask_counts"], {"3": 4})
        self.assertEqual(summary["diagnostic_sequence_discontinuities"], 2)

    def test_invalid_diagnostics_preserve_raw_event(self):
        stream = io.StringIO()
        recorder = capture.Capture(stream, {"admission": "/admission"}, 0)
        for value in ("{bad", "null", "[]", '{}', json.dumps({
                "schema": "pid-admission/v1", "sequence": True, "reason_mask": 3})):
            recorder.record("admission", {"data": value}, 1, 1)
        self.assertEqual(recorder.summary(2)["invalid_diagnostics"], 5)
        self.assertEqual(len(stream.getvalue().splitlines()), 5)

    def test_event_limit_is_enforced(self):
        recorder = capture.Capture(io.StringIO(), {"state": "/state"}, 0, max_events=1)
        self.assertTrue(recorder.record("state", {}, 1, 1))
        self.assertFalse(recorder.record("state", {}, 2, 2))
        self.assertEqual(recorder.summary(3)["events"], 1)
        self.assertTrue(recorder.summary(3)["event_limit_reached"])

    def test_duration_bounds(self):
        for value in ("nan", "inf", "0", "-1", "301"):
            with self.assertRaises(argparse.ArgumentTypeError):
                capture.bounded_duration(value)
        self.assertEqual(capture.bounded_duration("15"), 15)

    def test_bad_stamp_is_unknown(self):
        for stamp in ({}, {"sec": True, "nanosec": 0}, {"sec": 0, "nanosec": None}):
            self.assertIsNone(capture.source_stamp_ns({"header": {"stamp": stamp}}))

    def test_nonfinite_payload_fails_without_corrupting_prior_events(self):
        stream = io.StringIO()
        recorder = capture.Capture(stream, {"odom": "/odom"}, 0)
        recorder.record("odom", {"x": 1.0}, 1, 1)
        with self.assertRaises(ValueError):
            recorder.record("odom", {"x": float("nan")}, 2, 2)
        self.assertEqual(len(stream.getvalue().splitlines()), 1)
        self.assertEqual(recorder.summary(3)["events"], 1)




class GraphSnapshotTests(unittest.TestCase):
    def test_live_graph_preserves_read_only_metadata(self):
        class Node:
            def count_publishers(self, topic): return 1
            def get_name(self): return 'capture'
            def get_namespace(self): return '/'
            def get_publisher_names_and_types_by_node(self, name, namespace):
                return [('/parameter_events', ['ParameterEvent'])]
            def get_service_names_and_types_by_node(self, name, namespace): return []
        result=capture.graph_snapshot(Node(), '/admission')
        self.assertTrue(result['available']);self.assertIsNone(result['error'])
        self.assertEqual(result['publishers'],1)
        self.assertEqual(result['capture_services'],[])
        self.assertEqual(result['capture_publishers'][0][0],'/parameter_events')

    def test_invalid_context_does_not_fabricate_empty_graph_or_escape(self):
        class Node:
            def count_publishers(self, topic): raise RuntimeError('context is invalid')
        result=capture.graph_snapshot(Node(), '/admission')
        self.assertFalse(result['available'])
        self.assertIn('context is invalid',result['error'])
        for key in ['publishers','capture_publishers','capture_services']:
            self.assertIsNone(result[key])

class CaptureChannelTests(unittest.TestCase):
    def test_channels_have_unique_names_and_capture_both_authority_streams(self):
        import ast
        tree = ast.parse(Path(capture.__file__).read_text())
        assignment = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                          and any(isinstance(t, ast.Name) and t.id == 'specs' for t in n.targets))
        keys = [ast.literal_eval(k) for k in assignment.value.keys]
        self.assertEqual(len(keys), len(set(keys)))
        topics = {ast.literal_eval(k): ast.literal_eval(v.elts[0].right if isinstance(v.elts[0], ast.BinOp) else v.elts[0])
                  for k, v in zip(assignment.value.keys, assignment.value.values)}
        self.assertEqual(topics['sim_clock'], '/clock')
        self.assertEqual(topics['authority'], '/interface/has_control')
        self.assertEqual(topics['authority_diagnostic'],
                         '/takeoff_landing_planner/takeoff_landing_task/authority_diagnostic')

if __name__ == "__main__":
    unittest.main()
