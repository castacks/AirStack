"""Private visual-verifier transport tests; no model, ROS, or dispatcher."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from rrm.cosmos_entity_verifier_client import CosmosEntityVerifierClient
from rrm.visual_world_builder import (MediaArtifact, VisualGroundingInput,
                                      parse_visual_candidate)
from rrm_cosmos_reason2 import load_context


ROOT = Path(__file__).parents[1]


class EntityVerifierClientTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        fixture = ROOT / "examples" / "office_visual_eval"
        path = Path(self.temp.name) / "context.json"
        shutil.copy(fixture / "navigation_context.json", path)
        self.context = load_context(path)
        self.image = b"verified-live-image"
        self.metadata = {"sha256": hashlib.sha256(self.image).hexdigest(), "source_stamp_ns": 77}

    def server(self, *, dispatch=False, localized=True):
        outer = self
        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                value = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                if self.path != "/v1/verify-entities" or value["observation_sha256"] != outer.metadata["sha256"]:
                    raise AssertionError("client did not bind the verifier request")
                context = VisualGroundingInput(
                    task_id=outer.context.task.task_id, episode_id=outer.context.snapshot.episode_id,
                    state_revision=outer.context.snapshot.revision,
                    entity_catalog=value["entity_catalog"],
                    media=MediaArtifact(source_ref="test", sha256=value["observation_sha256"],
                                        observed_monotonic_s=5.0),
                    received_monotonic_s=5.0, max_age_s=5.0, model_ref="test-verifier",
                )
                claims = [{"subject": "blue_marker", "predicate": "exists", "obj": None, "truth": "TRUE"}]
                if localized:
                    claims.append({"subject": "blue_marker", "predicate": "localized", "obj": None, "truth": "TRUE"})
                candidate = parse_visual_candidate(json.dumps({"status": "READY", "claims": claims}), context)
                result = {"cycle_id": value["cycle_id"], "step_index": value["step_index"],
                          "observation_sha256": value["observation_sha256"],
                          "visual_candidate": candidate.model_dump(mode="json"),
                          "now_monotonic_s": 5.0, "execution_dispatch": dispatch}
                encoded = json.dumps(result).encode()
                self.send_response(200); self.send_header("Content-Length", str(len(encoded))); self.end_headers()
                self.wfile.write(encoded)
            def log_message(self, *_):
                pass
        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.shutdown); self.addCleanup(server.server_close)
        return f"http://127.0.0.1:{server.server_port}"

    def test_verified_localized_entity_is_bound_to_one_frame(self):
        scene, context = CosmosEntityVerifierClient(self.server()).verify(
            cycle_id="a" * 32, step_index=2, metadata=self.metadata, image=self.image,
            context=self.context, entity_catalog={"blue_marker": "blue navigation marker"})
        self.assertEqual(scene["verified_entities"], ["blue_marker"])
        self.assertEqual(context.task, self.context.task)

    def test_unlocalized_or_control_bearing_result_is_rejected(self):
        client = CosmosEntityVerifierClient(self.server(localized=False))
        with self.assertRaisesRegex(RuntimeError, "no localized"):
            client.verify(cycle_id="a" * 32, step_index=2, metadata=self.metadata, image=self.image,
                          context=self.context, entity_catalog={"blue_marker": "blue navigation marker"})
        client = CosmosEntityVerifierClient(self.server(dispatch=True))
        with self.assertRaisesRegex(RuntimeError, "not bound"):
            client.verify(cycle_id="a" * 32, step_index=2, metadata=self.metadata, image=self.image,
                          context=self.context, entity_catalog={"blue_marker": "blue navigation marker"})

    def test_changed_image_is_rejected_before_private_rpc(self):
        with self.assertRaisesRegex(ValueError, "does not match"):
            CosmosEntityVerifierClient("http://worker").verify(
                cycle_id="a" * 32, step_index=2, metadata=self.metadata, image=b"changed",
                context=self.context, entity_catalog={"blue_marker": "blue navigation marker"})


if __name__ == "__main__":
    unittest.main()
