"""Tests for the private worker transport boundary; no model or control path."""
import base64
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from rrm.cosmos_worker_client import CosmosWorkerClient
from rrm.live_replan import LiveCycleRequest
from rrm_cosmos_reason2 import load_context


class CosmosWorkerClientTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        fixture = ROOT / "examples" / "office_visual_eval"
        self.context_path = Path(self.temp.name) / "context.json"
        shutil.copy(fixture / "navigation_context.json", self.context_path)
        self.context = load_context(self.context_path)
        self.image_path = Path(self.temp.name) / "input.png"
        self.image_path.write_bytes(b"fresh-live-image")

    def request(self):
        image = self.image_path.read_bytes()
        return LiveCycleRequest(
            cycle_id="b" * 32, step_index=3, context=self.context,
            observation={"sha256": hashlib.sha256(image).hexdigest()}, image_path=self.image_path,
            scene_state={"verified_entities": ["blue_marker"]}, prior_outcome=None,
            prior_plan_action_ids=(),
        )

    def server(self, *, dispatch=False):
        outer = self
        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                assert self.path == "/v1/propose"
                assert base64.b64decode(body["image_base64"]) == outer.image_path.read_bytes()
                result = {
                    "cycle_id": body["cycle_id"], "step_index": body["step_index"],
                    "observation_sha256": body["observation_sha256"],
                    "candidate": {
                        "status": "REJECTED", "task_id": body["context"]["task"]["task_id"],
                        "raw_response": "test rejection", "reasons": ["test rejection"], "plan": None,
                    },
                    "execution_dispatch": dispatch,
                }
                encoded = json.dumps(result).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)
            def log_message(self, *_):
                pass
        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.shutdown)
        self.addCleanup(server.server_close)
        return f"http://127.0.0.1:{server.server_port}"

    def test_client_binds_image_context_cycle_and_returns_no_dispatch_response(self):
        response = CosmosWorkerClient(self.server()).propose(self.request())
        self.assertEqual(response.cycle_id, "b" * 32)
        self.assertEqual(response.step_index, 3)
        self.assertEqual(response.candidate.status.value, "REJECTED")

    def test_client_rejects_dispatch_or_mutated_input(self):
        with self.assertRaisesRegex(RuntimeError, "no-dispatch"):
            CosmosWorkerClient(self.server(dispatch=True)).propose(self.request())
        request = self.request()
        self.image_path.write_bytes(b"mutated-after-observation")
        with self.assertRaisesRegex(ValueError, "no longer matches"):
            CosmosWorkerClient(self.server()).propose(request)

    def test_client_refuses_non_private_style_urls(self):
        for url in ("https://worker", "http://user@worker", "http://worker/path", "http://worker?x=1"):
            with self.subTest(url=url), self.assertRaisesRegex(ValueError, "plain private"):
                CosmosWorkerClient(url)

    def test_client_source_has_no_vehicle_or_scheduler_control_surface(self):
        source = (ROOT / "rrm" / "cosmos_worker_client.py").read_text(encoding="utf-8")
        for forbidden in ("rclpy", "mavros", "px4", "ActionClient", "subprocess", "sbatch", "ssh "):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
