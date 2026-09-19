"""Private Cosmos-worker request boundary tests; no heavyweight model load."""
import base64
import hashlib
import json
from pathlib import Path
import sys
import threading
import unittest
from http.server import ThreadingHTTPServer
from urllib.request import Request, urlopen


ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from rrm_cosmos_worker import decode_request, make_handler


def payload(image=b"fresh-image"):
    return {
        "cycle_id": "a" * 32,
        "step_index": 2,
        "observation_sha256": hashlib.sha256(image).hexdigest(),
        "context": {"task": "transport-only"},
        "image_base64": base64.b64encode(image).decode(),
    }


class CosmosWorkerBoundaryTests(unittest.TestCase):
    def test_request_requires_exact_cycle_binding_and_image_digest(self):
        value = payload()
        cycle_id, step_index, digest, context, image = decode_request(value)
        self.assertEqual((cycle_id, step_index, context, image),
                         ("a" * 32, 2, {"task": "transport-only"}, b"fresh-image"))
        self.assertEqual(digest, hashlib.sha256(image).hexdigest())
        value["observation_sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "does not match"):
            decode_request(value)
        value = payload()
        value["cycle_id"] = "not-a-cycle"
        with self.assertRaisesRegex(ValueError, "cycle ID"):
            decode_request(value)

    def test_health_and_rpc_boundary_do_not_need_a_model_for_routing(self):
        class FakeWorker:
            def propose(self, value):
                return {"echo_step": value["step_index"], "execution_dispatch": False}

        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(FakeWorker()))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        base = f"http://127.0.0.1:{server.server_port}"
        with urlopen(base + "/healthz") as response:
            self.assertEqual(json.load(response), {"status": "ready", "execution_dispatch": False})
        with urlopen(Request(base + "/v1/propose", data=json.dumps(payload()).encode(),
                             headers={"Content-Type": "application/json"})) as response:
            self.assertEqual(json.load(response), {"echo_step": 2, "execution_dispatch": False})

    def test_worker_source_has_no_vehicle_or_scheduler_control_surface(self):
        source = (ROOT / "scripts" / "rrm_cosmos_worker.py").read_text(encoding="utf-8")
        for forbidden in ("rclpy", "mavros", "px4", "ActionClient", "sbatch", "ssh "):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
