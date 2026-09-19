#!/usr/bin/env python3
"""Private persistent Cosmos Reason2 worker for an OSMO live-replan group.

The worker loads one local model snapshot at startup and accepts one image/context
request at a time. It returns an unexecuted C04/C05 candidate bound to the caller's
cycle, step, and image digest. It has no ROS, PSC, action, or vehicle-control path.
"""
from __future__ import annotations

import argparse
import base64
import binascii
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hashlib
import json
from pathlib import Path
import re
import tempfile
import threading
import time

from rrm.cosmos_reason2 import parse_cosmos_candidate, render_cosmos_prompt
from rrm_cosmos_reason2 import CosmosGenerator, load_context_payload


MAX_REQUEST_BYTES = 12 * 1024 * 1024
HEX_ID = re.compile(r"[0-9a-f]{32}")


def decode_request(value: dict) -> tuple[str, int, str, dict, bytes]:
    """Validate and decode one provider request before it reaches the model."""
    if not isinstance(value, dict):
        raise ValueError("worker request must be an object")
    cycle_id = value.get("cycle_id")
    step_index = value.get("step_index")
    observation_sha256 = value.get("observation_sha256")
    context = value.get("context")
    image_base64 = value.get("image_base64")
    if not isinstance(cycle_id, str) or not HEX_ID.fullmatch(cycle_id):
        raise ValueError("worker request has an invalid cycle ID")
    if type(step_index) is not int or step_index < 0:
        raise ValueError("worker request has an invalid step index")
    if not isinstance(observation_sha256, str) or not re.fullmatch(r"[0-9a-f]{64}", observation_sha256):
        raise ValueError("worker request has an invalid observation checksum")
    if not isinstance(context, dict) or not isinstance(image_base64, str):
        raise ValueError("worker request is missing context or image")
    try:
        image = base64.b64decode(image_base64, validate=True)
    except (ValueError, binascii.Error) as error:
        raise ValueError("worker image is not valid base64") from error
    if not image or hashlib.sha256(image).hexdigest() != observation_sha256:
        raise ValueError("worker image does not match the active observation")
    return cycle_id, step_index, observation_sha256, context, image


class CosmosWorker:
    """Serializes generation through a single warm model instance."""

    def __init__(self, model_path: str, *, max_new_tokens: int):
        if max_new_tokens <= 0:
            raise ValueError("max_new_tokens must be positive")
        if not Path(model_path).is_dir():
            raise ValueError("Cosmos model path is unavailable; mount or bake an approved local snapshot.")
        self.model_path = model_path
        self.max_new_tokens = max_new_tokens
        self.generator = CosmosGenerator(model_path)
        self.lock = threading.Lock()

    def propose(self, value: dict) -> dict:
        cycle_id, step_index, observation_sha256, payload, image = decode_request(value)
        context = load_context_payload(payload)
        prompt = render_cosmos_prompt(context)
        with tempfile.TemporaryDirectory(prefix="rrm-cosmos-request-") as temporary:
            image_path = Path(temporary) / "input.png"
            image_path.write_bytes(image)
            started = time.monotonic()
            with self.lock:
                raw = self.generator.generate(prompt=prompt, image_path=str(image_path),
                                              video_path=None, max_new_tokens=self.max_new_tokens)
        candidate = parse_cosmos_candidate(raw, context)
        return {
            "cycle_id": cycle_id,
            "step_index": step_index,
            "observation_sha256": observation_sha256,
            "candidate": candidate.model_dump(mode="json"),
            "inference_wall_s": round(time.monotonic() - started, 3),
            "execution_dispatch": False,
        }


def make_handler(worker: CosmosWorker):
    class Handler(BaseHTTPRequestHandler):
        def respond(self, value: dict, status: int = 200):
            body = json.dumps(value).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/healthz":
                return self.respond({"status": "ready", "execution_dispatch": False})
            self.respond({"error": "Not found"}, status=404)

        def do_POST(self):
            if self.path != "/v1/propose":
                return self.respond({"error": "Not found"}, status=404)
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= MAX_REQUEST_BYTES:
                    raise ValueError("worker request size is invalid")
                value = json.loads(self.rfile.read(length))
                return self.respond(worker.propose(value))
            except (ValueError, TypeError, json.JSONDecodeError) as error:
                self.respond({"error": str(error)}, status=400)
            except Exception:
                # Do not return model paths, stack traces, or credential details over
                # the private RPC boundary. Operators inspect the worker task logs.
                self.respond({"error": "Cosmos worker inference failed"}, status=503)
    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-path", required=True)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--max-new-tokens", type=int, default=768)
    args = parser.parse_args()
    worker = CosmosWorker(args.model_path, max_new_tokens=args.max_new_tokens)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(worker))
    print(f"RRM Cosmos worker listening on {args.host}:{args.port}; execution_dispatch=false", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
