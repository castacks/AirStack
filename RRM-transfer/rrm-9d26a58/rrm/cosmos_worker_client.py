"""Private HTTP client for the warm OSMO Cosmos worker.

This module is deliberately transport-only: it converts one immutable
``LiveCycleRequest`` into one checksum-bound worker RPC and returns an unexecuted
``LiveCycleResponse``.  It has no simulator, ROS, scheduler, or action-dispatch
dependency.
"""
from __future__ import annotations

import base64
import hashlib
import json
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

from rrm.cosmos_reason2 import CosmosReasoningCandidate
from rrm.live_replan import LiveCycleRequest, LiveCycleResponse


class CosmosWorkerClient:
    """Call one group-private worker endpoint with a bounded JSON request."""

    def __init__(self, base_url: str, *, timeout_s: float = 180.0):
        parsed = urlsplit(base_url)
        if (parsed.scheme != "http" or not parsed.hostname or parsed.username or
                parsed.password or parsed.query or parsed.fragment or
                parsed.path.rstrip("/")):
            raise ValueError("Cosmos worker URL must be a plain private http://host[:port] URL.")
        if timeout_s <= 0 or timeout_s > 600:
            raise ValueError("Cosmos worker timeout must be between 0 and 600 seconds.")
        self.base_url = base_url.rstrip("/")
        self.timeout_s = timeout_s

    def propose(self, request: LiveCycleRequest) -> LiveCycleResponse:
        image = request.image_path.read_bytes()
        checksum = hashlib.sha256(image).hexdigest()
        if checksum != request.observation["sha256"]:
            raise ValueError("Worker request image no longer matches the active observation.")
        body = json.dumps({
            "cycle_id": request.cycle_id,
            "step_index": request.step_index,
            "observation_sha256": checksum,
            "context": _context_payload(request),
            "image_base64": base64.b64encode(image).decode("ascii"),
        }, separators=(",", ":")).encode("utf-8")
        rpc = Request(self.base_url + "/v1/propose", data=body,
                      headers={"Content-Type": "application/json", "Accept": "application/json"},
                      method="POST")
        try:
            with urlopen(rpc, timeout=self.timeout_s) as response:
                if response.status != 200:
                    raise RuntimeError("Cosmos worker rejected the live proposal request.")
                raw = response.read(4 * 1024 * 1024 + 1)
        except HTTPError as error:
            raise RuntimeError("Cosmos worker rejected the live proposal request.") from error
        except URLError as error:
            raise RuntimeError("Cosmos worker is unavailable on the private OSMO network.") from error
        if len(raw) > 4 * 1024 * 1024:
            raise RuntimeError("Cosmos worker response exceeds the allowed size.")
        try:
            value = json.loads(raw)
            candidate = CosmosReasoningCandidate.model_validate(value["candidate"])
            response = LiveCycleResponse(
                cycle_id=value["cycle_id"], step_index=value["step_index"],
                observation_sha256=value["observation_sha256"], candidate=candidate,
            )
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError("Cosmos worker returned an invalid live proposal response.") from error
        if value.get("execution_dispatch") is not False:
            raise RuntimeError("Cosmos worker response violated the no-dispatch contract.")
        return response


def _context_payload(request: LiveCycleRequest) -> dict:
    """Serialize the exact C01/C02/C03 shape accepted by the worker runtime."""
    context = request.context
    return {
        "task": context.task.model_dump(mode="json"),
        "snapshot": context.snapshot.model_dump(mode="json"),
        "capabilities": {
            "embodiment_id": context.capabilities.embodiment_id,
            "revision": context.capabilities.revision,
            "operations": sorted(context.capabilities.operations),
            "resources": sorted(context.capabilities.resources),
            "available_resources": sorted(context.capabilities.available_resources),
            "limits_ref": context.capabilities.limits_ref,
        },
        "now_monotonic_s": context.now_monotonic_s,
    }
