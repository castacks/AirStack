"""Private transport for checksum-bound live visual entity evidence.

The worker endpoint returns only C02-style visual evidence.  It cannot return a C05
plan or dispatch authorization; callers combine the accepted snapshot with their
unchanged C01/C03 context before separately requesting a planner proposal.
"""
from __future__ import annotations

import base64
import hashlib
import json
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

from rrm.contracts import Truth
from rrm.cosmos_reason2 import CosmosReasoningInput
from rrm.cosmos_worker_client import _context_payload
from rrm.state_contracts import FactKey
from rrm.visual_world_builder import VisualCandidateStatus, VisualGroundingCandidate


class CosmosEntityVerifierClient:
    def __init__(self, base_url: str, *, timeout_s: float = 180.0):
        parsed = urlsplit(base_url)
        if (parsed.scheme != "http" or not parsed.hostname or parsed.username or
                parsed.password or parsed.query or parsed.fragment or parsed.path.rstrip("/")):
            raise ValueError("Cosmos worker URL must be a plain private http://host[:port] URL.")
        if timeout_s <= 0 or timeout_s > 600:
            raise ValueError("Entity verifier timeout must be between 0 and 600 seconds.")
        self.base_url, self.timeout_s = base_url.rstrip("/"), timeout_s

    def verify(self, *, cycle_id: str, step_index: int, metadata: dict, image: bytes,
               context: CosmosReasoningInput, entity_catalog: dict[str, str]) -> tuple[dict, CosmosReasoningInput]:
        checksum = hashlib.sha256(image).hexdigest()
        if metadata.get("sha256") != checksum:
            raise ValueError("Entity verifier image does not match observation metadata.")
        body = json.dumps({
            "cycle_id": cycle_id, "step_index": step_index, "observation_sha256": checksum,
            "context": _context_payload_for_context(context), "entity_catalog": entity_catalog,
            "image_base64": base64.b64encode(image).decode("ascii"),
        }, separators=(",", ":")).encode("utf-8")
        request = Request(self.base_url + "/v1/verify-entities", data=body,
                          headers={"Content-Type": "application/json", "Accept": "application/json"}, method="POST")
        try:
            with urlopen(request, timeout=self.timeout_s) as response:
                if response.status != 200:
                    raise RuntimeError("Cosmos entity verifier rejected the live frame.")
                raw = response.read(4 * 1024 * 1024 + 1)
        except HTTPError as error:
            raise RuntimeError("Cosmos entity verifier rejected the live frame.") from error
        except URLError as error:
            raise RuntimeError("Cosmos entity verifier is unavailable on the private OSMO network.") from error
        if len(raw) > 4 * 1024 * 1024:
            raise RuntimeError("Cosmos entity verifier response exceeds the allowed size.")
        try:
            value = json.loads(raw)
            candidate = VisualGroundingCandidate.model_validate(value["visual_candidate"])
            now = value["now_monotonic_s"]
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError("Cosmos entity verifier returned invalid evidence.") from error
        if (value.get("execution_dispatch") is not False or value.get("cycle_id") != cycle_id
                or value.get("step_index") != step_index or value.get("observation_sha256") != checksum):
            raise RuntimeError("Cosmos entity verifier response is not bound to the active frame.")
        if candidate.status is not VisualCandidateStatus.ACCEPTED or candidate.snapshot is None:
            raise RuntimeError("Cosmos entity verifier did not produce accepted visual evidence.")
        verified = sorted({item.key.subject for item in candidate.snapshot.evidence
                           if item.truth is Truth.TRUE and item.key.predicate == "exists"
                           and candidate.snapshot.resolve(FactKey(subject=item.key.subject, predicate="localized"),
                                                          now_monotonic_s=now) is Truth.TRUE})
        if not verified:
            raise RuntimeError("Cosmos entity verifier proved no localized catalog entity in this frame.")
        scene = {
            "source_stamp_ns": metadata["source_stamp_ns"], "observation_sha256": checksum,
            "verified_entities": verified,
            "provenance": "cosmos-reason2-live-entity-verifier/v1",
        }
        return scene, CosmosReasoningInput(task=context.task, snapshot=candidate.snapshot,
                                           capabilities=context.capabilities, now_monotonic_s=now)


def _context_payload_for_context(context: CosmosReasoningInput) -> dict:
    """Reuse the worker's exact C01/C02/C03 serialization without a fake cycle request."""
    class RequestShape:
        pass
    request = RequestShape()
    request.context = context
    return _context_payload(request)
