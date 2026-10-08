"""Read-only worker capability inspection; never calls inference or dispatch."""
import json
import re
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from rrm.cosmos_entity_verifier_client import CosmosEntityVerifierClient


def probe_visual_service(base_url: str) -> dict:
    report = {"schema_version": "rrm-visual-service/v1", "status": "BLOCKED",
              "entity_api_declared": False, "execution_dispatch": False}
    if not base_url:
        return {**report, "reason": "No private visual worker is configured."}
    try:
        client = CosmosEntityVerifierClient(base_url, timeout_s=4.0)
        request = Request(client.base_url + "/v1/capabilities", headers={"Accept": "application/json"})
        with urlopen(request, timeout=4.0) as response:
            raw = response.read(4097)
        if len(raw) > 4096:
            raise ValueError("Oversized capability response")
        value = json.loads(raw)
        if (not isinstance(value, dict) or value.get("schema_version") != "rrm-cosmos-capabilities/v1"
                or value.get("execution_dispatch") is not False
                or not isinstance(value.get("routes"), list)
                or "/v1/verify-entities" not in value["routes"]
                or not isinstance(value.get("worker_source_sha256"), str)
                or not re.fullmatch(r"[0-9a-f]{64}", value["worker_source_sha256"])):
            raise ValueError("Invalid capability response")
        return {**report, "status": "DECLARED", "entity_api_declared": True,
                "worker_source_sha256": value["worker_source_sha256"],
                "reason": "Worker declares entity grounding. Model revision, scene labels and visual accuracy still require verification."}
    except HTTPError as error:
        return {**report, "http_status": error.code,
                "reason": "Worker capability API is unavailable; deploy the current worker before visual evaluation."}
    except (URLError, OSError, ValueError, TypeError):
        return {**report, "reason": "Visual worker capability check failed or returned invalid evidence."}
