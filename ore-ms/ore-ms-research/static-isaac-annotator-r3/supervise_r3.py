"""Apply the R3 readiness contract before the recorder may certify completion."""

from functools import partial
import json

import recorder
from readiness import ASSOCIATION_SOURCE, MAX_ATTEMPTS, UNVERIFIED

PHASES = ("isaac_startup", "stage_open", "camera_initialize", "annotators_attach",
          "frame_readiness", "write_artifacts", "shutdown")
RETRY_STATUS = {"PAYLOAD_NOT_READY", UNVERIFIED, "FRAME_MISMATCH", "INVALID_BUFFER_SIZE",
                "NO_VALID_TARGET_DEPTH"}
JOURNAL = "readiness.jsonl"


def validate_capture(capture, mode):
    recorder.validate_capture(capture, mode, phases=PHASES, schema=2)
    metadata = json.loads((capture / "frame.json").read_text())
    association = metadata["frame_association"]
    attempt = association["attempt"]
    depth_ref = association["depth_reference"]
    instance_ref = association["instance_reference"]
    if (association["source"] != ASSOCIATION_SOURCE
            or not isinstance(depth_ref, list) or len(depth_ref) != 2
            or any(type(value) is not int for value in depth_ref)
            or depth_ref[0] < 0 or depth_ref[1] <= 0
            or instance_ref != depth_ref
            or any(type(value) is not int for value in instance_ref)
            or not recorder.finite_number(association["rendering_time_s"])
            or association["rendering_time_s"] != metadata["rendering_time_s"]
            or type(attempt) is not int or not 1 <= attempt <= MAX_ATTEMPTS
            or not isinstance(metadata["render_product"], str)
            or not metadata["render_product"].startswith("/")):
        raise ValueError(UNVERIFIED)

    rows = [json.loads(line) for line in (capture / JOURNAL).read_text().splitlines()]
    if len(rows) != 2 * attempt:
        raise ValueError("incomplete readiness attempts")
    for index, (start, result) in enumerate(zip(rows[::2], rows[1::2]), start=1):
        allowed = {"READY"} if index == attempt else RETRY_STATUS
        if (start["attempt"] != index or start["status"] != "STARTED"
                or result["attempt"] != index or result["status"] not in allowed):
            raise ValueError("invalid readiness attempt sequence")
    ready = rows[-1]
    if (ready["association"] != association
            or any(ready[key]["status"] != "valid" or ready[key]["reference"] != depth_ref
                   for key in ("depth", "instances"))):
        raise ValueError(UNVERIFIED)


supervise = partial(recorder.supervise, validator=validate_capture,
                    artifacts=(*recorder.ARTIFACTS, JOURNAL))


if __name__ == "__main__":
    recorder.main(supervise)
