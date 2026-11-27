"""Select a single validated pair using native references delivered by Replicator."""

from collections.abc import Mapping
from dataclasses import dataclass
import json
from numbers import Integral
import traceback

from recorder import HEIGHT, WIDTH, append_record, finite_number, has_target_depth

DEPTH = "distance_to_image_plane"
INSTANCE = "instance_id_segmentation"
MAX_ATTEMPTS = 8
ASSOCIATION_SOURCE = "replicator_writer_reference_time"
UNVERIFIED = "UNVERIFIED_FRAME_ASSOCIATION"


@dataclass
class Packet:
    reference: tuple | None
    array: object
    labels: object = None


@dataclass
class Frame:
    depth: bytes
    instances: bytes
    labels: dict
    association: dict


def native_reference(data):
    if not isinstance(data, Mapping):
        return None
    numerator = data.get("referenceTimeNumerator")
    denominator = data.get("referenceTimeDenominator")
    if (isinstance(numerator, Integral) and not isinstance(numerator, bool)
            and isinstance(denominator, Integral) and not isinstance(denominator, bool)
            and numerator >= 0 and denominator > 0):
        return int(numerator), int(denominator)
    return None


def writer_packet(data, annotator):
    """Decode the single-product writer's native annotation packet."""
    payload = data.get(annotator)
    reference = native_reference(data.get("ReferenceTime"))
    if not isinstance(payload, Mapping):
        return Packet(reference, payload)
    info = payload.get("info", payload)
    labels = info.get("idToLabels") if isinstance(info, Mapping) else None
    return Packet(reference, payload.get("data"), labels)


def payload_status(packet, dtype):
    array = packet.array
    shape = getattr(array, "shape", None)
    actual_type = getattr(getattr(array, "dtype", None), "name", None)
    record = {"reference": packet.reference, "shape": list(shape) if shape is not None else None,
              "dtype": actual_type, "status": "valid"}
    if array is None:
        record["status"] = "missing"
    elif not getattr(array, "size", 0) or actual_type != dtype:
        record["status"] = "empty_or_malformed"
    elif shape != (HEIGHT, WIDTH) and not (dtype == "uint32" and shape == (HEIGHT, WIDTH, 1)):
        record["status"] = "unexpected_dimensions"
    return record


def evaluate(depth, instances, sim_time):
    depth_status = payload_status(depth, "float32")
    instance_status = payload_status(instances, "uint32")
    details = {"depth": depth_status, "instances": instance_status}
    if not all(item["status"] == "valid" for item in details.values()):
        return "PAYLOAD_NOT_READY", details, None
    if depth.reference is None or instances.reference is None:
        return UNVERIFIED, details, None
    if depth.reference != instances.reference:
        return "FRAME_MISMATCH", details, None
    rendering_time = sim_time(depth.reference)
    if not finite_number(rendering_time) or rendering_time < 0:
        return UNVERIFIED, details, None

    try:
        labels = json.loads(instances.labels) if isinstance(instances.labels, str) else instances.labels
    except json.JSONDecodeError:
        labels = None
    if not isinstance(labels, Mapping):
        instance_status["status"] = "invalid_labels"
        return "PAYLOAD_NOT_READY", details, None
    labels = {str(key): value for key, value in labels.items()}
    depth_bytes = depth.array.astype("<f4", copy=False).tobytes()
    ids_bytes = instances.array.astype("<u4", copy=False).tobytes()
    if len(depth_bytes) != WIDTH * HEIGHT * 4 or len(ids_bytes) != len(depth_bytes):
        return "INVALID_BUFFER_SIZE", details, None
    if not has_target_depth(depth_bytes, ids_bytes, labels):
        return "NO_VALID_TARGET_DEPTH", details, None

    association = {
        "source": ASSOCIATION_SOURCE, "depth_reference": list(depth.reference),
        "instance_reference": list(instances.reference), "rendering_time_s": rendering_time,
    }
    return "READY", details, Frame(depth_bytes, ids_bytes, labels, association)


def acquire(output, step_and_read, sim_time):
    journal = output / "readiness.jsonl"
    association_failure = False
    for attempt in range(1, MAX_ATTEMPTS + 1):
        append_record(journal, {"attempt": attempt, "status": "STARTED"})
        try:
            depth, instances = step_and_read()
            status, details, frame = evaluate(depth, instances, sim_time)
        except BaseException:
            append_record(journal, {"attempt": attempt, "status": "EXCEPTION",
                                    "traceback": traceback.format_exc()})
            raise
        event = {"attempt": attempt, "status": status, **details}
        if frame is not None:
            frame.association["attempt"] = attempt
            event["association"] = frame.association
        append_record(journal, event)
        if frame is not None:
            return frame
        association_failure |= status in (UNVERIFIED, "FRAME_MISMATCH")

    reason = UNVERIFIED if association_failure else "READINESS_BOUND_EXHAUSTED"
    raise RuntimeError(f"{reason} after {MAX_ATTEMPTS} render/update attempts")
