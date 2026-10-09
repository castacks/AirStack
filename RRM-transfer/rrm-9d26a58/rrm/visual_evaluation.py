"""Frozen image/independent-teacher pairing, not scene acquisition or authority.

The caller owns trusted manifest, launcher/exporter identity and same-epoch clock.
Checksums bind supplied records; they do not authenticate an exporter or prove its
labels correct. Teacher records stay assessment-only, never candidate input.
"""
from __future__ import annotations

import hashlib
import json
import struct
import zlib
from typing import Literal, Mapping

from pydantic import BaseModel, ConfigDict, Field, model_validator


class _Record(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid", strict=True)


class FrameBinding(_Record):
    episode_id: str = Field(min_length=1)
    scene_revision: str = Field(min_length=1)
    topic: str = Field(min_length=1)
    frame_id: str = Field(min_length=1)
    source_stamp_ns: int = Field(ge=0)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    width: int = Field(gt=0)
    height: int = Field(gt=0)


class RegisteredEntity(_Record):
    entity_id: str = Field(min_length=1)
    scene_prim: str = Field(pattern=r"^/[^\s]+$")
    kind: str = Field(min_length=1)


class FixtureRegistration(_Record):
    schema_version: Literal["rrm-visual-fixture/v1"]
    fixture_id: str = Field(min_length=1)
    episode_id: str = Field(min_length=1)
    scene_revision: str = Field(min_length=1)
    manifest_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    launcher_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    producer_ref: str = Field(min_length=1)
    camera_topic: str = Field(min_length=1)
    camera_frame_id: str = Field(min_length=1)
    entities: tuple[RegisteredEntity, ...]


class TeacherLabel(RegisteredEntity):
    exists: bool
    visibility: Literal["VISIBLE", "OCCLUDED", "UNKNOWN", "ABSENT"]
    # Pixel bounds [left, top, right, bottom), not a waypoint or reachability claim.
    image_region: tuple[int, int, int, int] | None = None

    @model_validator(mode="after")
    def consistent_visibility(self) -> "TeacherLabel":
        if (not self.exists) != (self.visibility == "ABSENT"):
            raise ValueError("absence requires explicit absent visibility")
        if (self.visibility == "VISIBLE") != (self.image_region is not None):
            raise ValueError("only a visible label has an image region")
        return self


class TeacherFrame(_Record):
    schema_version: Literal["rrm-visual-teacher-frame/v1"]
    provenance: Literal["SIMULATOR"]
    producer_ref: str = Field(min_length=1)
    fixture_id: str = Field(min_length=1)
    manifest_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    frame: FrameBinding
    labels: tuple[TeacherLabel, ...]


def canonical_sha256(value: Mapping) -> str:
    """Hash JSON semantics; reject NaN/Infinity rather than canonicalizing them."""
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


def _from_mapping(model, value: Mapping):
    # JSON arrays are legitimate tuples; strict JSON validation still rejects bool
    # stamps, numeric strings, extra fields and non-finite region coordinates.
    return model.model_validate_json(json.dumps(value, allow_nan=False))


def validate_visual_pair(
    *, image: bytes, frame: Mapping, registration: Mapping | None,
    teacher: Mapping | None, manifest: Mapping, launcher_sha256: str,
    expected_teacher_source_ref: str, assessment_episode_id: str,
    assessment_source_stamp_ns: int, max_age_ns: int,
) -> dict:
    """Return BOUND_FOR_ASSESSMENT or BLOCKED with no execution/score authority.

    Exact source stamps require synchronous same-frame teacher acquisition. A
    nearby pose/time, static snapshot or matching image hash alone is insufficient.
    This gate is opt-in: it does not retrofit or certify older evaluation runners.
    """
    report = {"schema_version": "rrm-visual-pairing/v1", "status": "BLOCKED",
              "execution_dispatch": False, "teacher_sent_to_candidate": False,
              "scored": False, "image_sha256": hashlib.sha256(image).hexdigest()}

    def blocked(reason: str) -> dict:
        return {**report, "reason": reason}

    if (not image or type(assessment_source_stamp_ns) is not int
            or assessment_source_stamp_ns < 0 or type(max_age_ns) is not int
            or max_age_ns <= 0 or not isinstance(expected_teacher_source_ref, str)
            or not expected_teacher_source_ref.strip()
            or not isinstance(assessment_episode_id, str) or not assessment_episode_id.strip()):
        return blocked("invalid_assessment_context")
    if registration is None:
        return blocked("fixture_registration_missing")
    if teacher is None:
        return blocked("independent_teacher_missing")
    try:
        capture = _from_mapping(FrameBinding, frame)
        fixture = _from_mapping(FixtureRegistration, registration)
        labels = _from_mapping(TeacherFrame, teacher)
        manifest_hash = canonical_sha256(manifest)
        expected_entities = tuple(sorted(
            (entity_id, item["scene_prim"], item["kind"])
            for entity_id, item in manifest["markers"].items()))
        if not expected_entities:
            return blocked("empty_fixture_catalog")
    except (ValueError, TypeError, KeyError, AttributeError):
        return blocked("malformed_pairing_evidence")
    if (fixture.fixture_id != manifest.get("scene_id")
            or fixture.manifest_sha256 != manifest_hash
            or fixture.launcher_sha256 != launcher_sha256
            or fixture.camera_topic != manifest.get("camera_topic")
            or fixture.camera_frame_id != manifest.get("camera_frame_id")):
        return blocked("fixture_identity_mismatch")
    if (fixture.producer_ref != expected_teacher_source_ref
            or labels.producer_ref != expected_teacher_source_ref):
        return blocked("teacher_source_mismatch")
    observed_entities = tuple(sorted((e.entity_id, e.scene_prim, e.kind)
                                     for e in fixture.entities))
    teacher_entities = tuple(sorted((e.entity_id, e.scene_prim, e.kind)
                                    for e in labels.labels))
    if observed_entities != expected_entities or teacher_entities != expected_entities:
        return blocked("entity_labels_incomplete_or_mismatched")
    if (capture.episode_id != fixture.episode_id
            or capture.episode_id != assessment_episode_id
            or capture.scene_revision != fixture.scene_revision):
        return blocked("scene_epoch_or_revision_mismatch")
    if (labels.fixture_id != fixture.fixture_id
            or labels.manifest_sha256 != manifest_hash or labels.frame != capture):
        return blocked("teacher_frame_mismatch")
    if (capture.topic != fixture.camera_topic
            or capture.frame_id != fixture.camera_frame_id):
        return blocked("camera_identity_mismatch")
    if capture.sha256 != report["image_sha256"]:
        return blocked("image_integrity_mismatch")
    # Current capture utility emits PNG. Check IHDR dimensions/CRC, not decoded
    # visibility; the acquisition adapter must separately qualify pixel decoding.
    if (len(image) < 33 or image[:8] != b"\x89PNG\r\n\x1a\n"
            or image[8:16] != b"\x00\x00\x00\rIHDR"
            or zlib.crc32(image[12:29]) != struct.unpack(">I", image[29:33])[0]):
        return blocked("invalid_png_header")
    if struct.unpack(">II", image[16:24]) != (capture.width, capture.height):
        return blocked("image_dimensions_mismatch")
    age = assessment_source_stamp_ns - capture.source_stamp_ns
    if age < 0 or age > max_age_ns:
        return blocked("future_or_stale_frame")
    for label in labels.labels:
        if label.image_region is not None:
            left, top, right, bottom = label.image_region
            if not (0 <= left < right <= capture.width
                    and 0 <= top < bottom <= capture.height):
                return blocked("invalid_image_region")
    return {**report, "status": "BOUND_FOR_ASSESSMENT", "reason": "bound_records_only",
            "fixture_id": fixture.fixture_id, "episode_id": capture.episode_id,
            "scene_revision": capture.scene_revision, "source_stamp_ns": capture.source_stamp_ns,
            "observation_age_ns": age, "teacher_sha256": canonical_sha256(teacher),
            "registration_sha256": canonical_sha256(registration),
            "label_count": len(labels.labels)}
