"""Convert one synchronous renderer payload into assessment-only teacher records.

This pure boundary does not acquire pixels or authenticate the renderer. The
acquisition adapter must bind all inputs to one render product/reference time.
"""
import hashlib
import colorsys
import statistics
from typing import Mapping

from .visual_evaluation import canonical_sha256, validate_visual_pair


def qualify_probe_colors(samples: Mapping[str, list[list[int]]]) -> dict:
    """Frozen emissive-v1 fixture gate, not general perception/grounding truth."""
    bounds = {'blue_marker': (190, 255), 'orange_marker': (5, 35)}
    records = {}
    if not samples or set(samples) - set(bounds):
        raise ValueError('invalid probe color entities')
    for entity, pixels in samples.items():
        if not pixels or any(len(pixel) != 3 or any(type(v) is not int or not 0 <= v <= 255
                                                   for v in pixel) for pixel in pixels):
            raise ValueError('invalid probe RGB samples')
        low, high = bounds[entity]
        accepted = 0
        for pixel in pixels:
            hue, saturation, value = colorsys.rgb_to_hsv(*(v / 255 for v in pixel))
            accepted += low <= hue * 360 <= high and saturation >= .65 and value >= .3
        median = [statistics.median(pixel[channel] for pixel in pixels) for channel in range(3)]
        hue, saturation, value = colorsys.rgb_to_hsv(*(v / 255 for v in median))
        records[entity] = dict(pixel_count=len(pixels), median_rgb=median,
            median_hue_degrees=hue * 360, median_saturation=saturation, median_value=value,
            qualifying_pixel_fraction=accepted / len(pixels), passed=accepted / len(pixels) >= .9)
    return dict(profile='emissive-v1', passed=all(row['passed'] for row in records.values()),
                entities=records, hue_bounds_degrees=bounds, min_saturation=.65, min_value=.3,
                min_qualifying_pixel_fraction=.9)


def reference_stamp_ns(numerator: int, denominator: int) -> int:
    if (type(numerator) is not int or type(denominator) is not int
            or numerator < 0 or denominator <= 0):
        raise ValueError("invalid render reference time")
    # Integer truncation is recorded with the original rational in acquisition data.
    return numerator * 1_000_000_000 // denominator


def semantic_regions(rows: list[list[int]], id_to_labels: Mapping,
                     entity_ids: set[str]) -> dict[str, list[int]]:
    """Half-open bounds of visible semantic pixels; missing pixels stay unknown."""
    if not rows or not rows[0] or any(len(row) != len(rows[0]) for row in rows):
        raise ValueError("segmentation must be a nonempty rectangular image")
    label_map = {}
    seen_entities = set()
    for raw_id, label in id_to_labels.items():
        if (not isinstance(raw_id, str) or not raw_id.isdecimal()
                or str(int(raw_id)) != raw_id or not isinstance(label, dict)):
            raise ValueError("malformed segmentation label mapping")
        entity = label.get("class")
        if not isinstance(entity, str):
            raise ValueError("semantic class must be an explicit string")
        if entity in entity_ids:
            if entity in seen_entities:
                raise ValueError("ambiguous duplicate semantic entity mapping")
            seen_entities.add(entity)
            label_map[int(raw_id)] = entity
    known_ids = {int(raw_id) for raw_id in id_to_labels}
    regions = {}
    for y, row in enumerate(rows):
        for x, semantic_id in enumerate(row):
            if type(semantic_id) is not int or semantic_id < 0 or semantic_id not in known_ids:
                raise ValueError("unmapped or invalid segmentation pixel")
            entity = label_map.get(semantic_id)
            if entity is None:
                continue
            box = regions.setdefault(entity, [x, y, x + 1, y + 1])
            box[:] = [min(box[0], x), min(box[1], y), max(box[2], x + 1), max(box[3], y + 1)]
    return regions


def build_render_pair(*, image: bytes, semantic_pixels: list[list[int]], id_to_labels: Mapping,
                      stage_entities: list[dict], manifest: Mapping, launcher_sha256: str,
                      producer_ref: str, episode_id: str, scene_revision: str,
                      reference_numerator: int, reference_denominator: int) -> dict:
    """Construct a pair only from stage observations and same-render annotations.

    stage_entities contains measured active prim identity/kind/existence, not merely
    the expected catalog. An absent prim still has a registered expected identity.
    """
    stamp = reference_stamp_ns(reference_numerator, reference_denominator)
    expected = {entity: {"entity_id": entity, "scene_prim": item["scene_prim"], "kind": item["kind"]}
                for entity, item in manifest["markers"].items()}
    if len(stage_entities) != len(expected):
        raise ValueError("incomplete stage entity observations")
    observed = {}
    for item in stage_entities:
        entity = item.get("entity_id")
        if (entity not in expected or entity in observed or type(item.get("exists")) is not bool
                or set(item) != {"entity_id", "scene_prim", "kind", "exists"}
                or {key: item[key] for key in expected[entity]} != expected[entity]):
            raise ValueError("stage entity identity mismatch")
        observed[entity] = item
    regions = semantic_regions(semantic_pixels, id_to_labels, set(expected))
    frame = dict(episode_id=episode_id, scene_revision=scene_revision, topic=manifest["camera_topic"],
                 frame_id=manifest["camera_frame_id"], source_stamp_ns=stamp,
                 sha256=hashlib.sha256(image).hexdigest(), width=len(semantic_pixels[0]),
                 height=len(semantic_pixels))
    manifest_hash = canonical_sha256(manifest)
    registration = dict(schema_version="rrm-visual-fixture/v1", fixture_id=manifest["scene_id"],
                        episode_id=episode_id, scene_revision=scene_revision,
                        manifest_sha256=manifest_hash, launcher_sha256=launcher_sha256,
                        producer_ref=producer_ref, camera_topic=frame["topic"],
                        camera_frame_id=frame["frame_id"], entities=list(expected.values()))
    labels = []
    for entity, identity in expected.items():
        exists = observed[entity]["exists"]
        region = regions.get(entity)
        if region is not None and not exists:
            raise ValueError("rendered entity contradicts stage absence")
        labels.append(dict(identity, exists=exists, image_region=region,
                           visibility="ABSENT" if not exists else "VISIBLE" if region else "UNKNOWN"))
    teacher = dict(schema_version="rrm-visual-teacher-frame/v1", provenance="SIMULATOR",
                   producer_ref=producer_ref, fixture_id=manifest["scene_id"],
                   manifest_sha256=manifest_hash, frame=dict(frame), labels=labels)
    report = validate_visual_pair(image=image, frame=frame, registration=registration, teacher=teacher,
                                  manifest=manifest, launcher_sha256=launcher_sha256,
                                  expected_teacher_source_ref=producer_ref,
                                  assessment_episode_id=episode_id, assessment_source_stamp_ns=stamp,
                                  max_age_ns=1)
    if report["status"] != "BOUND_FOR_ASSESSMENT":
        raise ValueError("render pair rejected: " + report["reason"])
    return dict(frame=frame, registration=registration, teacher=teacher, pairing_report=report,
                render_reference=dict(numerator=reference_numerator, denominator=reference_denominator),
                execution_dispatch=False)
