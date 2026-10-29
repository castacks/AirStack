"""Adapters for simulator pixel counts and the coordinator demo's evidence rows."""

from collections.abc import Mapping

from .model import BODY_PARTS, Observation, _integer


def parts_from_placement(placement, *, field="visible_parts"):
    """Assume listed parts are fully exposed and all other parts fully hidden.

    This is a fixed, direction-independent placement assumption, NOT measured
    per-camera part visibility. Select field='planned_visible_parts' explicitly
    for scenes storing their intended exposure there. Missing/null is unknown
    and raises; an explicit empty list means no complete parts are exposed.

    Canonical six-part names are accepted directly. Scene 'arms'/'legs' expose
    both complete limbs. A segmented leg needs thigh, shin AND foot listed;
    'hips' and 'shoulders' alone do not expose an entire torso or arm.
    """
    if not isinstance(placement, Mapping):
        raise ValueError("placement must be a mapping")
    if field not in ("visible_parts", "planned_visible_parts"):
        raise ValueError("field must be visible_parts or planned_visible_parts")
    labels = placement.get(field)
    if not isinstance(labels, (list, tuple, set, frozenset)) or any(not isinstance(p, str) for p in labels):
        raise ValueError(f"placement {field} must be a list of part names; missing/null is unknown")
    labels = set(labels)
    allowed = set(BODY_PARTS) | {"arms", "legs", "feet", "shins", "thighs", "hips", "shoulders"}
    allowed |= {f"{side}_{segment}" for side in ("left", "right") for segment in ("foot", "shin", "thigh")}
    if labels - allowed:
        raise ValueError(f"unknown placement parts: {', '.join(sorted(labels - allowed))}")
    parts = {p: float(p in labels) for p in BODY_PARTS}
    for side in ("left", "right"):
        parts[f"{side}_arm"] = float(f"{side}_arm" in labels or "arms" in labels)
        complete_leg = all(plural in labels or f"{side}_{segment}" in labels
                           for plural, segment in (("thighs", "thigh"), ("shins", "shin"), ("feet", "foot")))
        parts[f"{side}_leg"] = float(f"{side}_leg" in labels or "legs" in labels or complete_leg)
    return parts


def from_placement_evidence(evidence, placement, *, field="visible_parts",
                            contrast=None, clutter=None, image_width=640, image_height=400):
    """Combine measured camera pixels/body fraction with a fixed placement mask.

    Only part availability comes from placement. Pose, range, covered_frac and
    placement visibility labels never substitute for measured body fraction.
    An empty camera view has zero visible parts regardless of placement.
    """
    parts = parts_from_placement(placement, field=field)
    if evidence.get("pixels") == 0:
        parts = {p: 0.0 for p in BODY_PARTS}
    return from_demo_evidence(evidence, visible_parts=parts, contrast=contrast,
                              clutter=clutter, image_width=image_width, image_height=image_height)


def from_counts(*, visible_pixels, unoccluded_pixels, visible_parts=None,
                contrast=None, clutter=None, image_width=640, image_height=400):
    """Use matching in-frame silhouettes rendered with/without external occluders.

    The denominator is the person's body silhouette, not its bounding-box area.
    Both masks must use the same camera, pose, resolution and rasterization.
    Body-part fractions, contrast and clutter are supplied separately if known.
    """
    _integer("visible_pixels", visible_pixels, 0)
    _integer("unoccluded_pixels", unoccluded_pixels, 0)
    _integer("image_width", image_width, 1)
    _integer("image_height", image_height, 1)
    if visible_pixels > unoccluded_pixels:
        raise ValueError("visible_pixels exceeds unoccluded_pixels; check mask alignment/rasterization")
    if unoccluded_pixels > image_width * image_height:
        raise ValueError("unoccluded_pixels exceeds image area")
    return Observation(visible_pixels=visible_pixels,
                       visible_body_fraction=visible_pixels / unoccluded_pixels if unoccluded_pixels else 0.0,
                       visible_parts=visible_parts, contrast=contrast, clutter=clutter,
                       image_width=image_width, image_height=image_height)


def from_demo_evidence(evidence, *, visible_parts=None, contrast=None, clutter=None,
                       image_width=640, image_height=400):
    """Convert an existing person_visibility.py candidate or JSONL people entry.

    Uses measured pixels/visibleBodyFraction only. Ignores range, pose, box and
    knownCoverFraction. Missing parts/appearance remain explicitly unknown.
    Feed unfiltered candidates when testing a new detector, not old accept/reject
    results. Archived rows may already have a minimum-pixel filter applied.
    """
    if "pixels" not in evidence or evidence.get("visibleBodyFraction") is None:
        raise ValueError("demo evidence needs pixels and measured visibleBodyFraction")
    return Observation(visible_pixels=evidence["pixels"],
                       visible_body_fraction=evidence["visibleBodyFraction"],
                       visible_parts=visible_parts, contrast=contrast, clutter=clutter,
                       image_width=image_width, image_height=image_height)
