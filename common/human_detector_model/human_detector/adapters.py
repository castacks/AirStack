"""Adapters for simulator pixel counts and the coordinator demo's evidence rows."""

from .model import Observation, _integer


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
