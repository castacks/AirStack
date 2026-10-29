"""Geometry-conditioned detection probabilities; all defaults are assumptions."""

from dataclasses import dataclass, field, fields
import hashlib
from importlib.resources import files
import json
import math
from pathlib import Path
from types import MappingProxyType
from typing import Mapping


BODY_PARTS = ("head", "torso", "left_arm", "right_arm", "left_leg", "right_leg")


def _number(name, value, low=0.0, high=None, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    if not math.isfinite(value) or value < low or (high is not None and value > high):
        raise ValueError(f"{name} must be finite and in [{low}, {high or 'infinity'}]")
    if positive and value == 0:
        raise ValueError(f"{name} must be greater than zero")


def _integer(name, value, minimum):
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")


def _parts(name, value):
    if not isinstance(value, Mapping) or set(value) != set(BODY_PARTS):
        raise ValueError(f"{name} must contain exactly: {', '.join(BODY_PARTS)}")
    for part, amount in value.items():
        _number(f"{name}.{part}", amount, high=1.0)
    return MappingProxyType(dict(value))


@dataclass(frozen=True)
class Observation:
    """One person's measurements from one image, at detector input resolution.

    visible_parts contains the visible fraction of EACH part's unoccluded,
    in-frame projected silhouette (not each part's share of visible pixels).
    None means unavailable, never fully hidden. The placement adapter can supply
    fixed binary exposure assumptions instead. Zero for every part is valid:
    visible fragments need not expose any complete part. No pose/temporal inputs.
    """

    visible_pixels: int
    visible_body_fraction: float
    visible_parts: Mapping[str, float] | None = None
    contrast: float | None = None
    clutter: float | None = None
    image_width: int = 640
    image_height: int = 400

    def __post_init__(self):
        _integer("visible_pixels", self.visible_pixels, 0)
        _integer("image_width", self.image_width, 1)
        _integer("image_height", self.image_height, 1)
        if self.visible_pixels > self.image_width * self.image_height:
            raise ValueError("visible_pixels exceeds the image area")
        _number("visible_body_fraction", self.visible_body_fraction, high=1.0)
        if (self.visible_pixels == 0) != (self.visible_body_fraction == 0):
            raise ValueError("zero visible pixels and zero visible body fraction must agree")
        if self.visible_parts is not None:
            values = _parts("visible_parts", self.visible_parts)
            if self.visible_pixels == 0 and any(values.values()):
                raise ValueError("a hidden person cannot have visible parts")
            object.__setattr__(self, "visible_parts", values)
        for name in ("contrast", "clutter"):
            value = getattr(self, name)
            if value is not None:
                _number(name, value, high=1.0)

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, Mapping):
            raise ValueError("observation must be a JSON object")
        try:
            return cls(**data)
        except TypeError as exc:
            raise ValueError(f"invalid observation fields: {exc}") from exc


@dataclass(frozen=True)
class DetectorConfig:
    """Parameters of an additive log-odds model. Set penalty weights to 0 to ablate."""

    pixels_50: float = 80.0
    pixel_slope: float = 1.5
    visibility_weight: float = 1.0
    parts_weight: float = 0.75
    contrast_weight: float = 1.0
    clutter_weight: float = 1.0
    part_weights: Mapping[str, float] = field(default_factory=lambda: {
        "head": 0.30, "torso": 0.40, "left_arm": 0.10,
        "right_arm": 0.10, "left_leg": 0.05, "right_leg": 0.05,
    })
    image_width: int = 640
    image_height: int = 400

    def __post_init__(self):
        _number("pixels_50", self.pixels_50, positive=True)
        _number("pixel_slope", self.pixel_slope, positive=True, high=1000.0)
        for name in ("visibility_weight", "parts_weight", "contrast_weight", "clutter_weight"):
            _number(name, getattr(self, name), high=1000.0)
        values = _parts("part_weights", self.part_weights)
        if sum(values.values()) <= 0:
            raise ValueError("part_weights must have a positive sum")
        object.__setattr__(self, "part_weights", values)
        _integer("image_width", self.image_width, 1)
        _integer("image_height", self.image_height, 1)

    def to_dict(self):
        return {f.name: dict(self.part_weights) if f.name == "part_weights"
                else getattr(self, f.name) for f in fields(self)}

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, Mapping):
            raise ValueError("config must be a JSON object")
        try:
            return cls(**data)
        except TypeError as exc:
            raise ValueError(f"invalid config fields: {exc}") from exc

    @classmethod
    def load(cls, path):
        return cls.from_dict(json.loads(Path(path).read_text()))

    @classmethod
    def profile(cls, name="baseline"):
        if name not in ("baseline", "stronger", "weaker", "small_target", "occlusion_tolerant"):
            raise ValueError(f"unknown profile: {name}")
        resource = files("human_detector").joinpath("profiles", name + ".json")
        return cls.from_dict(json.loads(resource.read_text()))


class Detector:
    def __init__(self, config: DetectorConfig | None = None):
        self.config = config if config is not None else DetectorConfig()

    def explain(self, observation: Observation) -> dict:
        """Return probability and log-odds contributions for experiment diagnostics."""
        c, o = self.config, observation
        if (o.image_width, o.image_height) != (c.image_width, c.image_height):
            raise ValueError("image resolution differs from config; measure at the configured detector input resolution")
        missing = [name for name in ("visible_parts", "contrast", "clutter")
                   if getattr(o, name) is None]
        if o.visible_pixels == 0:
            return {"probability": 0.0, "gated": "no_visible_pixels",
                    "contributions": {}, "missing_features": missing}
        part_score = None
        if o.visible_parts is not None:
            part_score = sum(c.part_weights[p] * o.visible_parts[p] for p in BODY_PARTS) / sum(c.part_weights.values())
        terms = {
            "pixels": c.pixel_slope * (math.log(o.visible_pixels) - math.log(c.pixels_50)),
            "body_fraction": c.visibility_weight * math.log(o.visible_body_fraction),
            "body_parts": 0.0 if part_score is None else c.parts_weight * math.log(max(part_score, 1e-12)),
            "contrast": 0.0 if o.contrast is None else -c.contrast_weight * (1.0 - o.contrast),
            "clutter": 0.0 if o.clutter is None else -c.clutter_weight * o.clutter,
        }
        z = sum(terms.values())
        # Stable sigmoid, including very difficult observations.
        probability = 1.0 / (1.0 + math.exp(-z)) if z >= 0 else math.exp(z) / (1.0 + math.exp(z))
        return {"probability": probability, "gated": None, "contributions": terms,
                "part_score": part_score, "missing_features": missing}

    def probability(self, observation: Observation) -> float:
        return self.explain(observation)["probability"]

    def detect(self, observation: Observation, *, seed: int, key: str) -> dict:
        """Stateless Bernoulli outcome keyed by an experiment observation ID.

        Same seed/key => same draw, independent of call order or detector profile.
        No observation history, confirmation, tracking, or hidden RNG state.
        """
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise ValueError("seed must be an integer")
        if not isinstance(key, str) or not key:
            raise ValueError("key must be a nonempty observation identifier")
        result = self.explain(observation)
        payload = json.dumps([seed, key], ensure_ascii=True, separators=(",", ":")).encode()
        bits = int.from_bytes(hashlib.sha256(payload).digest()[:8], "big") >> 11
        draw = bits / 2**53
        return {**result, "detected": draw < result["probability"], "draw": draw}
