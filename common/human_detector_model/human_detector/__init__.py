"""A simulator-independent, stateless detection surrogate. No trained model."""

from .model import BODY_PARTS, Detector, DetectorConfig, Observation
from .adapters import from_counts, from_demo_evidence, from_placement_evidence, parts_from_placement

__all__ = ["BODY_PARTS", "Detector", "DetectorConfig", "Observation",
           "from_counts", "from_demo_evidence", "from_placement_evidence", "parts_from_placement"]
__version__ = "0.2.0"
