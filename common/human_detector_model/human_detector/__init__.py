"""A simulator-independent, stateless detection surrogate. No trained model."""

from .model import BODY_PARTS, Detector, DetectorConfig, Observation
from .adapters import from_counts, from_demo_evidence

__all__ = ["BODY_PARTS", "Detector", "DetectorConfig", "Observation",
           "from_counts", "from_demo_evidence"]
__version__ = "0.1.0"
