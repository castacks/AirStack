"""Monochromatic LTE transfer and an illustrative linear LWIR detector."""
from dataclasses import dataclass
import math

H = 6.62607015e-34
C = 299792458
K = 1.380649e-23


def nonnegative(*values):
    if not all(math.isfinite(value) and value >= 0 for value in values):
        raise ValueError("expected finite, nonnegative physical values")


def planck_um(wavelength_um, kelvin):
    """Blackbody spectral radiance in W m^-2 sr^-1 µm^-1."""
    if not (0.05 <= wavelength_um <= 10000 and 100 <= kelvin <= 2000):
        raise ValueError("outside supported wavelength or temperature domain")
    wavelength = wavelength_um * 1e-6
    exponent = H * C / (wavelength * K * kelvin)
    occupation = math.exp(-exponent) / -math.expm1(-exponent)
    return (2 * H * C**2 * 1e-6 / wavelength**5) * occupation


def profile(wavelength, values):
    """Interpolate a continuous piecewise-linear profile on 8, 11 and 14 µm."""
    low, mid, high = values
    nonnegative(low, mid, high)
    if not 8 <= wavelength <= 14:
        raise ValueError("profile outside 8–14 µm")
    if wavelength <= 11:
        return low + (mid - low) * (wavelength - 8) / 3
    return mid + (high - mid) * (wavelength - 11) / 3


def surface_components(blackbody, emissivity, reflected_environment):
    nonnegative(blackbody, reflected_environment)
    if not 0 <= emissivity <= 1:
        raise ValueError("emissivity must be in [0, 1]")
    return emissivity * blackbody, (1 - emissivity) * reflected_environment


@dataclass(frozen=True)
class Slab:
    optical_depth: float
    source: float

    @classmethod
    def absorbing(cls, wavelength, length_m, absorption_m_inv, temperature):
        nonnegative(length_m, absorption_m_inv)
        optical_depth = length_m * absorption_m_inv
        nonnegative(optical_depth)
        return cls(optical_depth, planck_um(wavelength, temperature))

    @classmethod
    def gray_fixture(cls, wavelength, transmission, temperature):
        if not 0 <= transmission <= 1:
            raise ValueError("transmission must be in [0, 1]")
        return cls(-math.log(transmission) if transmission else math.inf,
                   planck_um(wavelength, temperature))

    def __post_init__(self):
        if not self.optical_depth >= 0:
            raise ValueError("optical depth must be nonnegative")
        nonnegative(self.source)


def atmosphere(slabs):
    """Compose source terms from the surface to the camera."""
    depths, emitted_path = [], 0.0
    for slab in slabs:
        depth = slab.optical_depth
        depths.append(depth)
        emitted_path = math.exp(-depth) * emitted_path - math.expm1(-depth) * slab.source
    return math.exp(-math.fsum(depths)), emitted_path


def weighted_band(samples, weights, step_um):
    samples = [value * weight for value, weight in zip(samples, weights, strict=True)]
    if len(samples) < 2 or not math.isfinite(step_um) or step_um <= 0:
        raise ValueError("integration requires a positive interval and at least two samples")
    return step_um * math.fsum([samples[0] / 2, *samples[1:-1], samples[-1] / 2])


@dataclass(frozen=True)
class Detector:
    low: float
    high: float
    bits: int
    sigma: float

    def __post_init__(self):
        nonnegative(self.low, self.high, self.sigma)
        if self.high <= self.low or not isinstance(self.bits, int) or not 1 <= self.bits <= 24:
            raise ValueError("invalid detector range or bit depth")

    @property
    def max_code(self):
        return (1 << self.bits) - 1

    @property
    def lsb(self):
        return (self.high - self.low) / self.max_code

    def measure(self, band, rng=None):
        nonnegative(band)
        analog = band + rng.gauss(0, self.sigma) if rng is not None and self.sigma else band
        code = round((min(self.high, max(self.low, analog)) - self.low) / self.lsb)
        return analog, code, analog < self.low, analog > self.high

    def decode(self, code):
        return self.low + self.lsb * code
