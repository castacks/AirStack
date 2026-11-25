"""Visible radiometry for a static ray, with spectra expressed per nanometre."""
from dataclasses import dataclass
import math


def nonnegative(*values):
    if not all(math.isfinite(value) and value >= 0 for value in values):
        raise ValueError("expected finite, nonnegative physical inputs")


@dataclass(frozen=True)
class Layer:
    length_m: float
    absorption_m_inv: float
    scattering_m_inv: float
    incident_radiance: float
    incident_solid_angle_sr: float

    def __post_init__(self):
        nonnegative(self.length_m, self.absorption_m_inv, self.scattering_m_inv,
                    self.incident_radiance, self.incident_solid_angle_sr)
        if self.incident_solid_angle_sr > 4 * math.pi:
            raise ValueError("incident solid angle exceeds the sphere")


@dataclass(frozen=True)
class Transfer:
    transmission: float
    direct: float
    path: float

    @property
    def total(self):
        return self.direct + self.path


def henyey_greenstein(cos_angle, g):
    if not (-1 <= cos_angle <= 1 and -1 < g < 1):
        raise ValueError("invalid scattering angle or asymmetry")
    # Algebraic forms that avoid subtracting nearly equal terms near ±1.
    difference = 1 - abs(g)
    angular = 1 - cos_angle if g >= 0 else 1 + cos_angle
    denominator = difference**2 + 2 * abs(g) * angular
    return difference * (1 + abs(g)) / (4 * math.pi * denominator**1.5)


def spectral_coefficient(reference_m_inv, wavelength_nm, reference_nm, power):
    nonnegative(reference_m_inv)
    if not (math.isfinite(wavelength_nm) and wavelength_nm > 0
            and math.isfinite(reference_nm) and reference_nm > 0
            and math.isfinite(power)):
        raise ValueError("invalid wavelength or exponent")
    return reference_m_inv * (reference_nm / wavelength_nm)**power


def lambertian(irradiance, reflectance):
    nonnegative(irradiance)
    if not 0 <= reflectance <= 1:
        raise ValueError("reflectance must be in [0, 1]")
    return irradiance * reflectance / math.pi


def transfer(boundary, layers, phase_sr_inv):
    """Compose slabs from surface to camera at one wavelength."""
    nonnegative(boundary, phase_sr_inv)
    depths = []
    path = 0.0
    for layer in layers:
        extinction = layer.absorption_m_inv + layer.scattering_m_inv
        nonnegative(extinction)
        depth = extinction * layer.length_m
        depths.append(depth)
        if not depth:
            continue
        scattered_fraction = layer.scattering_m_inv / extinction
        source = scattered_fraction * layer.incident_radiance
        source *= layer.incident_solid_angle_sr * phase_sr_inv
        path = math.exp(-depth) * path - math.expm1(-depth) * source
    transmission = math.exp(-math.fsum(depths))
    return Transfer(transmission, boundary * transmission, path)


def triangle_response(wavelength_nm, low_nm, peak_nm, high_nm):
    if not 0 < low_nm < peak_nm < high_nm:
        raise ValueError("response knots must be ordered and positive")
    return max(0.0, min((wavelength_nm - low_nm) / (peak_nm - low_nm),
                        (high_nm - wavelength_nm) / (high_nm - peak_nm)))


def integrate_response(wavelengths_nm, radiance, response):
    """Integrate the product of piecewise-linear radiance and sensitivity."""
    samples = tuple(zip(wavelengths_nm, radiance, response, strict=True))
    if len(samples) < 2:
        raise ValueError("at least two wavelength samples are required")
    for wavelength, value, weight in samples:
        nonnegative(wavelength, value)
        if wavelength == 0 or not 0 <= weight <= 1:
            raise ValueError("invalid sampled wavelength or relative response")
    intervals = []
    for (lo, left, weight_left), (hi, right, weight_right) in zip(samples, samples[1:]):
        if hi <= lo:
            raise ValueError("spectral grid must increase")
        intervals.append((hi - lo) / 6 * (
            left * (2 * weight_left + weight_right)
            + right * (weight_left + 2 * weight_right)))
    return math.fsum(intervals)


@dataclass(frozen=True)
class Sensor:
    gain_e_per_radiance: float
    full_well_e: float
    bits: int
    read_noise_e: float = 0.0
    shot_noise: bool = False

    def __post_init__(self):
        nonnegative(self.gain_e_per_radiance, self.full_well_e, self.read_noise_e)
        if (self.gain_e_per_radiance == 0 or self.full_well_e == 0
                or not isinstance(self.bits, int) or not 1 <= self.bits <= 24):
            raise ValueError("invalid gain, full well or bit depth")

    @property
    def max_code(self):
        return (1 << self.bits) - 1

    def capture(self, radiance, rng=None):
        nonnegative(radiance)
        electrons = radiance * self.gain_e_per_radiance
        nonnegative(electrons)
        if rng is not None:
            variance = self.read_noise_e**2 + (electrons if self.shot_noise else 0)
            if variance:
                electrons += rng.gauss(0, math.sqrt(variance))
        bounded = min(self.full_well_e, max(0.0, electrons))
        return round(bounded / self.full_well_e * self.max_code), electrons >= self.full_well_e
