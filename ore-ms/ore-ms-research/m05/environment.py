"""Synthetic transported environmental fields with exact Gaussian OU updates."""
from dataclasses import dataclass
import math


def transition(dt_s, tau_s, sigma):
    if (not all(map(math.isfinite, (dt_s, tau_s, sigma)))
            or dt_s <= 0 or tau_s <= 0 or sigma < 0):
        raise ValueError("invalid OU parameters")
    return math.exp(-dt_s / tau_s), sigma * math.sqrt(-math.expm1(-2 * dt_s / tau_s))


def advance(coefficients, means, update, rng=None):
    decay, scale = update
    if scale and rng is None:
        raise ValueError("a stochastic update requires an explicit RNG")
    return tuple(mu + (value - mu) * decay + (scale * rng.gauss(0, 1) if scale else 0)
                 for value, mu in zip(coefficients, means, strict=True))


def stationary(rng, sigma, size):
    if not math.isfinite(sigma) or sigma < 0:
        raise ValueError("invalid stationary standard deviation")
    return tuple(rng.gauss(0, sigma) for _ in range(size))


@dataclass(frozen=True)
class Field:
    period_m: float
    weights: tuple

    def __post_init__(self):
        if (not math.isfinite(self.period_m) or self.period_m <= 0 or len(self.weights) < 2
                or any(not math.isfinite(w) or w <= 0 for w in self.weights)
                or not math.isclose(math.fsum(self.weights), 1, rel_tol=0, abs_tol=1e-12)):
            raise ValueError("invalid periodic spectrum")

    @property
    def size(self):
        return 2 * len(self.weights) - 1

    def basis(self, position_m, time_s=0, velocity_m_s=0):
        angle = math.tau * ((position_m - velocity_m_s * time_s) % self.period_m) / self.period_m
        basis = [math.sqrt(self.weights[0])]
        for n, weight in enumerate(self.weights[1:], 1):
            scale = math.sqrt(weight)
            basis.extend((scale * math.cos(n * angle), scale * math.sin(n * angle)))
        return tuple(basis)

    def mean_coefficients(self, mean, cosine=0):
        return (mean / math.sqrt(self.weights[0]), cosine / math.sqrt(self.weights[1]),
                *(0 for _ in range(self.size - 2)))


def evaluate(coefficients, basis):
    return math.fsum(a * b for a, b in zip(coefficients, basis, strict=True))


def logistic(value):
    if not math.isfinite(value):
        raise ValueError("latent state must be finite")
    tail = math.exp(-abs(value))
    return 1 / (1 + tail) if value >= 0 else tail / (1 + tail)


@dataclass(frozen=True)
class MediumSample:
    burden: float
    temperature_K: float
    rgb_extinction_550_m_inv: float
    rgb_absorption_550_m_inv: float
    rgb_scattering_550_m_inv: float
    rgb_exponent: float
    scattering_fraction: float
    lwir_absorption_m_inv: tuple
    velocity_m_s: float


def physical_sample(latent, temperature_latent, velocity, optical, thermal):
    burden = logistic(latent)
    temperature = thermal["min_K"] + (thermal["max_K"] - thermal["min_K"]) * logistic(
        temperature_latent)
    extinction = optical["rgb_floor"] + optical["rgb_span"] * burden
    fraction = optical["scatter_fraction"]
    scattering = extinction * fraction
    absorption = tuple(floor + span * burden
                       for floor, span in zip(optical["lwir_floor"], optical["lwir_span"],
                                              strict=True))
    values = (velocity, optical["rgb_exponent"], extinction, scattering, temperature, *absorption)
    if (not all(map(math.isfinite, values)) or not 0 <= fraction <= 1
            or min(extinction, *absorption) < 0 or not 0 < thermal["min_K"] <= thermal["max_K"]
            or len(absorption) != 3):
        raise ValueError("invalid physical parameter mapping")
    return MediumSample(burden, temperature, extinction, extinction - scattering, scattering,
                        optical["rgb_exponent"], fraction, absorption, velocity)
