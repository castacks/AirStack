"""Radiance in SI units. Wavelength arguments are in vacuum metres."""
import math
import random


def transport(boundary, source, extinction, distance):
    if not all(math.isfinite(x) and x >= 0 for x in
               (boundary, source, extinction, distance)):
        raise ValueError("path parameters must be finite and nonnegative")
    depth = extinction * distance
    return math.exp(-depth) * boundary - math.expm1(-depth) * source


def planck(wavelength, kelvin):
    if not (1e-6 <= wavelength <= 100e-6 and 100 <= kelvin <= 2000):
        raise ValueError("outside supported wavelength or temperature range")
    h, c, k = 6.62607015e-34, 299792458, 1.380649e-23
    return (2 * h * c * c / wavelength**5
            / math.expm1(h * c / (wavelength * k * kelvin)))


def thermal_band(temperature, emissivity, reflected, air, extinction,
                 distance, response_edges, intervals):
    if not 0 <= emissivity <= 1 or intervals < 1:
        raise ValueError("invalid emissivity or integration interval count")
    lo, hi = response_edges
    if not all(0 <= value <= 1 for value in response_edges):
        raise ValueError("response must be between zero and one")
    step = 6e-6 / intervals
    samples = []
    for i in range(intervals + 1):
        fraction = i / intervals
        wavelength = 8e-6 + step * i
        boundary = (emissivity * planck(wavelength, temperature)
                    + (1 - emissivity) * planck(wavelength, reflected))
        radiance = transport(boundary, planck(wavelength, air), extinction, distance)
        weight = 0.5 if i in (0, intervals) else 1
        samples.append(weight * (lo + (hi - lo) * fraction) * radiance)
    return step * math.fsum(samples)


def noise_stream(seed, sigma):
    if not math.isfinite(sigma) or sigma < 0:
        raise ValueError("noise sigma must be finite and nonnegative")
    rng = random.Random(seed)
    while True:
        yield rng.gauss(0, sigma)
