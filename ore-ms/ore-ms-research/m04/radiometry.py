"""Thin compatibility adapters for the accepted M02 and M03 forward kernels."""
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parent.parent
sys.path.extend(str(PROJECT / milestone) for milestone in ("m02", "m03"))

from rgb_model import (Layer, Sensor, henyey_greenstein, integrate_response, lambertian,
                       transfer, triangle_response)
from thermal_model import (Detector, Slab, atmosphere, planck_um, profile, surface_components,
                           weighted_band)


def spectral_triplet(wavelength, values):
    """Interpolate M04 visible properties declared at 400, 550 and 700 nm."""
    low, middle, high = values
    if wavelength <= 550:
        return low + (middle - low) * (wavelength - 400) / 150
    return middle + (high - middle) * (wavelength - 550) / 150


def rgb(config, material, distance, medium, settings):
    wave = tuple(range(400, 701, 5))
    sensor = Sensor(settings["rgb_gain"], settings["rgb_full_well"], settings["bits"])
    phase = henyey_greenstein(1, 0)
    direct, path = [], []
    for wavelength in wave:
        reflection = spectral_triplet(wavelength, material["rgb_reflectance"])
        irradiance = spectral_triplet(wavelength, material["rgb_irradiance"])
        boundary = lambertian(irradiance, reflection)
        layers = [Layer(distance, medium["rgb_absorption_m_inv"], medium["rgb_scattering_m_inv"],
                        medium["rgb_ambient"], medium["rgb_solid_angle_sr"])] if medium else []
        result = transfer(boundary, layers, phase)
        direct.append(result.direct)
        path.append(result.path)
    output = []
    for channel in "RGB":
        response = [triangle_response(w, *config["rgb_response"][channel]) for w in wave]
        first = integrate_response(wave, direct, response)
        air = integrate_response(wave, path, response)
        code, clip = sensor.capture(first + air)
        output.extend((first, air, code, int(clip)))
    return output


def lwir(config, material, distance, medium, settings):
    step = 6 / 192
    waves = [8 + i * step for i in range(193)]
    response = [profile(w, config["lwir_response"]) for w in waves]
    emission, reflection, path = [], [], []
    for w in waves:
        e, r = surface_components(planck_um(w, material["temperature_K"]),
                                  profile(w, material["emissivity"]),
                                  planck_um(w, material["reflected_K"]))
        layers = [Slab.absorbing(w, distance, medium["lwir_absorption_m_inv"],
                                 medium["lwir_air_K"])] if medium else []
        t, air = atmosphere(layers)
        emission.append(t * e)
        reflection.append(t * r)
        path.append(air)
    e, r, air = (weighted_band(series, response, step) for series in (emission, reflection, path))
    detector = Detector(0, settings["lwir_high"], settings["bits"], 0)
    _, code, _, clip = detector.measure(e + r + air)
    return e, r, air, code, int(clip)
