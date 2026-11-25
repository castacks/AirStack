"""Connect unchanged milestone kernels to a shared, segmented optical path."""
from dataclasses import dataclass
from functools import lru_cache
import math
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parent.parent
KERNELS = ("m02/rgb_model.py", "m03/thermal_model.py", "m04/geometry.py", "m05/environment.py")
sys.path.extend(str(ROOT / Path(kernel).parent) for kernel in KERNELS)

from environment import Field, advance, evaluate, physical_sample, stationary, transition
from rgb_model import (Layer, Sensor, henyey_greenstein, integrate_response, lambertian,
                       spectral_coefficient, transfer, triangle_response)
from thermal_model import Detector, Slab, atmosphere, planck_um, profile, surface_components, weighted_band


@dataclass(frozen=True)
class Segment:
    lo_m: float
    hi_m: float
    z_m: float
    burden: float
    air_K: float
    rgb_a: float
    rgb_s: float
    rgb_power: float
    lwir_a: tuple

    @property
    def length(self):
        return self.hi_m - self.lo_m

    def values(self):
        return (self.lo_m, self.hi_m, self.z_m, self.burden, self.air_K, self.rgb_a,
                self.rgb_s, self.rgb_power, *self.lwir_a)


def snapshots(config, profile_id, seed, times):
    field = config["field"]
    model = Field(field["period_m"], tuple(field["weights"]))
    if profile_id == "advected":
        for time in times:
            yield time, model.mean_coefficients(-.7, 2), (0,) * model.size
        return
    if profile_id != "stochastic":
        for time in times:
            yield time, (), ()
        return
    rng = random.Random(seed)
    thermal_rng = random.Random(seed + 1_000_000)
    burden = stationary(rng, field["burden_sigma"], model.size)
    heat = stationary(thermal_rng, field["thermal"]["sigma"], model.size)
    zero = (0,) * model.size
    last = 0
    for time in times:
        if time > last:
            burden = advance(burden, zero, transition(
                time - last, field["burden_tau"], field["burden_sigma"]), rng)
            heat = advance(heat, zero, transition(
                time - last, field["thermal"]["tau_s"], field["thermal"]["sigma"]), thermal_rng)
        yield time, burden, heat
        last = time


def segments(config, profile_id, time, burden, heat, origin_z, dz, length, count=None):
    if not math.isfinite(length) or length <= 0:
        raise ValueError("expected a finite positive ray range in metres")
    if profile_id not in ("stochastic", "advected"):
        p = config["homogeneous"]
        if profile_id not in ("vacuum", "rgb", "lwir", "dual"):
            raise ValueError("unknown medium profile")
        rgb_a, rgb_s = ((p["rgb_absorption"], p["rgb_scattering"])
                        if profile_id in ("rgb", "dual") else (0, 0))
        thermal = p["lwir_absorption"] if profile_id in ("lwir", "dual") else 0
        return (Segment(0, length, origin_z + dz * length / 2, 0, p["air_K"],
                        rgb_a, rgb_s, 0, (thermal,) * 3),)
    count = config["segments"] if count is None else count
    field = config["field"]
    model = Field(field["period_m"], tuple(field["weights"]))
    result = []
    for i in reversed(range(count)):
        lo, hi = length * i / count, length * (i + 1) / count
        z = origin_z + dz * (lo + hi) / 2
        basis = model.basis(z, time, field["velocity_m_s"])
        p = physical_sample(evaluate(burden, basis), evaluate(heat, basis),
                            field["velocity_m_s"], field["optical"], field["thermal"])
        result.append(Segment(lo, hi, z, p.burden, p.temperature_K,
                              p.rgb_absorption_550_m_inv, p.rgb_scattering_550_m_inv,
                              p.rgb_exponent, p.lwir_absorption_m_inv))
    return tuple(result)


def optical_depth(path, modality, wavelength):
    if modality == "RGB":
        return math.fsum(s.length * spectral_coefficient(
            s.rgb_a + s.rgb_s, wavelength, 550, s.rgb_power) for s in path)
    return math.fsum(s.length * profile(wavelength, s.lwir_a) for s in path)


def visible_interp(nm, values):
    first, middle, last = values
    if nm <= 550:
        return first + (middle - first) * (nm - 400) / 150
    return middle + (last - middle) * (nm - 550) / 150


blackbody = lru_cache(maxsize=100_000)(planck_um)


def radiance(config, material, path):
    rgb_wave = tuple(range(400, 701, 5))
    phase = henyey_greenstein(1, 0)
    direct, path_rgb = [], []
    for w in rgb_wave:
        layers = [Layer(s.length, spectral_coefficient(s.rgb_a, w, 550, s.rgb_power),
                        spectral_coefficient(s.rgb_s, w, 550, s.rgb_power),
                        config["rgb_ambient"], config["rgb_solid_angle"]) for s in path]
        boundary = lambertian(visible_interp(w, material["irradiance"]),
                              visible_interp(w, material["rho"]))
        result = transfer(boundary, layers, phase)
        direct.append(result.direct)
        path_rgb.append(result.path)
    rgb = []
    for channel in "RGB":
        weights = [triangle_response(w, *config["rgb_response"][channel]) for w in rgb_wave]
        rgb.append((integrate_response(rgb_wave, direct, weights),
                    integrate_response(rgb_wave, path_rgb, weights)))
    lwir = [[], [], []]
    count = config["lwir_intervals"]
    step = 6 / count
    weights = []
    for i in range(count + 1):
        w = 8 + i * step
        weights.append(profile(w, config["lwir_response"]))
        emission, reflection = surface_components(blackbody(w, material["temperature_K"]),
                                                  profile(w, material["emissivity"]),
                                                  blackbody(w, material["reflected_K"]))
        layers = [Slab(s.length * profile(w, s.lwir_a), blackbody(w, s.air_K)) for s in path]
        t, path_lwir = atmosphere(layers)
        lwir[0].append(t * emission)
        lwir[1].append(t * reflection)
        lwir[2].append(path_lwir)
    return tuple(rgb), tuple(weighted_band(term, weights, step) for term in lwir)


class AnalogNoise(random.Random):
    last = 0.0

    def gauss(self, mu=0, sigma=1):
        self.last = super().gauss(mu, sigma)
        return self.last


def detect(rgb, lwir, settings, rgb_noise, lwir_noise):
    visible = Sensor(settings["rgb_gain"], settings["rgb_well"], settings["bits"],
                     settings["rgb_read_noise_e"])
    infrared = Detector(0, settings["lwir_high"], settings["bits"], settings["lwir_sigma"])
    output = []
    for direct, path in rgb:
        rgb_noise.last = 0
        code, high = visible.capture(direct + path, rgb_noise)
        analog = direct + path + rgb_noise.last / visible.gain_e_per_radiance
        output.extend((direct, path, analog, code, int(analog < 0), int(high)))
    analog, code, low, high = infrared.measure(sum(lwir), lwir_noise)
    return (*output, *lwir, analog, code, int(low), int(high))
