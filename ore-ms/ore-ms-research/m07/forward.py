"""Frozen conditional forward experiment using unchanged M06 and physical kernels."""
from functools import lru_cache
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.append(str(HERE.parent / "m06"))
from adapters import AnalogNoise, Segment, detect, radiance
from thermal_model import planck_um, profile, weighted_band

CONFIG = json.loads((HERE / "config.json").read_text())
CASES = {case["id"]: case for case in CONFIG["cases"] + CONFIG["m06_cases"]}


def setting(case, key):
    return case.get(key, CONFIG[key])


def context(case):
    material = case.get("material", "target")
    length, air = (setting(case, key) for key in ("length_m", "air_K"))
    return material, length, air


def settings(case):
    return {"rgb_gain": CONFIG["rgb_gain"], "rgb_well": setting(case, "rgb_well"),
            "rgb_read_noise_e": setting(case, "rgb_read_noise_e"), "bits": CONFIG["bits"],
            "lwir_high": setting(case, "lwir_high"), "lwir_sigma": setting(case, "lwir_noise")}


@lru_cache(maxsize=20000)
def bands(material, length, air, burden):
    optical = CONFIG["optical"]
    extinction = optical["rgb_floor"] + optical["rgb_span"] * burden
    scatter = extinction * optical["scatter_fraction"]
    lw = tuple(f + s * burden for f, s in zip(optical["lwir_floor"], optical["lwir_span"],
                                             strict=True))
    segment = Segment(0, length, length/2, burden, air, extinction-scatter, scatter,
                      optical["rgb_exponent"], lw)
    return radiance(CONFIG, CONFIG["materials"][material], (segment,))


def ideal(material, length, air, burden):
    rgb, lwir = bands(material, length, air, burden)
    return (*map(sum, rgb), sum(lwir))


def code_observation(case, actual, burden, rgb_rng: AnalogNoise, lwir_rng):
    rgb, lwir = bands(*actual, burden)
    output = detect(rgb, lwir, settings(case), rgb_rng, lwir_rng)
    return [output[i] for i in (3, 9, 15, 22)]


def black_band(temperature):
    grid = [8 + 6*i/192 for i in range(193)]
    return weighted_band([planck_um(w, temperature) for w in grid],
                         [profile(w, CONFIG["lwir_response"]) for w in grid], 6/192)
