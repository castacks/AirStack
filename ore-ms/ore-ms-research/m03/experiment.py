"""Run the frozen LWIR target/background experiment matrix."""
import argparse
import csv
import json
import math
from pathlib import Path
import random
import statistics

from thermal_model import (Detector, Slab, atmosphere, planck_um, profile,
                           surface_components, weighted_band)

SPECTRAL_FIELDS = ("emission", "reflection", "transmission", "direct_emission",
                   "direct_reflection", "path", "observed")


def cases(config):
    for name, samples in config["sweeps"].items():
        for value in samples:
            case = {"name": f"{name}_{value}", "sweep": name}
            if name == "crossover_temperature_K":
                case.update(target_temperature_K=value, target_emissivity=[.5] * 3,
                            reflection_temperature_K=280, detector={"sigma": .12})
            elif name == "emissivity":
                case["target_emissivity"] = [value] * 3
            elif name == "path_temperature_K":
                case.update(path_temperature_K=value, gray_transmission=.5)
            else:
                case[name] = value
            yield case
    yield from config["controls"]


def surface(config, case, role):
    if role == "target_ray" and "foreground" in case:
        return "foreground", case["foreground"]
    name = "target" if role == "target_ray" else "background"
    properties = config[f"default_{name}"]
    return name, {key: case.get(f"{name}_{key}", default) for key, default in properties.items()}


def spectrum(config, case, properties, wavelength):
    emissivity = profile(wavelength, properties["emissivity"])
    if "reflected_radiance" in case:
        environment = profile(wavelength, case["reflected_radiance"])
    else:
        environment = planck_um(wavelength, case.get(
            "reflection_temperature_K", config["reflection"]["temperature_K"]))
    emission, reflection = surface_components(
        planck_um(wavelength, properties["temperature_K"]), emissivity, environment)
    layers = [config["layers"][i] for i in case.get("layer_order", [])]
    slabs = [Slab.absorbing(wavelength, layer["length_m"],
                           profile(wavelength, layer["absorption_m_inv"]), layer["temperature_K"])
             for layer in layers]
    if "gray_transmission" in case:
        if layers:
            raise ValueError("gray fixture and layered atmosphere are mutually exclusive")
        slabs = [Slab.gray_fixture(wavelength, case["gray_transmission"],
                                   case.get("path_temperature_K", config["gray_path_temperature_K"]))]
    transmission, path = atmosphere(slabs)
    direct_emission, direct_reflection = transmission * emission, transmission * reflection
    return (emission, reflection, transmission, direct_emission, direct_reflection, path,
            direct_emission + direct_reflection + path)


def render(config, case, role, intervals=None):
    name, properties = surface(config, case, role)
    start, end = config["wavelength_limits_um"]
    count = config["intervals"] if intervals is None else intervals
    step = (end - start) / count
    wavelengths = [start + i * step for i in range(count + 1)]
    weights = [profile(w, case.get("response", config["response"])) for w in wavelengths]
    if any(w > 1 for w in weights):
        raise ValueError("response exceeds unit peak")
    rows = [spectrum(config, case, properties, wavelength) for wavelength in wavelengths]
    integrals = [weighted_band([row[i] for row in rows], weights, step) for i in (3, 4, 5, 6)]
    return name, properties["temperature_K"], wavelengths, weights, rows, integrals


def run_case(config, case, writers):
    spectral, digital, summary = writers
    settings = dict(config["detector"], **case.get("detector", {}))
    detector = Detector(*(settings[key] for key in ("low", "high", "bits", "sigma")))
    rng = random.Random(settings["seed"])
    bands = []
    metrics = []
    for role in ("target_ray", "background_ray"):
        name, temperature, wavelengths, weights, rows, components = render(config, case, role)
        band = components[-1]
        bands.append(band)
        for wavelength, response, row in zip(wavelengths, weights, rows, strict=True):
            spectral.writerow((case["name"], role, name, wavelength, response, *row))
        _, ideal, ideal_low, ideal_high = detector.measure(band)
        retrieved = []
        clipped = 0
        for i in range(settings["repeats"]):
            analog, code, low, high = detector.measure(band, rng)
            decoded = detector.decode(code)
            retrieved.append(decoded)
            clipped += low or high
            digital.writerow((case["name"], role, i, band, analog, ideal, code, decoded,
                              int(low), int(high)))
        metrics.extend((name, temperature, *components, ideal, int(ideal_low), int(ideal_high),
                        statistics.fmean(retrieved), statistics.stdev(retrieved),
                        math.sqrt(statistics.fmean((value - band)**2 for value in retrieved)),
                        clipped / settings["repeats"]))
    first, second = bands
    delta = first - second
    scale = first + second
    visible = "foreground" not in case
    tolerance = max(detector.lsb, 2 * math.sqrt(2) * detector.sigma)
    summary.writerow((case["name"], case.get("sweep", "control"), int(visible),
                      delta if visible else "", delta / scale if visible and scale else "",
                      delta if not visible else "", tolerance,
                      int(abs(delta) <= tolerance) if visible else "", *metrics))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    args.output.mkdir()
    with ((args.output / "spectra.csv").open("w", newline="") as spectra,
          (args.output / "detector.csv").open("w", newline="") as digital,
          (args.output / "metrics.csv").open("w", newline="") as metrics):
        writers = tuple(csv.writer(out, lineterminator="\n") for out in (spectra, digital, metrics))
        writers[0].writerow(("case", "ray", "surface", "wavelength_um", "response",
                             *(f"{field}_W_m2_sr_um" if field != "transmission" else field
                               for field in SPECTRAL_FIELDS)))
        writers[1].writerow(("case", "ray", "sample", "band", "analog", "ideal_code",
                             "code", "decoded", "low_clip", "high_clip"))
        fields = ("surface", "temperature_K", "direct_emission", "direct_reflection", "path",
                  "band", "ideal_code", "ideal_low_clip", "ideal_high_clip", "decoded_mean",
                  "decoded_sd", "decoded_rmse", "clip_fraction")
        writers[2].writerow(("case", "sweep", "target_visible", "target_delta",
                             "normalized_target_delta", "foreground_delta", "detector_threshold",
                             "within_threshold",
                             *(f"{role}_{field}" for role in ("first", "background")
                               for field in fields)))
        for case in cases(config):
            run_case(config, case, writers)


if __name__ == "__main__":
    main()
