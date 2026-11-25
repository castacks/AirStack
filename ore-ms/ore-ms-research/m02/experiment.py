"""Render the frozen static scene and retain every channel-level sample."""
import argparse
import csv
import json
import math
from pathlib import Path
import random
import statistics

from metrics import local_ssim, michelson, rmse, snr_db
from rgb_model import (Layer, Sensor, henyey_greenstein, integrate_response,
                       lambertian, spectral_coefficient, transfer, triangle_response)


def lerp(endpoints, fraction):
    return endpoints[0] * (1 - fraction) + endpoints[1] * fraction


def model_case(config, case):
    start, stop, step = (config[f"wavelength_{key}_nm"] for key in ("min", "max", "step"))
    grid = tuple(range(start, stop + 1, step))
    response = config["response_nm"]
    if not all(knot in grid for knots in response.values() for knot in knots):
        raise ValueError("each response knot must be sampled")
    distance = case.get("distance_m", 10)
    profile = case.get("layers", [[1, 0, 0, 0]])
    if not math.isclose(math.fsum(layer[0] for layer in profile), 1):
        raise ValueError("layer fractions must span the configured range")
    phase = henyey_greenstein(config["scatter_cos_angle"], case.get("phase_g", 0))
    sensor = Sensor(config["gain_e_per_W_m2_sr"], config["full_well_e"], config["bits"],
                    case.get("read_noise_e", 0), case.get("shot_noise", False))
    spectrum = {"dark": [], "light": []}
    risk = []
    for wavelength in grid:
        fraction = (wavelength - start) / (stop - start)
        irradiance = lerp(config["illuminance_spectrum_endpoints_W_m2_nm"], fraction)
        irradiance *= case.get("illumination_scale", 1)
        layers = tuple(
            Layer(distance * share,
                  spectral_coefficient(absorption, wavelength, config["extinction_reference_nm"],
                                       config["absorption_power"]),
                  spectral_coefficient(scattering, wavelength, config["extinction_reference_nm"],
                                       config["scattering_power"]),
                  ambient_scale * config["ambient_radiance_W_m2_sr_nm"],
                  config["incident_solid_angle_sr"])
            for share, absorption, scattering, ambient_scale in profile
        )
        scatter_depth = math.fsum(layer.length_m * layer.scattering_m_inv for layer in layers)
        risk.append(scatter_depth)
        path = transfer(0, layers, phase)
        for material, samples in spectrum.items():
            rho = lerp(config[f"reflectance_{material}_endpoints"], fraction)
            boundary = lambertian(irradiance, rho)
            samples.append((boundary, boundary * path.transmission, path.path))
    bands = {}
    for material, samples in spectrum.items():
        for channel, knots in response.items():
            weights = [triangle_response(wavelength, *knots) for wavelength in grid]
            bands[material, channel] = tuple(
                integrate_response(grid, component, weights) for component in zip(*samples))
    return sensor, bands, max(risk)


def capture(config, case, reference, writer):
    sensor, bands, scatter_depth = model_case(config, case)
    rng = random.Random(case.get("seed", config["seed"]))
    width, height, checker = (config[key] for key in ("width", "height", "checker_px"))
    observed, ideal, clear = [], [], []
    contrasts = {channel: {"light": [], "dark": []} for channel in "RGB"}
    clipped = 0
    for y in range(height):
        for x in range(width):
            material = "light" if (x // checker + y // checker) % 2 else "dark"
            for channel in "RGB":
                boundary, direct, path = bands[material, channel]
                clear_radiance = reference[material, channel][0]
                ideal_code, ideal_clip = sensor.capture(direct + path)
                code, clipped_high = sensor.capture(direct + path, rng)
                observed.append(code)
                ideal.append(ideal_code)
                clear.append(clear_radiance)
                contrasts[channel][material].append(code)
                clipped += clipped_high
                writer.writerow((case["name"], y, x, material, channel, boundary, direct, path,
                                 clear_radiance, ideal_code, code, int(ideal_clip), int(clipped_high)))
    def proxy(codes):
        return [sum(codes[i:i + 3]) / (3 * sensor.max_code) for i in range(0, len(codes), 3)]
    clear_codes = [sensor.capture(value)[0] for value in clear]
    inverse_gain = sensor.full_well_e / (sensor.max_code * sensor.gain_e_per_radiance)
    contrast = [michelson(statistics.fmean(contrasts[c]["light"]), statistics.fmean(contrasts[c]["dark"]))
                for c in "RGB"]
    interaction_proxy = -math.expm1(-scatter_depth) - scatter_depth * math.exp(-scatter_depth)
    return (case["name"], *contrast, snr_db(observed, ideal),
            local_ssim(proxy(observed), proxy(clear_codes), width, height),
            rmse([code * inverse_gain for code in observed], clear),
            clipped / len(observed), scatter_depth, interaction_proxy)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    args.output.mkdir()
    _, reference, _ = model_case(config, {"name": "clear"})
    with ((args.output / "pixels.csv").open("w", newline="") as captures,
          (args.output / "metrics.csv").open("w", newline="") as metrics):
        writer = csv.writer(captures, lineterminator="\n")
        writer.writerow(("case", "row", "column", "material", "channel", "boundary_W_m2_sr",
                         "direct_W_m2_sr", "path_W_m2_sr", "clear_W_m2_sr",
                         "ideal_code", "code", "ideal_high_clip", "high_clip"))
        summary = csv.writer(metrics, lineterminator="\n")
        summary.writerow(("case", "contrast_R", "contrast_G", "contrast_B", "sensor_SNR_dB",
                          "SSIM", "radiance_RMSE_W_m2_sr", "high_clip_fraction",
                          "max_scatter_depth", "max_two_or_more_interaction_proxy"))
        summary.writerows(capture(config, case, reference, writer) for case in config["cases"])


if __name__ == "__main__":
    main()
