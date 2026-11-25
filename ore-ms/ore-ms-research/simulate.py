"""Generate deterministic paired observations in row-major order."""
import argparse
import csv
import json
import math
from functools import lru_cache
from pathlib import Path

from physics import noise_stream, thermal_band, transport


def observations(config, case):
    width, height, focal = (config[k] for k in ("width", "height", "focal_px"))
    near, far = (config[k] for k in ("foreground_z_m", "target_z_m"))
    xmin, xmax, ymin, ymax = config["foreground_xy_m"]
    rgb_noise = noise_stream(config["rgb_seed"], config["rgb_noise_sigma"])
    lwir_noise = noise_stream(config["lwir_seed"], config["lwir_noise_sigma"])

    @lru_cache(maxsize=None)
    def radiances(surface, distance):
        material = config[surface]
        rgb = tuple(transport(value, source, extinction, distance)
                    for value, source, extinction in zip(
                        material["rgb"], config["rgb_path_source"],
                        case["rgb_extinction"], strict=True))
        thermal = thermal_band(material["temperature_K"], material["emissivity"],
                               config["reflected_K"], config["air_K"],
                               case["lwir_extinction"], distance,
                               config["response_edges"], config["intervals"])
        return rgb, thermal

    for row in range(height):
        for col in range(width):
            x = (col - (width - 1) / 2) / focal
            y = (row - (height - 1) / 2) / focal
            blocked = xmin <= near * x <= xmax and ymin <= near * y <= ymax
            surface = "foreground" if blocked else "target"
            distance = (near if blocked else far) * math.hypot(x, y, 1)
            rgb, thermal = radiances(surface, distance)
            yield (row, col, int(not blocked), surface, distance,
                   *rgb, thermal, *(value + next(rgb_noise) for value in rgb),
                   thermal + next(lwir_noise))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    with args.output.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(("case", "row", "column", "target_visible", "first_surface",
                         "range_m", "r_truth", "g_truth", "b_truth", "lwir_truth",
                         "r_observed", "g_observed", "b_observed", "lwir_observed"))
        for name, case in config["cases"].items():
            writer.writerows((name, *pixel) for pixel in observations(config, case))


if __name__ == "__main__":
    main()
