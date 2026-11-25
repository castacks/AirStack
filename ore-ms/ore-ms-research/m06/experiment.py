"""Capture synchronized RGB and LWIR pixels and normalized path ground truth."""
import argparse
import csv
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import random

from adapters import (ROOT, KERNELS, AnalogNoise, detect, optical_depth, radiance,
                      segments, snapshots)
from geometry import Camera, Rectangle, dot, hits

FRAME_KEY = ("case", "realization", "time_s")
FRAME_FIELDS = (*FRAME_KEY, "scene", "sample_mode", "split", "environment", "profile",
                "environment_seed", "rgb_seed", "lwir_seed", "burden_coefficients",
                "temperature_coefficients", "projected", "visible", "visible_fraction")
OBS_FIELDS = (*(f"{band}_{field}" for band in "RGB" for field in
                ("direct", "path", "analog", "code", "low_clip", "high_clip")),
              "lwir_emission", "lwir_reflection", "lwir_path", "lwir_analog", "lwir_code",
              "lwir_low_clip", "lwir_high_clip")
PAIR_FIELDS = (*FRAME_KEY, "row", "column", "scene", "environment", "dx", "dy", "dz",
               "target_projected", "target_visible", "status", "first_id", "material",
               "first_x_m", "first_y_m", "first_z_m", "first_range_m", "first_camera_depth_m",
               "path_id", "burden_avg", "rgb_tau550", "lwir_tau11", *OBS_FIELDS)
SEGMENT_FIELDS = ("path_id", "segment", "environment", "lo_m", "hi_m", "z_m", "burden", "air_K",
                  "rgb_absorption_550_m_inv", "rgb_scattering_550_m_inv", "rgb_exponent",
                  "lwir_absorption_8_m_inv", "lwir_absorption_11_m_inv", "lwir_absorption_14_m_inv")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def configurations(config):
    for case in config["cases"]:
        yield case, 0, "controls"
    ensemble = config["ensemble"]
    for i in range(ensemble["size"]):
        split = "development" if i < ensemble["dev_size"] else "holdout"
        fields = {"profile": "stochastic", "environment_seed": ensemble["seed_start"] + i,
                  "times_s": ensemble["times_s"], "probe": ensemble["probe"]}
        yield fields | {"id": "ensemble_fixed"}, i, split
        yield fields | {"id": "ensemble_covered", "obstacle": "partial"}, i, split


class Dataset:
    def __init__(self, config, frame_out, pair_out, path_out):
        self.config = config
        self.camera = Camera(**config["camera"])
        self.frame_out, self.pair_out, self.path_out = frame_out, pair_out, path_out
        self.path_count = 0
        self.counts = dict.fromkeys(("frames", "pairs", "segments"), 0)
        self.rgb_lwir = lru_cache(maxsize=128)(self.rgb_lwir)

    def rgb_lwir(self, material, path):
        return radiance(self.config, self.config["materials"][material], path)

    def capture(self, case, realization, split):
        cfg = self.config
        camera = self.camera
        target = dict(cfg["target"])
        target["material"] = case.get("material", target["material"])
        background = dict(cfg["background"])
        for surface, name in ((target, "target_x"), (background, "background_x")):
            if name in case:
                surface["center"] = [case[name], *surface["center"][1:]]
        objects = [Rectangle(**target, target=True), Rectangle(**background)]
        obstacle = case.get("obstacle")
        if obstacle:
            objects.append(Rectangle(**cfg["obstacles"][obstacle]))
        scene = ("offscreen" if "background_x" in case else
                 "empty_target" if "target_x" in case else obstacle or "room")
        mode = "probe" if "probe" in case else "image"
        samples = ([tuple(case["probe"])] if mode == "probe" else
                   [(r, c) for r in range(camera.height) for c in range(camera.width)])
        settings = cfg["sensor"] | case.get("sensor", {})
        profile_id = case["profile"]
        env_seed = case.get("environment_seed", 0)
        for time, burden, heat in snapshots(cfg, profile_id, env_seed, case.get("times_s", [0])):
            key = (case["id"], realization, time)
            environment = f"{profile_id}:{env_seed}:{time}"
            rgb_seed = cfg["sensor_seed"] + env_seed * 10000 + time * 2
            lwir_seed = rgb_seed + 1
            rng_rgb, rng_lwir = AnalogNoise(rgb_seed), random.Random(lwir_seed)
            posed = [(obj, obj.at(time)) for obj in objects]
            projected = visible = 0
            path_cache = {}
            for row, col in samples:
                self.counts["pairs"] += 1
                direction = camera.ray(col, row)
                target_range, first, length = hits(camera, direction, posed)
                target_projected = target_range is not None
                target_visible = first is not None and first.target
                projected += target_projected
                visible += target_visible
                common = (*key, row, col, scene, environment, *direction,
                          int(target_projected), int(target_visible))
                if first is None:
                    self.pair_out.writerow((*common, "no_hit", *[""] *
                                            (len(PAIR_FIELDS) - len(common) - 1)))
                    continue
                z_direction = direction[2]
                path_key = (length, z_direction)
                if path_key not in path_cache:
                    path = segments(cfg, profile_id, time, burden, heat,
                                    camera.origin[2], z_direction, length)
                    self.path_count += 1
                    for index, segment in enumerate(path):
                        self.path_out.writerow((self.path_count, index, environment, *segment.values()))
                        self.counts["segments"] += 1
                    path_cache[path_key] = (self.path_count, path)
                path_id, path = path_cache[path_key]
                point = tuple(o + length * d for o, d in zip(camera.origin, direction, strict=True))
                tau_rgb = math.exp(-optical_depth(path, "RGB", 550))
                tau_lwir = math.exp(-optical_depth(path, "LWIR", 11))
                mean_burden = math.fsum(s.length * s.burden for s in path) / length
                bands = self.rgb_lwir(first.material, () if profile_id == "vacuum" else path)
                status = ("target_visible" if target_visible else "target_occluded"
                          if target_projected else "other_hit")
                self.pair_out.writerow((*common, status, first.id, first.material, *point, length,
                                         length * dot(direction, camera.axes[2]), path_id, mean_burden,
                                         tau_rgb, tau_lwir, *detect(*bands, settings, rng_rgb, rng_lwir)))
            self.frame_out.writerow((*key, scene, mode, split, environment, profile_id, env_seed,
                                      rgb_seed, lwir_seed, json.dumps(burden), json.dumps(heat),
                                      projected, visible, visible / projected if projected else None))
            self.counts["frames"] += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    args.output.mkdir()
    with ((args.output / "frames.csv").open("w", newline="") as frames,
          (args.output / "pairs.csv").open("w", newline="") as pairs,
          (args.output / "segments.csv").open("w", newline="") as paths):
        writers = tuple(csv.writer(f, lineterminator="\n") for f in (frames, pairs, paths))
        for writer, fields in zip(writers, (FRAME_FIELDS, PAIR_FIELDS, SEGMENT_FIELDS), strict=True):
            writer.writerow(fields)
        dataset = Dataset(config, *writers)
        for case, realization, split in configurations(config):
            dataset.capture(case, realization, split)
    metadata = {"experiment_id": config["experiment_id"], "schema_version": config["schema_version"],
                "configuration": config, "configuration_sha256": digest(args.config),
                "starting_revision": config["starting_revision"],
                "kernel_sha256": {file: digest(ROOT / file) for file in KERNELS},
                "table_counts": dataset.counts,
                "path_count": dataset.path_count, "time_units": "s", "range_units": "m",
                "band_radiance_units": "W m^-2 sr^-1",
                "synthetic_co_registration": True}
    (args.output / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
