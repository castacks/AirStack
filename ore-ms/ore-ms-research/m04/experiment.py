"""Generate per-pixel geometry, first-hit radiometry and frame-level visibility."""
import argparse
import csv
from functools import lru_cache
import json
from pathlib import Path

from geometry import Camera, Rectangle, dot, hits
from radiometry import rgb, lwir


def frame_inputs(config, case):
    camera_fields = config["camera"] | {
        field: case[f"camera_{field}"] for field in ("origin", "axes")
        if f"camera_{field}" in case}
    target_fields = config["target"] | {
        field: case[f"target_{field}"] for field in ("center", "material", "velocity")
        if f"target_{field}" in case}
    objects = [Rectangle(**target_fields, target=True)]
    objects.extend(Rectangle(**config["obstacles"][key]) for key in case.get("obstacles", []))
    if len({item.id for item in objects}) != len(objects):
        raise ValueError("surface IDs must be unique in each scene")
    return Camera(**camera_fields), objects


def run_case(config, case, geometry, radiometry, frames):
    camera, objects = frame_inputs(config, case)
    settings = config["default_detectors"] | {
        key: case[key] for key in config["default_detectors"] if key in case}
    medium = config["medium"] if case.get("medium") else None

    @lru_cache(maxsize=None)
    def measure(material_id, distance):
        material = config["materials"][material_id]
        return (*rgb(config, material, distance, medium, settings),
                *lwir(config, material, distance, medium, settings))

    for time in case.get("times", [0]):
        surfaces = [(item, item.at(time)) for item in objects]
        support = visible = foreground_hits = 0
        for row in range(camera.height):
            for col in range(camera.width):
                direction = camera.ray(col, row)
                target_range, first, first_range = hits(camera, direction, surfaces)
                projected = target_range is not None
                target_visible = first is not None and first.target
                support += projected
                visible += target_visible
                foreground_hits += first is not None and not first.target
                depth = first_range * dot(direction, camera.axes[2]) if first else None
                geometry.writerow((case["name"], time, row, col, int(projected), int(target_visible),
                                   target_range, first.id if first else "", first_range, depth))
                if first:
                    observed = measure(first.material, first_range if medium else 0)
                    radiometry.writerow((case["name"], time, row, col, first.id, first.material,
                                         first_range, *observed))
        frames.writerow((case["name"], time, support, visible, visible / support if support else None,
                         foreground_hits))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    args.output.mkdir()
    with ((args.output / "geometry.csv").open("w", newline="") as geometry,
          (args.output / "radiometry.csv").open("w", newline="") as radiometry,
          (args.output / "frames.csv").open("w", newline="") as frames):
        writers = tuple(csv.writer(source, lineterminator="\n") for source in
                        (geometry, radiometry, frames))
        writers[0].writerow(("case", "time_s", "row", "column", "target_projected",
                             "target_visible", "target_only_range_m", "first_id",
                             "first_range_m", "first_camera_depth_m"))
        writers[1].writerow(("case", "time_s", "row", "column", "first_id", "material",
                             "first_range_m", *(f"{channel}_{name}" for channel in "RGB" for name in
                                               ("direct", "path", "code", "clipped")),
                             "lwir_emission", "lwir_reflection", "lwir_path", "lwir_code", "lwir_clipped"))
        writers[2].writerow(("case", "time_s", "projected_target", "visible_target",
                             "visible_fraction", "foreground_first_hits"))
        for case in config["cases"]:
            run_case(config, case, *writers)


if __name__ == "__main__":
    main()
