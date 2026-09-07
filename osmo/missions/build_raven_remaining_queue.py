#!/usr/bin/env python3
"""Build per-locale three-cell shared-RAVEN missions from frozen overlays.

The recovery mission is the runtime template because it carries the validated
two-GPU split, original sensor/contact settings, health gates, recorder, and
pass-only upload policy. Scene geometry comes from the already-generated
frozen overlays and their matching baseline mission spawn records.
"""

from copy import deepcopy
from pathlib import Path
import json
import sys

import yaml


ROOT = Path(__file__).resolve().parents[2]
MISSION_DIR = ROOT / "osmo" / "missions"
OVERLAY_DIR = (ROOT / "robot" / "ros_ws" / "src" / "global" / "planners"
               / "search_baselines" / "config")
sys.path.insert(0, str(ROOT / "robot" / "ros_ws" / "src" / "global"
                       / "planners" / "search_baselines" / "search_baselines"))
import sector  # noqa: E402


GROUPS = {
    # Prepared but not inserted in the durable queue while the active L1
    # attempt is still allowed to finish. If that attempt is stopped or fails,
    # this provides the same one-cell 12/8 retry contract as every later cell.
    "fire_suburban_l1_retry": [
        ("FireSuburbanL1V1", "raven_fire_suburban_recovery_2gpu1.yaml"),
    ],
    "fire_suburban_after_l1": [
        ("FireSuburbanL2V1", "raven_fire_suburban_recovery_2gpu1.yaml"),
        ("FireSuburbanL3V1", "raven_fire_suburban_recovery_2gpu1.yaml"),
    ],
    "hurricane_suburban": [
        ("HurricaneSuburbanL1V1", "raven_hurricane_suburban_8robot_batch.yaml"),
        ("HurricaneSuburbanL2V1", "raven_hurricane_suburban_8robot_batch.yaml"),
        ("HurricaneSuburbanL3V1", "raven_hurricane_suburban_8robot_batch.yaml"),
    ],
    "tornado_suburban": [
        ("TornadoSuburbanL1V1", "raven_tornado_suburban_8robot_batch.yaml"),
        ("TornadoSuburbanL2V1", "raven_tornado_suburban_8robot_batch.yaml"),
        ("TornadoSuburbanL3V1", "raven_tornado_suburban_8robot_batch.yaml"),
    ],
    "earthquake_suburban": [
        ("EarthquakeSuburbanL1V1", "earthquake_suburban_l1_8robot_optimized_pod57.yaml"),
        ("EarthquakeSuburbanL2V1", "earthquake_suburban_l2_8robot_optimized_pod56.yaml"),
        ("EarthquakeSuburbanL3V1", "earthquake_suburban_l3_8robot_optimized_pod57.yaml"),
    ],
    "fire_urban": [
        ("FireUrbanL1V1", "urban_fire_8robot.yaml"),
        ("FireUrbanL2V1", "urban_fire_8robot.yaml"),
        ("FireUrbanL3V1", "urban_fire_8robot.yaml"),
    ],
    "hurricane_urban": [
        ("HurricaneUrbanL1V1", "hurricane_urban_l1_8robot_optimized_dev191.yaml"),
        ("HurricaneUrbanL2V1", "hurricane_urban_l2_8robot_optimized_dev191.yaml"),
        ("HurricaneUrbanL3V1", "hurricane_urban_l3_8robot_optimized_dev191.yaml"),
    ],
    "tornado_urban": [
        ("TornadoUrbanL1V1", "raven_tornado_urban_l123_8robot_2gpu.yaml"),
        ("TornadoUrbanL2V1", "raven_tornado_urban_l123_8robot_2gpu.yaml"),
        ("TornadoUrbanL3V1", "raven_tornado_urban_l123_8robot_2gpu.yaml"),
    ],
    "earthquake_urban": [
        ("EarthquakeUrbanL1V1", "urban_earthquake_l12_8robot_optimized_batch.yaml"),
        ("EarthquakeUrbanL2V1", "urban_earthquake_l12_8robot_optimized_batch.yaml"),
        ("EarthquakeUrbanL3V1", "urban_earthquake_l3_8robot_optimized_batch.yaml"),
    ],
}


def load_yaml(path):
    with path.open() as stream:
        return yaml.safe_load(stream)


def find_env(scene, source_name):
    source = load_yaml(MISSION_DIR / source_name)
    return next(e for e in source["environments"]
                if e.get("RESULTS_SCENE") == scene)


def polygon_from_overlay(env):
    overlay = load_yaml(OVERLAY_DIR / env["scene_yaml"])
    flat = overlay["search_planner"]["ros__parameters"]["search_area_xy"]
    return [[float(flat[i]), float(flat[i + 1])]
            for i in range(0, len(flat), 2)]


def points(poly):
    return {"points": [{"x": float(x), "y": float(y), "z": 0.0}
                       for x, y in poly]}


def raven_env(scene, source_name):
    source = deepcopy(find_env(scene, source_name))
    # Existing RAVEN missions already carry the exact frozen-plan polygons.
    if "search_area" in source and "goal_per_robot" in source:
        source["name"] = source["name"].split("_raven", 1)[0] + "_raven"
        source["method"] = "raven"
        return source

    poly = polygon_from_overlay(source)
    sectors = [sector.sector_for(poly, 8, i, mode="rect",
                                 axis="principal", margin_m=0.0)
               for i in range(8)]
    keep = {key: source[key] for key in
            ("FROZEN_SCENE", "RESULTS_SCENE", "SPAWN_CONFIGS")}
    keep.update({
        "name": scene.lower() + "_raven",
        "method": "raven",
        "search_area": points(poly),
        "goal_per_robot": {i + 1: {"search_area": points(part)}
                           for i, part in enumerate(sectors)},
    })
    # Assert the frozen spawn plan is structurally complete; do not invent it.
    assert len(json.loads(keep["SPAWN_CONFIGS"])) == 8
    return keep


def main():
    template = load_yaml(MISSION_DIR / "raven_fire_suburban_recovery_2gpu1.yaml")
    for group, scenes in GROUPS.items():
        mission = deepcopy(template)
        mission["name"] = f"raven_{group}_remaining_2gpu1"
        mission["iterations"] = len(scenes)
        mission["environment_order"] = "round_robin"
        mission["environments"] = [raven_env(*scene) for scene in scenes]
        mission["env"]["ZED_TIME_SLICE_GROUPS"] = "12"
        mission["env"]["ZED_TIME_SLICE_BURST"] = "8"
        mission["env"]["ZED_HYDRA_TIME_SLICE"] = "true"
        for step in mission["steps"]:
            if step.get("action", {}).get("task") == "semantic_search":
                # 600 simulated seconds can take several wall hours at the
                # observed 8-robot RTF. Keep the science budget unchanged but
                # leave enough wall-clock margin for a healthy search.
                step["action"]["timeout_s"] = 21600
        out = MISSION_DIR / f"raven_{group}_remaining_2gpu1.yaml"
        out.write_text(yaml.safe_dump(mission, sort_keys=False, width=100))
        print(out.relative_to(ROOT))

        # Also emit one-scene missions. The held-mission wrapper is capped just
        # below 12 hours, so an independently bounded process per scene cannot
        # interrupt a healthy later cell merely because earlier cells were slow.
        for env in mission["environments"]:
            single = deepcopy(mission)
            scene_slug = env["name"]
            single["name"] = f"raven_{scene_slug}_remaining_2gpu1"
            single["iterations"] = 1
            single["environments"] = [deepcopy(env)]
            single_out = MISSION_DIR / f"raven_{scene_slug}_remaining_2gpu1.yaml"
            single_out.write_text(yaml.safe_dump(single, sort_keys=False, width=100))
            print(single_out.relative_to(ROOT))


if __name__ == "__main__":
    main()
