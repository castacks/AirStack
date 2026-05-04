#!/usr/bin/env python3
"""Generate AirSim settings.json from Jinja2 template and environment variables."""

import os
import json
from pathlib import Path
from jinja2 import Template

SCRIPT_DIR = Path(__file__).parent
TEMPLATE_PATH = SCRIPT_DIR / "settings.json.j2"
OUTPUT_PATH = SCRIPT_DIR / "settings.json"

params = {
    "num_robots": int(os.environ.get("NUM_ROBOTS", "1")),
    "width": int(os.environ.get("AIRSIM_CAM_WIDTH", "480")),
    "height": int(os.environ.get("AIRSIM_CAM_HEIGHT", "300")),
    "fov": int(os.environ.get("AIRSIM_CAM_FOV", "90")),
    "cam_x": float(os.environ.get("AIRSIM_CAM_X", "0.4")),
    "cam_y": float(os.environ.get("AIRSIM_CAM_Y", "0.06")),
    "cam_z": float(os.environ.get("AIRSIM_CAM_Z", "0")),
    "cam_pitch": float(os.environ.get("AIRSIM_CAM_PITCH", "0")),
    "spawn_spacing": float(os.environ.get("AIRSIM_SPAWN_SPACING", "3")),
}

template = Template(TEMPLATE_PATH.read_text())
rendered = template.render(**params)

# Validate JSON and pretty-print
settings = json.loads(rendered)
OUTPUT_PATH.write_text(json.dumps(settings, indent=2) + "\n")

vehicles = list(settings["Vehicles"].keys())
print(f"Generated {OUTPUT_PATH.name}: {len(vehicles)} vehicle(s) — {', '.join(vehicles)}")
