"""Validated, replayable Office condition data, shared by simulator and runner."""
import math

DIFFICULTY_COUNTS={'easy':1,'medium':3,'hard':5}


def validate(raw):
    if not isinstance(raw, dict):
        raise ValueError("condition must be a mapping")
    allowed = {"name", "layout", "layout_seed", "seed", "light", "rgb_noise", "depth_noise", "delay", "patch_enabled", "patch_strength", "patch_size", "patch_height"}
    if set(raw) - allowed:
        raise ValueError(f"unknown condition fields: {set(raw) - allowed}")
    c = dict(name="clean", layout="furnished_a", layout_seed=0, seed=42, light=1800., rgb_noise=0.,
             depth_noise=0., delay=0., patch_enabled=False, patch_strength=0.,
             patch_size=.8, patch_height=1.2)
    c.update(raw)
    if not isinstance(c["name"], str) or not c["name"].strip():
        raise ValueError("condition name required")
    if c["layout"] not in ("stock", "furnished_a", "furnished_b",*DIFFICULTY_COUNTS):
        raise ValueError("unsupported layout")
    if isinstance(c["layout_seed"], bool) or not isinstance(c["layout_seed"], int) or not 0<=c["layout_seed"]<=7:
        raise ValueError("layout_seed must select one of the validated layouts, 0..7")
    if isinstance(c["seed"], bool) or not isinstance(c["seed"], int) or not 0 <= c["seed"] < 2**31:
        raise ValueError("seed must be a nonnegative 31-bit integer")
    if not isinstance(c["patch_enabled"], bool):
        raise ValueError("patch_enabled must be boolean")
    for name, limits in {"light": (100, 6000), "rgb_noise": (0, 80), "depth_noise": (0, 2),
                         "delay": (0, 2), "patch_strength": (0, 1), "patch_size": (.1, .95),
                         "patch_height": (.5, 2.4)}.items():
        value = c[name]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not limits[0] <= value <= limits[1]:
            raise ValueError(f"{name} must be in {limits}")
        c[name] = float(value)
    if c["patch_enabled"] and c["layout"] == "stock":
        raise ValueError("patch requires the added column in a furnished layout")
    return c


def sensor_parameters(c):
    return {"disturbance_seed": c["seed"], "rgb_noise_stddev": c["rgb_noise"],
            "depth_noise_stddev_m": c["depth_noise"], "fixed_sensor_delay_s": c["delay"]}
