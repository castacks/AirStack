"""Strict, no-noise action schemas for the WS2 adaptive campaign policies."""
from __future__ import annotations

import hashlib
import json
import math

from conditions import DIFFICULTY_COUNTS


ACTION_SCHEMA_VERSION = "ws2_agent_action_v1"
INTENT_SCHEMA_VERSION = "ws2_agent_intent_v1"
# conditions.validate() also accepts furnished_a/furnished_b, which are the
# two SIMPLEST families in layouts.json (6 and 7 objects per variant, versus
# 11 for easy). Deriving the action space from DIFFICULTY_COUNTS alone hid
# them from every agent policy, confining 'let the agent pick a layout' to
# the three hardest families. 'stock' stays out: it is the no-layout
# baseline, not a scene the agent should be able to choose.
LAYOUTS = ("furnished_a", "furnished_b", *DIFFICULTY_COUNTS)
DELAYS = (0.0, 0.05, 0.15, 0.25)
PATCH_SIZES = (0.3, 0.6, 0.9)
PATCH_STARTS = (0.0, 5.0, 10.0)
PATCH_DURATIONS = (0.0, 5.0, 10.0)  # zero means active for the rest of the mission


def _number(value, name, lo, hi):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if not lo <= value <= hi:
        raise ValueError(f"{name} must be in {lo}..{hi}")
    return value


def _choice(value, choices, name):
    if value not in choices:
        raise ValueError(f"{name} must be one of {list(choices)}")
    return value


def validate_action(raw):
    """Validate one attacked-flight action.

    Noise is intentionally absent. The clean twin is made by the campaign
    runner and disables only delay and patch state.
    """
    if not isinstance(raw, dict):
        raise ValueError("action must be a mapping")
    forbidden = {"rgb_noise", "depth_noise", "noise", "light", "patch_strength", "patch_height"}
    if forbidden & raw.keys():
        raise ValueError("agent actions cannot control noise, light, or legacy patch settings")
    allowed = {"layout", "layout_seed", "delay_s", "patch_enabled", "patch_size_m", "patch_start_s", "patch_duration_s"}
    if raw.keys() - allowed:
        raise ValueError(f"unknown action fields: {raw.keys() - allowed}")
    action = {
        "layout": "easy",
        "layout_seed": 0,
        "delay_s": 0.0,
        "patch_enabled": True,
        "patch_size_m": 0.6,
        "patch_start_s": 0.0,
        "patch_duration_s": 0.0,
    }
    action.update(raw)
    action["layout"] = _choice(action["layout"], LAYOUTS, "layout")
    if isinstance(action["layout_seed"], bool) or not isinstance(action["layout_seed"], int) or not 0 <= action["layout_seed"] <= 7:
        raise ValueError("layout_seed must be an integer in 0..7")
    action["delay_s"] = _number(action["delay_s"], "delay_s", min(DELAYS), max(DELAYS))
    if not isinstance(action["patch_enabled"], bool):
        raise ValueError("patch_enabled must be boolean")
    action["patch_size_m"] = _number(action["patch_size_m"], "patch_size_m", min(PATCH_SIZES), max(PATCH_SIZES))
    action["patch_start_s"] = _number(action["patch_start_s"], "patch_start_s", min(PATCH_STARTS), max(PATCH_STARTS))
    action["patch_duration_s"] = _number(action["patch_duration_s"], "patch_duration_s", min(PATCH_DURATIONS), max(PATCH_DURATIONS))
    if not action["patch_enabled"] and (action["patch_start_s"] or action["patch_duration_s"]):
        raise ValueError("patch timing requires patch_enabled=true")
    return action


def action_key(action):
    return json.dumps(validate_action(action), sort_keys=True, separators=(",", ":"))


def action_hash(action):
    return hashlib.sha256(action_key(action).encode()).hexdigest()


def action_to_episode(action, planner, campaign_seed, name, mission):
    """Build a resolved-episode input while keeping noise pinned to zero."""
    action = validate_action(action)
    return {
        "name": name,
        "planner": planner,
        "condition": {
            "layout": action["layout"],
            "layout_seed": action["layout_seed"],
            "seed": campaign_seed,
            "light": 1800.0,
            "rgb_noise": 0.0,
            "depth_noise": 0.0,
            "delay": action["delay_s"],
            "patch_enabled": action["patch_enabled"],
            "patch_size": action["patch_size_m"],
        },
        "patch_start_s": action["patch_start_s"],
        "patch_duration_s": action["patch_duration_s"],
        **mission,
    }


def validate_intent(raw):
    """Validate the high-level, non-executable output from an LLM provider."""
    if not isinstance(raw, dict):
        raise ValueError("intent must be a mapping")
    allowed = {"hypothesis", "reason", "layouts", "delay_band", "patch_mode"}
    if raw.keys() - allowed:
        raise ValueError(f"unknown intent fields: {raw.keys() - allowed}")
    if allowed - raw.keys():
        raise ValueError(f"missing intent fields: {allowed - raw.keys()}")
    intent = {
        "hypothesis": "explore an untested valid scene",
        "reason": "cover a new valid condition",
        "layouts": list(LAYOUTS),
        "delay_band": "medium",
        "patch_mode": "continuous",
    }
    intent.update(raw)
    for field in ("hypothesis", "reason"):
        if not isinstance(intent[field], str) or not intent[field].strip() or len(intent[field]) > 240:
            raise ValueError(f"{field} must be nonempty text of at most 240 characters")
        intent[field] = intent[field].strip()
    if not isinstance(intent["layouts"], list) or not intent["layouts"]:
        raise ValueError("layouts must be a nonempty list")
    if len(set(intent["layouts"])) != len(intent["layouts"]):
        raise ValueError("layouts must not contain duplicates")
    intent["layouts"] = [_choice(x, LAYOUTS, "layout") for x in intent["layouts"]]
    intent["delay_band"] = _choice(intent["delay_band"], ("low", "medium", "high"), "delay_band")
    intent["patch_mode"] = _choice(intent["patch_mode"], ("disabled", "continuous", "timed"), "patch_mode")
    return intent


def intent_prompt_schema(layouts=None):
    """A JSON-serializable schema supplied to an OpenAI-compatible provider.

    `layouts` narrows the offered tiers to the ones the campaign can actually
    fly. Offering a tier with no qualified scene lets the model pick an intent
    that filters to nothing, which aborts the campaign.
    """
    layouts = list(layouts) if layouts else list(LAYOUTS)
    if not set(layouts) <= set(LAYOUTS):
        raise ValueError("layouts must be a subset of the known tiers")
    return {
        "type": "object",
        "additionalProperties": False,
        "required": ["hypothesis", "reason", "layouts", "delay_band", "patch_mode"],
        "properties": {
            "hypothesis": {"type": "string", "maxLength": 240},
            "reason": {"type": "string", "maxLength": 240},
            "layouts": {"type": "array", "minItems": 1, "uniqueItems": True,
                        "items": {"type": "string", "enum": layouts}},
            "delay_band": {"type": "string", "enum": ["low", "medium", "high"]},
            "patch_mode": {"type": "string", "enum": ["disabled", "continuous", "timed"]},
        },
    }
