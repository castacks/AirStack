import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from agent_schema import action_to_episode, validate_action, validate_intent


def test_action_is_no_noise_and_builds_a_valid_episode():
    action = validate_action({"layout": "hard", "layout_seed": 7, "delay_s": .25,
                              "patch_enabled": True, "patch_size_m": .9,
                              "patch_start_s": 5, "patch_duration_s": 10})
    episode = action_to_episode(action, "mononav", 42, "test", {"mission_mode": "goal", "timeout": 180})
    assert episode["condition"]["rgb_noise"] == 0
    assert episode["condition"]["depth_noise"] == 0
    assert episode["condition"]["delay"] == .25
    assert episode["patch_start_s"] == 5


@pytest.mark.parametrize("action", [
    {"noise": 1}, {"rgb_noise": 1}, {"depth_noise": 1}, {"light": 900},
    {"layout": "stock"}, {"layout_seed": 8}, {"delay_s": .3},
    {"patch_enabled": False, "patch_start_s": 5}, {"patch_size_m": 1.0},
])
def test_invalid_or_out_of_scope_actions_are_rejected(action):
    with pytest.raises(ValueError):
        validate_action(action)


def test_intent_is_high_level_and_cannot_request_noise_or_commands():
    intent = validate_intent({"hypothesis": "late obstacle visibility", "reason": "test a turn",
                              "layouts": ["medium"], "delay_band": "high", "patch_mode": "timed"})
    assert intent["layouts"] == ["medium"]
    for invalid in [{"noise": "high"}, {"command": "docker stop"}, {"layouts": ["stock"]}]:
        with pytest.raises(ValueError):
            validate_intent(invalid)
