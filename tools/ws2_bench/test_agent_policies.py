import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from agent_policies import AgentSearchPolicy, RandomPolicy, SearchPolicy
from mission import SUCCESSES


def result(outcome, clearance=.8, progress=100):
    return {"outcome": outcome, "metrics": {"minimum_obstacle_clearance_m": clearance,
                                                "mission_progress_percent": progress}}


def round_from(decision, clean="goal_reached", attacked="goal_reached", clearance=.8):
    return {"decision": decision, "pairs": [{"planner": "mononav", "clean": result(clean),
            "perturbed": result(attacked, clearance), "verdict": "pass"}]}


def test_random_policy_is_seeded_and_does_not_repeat_an_action():
    first = RandomPolicy(42).choose_next([], 4)
    second = RandomPolicy(42).choose_next([], 4)
    assert first == second
    next_ = RandomPolicy(42).choose_next([round_from(first)], 3)
    assert next_["action"] != first["action"]


def test_random_policy_can_be_limited_to_clean_qualified_layouts():
    allowed = [
        {"layout": "easy", "layout_seed": 2, "delay_s": 0.0, "patch_enabled": False,
         "patch_start_s": 0.0, "patch_duration_s": 0.0},
        {"layout": "easy", "layout_seed": 2, "delay_s": 0.15, "patch_enabled": True,
         "patch_size_m": 0.6, "patch_start_s": 5.0, "patch_duration_s": 10.0},
    ]
    decision = RandomPolicy(42, allowed).choose_next([], 2)
    assert (decision["action"]["layout"], decision["action"]["layout_seed"]) == ("easy", 2)


def test_search_policy_repeats_a_clean_pass_attack_failure_once():
    policy = SearchPolicy(42)
    first = policy.choose_next([], 4)
    failed = round_from(first, attacked="collision")
    confirmation = policy.choose_next([failed], 3)
    assert confirmation["rule"] == "confirm_failure"
    assert confirmation["action"] == first["action"]
    repeated = round_from(confirmation, attacked="collision")
    next_ = policy.choose_next([failed, repeated], 2)
    assert next_["action"] != first["action"]


class FakeProvider:
    def __init__(self):
        self.calls = 0

    def propose(self, context):
        self.calls += 1
        return {"hypothesis": "delayed patch after a turn", "reason": "test delayed visual exposure",
                "layouts": ["hard"], "delay_band": "high", "patch_mode": "timed"}


def test_agent_chooses_a_high_level_intent_and_search_selects_the_action():
    provider = FakeProvider()
    decision = AgentSearchPolicy(42, provider).choose_next([], 4)
    assert provider.calls == 1
    assert decision["intent"]["layouts"] == ["hard"]
    assert decision["action"]["layout"] == "hard"
    assert decision["action"]["delay_s"] in (.15, .25)
    assert decision["action"]["patch_enabled"]
    assert decision["action"]["patch_start_s"] > 0
