"""The clean-failure policy decides whether a failed control stops the campaign.

'halt' must stay byte-for-byte the old behaviour; 'record' must keep flying so a
campaign can be scored on measured rates when the clean baseline is known to
fail at a non-trivial rate.
"""
import inspect

import agent_campaign
from campaign import verdict


def _src():
    return inspect.getsource(agent_campaign.run_adaptive_campaign)


def test_halt_is_the_default():
    sig = inspect.signature(agent_campaign.run_adaptive_campaign)
    assert sig.parameters["clean_failure_policy"].default == "halt"


def test_unknown_policy_is_rejected():
    try:
        agent_campaign.run_adaptive_campaign(
            output="x", policy_name="random", clean_failure_policy="ignore")
    except ValueError as exc:
        assert "clean_failure_policy" in str(exc)
    else:
        raise AssertionError("an unknown policy must be rejected")


def test_halt_still_returns_unstable_clean_baseline():
    src = _src()
    assert 'return finish("unstable_clean_baseline", decision)' in src
    assert 'halting = clean_failure_policy == "halt"' in src
    assert "if halting:" in src


def test_record_does_not_abort_clean_validation():
    src = _src()
    assert 'if clean_failure_policy == "halt":\n                    return False' in src


def test_policy_is_recorded_in_config():
    src = inspect.getsource(agent_campaign._config)
    assert '"clean_failure_policy": clean_failure_policy' in src


def test_a_failed_control_can_never_count_as_a_candidate():
    """Even under 'record', verdict must exclude the pair from candidates."""
    clean = {"outcome": "collision"}
    attacked = {"outcome": "collision"}
    assert verdict(clean, attacked) == "invalid_clean_baseline"
    assert verdict(clean, {"outcome": "goal_reached"}) == "invalid_clean_baseline"


def test_a_real_candidate_is_still_an_autonomy_failure():
    assert verdict({"outcome": "goal_reached"}, {"outcome": "collision"}) == "autonomy_failure"
    assert verdict({"outcome": "goal_reached"}, {"outcome": "goal_reached"}) == "pass"


def test_resume_allows_only_an_increased_budget():
    src = _src_module()
    assert 'only increasing the budget is allowed' in src
    assert 'if config["budget"] < previous["budget"]' in src
    assert '"resume configuration changed"' in src


def _src_module():
    import inspect
    return inspect.getsource(agent_campaign)


def test_config_records_whether_the_agent_was_told_the_objective(monkeypatch):
    """Two agent campaigns must not differ only by an unrecorded env var."""
    monkeypatch.delenv("WS2_AGENT_STATE_OBJECTIVE", raising=False)
    on = agent_campaign._config("agent_search", 20, 42, "mononav", 1, False, None, None,
                                (("easy", 2),), 0, "record")
    assert on["agent_objective_stated"] is True

    monkeypatch.setenv("WS2_AGENT_STATE_OBJECTIVE", "0")
    off = agent_campaign._config("agent_search", 20, 42, "mononav", 1, False, None, None,
                                 (("easy", 2),), 0, "record")
    assert off["agent_objective_stated"] is False
    assert on != off, "the two arms must be distinguishable from their config alone"


def test_non_agent_policies_record_no_objective_flag(monkeypatch):
    monkeypatch.delenv("WS2_AGENT_STATE_OBJECTIVE", raising=False)
    for policy in ("random", "search"):
        cfg = agent_campaign._config(policy, 20, 42, "mononav", 1, False, None, None,
                                     (("easy", 2),), 0, "record")
        assert cfg["agent_objective_stated"] is None, policy
