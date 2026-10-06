"""The model must only be offered layout tiers the campaign can actually fly.

Offering an unqualified tier lets a valid intent filter to zero actions, which
aborts the campaign. Observed live: 2 of 4 sampled replies chose "medium" while
only easy:2 was qualified.
"""
import json

import agent_policies
from agent_schema import LAYOUTS, intent_prompt_schema


def _enum(schema):
    return schema["properties"]["layouts"]["items"]["enum"]


def test_default_schema_offers_every_tier():
    assert _enum(intent_prompt_schema()) == list(LAYOUTS)


def test_schema_can_be_narrowed():
    assert _enum(intent_prompt_schema(["easy"])) == ["easy"]


def test_empty_narrowing_falls_back_to_all_tiers():
    assert _enum(intent_prompt_schema([])) == list(LAYOUTS)


def test_unknown_tier_is_rejected():
    try:
        intent_prompt_schema(["impossible"])
    except ValueError:
        return
    raise AssertionError("an unknown tier must be rejected")


def test_provider_sends_only_the_allowed_tiers(monkeypatch):
    sent = {}

    class Resp:
        def __enter__(self): return self
        def __exit__(self, *a): return False

    def capture(req, *a, **k):
        sent["body"] = json.loads(req.data.decode())
        return Resp()

    monkeypatch.setenv("WS2_AGENT_ENDPOINT", "https://example.invalid/chat/completions")
    monkeypatch.setenv("WS2_AGENT_API_KEY", "k")
    monkeypatch.setenv("WS2_AGENT_MODEL", "m")
    monkeypatch.setattr(agent_policies.urllib.request, "urlopen", capture)
    monkeypatch.setattr(agent_policies.json, "load", lambda _: {
        "choices": [{"message": {"content": '{"hypothesis":"h","reason":"r","layouts":["easy"],'
                                            '"delay_band":"low","patch_mode":"disabled"}'}}]})
    p = agent_policies.OpenAICompatibleIntentProvider()
    p.propose({"allowed_layout_tiers": ["easy"]})
    body = json.loads(sent["body"]["messages"][1]["content"])
    assert _enum(body["schema"]) == ["easy"]


def test_campaign_advertises_the_tiers_it_can_fly():
    actions = [a for a in agent_policies.all_actions()
               if (a["layout"], a["layout_seed"]) == ("easy", 2)]
    captured = {}

    class Provider:
        def propose(self, context):
            captured.update(context)
            return {"hypothesis": "h", "reason": "r", "layouts": ["easy"],
                    "delay_band": "low", "patch_mode": "disabled"}

    policy = agent_policies.AgentSearchPolicy(42, Provider(), actions)
    policy.choose_next([], 10)
    assert captured["allowed_layout_tiers"] == ["easy"]


def test_empty_selection_error_names_the_intent():
    actions = [a for a in agent_policies.all_actions()
               if (a["layout"], a["layout_seed"]) == ("easy", 2)]

    class Provider:
        def propose(self, context):
            return {"hypothesis": "h", "reason": "r", "layouts": ["hard"],
                    "delay_band": "low", "patch_mode": "disabled"}

    policy = agent_policies.AgentSearchPolicy(42, Provider(), actions)
    try:
        policy.choose_next([], 10)
    except RuntimeError as exc:
        assert "hard" in str(exc) and "qualified tiers" in str(exc), str(exc)
        return
    raise AssertionError("an intent selecting no actions must raise")
