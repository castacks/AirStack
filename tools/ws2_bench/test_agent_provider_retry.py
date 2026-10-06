"""Transport failures may retry; a malformed intent must still fail the campaign."""
import urllib.error

import agent_policies
from agent_policies import OpenAICompatibleIntentProvider as P


def _provider(monkeypatch, **kw):
    monkeypatch.setenv("WS2_AGENT_ENDPOINT", "https://example.invalid/chat/completions")
    monkeypatch.setenv("WS2_AGENT_API_KEY", "k")
    monkeypatch.setenv("WS2_AGENT_MODEL", "m")
    return P(**kw)


def test_defaults_are_unchanged(monkeypatch):
    p = _provider(monkeypatch)
    assert p.timeout_s == 30
    assert p.attempts == 1


def test_env_overrides_timeout_and_attempts(monkeypatch):
    monkeypatch.setenv("WS2_AGENT_TIMEOUT_S", "90")
    monkeypatch.setenv("WS2_AGENT_ATTEMPTS", "3")
    p = _provider(monkeypatch)
    assert p.timeout_s == 90 and p.attempts == 3


def test_zero_attempts_is_rejected(monkeypatch):
    try:
        _provider(monkeypatch, attempts=0)
    except ValueError:
        return
    raise AssertionError("attempts=0 must be rejected")


def test_transport_failure_is_retried_then_raises(monkeypatch):
    calls = []

    def boom(*a, **k):
        calls.append(1)
        raise urllib.error.URLError("down")

    monkeypatch.setattr(agent_policies.urllib.request, "urlopen", boom)
    monkeypatch.setattr(agent_policies.time, "sleep", lambda *_: None)
    p = _provider(monkeypatch, attempts=3)
    try:
        p.propose({"x": 1})
    except RuntimeError as exc:
        assert "after 3 attempts" in str(exc)
    else:
        raise AssertionError("must raise once attempts are exhausted")
    assert len(calls) == 3, f"expected 3 transport attempts, got {len(calls)}"


def test_transport_recovers_on_a_later_attempt(monkeypatch):
    state = {"n": 0}

    class Resp:
        def __enter__(self): return self
        def __exit__(self, *a): return False
        def read(self): return b""

    def flaky(*a, **k):
        state["n"] += 1
        if state["n"] < 2:
            raise urllib.error.URLError("blip")
        return Resp()

    monkeypatch.setattr(agent_policies.urllib.request, "urlopen", flaky)
    monkeypatch.setattr(agent_policies.time, "sleep", lambda *_: None)
    monkeypatch.setattr(agent_policies.json, "load", lambda _: {
        "choices": [{"message": {"content": '{"hypothesis":"h","reason":"r","layouts":["easy"],'
                                            '"delay_band":"low","patch_mode":"disabled"}'}}]})
    p = _provider(monkeypatch, attempts=3)
    intent = p.propose({"x": 1})
    assert intent["delay_band"] == "low"
    assert state["n"] == 2


def test_malformed_intent_is_not_retried(monkeypatch):
    calls = []

    class Resp:
        def __enter__(self): return self
        def __exit__(self, *a): return False

    def once(*a, **k):
        calls.append(1)
        return Resp()

    monkeypatch.setattr(agent_policies.urllib.request, "urlopen", once)
    monkeypatch.setattr(agent_policies.time, "sleep", lambda *_: None)
    monkeypatch.setattr(agent_policies.json, "load", lambda _: {
        "choices": [{"message": {"content": '{"delay_band":"banana"}'}}]})
    p = _provider(monkeypatch, attempts=3)
    try:
        p.propose({"x": 1})
    except RuntimeError as exc:
        assert "invalid structured output" in str(exc)
    else:
        raise AssertionError("a malformed intent must fail the campaign")
    assert len(calls) == 1, "a bad intent must not be retried"


def test_system_prompt_states_the_length_limit(monkeypatch):
    """The schema is only data in the user message; the limit must be an instruction."""
    sent = {}

    class Resp:
        def __enter__(self): return self
        def __exit__(self, *a): return False

    def capture(req, *a, **k):
        sent["body"] = __import__("json").loads(req.data.decode())
        return Resp()

    monkeypatch.setattr(agent_policies.urllib.request, "urlopen", capture)
    monkeypatch.setattr(agent_policies.json, "load", lambda _: {
        "choices": [{"message": {"content": '{"hypothesis":"h","reason":"r","layouts":["easy"],'
                                            '"delay_band":"low","patch_mode":"disabled"}'}}]})
    p = _provider(monkeypatch)
    p.propose({"x": 1})
    system = sent["body"]["messages"][0]["content"]
    assert "240 characters" in system, "the system prompt must state the hard limit"
    assert sent["body"]["messages"][0]["role"] == "system"


def _capture_system(monkeypatch):
    sent = {}

    class Resp:
        def __enter__(self): return self
        def __exit__(self, *a): return False

    def capture(req, *a, **k):
        sent["body"] = __import__("json").loads(req.data.decode())
        return Resp()

    monkeypatch.setattr(agent_policies.urllib.request, "urlopen", capture)
    monkeypatch.setattr(agent_policies.json, "load", lambda _: {
        "choices": [{"message": {"content": '{"hypothesis":"h","reason":"r","layouts":["easy"],'
                                            '"delay_band":"low","patch_mode":"disabled"}'}}]})
    p = _provider(monkeypatch)
    p.propose({"x": 1})
    return sent["body"]["messages"][0]["content"]


def test_objective_is_stated_by_default(monkeypatch):
    """An attack-selection agent is told what it is for unless asked otherwise."""
    monkeypatch.delenv("WS2_AGENT_STATE_OBJECTIVE", raising=False)
    system = _capture_system(monkeypatch)
    assert "objective" in system.lower()
    assert "clean control passes" in system
    assert "240 characters" in system, "the length limit must survive"


def test_objective_can_be_switched_off_to_reproduce_the_first_arm(monkeypatch):
    monkeypatch.setenv("WS2_AGENT_STATE_OBJECTIVE", "0")
    assert "objective" not in _capture_system(monkeypatch).lower()


def test_only_an_explicit_zero_disables_it(monkeypatch):
    for value in ("1", "true", "yes", ""):
        monkeypatch.setenv("WS2_AGENT_STATE_OBJECTIVE", value)
        assert agent_policies.objective_is_stated() is (value != "0"), value
    monkeypatch.setenv("WS2_AGENT_STATE_OBJECTIVE", "0")
    assert agent_policies.objective_is_stated() is False
