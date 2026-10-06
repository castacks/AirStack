"""Deterministic WS2 baselines and an optional structured LLM strategist."""
from __future__ import annotations

import json
import os
import time
from pathlib import Path
import random
import urllib.error
import urllib.request

from agent_schema import (ACTION_SCHEMA_VERSION, DELAYS, INTENT_SCHEMA_VERSION, LAYOUTS,
                          PATCH_DURATIONS, PATCH_SIZES, PATCH_STARTS, action_key,
                          validate_action, validate_intent, intent_prompt_schema)
from mission import SUCCESSES


def provider_from_name(name=None, audit_dir=None):
    name = name or os.environ.get("WS2_AGENT_PROVIDER", "openai")
    if name == "claude":
        from claude_provider import ClaudeSubscriptionProvider
        return ClaudeSubscriptionProvider(audit_dir=audit_dir)
    if name == "openai":
        return OpenAICompatibleIntentProvider()
    raise ValueError("provider must be claude or openai")


def all_actions():
    actions = []
    for layout in LAYOUTS:
        for layout_seed in range(8):
            for delay_s in DELAYS:
                actions.append(validate_action({"layout": layout, "layout_seed": layout_seed,
                                                "delay_s": delay_s, "patch_enabled": False,
                                                "patch_start_s": 0.0, "patch_duration_s": 0.0}))
                for patch_size_m in PATCH_SIZES:
                    actions.append(validate_action({"layout": layout, "layout_seed": layout_seed,
                                                    "delay_s": delay_s, "patch_enabled": True,
                                                    "patch_size_m": patch_size_m,
                                                    "patch_start_s": 0.0, "patch_duration_s": 0.0}))
                    for patch_start_s in PATCH_STARTS[1:]:
                        for patch_duration_s in PATCH_DURATIONS[1:]:
                            actions.append(validate_action({"layout": layout, "layout_seed": layout_seed,
                                                            "delay_s": delay_s, "patch_enabled": True,
                                                            "patch_size_m": patch_size_m,
                                                            "patch_start_s": patch_start_s,
                                                            "patch_duration_s": patch_duration_s}))
    return actions


def _tested(history):
    return {action_key(round_["decision"]["action"]) for round_ in history if "action" in round_["decision"]}


def _pair_score(pair):
    clean, attacked = pair["clean"], pair["perturbed"]
    if "infrastructure_error" in (clean["outcome"], attacked["outcome"]):
        return None
    if clean["outcome"] not in SUCCESSES:
        return -10.0
    if attacked["outcome"] not in SUCCESSES:
        return 100.0
    metrics = attacked.get("metrics", {})
    clearance = metrics.get("minimum_obstacle_clearance_m")
    progress = metrics.get("mission_progress_percent")
    score = 0.0
    if isinstance(clearance, (int, float)):
        score += max(0.0, 2.0 - float(clearance))
    if isinstance(progress, (int, float)):
        score += max(0.0, 100.0 - float(progress)) / 100.0
    return score


def _best_valid_round(history):
    best = None
    for round_ in history:
        for pair in round_.get("pairs", []):
            score = _pair_score(pair)
            if score is not None and (best is None or score > best[0]):
                best = score, round_, pair
    return best


def _confirmation(history):
    if not history:
        return None
    previous = history[-1]
    if previous["decision"].get("rule") == "confirm_failure":
        return None
    for pair in previous.get("pairs", []):
        if (pair["clean"]["outcome"] in SUCCESSES and
                pair["perturbed"]["outcome"] not in SUCCESSES and
                pair["perturbed"]["outcome"] != "infrastructure_error"):
            return dict(previous["decision"]["action"])
    return None


def _decision(round_number, policy, action, rule, reason, hypothesis):
    return {
        "schema_version": ACTION_SCHEMA_VERSION,
        "round": round_number,
        "policy": policy,
        "rule": rule,
        "reason": reason,
        "hypothesis": hypothesis,
        "action": validate_action(action),
    }


class RandomPolicy:
    name = "random"

    def __init__(self, seed, allowed_actions=None):
        self.seed = seed
        self.actions = list(allowed_actions or all_actions())

    def choose_next(self, history, remaining_pairs):
        confirm = _confirmation(history)
        if confirm is not None:
            return _decision(len(history) + 1, self.name, confirm, "confirm_failure",
                             "Repeat the same clean-pass/attack-fail condition once.",
                             "reproduce the candidate failure")
        tested = _tested(history)
        available = [a for a in self.actions if action_key(a) not in tested]
        if not available:
            raise RuntimeError("random policy exhausted the action space")
        rng = random.Random(self.seed + len(history) * 1009)
        action = available[rng.randrange(len(available))]
        return _decision(len(history) + 1, self.name, action, "random_untried",
                         "Sample an untried valid condition.", "cover the saved layout space")


class SearchPolicy:
    name = "search"

    def __init__(self, seed, allowed_actions=None):
        self.seed = seed
        self.actions = list(allowed_actions or all_actions())

    def _initial_action(self, available, round_number):
        # Broad, deterministic coverage of tier, seed, delay, and patch mode.
        ordered = sorted(available, key=action_key)
        stride = 131
        return ordered[(self.seed + (round_number - 1) * stride) % len(ordered)]

    def _rank(self, action, history):
        score = 0.0
        best = _best_valid_round(history)
        if best is None:
            return score
        previous = best[1]["decision"]["action"]
        # Local mutation around the most stressful valid condition, while still
        # rewarding a new layout/seed after a successful, comfortable run.
        if action["layout"] == previous["layout"]:
            score += 2.0
        if action["layout_seed"] == previous["layout_seed"]:
            score += 1.0
        if action["delay_s"] >= previous["delay_s"]:
            score += 0.8
        if action["patch_enabled"] and not previous["patch_enabled"]:
            score += 0.5
        if action["patch_enabled"] and previous["patch_enabled"] and action["patch_size_m"] >= previous["patch_size_m"]:
            score += 0.4
        if action["patch_start_s"] > 0:
            score += 0.1
        return score

    def choose_next(self, history, remaining_pairs):
        confirm = _confirmation(history)
        if confirm is not None:
            return _decision(len(history) + 1, self.name, confirm, "confirm_failure",
                             "Repeat the same clean-pass/attack-fail condition once.",
                             "reproduce the candidate failure")
        tested = _tested(history)
        available = [a for a in self.actions if action_key(a) not in tested]
        if not available:
            raise RuntimeError("search policy exhausted the action space")
        if not history:
            action = self._initial_action(available, 1)
            rule, reason = "initial_coverage", "Start with a valid condition from the saved layout catalogue."
        else:
            action = max(available, key=lambda x: (self._rank(x, history), action_key(x)))
            rule, reason = "guided_search", "Use clean/attack outcomes and stress metrics to choose an untried condition."
        return _decision(len(history) + 1, self.name, action, rule, reason,
                         "search around the most stressful valid result")


class OpenAICompatibleIntentProvider:
    """Small optional provider for an OpenAI-compatible chat-completions endpoint."""

    def __init__(self, endpoint=None, api_key=None, model=None, timeout_s=None, attempts=None):
        self.endpoint = endpoint or os.environ.get("WS2_AGENT_ENDPOINT")
        self.api_key = api_key or os.environ.get("WS2_AGENT_API_KEY")
        self.model = model or os.environ.get("WS2_AGENT_MODEL")
        self.timeout_s = timeout_s if timeout_s is not None else int(os.environ.get("WS2_AGENT_TIMEOUT_S", "30"))
        # Only the transport is retried. A malformed intent still fails the
        # campaign, because that is a result about the model, not a network blip.
        self.attempts = attempts if attempts is not None else int(os.environ.get("WS2_AGENT_ATTEMPTS", "1"))
        if self.attempts < 1:
            raise ValueError("agent provider attempts must be at least 1")
        if not self.endpoint or not self.api_key or not self.model:
            raise ValueError("agent mode requires WS2_AGENT_ENDPOINT, WS2_AGENT_API_KEY, and WS2_AGENT_MODEL")

    def propose(self, context):
        # The schema travels in the user message as data, and json_object mode only
        # enforces valid JSON, not the schema. Gemini 2.5 Pro overran maxLength on
        # 'reason' in 2 of 3 sampled replies, which aborts the campaign, so the
        # length limit is restated here as an instruction.
        system = (
            "You are a WS2 test strategist. Return JSON only. All generated text must be English. You cannot issue commands, "
            "control a drone, add noise, alter patch colors, or change planner code. "
            "Choose a high-level hypothesis using the supplied JSON schema. "
            "Hard limit: 'hypothesis' and 'reason' must each be at most 240 characters, "
            "including spaces. Aim for 150. One short sentence each. Replies that exceed "
            "240 characters are rejected and the test run is aborted."
        )
        if objective_is_stated():
            system += (
                " Your objective is to find conditions where the attacked flight fails "
                "while its clean control passes. Prefer settings that have not been tried "
                "and that are more severe than ones that already passed."
            )
        payload = {
            "model": self.model,
            "temperature": 0,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": json.dumps(
                    {"context": context,
                     "schema": intent_prompt_schema(context.get("allowed_layout_tiers"))})},
            ],
            "response_format": {"type": "json_object"},
        }
        request = urllib.request.Request(self.endpoint, data=json.dumps(payload).encode(), method="POST",
                                         headers={"Authorization": "Bearer " + self.api_key,
                                                  "Content-Type": "application/json"})
        last = None
        for attempt in range(self.attempts):
            try:
                with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                    body = json.load(response)
                break
            except (urllib.error.URLError, TimeoutError) as exc:
                last = exc
                if attempt + 1 < self.attempts:
                    time.sleep(2 ** attempt)
        else:
            raise RuntimeError(f"agent provider request failed after {self.attempts} attempts: {last}") from last
        try:
            content = body["choices"][0]["message"]["content"]
            return validate_intent(json.loads(content))
        except (KeyError, IndexError, TypeError, json.JSONDecodeError, ValueError) as exc:
            raise RuntimeError("agent provider returned invalid structured output") from exc


class AgentSearchPolicy(SearchPolicy):
    name = "agent_search"

    def __init__(self, seed, provider, allowed_actions=None):
        super().__init__(seed, allowed_actions)
        self.provider = provider
        self.intent_log = []
        self.campaign_context = {}

    @staticmethod
    def _filter_for_intent(actions, intent):
        delays = {
            "low": {0.0, 0.05},
            "medium": {0.05, 0.15},
            "high": {0.15, 0.25},
        }[intent["delay_band"]]
        selected = []
        for action in actions:
            if action["layout"] not in intent["layouts"] or action["delay_s"] not in delays:
                continue
            if intent["patch_mode"] == "disabled" and action["patch_enabled"]:
                continue
            if intent["patch_mode"] == "continuous" and (not action["patch_enabled"] or action["patch_start_s"] != 0):
                continue
            if intent["patch_mode"] == "timed" and (not action["patch_enabled"] or action["patch_start_s"] == 0):
                continue
            selected.append(action)
        return selected

    def choose_next(self, history, remaining_pairs):
        confirm = _confirmation(history)
        if confirm is not None:
            return _decision(len(history) + 1, self.name, confirm, "confirm_failure",
                             "Repeat the same clean-pass/attack-fail condition once.",
                             "reproduce the candidate failure")
        context = {
            **self.campaign_context,
            "intent_schema_version": INTENT_SCHEMA_VERSION,
            "remaining_pairs": remaining_pairs,
            "clean_qualified_layouts": sorted({f"{action['layout']}:{action['layout_seed']}"
                                                for action in self.actions}),
            "allowed_layout_tiers": sorted({action["layout"] for action in self.actions}),
            "history": [
                {"round": r["decision"].get("round"), "hypothesis": r["decision"].get("hypothesis"),
                 "action": r["decision"].get("action"), "pairs": r.get("pairs", [])}
                for r in history
            ],
        }
        if getattr(self.provider, "chooses_action", False):
            tested = _tested(history)
            available = {action_key(a): a for a in self.actions if action_key(a) not in tested}
            if not available:
                raise RuntimeError("agent exhausted the allowed action space")
            context["action_space"] = {key: sorted({a[key] for a in self.actions}) for key in
                                       ("layout", "layout_seed", "delay_s", "patch_size_m",
                                        "patch_start_s", "patch_duration_s")}
            context["available_action_keys"] = list(available)
            from claude_provider import validate_proposal
            proposal = validate_proposal(self.provider.propose(context))
            key = action_key(proposal["action"])
            if key not in available:
                raise ValueError("agent selected an unavailable or previously tested action")
            decision = _decision(len(history) + 1, self.name, available[key], "llm_selected_configuration",
                                 proposal["reason"], proposal["hypothesis"])
            decision["llm_call_id"] = getattr(self.provider, "last_call_id", None)
            return decision
        intent = validate_intent(self.provider.propose(context))
        self.intent_log.append(intent)
        tested = _tested(history)
        available = [a for a in self._filter_for_intent(self.actions, intent) if action_key(a) not in tested]
        if not available:
            tiers = sorted({a["layout"] for a in self.actions})
            raise RuntimeError(
                "agent intent has no untried valid actions "
                f"(intent layouts={intent['layouts']}, delay_band={intent['delay_band']}, "
                f"patch_mode={intent['patch_mode']}; qualified tiers={tiers})")
        action = max(available, key=lambda x: (self._rank(x, history), action_key(x)))
        decision = _decision(len(history) + 1, self.name, action, "agent_guided_search", intent["reason"], intent["hypothesis"])
        decision["intent"] = intent
        return decision


def objective_is_stated():
    """Whether the agent is told what the campaign is for. On unless disabled.

    Leaving it unstated is what the first agent campaign did, and the agent
    characterised the planner instead of attacking it: it held delay at the
    minimum for 10 of 10 rounds and varied only the patch, the factor measured
    to have no effect. Kept switchable so that arm stays reproducible.
    """
    return os.environ.get("WS2_AGENT_STATE_OBJECTIVE", "1") != "0"


def policy_from_name(name, seed, provider=None, allowed_actions=None):
    if name == "random":
        return RandomPolicy(seed, allowed_actions)
    if name == "search":
        return SearchPolicy(seed, allowed_actions)
    if name == "agent_search":
        return AgentSearchPolicy(seed, provider or provider_from_name(), allowed_actions)
    raise ValueError("policy must be random, search, or agent_search")
