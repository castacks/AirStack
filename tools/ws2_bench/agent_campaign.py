"""Run no-noise WS2 random, search, or agent-guided adaptive campaigns."""
from __future__ import annotations

import argparse
import csv
import fcntl
import json
import math
from pathlib import Path
import time

from agent_policies import all_actions, objective_is_stated, policy_from_name, provider_from_name
from agent_schema import LAYOUTS, ACTION_SCHEMA_VERSION, action_to_episode, validate_action
from campaign import clean_twin, verdict
from conditions import PATCH_POLICY
from episode import RUNTIME, atomic, fingerprint, resolved, run_episode
from feedback import summary
from mission import SUCCESSES, defaults
from operator_control import UserStop, wait_between_trials
from vulnerability_report import write_report, report_rows
from model_adapters import REGISTRY, get_adapter


BACKEND_LABELS = {
    "random": "Seeded random sampling (no noise)",
    "search": "Deterministic result-guided search (no noise)",
    "agent_search": "Agent-guided testing (no noise)",
}


def parse_qualified_layouts(values):
    """Parse explicit clean-qualified layout selectors such as ``easy:2``."""
    qualified = set()
    for value in values:
        try:
            layout, seed_text = value.split(":", 1)
            seed = int(seed_text)
        except (AttributeError, ValueError) as exc:
            raise ValueError("qualified layouts must use layout:seed, for example easy:2") from exc
        # Validate against the agent's own action space, not a second
        # hardcoded copy of it. The two drifted apart and cost a campaign.
        if layout not in LAYOUTS or not 0 <= seed <= 7:
            raise ValueError("qualified layouts must use %s and seed 0..7"
                             % "|".join(LAYOUTS))
        qualified.add((layout, seed))
    if not qualified:
        raise ValueError("at least one clean-qualified layout is required")
    return tuple(sorted(qualified))


def _config(policy_name, budget, seed, planner, retries, record_bags, timeout, goal_distance,
            qualified_layouts, clean_validation_runs, clean_failure_policy):
    mission = defaults(planner)
    if timeout is not None:
        mission["timeout"] = timeout
    if goal_distance is not None:
        mission["goal_distance"] = goal_distance
    return {
        "backend": policy_name,
        "label": BACKEND_LABELS[policy_name],
        "action_schema": ACTION_SCHEMA_VERSION,
        "profile": "delay_patch",
        "noise": "disabled",
        "budget": budget,
        "seed": seed,
        "planners": [planner],
        "retries": retries,
        "record_bags": record_bags,
        "mission": mission,
        "patch_policy": PATCH_POLICY,
        "clean_qualified_layouts": [f"{layout}:{layout_seed}" for layout, layout_seed in qualified_layouts],
        "clean_validation_runs": clean_validation_runs,
        "clean_failure_policy": clean_failure_policy,
        # Recorded because two agent campaigns can otherwise differ only by an
        # environment variable, leaving no way to tell them apart afterwards.
        "agent_objective_stated": objective_is_stated() if policy_name == "agent_search" else None,
    }


def _candidate(decision, config, index):
    if config.get('action_space')=='expanded':
        from expanded_space import action_to_episode as expanded_episode
        raw=expanded_episode(decision['action'],config['planners'][0],config['seed'],
            f"{config['backend']} round {index+1} {config['planners'][0]}",config['mission'],
            placement=decision.get('condition',{}).get('placement'))
        candidate=resolved(raw);decision=dict(decision)
        decision.update(condition=candidate['condition'],patch_start_s=candidate['patch_start_s'],patch_duration_s=candidate['patch_duration_s'])
        return decision,candidate
    action = validate_action(decision["action"])
    raw = action_to_episode(action, config["planners"][0], config["seed"],
                            f"{config['backend']} round {index + 1} {config['planners'][0]}", config["mission"])
    candidate = resolved(raw)
    decision = dict(decision)
    decision.update(condition=candidate["condition"], patch_start_s=candidate["patch_start_s"],
                    patch_duration_s=candidate["patch_duration_s"])
    return decision, candidate


def _load_json(path):
    return json.loads(path.read_text())


def run_adaptive_campaign(output, policy_name, budget=8, seed=42, planner="mononav", retries=1,
                          record_bags=False, pause_seconds=5, timeout=None, goal_distance=None,
                          provider=None, qualified_layouts=(("easy", 2),), clean_validation_runs=2,
                          clean_failure_policy="halt", provider_name=None, llm_report=True, action_space="saved"):
    """Run one paired campaign after repeated pristine-clean validation.

    The scheduled budget counts matched clean/attack flights. Validation and
    infrastructure retries are tracked separately as actual attempts.
    """
    if policy_name not in BACKEND_LABELS:
        raise ValueError("policy must be random, search, or agent_search")
    adapter=get_adapter(planner)
    if budget < 2 or budget % 2:
        raise ValueError("budget must be even: one clean/attack pair needs two flights")
    if retries not in (0, 1, 2):
        raise ValueError("retries must be 0, 1, or 2")
    if clean_validation_runs not in (0, 1, 2, 3):
        raise ValueError("clean_validation_runs must be 0, 1, 2, or 3")
    if clean_failure_policy not in ("halt", "record"):
        raise ValueError("clean_failure_policy must be 'halt' or 'record'")
    if not 0 <= pause_seconds <= 60 or not math.isfinite(pause_seconds):
        raise ValueError("pause_seconds must be 0..60")

    if action_space not in ('saved','expanded'):raise ValueError('action_space must be saved or expanded')
    if action_space=='saved':
        qualified_layouts = parse_qualified_layouts([f"{layout}:{layout_seed}" for layout, layout_seed in qualified_layouts])
        allowed_actions = [action for action in all_actions() if (action["layout"], action["layout_seed"]) in qualified_layouts]
        if 'fcrn_patch' not in adapter.attacks:
            allowed_actions=[action for action in allowed_actions if not action['patch_enabled']]
        validate_selected=validate_action
    else:
        from expanded_space import validate_action as validate_selected
        qualified_layouts=();allowed_actions=None
    root = Path(output).resolve()
    root.relative_to(RUNTIME.resolve())
    root.mkdir(parents=True, exist_ok=True)
    if policy_name == 'agent_search' and provider is None:
        provider = provider_from_name(provider_name, audit_dir=root/'llm_calls')
    if provider is not None and hasattr(provider, 'check_auth'):
        provider.check_auth()  # fail before any simulator or validation flight
    config = _config(policy_name, budget, seed, planner, retries, record_bags, timeout, goal_distance,
                     qualified_layouts, clean_validation_runs, clean_failure_policy)
    if action_space=='expanded':
        from expanded_space import bounds, VERSION
        config.update(action_space='expanded',action_schema=VERSION,profile='expanded',noise='configurable',expanded_bounds=bounds(),
                      label={'random':'Seeded random sampling','search':'Result-guided search','agent_search':'Claude-guided testing'}[policy_name])
        if planner=='mononav' and config['mission']['goal_distance']!=8:
            raise ValueError('Generated placement currently protects the qualified 8m mission; use goal_distance=8')
    if provider is not None and hasattr(provider, 'metadata'):
        config['llm_provider'] = provider.metadata()
        config['llm_report'] = bool(llm_report)
    resolved({"planner": planner, **config["mission"]})
    config_path = root / "config.json"
    if config_path.exists():
        previous = _load_json(config_path)
        # Extending the budget is the normal way to add rounds to a finished
        # campaign; every flight already on disk stays valid because nothing
        # else about the configuration moved. Shrinking it is not allowed,
        # because the recorded rounds would no longer fit the campaign.
        if {k: v for k, v in previous.items() if k != "budget"} !=            {k: v for k, v in config.items() if k != "budget"}:
            raise ValueError("resume configuration changed")
        if config["budget"] < previous["budget"]:
            raise ValueError(
                f"resume budget {config['budget']} is below the recorded "
                f"{previous['budget']}; only increasing the budget is allowed")
    atomic(config_path, config)

    global_lock = (RUNTIME / "adaptive.lock").open("w")
    fcntl.flock(global_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    lock = (root / "campaign.lock").open("w")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    control_path = root / "operator_control.json"
    atomic(control_path, {"action": "run"})
    history_path = root / "history.json"
    history = _load_json(history_path) if history_path.exists() else []
    if len(history) > budget // 2:
        raise ValueError("saved history exceeds the configured budget")
    if action_space=='expanded':
        from expanded_space import ExpandedPolicy
        if policy_name=='agent_search' and not getattr(provider,'chooses_action',False):
            raise ValueError('Expanded agent mode requires the exact-action Claude provider')
        policy=ExpandedPolicy(policy_name,seed,provider)
    else:policy = policy_from_name(policy_name, seed, provider, allowed_actions)
    if hasattr(policy, 'campaign_context'):
        policy.campaign_context = {'target_planner': planner, 'mission': config['mission'],
                                   'model_adapter':adapter.describe(),
                                   'metric_notes': 'clearance is a nominal envelope margin; collision is simulator contact'}
    completed = len(history) * 2
    actual_attempts = sum(p.is_dir() for pattern in ('round_*/*/*/attempt_*','clean_validation/attempt_*')
                          for p in root.glob(pattern))
    live_rows = []

    def publish(phase, decision=None, trial=None):
        value = {
            "backend": config['label'],
            "target_planner": planner,
            "phase": phase,
            "budget": budget,
            "completed": completed,
            "actual_attempts": actual_attempts,
            "decision": decision,
            "trial": trial,
            "history": summary(history, live_rows),
            "rows": report_rows(history, live_rows),
            "output": str(root),
            "wall_time": time.time(),
            "mission": config["mission"],
            "profile": config['profile'],
            "action_space": action_space,
        }
        if decision and decision.get('condition'):
            from layout_summary import describe
            value['layout_description']=describe(decision['condition'])
        atomic(root / "presentation.json", value)
        atomic(RUNTIME / "live_campaign.json", value)

    def finish(phase, decision=None, interpret=True):
        report = write_report(root, history, live_rows, config, phase)
        if (interpret and llm_report and provider is not None
                and getattr(provider, 'chooses_action', False)):
            from agent_analysis import write_analysis
            publish('analyzing_results', decision)
            write_analysis(root, report, provider)
        table = summary(history, live_rows)
        atomic(root / "report.json", dict(table, analysis=report, actual_attempts=actual_attempts))
        if table["rows"]:
            with (root / "trials.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(table["rows"][0]))
                writer.writeheader()
                writer.writerows(table["rows"])
        publish(phase, decision)
        lock.close()
        global_lock.close()
        return history

    def validate_clean_baseline():
        """Require repeated pristine clean successes before spending attack budget."""
        nonlocal actual_attempts
        if not clean_validation_runs:
            return True
        validation_root = root / "clean_validation"
        validation_path = validation_root / "validation.json"
        saved = _load_json(validation_path) if validation_path.exists() else []
        if len(saved) > clean_validation_runs:
            raise ValueError("saved clean validation exceeds configured runs")
        if action_space=='expanded':
            from expanded_space import reference_action,action_to_episode as expanded_episode
            base=resolved(expanded_episode(reference_action(),planner,seed,'pristine clean validation',config['mission']))
        else:
            action = dict(allowed_actions[0])
            action.update(delay_s=0.0, patch_enabled=False, patch_start_s=0.0, patch_duration_s=0.0)
            base = resolved(action_to_episode(action, planner, seed, "pristine clean validation", config["mission"]))
        for index in range(len(saved), clean_validation_runs):
            folder = validation_root / f"attempt_{index}"
            if (folder / "result.json").exists():
                result = _load_json(folder / "result.json")
                if result["configuration_hash"] != fingerprint(base):
                    raise ValueError("saved clean validation config changed")
            elif folder.exists():
                result = {"outcome": "infrastructure_error", "metrics": {}, "result_dir": str(folder),
                          "termination": {"reason": "interrupted validation attempt"}}
            else:
                publish("validating_clean_baseline", trial={"planner": planner, "role": "clean_validation",
                                                             "attempt": index + 1, "directory": str(folder)})
                result = run_episode(base, folder, record_bag=record_bags, control_path=control_path)
                actual_attempts += 1
            saved.append(result)
            validation_root.mkdir(parents=True, exist_ok=True)
            atomic(validation_path, saved)
            if result["outcome"] not in SUCCESSES:
                atomic(root / "clean_baseline_failure.json", {"phase": "clean_validation_failed",
                                                               "result": result, "attempt": index + 1,
                                                               "policy": clean_failure_policy})
                if clean_failure_policy == "halt":
                    return False
        return True

    try:
        if not validate_clean_baseline():
            return finish("clean_validation_failed")
        for index in range(len(history), budget // 2):
            round_root = root / f"round_{index + 1:02d}"
            round_root.mkdir(exist_ok=True)
            decision_path = round_root / "decision.json"
            if decision_path.exists():
                decision = _load_json(decision_path)
                decision["action"] = validate_selected(decision["action"])
            else:
                decision = policy.choose_next(history, budget // 2 - len(history))
                decision["action"] = validate_selected(decision["action"])
            decision, candidate = _candidate(decision, config, index)
            if decision_path.exists() and _load_json(decision_path) != decision:
                raise ValueError("saved decision differs from its resolved action")
            if not decision_path.exists():
                atomic(decision_path, decision)
            try:
                wait_between_trials(control_path, lambda phase: publish(phase, decision))
            except UserStop:
                return finish("stopped", decision)
            publish("selecting_configuration", decision)
            pair = {"planner": planner}
            for role, trial in (("clean", clean_twin(candidate)), ("perturbed", candidate)):
                try:
                    wait_between_trials(control_path, lambda phase: publish(phase, decision))
                except UserStop:
                    return finish("stopped", decision)
                attempts = []
                for attempt in range(retries + 1):
                    folder = round_root / planner / role / f"attempt_{attempt}"
                    publish("running", decision, {"planner": planner, "role": role, "round": index + 1,
                                                  "condition": trial["condition"], "directory": str(folder)})
                    if (folder / "result.json").exists():
                        result = _load_json(folder / "result.json")
                        if result["configuration_hash"] != fingerprint(trial):
                            raise ValueError("saved trial config changed")
                    elif folder.exists():
                        result = {"outcome": "infrastructure_error", "metrics": {}, "result_dir": str(folder),
                                  "termination": {"reason": "interrupted attempt"}}
                    else:
                        folder.parent.mkdir(parents=True, exist_ok=True)
                        result = run_episode(trial, folder, record_bag=record_bags, control_path=control_path)
                        actual_attempts += 1
                    attempts.append({"directory": str(folder), "outcome": result["outcome"]})
                    if result["outcome"] != "infrastructure_error":
                        break
                pair[role] = dict(result, attempts=attempts)
                completed += 1
                live_rows.append({"round": index + 1, "planner": planner, "role": role,
                                  "outcome": result["outcome"], **result["metrics"]})
                atomic(round_root / planner / (role + ".json"), pair[role])
                if result["outcome"] == "user_stopped":
                    return finish("stopped", decision)
                if role == "clean" and result["outcome"] not in SUCCESSES:
                    halting = clean_failure_policy == "halt"
                    atomic(round_root / planner / "clean_baseline_failure.json", {
                        "phase": "unstable_clean_baseline", "decision": decision, "result": pair["clean"],
                        "policy": clean_failure_policy,
                        "message": ("Attack flight was not run because its pristine clean control failed."
                                    if halting else
                                    "Clean control failed; the attack flight still ran so the round "
                                    "contributes to the measured failure rate. verdict() marks the pair "
                                    "invalid_clean_baseline so it cannot count as a candidate."),
                    })
                    if halting:
                        return finish("unstable_clean_baseline", decision)
                publish("trial_complete", decision, {"planner": planner, "role": role, "result": result})
                if pause_seconds:
                    time.sleep(pause_seconds)
            pair["verdict"] = verdict(pair["clean"], pair["perturbed"])
            history.append({"decision": decision, "pairs": [pair]})
            atomic(history_path, history)
            write_report(root, history, live_rows, config, "running")
            publish("complete" if completed == budget else "reviewing_results", decision)
        return finish("complete", history[-1]["decision"] if history else None)
    except Exception as exc:
        atomic(root/'campaign_error.json', {'error': str(exc), 'completed': completed})
        try:
            finish('error', interpret=False)
        finally:
            lock.close()
            global_lock.close()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--policy", choices=sorted(BACKEND_LABELS), default="search")
    parser.add_argument("--provider", choices=("claude", "openai"), default=None,
                        help="claude: official subscription CLI; openai: Gemini-compatible API endpoint")
    parser.add_argument("--no-llm-report", action="store_true", help="skip final Claude interpretation")
    parser.add_argument("--budget", type=int, default=8)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--planner", choices=("mononav", "kim"), required=True)
    parser.add_argument("--infrastructure-retries", choices=(0, 1, 2), type=int, default=1)
    parser.add_argument("--record-bags", action="store_true")
    parser.add_argument("--review-seconds", type=float, default=5)
    parser.add_argument("--timeout", type=float)
    parser.add_argument("--goal-distance", type=float)
    parser.add_argument("--clean-failure-policy", choices=("halt", "record"), default="halt",
                        help="halt: stop the campaign the first time a clean control fails (default). "
                             "record: keep flying and score on measured rates, for use when the clean "
                             "baseline is known to fail at a non-trivial rate.")
    parser.add_argument("--clean-validation-runs", choices=(0, 1, 2, 3), type=int, default=2,
                        help="pristine clean successes required before the paired flight budget (default: 2)")
    parser.add_argument('--action-space',choices=('saved','expanded'),default='saved')
    parser.add_argument("--qualified-layout", action="append", metavar="LAYOUT:SEED",
                        help="a layout that has already passed a clean flight, e.g. easy:2 (repeatable)")
    args = parser.parse_args()
    run_adaptive_campaign(args.output, args.policy, args.budget, args.seed, args.planner,
                          args.infrastructure_retries, args.record_bags, args.review_seconds,
                          args.timeout, args.goal_distance,
                          qualified_layouts=parse_qualified_layouts(args.qualified_layout or []) if args.action_space=='saved' else (),
                          clean_validation_runs=args.clean_validation_runs,
                           clean_failure_policy=args.clean_failure_policy,
                           provider_name=args.provider, llm_report=not args.no_llm_report,action_space=args.action_space)


if __name__ == "__main__":
    main()
