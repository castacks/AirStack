import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import agent_campaign
from episode import fingerprint


class FakeProvider:
    def __init__(self):
        self.calls = 0

    def propose(self, context):
        self.calls += 1
        return {"hypothesis": "delayed patch after a turn", "reason": "test a timed patch",
                "layouts": ["easy"], "delay_band": "high", "patch_mode": "timed"}


def test_qualified_layout_parser_rejects_unverified_selector_forms():
    assert agent_campaign.parse_qualified_layouts(["easy:2", "easy:2"]) == (("easy", 2),)
    for value in ("easy", "stock:2", "hard:8"):
        try:
            agent_campaign.parse_qualified_layouts([value])
        except ValueError:
            pass
        else:
            raise AssertionError(f"expected invalid selector: {value}")


def test_agent_campaign_is_no_noise_and_resumes_without_another_model_call(tmp_path, monkeypatch):
    monkeypatch.setattr(agent_campaign, "RUNTIME", tmp_path)
    calls = []

    def fake_run(config, folder, **kwargs):
        folder.mkdir()
        calls.append(config)
        result = {
            "configuration_hash": fingerprint(config),
            "outcome": "goal_reached" if not config["condition"]["patch_enabled"] else "collision",
            "metrics": {"minimum_obstacle_clearance_m": .4, "mission_progress_percent": 100},
            "result_dir": str(folder),
        }
        (folder / "result.json").write_text(json.dumps(result))
        return result

    monkeypatch.setattr(agent_campaign, "run_episode", fake_run)
    provider = FakeProvider()
    root = tmp_path / "campaign"
    history = agent_campaign.run_adaptive_campaign(root, "agent_search", 4, 42, "kim",
                                                    retries=0, pause_seconds=0, provider=provider,
                                                    clean_validation_runs=0)
    assert len(history) == 2
    assert provider.calls == 1  # round two is the required confirmation
    assert len(calls) == 4
    assert all(c["condition"]["rgb_noise"] == 0 and c["condition"]["depth_noise"] == 0 for c in calls)
    assert {k: v for k, v in calls[1]["condition"].items() if k != "name"} == {
        k: v for k, v in calls[3]["condition"].items() if k != "name"
    }
    assert calls[1]["patch_start_s"] == calls[3]["patch_start_s"]
    assert calls[1]["patch_duration_s"] == calls[3]["patch_duration_s"]
    assert history[1]["decision"]["rule"] == "confirm_failure"

    agent_campaign.run_adaptive_campaign(root, "agent_search", 4, 42, "kim",
                                         retries=0, pause_seconds=0, provider=provider,
                                         clean_validation_runs=0)
    assert provider.calls == 1
    assert len(calls) == 4


def test_clean_failure_stops_before_the_attack_twin(tmp_path, monkeypatch):
    monkeypatch.setattr(agent_campaign, "RUNTIME", tmp_path)
    calls = []

    def fake_run(config, folder, **kwargs):
        folder.mkdir()
        calls.append(config)
        result = {
            "configuration_hash": fingerprint(config),
            "outcome": "collision",
            "metrics": {"minimum_obstacle_clearance_m": .01, "mission_progress_percent": 20},
            "result_dir": str(folder),
        }
        (folder / "result.json").write_text(json.dumps(result))
        return result

    monkeypatch.setattr(agent_campaign, "run_episode", fake_run)
    root = tmp_path / "campaign"
    history = agent_campaign.run_adaptive_campaign(root, "search", 2, 42, "mononav",
                                                    retries=0, pause_seconds=0,
                                                    clean_validation_runs=0)

    assert history == []
    assert len(calls) == 1
    assert json.loads((root / "presentation.json").read_text())["phase"] == "unstable_clean_baseline"
    assert (root / "round_01" / "mononav" / "clean_baseline_failure.json").exists()
