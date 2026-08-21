"""Campaign classification, fingerprint, baseline, and advisory contracts."""

import json
import sys
from types import SimpleNamespace

import pytest

from harness.baseline import select_baseline
from harness.image_prep import build_image_preparation
from harness.run_meta import build_run_meta, campaign_fingerprint
import parse_metrics
from parse_metrics import _score

pytestmark = pytest.mark.unit

NODE = "system/test_liveliness.py::TestLiveliness::test_sim_ready_time[isaacsim-1-iter0]"


def _report(when, outcome):
    return SimpleNamespace(
        nodeid=NODE,
        when=when,
        failed=outcome == "failed",
        skipped=outcome == "skipped",
        passed=outcome == "passed",
    )


def _item():
    return SimpleNamespace(nodeid=NODE)


def test_schema_v2_distinguishes_assertion_from_infrastructure():
    assertion = build_run_meta(
        [_item()],
        1,
        reports=[_report("setup", "passed"), _report("call", "failed")],
        campaign_config={"sim": "isaacsim", "num_robots": "1"},
    )
    infrastructure = build_run_meta(
        [_item()],
        1,
        reports=[_report("setup", "failed")],
        campaign_config={"sim": "isaacsim", "num_robots": "1"},
    )
    assert assertion["schema_version"] == 2
    assert assertion["complete"] is True
    assert assertion["failure_class"] == "assertion"
    assert infrastructure["complete"] is False
    assert infrastructure["failure_class"] == "infrastructure"


def test_behavior_options_participate_in_campaign_fingerprint():
    first = campaign_fingerprint([NODE], {"sim": "isaacsim", "num_robots": "1"})
    second = campaign_fingerprint([NODE], {"sim": "isaacsim", "num_robots": "3"})
    assert first != second


def test_assertion_with_downstream_dependency_skip_is_finalized_campaign():
    downstream = NODE.replace("test_sim_ready_time", "test_stable")
    skipped = SimpleNamespace(
        nodeid=downstream,
        when="setup",
        failed=False,
        skipped=True,
        passed=False,
    )
    meta = build_run_meta(
        [_item(), SimpleNamespace(nodeid=downstream)],
        1,
        reports=[_report("call", "failed"), skipped],
        campaign_config={"sim": "isaacsim", "num_robots": "1"},
    )
    assert meta["outcome"] == "simulation"
    assert meta["failure_class"] == "assertion"
    assert meta["simulation_completed"] == 1
    assert meta["simulation_finalized"] == 2


def test_call_phase_infrastructure_failure_is_not_an_algorithm_assertion():
    report = _report("call", "failed")
    report.airstack_failure_class = "infrastructure"
    meta = build_run_meta(
        [_item()],
        1,
        reports=[report],
        campaign_config={"sim": "msairsim", "num_robots": "1"},
    )
    assert meta["outcome"] == "incomplete"
    assert meta["failure_class"] == "infrastructure"
    assert meta["complete"] is False


def _write_run(path, fingerprint, complete=True):
    path.mkdir()
    (path / "results.xml").write_text("<testsuites/>")
    (path / "run_meta.json").write_text(json.dumps({
        "schema_version": 2,
        "complete": complete,
        "completion_state": "completed" if complete else "interrupted",
        "outcome": "simulation" if complete else "incomplete",
        "campaign_fingerprint": fingerprint,
    }))


def test_baseline_selector_ignores_newer_mismatch_and_partial(tmp_path):
    matching = tmp_path / "matching"
    mismatch = tmp_path / "mismatch"
    partial = tmp_path / "partial"
    _write_run(matching, "wanted")
    _write_run(mismatch, "other")
    _write_run(partial, "wanted", complete=False)
    selected, rejected = select_baseline(
        [mismatch, partial, matching],
        {"complete": True, "outcome": "simulation", "campaign_fingerprint": "wanted"},
    )
    assert selected == matching
    assert len(rejected) == 2


def test_timeout_or_missing_data_is_never_numeric_regression():
    numeric = {"value": 1.0, "direction": "lower_is_better"}
    assert _score({"value": "timeout"}, numeric, 20)[1] == ""
    assert _score(None, numeric, 20)[1] == ""


@pytest.mark.parametrize(
    "outcome,field",
    [
        ("already-present", None),
        ("pulled-versioned", "versioned_pulled"),
        ("cache-retagged", "cache_retagged"),
        ("locally-built", "locally_built"),
        ("missing", "missing"),
    ],
)
def test_image_preparation_paths_have_explicit_outcomes(outcome, field):
    kwargs = {}
    if field:
        argument = {
            "versioned_pulled": "pulled",
            "cache_retagged": "retagged",
            "locally_built": "built",
            "missing": "missing",
        }[field]
        kwargs[argument] = ["registry/image:tag"]
    payload = build_image_preparation(outcome, **kwargs)
    assert payload["outcome"] == outcome
    if field:
        assert payload[field] == ["registry/image:tag"]


def test_metric_delta_cli_is_advisory(monkeypatch, tmp_path):
    output = tmp_path / "report.md"
    monkeypatch.setattr(
        parse_metrics,
        "generate_report",
        lambda *args, **kwargs: ("advisory", True),
    )
    monkeypatch.setattr(
        sys,
        "argv",
        ["parse_metrics.py", "--current", str(tmp_path), "--output", str(output)],
    )
    with pytest.raises(SystemExit) as exc:
        parse_metrics.main()
    assert exc.value.code == 0
    assert output.read_text() == "advisory"


def test_report_parser_crash_remains_blocking(monkeypatch, tmp_path):
    output = tmp_path / "report.md"

    def crash(*args, **kwargs):
        raise RuntimeError("broken parser")

    monkeypatch.setattr(parse_metrics, "generate_report", crash)
    monkeypatch.setattr(
        sys,
        "argv",
        ["parse_metrics.py", "--current", str(tmp_path), "--output", str(output)],
    )
    with pytest.raises(SystemExit) as exc:
        parse_metrics.main()
    assert exc.value.code == 2
    assert "Report generation failed" in output.read_text()
