from pathlib import Path
import importlib.util


_SCRIPT = Path(__file__).with_name("make_rerun_mission.py")
_SPEC = importlib.util.spec_from_file_location("make_rerun_mission", _SCRIPT)
_MOD = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MOD)


def test_failures_from_four_cell_mission_log(tmp_path):
    log = tmp_path / "mission.log"
    log.write_text(
        "[mission] iteration 1/4\n"
        "[mission] environment: eq_l1_frontier\n"
        "[mission] iteration 1 failed on attempt 2/2\n"
        "[mission] iteration 2/4\n"
        "[mission] environment: eq_l1_lawnmower\n"
        "[mission] ERROR: iteration aborted: takeoff\n"
        "[mission] step 17: OK\n"
        "[mission] iteration 3/4\n"
        "[mission] environment: eq_l1_vlfm\n"
        "[mission] iteration 3 failed on attempt 2/2\n"
    )
    assert _MOD.failures_from_log(log) == ["eq_l1_frontier", "eq_l1_vlfm"]
