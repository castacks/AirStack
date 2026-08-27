#!/usr/bin/env python3
"""
test_people_rules.py — the two POSTURE RULES, pinned without Isaac.

  1. NOBODY WAVES, in any scene. The pose is gone from the table, every
     chooser, the bench scripts and the checker, and the two doors a pose
     goes through (`scene_generator._bind_human_pose`, `people.add_person`)
     refuse it rather than letting a stale caller through.
  2. IN AN UNDAMAGED SCENE NOBODY SITS OR LIES ON THE GROUND. Seated only on
     a seat the caller supplies — a car (`in_vehicle`) or a bench (`seat=`).
     Disaster scenes keep the ground postures.

RUNS WITHOUT ISAAC. `disaster/people.py` imports only the standard library;
`Ground` (which pulls in `suburb_scene` -> `pxr`) and `_pose_dz` (which pulls
in `scene_generator`) are stubbed, because neither is what is under test.
`scene_generator.py` is read as SOURCE and its pose table and `_check_pose`
are sliced out and exec'd, so the check runs the repo's real code.

USAGE
    python3 scene_gen/tests/test_people_rules.py
    pytest -s scene_gen/tests/test_people_rules.py
"""

import ast
import os
import random
import re
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
_REPO = os.path.dirname(_SCENE_GEN)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import people as ppl          # noqa: E402

_SG_PY = os.path.join(_SCENE_GEN, "scene_generator.py")
with open(_SG_PY, encoding="utf-8") as _f:
    SG_SRC = _f.read()

# Every file a pose name is chosen or listed in.
_POSE_FILES = [
    os.path.join(_SCENE_GEN, "disaster", "people.py"),
    os.path.join(_SCENE_GEN, "disaster", "tornado_people.py"),
    os.path.join(_SCENE_GEN, "scene_generator.py"),
    os.path.join(_SCENE_GEN, "tools", "pose_check.py"),
    os.path.join(_REPO, "simulation", "isaac-sim", "launch_scripts",
                 "people_showcase_launch_script.py"),
    os.path.join(_REPO, "simulation", "isaac-sim", "launch_scripts",
                 "tornado_people_preview_launch_script.py"),
]


# ── stubs ────────────────────────────────────────────────────────────────────

class _Pools:
    def scale_of(self, usd): return 0.01
    def axis_of(self, usd): return "z"
    def yaw_of(self, usd): return 90.0
    def roll_of(self, usd): return 0.0


class _Resolver:
    def get(self, usd, cat, **kw):
        return {"sz": 1.8, "sy": 0.35, "sx": 0.6, "base": 0.0}


class _FreeGround:
    """Everything is free; nothing is a road, a house or a car."""
    def __init__(self, *a, **k): pass
    def free(self, *a, **k): return True
    def take(self, x, y): return None
    def house_dist(self, x, y): return 1e9
    def solid_dist(self, x, y): return 1e9
    def on_road(self, x, y, margin=0.0): return False
    def car_hit(self, box, pad=0.0): return False
    def add_cars(self, boxes): return None
    def add_solids(self, boxes): return None


ppl.Ground = _FreeGround
ppl._pose_dz = lambda usd, pose, height: 0.0


def make_plan(peacetime, seed=1, **ctx_extra):
    cfg = ppl.resolve_cfg({}, has_disaster=not peacetime)
    ctx = {
        "humans": ["rp_carla_rigged_001_ue4.usd", "rp_eric_rigged_001_ue4.usd"],
        "humans_posed": (),
        "asset_pools": _Pools(), "resolver": _Resolver(),
        "houses": [], "cars": [], "placements": [], "trees": [],
        "age": (lambda x, y: -1.0) if peacetime else (lambda x, y: 100.0),
        "span": 100.0, "elapsed": 300.0,
        "car_pool": {"residential": []}, "glassy": frozenset(),
        "blocks": [{"bulbs": [{"c": [0.0, 0.0], "r_pave": 12.0}]}],
    }
    ctx.update(ctx_extra)
    return ppl._Plan(ctx, cfg, random.Random(seed))


HOUSE = {"index": 0, "x": 0.0, "y": 0.0, "n": (0.0, 1.0), "u": (1.0, 0.0),
         "d": 10.0, "w": 12.0, "yaw_box": 0.0}


def poses_of(plan):
    return [r["pose"] for r in plan.records]


# ── 1. no wave, anywhere ─────────────────────────────────────────────────────

def _code_lines(path):
    with open(path, encoding="utf-8") as fh:
        for n, line in enumerate(fh, 1):
            code = line.split("#", 1)[0]
            if code.strip():
                yield n, code


def test_01_no_wave_pose_anywhere():
    """No file chooses, lists or authors a `wave`; the pose table has none;
    both doors refuse it."""
    hits = []
    for path in _POSE_FILES:
        for n, code in _code_lines(path):
            # the two places that are allowed to say the word: the ban itself
            # and the error message that enforces it
            if "BANNED_POSES = " in code or "deliberately no" in code:
                continue
            if re.search(r"""["']wave["']""", code) or "wave_share" in code:
                hits.append("%s:%d" % (os.path.relpath(path, _REPO), n))
    assert not hits, "wave still referenced: %s" % hits

    tree = ast.parse(SG_SRC)
    tables = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and isinstance(node.targets[0], ast.Name) \
                and node.targets[0].id in ("_HUMAN_POSES", "_POSE_Z_OFFSET"):
            keys = [k.value for k in node.value.keys if isinstance(k, ast.Constant)]
            tables[node.targets[0].id] = keys
    assert "wave" not in tables["_HUMAN_POSES"], tables["_HUMAN_POSES"]
    assert "wave" not in tables["_POSE_Z_OFFSET"], tables["_POSE_Z_OFFSET"]
    assert {"idle", "walk", "sit_ground", "sit_edge", "crouch"} <= set(tables["_HUMAN_POSES"])

    # the binder and the z-offset both go through _check_pose
    i = SG_SRC.index("def _bind_human_pose(")
    j = SG_SRC.index("\ndef ", i + 10)
    assert "_check_pose(pose)" in SG_SRC[i:j], "_bind_human_pose does not check the pose"
    i = SG_SRC.index("def pose_z_offset(")
    j = SG_SRC.index("\n_POSE_Z_OFFSET", i)
    assert "_check_pose(pose)" in SG_SRC[i:j], "pose_z_offset does not check the pose"
    assert ppl.BANNED_POSES == ("wave",)
    print("    no 'wave' in %d files; pose tables %s" % (
        len(_POSE_FILES), sorted(tables["_HUMAN_POSES"])))


def test_02_the_real_check_pose_refuses_unknown_names():
    """Sliced from scene_generator: an unknown pose name raises instead of
    leaving the rig in its A-pose. None / '' still mean 'leave it alone'."""
    i = SG_SRC.index("def _check_pose(")
    j = SG_SRC.index("\ndef _bind_human_pose(", i)
    ns = {"_HUMAN_POSES": {"idle": {}, "walk": {}, "sit_ground": {}}}
    exec(compile(SG_SRC[i:j], "<scene_generator:_check_pose>", "exec"), ns)
    check = ns["_check_pose"]
    for ok in ("idle", "walk", "sit_ground", None, ""):
        check(ok)
    for bad in ("wave", "Idle", "sit-ground", "kneel"):
        raised = False
        try:
            check(bad)
        except ValueError as exc:
            raised = True
            assert repr(bad) in str(exc)
        assert raised, "%r accepted" % bad
    print("    idle/walk/sit_ground/None pass; wave/Idle/sit-ground/kneel raise")


def test_03_add_person_raises_on_wave_in_every_scene():
    for peacetime in (True, False):
        plan = make_plan(peacetime)
        raised = False
        try:
            plan.add_person("at_home", 0, 1.0, 1.0, 0.0, 0.0, "wave")
        except ValueError as exc:
            raised = True
            assert "banned" in str(exc)
        assert raised
        assert plan.records == []
    print("    add_person('wave') raises in peacetime and in disaster")


# ── 2. no ground sitting / prone in peacetime ────────────────────────────────

def test_04_peacetime_coerces_ground_postures_to_standing():
    plan = make_plan(True)
    plan.add_person("open_ground", 0, 1.0, 1.0, 0.0, 0.0, "sit_ground")
    plan.add_person("at_home", 0, 2.0, 2.0, 0.18, 0.0, "sit_edge")
    plan.add_person("cul_de_sac", 0, 3.0, 3.0, 0.0, 0.0, "crouch")
    plan.add_person("at_home", 0, 4.0, 4.0, 0.0, 0.0, "idle", prone=True)
    plan.add_person("at_home", 0, 5.0, 5.0, 0.0, 0.0, "idle")
    assert poses_of(plan) == ["idle"] * 5, poses_of(plan)
    assert all(r["alive"] for r in plan.records)
    # standing: at ground level, no roll — the prone request is upright too
    for q in plan.humans:
        assert q["pose"] == "idle"
        assert abs(q["z_m"]) < 1e-9, q
        assert abs(q["roll_deg"]) < 1e-9, q
    assert plan.coerced == {"sit_ground": 1, "sit_edge": 1, "crouch": 1,
                            "prone": 1}, plan.coerced
    # the same calls in a disaster scene keep their postures
    d = make_plan(False)
    d.add_person("open_ground", 0, 1.0, 1.0, 0.0, 0.0, "sit_ground")
    d.add_person("at_home", 0, 2.0, 2.0, 0.18, 0.0, "sit_edge")
    d.add_person("cul_de_sac", 0, 3.0, 3.0, 0.0, 0.0, "crouch")
    d.add_person("at_home", 0, 4.0, 4.0, 0.0, 0.0, "idle", prone=True)
    assert poses_of(d) == ["sit_ground", "sit_edge", "crouch", "idle"]
    assert abs(abs(d.humans[3]["roll_deg"]) - 90.0) < 1e-9, "prone lost its roll"
    assert abs(d.humans[3]["z_m"] - 0.175) < 1e-9
    assert d.coerced == {}
    print("    peacetime: sit_ground/sit_edge/crouch/prone -> standing idle, "
          "counted %s; disaster: unchanged" % plan.coerced)


def test_05_seats_are_allowed_in_peacetime():
    """Car occupants and bench sitters are seated ON something; they stay."""
    plan = make_plan(True)
    plan.add_person("cul_de_sac", 0, 1.0, 1.0, 0.45, 0.0, "seated_car_arms_down",
                    in_vehicle="car_0")
    plan.add_person("cul_de_sac", 0, 2.0, 2.0, 0.45, 0.0, "seated_car",
                    in_vehicle="car_0")
    plan.add_person("at_home", 0, 3.0, 3.0, 0.45, 0.0, "sit_edge", seat="bench")
    assert poses_of(plan) == ["seated_car_arms_down", "seated_car", "sit_edge"]
    # seat height kept, plus the per-rig seated correction (male rigs -0.15)
    want = 0.45 + ppl._seated_asset_dz(plan.humans[2]["usd"], "sit_edge")
    assert abs(plan.humans[2]["z_m"] - want) < 1e-9, (plan.humans[2]["z_m"], want)
    assert plan.coerced == {}
    # ... but a sit_edge with no seat named is a ground sit
    plan.add_person("at_home", 0, 4.0, 4.0, 0.18, 0.0, "sit_edge")
    assert poses_of(plan)[-1] == "idle"
    assert plan.humans[-1]["z_m"] == 0.0, "coerced sitter kept the step height"
    print("    seated_car(_arms_down) with in_vehicle and sit_edge with "
          "seat='bench' kept; bare sit_edge -> stander at z=0")


def test_06_front_yard_never_sits_in_peacetime():
    for seed in range(12):
        plan = make_plan(True, seed=seed, houses=[HOUSE])
        ppl._front_yard(plan, 0, HOUSE, None, 3)
        assert len(plan.records) == 3, poses_of(plan)
        assert set(poses_of(plan)) == {"idle"}, (seed, poses_of(plan))
        assert plan.coerced == {}, "a scenario asked for a ground posture"
        assert not any("step" in (r.get("note") or "") for r in plan.records)
    # the disaster front yard still has its step sitter
    plan = make_plan(False, seed=3, houses=[HOUSE])
    ppl._front_yard(plan, 0, HOUSE, None, 3)
    assert "sit_edge" in poses_of(plan), poses_of(plan)
    print("    12 peacetime seeds: 3/3 standing, no step; disaster keeps sit_edge")


def test_07_cul_de_sac_turnaround_stands_in_peacetime():
    seen = set()
    for seed in range(12):
        plan = make_plan(True, seed=seed)
        ppl._cul_de_sac(plan, 6)
        assert len(plan.records) == 6, (seed, plan.notes)
        seen |= set(poses_of(plan))
        assert plan.coerced == {}, plan.coerced
    assert seen == {"idle"}, seen
    seen = set()
    for seed in range(12):
        plan = make_plan(False, seed=seed)
        ppl._cul_de_sac(plan, 6)
        seen |= set(poses_of(plan))
    assert seen == {"idle", "crouch"}, seen
    print("    peacetime bulbs: all idle over 12 seeds; disaster: idle+crouch")


def test_08_casualties_off_in_peacetime():
    plan = make_plan(True)
    plan.cfg["casualty_share"] = 0.5
    for k in range(6):
        plan.add_person("at_home", 0, float(k), 0.0, 0.0, 0.0, "idle")
    ppl._apply_casualties(plan)
    assert all(r["alive"] for r in plan.records)
    assert all(abs(q["roll_deg"]) < 1e-9 for q in plan.humans)
    assert any("ignored" in n for n in plan.notes), plan.notes
    d = make_plan(False)
    d.cfg["casualty_share"] = 0.5
    for k in range(6):
        d.add_person("at_home", 0, float(k), 0.0, 0.0, 0.0, "idle")
    ppl._apply_casualties(d)
    assert sum(1 for r in d.records if not r["alive"]) == 3
    print("    peacetime casualty_share 0.5 -> 0 face-down (noted); disaster -> 3 of 6")


def test_09_peacetime_flag_is_derived_not_configured():
    assert ppl.resolve_cfg({}, has_disaster=False)["peacetime"] is True
    assert ppl.resolve_cfg({}, has_disaster=True)["peacetime"] is False
    # a preset cannot flip it
    cfg = ppl.resolve_cfg({"people": {"peacetime": False, "total": 5}},
                          has_disaster=False)
    assert cfg["peacetime"] is True and cfg["total"] == 5
    # and the peacetime mix has no ground-refuge scenarios
    shares = {k: v.get("share") for k, v in cfg["scenarios"].items()}
    assert shares["open_ground"] == 0.0 and shares["gridlock"] == 0.0
    print("    peacetime = not has_disaster, preset cannot override; "
          "open_ground/gridlock shares 0")


# ── runner ───────────────────────────────────────────────────────────────────

def main():
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith("test_") and callable(o)]
    failures = []
    print("people posture rules: %d tests, offline\n" % len(tests))
    for name, fn in tests:
        print("  " + name)
        try:
            fn()
            print("    PASS\n")
        except Exception as exc:                        # noqa: BLE001
            import traceback
            failures.append(name)
            print("    FAIL: %s" % exc)
            print(traceback.format_exc())
    print("=" * 70)
    if failures:
        print("FAILED %d of %d: %s" % (len(failures), len(tests), ", ".join(failures)))
        return 1
    print("PASSED %d of %d" % (len(tests), len(tests)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
