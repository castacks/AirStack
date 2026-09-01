"""test_hurricane_trap_debris.py — the DRY-LAND casualties are actually
under debris, on the stage and not only in the JSON.

WHAT BROKE. `hurricane_people.plan_people` returns `(humans, debris,
records)`, and `_plan_dry` runs `tornado_people` verbatim: it solves a
covering piece against each body it lays (`tornado_people._cover`), writes
`occlusion` / `covered_frac` / `visible_parts` / `boards` into the record,
and hands the caller the plank specs that make those labels true. The
hurricane launcher bound that second return value as `p_debris` and NEVER
AUTHORED IT. So every dry figure the hurricane scene ever shipped lay fully
exposed while its ground truth said it was half buried — measured on the
reference plate: 24 of 28 dry casualties, 35 boards, none of them on the
stage. The tornado launcher's own `trap_debris` block has always built them.

WHY SOURCE-READ FOR THE LAUNCHERS. `suburb_hurricane_launch_script.py` calls
`isaacsim.SimulationApp(...)` at MODULE SCOPE, so it cannot be imported
without a licensed Isaac Sim — the same constraint
`test_hurricane_tornado_parity_launcher.py` documents and the same technique
it uses. Everything that CAN be exercised for real (the planner, the spec
mapping, `planks.clip_to_region`) is exercised for real, against the actual
`hurricane_people` output.

    python3 scene_gen/tests/test_hurricane_trap_debris.py
    pytest -q scene_gen/tests/test_hurricane_trap_debris.py
"""
import math
import os
import random
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_HERE)
_LAUNCH_DIR = os.path.normpath(os.path.join(
    _SCENE_GEN_DIR, "..", "simulation", "isaac-sim", "launch_scripts"))
_LAUNCHER = os.path.join(_LAUNCH_DIR, "suburb_hurricane_launch_script.py")
_TORNADO_LAUNCHER = os.path.join(_LAUNCH_DIR, "suburb_tornado_launch_script.py")
_BENCH = os.path.join(_LAUNCH_DIR, "hurricane_people_bench_launch_script.py")
sys.path.insert(0, _SCENE_GEN_DIR)
sys.path.insert(0, _HERE)

# The stub resolver/pools and the synthetic shoreline live in the module that
# already owns them — importing it also installs its `ppl._pose_dz` stub, so
# these two files agree on what a placement costs instead of drifting apart.
import test_hurricane_people as TH                              # noqa: E402
from disaster import hurricane_people as hp                     # noqa: E402
from disaster import planks                                     # noqa: E402


def _read(path):
    with open(path) as fh:
        return fh.read()


# ---------------------------------------------------------------------------
# (a) THE PLANNER REALLY DOES PRODUCE COVERING PIECES FOR THE DRY POPULATION
# ---------------------------------------------------------------------------

def _plan(seed=101):
    cfg = hp.resolve_cfg({})
    ctx = TH._make_ctx(random.Random(seed))
    humans, debris, records = hp.plan_people(cfg, ctx, random.Random(seed))
    return ctx, humans, debris, records


def test_dry_pass_returns_boards_for_the_bodies_it_covers():
    """Not a tautology: it pins that the hurricane's own reweighted
    `occlusion` bag (`resolve_cfg`: `none` 0.44 -> 0.18, "more under debris")
    actually reaches `_cover`, and that the covered population is the
    MAJORITY of the dry one rather than a handful."""
    _ctx, _h, debris, records = _plan()
    dry = [r for r in records if r.get("domain") == "dry_wreck"]
    assert dry, "no dry casualties planned at all"
    covered = [r for r in dry if float(r.get("covered_frac") or 0.0) > 0.0]
    assert len(covered) >= 0.6 * len(dry), (
        "only %d of %d dry casualties are under anything; the hurricane "
        "occlusion reweighting is not reaching _cover"
        % (len(covered), len(dry)))
    assert debris, "dry pass covered %d bodies but returned no boards" % len(
        covered)


def test_board_count_matches_what_the_ground_truth_claims():
    """`_cover` writes `boards` per record. The specs the launcher authors
    must number EXACTLY that, summed — the check that would have caught the
    dropped `p_debris` immediately, since the stage had 0 and the JSON
    claimed 35."""
    _ctx, _h, debris, records = _plan()
    claimed = sum(int(r.get("boards") or 0) for r in records)
    assert claimed == len(debris), (
        "ground truth claims %d board(s) over casualties, planner returned "
        "%d spec(s)" % (claimed, len(debris)))
    assert claimed > 0


def test_every_covered_casualty_has_a_board_over_it():
    """Position check, not just a count: each covered body must have at
    least one spec within its own reach, so the boards are ON the casualties
    rather than somewhere in the same field."""
    _ctx, _h, debris, records = _plan()
    pts = [(d["x"], d["y"]) for d in debris]
    orphan = []
    for r in records:
        if int(r.get("boards") or 0) <= 0:
            continue
        # A lying figure is ~1.8 m end to end and a piece is seated at the
        # centre of the span it covers, so the farthest legitimate piece is
        # about a body-length away. 2.5 m is that with margin.
        if not any(math.hypot(px - r["x"], py - r["y"]) <= 2.5
                   for (px, py) in pts):
            orphan.append((r["x"], r["y"], r.get("occlusion")))
    assert not orphan, "casualties claiming cover with no board near: %r" % (
        orphan[:5],)


# ---------------------------------------------------------------------------
# (b) THE SPECS SURVIVE THE LAUNCHER'S OWN MAPPING AND `planks.build`
# ---------------------------------------------------------------------------

def _launcher_spec(d, rng):
    """The literal transform the hurricane launcher applies to `p_debris`
    (`len`/`wide` -> `l`/`w`, planner-solved `t`/`pitch`/`roll` kept). Kept
    in sync with the launcher by `test_spec_mapping_matches_the_tornado_
    launcher_verbatim` below."""
    return {"x": d["x"], "y": d["y"], "z": d["z"],
            "l": d["len"], "w": d["wide"], "yaw": d["yaw"],
            "t": d.get("t", rng.uniform(0.02, 0.05)),
            "pitch": d.get("pitch", rng.uniform(-8.0, 8.0)),
            "roll": d.get("roll", rng.uniform(-10.0, 10.0)),
            "class": d.get("class",
                           "sheathing" if d["wide"] > 0.4 else "board")}


def test_mapped_specs_are_buildable():
    """`planks.build` reads `class` for its per-class material and `_box`
    reads x/y/z/l/w/t/yaw/pitch/roll. A spec missing any of them raises
    inside the launcher's try/except and the boards vanish silently again,
    which is exactly the failure mode being fixed."""
    _ctx, _h, debris, _records = _plan()
    rng = random.Random(3)
    for d in debris:
        s = _launcher_spec(d, rng)
        for k in ("x", "y", "z", "l", "w", "t", "yaw", "pitch", "roll"):
            assert isinstance(s[k], float), "%s is %r" % (k, s[k])
        assert s["l"] > 0.0 and s["w"] > 0.0 and s["t"] > 0.0
        assert s["class"] in planks.STOCK, (
            "class %r has no material in planks.STOCK" % (s["class"],))
        planks._box(s)          # raises if a key or a type is wrong


def test_boards_stay_on_the_plate_after_clipping():
    """`clip_to_region` is in the launcher path and drops boards whose
    GEOMETRY leaves the plate. A covering piece is seated on the body it
    covers, and the planner already refuses a body whose own ends leave the
    region — so clipping must not be quietly eating the cover."""
    _ctx, _h, debris, _records = _plan()
    rng = random.Random(3)
    specs = [_launcher_spec(d, rng) for d in debris]
    kept, dropped = planks.clip_to_region(specs, TH.REGION, verbose=False)
    assert dropped == 0, (
        "%d of %d covering board(s) clipped off the plate" % (dropped,
                                                              len(specs)))
    assert len(kept) == len(specs)


def test_determinism():
    """Same seed, same boards — the launcher reruns this pass on every
    rebuild and a scene whose burial moves between builds cannot be
    reviewed."""
    _c1, _h1, d1, _r1 = _plan(seed=77)
    _c2, _h2, d2, _r2 = _plan(seed=77)
    assert len(d1) == len(d2)
    for a, b in zip(d1, d2):
        assert a == b


# ---------------------------------------------------------------------------
# (c) THE LAUNCHER ACTUALLY AUTHORS THEM — the regression guard
# ---------------------------------------------------------------------------

def test_launcher_builds_the_trap_debris():
    src = _read(_LAUNCHER)
    assert re.search(r'\bp_humans,\s*p_debris,\s*p_recs\s*=\s*hpp\.plan_people',
                     src), "the hurricane launcher no longer calls plan_people"
    assert re.search(r'if\s+p_debris\s*:', src), (
        "`p_debris` is bound but never consumed — the exact defect this file "
        "exists to prevent: the dry casualties' covering boards are planned "
        "and thrown away, and the ground truth lies about every one of them")
    assert re.search(r'planks\.build\(\s*stage,\s*PARENT\s*\+\s*'
                     r'"/trap_debris"', src), (
        "no `planks.build` under PARENT/trap_debris")
    assert "planks.clip_to_region" in src


def test_spec_mapping_matches_the_tornado_launcher_verbatim():
    """The user's instruction was "literally do the exact same thing that
    tornado does". Both launchers must map `p_debris` -> plank specs with
    the same keys and the same fallbacks; a divergence here is a divergence
    in what a casualty looks like between the two disasters."""
    def _keys(src):
        m = re.search(r'\{"x": d\["x"\].*?for d in p_debris\]',
                      src, re.S)
        assert m, "no p_debris spec comprehension found"
        body = m.group(0)
        return sorted(set(re.findall(r'"([a-z]+)":', body)))
    assert _keys(_read(_LAUNCHER)) == _keys(_read(_TORNADO_LAUNCHER))


# ---------------------------------------------------------------------------
# (d) THE BENCH SHOWS IT — it is the only way this gets reviewed
# ---------------------------------------------------------------------------

def test_bench_runs_the_hurricane_dry_pass_not_the_raw_tornado_one():
    src = _read(_BENCH)
    assert 'hpp.resolve_cfg(config)["dry"]' in src, (
        "the bench builds its config from the tornado defaults, so it shows "
        "the tornado occlusion mix, not the hurricane's own")
    assert "hpp._plan_dry(" in src, (
        "the bench must exercise the same call the hurricane scene makes")


def test_raw_planner_specs_are_not_buildable_without_the_mapping():
    """The reason both the launcher block and the bench need a mapping at
    all, pinned as behaviour rather than as a comment: `_cover_piece` emits
    `len`/`wide` and `planks._box` reads `l`/`w`. If a future `planks` ever
    accepts the planner's own key names this test fails LOUDLY and the two
    mappings can be deleted deliberately, rather than rotting."""
    _ctx, _h, debris, _records = _plan()
    assert debris
    try:
        planks._box(dict(debris[0]))
    except KeyError as exc:
        assert "l" in str(exc)
        return
    raise AssertionError(
        "planks._box now accepts a raw planner spec; the len/wide -> l/w "
        "mapping in both launchers and the bench is dead code")


def test_bench_maps_the_specs_before_building_them():
    """The bench handed `planks.build` the RAW planner specs, so `_box`
    raised `KeyError: 'l'` on the first board and the bare `except` printed
    one line and moved on — zero covering boards authored, on every run this
    bench has ever made."""
    src = _read(_BENCH)
    m = re.search(r'specs\s*=\s*\[\{"x": d\["x"\].*?for d in trapped_specs\]',
                  src, re.S)
    assert m, ("the bench builds `trapped_specs` directly; `planks._box` "
               "reads l/w/t and the planner emits len/wide, so every board "
               "is lost to a swallowed KeyError")
    body = m.group(0)
    for k in ("l", "w", "t", "yaw", "pitch", "roll", "class"):
        assert '"%s":' % k in body, "mapped spec is missing %r" % k
    assert 'd["len"]' in body and 'd["wide"]' in body


def test_bench_aims_its_trapped_cameras_at_the_casualties():
    """`wx`/`wy` are the WATER loop's leftover iteration variables — about
    24 m from the wreck, out over the water quad. Both trapped captures used
    them, so every bench frame of the burial has been a picture of empty
    water."""
    src = _read(_BENCH)
    assert not re.search(r'\(wx\s*-\s*7\.0,\s*wy\s*-\s*2\.0\)', src), (
        "a trapped camera is still aimed at the water loop's leftovers")
    assert '"trapped_debris": trap_xy' in src
    assert '("trapped_close", trap_xy, trap_span)' in src


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_") or not callable(fn):
            continue
        try:
            fn()
            print("PASS  %s" % name)
        except Exception as exc:
            fails += 1
            print("FAIL  %s: %s" % (name, exc))
    print("%s" % ("ALL PASS" if not fails else "%d FAILURE(S)" % fails))
    sys.exit(1 if fails else 0)
