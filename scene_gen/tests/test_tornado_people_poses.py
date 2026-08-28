#!/usr/bin/env python3
"""
test_tornado_people_poses.py — the tornado CASUALTIES: their attitude, the
debris on top of them, and how much of each one is left visible. Pinned
without Isaac.

WHAT THIS EXISTS FOR. Two renders drove the module to its current shape and
this file is the memory of both.

  * The FIRST 100 m render was "a pavement full of commuters": 67 figures of
    which 55 were standing or walking, and 12 were casualties. The module now
    places casualties and nothing else — see WHAT IS DELIBERATELY ABSENT in
    `tornado_people` — so a test that the mix is casualties is a test that the
    regression cannot come back.
  * The FIRST BENCH render showed the two geometry faults that a host test can
    actually catch: limbs standing straight UP off every side-lying figure
    (the rest pose is an A-pose, so the arm is already 45 degrees out in the
    character's X — which after the long-axis spin is vertical — and swinging
    it forward about X without first bringing it to plumb leaves it there),
    and covering boards passing THROUGH raised knees and heels (a piece was
    seated at the body's chest height everywhere).

So this file pins, in order:

  1. THE VOCABULARY EXISTS AND IS REACHABLE. Every pose the config can draw is
     in `scene_generator._HUMAN_POSES`, in `people.LYING_POSES` and in
     `tornado_people._ATTITUDE`.
  2. NO LIMB STANDS UP. Every limb of every lying pose is carried through the
     pose's own deltas and its lay-down rotation analytically, and its
     elevation above the ground plane is bounded.
  3. THE FIGURE IS LAID DOWN THE RIGHT WAY UP, and a side-lying one is spun
     about its own long axis by the pitch — which does not move its head.
  4. NOTHING IS FULLY BURIED. `max_covered_frac` is enforced while the pieces
     are generated, over every seed.
  5. THE OCCLUSION PATTERN MEANS WHAT IT SAYS. `legs` covers the legs and
     leaves the head; `all_but_head` leaves a head. Asserted from the
     geometry, against `visible_parts`.
  6. THE DEBRIS IS ON THE BODY, as its true rotated box (`planks._box`, the
     same geometry the launcher builds), and it CLEARS the body's crest.
  7. NOBODY LEAVES THE PLATE, tested end to end down each body.

RUNS WITHOUT ISAAC. `disaster/tornado_people.py`, `disaster/people.py` and
`disaster/planks.py` import only the standard library at module scope;
`people._pose_dz` reaches for `scene_generator` (and through it `pxr`) and is
stubbed, because the pose table is read as SOURCE and `literal_eval`'d instead
— so the checks run the repo's real data without needing a USD build.

USAGE
    python3 scene_gen/tests/test_tornado_people_poses.py
    pytest -s scene_gen/tests/test_tornado_people_poses.py
"""

import ast
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
_REPO = os.path.dirname(_SCENE_GEN)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import people as ppl                    # noqa: E402
from disaster import planks                           # noqa: E402
from disaster import tornado_people as tpp            # noqa: E402

# `_pose_dz` is the one line in `people` that pulls in `scene_generator` and
# with it `pxr`. It answers "how far to drop a posed figure", which is not what
# any check here is about — the z values this file asserts on are the lying
# branch's own, which never call it.
ppl._pose_dz = lambda usd, pose, height: 0.0

_SG_PY = os.path.join(_SCENE_GEN, "scene_generator.py")
with open(_SG_PY, encoding="utf-8") as _f:
    SG_SRC = _f.read()


def _sg_table(name):
    """A top-level dict literal out of scene_generator, by name.

    Read as SOURCE rather than imported, exactly as `test_people_rules` does
    it: importing `scene_generator` needs `pxr`, and the pose table is data.
    """
    tree = ast.parse(SG_SRC)
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)
                and node.targets[0].id == name):
            return ast.literal_eval(node.value)
    raise AssertionError("%s not found in scene_generator.py" % name)


HUMAN_POSES = _sg_table("_HUMAN_POSES")
GROUND_CONTACT = _sg_table("_POSE_GROUND_CONTACT")

# The casualty attitudes the benchmark is defined on. Named here rather than
# derived from the table so that DELETING one is a failure and not a silent
# narrowing back toward "everybody is face-up".
# `trapped_sit` IS DELIBERATELY NOT HERE. The one upright casualty was cut on
# the second review — a figure sitting bolt upright in a levelled block reads
# as somebody who sat down — so every attitude in the mix is horizontal and
# `test_08` asserts exactly that.
CASUALTY_POSES = ("lying_supine", "lying_supine_open",
                  "lying_prone", "lying_prone_reach",
                  "lying_side_l", "lying_side_r", "lying_curled_l")

# Joints the UE4 mannequin skeleton these rigs are built on actually has. A
# delta on anything else is silently dropped by `_pose_joint_transforms`, so a
# typo would cost part of a posture with no error anywhere.
UE4_JOINTS = {
    "pelvis", "spine_01", "spine_02", "spine_03", "neck_01", "head",
    "clavicle_l", "clavicle_r", "upperarm_l", "upperarm_r",
    "lowerarm_l", "lowerarm_r", "hand_l", "hand_r",
    "thigh_l", "thigh_r", "calf_l", "calf_r", "foot_l", "foot_r",
    "ball_l", "ball_r",
}


# ── stubs: the two objects a placement needs, and nothing else ───────────────

class _Pools:
    def scale_of(self, usd): return 0.01
    def axis_of(self, usd): return "z"
    def yaw_of(self, usd): return 90.0          # the pack's authored facing
    def roll_of(self, usd): return 0.0


class _Resolver:
    """A 1.80 m character 0.35 m deep — the reference the pose table is
    derived on, so the numbers in a failure message are readable."""
    def get(self, usd, cat, **kw):
        return {"sx": 1.25, "sy": 0.35, "sz": 1.80, "base": 0.0,
                "cx": 0.0, "cy": 0.0, "cz": 0.0}


HUMANS = ["rp_carla_rigged_001_ue4.usd", "rp_eric_rigged_001_ue4.usd",
          "rp_sophia_rigged_002_ue4.usd", "rp_nathan_rigged_003_ue4.usd"]

REGION = (-100.0, -100.0, 100.0, 100.0)


def make_ctx(region=REGION, planks_on=True):
    """A context shaped like the assembly launcher's, built by arithmetic.

    NOT the real plat: that reads every file in `config/asset_sets/`, which
    other work edits, so a suite asserting on its output would go red for
    reasons that have nothing to do with people. The SHAPE is what matters —
    a spread of wrecks across the damage ladder, a street, and (optionally) a
    real `planks` field so `_Deck` has a real surface to measure.
    """
    lv = ("leveled", "swept", "partial_collapse", "roof_collapsed")
    wrecks = []
    k = 0
    for ix in range(-3, 4):
        for iy in range(-3, 4):
            if (ix + iy) % 3 == 0:
                continue
            wrecks.append({"x": ix * 24.0, "y": iy * 24.0,
                           "fp": 12.0 + (k % 3) * 2.0,
                           "intensity": 0.4 + 0.1 * (k % 5),
                           "level": lv[k % len(lv)]})
            k += 1
    road_pts = []
    for ix in range(-3, 4):
        for t in range(-3, 4):
            road_pts.append((ix * 24.0 + 12.0, t * 24.0, 90.0))
    ctx = {"wrecks": wrecks, "road_pts": road_pts, "throw_deg": 38.0,
           "humans": list(HUMANS), "resolver": _Resolver(),
           "asset_pools": _Pools()}
    if planks_on:
        prng = random.Random(7)
        specs = []
        for w in wrecks:
            specs += planks.scatter_from_wreck(
                w["x"], w["y"], w["fp"], w["intensity"], 38.0, 14.0, prng,
                n_pieces=90)
        if region is not None:
            specs, _n = planks.clip_to_region(specs, region, verbose=False)
        ctx["plank_specs"] = specs
    if region is not None:
        ctx["region"] = region
    return ctx


def run_plan(seed=91, region=REGION, cfg_over=None, planks_on=True):
    cfg = tpp.resolve_cfg({"people": cfg_over} if cfg_over else {})
    ctx = make_ctx(region=region, planks_on=planks_on)
    humans, debris, recs = tpp.plan_people(cfg, ctx, random.Random(seed))
    return cfg, ctx, humans, debris, recs


def _hist(recs, key):
    out = {}
    for r in recs:
        out[r.get(key)] = out.get(r.get(key), 0) + 1
    return out


# ── 1. the vocabulary ───────────────────────────────────────────────────────

def test_01_every_casualty_pose_is_reachable_end_to_end():
    """A pose the config can draw has to exist in all three tables it passes
    through, or it fails at authoring time inside a twenty-minute build."""
    cfg = tpp.resolve_cfg({})
    for pose in CASUALTY_POSES:
        assert pose in HUMAN_POSES, "%s is not in _HUMAN_POSES" % pose
        assert pose in tpp._ATTITUDE, "%s has no body plan" % pose
        assert pose in cfg["poses"], "%s is not in the default mix" % pose
    for pose in cfg["poses"]:
        assert pose in HUMAN_POSES, "the mix draws unknown pose %r" % pose
    # EVERY ONE OF THEM IS LAID DOWN. There is no upright casualty any more.
    for pose in CASUALTY_POSES:
        assert pose in ppl.LYING_POSES, "%s is not laid down" % pose
        assert tpp._ATTITUDE[pose]["attitude"] in ("face_up", "face_down",
                                                   "side"), pose
    assert "trapped_sit" not in tpp._ATTITUDE, (
        "the upright casualty is back in the mix")
    print("    %d casualty poses, all three tables agree" % len(CASUALTY_POSES))


def test_02_pose_deltas_name_joints_the_rig_has():
    """A delta on a joint that is not there is dropped in silence."""
    bad = {}
    for pose, deltas in HUMAN_POSES.items():
        for name in deltas:
            if name.rstrip("+") not in UE4_JOINTS:
                bad.setdefault(pose, []).append(name)
    assert not bad, "deltas on unknown joints: %s" % bad
    i = SG_SRC.index("def _bind_human_pose(")
    j = SG_SRC.index("\ndef ", i + 10)
    assert "joint(s) not in this rig" in SG_SRC[i:j], (
        "_bind_human_pose no longer reports dropped deltas")
    print("    %d poses, every delta on a UE4 joint" % len(HUMAN_POSES))


# ── 2. NO LIMB STANDS UP ────────────────────────────────────────────────────
#
# The bug the first bench render showed, as arithmetic. Each limb is a unit
# vector from its joint; the rest arm hangs 45 deg OUT to the side (the pack is
# A-posed, which `idle`'s +-40-to-plumb delta measures) and the rest leg hangs
# plumb. The pose's deltas are WORLD-axis rotations applied about the joint's
# own origin in dict order, and a child inherits its parent's accumulated
# rotation — exactly what `_pose_joint_transforms` does. Then the lay-down
# rotation Rx(roll) . Ry(pitch) is applied and the world z read off.

_R2 = 0.70710678
_REST_DIR = {"upperarm_l": (_R2, 0.0, -_R2), "upperarm_r": (-_R2, 0.0, -_R2),
             "thigh_l": (0.0, 0.0, -1.0), "thigh_r": (0.0, 0.0, -1.0)}
_PARENT = {"lowerarm_l": "upperarm_l", "lowerarm_r": "upperarm_r",
           "calf_l": "thigh_l", "calf_r": "thigh_r"}
LIMBS = tuple(_REST_DIR) + tuple(_PARENT)


def _rot(v, axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    x, y, z = v
    if axis[0]:
        return (x, y * c - z * s, y * s + z * c)
    if axis[1]:
        return (x * c + z * s, y, -x * s + z * c)
    return (x * c - y * s, x * s + y * c, z)


def _limb_elevation(pose, limb):
    """Degrees the limb rises above the ground plane once the figure is down."""
    chain = []
    j = limb
    while j is not None:
        chain.append(j)
        j = _PARENT.get(j)
    chain.reverse()
    v = _REST_DIR[chain[0]]
    for link in chain:
        for name, (axis, deg) in HUMAN_POSES[pose].items():
            if name.rstrip("+") == link:
                v = _rot(v, axis, deg)
    v = _rot(v, (1, 0, 0), ppl.LYING_POSES[pose])
    v = _rot(v, (0, 1, 0), ppl.LYING_SPIN.get(pose, 0.0))
    return math.degrees(math.asin(max(-1.0, min(1.0, v[2]))))


def test_03_no_limb_of_a_lying_figure_stands_up():
    """THE BUG THE BENCH FOUND, pinned.

    A limb may be flexed — a drawn-up knee is what makes a casualty read as a
    body rather than a mannequin — but nothing may stand up off the ground.
    The bound is 45 degrees, which is a knee relaxed open with the heel a
    hand's breadth off the ground; a
    limb that has kept the A-pose's 45 degrees of abduction through a
    long-axis spin comes out at 80-90 and is what the first render showed on
    every side-lying figure.
    """
    worst = []
    for pose in sorted(ppl.LYING_POSES):
        for limb in LIMBS:
            el = _limb_elevation(pose, limb)
            worst.append((abs(el), pose, limb, el))
            assert el <= 45.0, (
                "%s: %s stands %.0f deg off the ground — has the A-pose "
                "abduction been taken out before the swing? (see the plumb "
                "correction in the lateral poses)" % (pose, limb, el))
    worst.sort(reverse=True)
    print("    %d lying poses x %d limbs; steepest %s %s at %+.0f deg"
          % (len(ppl.LYING_POSES), len(LIMBS), worst[0][1], worst[0][2],
             worst[0][3]))


def test_04_the_lateral_poses_bend_in_THEIR_ground_plane():
    """THE AXIS TRAP, and it is the mirror image of the face-up one.

    For a face-up or face-down figure the ground plane is span(X, Z), so a
    delta about Y keeps a limb on the ground. Spin the body about its long
    axis and the plane becomes span(Y, Z): now it is a delta about X that
    keeps a limb down. Every SWING in a lateral pose must therefore be about
    X, and the Y deltas that are there are the plumb correction and the
    deliberate lift of the top limb.
    """
    for pose in sorted(ppl.LYING_SPIN):
        deltas = HUMAN_POSES[pose]
        for joint, (axis, deg) in deltas.items():
            bare = joint.rstrip("+")
            if bare.startswith(("thigh", "calf", "lowerarm")):
                if tuple(axis) == (0.0, 1.0, 0.0):
                    assert abs(deg) <= 20.0, (
                        "%s: %s has a %.0f deg Y delta — for a side-lying "
                        "figure that is straight up or straight down"
                        % (pose, joint, deg))
        # ...and every arm is brought to plumb before it is swung
        for side in ("l", "r"):
            key = "upperarm_%s" % side
            firsts = [(j, d) for j, d in deltas.items()
                      if j.rstrip("+") == key]
            assert firsts, "%s: %s is not posed at all" % (pose, key)
            axis, deg = firsts[0][1]
            assert tuple(axis) == (0.0, 1.0, 0.0) and abs(abs(deg) - 40.0) < 6, (
                "%s: %s's FIRST delta must be the +-40 plumb correction, not "
                "%s %s — see the A-pose note" % (pose, key, axis, deg))
    for pose in ("lying_supine", "lying_prone", "lying_supine_open",
                 "lying_prone_reach"):
        for joint, (axis, deg) in HUMAN_POSES[pose].items():
            if joint.rstrip("+").startswith(("upperarm", "lowerarm")):
                assert tuple(axis) == (0.0, 1.0, 0.0), (
                    "%s: %s swings about %s, which after the roll drives it "
                    "into the ground or holds it in the air"
                    % (pose, joint, axis))
    print("    %d lateral poses swing about X; %d flat ones swing about Y"
          % (len(ppl.LYING_SPIN), 4))


# ── 3. laid down, the right way up, and spun the right way ──────────────────

def test_05_the_roll_and_the_spin_come_from_the_pose():
    ctx = {"asset_pools": _Pools(), "resolver": _Resolver()}
    for pose, want in ppl.LYING_POSES.items():
        for (x, y) in ((5.0, 5.0), (-5.0, -5.0)):
            # BOTH SIGNS OF (x + y) — the old rule flipped on exactly that, so
            # half the supine figures used to come out face-down.
            q = ppl._human_placement(ctx, HUMANS[0], x, y, 0.0, 40.0, pose,
                                     prone=True)
            assert abs(q["roll_deg"] - want) < 1e-9, (pose, q["roll_deg"])
            assert abs(q["pitch_deg"]
                       - ppl.LYING_SPIN.get(pose, 0.0)) < 1e-9, q
            assert q["pose"] == pose
            # ...and the lift follows the attitude: half the measured DEPTH on
            # the back or the face, half the MODELLED breadth on a side.
            want_z = (ppl._LATERAL_HALF_BREADTH_H * 1.80
                      if pose in ppl.LYING_SPIN else 0.175)
            assert abs(q["z_m"] - want_z) < 1e-6, (pose, q["z_m"], want_z)
    print("    %d lying poses: roll, spin and lift all from the pose"
          % len(ppl.LYING_POSES))


def test_06_a_lying_pose_standing_up_is_refused():
    ctx = {"asset_pools": _Pools(), "resolver": _Resolver()}
    for pose in ppl.LYING_POSES:
        try:
            ppl._human_placement(ctx, HUMANS[0], 0.0, 0.0, 0.0, 0.0, pose,
                                 prone=False)
        except ValueError as exc:
            assert "prone=True" in str(exc)
        else:
            raise AssertionError("%s was placed upright" % pose)
    print("    every lying pose refuses to be placed standing")


def test_07_the_spin_does_not_move_the_head():
    """`_body_axis` reads the ROLL alone, and that is only sound because a
    rotation about an axis cannot move a vector lying along it. Pinned,
    because a side-lying figure whose head end is computed backwards puts
    every board over its shoulders instead of its shins."""
    for pose in sorted(ppl.LYING_POSES):
        roll = ppl.LYING_POSES[pose]
        for yaw in (0.0, 37.0, 180.0, 314.0):
            ux, uy = tpp._body_axis(pose, yaw, roll)
            want = 1.0 if roll >= 0 else -1.0
            assert abs(ux - want * math.cos(math.radians(yaw))) < 1e-9
            assert abs(uy - want * math.sin(math.radians(yaw))) < 1e-9
    print("    the head end is a function of the roll alone, for every spin")


# ── 4. the mix is casualties, and it covers both axes ───────────────────────

def test_08_every_figure_is_a_casualty():
    """The 55-standing-figures regression, as an assertion."""
    _c, _x, humans, _d, recs = run_plan()
    assert recs, "no casualties at all"
    upright = [r for r in recs if r["pose"] not in ppl.LYING_POSES]
    assert not upright, "upright figures are back: %s" % _hist(upright, "pose")
    for r in recs:
        assert r["attitude"] in ("face_up", "face_down", "side"), r
    print("    %d figures, all casualties: %s"
          % (len(recs), _hist(recs, "attitude")))


def test_09_both_axes_are_covered_over_seeds():
    """Front, back and sideways all appear, and so does an unoccluded body and
    a covered one — over a spread of seeds, because any one plate is small."""
    att, occ, vis = {}, {}, {}
    for seed in (3, 11, 29, 91, 137):
        _c, _x, _h, _d, recs = run_plan(seed=seed)
        for r in recs:
            att[r["attitude"]] = att.get(r["attitude"], 0) + 1
            occ[r["occlusion"]] = occ.get(r["occlusion"], 0) + 1
            vis[r["visibility"]] = vis.get(r["visibility"], 0) + 1
    for a in ("face_up", "face_down", "side"):
        assert att.get(a, 0) >= 3, ("attitude %s barely appears" % a, att)
    assert vis.get("full", 0) >= 5, ("no fully observable casualties", vis)
    assert vis.get("partial", 0) >= 10, ("nothing is partially covered", vis)
    assert len(occ) >= 7, ("the occlusion patterns are not spread", occ)
    print("    attitudes %s" % att)
    print("    visibility %s over %d patterns" % (vis, len(occ)))


# ── 5. NOTHING IS FULLY BURIED ──────────────────────────────────────────────

def test_10_nothing_is_ever_fully_buried():
    """THE ONE HARD RULE. A target a camera cannot see is not a hard example,
    it is a wrong label — enforced while the pieces are generated, so it holds
    over every seed rather than on average."""
    cap = tpp.DEFAULTS["max_covered_frac"]
    worst = 0.0
    for seed in (3, 11, 29, 91, 137):
        _c, _x, _h, _d, recs = run_plan(seed=seed)
        for r in recs:
            worst = max(worst, r["covered_frac"])
            assert r["covered_frac"] <= cap + 1e-6, r
            assert r["visible_parts"], ("nothing visible at all", r)
    print("    worst covered fraction over 5 seeds: %.2f (cap %.2f)"
          % (worst, cap))


def test_11_the_pattern_means_what_it_says():
    """`legs` covers the legs and leaves the head. `all_but_head` leaves a
    head. Read off `visible_parts`, which is what a scorer reads.

    RUN AT A CAP THAT ADMITS EVERY PATTERN, and that is the point rather than a
    dodge. This checks the VOCABULARY — what each name in `_OCCLUSION` means —
    and `_trim_spans` shortens any pattern wider than `max_covered_frac`, which
    after the 2026-08-28 visibility pass (0.80 -> 0.55) is the four widest.
    A trimmed `all_but_head` genuinely does leave a torso, so asserting the
    untrimmed meaning against the scene's ceiling would be asserting that the
    trim does not work. The scene's own ceiling is `test_10`; that the four
    trimmed names are NOT drawable in a scene is `test_20`.
    """
    ctx = make_ctx(region=None, planks_on=False)
    cfg = tpp.resolve_cfg({"people": {"max_covered_frac": 0.90}})
    want = {
        "none": (("head", "torso", "feet", "thighs"), ()),
        "legs": (("head", "torso"), ("thighs",)),
        "feet_shins": (("head", "torso", "thighs"), ("feet",)),
        "torso_head": (("feet", "shins"), ("torso", "head")),
        "head_only": (("feet", "torso"), ("head",)),
        "all_but_head": (("head",), ("torso", "thighs")),
        "all_but_feet": (("feet",), ("torso", "head")),
        "upper_body": (("feet", "shins"), ("torso", "head")),
    }
    for pattern, (seen, hidden) in sorted(want.items()):
        _h, _d, recs, _cells = tpp.plan_catalogue(
            cfg, ctx, random.Random(5),
            poses=("lying_supine_open", "lying_prone_reach", "lying_side_l"),
            patterns=(pattern,))
        assert recs, pattern
        for r in recs:
            vis = set(r["visible_parts"])
            for part in seen:
                assert part in vis, (
                    "%s / %s: %s should still be visible, got %s"
                    % (pattern, r["pose"], part, sorted(vis)))
            for part in hidden:
                assert part not in vis, (
                    "%s / %s: %s should be covered, got %s"
                    % (pattern, r["pose"], part, sorted(vis)))
    print("    %d patterns cover exactly what they name" % len(want))


# ── 6. the debris is ON the body, and CLEARS it ─────────────────────────────

def _obb(spec):
    """The piece's true rotated box, from the real geometry."""
    pts, _n = planks._box({"x": spec["x"], "y": spec["y"], "z": spec["z"],
                           "l": spec["len"], "w": spec["wide"],
                           "t": spec["t"], "yaw": spec["yaw"],
                           "pitch": spec["pitch"], "roll": spec["roll"]})
    return pts


def _seg_point_dist(ax, ay, bx, by, px, py):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 <= 0 else max(
        0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + dx * t), py - (ay + dy * t))


def test_12_every_covering_piece_lands_on_its_own_body():
    """As geometry rather than as luck: each piece's plan-projected centre is
    within a body's own extent, and the set as a whole crosses the axis."""
    _c, ctx, humans, debris, recs = run_plan()
    assert debris, "nothing was laid on anybody"
    bodies = []
    for r in recs:
        if r["occlusion"] == "none":
            continue
        a = math.radians(r["body_axis_deg"])
        bodies.append((r["x"], r["y"],
                       r["x"] + math.cos(a) * r["reach_m"],
                       r["y"] + math.sin(a) * r["reach_m"]))
    assert bodies
    stray = 0
    for d in debris:
        near = min(_seg_point_dist(ax, ay, bx, by, d["x"], d["y"])
                   for (ax, ay, bx, by) in bodies)
        if near > 1.15:
            stray += 1
    assert stray == 0, "%d of %d pieces are not on anybody" % (stray,
                                                               len(debris))
    print("    %d pieces, every one within 1.15 m of a body's own axis"
          % len(debris))


def test_13_a_covering_piece_clears_the_body_it_lies_on():
    """THE OTHER BENCH BUG. A piece seated at chest height passes through a
    raised knee, a drawn-up heel or a side-lying shoulder. Each piece's
    UNDERSIDE must be at or above the crest of the body under its footprint.
    """
    ctx = make_ctx(region=None, planks_on=False)
    cfg = tpp.resolve_cfg({})
    checked = 0
    for pose in ("lying_supine", "lying_prone", "lying_side_l",
                 "lying_curled_l"):
        humans, debris, recs, _cells = tpp.plan_catalogue(
            cfg, ctx, random.Random(17), poses=(pose,),
            patterns=("legs", "torso", "all_but_feet", "banded"))
        for i, r in enumerate(recs):
            base = r["z"] - ppl._lying_lift(pose, 0.35, 1.80)
            for d in debris:
                if _seg_point_dist(
                        r["x"], r["y"],
                        r["x"] + math.cos(math.radians(r["body_axis_deg"]))
                        * r["reach_m"],
                        r["y"] + math.sin(math.radians(r["body_axis_deg"]))
                        * r["reach_m"], d["x"], d["y"]) > 1.0:
                    continue
                low = min(p[2] for p in _obb(d))
                # the crest under a piece is at most the pose's tallest band
                crest = tpp._crest(pose, 0.0, 1.0, 1.80)
                assert low >= base - 0.06, (
                    "%s: a piece reaches %.3f m, below the body's own ground "
                    "plane at %.3f" % (pose, low, base))
                assert low <= base + crest + 0.75, (
                    "%s: a piece floats at %.3f m over a crest of %.3f"
                    % (pose, low, crest))
                checked += 1
    assert checked > 20
    print("    %d piece/body pairs: none through the body, none floating"
          % checked)


def test_14_a_propped_piece_leans_the_way_it_is_supposed_to():
    """`planks._box` puts the LENGTH along local +X and lands that end at
    `-sin(pitch) * L/2`, so the high end needs a NEGATIVE pitch."""
    _c, _x, _h, debris, _r = run_plan()
    propped = [d for d in debris if d.get("propped")]
    assert propped, "no propped pieces at all"
    for d in propped:
        assert d["pitch"] < 0.0, d
        pts = _obb(d)
        assert max(p[2] for p in pts) - min(p[2] for p in pts) > 0.10, d
    for d in (x for x in debris if not x.get("propped")):
        assert abs(d["pitch"]) <= 6.0 and abs(d["roll"]) <= 9.0, d
    print("    %d propped (all high-end-up), %d flat"
          % (len(propped), len(debris) - len(propped)))


# ── 7. the plate, and the surface ───────────────────────────────────────────

def test_16_no_body_leaves_the_region():
    """END TO END. A lying figure extends its whole reach past its placement
    point, so feet a metre inside the boundary can still put a head outside."""
    for seed in (3, 11, 29, 91, 137):
        _c, _x, _h, _d, recs = run_plan(seed=seed)
        for r in recs:
            a = math.radians(r["body_axis_deg"])
            for t in (0.0, 0.5, 1.0):
                px = r["x"] + math.cos(a) * r["reach_m"] * t
                py = r["y"] + math.sin(a) * r["reach_m"] * t
                assert REGION[0] <= px <= REGION[2], (r, px)
                assert REGION[1] <= py <= REGION[3], (r, py)
    print("    every body, end to end, inside the plate over 5 seeds")


def test_17_the_deck_is_measured_off_the_boards_that_were_authored():
    """`_Deck` reads the same specs `planks.build` gets, so a body lands on
    the boards rather than on a per-damage-level constant."""
    ctx = make_ctx()
    deck = tpp._Deck(ctx["plank_specs"])
    assert deck.n > 500, deck.n
    tops = [deck.at(w["x"], w["y"]) for w in ctx["wrecks"]]
    assert max(tops) > 0.05, ("the deck is flat everywhere", tops)
    # ...and a body is seated on the HIGHEST station under it, never inside a
    # board: its own ground plane is at or above every sample it spans.
    _c, _x, _h, _d, recs = run_plan()
    sunk = 0
    for r in recs:
        if r["where"] in ("yard", "street", "trail"):
            continue
        a = math.radians(r["body_axis_deg"])
        base = r["z"] - ppl._lying_lift(r["pose"], 0.35, 1.80)
        prof = deck.profile(r["x"], r["y"], math.cos(a), math.sin(a),
                            r["reach_m"])
        if base < max(prof) - 0.05 - r["sunk_frac"] * 0.35:
            sunk += 1
    assert sunk == 0, "%d bodies are inside the board they lie on" % sunk
    print("    %d boards in the deck, tallest cell %.2f m; no body inside one"
          % (deck.n, max(deck.g.values())))


def test_18_a_tilted_pile_is_refused_rather_than_rendered():
    """The tilt test is what stopped a body being laid on the sloping face of
    a half sheet of plywood. Turn it down and the planner must place FEWER
    bodies, not the same ones.

    MEASURED ON A ROUGH DECK, and it has to be: once `planks._lay` was fixed
    to lay boards flat the board field became genuinely level, so the tilt
    rule stopped firing on it and a cap comparison over the plank deck alone
    compares nothing (measured 85 against 84). The roughness that matters now
    is the ARCHETYPE PILE, which reaches the planner as `ctx["deck_points"]`,
    so that is what this drives.
    """
    ctx = make_ctx()
    # A rough pile over every wreck: a 1 m lattice of alternating heights, so
    # any 1.8 m body spans a step whatever bearing it takes.
    pts = []
    for w in ctx["wrecks"]:
        for a in range(-7, 8):
            for b in range(-7, 8):
                pts.append((w["x"] + a, w["y"] + b,
                            0.9 if (a + b) % 2 else 0.2))
    ctx["deck_points"] = pts
    _w = {"pile": 1.0, "skirt": 0.0, "yard": 0.0, "street": 0.0}

    def run(cap):
        cfg = tpp.resolve_cfg({"people": {
            "max_deck_tilt_m": cap, "max_deck_tilt_pile_m": cap,
            "max_total": 400, "where": _w}})
        return tpp.plan_people(cfg, ctx, random.Random(91))[2]

    loose, tight = run(10.0), run(0.05)
    assert len(tight) < len(loose), (len(tight), len(loose))
    # ...and the deck the bodies were seated on is the PILE, not the boards.
    on_pile = [r for r in loose if r["z"] > 0.4]
    assert on_pile, "no body was seated on the archetype pile at all"
    print("    rough pile: tilt cap 10 m -> %d bodies, 0.05 m -> %d; "
          "%d seated above 0.4 m" % (len(loose), len(tight), len(on_pile)))

def test_19_the_record_says_what_was_actually_authored():
    _c, _x, humans, _d, recs = run_plan()
    assert len(humans) == len(recs)
    for p, r in zip(humans, recs):
        assert abs(p["x_m"] - r["x"]) < 2e-3 and abs(p["y_m"] - r["y"]) < 2e-3
        assert p["pose"] == r["pose"]
        for k in ("attitude", "where", "occlusion", "covered_frac",
                  "visible_parts", "visibility", "body_axis_deg", "reach_m",
                  "alive", "sunk_frac", "boards"):
            assert k in r, "the record is missing %s" % k
    s = tpp.summarise(recs)
    assert s["total"] == len(recs)
    assert set(s["by_attitude"]) <= {"face_up", "face_down", "side"}
    print("    ground truth matches the placements; summary keys %s"
          % sorted(s))


# ── 8. the gates: the track, and the occluders nobody could see ─────────────

def test_20_every_drawable_occlusion_fits_under_the_cap():
    """A PATTERN THE SCENE CAN DRAW HAS TO MEAN ITS OWN NAME.

    `_trim_spans` shortens anything wider than `max_covered_frac`, which keeps
    the FIGURE visible and makes the LABEL wrong: an `all_but_head` record
    trimmed from 0.80 to 0.55 is a body with its head, arms and chest in clear
    view, filed under the name of the pattern that hides them — a wrong label,
    which this module's whole occlusion argument is against. So every pattern
    with a non-zero default weight must fit under the default cap untrimmed,
    and the four that do not are weighted 0 rather than deleted (the bench
    still photographs all thirteen).
    """
    cap = tpp.DEFAULTS["max_covered_frac"]
    drawn = [k for k, v in tpp.DEFAULTS["occlusion"].items() if float(v) > 0.0]
    assert drawn, "no occlusion pattern is drawable at all"
    for name in sorted(drawn):
        spans = tpp._OCCLUSION[name]
        if spans == "lateral":
            continue
        span = sum(min(b, 1.0) - max(a, 0.0) for (a, b) in spans)
        assert span <= cap + 1e-6, (
            "%s covers %.2f of a body but the cap is %.2f: it would be trimmed "
            "and the record would carry a name that no longer describes it"
            % (name, span, cap))
    off = sorted(k for k, v in tpp.DEFAULTS["occlusion"].items()
                 if float(v) <= 0.0)
    print("    %d drawable pattern(s) all fit under the %.2f cap; %s are off"
          % (len(drawn), cap, off or "none"))


def test_21_nobody_is_placed_outside_the_track():
    """`min_intensity`, AT ALL THREE STATIONS.

    The gate the skill described for a fortnight and the code did not have. A
    synthetic corridor (intensity 1 inside a band, 0 outside) with wrecks
    straddling its edge: with the gate on, no station of any body may be
    outside the band, and with it off some are — otherwise the test would pass
    against an unwired gate, which is exactly the failure being fixed.
    """
    half = 30.0

    def inten(x, y):
        return 1.0 if abs(y) <= half else 0.0

    ctx = make_ctx(planks_on=False)
    ctx["intensity_at"] = inten

    def _stations(r):
        roll = tpp._lying_roll(r["pose"])
        ux, uy = tpp._body_axis(r["pose"], r["yaw"], roll)
        reach = float(r["reach_m"])
        return ((r["x"], r["y"]),
                (r["x"] + ux * reach * 0.5, r["y"] + uy * reach * 0.5),
                (r["x"] + ux * reach, r["y"] + uy * reach))

    out = {}
    for on in (True, False):
        cfg = tpp.resolve_cfg({"people": {
            "min_intensity": 0.5 if on else 0.0,
            # the keepouts are a different gate and would confuse the count
            "wreck_clear_m": 0.0, "house_clear_m": 0.0,
            "avoid_canopies": False, "avoid_blockers": False}})
        _h, _d, recs = tpp.plan_people(cfg, ctx, random.Random(91))
        out[on] = sum(1 for r in recs
                      for (px, py) in _stations(r) if abs(py) > half)
        if on:
            for r in recs:
                assert r.get("intensity") is not None, (
                    "the record does not carry the intensity it was gated on")
    assert out[True] == 0, (
        "%d station(s) outside the corridor with the gate ON" % out[True])
    assert out[False] > 0, (
        "the gate-off run put nothing outside the corridor either, so this "
        "test proves nothing about the gate")
    print("    stations outside the corridor: gate off %d -> gate on %d"
          % (out[False], out[True]))


def test_22_nobody_lies_inside_an_occluder_the_deck_cannot_see():
    """THE WRECK PILE, A STANDING HOUSE, A CROWN, AND THE SCOUR RELIEF.

    `covered_frac` counts only the boards this module lays, so the four things
    that can stand OVER a body and are not in it have to be refused instead.
    Same three stations. Off, some bodies land inside one; on, none does.
    """
    ctx = make_ctx()
    ctx["intact"] = [(w["x"] + 40.0, w["y"] + 40.0) for w in ctx["wrecks"]]
    ctx["canopies"] = [(w["x"] - 9.0, w["y"] + 9.0, 4.2) for w in ctx["wrecks"]]
    ctx["blockers"] = [(w["x"] + 11.0, w["y"] - 11.0, 2.0, "in_relief")
                       for w in ctx["wrecks"]]
    ko = tpp._Keepout(tpp._blocker_list(tpp.resolve_cfg({}), ctx))
    assert len(ko) > 0

    def _inside(recs):
        n = 0
        for r in recs:
            roll = tpp._lying_roll(r["pose"])
            ux, uy = tpp._body_axis(r["pose"], r["yaw"], roll)
            reach = float(r["reach_m"])
            for t in (0.0, 0.5, 1.0):
                if ko.hit(r["x"] + ux * reach * t, r["y"] + uy * reach * t):
                    n += 1
                    break
        return n

    on = {"min_intensity": 0.0}
    off = {"min_intensity": 0.0, "wreck_clear_m": 0.0, "house_clear_m": 0.0,
           "avoid_canopies": False, "avoid_blockers": False}
    got = {}
    for name, over in (("on", on), ("off", off)):
        _h, _d, recs = tpp.plan_people(tpp.resolve_cfg({"people": over}), ctx,
                                       random.Random(91))
        got[name] = (_inside(recs), len(recs))
    assert got["on"][0] == 0, (
        "%d of %d bodies lie inside an occluder with the keepouts ON"
        % got["on"])
    assert got["off"][0] > 0, (
        "the keepouts-off run put nothing inside an occluder either, so this "
        "test proves nothing")
    print("    bodies inside an unseen occluder: keepouts off %d/%d -> on "
          "%d/%d" % (got["off"] + got["on"]))


def main():
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    bad = 0
    for name, fn in fns:
        try:
            fn()
            print("PASS %s" % name)
        except AssertionError as exc:
            bad += 1
            print("FAIL %s\n     %s" % (name, exc))
    print("\n%d/%d passed" % (len(fns) - bad, len(fns)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
