"""The three things the A/B/C unification rests on.

Each of these is silent when it breaks — nothing raises, scenes still build,
and the damage is only visible to someone who looks at a frame or diffs two
libraries. Hence a test each.

1. ONE FIELD OBJECT SERVES BOTH QUESTIONS. `where did it hit` and `how thickly
   does debris lie` used to be two functions that each dispatched on
   `field.kind` and each rebuilt the geometry. They could drift — a corridor
   for the damage and a slightly different corridor for the scour — and the
   only symptom is debris that misses the track it belongs to.

2. THE BURN CLOCK HAS ONE DEFINITION. The live driver walks it forward to
   answer "what is this fuel doing now"; the bake walks it to answer "when
   does it change". Two statements of the same schedule is how a baked scene
   ends up playing a different fire from the one the launch script plays.

3. WHICH SCRIPT WRECKS A BUILDING IS THE REGISTRY'S ANSWER. `Disaster` reads
   `mesh_damage.DAMAGE_SCRIPTS`; it must not shadow it, or Stage A bakes a
   library the live path would never have produced.
"""

import os
import random
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import compile_disaster as cd                                    # noqa: E402
from disaster import field as F                                  # noqa: E402
from disaster import fire as FIRE                                # noqa: E402
from disaster import kinds                                       # noqa: E402

REGION = (-400.0, -400.0, 400.0, 400.0)

FIELD_CFGS = [
    {"kind": "uniform", "inside": 1.0},
    {"kind": "uniform", "inside": 0.0},
    {"kind": "radial", "center": [10.0, -20.0], "radius_m": 220.8,
     "falloff_m": 40.0, "inside": 1.0, "outside": 0.0},
    {"kind": "radial", "center": [0.0, 0.0], "radius_m": 360.0,
     "falloff_m": 440.0, "inside": 1.0, "outside": 0.45},
    {"kind": "path", "width_m": 90.0, "falloff_m": 60.0, "heading_deg": 31.0},
    {"kind": "path", "points": [[-300, -300], [0, 50], [280, 300]],
     "width_m": 60.0, "falloff_m": 40.0},
    {"kind": "ellipse", "origin_m": [-100.0, -80.0], "heading_deg": 45.0,
     "head_mps": 1.2, "flank_mps": 0.3, "back_mps": 0.1, "duration_s": 600.0},
]


def _samples(n=400, seed=11):
    rng = random.Random(seed)
    return [(rng.uniform(-500, 500), rng.uniform(-500, 500)) for _ in range(n)]


# --------------------------------------------------------------------------
# 1. the field
# --------------------------------------------------------------------------

@pytest.mark.parametrize("cfg", FIELD_CFGS, ids=lambda c: c["kind"])
def test_field_is_callable_with_bounds(cfg):
    """A field is a drop-in for the closure it replaced: callable, with
    `.lo` / `.hi` bounds that actually bound it."""
    f = F.make_damage_field(cfg, REGION)
    assert isinstance(f, F.DamageField)
    for x, y in _samples():
        v = f(x, y)
        assert f.lo - 1e-9 <= v <= f.hi + 1e-9, (cfg["kind"], x, y, v)
        assert f.intensity(x, y) == v


@pytest.mark.parametrize("cfg", FIELD_CFGS, ids=lambda c: c["kind"])
def test_scour_comes_off_the_same_field(cfg):
    """`make_scour_density` must be the field's own density, not a second
    implementation of the same geometry — see (1) in the module docstring."""
    f = F.make_damage_field(cfg, REGION)
    g = F.make_scour_density(cfg, REGION, shape=1.6)
    for x, y in _samples():
        assert g(x, y) == f.density(x, y, 1.6)
        assert 0.0 <= g(x, y) <= 1.0


def test_uniform_bounds_ignore_outside():
    """A uniform field is `inside` everywhere, so `outside` must not widen its
    bounds — `hi <= 0` is how callers detect "nothing happened"."""
    f = F.make_damage_field({"kind": "uniform", "inside": 0.0,
                             "outside": 1.0}, REGION)
    assert (f.lo, f.hi) == (0.0, 0.0)


def test_unknown_field_kind_raises():
    with pytest.raises(ValueError):
        F.make_damage_field({"kind": "spiral"}, REGION)


# --------------------------------------------------------------------------
# 2. the burn clock
# --------------------------------------------------------------------------

@pytest.mark.parametrize("flames", [True, False])
@pytest.mark.parametrize("ignition_s,flame_s,smoulder_s",
                         [(6.0, 120.0, 240.0), (0.0, 120.0, 240.0),
                          (6.0, 0.0, 240.0), (6.0, 120.0, 0.0)])
def test_breakpoints_reconstruct_the_live_clock(flames, ignition_s, flame_s,
                                                smoulder_s):
    """Stepping the baked breakpoints must land on exactly the state the live
    driver would report, at any time — see (2)."""
    cfg = dict(FIRE.DEFAULTS, ignition_s=ignition_s, flame_s=flame_s,
               smoulder_s=smoulder_s)
    rng = random.Random(4)
    for _ in range(40):
        t_ig = rng.uniform(-300.0, 300.0)
        bps = FIRE.burn_breakpoints(t_ig, cfg, flames)
        assert bps == sorted(bps), bps
        for t in [t_ig - 1.0] + [rng.uniform(-400, 900) for _ in range(30)]:
            want = FIRE.state_at(t, t_ig, cfg, flames)
            got = FIRE.UNBURNT
            for bt, st in bps:
                if t >= bt:
                    got = st
            assert got == want, (t, t_ig, bps)


def test_baked_schedule_matches_the_live_driver():
    """The end-to-end promise of `bake_emitters`: the timeSamples on the stage
    say what a `WildfireDriver` would have said, at every time."""
    from pxr import Gf, Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    rng = random.Random(1)
    placements = []
    for i in range(30):
        x, y = rng.uniform(-200, 200), rng.uniform(-200, 200)
        path = "/World/tree_{0:03d}".format(i)
        cube = UsdGeom.Cube.Define(stage, path)
        cube.GetSizeAttr().Set(6.0)
        UsdGeom.Xformable(cube).AddTranslateOp().Set(Gf.Vec3d(x, y, 3.0))
        placements.append({"category": "street_tree", "x_m": x, "y_m": y,
                           "prim_path": path})

    cfg = {"origin_m": [-200.0, -200.0], "heading_deg": 45.0, "head_mps": 1.2,
           "duration_s": 600.0, "start_offset_frac": 0.35,
           "max_emitters": 30, "emitter_spacing_m": 0.0}
    rep = FIRE.bake_emitters(stage, placements, cfg, verbose=False)
    assert rep["emitters"] > 0

    merged = dict(FIRE.DEFAULTS)
    merged.update(cfg)
    tcps = stage.GetTimeCodesPerSecond()
    checked = 0
    for prim in stage.Traverse():
        meta = prim.GetCustomDataByKey("airstack:fire")
        if not meta:
            continue
        t_ig = meta["t_ignite_s"]
        inten = meta["intensity"]
        flames = meta["will_flame"]
        for t in (0, 1, 5, 20, 60, 120, 300, 600, 900):
            st = FIRE.state_at(t, t_ig, merged, flames)
            tc = t * tcps
            assert bool(prim.GetAttribute("enabled").Get(tc)) == \
                (st != FIRE.UNBURNT), (prim.GetPath(), t)
            if st != FIRE.UNBURNT:
                e = FIRE.STATE_EMISSION[FIRE._STATE_VIS[st]]
                for name, want in (("fuel", e["fuel"] * inten),
                                   ("smoke", e["smoke"] * inten),
                                   ("temperature", e["temperature"])):
                    got = prim.GetAttribute(name).Get(tc)
                    assert abs(got - want) < 1e-5, (prim.GetPath(), t, name)
            checked += 1
    assert checked > 0

    # The rig itself has to survive into the file, or there is nothing for the
    # schedule to drive.
    for path in ("/World/flow/flowSimulate",
                 "/World/flow/flowSimulate/advection",
                 "/World/flow/flowOffscreen/colormap",
                 "/World/flow/flowRender/rayMarch"):
        assert stage.GetPrimAtPath(path).IsValid(), path

    # And the renderer state that is NOT USD has to be recorded, because a
    # consumer who does not apply it sees an empty sky.
    meta = stage.GetPrimAtPath("/World/flow").GetCustomDataByKey("airstack:flow")
    assert meta["baked"] is True
    assert meta["carb"]["rtx/flow/enabled"] is True
    assert meta["time_codes_per_second"] == tcps


def test_bake_is_deterministic():
    """Same config, same seed, same burn — the whole point of baking it."""
    from pxr import Usd, UsdGeom

    def once():
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")
        pls = [{"category": "tree", "x_m": float(i % 7) * 30.0,
                "y_m": float(i // 7) * 30.0,
                "prim_path": "/World/t{0}".format(i)} for i in range(28)]
        for p in pls:
            UsdGeom.Xform.Define(stage, p["prim_path"])
        FIRE.bake_emitters(stage, pls, {"emitter_spacing_m": 0.0},
                           verbose=False)
        return [(str(p.GetPath()), p.GetCustomDataByKey("airstack:fire"))
                for p in stage.Traverse()
                if p.GetCustomDataByKey("airstack:fire")]

    assert once() == once()


# --------------------------------------------------------------------------
# 3. the registry
# --------------------------------------------------------------------------

def test_every_compiled_disaster_has_a_kind():
    """A type the compiler knows must be a type the stages know."""
    for name in cd.DISASTERS:
        assert kinds.get(name).name == name, name


def test_default_field_kinds_are_real():
    for name in kinds.names():
        d = kinds.get(name)
        assert d.default_field_kind in F.FIELDS, (name, d.default_field_kind)


def test_get_accepts_a_config_and_degrades():
    assert kinds.get({"disaster": {"type": "fire"}}).name == "fire"
    assert kinds.get({"type": "earthquake"}).name == "earthquake"
    assert kinds.get(None).name == "none"
    assert kinds.get("volcano").name == "none"
    assert kinds.get("fire") is kinds.get("fire")


def test_damage_script_is_the_registry_not_a_copy():
    """See (3). `Disaster.damage_script` must be a view on
    `mesh_damage.DAMAGE_SCRIPTS`, so wiring a script in one place wires it in
    both."""
    from disaster import mesh_damage as md

    for name in kinds.names():
        assert kinds.get(name).damage_script == md.DAMAGE_SCRIPTS.get(name)


def test_compiled_field_kind_wins_over_the_default():
    """`Disaster.field` must not substitute its own shape for the config's.

    Fire compiles to a `radial` burn scar even though `EllipseField` exists and
    is what the front actually is; silently swapping in the ellipse here would
    move every building's damage level.
    """
    dis = {"type": "fire", "field": {"kind": "radial", "center": [0.0, 0.0],
                                     "radius_m": 100.0, "falloff_m": 10.0}}
    assert isinstance(kinds.get("fire").field(dis, REGION), F.RadialField)
    # ...and with no `field` block at all it falls back to its own default.
    assert isinstance(kinds.get("tornado").field({"type": "tornado"}, REGION),
                      F.PathField)


def test_only_fire_burns_vegetation():
    """`chars_vegetation` is what Stage A passes to `vegetation.burn_tree`.
    Everything else uses the same felling geometry with no soot on it."""
    assert kinds.get("fire").chars_vegetation is True
    assert not any(kinds.get(n).chars_vegetation
                   for n in kinds.names() if n != "fire")


def test_only_fire_authors_stage_b_extras():
    """Every other type's whole effect is in the placement list, so its Stage B
    hook must be a no-op — a type that quietly authored prims here would not be
    reproducible from the placement list the tools all read."""
    for name in kinds.names():
        if name == "fire":
            continue
        assert kinds.get(name).bake_stage_b(None, {}, []) == {}
        assert kinds.get(name).attach_runtime(None) == 0
