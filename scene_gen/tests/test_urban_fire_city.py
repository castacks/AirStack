#!/usr/bin/env python3
"""
test_urban_fire_city.py — the generated-city fire pass, pinned without Isaac.

`disaster/urban_fire_city.assemble` is the join between three things that were
never joined before: the city generator's placement list, the spread solve, and
`urban_fire.burn_monolith`. Every claim below is one that looks identical to
its own failure until a scene is built, which is a twenty-minute round trip:

  1. `burn_monolith` IS HANDED `holder=<prim_path>` FOR EVERY BUILDING, and
     never places one itself. If this regresses, each building is referenced a
     SECOND time under the fire scope with a differently-computed transform —
     a duplicate city half a metre off the first, which from any distance just
     reads as "the buildings look wrong".
  2. F0 BUILDINGS ARE NOT TOUCHED. An untouched building must keep its own
     materials; sooting it "a little" is what turns a moderate fire into a
     grey city.
  3. THE REPORTED TALLY IS THE SOLVE'S TALLY. The banner is the only thing
     anyone reads, so it has to count what was actually authored.
  4. THE ROOFTOP PLANT OF A BURNING BUILDING IS SOOTED, matched by the `of`
     identity tag `gac_props` writes — not by proximity, which on a real
     street finds the neighbour's tank first.
  5. THE LADDER IS A SHARE OF THE CITY AND THE CLOCK IS SOLVED FOR IT. Stating
     a rung as a fixed elapsed time is what broke once already: the same 195
     minutes involved 8 buildings from a poorly-connected origin and 63 from a
     well-connected one, and an unrelated layout edit moved "moderate" from
     21% of the city to 6% with no change to the fire code.
  6. THE LAUNCHER BUILDS A DOWNTOWN, not the 250 m suburb that `.env`'s
     inherited `SCENE_CONFIG` / `REGION_M` would otherwise select. That one is
     invisible from inside the scene: it builds cleanly, it just builds the
     wrong place.

RUNS WITHOUT ISAAC. `urban_fire` and `urban_fire_city` import no `pxr` at
module scope — every USD import in them is function-local — so only the few
names `assemble` itself reaches are stubbed, in the same style as
`tools/plan_png.py` and `test_districts_facing.py`.

USAGE
    python3 scene_gen/tests/test_urban_fire_city.py
    pytest -s scene_gen/tests/test_urban_fire_city.py
"""

import os
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

# `assemble` reaches `pxr.Sdf.Path` and `pxr.UsdGeom.Scope.Define`, and
# `quake.style_of` reaches nothing at all. Stub the two, exactly as plan_png
# does — nothing here touches USD or a GPU.
for _m in ("pxr", "pxr.Gf", "pxr.Sdf", "pxr.Usd", "pxr.UsdGeom",
           "pxr.UsdShade", "pxr.UsdSkel", "pxr.Vt", "pxr.UsdPhysics"):
    sys.modules.setdefault(_m, types.ModuleType(_m))
for _n in ("Gf", "Sdf", "Usd", "UsdGeom", "UsdShade", "UsdSkel", "Vt",
           "UsdPhysics"):
    setattr(sys.modules["pxr"], _n, sys.modules["pxr." + _n])
sys.modules["pxr.Sdf"].Path = lambda p: p
sys.modules["pxr.UsdGeom"].Scope = types.SimpleNamespace(
    Define=lambda stage, path: None)


class _StubXformable(object):
    def __init__(self, prim=None):
        self._prim = prim if prim is not None else _StubPrim()

    def GetPrim(self):
        return self._prim


class _StubPrim(object):
    def IsValid(self):
        return True


sys.modules["pxr.UsdGeom"].Xformable = _StubXformable
sys.modules["pxr.UsdGeom"].Xform = types.SimpleNamespace(
    Define=lambda stage, path: _StubXformable())

from disaster import urban_fire as uf                        # noqa: E402
from disaster import urban_fire_city as ufc                  # noqa: E402
from disaster import urban_fire_spread as ufs                # noqa: E402

FAILURES = []


def check(label, cond, detail=""):
    print("  {0:<4} {1}{2}".format("ok" if cond else "FAIL", label,
                                   "" if cond else "  -- " + detail))
    if not cond:
        FAILURES.append(label)


# ---------------------------------------------------------------------------
# A city, without a stage
# ---------------------------------------------------------------------------
class _Prim(object):
    def __init__(self, path):
        self._p = path

    def IsValid(self):
        return True


class _Stage(object):
    """Just enough stage for `assemble`: prims exist, nothing else."""

    def __init__(self):
        self.asked = []

    def GetPrimAtPath(self, path):
        self.asked.append(str(path))
        return _Prim(path)


def _city(n_x=6, n_y=6, pitch=26.0, gap_h=42.0):
    """A grid of buildings tight enough for radiation to cross within a row
    and wide enough that only a brand crosses between rows — the same shape a
    real block grid has, so the solve exercises all three mechanisms."""
    out = []
    for iy in range(n_y):
        for ix in range(n_x):
            h = 14.0 if (ix + iy) % 4 == 0 else 34.0
            out.append({
                "usd": "omni://x/SM_Building_{0:02d}.usd".format(1 + ix % 9),
                "category": "house", "scale": 0.01, "axis_up": "Z",
                "x_m": -0.5 * (n_x - 1) * pitch + ix * pitch,
                "y_m": -0.5 * (n_y - 1) * gap_h + iy * gap_h,
                "z_m": 0.0, "yaw_deg": 90.0 * (ix % 4),
                "prim_path": "/World/stage/generated/house_{0}_{1}".format(
                    ix, iy),
                "_dims": (20.0, 18.0, h)})
    # one rooftop tank, carrying the identity tag `gac_props._place` writes
    b0 = out[0]
    out.append({"usd": "omni://x/SM_Water_Tank_02.usd", "category": "roof_tank",
                "x_m": b0["x_m"], "y_m": b0["y_m"], "z_m": 34.0,
                "yaw_deg": 0.0, "scale": 1.0,
                "prim_path": "/World/stage/generated/roof_tank_9_900",
                "of": "SM_Building_01@{0:.1f},{1:.1f}".format(b0["x_m"],
                                                             b0["y_m"])})
    return out


def _install_spies(monkey):
    """Replace the three things `assemble` calls into the stage with spies."""
    calls = {"burn": [], "soot": [], "materials": 0}

    def _materials(stage, parent):
        calls["materials"] += 1
        return {"_stub": True}

    def _burn_monolith(stage, parent, usd, x, y, yaw, dims, level, rng, nrng,
                       mats, tag, **kw):
        calls["burn"].append({"parent": parent, "usd": usd, "level": level,
                              "tag": tag, "dims": dims, "kw": kw})
        return {"loose": [], "static_extra": [], "velocity": {},
                "authored": ["a", "b"], "notes": ["note " + tag],
                "fire": {"level": level}}

    def _darken(stage, path, k, **kw):
        calls["soot"].append((path, k))
        return 1

    monkey.append((uf, "materials", uf.materials))
    monkey.append((uf, "burn_monolith", uf.burn_monolith))
    monkey.append((uf, "_darken_asset", uf._darken_asset))
    uf.materials = _materials
    uf.burn_monolith = _burn_monolith
    uf._darken_asset = _darken

    monkey.append((ufc, "_bbox_dims", ufc._bbox_dims))
    ufc._bbox_dims = lambda stage, prim, p: p["_dims"]
    return calls


def _restore(monkey):
    for obj, name, val in monkey:
        setattr(obj, name, val)


def run_assemble(level="moderate", extra=(), **kw):
    monkey, calls = [], None
    placements = _city() + list(extra)
    stage = _Stage()
    config = {"layout": {"region_m": [500.0, 500.0]}}
    try:
        calls = _install_spies(monkey)
        stats = ufc.assemble(stage, config, placements,
                             parent="/World/stage/generated", level=level,
                             flow=False, verbose=False, **kw)
    finally:
        _restore(monkey)
    return stats, calls, placements


# ---------------------------------------------------------------------------
# 1-3: what assemble does with the solve
# ---------------------------------------------------------------------------
def test_burns_in_place():
    print("\n1. burn_monolith is handed an EXISTING prim, never asked to place")
    stats, calls, placements = run_assemble()
    paths = {p["prim_path"] for p in placements
             if p.get("category") == "house"}
    check("at least one building burnt", len(calls["burn"]) > 0,
          "the ladder produced no involved buildings at all")
    missing = [c for c in calls["burn"] if "holder" not in c["kw"]]
    check("every call passes holder", not missing,
          "{0} call(s) with no holder — those would REFERENCE the asset a "
          "second time".format(len(missing)))
    bad = [c for c in calls["burn"] if c["kw"].get("holder") not in paths]
    check("every holder is a placement's own prim_path", not bad,
          str([c["kw"].get("holder") for c in bad][:3]))
    # the placement transform must not be re-derived: a placed building is
    # positioned already, so the centre/base offsets must be inert
    off = [c for c in calls["burn"]
           if any(c["dims"].get(k, 0.0) for k in ("cx", "cy", "zmin"))]
    check("no centroid/base offset is passed for a placed building", not off)
    scopes = {c["parent"] for c in calls["burn"]}
    check("art is authored under the fire scope, not the building",
          all(s.startswith("/World/stage/generated/urban_fire/")
              for s in scopes), str(sorted(scopes)[:3]))
    check("materials built exactly once", calls["materials"] == 1,
          str(calls["materials"]))


def test_f0_untouched():
    print("\n2. F0 buildings are not touched, and the tally is the solve's")
    stats, calls, _ = run_assemble()
    check("no F0 building was burnt",
          all(c["level"] != "F0" for c in calls["burn"]),
          "an untouched building was sooted anyway")
    check("burn count == involved", len(calls["burn"]) == stats["involved"],
          "{0} calls vs involved {1}".format(len(calls["burn"]),
                                             stats["involved"]))
    check("tally sums to the building count",
          sum(stats["tally"].values()) == stats["buildings"],
          "{0} vs {1}".format(sum(stats["tally"].values()),
                              stats["buildings"]))
    check("involved == buildings - F0",
          stats["involved"] == stats["buildings"] - stats["tally"].get("F0", 0))
    check("one record per burnt building",
          len(stats["records"]) == len(calls["burn"]))
    lv = {r["level"] for r in stats["records"]}
    check("every record carries a real level", lv <= set(ufs.LEVELS), str(lv))


def test_ladder_is_a_ladder():
    print("\n3. each rung HITS ITS TARGET SHARE, and the clock follows")
    seen, n = [], None
    for name in ("light", "moderate", "severe"):
        stats, _c, _p = run_assemble(level=name)
        n = stats["buildings"]
        frac = stats["involved"] / float(n)
        seen.append((name, stats["involved"], stats["elapsed_min"], frac))
        want = ufc.LADDER[name]["involved_frac"]
        print("     {0:<9} T+{1:>3.0f} min  involved {2}/{3} = {4:.0%} "
              "(want {5:.0%})".format(name, stats["elapsed_min"],
                                      stats["involved"], n, frac, want))
        # one building's worth of quantisation either side
        check("{0} lands on its target share".format(name),
              abs(frac - want) <= 1.5 / n,
              "{0:.0%} vs {1:.0%} over {2} buildings".format(frac, want, n))
    check("the solved clock rises with the rung",
          [s[2] for s in seen] == sorted(s[2] for s in seen), str(seen))
    check("involvement rises with the rung",
          [s[1] for s in seen] == sorted(s[1] for s in seen), str(seen))
    check("severe still leaves most of the city intact",
          seen[-1][1] < n, "{0} of {1} involved".format(seen[-1][1], n))
    check("every rung has an F0 population to contrast against",
          all(s[1] < n for s in seen), str(seen))


# ---------------------------------------------------------------------------
# 4: the roof plant
# ---------------------------------------------------------------------------
def test_roof_plant_adopted():
    print("\n4. the rooftop plant of a burning building is sooted by identity")
    b0 = _city()[0]
    # A DECOY: same asset, half a metre away, NO `of` tag. If adoption ever
    # falls back to proximity this is the prop it picks up, because it is
    # nearer the burning building than that building's own far parapet.
    decoy = {"usd": "omni://x/SM_Water_Tank_02.usd", "category": "roof_tank",
             "x_m": b0["x_m"] + 0.5, "y_m": b0["y_m"], "z_m": 34.0,
             "yaw_deg": 0.0, "scale": 1.0,
             "prim_path": "/World/stage/generated/roof_tank_decoy"}
    # ignite ON the tank's building, so that building is certain to burn
    stats, calls, _p = run_assemble(level="severe", extra=[decoy],
                                    origin=(b0["x_m"], b0["y_m"]))
    tank = "/World/stage/generated/roof_tank_9_900"
    sooted = [p for p, _k in calls["soot"]]
    check("the tank on a burning roof was sooted", tank in sooted,
          "sooted: {0}".format(sooted[:4]))
    ks = [k for p, k in calls["soot"] if p == tank]
    check("sooted with a multiplier < 1",
          bool(ks) and all(0.0 < k < 1.0 for k in ks), str(ks))
    check("the untagged decoy 0.5 m away was NOT adopted",
          "/World/stage/generated/roof_tank_decoy" not in sooted,
          "adoption fell back to proximity")


# ---------------------------------------------------------------------------
# 5: the tables
# ---------------------------------------------------------------------------
def test_tables():
    print("\n5. the ladder and the construction-type rule")
    check("module self-check passes", not ufc.check(verbose=False))
    check("every rung is at or under the wind saturation",
          all(r["wind"][1] <= ufc.WIND_SATURATION_MPS
              for r in ufc.LADDER.values()))
    check("the rungs are distinct, increasing shares",
          [ufc.LADDER[k]["involved_frac"]
           for k in ("light", "moderate", "severe")] == [0.10, 0.22, 0.40])
    check("no rung states a clock",
          all("elapsed_min" not in r for r in ufc.LADDER.values()),
          "a fixed elapsed_min is what this design exists to avoid")
    check("numeric aliases point at the named rungs",
          ufc.LADDER["2"] is ufc.LADDER["moderate"])
    check("a 55 m tower is rc",
          ufc._btype({"usd": "SM_Building_01.usd", "H": 55.0}) == "rc")
    check("a 12 m building is urm",
          ufc._btype({"usd": "SM_Building_02.usd", "H": 12.0}) == "urm")
    check("a brownstone is urm at any height",
          ufc._btype({"usd": "aec/brownstone/b.usd", "H": 40.0}) == "urm")
    check("only urm can ever reach F5",
          ufs.level_for_age(ufs.T_OUT + 60, "rc",
                            __import__("random").Random(0)) == "F4")
    # REGRESSION: `summarise` used to `rows.sort()` on tuples whose second
    # element is a dict, so any TIE on the sort key raised `TypeError: '<' not
    # supported between instances of 'dict' and 'dict'`. Every building the
    # fire never reached ties at -1e9 — a 100 m block never had two, this
    # 500 m city has eighteen, and it killed the launch after the layout had
    # already been built.
    far = [{"x": 400.0 * k, "y": 0.0, "W": 10.0, "D": 10.0, "H": 20.0,
            "yaw": 0.0, "style": "b%d" % k} for k in range(4)]
    fplan = ufs.solve(far, 0, 3600.0)
    check("more than one building is out of reach in the fixture",
          sum(1 for q in fplan if q["t_ignite"] is None) >= 2)
    try:
        lines = ufs.summarise(far, fplan, 3600.0)
    except TypeError as exc:
        check("summarise survives several unreached buildings", False, str(exc))
    else:
        check("summarise survives several unreached buildings",
              len(lines) == len(far), str(lines))


# ---------------------------------------------------------------------------
# 6: the launcher's env resolution, sliced out of the launch script
# ---------------------------------------------------------------------------
_LAUNCHER = os.path.normpath(os.path.join(
    _SCENE_GEN, "..", "simulation", "isaac-sim", "launch_scripts",
    "downtown_fire_launch_script.py"))


def _slice_launcher(env):
    """Exec ONLY the pure-Python helpers out of the launch script.

    The module cannot be imported: it builds a `SimulationApp` at import, and
    a second Kit app in one process is a segfault. So the four functions that
    decide WHICH SCENE GETS BUILT are lifted out by AST and run against a
    controlled environment — which is the only way to test them without a GPU.
    """
    import ast as _ast

    src = open(_LAUNCHER).read()
    tree = _ast.parse(src)
    want = {"_env", "_resolve_config", "_spec_overrides", "_fire_kwargs"}
    body = [n for n in tree.body
            if isinstance(n, _ast.FunctionDef) and n.name in want]
    missing = want - {n.name for n in body}
    assert not missing, "launcher lost {0}".format(missing)
    ns = {"os": types.SimpleNamespace(environ=dict(env),
                                      path=os.path),
          "ufc": ufc, "print": lambda *a, **k: None}
    _mod = _ast.Module(body=body, type_ignores=[])
    exec(compile(_mod, _LAUNCHER, "exec"), ns)
    ns["SCENE_CONFIG"], ns["_SHARED_OK"] = ns["_resolve_config"]()
    ns["UF_LEVEL"] = ns["_env"]("UF_LEVEL", "moderate")
    ns["UF_SEED"] = int(ns["_env"]("UF_SEED", "21"))
    ns["UF_FLOW"] = ns["_env"]("UF_FLOW", "1") not in ("0", "false", "no")
    ns["UF_EMITTERS"] = int(ns["_env"]("UF_EMITTERS",
                                       str(ufc.FLOW_EMITTERS_PER_BUILDING)))
    ns["UF_FLOW_CELL"] = float(ns["_env"]("UF_FLOW_CELL", str(ufc.FLOW_CELL_M)))
    ns["UF_FLOW_BLOCKS"] = int(ns["_env"]("UF_FLOW_BLOCKS",
                                          str(ufc.FLOW_MAX_BLOCKS)))
    ns["UF_FLOW_BUDGET"] = int(ns["_env"]("UF_FLOW_BUDGET",
                                          str(ufc.FLOW_EMITTER_BUDGET)))
    return ns


def test_launcher_env():
    print("\n6. the launcher ignores the suburb scene `.env` hands every "
          "container")
    # exactly what `.env` ships today
    dotenv = {"SCENE_CONFIG": "suburb", "REGION_M": "250x250",
              "DISASTER_TYPE": "none"}
    ns = _slice_launcher(dotenv)
    check("inherited SCENE_CONFIG=suburb is refused",
          ns["SCENE_CONFIG"] == "downtown_gac", ns["SCENE_CONFIG"])
    check("inherited REGION_M=250x250 is refused with it",
          ns["_spec_overrides"]()["region_m"] == [500.0, 500.0],
          str(ns["_spec_overrides"]()))

    ns = _slice_launcher(dict(dotenv, SCENE_CONFIG="downtown_gac",
                              REGION_M="800"))
    check("a downtown SCENE_CONFIG is accepted",
          ns["SCENE_CONFIG"] == "downtown_gac")
    check("and its REGION_M travels with it",
          ns["_spec_overrides"]()["region_m"] == [800.0, 800.0])

    ns = _slice_launcher(dict(dotenv, UF_CONFIG="downtown_1000",
                              UF_REGION="640x640", UF_LAYOUT_SEED="7"))
    check("UF_CONFIG wins over the shared name",
          ns["SCENE_CONFIG"] == "downtown_1000")
    ov = ns["_spec_overrides"]()
    check("UF_REGION wins over REGION_M", ov["region_m"] == [640.0, 640.0],
          str(ov))
    check("UF_LAYOUT_SEED is passed through", ov.get("seed") == 7, str(ov))

    ns = _slice_launcher(dict(dotenv, UF_LEVEL="severe", UF_ELAPSED="123",
                              UF_WIND="200,6", UF_ORIGIN="10,-20",
                              UF_FLOW="0"))
    kw = ns["_fire_kwargs"]()
    check("level, elapsed, wind and origin all reach assemble",
          (kw["level"], kw["elapsed_min"], kw["wind"], kw["origin"])
          == ("severe", 123.0, (200.0, 6.0), (10.0, -20.0)), str(kw))
    check("UF_FLOW=0 turns Flow off", kw["flow"] is False, str(kw))
    import inspect
    accepted = set(inspect.signature(ufc.assemble).parameters)
    check("every kwarg is one assemble accepts", set(kw) <= accepted,
          "unknown: {0}".format(sorted(set(kw) - accepted)))
    # REGRESSION: the bench's 0.10 m cell over this plate asked for more GPU
    # than the city had left and Flow fell back to a 1x1x1 texture — a scene
    # that renders with NO SMOKE IN IT and reports success.
    check("the Flow cell defaults well above the bench's 0.10 m",
          kw["flow_cell_m"] >= 0.25, str(kw["flow_cell_m"]))
    check("the plate-wide emitter budget is bounded",
          0 < kw["flow_budget"] <= 96, str(kw["flow_budget"]))


# ---------------------------------------------------------------------------
# 7: burn_monolith's own dispatch
# ---------------------------------------------------------------------------
def test_burn_monolith_dispatch():
    print("\n7. burn_monolith places only when it is NOT given a holder")
    seen = {"place": 0, "burn": []}
    keep = (uf._mono_place, uf._burn_mono_on_stage)
    try:
        uf._mono_place = lambda *a, **k: seen.__setitem__("place",
                                                          seen["place"] + 1)
        uf._burn_mono_on_stage = (
            lambda stage, parent, holder, *a, **k:
            seen["burn"].append(holder) or {})
        dims = {"W": 20.0, "D": 18.0, "H": 34.0}
        uf.burn_monolith(_Stage(), "/f/b0", "u.usd", 0.0, 0.0, 0.0, dims,
                         "F3", None, None, {}, "b0",
                         holder="/World/stage/generated/house_1_1")
        check("a holder is burnt in place, nothing placed", seen["place"] == 0)
        check("and the burn runs on that exact prim",
              seen["burn"] == ["/World/stage/generated/house_1_1"],
              str(seen["burn"]))
        uf.burn_monolith(_Stage(), "/f/b1", "u.usd", 0.0, 0.0, 0.0, dims,
                         "F3", None, None, {}, "b1")
        check("without one, the asset IS placed (the bench path is intact)",
              seen["place"] == 1, str(seen["place"]))
        check("and burnt under the caller's own holder",
              seen["burn"][-1] == "/f/b1/mono", str(seen["burn"][-1]))
        try:
            uf.burn_monolith(_Stage(), "/f/b2", "u.usd", 0.0, 0.0, 0.0, dims,
                             "F3", None, None, {}, "b2", holder=None)
        except Exception as exc:                              # pragma: no cover
            check("holder=None is the same as omitting it", False, str(exc))
        else:
            check("holder=None is the same as omitting it", seen["place"] == 2)
    finally:
        uf._mono_place, uf._burn_mono_on_stage = keep


def main():
    print(__doc__.strip().splitlines()[1])
    test_burns_in_place()
    test_f0_untouched()
    test_ladder_is_a_ladder()
    test_roof_plant_adopted()
    test_tables()
    test_launcher_env()
    test_burn_monolith_dispatch()
    print("\n{0}".format("ALL OK" if not FAILURES
                         else "FAILED: " + ", ".join(FAILURES)))
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main())
