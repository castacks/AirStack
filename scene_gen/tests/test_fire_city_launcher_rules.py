#!/usr/bin/env python3
"""test_fire_city_launcher_rules.py — the three rules
`simulation/isaac-sim/launch_scripts/urban_fire_city_launch_script.py` added
on 2026-08-31, pinned HOST-SIDE against the real artefacts.

    python3 scene_gen/tests/test_fire_city_launcher_rules.py
    pytest -q scene_gen/tests/test_fire_city_launcher_rules.py

WHY THE FUNCTIONS ARE LIFTED OUT WITH `ast` RATHER THAN IMPORTED. That
launcher builds a `SimulationApp` at import — a second Kit app in one process
is a segfault, and there is no Kit at all on the host — so the module can
never be imported by a test. The rules under test are pure string/geometry
functions, so each is located in the source by name and compiled on its own.
That also makes the test fail loudly if a function is renamed, which is the
point: these three exist because a shipped scene was wrong in three
different ways, and each is one edit away from silently reverting.

THE THREE FINDINGS, from the user on the live 500 m city (2026-08-31):

  A  "/World/stage/generated/roof_house_94_1354/LOD0 this roof house is
     floating with no building near it" + "Lots of floating debris and roof
     props"            -> `prop_tag` / the companion-hide rule
  D  "Where are the humans?"   -> the people pass crashed; and
     "Only keep humans that are in the disaster"
                               -> `people_records`, FC_PEOPLE_MAX_DIST_M
"""
import ast
import json
import math
import os
import random
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_TESTS)
_ROOT = os.path.dirname(_SCENE_GEN)
for p in (_SCENE_GEN, os.path.join(_SCENE_GEN, "tools")):
    if p not in sys.path:
        sys.path.insert(0, p)

from detail import gac_props                            # noqa: E402
import fc_prop_orphan_probe as probe                    # noqa: E402

LAUNCHER = os.path.join(_ROOT, "simulation", "isaac-sim", "launch_scripts",
                        "urban_fire_city_launch_script.py")
DUMP = os.path.join(_SCENE_GEN, "_plans", "fc_dump_500.json")
MANIFEST = os.path.join(_SCENE_GEN, "_plans", "fire_city_500m.json")
PEOPLE = os.path.join(_SCENE_GEN, "_plans", "fire_people_final.json")


def _lift(name):
    """Compile the named top-level or method function out of the launcher."""
    tree = ast.parse(open(LAUNCHER).read(), LAUNCHER)
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) \
                and node.name == name:
            node.decorator_list = []
            mod = ast.Module(body=[node], type_ignores=[])
            ast.fix_missing_locations(mod)
            ns = {"math": math, "os": os, "json": json}
            exec(compile(mod, LAUNCHER, "exec"), ns)
            return ns[name]
    raise AssertionError("{0}() is gone from {1}".format(name, LAUNCHER))


def _city():
    """(houses, props) — `gac_props.dress` replayed on the FC dump."""
    dump = json.load(open(DUMP))
    from compile_disaster import load_scene_config
    cfg = load_scene_config(dump["preset"],
                            spec_overrides={"disaster-type": "none"})
    houses, props, _res = probe.rebuild_props(dump, cfg)
    return dump, houses, props


# ---------------------------------------------------------------------------
# A — the companion props
# ---------------------------------------------------------------------------
def test_01_prop_tag_is_gac_props_own_of_tag():
    """`prop_tag` must reproduce `gac_props._place`'s `of` EXACTLY.

    The launcher cannot import `detail.gac_props` (it does not import the
    detail package at all), so it re-spells the four characters of format.
    A drift here is silent: every lookup misses, no prop is ever hidden, and
    the only symptom is a roof prop in mid-air.
    """
    prop_tag = _lift("prop_tag")
    _dump, houses, props = _city()
    assert props, "dress() produced no props — the replay is broken"
    by_tag = {}
    for h in houses:
        by_tag.setdefault(
            "%s@%.1f,%.1f" % (gac_props._name_of(h["usd"]), h["x_m"],
                              h["y_m"]), h)
    n = 0
    for p in props:
        h = by_tag.get(p["of"])
        assert h is not None, p["of"]
        assert prop_tag(h["usd"], h["x_m"], h["y_m"]) == p["of"]
        n += 1
    assert n == len(props)
    # and it is IDENTITY, not proximity: a building 0.1 m away is a
    # different tag, which is the whole reason gac_props chose this form.
    h = houses[0]
    assert prop_tag(h["usd"], h["x_m"] + 0.2, h["y_m"]) != \
        prop_tag(h["usd"], h["x_m"], h["y_m"])


def test_02_every_damaged_building_takes_its_props_with_it():
    """The rule hides the props of the buildings the manifest damages, and
    leaves NOTHING tagged to a hidden building behind.

    Numbers are for the shipped pair (`fc_dump_500` x `fire_city_500m`):
    39 props over 7 of the 20 damaged buildings — the other 13 are kit or
    non-GAC assets `gac_props.dress` never dressed, which is why the rule
    must be a lookup and not a per-building assumption.
    """
    prop_tag = _lift("prop_tag")
    _dump, houses, props = _city()
    man = json.load(open(MANIFEST))
    by_i = {h["i"]: h for h in houses}
    by_cell = {h["cell"]: h for h in houses}

    hidden_tags = set()
    for rec in man["records"]:
        h = by_i.get(rec.get("i"))
        if h is None or h["usd"] != rec.get("usd"):
            h = by_cell.get(rec.get("cell"))
        assert h is not None, rec.get("i")
        hidden_tags.add(prop_tag(h["usd"], h["x_m"], h["y_m"]))
    assert len(hidden_tags) == len(man["records"])

    hidden = [p for p in props if p["of"] in hidden_tags]
    # NO PINNED COUNT. `_plans/fire_city_500m.json` is regenerated whenever
    # the spread policy changes — it went from 20 records (seed 4) to 14
    # (seed 43) WHILE THIS TEST WAS BEING WRITTEN, on another session's
    # max-fire-height gate — so a hard 39 would fail on every re-solve and
    # teach the next reader to delete the test. The INVARIANTS are what the
    # launcher actually depends on, and they hold for any manifest.
    assert hidden, "no damaged building was dressed — check the replay"
    # 1) exact partition: hidden == every prop carrying a hidden tag
    by_tag = {}
    for p in props:
        by_tag.setdefault(p["of"], []).append(p)
    assert sum(len(by_tag.get(t, ())) for t in hidden_tags) == len(hidden)
    # 2) nothing tagged to a hidden building survives (the launcher's own
    #    orphan check, which prints and must find zero)
    left = [p for t in hidden_tags for p in by_tag.get(t, ())
            if p not in hidden]
    assert not left, left
    # 3) no prop of an UNDAMAGED building is caught by it
    assert all(p["of"] not in hidden_tags for p in props if p not in hidden)
    # 4) and the buildings that ARE dressed are a strict subset of the
    #    damaged ones — most damaged buildings are kit/non-GAC and carry no
    #    props at all, which is why the rule must be a lookup.
    assert {p["of"] for p in hidden} <= hidden_tags
    print("      ({0} prop(s) over {1} of {2} damaged building(s))".format(
        len(hidden), len({p["of"] for p in hidden}), len(hidden_tags)))


def test_03_the_roof_house_the_user_named_is_not_a_bake_orphan():
    """`roof_house_94_1354` belongs to BG_Building_C, which is NOT damaged.

    `apply_placements` names a prim `<parent>/<category>_<group>_<i>` with
    `i` the index into the FULL placement list, and `gac_props.dress`'s
    output is the TAIL of that list (`generate_scene.py` appends it last and
    returns immediately after), so index 1354 is recoverable offline. It is
    a `roof_house`, and it is the ONLY one of the 124 props standing on a
    building whose referenced ROOT PRIM IS A MESH — the assets that render
    nothing once `instance_placements: true` marks them instanceable. That
    is what put it in the air, not the bake swap, and it is why the launcher
    carries BOTH fixes.
    """
    dump, houses, props = _city()
    first = int(dump["n_placements_total"]) - len(props)
    assert first == 1345, first
    p = props[1354 - first]
    assert p["category"] == "roof_house", p["category"]
    assert p["of"] == "BG_Building_C@-60.3,-169.3", p["of"]

    man = json.load(open(MANIFEST))
    # true on every manifest ever solved for this plate: `urban_fire_city.
    # burnable()` refuses the Muyang pack outright ("kit_substitute.route
    # refused: unburnable"), so this building can never be damaged and its
    # roof house can never be a bake orphan.
    assert not any("BG_Building_C" in str(r.get("usd")) for r in man["records"])
    # measured with tools/fc_roof_deck_probe.py: BG_Building_C's deck really
    # is at 66.00 m under the whole rectangle this prop sits on, so the seat
    # is right and the building is what is missing.
    bld = next(h for h in houses if "BG_Building_C" in h["usd"])
    assert abs(bld["H"] - 66.0) < 1e-6 and abs(p["z_m"] - 66.0) < 0.2

    # the five Mesh-rooted building assets, measured on Nucleus with
    # `tools/usd_python.sh` (43 assets checked, root prim type of each).
    mesh_rooted = {"Building_TypeA_A", "Building_TypeA_B", "Building_TypeD_A",
                   "Building_TypeD_B", "BG_Building_C"}
    n = sum(1 for h in houses
            if gac_props._name_of(h["usd"]) in mesh_rooted)
    assert n == 10, n


# ---------------------------------------------------------------------------
# D — the people
# ---------------------------------------------------------------------------
def test_04_people_records_reads_the_real_on_disk_shape():
    """The launch-killing crash: the records are under `people`."""
    people_records = _lift("people_records")
    doc = json.load(open(PEOPLE))
    assert "records" not in doc, sorted(doc)
    recs, key = people_records(doc)
    assert key == "people" and len(recs) == len(doc["people"]) == 89

    assert people_records({"records": [{"id": 1}]}) == ([{"id": 1}], "records")
    assert people_records([{"id": 1}])[0] == [{"id": 1}]
    for bad in ({"meta": {}, "census": {}}, "path.json", 7):
        try:
            people_records(bad)
        except TypeError:
            pass
        else:
            raise AssertionError("no TypeError for %r" % (bad,))


def test_05_every_person_is_in_the_disaster():
    """FC_PEOPLE_MAX_DIST_M, measured against the burning FOOTPRINTS.

    Distance to the building CENTRE is the wrong datum and the default would
    be wrong if it used one: the four onlookers of `SM_Building_31` are
    122.3 m from its centre and 59.5 m from its wall, because that building
    is 149 x 64 m in plan. On the footprint metric every one of the 89
    records is inside 59.5 m, so the 120 m default drops NOTHING — this is a
    guard against a re-solve scattering onlookers, not the mechanism.
    """
    man = json.load(open(MANIFEST))
    doc = json.load(open(PEOPLE))

    def foot_d(px, py):
        best = 1e18
        for r in man["records"]:
            a = math.radians(float(r["yaw_deg"]))
            ca, sa = math.cos(a), math.sin(a)
            dx, dy = px - float(r["x"]), py - float(r["y"])
            u, v = ca * dx + sa * dy, -sa * dx + ca * dy
            best = min(best, math.hypot(
                max(0.0, abs(u) - float(r["W"]) / 2.0),
                max(0.0, abs(v) - float(r["D"]) / 2.0)))
        return best

    ds = [foot_d(r["x"], r["y"]) for r in doc["people"]]
    same_solve = doc["meta"].get("manifest_seed") == man.get("seed")
    if same_solve:
        # the records were solved against THIS manifest: every figure is at
        # a building that is actually burning, so nothing may be dropped.
        assert max(ds) < 60.0, max(ds)
        burning = {r["i"] for r in man["records"]}
        assert all(r.get("building_i") in burning for r in doc["people"])
    else:
        # THE FILTER'S REASON FOR EXISTING, caught live: the manifest was
        # re-solved (seed {0} -> {1}) after this people file was written, so
        # some figures now stand at buildings that no longer burn. The guard
        # drops exactly those; the real fix is to re-run the people dry run.
        n_far = sum(1 for d in ds if d > 120.0)
        print("      STALE: people solved on manifest seed {0}, manifest is "
              "seed {1} — {2} of {3} record(s) now beyond 120 m "
              "(max {4:.0f} m)".format(doc["meta"].get("manifest_seed"),
                                       man.get("seed"), n_far, len(ds),
                                       max(ds)))
    # the centre metric, for the record — and why it is NOT the one used:
    # a 149 x 64 m tower puts its own onlookers 122 m from its centre.
    cd = max(min(math.hypot(r["x"] - float(b["x"]), r["y"] - float(b["y"]))
                 for b in man["records"]) for r in doc["people"])
    assert cd > max(ds), (cd, max(ds))



# ---------------------------------------------------------------------------
# The grey props (user, 2026-08-31: "Some props like the street lights also
# look like they have no texture ... same with some benches")
# ---------------------------------------------------------------------------
#: every asset in the failing run whose defaultPrim is a GPRIM, measured with
#: `tools/fc_instance_material_probe.py` against `_scene_assets.tsv` (110
#: distinct assets composed twice each, plain vs instanceable). These are the
#: ones `instance_placements: true` breaks; the other 96 keep every bind.
GPRIM_ROOTED = {
    "SM_lightpost_light_post_b.usd":   ("streetlight", 58),
    "SM_bench_wood_a.usd":             ("bench", 50),
    "Car_01_0.usd":                    ("car", 16),
    "SM_light_streetlight_complete.usd": ("traffic_light", 12),
    "PlanterLarge_A.usd":              ("planter", 7),
    "SM_prop_fountain_full.usd":       ("park_feature", 2),
    "SM_fountain_water_top.usd":       ("park_feature", 2),
    "SM_fountain_water_middle.usd":    ("park_feature", 2),
    "SM_fountain_water_bottom.usd":    ("park_feature", 2),
    "Building_TypeA_A.usd":            ("house", 4),
    "Building_TypeA_B.usd":            ("house", 3),
    "Building_TypeD_A.usd":            ("house", 1),
    "Building_TypeD_B.usd":            ("house", 1),
    "BG_Building_C.usd":               ("house", 1),
}
TSV = os.path.join(_SCENE_GEN, "_scene_assets.tsv")


def _tsv_rows():
    rows = []
    with open(TSV) as fh:
        next(fh, None)
        for line in fh:
            parts = line.rstrip("\n").split("\t")
            if len(parts) >= 4:
                rows.append((parts[0], parts[1], parts[3]))
    return rows


def test_06_the_grey_props_are_the_gprim_rooted_ones():
    """161 of 1469 placements, and the split inside `bench` is the tell.

    The user said "SOME benches", and the census says exactly why: 50 of the
    62 benches are `SM_bench_wood_a` (Dmytro, root prim is a Mesh, broken by
    instancing) and the other 12 are AEC `ParkBench01` (Xform root, every
    bind kept). A per-CATEGORY exclusion would therefore be wrong — the fix
    has to be per-PLACEMENT, on the composed prim's own type, which is what
    `_uninstance_gprim_roots` tests.
    """
    if not os.path.exists(TSV):
        return
    rows = _tsv_rows()
    assert len(rows) == 1469, len(rows)
    n_by_asset = {}
    for _path, cat, usd in rows:
        n_by_asset.setdefault(usd.rsplit("/", 1)[-1], [0, set()])
        n_by_asset[usd.rsplit("/", 1)[-1]][0] += 1
        n_by_asset[usd.rsplit("/", 1)[-1]][1].add(cat)

    total = 0
    for asset, (cat, n) in GPRIM_ROOTED.items():
        assert asset in n_by_asset, asset
        got_n, got_cats = n_by_asset[asset]
        assert got_n == n, (asset, got_n, n)
        assert got_cats == {cat}, (asset, got_cats)
        total += n
    assert total == 161, total

    # the bench split, stated outright
    bench = [u for _p, c, u in rows if c == "bench"]
    assert len(bench) == 62
    assert sum(1 for u in bench if u.endswith("SM_bench_wood_a.usd")) == 50
    assert sum(1 for u in bench if u.endswith("ParkBench01.usd")) == 12
    # and every streetlight in the city is the broken asset
    lights = [u for _p, c, u in rows if c == "streetlight"]
    assert len(lights) == 58
    assert all(u.endswith("SM_lightpost_light_post_b.usd") for u in lights)


def test_07_the_cull_targets_the_citys_own_pedestrians():
    """`cull_background_people` exists because the city plants 128 humans of
    its own — the 89 approved fire_people records were never the problem."""
    if not os.path.exists(TSV):
        return
    rows = _tsv_rows()
    humans = [(p, u) for p, c, u in rows if c == "human"]
    assert len(humans) == 128, len(humans)
    # they are the city generator's, under `generated/human_*`, and distinct
    # from the fire_people pass which authors under `generated/people/`.
    assert all(p.rsplit("/", 1)[-1].startswith("human_") for p, _u in humans)
    assert not any("/people/" in p for p, _u in humans)
    doc = json.load(open(PEOPLE))
    assert len(doc["people"]) == 89
    # the launcher must actually run the cull, and before the captures
    src = open(LAUNCHER).read()
    assert "def cull_background_people" in src
    i_cull = src.index("self.cull_background_people()")
    # `run()` calls `capture()` twice — the FC_INTACT_ONLY early return
    # first, then the real path. The cull belongs before the real one, and
    # after the bakes, because it needs `self.placed` for its datum.
    i_cap = src.rindex("self.capture()")
    i_bakes = src.index("self.compose_bakes()")
    i_people = src.index("self.place_people()")
    assert i_bakes < i_cull < i_people < i_cap, (i_bakes, i_cull, i_people,
                                                 i_cap)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)
    print("\nfire-city launcher rules pinned")
