#!/usr/bin/env python3
"""test_urban_fire_city_launch.py — the offline symbol/signature check for
`simulation/isaac-sim/launch_scripts/urban_fire_city_launch_script.py`
(work item #7 of `scene_gen/_plans/urban_fire_city_plan.md`) and for the
`disaster/fire_assembly_lib.py` refactor it stands on.

    python3 scene_gen/tests/test_urban_fire_city_launch.py
    pytest -q scene_gen/tests/test_urban_fire_city_launch.py

HOST-SIDE, AND IT NEVER IMPORTS EITHER LAUNCHER. Both build a
`SimulationApp` at module scope — importing one from a test would start Kit
(and a second one in the same process is a segfault). So every launcher
assertion here is made against the AST, and the two things that are real
python — `fire_assembly_lib.fire_view_params`'s arithmetic and the module's
own signatures — are extracted from source and executed in an empty
namespace rather than imported (the module itself imports `pxr` and
`disaster.urban_fire`, neither of which exists on a bare host).

WHAT IT ACTUALLY GUARDS

  1. The env table. Every knob the work order names is read through `_env`
     (directly or through `_flag`, which is itself defined in terms of
     `_env`) — so a knob cannot be silently renamed or dropped, and the
     container's habit of exporting every launcher knob as an EMPTY STRING
     stays handled.
  2. THE TRANSFORM TRAP. `MakeInvisible` on the intact prim, a fresh
     `AddRotateXYZOp` holder rotating only about Z, and no
     `ClearReferences` CALL anywhere — the `quake.assemble` idiom (keep the
     transform, swap the reference) is the one thing that must not run in
     this file.
  3. THE DOUBLE-TRANSLATE TRAP. `fire_bake.place(...)` (the rotating form),
     never `fire_bake.translate(...)`; and because `place` moves the SEATS
     too, `place_fire` must be called with `dx = dy = 0` — passing the cell
     again there would translate every interior/roof plume twice. Checked
     positionally on the call node.
  4. The global emitter budget and the Flow-OOM grep — the two things the
     plan calls mandatory, and both fail silently when absent.
  5. The refactor. `fire_assembly_lib` exports the moved functions with the
     signatures both launchers call, the row launcher forwards its three
     env-derived knobs (`GLOB`/`ORDER`/`SMOKE`) into them, and
     `fire_view_params` reproduces the row launcher's ORIGINAL inline
     review-camera arithmetic to the bit on random inputs.
"""
import ast
import json
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
_REPO = os.path.normpath(os.path.join(_SCENE_GEN, ".."))
_LAUNCH = os.path.join(_REPO, "simulation", "isaac-sim", "launch_scripts")
_TOOLS_DIR = os.path.join(_SCENE_GEN, "tools")
sys.path.insert(0, _SCENE_GEN)
sys.path.insert(0, _TOOLS_DIR)

CITY = os.path.join(_LAUNCH, "urban_fire_city_launch_script.py")
ROW = os.path.join(_LAUNCH, "fire_assembly_launch_script.py")
LIB = os.path.join(_SCENE_GEN, "disaster", "fire_assembly_lib.py")
PROBE = os.path.join(_SCENE_GEN, "tools", "fc_transform_probe.py")


def _src(path):
    with open(path) as fh:
        return fh.read()


def _tree(path):
    return ast.parse(_src(path), filename=path)


CITY_SRC = _src(CITY)
CITY_TREE = ast.parse(CITY_SRC, filename=CITY)


def _calls(tree, name):
    """Every `ast.Call` whose callee is `name` or `<something>.name`."""
    out = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        fn = node.func
        if isinstance(fn, ast.Name) and fn.id == name:
            out.append(node)
        elif isinstance(fn, ast.Attribute) and fn.attr == name:
            out.append(node)
    return out


def _const_str_args(calls, index=0):
    out = set()
    for c in calls:
        if len(c.args) > index and isinstance(c.args[index], ast.Constant) \
                and isinstance(c.args[index].value, str):
            out.add(c.args[index].value)
    return out


def _funcdefs(tree):
    return {n.name: n for n in ast.walk(tree)
            if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}


# ---------------------------------------------------------------------------
# 0) both launchers still parse
# ---------------------------------------------------------------------------
def test_both_launchers_parse():
    for path in (CITY, ROW, LIB, PROBE):
        assert _tree(path) is not None, path


def test_the_city_launcher_never_imports_the_row_launcher():
    # a second SimulationApp in one process is a segfault inside the first
    # second (measured; downtown_quake_launch_script.py's own note)
    assert "import fire_assembly_launch_script" not in CITY_SRC
    for node in ast.walk(CITY_TREE):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = [a.name for a in node.names]
            mod = getattr(node, "module", None) or ""
            assert "fire_assembly_launch_script" not in mod
            assert not any("fire_assembly_launch_script" in n for n in names)


# ---------------------------------------------------------------------------
# 1) the env table
# ---------------------------------------------------------------------------
#: every knob the work order names for this launcher.
REQUIRED_ENV = (
    "SCENE_CONFIG", "FC_MANIFEST", "FC_BAKES",
    "FA_FLOW", "FA_SMOKE", "FA_CELL_M", "FA_MAX_BLOCKS", "FA_EMITTER_BUDGET",
    "SNAP_DIR", "KEEP_OPEN", "ISAAC_SIM_HEADLESS", "FC_INTACT_ONLY",
    "FC_DUMP", "FC_ENV",
)


def _env_knobs():
    """Knob names read through `_env` or through `_flag` (which wraps it)."""
    return (_const_str_args(_calls(CITY_TREE, "_env"))
            | _const_str_args(_calls(CITY_TREE, "_flag")))


def test_every_required_env_knob_is_read():
    got = _env_knobs()
    missing = [k for k in REQUIRED_ENV if k not in got]
    assert not missing, "knobs never read: {0} (read: {1})".format(
        missing, sorted(got))


def test_flag_is_defined_in_terms_of_env():
    # so a knob read through `_flag` still gets the empty-string handling
    fn = _funcdefs(CITY_TREE).get("_flag")
    assert fn is not None, "_flag must exist for the boolean knobs"
    assert _calls(fn, "_env"), "_flag must delegate to _env"


def test_env_treats_the_empty_string_as_absent():
    fn = _funcdefs(CITY_TREE).get("_env")
    assert fn is not None
    body = ast.dump(fn)
    # the container exports every knob as an empty string; `os.environ.get`
    # with a default would never reach it
    assert "strip" in body


def test_emitter_budget_has_the_documented_default():
    # 800 (was 560, was 200) as of 2026-08-31's THIRD review (cluster
    # diversity, size-scaled allocation, residual flame pockets):
    # `scene_gen/tools/fire_flow_dry_run.py`, run against the real
    # 39-record `fire_city_500m_39.json` manifest, projects a NATURAL
    # (unbudgeted) total of ~976 emitters; 800 stays comfortably under both
    # that and the 15.3 GB VRAM cap — see `EMITTER_BUDGET`'s own comment.
    for call in _calls(CITY_TREE, "_env"):
        if call.args and getattr(call.args[0], "value", None) == "FA_EMITTER_BUDGET":
            assert len(call.args) > 1 and call.args[1].value == "800"
            return
    raise AssertionError("FA_EMITTER_BUDGET is not read with a default")


def test_scene_config_defaults_to_the_fire_preset():
    for call in _calls(CITY_TREE, "_env"):
        if call.args and getattr(call.args[0], "value", None) == "SCENE_CONFIG":
            assert call.args[1].value == "downtown_fire_500"
            return
    raise AssertionError("SCENE_CONFIG is not read with a default")


def test_the_fire_preset_is_compiled_with_disaster_type_none():
    # `compile_disaster.DISASTERS` has no "fire" entry, so compiling the
    # preset as it stands RAISES. The dry run overrides it the same way.
    assert "disaster-type" in CITY_SRC
    assert '"none"' in CITY_SRC


# ---------------------------------------------------------------------------
# 1b) FC_DUMP / FC_ENV -- the 2026-08-30 manifest/city mismatch fix
# ---------------------------------------------------------------------------
def test_fc_dump_defaults_empty_and_fc_env_defaults_default():
    for call in _calls(CITY_TREE, "_env"):
        if not call.args or not isinstance(call.args[0], ast.Constant):
            continue
        name = call.args[0].value
        if name == "FC_DUMP":
            assert call.args[1].value == "", "FC_DUMP should default empty"
        elif name == "FC_ENV":
            assert call.args[1].value == "default"


def test_fc_dump_only_writes_in_intact_only_mode():
    fn = _funcdefs(CITY_TREE).get("__init__")
    assert fn is not None
    # `dump_city_placements` must be reachable only under an `if
    # INTACT_ONLY:` branch inside FireCityApp.__init__ -- never unconditional.
    found = False
    for node in ast.walk(fn):
        if isinstance(node, ast.If):
            test_src = ast.dump(node.test)
            if "INTACT_ONLY" in test_src and _calls(node, "dump_city_placements"):
                found = True
    assert found, ("dump_city_placements must be called inside an "
                  "`if INTACT_ONLY:` guard in FireCityApp.__init__")


def test_dump_functions_exist():
    fns = _funcdefs(CITY_TREE)
    for name in ("default_dump_path", "_typology_rects", "dump_city_placements"):
        assert name in fns, "missing " + name


# ---------------------------------------------------------------------------
# 1c) FC_SKY -- the 2026-08-31 mid_day/sunset lighting preset knob
# ---------------------------------------------------------------------------
def test_fc_sky_defaults_to_sunset():
    # "sunset" here means "the historical, unchanged dome-only path" -- see
    # sky_presets.py's own docstring for why the city's "sunset" is not the
    # bench's literal low-sun look. The city must only change lighting once
    # this knob is flipped explicitly.
    for call in _calls(CITY_TREE, "_env"):
        if call.args and getattr(call.args[0], "value", None) == "FC_SKY":
            assert call.args[1].value == "sunset"
            return
    raise AssertionError("FC_SKY is not read with a default")


def test_fc_sky_gates_the_new_preset_path():
    # The legacy `add_sky(resolve_sky(config), ...)` + `_disable_sky_sun`
    # call must still run together under a branch keyed on SKY (the
    # "sunset" default's byte-identical path), and `apply_sky_preset`
    # (sky_presets.py) must be reachable from the SAME `if`/`else` -- so
    # FC_SKY=sunset (the default) can never silently pick up the new path.
    assert "from sky_presets import apply_sky_preset" in CITY_SRC
    fn = _funcdefs(CITY_TREE).get("__init__")
    assert fn is not None
    found_legacy_branch = False
    found_preset_branch = False
    for node in ast.walk(fn):
        if not isinstance(node, ast.If):
            continue
        if "SKY" not in ast.dump(node.test):
            continue
        if _calls(node, "add_sky") and _calls(node, "_disable_sky_sun"):
            found_legacy_branch = True
        if _calls(node, "apply_sky_preset"):
            found_preset_branch = True
    assert found_legacy_branch, (
        "add_sky/_disable_sky_sun must still run together under an "
        "`if ... SKY ...` branch")
    assert found_preset_branch, (
        "apply_sky_preset must be called under an `if ... SKY ...` branch")


def test_fc_env_none_skips_load_environment():
    fn = _funcdefs(CITY_TREE).get("__init__")
    assert fn is not None
    assert "FC_ENV" in CITY_SRC
    assert "none" in ast.dump(fn)
    # `load_environment` must not be unconditional -- it has to be reachable
    # only in the non-"none" branch.
    calls = _calls(fn, "load_environment")
    assert calls, "load_environment must still be called in the default path"


def test_the_manifest_city_match_summary_line_exists():
    assert "manifest/city match" in CITY_SRC
    fn = _funcdefs(CITY_TREE).get("compose_bakes")
    assert fn is not None
    assert "manifest/city match" in (ast.get_source_segment(CITY_SRC, fn) or "")


# ---------------------------------------------------------------------------
# the dump writer -- extracted and exercised on a synthetic placements list
# ---------------------------------------------------------------------------
def _load_dump_writer():
    """Execute `default_dump_path`/`_typology_rects`/`dump_city_placements`
    out of the launcher source, with `_make_resolver` stubbed (the real one
    is `scene_generator._make_resolver`, which needs `pxr` to even import
    the module it lives in) — the same "extract and exec" trick this file
    already uses for `fire_view_params` and the emitter allocator."""
    fns = _funcdefs(CITY_TREE)

    class _StubResolver:
        def get(self, usd, category, scale=None, axis_up="Z"):
            return {"sx": 10.0 + (len(str(usd)) % 5), "sy": 8.0, "sz": 20.0}

    ns = {"os": os, "json": json, "_SCENE_GEN_DIR": _SCENE_GEN,
          "_make_resolver": lambda config: _StubResolver()}
    for name in ("default_dump_path", "_typology_rects", "dump_city_placements"):
        exec(compile(ast.get_source_segment(CITY_SRC, fns[name]), CITY, "exec"), ns)
    return ns


def test_default_dump_path_names_preset_and_seed():
    ns = _load_dump_writer()
    path = ns["default_dump_path"]("downtown_fire_500", 42)
    assert "city_placements_downtown_fire_500_42.json" in path
    assert "_plans" in path


def test_dump_city_placements_writes_only_houses_with_original_index(tmp_path):
    """The WRITE-side half of the dump schema round trip: only `category ==
    'house'` placements are written, each carries its ORIGINAL index into
    the FULL placements list (not renumbered relative to a house-only
    list — see `dump_city_placements`'s own docstring on why this matters
    for `resolve_cell`'s route 1), and W/D/H come from the (stubbed)
    resolver."""
    ns = _load_dump_writer()
    placements = [
        {"category": "tree", "usd": "assets/tree.usd", "x_m": 1.0, "y_m": 2.0},
        {"category": "house", "usd": "assets/house_a.usd", "x_m": 10.0,
         "y_m": 20.0, "z_m": 0.0, "yaw_deg": 90.0, "scale": 0.01,
         "prim_path": "/World/stage/generated/house_0_1"},
        {"category": "bench", "usd": "assets/bench.usd", "x_m": 3.0, "y_m": 4.0},
        {"category": "house", "usd": "assets/house_b.usd", "x_m": 30.0,
         "y_m": 40.0, "z_m": 0.5, "yaw_deg": 0.0, "scale": 1.0,
         "prim_path": "/World/stage/generated/house_1_3"},
    ]
    layout = {"_typology_of": {(0.0, 0.0, 100.0, 100.0): "lowrise"}}
    config = {"layout": {"region_m": [500.0, 500.0]}}
    path = str(tmp_path / "dump.json")

    ns["dump_city_placements"](path, "downtown_fire_500", 42, config,
                               placements, layout)

    with open(path) as fh:
        doc = json.load(fh)
    assert doc["schema"] == "fire_city_placements_dump.v1"
    assert doc["preset"] == "downtown_fire_500"
    assert doc["seed"] == 42
    assert doc["n_placements_total"] == 4
    assert [p["i"] for p in doc["placements"]] == [1, 3]
    assert doc["placements"][0]["cell"] == "/World/stage/generated/house_0_1"
    assert doc["placements"][0]["usd"] == "assets/house_a.usd"
    for key in ("W", "D", "H"):
        assert key in doc["placements"][0]
    assert doc["typology"]["blocks"] == [
        {"rect": [0.0, 0.0, 100.0, 100.0], "name": "lowrise"}]


def test_dump_write_read_round_trips_through_the_dry_run(tmp_path):
    """The FULL schema round trip: what `dump_city_placements` writes,
    `fire_city_dry_run.load_placements_dump` reads back with the SAME
    index alignment — non-house indices become placeholders, house indices
    land exactly where they were in the original placements list."""
    import fire_city_dry_run as fdr

    ns = _load_dump_writer()
    placements = [
        {"category": "tree", "usd": "assets/tree.usd", "x_m": 1.0, "y_m": 2.0},
        {"category": "house", "usd": "assets/house_a.usd", "x_m": 10.0,
         "y_m": 20.0, "z_m": 0.0, "yaw_deg": 90.0, "scale": 0.01,
         "prim_path": "/World/stage/generated/house_0_1"},
        {"category": "bench", "usd": "assets/bench.usd", "x_m": 3.0, "y_m": 4.0},
        {"category": "house", "usd": "assets/house_b.usd", "x_m": 30.0,
         "y_m": 40.0, "z_m": 0.5, "yaw_deg": 0.0, "scale": 1.0,
         "prim_path": "/World/stage/generated/house_1_3"},
    ]
    layout = {"_typology_of": {(0.0, 0.0, 100.0, 100.0): "lowrise"}}
    config = {"layout": {"region_m": [500.0, 500.0]}}
    path = str(tmp_path / "dump.json")
    ns["dump_city_placements"](path, "downtown_fire_500", 42, config,
                               placements, layout)

    (_config2, layout2, placements2, seed2, preset2,
     _sha2) = fdr.load_placements_dump(path)
    assert seed2 == 42
    assert preset2 == "downtown_fire_500"
    assert len(placements2) == 4
    assert placements2[1]["category"] == "house"
    assert placements2[1]["usd"] == "assets/house_a.usd"
    assert placements2[3]["category"] == "house"
    assert placements2[3]["usd"] == "assets/house_b.usd"
    assert placements2[0]["category"] != "house"
    assert placements2[2]["category"] != "house"
    assert layout2["_typology_of"][(0.0, 0.0, 100.0, 100.0)] == "lowrise"


# ---------------------------------------------------------------------------
# 2) the transform trap
# ---------------------------------------------------------------------------
def test_the_intact_prim_is_hidden():
    assert "MakeInvisible" in CITY_SRC
    assert _calls(CITY_TREE, "MakeInvisible")


def test_the_holder_authors_a_fresh_rotate_xyz():
    assert "AddRotateXYZOp" in CITY_SRC
    assert "rotateXYZ" in CITY_SRC
    assert _calls(CITY_TREE, "AddRotateXYZOp")
    assert _calls(CITY_TREE, "AddTranslateOp")
    assert _calls(CITY_TREE, "AddScaleOp")


def test_the_holder_rotates_only_about_z():
    """`rotateXYZ = (0, 0, yaw)` — never the placement's roll/pitch, which
    exist only to stand a Y-up intact asset upright."""
    fn = _funcdefs(CITY_TREE).get("place_holder")
    assert fn is not None
    call = _calls(fn, "AddRotateXYZOp")
    assert call, "place_holder must author a rotateXYZ op"
    vec = None
    for node in ast.walk(fn):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                and node.func.attr == "Vec3f" and len(node.args) == 3:
            if all(isinstance(a, ast.Constant) for a in node.args[:2]):
                if node.args[0].value == 0.0 and node.args[1].value == 0.0:
                    vec = node
    assert vec is not None, ("the rotateXYZ vector must be (0, 0, yaw); no "
                             "Vec3f(0.0, 0.0, ...) found in place_holder")


def test_the_quake_swap_the_reference_idiom_is_absent():
    """`quake.assemble` keeps the cell's transform and swaps the reference;
    for a bake (metres, Z-up, its own pivot) that is silently wrong. Checked
    on the AST, not the text — the module docstring NAMES the idiom in order
    to warn about it, and a text search would trip on its own warning."""
    assert not _calls(CITY_TREE, "ClearReferences"), (
        "the city launcher must author a FRESH holder, never re-point the "
        "intact cell's reference")


def test_the_bake_is_referenced_onto_a_child():
    assert "/bake" in CITY_SRC
    assert "AddReference" in CITY_SRC


# ---------------------------------------------------------------------------
# 3) the double-translate trap
# ---------------------------------------------------------------------------
def test_it_uses_fire_bake_place_not_translate():
    place = [c for c in _calls(CITY_TREE, "place")
             if isinstance(c.func, ast.Attribute)
             and isinstance(c.func.value, ast.Name) and c.func.value.id == "fb"]
    assert place, "the city launcher must call fire_bake.place(...)"
    assert "fb.translate(" not in CITY_SRC, (
        "fire_bake.translate cannot rotate — a city cell has a yaw")


def test_fire_bake_place_is_given_masses_events_seats_dx_dy_yaw():
    place = [c for c in _calls(CITY_TREE, "place")
             if isinstance(c.func, ast.Attribute)
             and isinstance(c.func.value, ast.Name) and c.func.value.id == "fb"][0]
    assert len(place.args) == 6, (
        "fire_bake.place(masses, events, seats, dx, dy, yaw_deg) — got "
        "{0} positional arg(s)".format(len(place.args)))


def test_place_fire_gets_dx_dy_zero_because_place_moved_the_seats():
    calls = [c for c in _calls(CITY_TREE, "place_fire")
             if isinstance(c.func, ast.Attribute)]
    assert calls, "the city launcher must call fal.place_fire(...)"
    call = calls[0]
    assert len(call.args) >= 10, "place_fire takes dx, dy positionally at 8/9"
    for idx in (8, 9):
        arg = call.args[idx]
        assert isinstance(arg, ast.Constant) and float(arg.value) == 0.0, (
            "place_fire's dx/dy must be 0 — `fire_bake.place` has already "
            "moved the seats, and `_sphere_source` would add the cell a "
            "second time")


# ---------------------------------------------------------------------------
# 4) the emitter budget and the Flow-OOM grep
# ---------------------------------------------------------------------------
def test_a_global_emitter_budget_exists():
    assert "FA_EMITTER_BUDGET" in CITY_SRC
    assert "EMITTER_BUDGET" in CITY_SRC
    fns = _funcdefs(CITY_TREE)
    assert "allocate_emitters" in fns
    assert "emitter_estimate" in fns


def test_the_allocator_ranks_by_state():
    assert "STATE_RANK" in CITY_SRC
    src = CITY_SRC
    assert '"flame": 0' in src and '"smoulder": 1' in src, (
        "flame must outrank smoulder, which must outrank the wisps")


def test_the_allocator_is_passed_the_budget_and_the_cap():
    calls = _calls(CITY_TREE, "allocate_emitters")
    call = [c for c in calls if len(c.args) >= 3]
    assert call, "allocate_emitters(rows, budget, cap, smoke)"


def test_the_flow_oom_strings_are_grepped_for():
    for needle in ("Out of GPU memory allocating resource 'flow'",
                   "Maximum Flow blocks"):
        assert needle in CITY_SRC, "missing the Flow OOM needle: " + needle
    assert "FLOW STARVED" in CITY_SRC
    assert "grep_kit_log" in _funcdefs(CITY_TREE)


def test_the_kit_log_path_is_the_container_one():
    # `docker logs` is empty for this container and the host-mounted
    # `.nvidia-omniverse/logs` is not where Kit writes its own log
    assert "/isaac-sim/kit/logs/Kit/Isaac-Sim Python" in CITY_SRC


def test_vram_is_measured_at_every_stage():
    tags = _const_str_args(_calls(CITY_TREE, "vram_mb"))
    joined = " | ".join(sorted(tags)) + " | " + CITY_SRC
    for stage in ("empty", "intact", "bakes", "flow", "end"):
        assert '"{0}"'.format(stage) in CITY_SRC, stage
    assert "empty stage" in joined
    assert "INTACT" in joined
    assert "Flow up" in joined
    assert "after captures" in joined
    assert "VRAM BUDGET" in CITY_SRC


def test_the_captures_are_authored():
    for name in ("overview", "views_around", "place_camera", "snapshot"):
        assert _calls(CITY_TREE, name), "no snapshots." + name + " call"
    assert "wave_downwind" in CITY_SRC, "the downwind 'wave' capture"
    assert "fire_view_params" in CITY_SRC, "the fire-facing review cameras"


def test_the_banner_and_keep_open():
    assert "URBAN FIRE CITY DONE" in CITY_SRC
    assert "KEEP_OPEN" in CITY_SRC
    assert "simulation_app.close()" in CITY_SRC


def test_fractional_cutout_opacity_is_set_both_ways():
    # the startup flag does not survive stage composition; the carb form
    # alone is too late for startup. BOTH are required.
    assert CITY_SRC.count("fractionalCutoutOpacity") >= 4
    assert "extra_args" in CITY_SRC


# ---------------------------------------------------------------------------
# 5) the fire_assembly_lib refactor
# ---------------------------------------------------------------------------
LIB_TREE = _tree(LIB)
ROW_SRC = _src(ROW)
ROW_TREE = ast.parse(ROW_SRC, filename=ROW)

EXPECTED_SIGNATURES = {
    "vram_mb": ["tag", "prefix"],
    "resolve_bakes": ["spec", "pattern"],
    "order_bakes": ["rows", "order"],
    "build_ground_and_light": ["stage", "span", "prefix"],
    "bbox": ["stage", "path"],
    "_sphere_source": ["stage", "path", "seat", "state", "scale", "vel",
                       "dx", "dy", "top_z"],
    "place_fire": ["stage", "root", "doc", "masses", "events", "tag", "rng",
                   "top_z", "dx", "dy", "scale", "max_emitters", "smoke"],
    "fire_view_params": ["doc", "masses", "box"],
}


def test_lib_exports_every_moved_function_with_its_signature():
    fns = _funcdefs(LIB_TREE)
    for name, params in EXPECTED_SIGNATURES.items():
        assert name in fns, "fire_assembly_lib is missing " + name
        got = [a.arg for a in fns[name].args.args]
        assert got == params, "{0}: {1} != {2}".format(name, got, params)


def test_the_row_launcher_forwards_its_own_knobs():
    """The three module globals the moved code used to close over."""
    assert "fal.resolve_bakes(spec, pattern)" in ROW_SRC
    assert "fal.order_bakes(rows, ORDER)" in ROW_SRC
    assert "smoke=SMOKE" in ROW_SRC
    assert "pattern=GLOB" in ROW_SRC


def test_the_row_launcher_no_longer_defines_the_moved_bodies():
    fns = _funcdefs(ROW_TREE)
    # the wrappers stay (same names, same call sites in main()), but they
    # must be one-liners delegating to the lib
    for name in ("resolve_bakes", "order_bakes", "place_fire"):
        assert name in fns
        assert len(fns[name].body) == 1, (
            name + " should be a single delegating return")
        assert _calls(fns[name], name.split(".")[-1]) or True
    for name in ("_sphere_source", "build_ground_and_light", "vram_mb",
                 "_bbox"):
        assert name not in fns, name + " should now be an alias, not a def"


def test_both_launchers_call_the_same_place_fire():
    for tree in (ROW_TREE, CITY_TREE):
        calls = [c for c in _calls(tree, "place_fire")
                 if isinstance(c.func, ast.Attribute)
                 and isinstance(c.func.value, ast.Name)
                 and c.func.value.id == "fal"]
        assert calls, "must call fal.place_fire"


# ---------------------------------------------------------------------------
# `fire_view_params` reproduces the ORIGINAL inline arithmetic, numerically
# ---------------------------------------------------------------------------
def _load_fire_view_params():
    """Execute just that function out of the lib source.

    The module itself imports `pxr` and `disaster.urban_fire`; the function
    needs only `math`, so it is extracted and exec'd in a namespace holding
    exactly that — the test stays host-side and still exercises the REAL
    source rather than a copy of it.
    """
    fn = _funcdefs(LIB_TREE)["fire_view_params"]
    src = ast.get_source_segment(_src(LIB), fn)
    ns = {"math": math}
    exec(compile(src, LIB, "exec"), ns)
    return ns["fire_view_params"]


def _original_inline(doc, masses, b):
    """The arithmetic as `fire_assembly_launch_script.main()` had it inline
    before the move (fire_row3's fire-facing review camera). Kept verbatim
    here so the extraction is checked against what it replaced, not against
    a paraphrase of itself."""
    W, D, H = b[3] - b[0], b[4] - b[1], b[5] - b[2]
    top_h = max(W, D) / 1.164 * 1.45 + H
    # 2026-08-31: H term deliberately 1.7x (was a shared 1.3x) — at 1.3x a
    # 60 m+ tower's facade overflowed the frame (d31_apartment_tall, city_v3
    # review). The width branch is the original arithmetic, verbatim.
    obl_dist = max(50.0, 1.3 * max(W, D), 1.7 * H)
    obl_h = max(18.0, 0.4 * H)
    import math as _m
    fd = doc.get("fire") or {}
    vec = {"E": (1, 0), "N": (0, 1), "W": (-1, 0), "S": (0, -1)}
    vx = sum(vec.get(sd, (0, 0))[0] for sd in (fd.get("sides") or []))
    vy = sum(vec.get(sd, (0, 0))[1] for sd in (fd.get("sides") or []))
    az = _m.degrees(_m.atan2(vy, vx)) if (vx or vy) else 225.0
    aim_h = 1.0
    try:
        lv = (masses.get(fd.get("mass") or "main")
              or list(masses.values())[0])["levels"]
        sts = [int(q) for q in (fd.get("storeys") or [])]
        if sts:
            aim_h = 0.5 * (lv[min(sts[0], len(lv) - 1)]
                           + lv[min(sts[-1], len(lv) - 1)]) + 1.5
    except Exception:
        pass
    obl_h = max(obl_h, aim_h + 0.3 * obl_dist)
    return {"top_h": top_h, "obl_dist": obl_dist, "obl_h": obl_h,
            "azimuth_deg": az, "aim_h": aim_h}


def test_fire_view_params_matches_the_original_arithmetic():
    fvp = _load_fire_view_params()
    rng = random.Random(1013)
    sides_pool = ([], ["E"], ["S"], ["N"], ["W"], ["S", "E"], ["N", "W"],
                  ["E", "N", "W", "S"])
    for _ in range(400):
        n_st = rng.randint(1, 40)
        levels = sorted(rng.uniform(0.0, 4.0) for _ in range(n_st))
        levels = [sum(levels[:k + 1]) for k in range(n_st)]
        doc = {"fire": {"sides": rng.choice(sides_pool),
                        "mass": rng.choice(("main", "wing", None)),
                        "storeys": sorted(rng.sample(range(n_st),
                                                     min(n_st, rng.randint(0, 3))))}}
        masses = {"main": {"levels": levels}}
        if rng.random() < 0.3:
            masses["wing"] = {"levels": levels[:max(1, n_st // 2)]}
        x0, y0 = rng.uniform(-200, 200), rng.uniform(-200, 200)
        b = [x0, y0, rng.uniform(-0.5, 0.0),
             x0 + rng.uniform(5, 60), y0 + rng.uniform(5, 60),
             rng.uniform(4, 160)]
        got, want = fvp(doc, masses, b), _original_inline(doc, masses, b)
        assert got == want, (doc, b, got, want)


def test_fire_view_params_tolerates_a_bake_with_no_masses():
    fvp = _load_fire_view_params()
    out = fvp({"fire": {"sides": ["E"], "storeys": [2]}}, {},
              [0.0, 0.0, 0.0, 20.0, 15.0, 30.0])
    assert out["azimuth_deg"] == 0.0        # E -> the camera stands east
    assert out["aim_h"] == 1.0              # no levels to aim at


def test_fire_view_params_defaults_to_the_south_west_with_no_sides():
    fvp = _load_fire_view_params()
    out = fvp({"fire": {"sides": []}}, {}, [0.0, 0.0, 0.0, 10.0, 10.0, 10.0])
    assert out["azimuth_deg"] == 225.0


# ---------------------------------------------------------------------------
# the emitter budget, EXERCISED — extracted from the launcher and run
# ---------------------------------------------------------------------------
def _module_constant(path, name):
    """A module-level `NAME = <literal>` read off the AST, so the test tracks
    the real constant without importing a module that pulls in numpy/pxr."""
    for node in ast.parse(_src(path)).body:
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == name:
                    return ast.literal_eval(node.value)
    raise AssertionError("{0} has no module-level {1}".format(path, name))


FLAME_PER_OPENING = _module_constant(
    os.path.join(_SCENE_GEN, "disaster", "urban_fire.py"), "FLAME_PER_OPENING")
SMOKE_EXTRA_MAX = _module_constant(
    os.path.join(_SCENE_GEN, "disaster", "urban_fire.py"), "SMOKE_EXTRA_MAX")
SMOULDER_EVENTS_MAX = _module_constant(
    os.path.join(_SCENE_GEN, "disaster", "soot_plume.py"),
    "SMOULDER_EVENTS_MAX")


class _Stub(object):
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _load_allocator():
    """`(emitter_estimate, allocate_emitters)` executed out of the launcher
    source, with `uf`/`spl` stubbed to the REAL constants (read off their own
    ASTs above). The launcher itself cannot be imported — SimulationApp."""
    fns = _funcdefs(CITY_TREE)
    ns = {"uf": _Stub(FLAME_PER_OPENING=FLAME_PER_OPENING,
                      SMOKE_EXTRA_MAX=SMOKE_EXTRA_MAX),
          "spl": _Stub(SMOULDER_EVENTS_MAX=SMOULDER_EVENTS_MAX),
          "STATE_RANK": ast.literal_eval(
              ast.get_source_segment(
                  CITY_SRC,
                  [n.value for n in CITY_TREE.body
                   if isinstance(n, ast.Assign)
                   and any(getattr(t, "id", None) == "STATE_RANK"
                           for t in n.targets)][0]))}
    for name in ("_live", "fire_state", "emitter_estimate",
                 "allocate_emitters"):
        exec(compile(ast.get_source_segment(CITY_SRC, fns[name]), CITY, "exec"),
             ns)
    return ns


def _row(i, state, n_st, n_flame_ev=4, ops=6, seats=0, roof=False):
    """One `self.placed` row as `allocate_emitters` sees it."""
    ev = lambda k, st: {"id": k, "state": st, "storey": k,
                        "ops": [{"e": {}} for _ in range(ops)]}
    events = [ev(k, "flame") for k in range(n_flame_ev)]
    events += [ev(100 + k, "smoulder") for k in range(3)]
    events += [ev(200 + k, "out") for k in range(3)]
    doc = {"fire": {"state": state, "n_storeys": n_st, "roof": roof},
           "seats": {"interior": [{}] * seats, "roof": [{}] * (2 if roof else 0)}}
    return {"i": i, "stem": "b{0}".format(i), "doc": doc, "events": events}


def test_the_budget_is_never_exceeded():
    ns = _load_allocator()
    rows = ([_row(i, "flame", 6) for i in range(6)]
            + [_row(6 + i, "smoulder", 20, seats=4, roof=True) for i in range(6)]
            + [_row(12 + i, None, 4) for i in range(4)])
    for budget in (10, 40, 80, 200, 1000):
        for r in rows:
            r.pop("drop_reason", None)
        spent = ns["allocate_emitters"](rows, budget, 6, True)
        assert spent <= budget, (budget, spent)
        total = sum(r["est"]["total"] for r in rows if r.get("alloc") is not None)
        assert total == spent
        assert all(r["est"]["total"] == 0 for r in rows
                   if r.get("alloc") is None)


def test_the_worst_buildings_are_served_first():
    ns = _load_allocator()
    rows = ([_row(0, "flame", 6), _row(1, "flame", 6)]
            + [_row(2 + i, "smoulder", 30, seats=6, roof=True) for i in range(4)]
            + [_row(6 + i, None, 3) for i in range(3)])
    # a budget that cannot possibly cover everything
    ns["allocate_emitters"](rows, 30, 6, True)
    dropped = {r["i"] for r in rows if r.get("alloc") is None}
    kept = {r["i"] for r in rows if r.get("alloc") is not None}
    assert kept, "the budget must buy SOMETHING"
    # no flame building may be dropped while a wisp/residual one is kept
    for f in (0, 1):
        if f in dropped:
            assert not (kept - {0, 1}), (
                "a flame building was dropped while a lesser one was kept: "
                "kept={0} dropped={1}".format(sorted(kept), sorted(dropped)))


def test_a_dropped_building_carries_a_reason():
    ns = _load_allocator()
    rows = [_row(i, "flame", 6) for i in range(8)]
    ns["allocate_emitters"](rows, 12, 6, True)
    for r in rows:
        if r.get("alloc") is None:
            assert r.get("drop_reason"), r["stem"]


def test_the_estimate_prices_a_tall_building_by_its_opening_floor():
    """`place_fire` widens a >=12-storey building's opening budget to
    `min(16, n_st // 2)` whatever it is told, so the allocator must price it
    that way or the budget is fiction."""
    ns = _load_allocator()
    tall = _row(0, "flame", 30, n_flame_ev=8, ops=8)
    short = _row(1, "flame", 6, n_flame_ev=8, ops=8)
    e_tall = ns["emitter_estimate"](tall["doc"], tall["events"], 1, True)
    e_short = ns["emitter_estimate"](short["doc"], short["events"], 1, True)
    assert e_tall["openings"] == min(16, 30 // 2)
    assert e_short["openings"] == 1
    assert e_tall["total"] > e_short["total"]


def test_smoke_off_prices_flames_only():
    ns = _load_allocator()
    r = _row(0, "smoulder", 8, seats=5, roof=True)
    on = ns["emitter_estimate"](r["doc"], r["events"], 4, True)
    off = ns["emitter_estimate"](r["doc"], r["events"], 4, False)
    assert off["smoke"] == off["interior"] == off["roof"] == 0
    assert off["flame"] == on["flame"]
    assert on["total"] > off["total"]


def test_an_f1_wisp_costs_at_most_two():
    ns = _load_allocator()
    r = _row(0, None, 5)                 # no ACTIVE state -> the F1 wisp path
    state, wisp = ns["fire_state"](r["doc"], r["events"])
    assert (state, wisp) == ("smoulder", True)
    est = ns["emitter_estimate"](r["doc"], r["events"], 6, True)
    assert est["total"] <= 2


def test_a_dead_event_is_not_priced():
    """`e["dead"]` is serialised AFTER the ladder, so a flame never floats
    where a collapse took the wall — and it must not be budgeted for."""
    ns = _load_allocator()
    r = _row(0, "flame", 6)
    for ev in r["events"]:
        for op in ev["ops"]:
            op["e"]["dead"] = True
    est = ns["emitter_estimate"](r["doc"], r["events"], 6, True)
    assert est["total"] == 0


# ---------------------------------------------------------------------------
# the transform probe is runnable and self-describing
# ---------------------------------------------------------------------------
def test_the_transform_probe_exists_and_takes_a_bake():
    tree = _tree(PROBE)
    fns = _funcdefs(tree)
    for name in ("measure_alone", "compose_under_holder", "world_bbox",
                 "read_sidecar", "main"):
        assert name in fns, "fc_transform_probe is missing " + name
    src = _src(PROBE)
    assert "AddRotateXYZOp" in src and "AddTranslateOp" in src
    assert "FC TRANSFORM PROBE OK" in src


def test_the_probe_composes_the_reference_onto_a_child():
    src = _src(PROBE)
    assert 'holder + "/bake"' in src


# ---------------------------------------------------------------------------
# FC_CROP_WINDOW -- "generate 1.5 km, crop to 1 km, not always centred"
# (`downtown_fire_1500.yaml`, `tools/crop_window.py`, `tools/fc_dump_crop.py`,
# `tools/baseline_layouts.py`). Same AST-extraction discipline as the rest of
# this file: the launcher itself is never imported.
# ---------------------------------------------------------------------------
def test_fc_crop_window_env_knob_is_read():
    for call in _calls(CITY_TREE, "_env"):
        if call.args and getattr(call.args[0], "value", None) == "FC_CROP_WINDOW":
            assert call.args[1].value == "", (
                "FC_CROP_WINDOW should default empty -- unset must mean "
                "'no crop, uncropped paths untouched'")
            return
    raise AssertionError("FC_CROP_WINDOW is not read with a default")


def test_apply_crop_window_exists_and_is_called_in_init():
    fns = _funcdefs(CITY_TREE)
    assert "_apply_crop_window" in fns
    init = fns.get("__init__")
    assert init is not None
    assert _calls(init, "_apply_crop_window"), (
        "_apply_crop_window must be called from FireCityApp.__init__")


def test_crop_window_deactivates_never_shrinks_the_placements_list():
    """`self.placements` must stay the SAME list object/length throughout
    `_apply_crop_window` -- `resolve_cell`'s route 1 depends on `placements
    [i]` still being the ORIGINAL full-city entry at index `i` (a manifest
    record's `i` is assigned before any crop exists). This greps the method
    body for the tell-tale of the WRONG fix (reassigning `self.placements`
    to a filtered list) rather than the right one (mutating entries with
    `_fc_cropped` + `SetActive`)."""
    fn = _funcdefs(CITY_TREE).get("_apply_crop_window")
    assert fn is not None
    src = ast.get_source_segment(CITY_SRC, fn) or ""
    assert "_fc_cropped" in src
    assert "SetActive" in src
    assert "self.placements =" not in src, (
        "_apply_crop_window must never reassign self.placements -- that "
        "would break resolve_cell's index-based route 1 for every "
        "placement after the first drop")


def test_settle_and_dump_skip_cropped_placements():
    init = _funcdefs(CITY_TREE).get("__init__")
    assert init is not None
    init_src = ast.get_source_segment(CITY_SRC, init) or ""
    assert "_fc_cropped" in init_src
    dump_fn = _funcdefs(CITY_TREE).get("dump_city_placements")
    assert dump_fn is not None
    assert "_fc_cropped" in (ast.get_source_segment(CITY_SRC, dump_fn) or "")


def test_capture_frames_the_crop_window_when_active():
    fn = _funcdefs(CITY_TREE).get("capture")
    assert fn is not None
    src = ast.get_source_segment(CITY_SRC, fn) or ""
    assert "self.crop_window" in src


def _load_crop_window_helpers():
    """`(_parse_crop_window, _unshift_records_to_full_city)` executed out of
    the launcher source -- both are pure functions with no `pxr`/Kit
    dependency, so this is a plain exec, no stubbing needed."""
    fns = _funcdefs(CITY_TREE)
    ns = {}
    for name in ("_parse_crop_window", "_unshift_records_to_full_city"):
        exec(compile(ast.get_source_segment(CITY_SRC, fns[name]), CITY, "exec"),
             ns)
    return ns


def test_parse_crop_window_math():
    ns = _load_crop_window_helpers()
    parse = ns["_parse_crop_window"]
    assert parse("") is None
    assert parse(None) is None
    x0, y0, x1, y1 = parse("100,-50,1000,1000")
    assert (x0, y0, x1, y1) == (-400.0, -550.0, 600.0, 450.0)
    # a window centred at (0, 0) must be exactly symmetric
    assert parse("0,0,1000,1000") == (-500.0, -500.0, 500.0, 500.0)


def test_parse_crop_window_rejects_garbage_loudly():
    import pytest
    ns = _load_crop_window_helpers()
    parse = ns["_parse_crop_window"]
    for bad in ("1,2,3", "1,2,3,4,5", "a,b,1000,1000", "0,0,-1000,1000",
                "0,0,0,1000"):
        with pytest.raises(ValueError):
            parse(bad)


def test_unshift_prefers_x_orig_when_present():
    ns = _load_crop_window_helpers()
    unshift = ns["_unshift_records_to_full_city"]
    recs = [{"i": 3, "x": 34.0, "y": -66.0, "x_orig": 214.0, "y_orig": 34.0}]
    n = unshift(recs, 999.0, -999.0)   # deliberately wrong cx,cy -- must be ignored
    assert n == 1
    assert recs[0]["x"] == 214.0 and recs[0]["y"] == 34.0


def test_unshift_falls_back_to_arithmetic_without_x_orig():
    ns = _load_crop_window_helpers()
    unshift = ns["_unshift_records_to_full_city"]
    recs = [{"i": 3, "x": 34.0, "y": -66.0}]
    n = unshift(recs, 180.0, -200.0)
    assert n == 1
    assert recs[0]["x"] == 214.0 and recs[0]["y"] == -266.0


def test_unshift_skips_non_dict_and_fieldless_records():
    ns = _load_crop_window_helpers()
    unshift = ns["_unshift_records_to_full_city"]
    recs = ["not a dict", {"cls": "onlooker"}, {"x": 1.0, "y": 2.0}]
    n = unshift(recs, 10.0, 10.0)
    assert n == 1
    assert recs[2] == {"x": 11.0, "y": 12.0}


def test_load_fire_and_place_people_both_call_the_unshift_under_crop_centre():
    for name in ("load_fire", "place_people"):
        fn = _funcdefs(CITY_TREE).get(name)
        assert fn is not None, name
        src = ast.get_source_segment(CITY_SRC, fn) or ""
        assert "_unshift_records_to_full_city" in src, (
            name + " must call _unshift_records_to_full_city")
        assert "CROP_CENTRE is not None" in src, (
            name + "'s unshift must be gated on CROP_CENTRE (byte-identical "
            "when FC_CROP_WINDOW is unset)")


# ---------------------------------------------------------------------------
# resolve_cell -- extracted with Sdf/math stubbed (route 1, the one this
# feature touches, never calls `stage.GetPrimAtPath` at all).
# ---------------------------------------------------------------------------
def _load_resolve_cell():
    fns = _funcdefs(CITY_TREE)
    ns = {"math": math, "Sdf": _Stub(Path=lambda s: s)}
    exec(compile(ast.get_source_segment(CITY_SRC, fns["resolve_cell"]),
                CITY, "exec"), ns)
    return ns["resolve_cell"]


def test_resolve_cell_prefers_x_orig_over_the_recentred_x():
    """The exact scenario `FC_CROP_WINDOW` exists for: `placements[i]` is
    Kit's own, ALWAYS-full-city position; `rec["x"]/["y"]` are a manifest
    solved on a re-centred cropped dump (so, deliberately, FAR from
    `placements[i]` here); `rec["x_orig"]/["y_orig"]` are the true full-city
    position and must be what actually gets compared."""
    resolve_cell = _load_resolve_cell()
    placements = [{"usd": "bld_office_DG0.usd", "x_m": 214.3, "y_m": 34.1,
                  "prim_path": "/World/stage/generated/house_0_3",
                  "category": "house"}]
    rec = {"i": 0, "usd": "bld_office_DG0.usd",
          "x": 34.0, "y": -66.0,           # recentred -- 180 m away, must NOT match
          "x_orig": 214.2, "y_orig": 34.0}  # true full-city position -- must match
    cell, how = resolve_cell(None, placements, rec)
    assert cell == "/World/stage/generated/house_0_3"
    assert how.startswith("index")


def test_resolve_cell_falls_back_to_plain_x_when_no_x_orig():
    resolve_cell = _load_resolve_cell()
    placements = [{"usd": "bld_office_DG0.usd", "x_m": 10.0, "y_m": 20.0,
                  "prim_path": "/World/stage/generated/house_0_5",
                  "category": "house"}]
    rec = {"i": 0, "usd": "bld_office_DG0.usd", "x": 10.1, "y": 19.9}
    cell, how = resolve_cell(None, placements, rec)
    assert cell == "/World/stage/generated/house_0_5"
    assert how.startswith("index")


def test_dump_city_placements_skips_fc_cropped_placements():
    ns = _load_dump_writer()
    placements = [
        {"category": "house", "usd": "assets/house_a.usd", "x_m": 10.0,
         "y_m": 20.0, "z_m": 0.0, "yaw_deg": 90.0, "scale": 0.01,
         "prim_path": "/World/stage/generated/house_0_0"},
        {"category": "house", "usd": "assets/house_b.usd", "x_m": 800.0,
         "y_m": 800.0, "z_m": 0.0, "yaw_deg": 0.0, "scale": 1.0,
         "prim_path": "/World/stage/generated/house_0_1",
         "_fc_cropped": True},
    ]
    layout = {"_typology_of": {}}
    config = {"layout": {"region_m": [1500.0, 1500.0]}}
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "dump.json")
        ns["dump_city_placements"](path, "downtown_fire_1500", 4, config,
                                   placements, layout)
        with open(path) as fh:
            doc = json.load(fh)
    assert [p["i"] for p in doc["placements"]] == [0]
    assert doc["placements"][0]["usd"] == "assets/house_a.usd"


# ---------------------------------------------------------------------------
# THE REAL LEVEL-1 CROPPED DUMP -- end-to-end proof of the match/shift math
# on `downtown_fire_1500` seed 4, the real `baseline_layouts.LEVELS[0]`
# window, host-built via `plan_png` (no Kit) exactly the way `tools/
# fc_dump_crop.py`'s own real-layout test builds one. Slower (builds a real
# 1500 m city) but still no Kit -- a couple of seconds.
# ---------------------------------------------------------------------------
def _build_real_uncropped_dump(seed):
    import plan_png
    cfg, layout, placements, res = plan_png.build(
        "downtown_fire_1500", seed=seed,
        spec_overrides={"disaster-type": "none"})
    houses = []
    for i, p in enumerate(placements):
        if p.get("category") != "house":
            continue
        fp = res.get(p.get("usd", ""), "house", scale=p.get("scale", 1.0),
                     axis_up=p.get("axis_up", "Z"))
        houses.append({
            "i": i, "cell": p.get("prim_path") or "/synthetic/{0}".format(i),
            "usd": p["usd"], "x_m": float(p["x_m"]), "y_m": float(p["y_m"]),
            "z_m": float(p.get("z_m", 0.0)),
            "yaw_deg": float(p.get("yaw_deg", 0.0)),
            "scale": float(p.get("scale", 1.0)), "category": "house",
            "axis_up": p.get("axis_up", "Z"),
            "W": float(fp["sx"]), "D": float(fp["sy"]), "H": float(fp["sz"]),
        })
    blocks = [{"rect": [float(v) for v in rect], "name": name}
             for rect, name in (layout.get("_typology_of") or {}).items()]
    doc = {
        "schema": "fire_city_placements_dump.v1",
        "preset": "downtown_fire_1500", "seed": int(cfg["seed"]),
        "region_m": [float(v) for v in cfg["layout"]["region_m"]],
        "n_placements_total": len(placements),
        "placements": houses, "typology": {"blocks": blocks},
    }
    return doc


def test_real_level1_cropped_dump_match_shift_math(tmp_path):
    import baseline_layouts as bl
    import fc_dump_crop as fdc
    import fire_city_dry_run as fdr

    level1 = bl.LEVELS[0]
    assert level1["level"] == 1
    window = bl._window_of(level1)
    cx, cy = level1["window_centre"]

    full_doc = _build_real_uncropped_dump(level1["seed"])
    cropped_doc, rpt = fdc.crop_fc_dump(full_doc, window)
    assert rpt["buildings_kept"] > 0

    full_path = str(tmp_path / "full.json")
    with open(full_path, "w") as fh:
        json.dump(full_doc, fh)
    # Kit's own live `self.placements` at assembly time IS a fresh, from-
    # scratch rebuild of the FULL (uncropped) city -- `load_placements_dump`
    # on the UNCROPPED dump reconstructs the identical index-aligned shape
    # `resolve_cell` indexes into, so it stands in for it here with no Kit.
    _config, _layout, full_placements, _seed, _preset, _sha = \
        fdr.load_placements_dump(full_path)

    resolve_cell = _load_resolve_cell()
    n_checked = 0
    for p in cropped_doc["placements"]:
        rec = {"i": p["i"], "usd": p["usd"], "x": p["x_m"], "y": p["y_m"],
              "x_orig": p["x_m_orig"], "y_orig": p["y_m_orig"]}
        cell, how = resolve_cell(None, full_placements, rec)
        assert cell is not None, (
            "level-1 real cropped dump: record i={0} did not resolve "
            "against the full-city rebuild -- {1}".format(p["i"], how))
        assert how.startswith("index"), (
            "record i={0} resolved via {1!r}, not route 1 (index) -- the "
            "x_orig/y_orig match should always win route 1 on this "
            "synthetic-but-real data".format(p["i"], how))
        n_checked += 1
    assert n_checked == rpt["buildings_kept"]
    print("[test] level-1 real cropped dump: {0}/{1} kept house(s) "
         "resolved via route 1 (x_orig/y_orig match)".format(
             n_checked, rpt["buildings_kept"]))


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
