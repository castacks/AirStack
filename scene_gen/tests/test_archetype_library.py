"""The library's BOOKKEEPING — provenance, the census, and preview framing.

None of it needs Isaac Sim, and all of it is the part that decides whether a
30 GB library can be trusted six weeks after it was baked. The bake itself is
Kit-only and is not tested here; what is tested is everything that would make
a correct bake unreadable or an invalid one look fine.
"""

import json
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from archetypes import census as C                              # noqa: E402
from archetypes import library as lib                           # noqa: E402
from archetypes import preview as PV                            # noqa: E402
from archetypes import version as V                             # noqa: E402


# --------------------------------------------------------------------------
# Provenance — the thing that says a library is stale
# --------------------------------------------------------------------------

def test_fingerprint_is_stable_across_calls():
    assert V.source_fingerprint() == V.source_fingerprint()


def test_fingerprint_moves_when_a_source_moves(tmp_path):
    """The whole point: an edit to the damage pipeline invalidates the
    library, whether or not anyone remembered to bump `PIPELINE_VERSION`."""
    root = tmp_path / "sg"
    for rel in V.SOURCES:
        p = root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("# stub\n")
    before = V.source_fingerprint(str(root))
    (root / V.SOURCES[0]).write_text("# stub\nCHANGED = 1\n")
    assert V.source_fingerprint(str(root)) != before


def test_fingerprint_ignores_files_outside_sources(tmp_path):
    """`library.py` is the read side. Renaming a manifest field must not
    invalidate every archetype on disk."""
    root = tmp_path / "sg"
    for rel in V.SOURCES:
        p = root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("# stub\n")
    before = V.source_fingerprint(str(root))
    (root / "archetypes" / "library.py").write_text("# unrelated\n")
    assert V.source_fingerprint(str(root)) == before


def test_stamp_carries_all_three_fields():
    st = V.stamp()
    assert set(st) == {"baked_at", "pipeline_version", "pipeline_fingerprint"}
    assert st["pipeline_version"] == V.PIPELINE_VERSION
    # An ISO timestamp with an offset, so a library baked on another box is
    # still comparable.
    assert "T" in st["baked_at"] and len(st["baked_at"]) >= 19


def test_a_record_with_no_provenance_is_stale():
    """Everything baked before this module predates every fix in `CHANGES`,
    so silence must not read as currency."""
    assert V.is_stale({"type": "x", "level": "cracked"})


def test_a_freshly_stamped_record_is_current():
    assert not V.is_stale(dict(V.stamp()))


def test_a_bumped_version_makes_an_old_record_stale(monkeypatch):
    rec = dict(V.stamp())
    monkeypatch.setattr(V, "PIPELINE_VERSION", V.PIPELINE_VERSION + "x")
    assert V.is_stale(rec)


def test_audit_splits_and_counts_by_pipeline():
    fresh = dict(V.stamp(), type="a", level="cracked")
    old = {"type": "b", "level": "cracked", "pipeline_version": "1",
           "pipeline_fingerprint": "deadbeefcafe"}
    none = {"type": "c", "level": "cracked"}
    res = V.audit({"archetypes": [fresh, old, none]})
    assert [r["type"] for r in res["current"]] == ["a"]
    assert sorted(r["type"] for r in res["stale"]) == ["b", "c"]
    assert res["stale_by_pipeline"][("1", "deadbeefcafe")] == 1


def test_changes_documents_the_current_version():
    """A bump with no note is a bump nobody can act on."""
    assert V.PIPELINE_VERSION in {v for v, _when, _why in V.CHANGES}


def test_every_fingerprinted_source_exists():
    """A typo in `SOURCES` is silent — the file is skipped and the hash is
    quietly weaker than it reads."""
    missing = [rel for rel in V.SOURCES
               if not os.path.isfile(os.path.join(_SCENE_GEN, rel))]
    assert missing == []


# --------------------------------------------------------------------------
# Census — which types a scene actually places
# --------------------------------------------------------------------------

@pytest.fixture
def cfg():
    return {
        "asset_pack": "test_pack",
        "seed": 42,
        "layout": {"region_m": [500, 500]},
        "usds": {
            "buildings": {
                "highrise": [{"usd": "Down/BG_Building_A.usd",
                              "material": "concrete", "tags": ["intact"]},
                             {"usd": "Down/ruin_01.usd",
                              "material": "steel", "tags": ["destroyed"]}],
                "townhouse": [{"usd": "row/house_01.usdc",
                               "material": "brick", "tags": ["intact"]}],
            },
            "trees": [{"usd": "veg/Black_Oak.usd"}],
        },
    }


def _house(usd, n=1):
    return [{"category": "house", "usd": usd, "x_m": 0, "y_m": 0}] * n


def test_census_counts_per_type_not_per_placement(cfg):
    doc = C.record(cfg, _house("root/Down/BG_Building_A.usd", 12), "urban_v3")
    a = [r for r in doc["assets"] if r["type"] == "BG_Building_A"]
    assert len(a) == 1 and a[0]["count"] == 12


def test_census_matches_on_the_slug_not_the_raw_path(cfg):
    """A placement's USD has been joined to `asset_root`; the pack's has not.
    A string compare here found every tree and not one building."""
    doc = C.record(cfg, _house(
        "omniverse://nucleus/Projects/SEI-COA/Down/BG_Building_A.usd"),
        "urban_v3")
    rec = next(r for r in doc["assets"] if r["type"] == "BG_Building_A")
    assert rec["pool"] == "highrise" and rec["material"] == "concrete"
    assert not doc["unknown"]


def test_census_records_the_pool_and_the_condition_tags(cfg):
    doc = C.record(cfg, _house("x/Down/ruin_01.usd"), "urban_v3")
    rec = next(r for r in doc["assets"] if r["type"] == "ruin_01")
    assert rec["tags"] == ["destroyed"]


def test_census_separates_vegetation_from_structures(cfg):
    pl = _house("a/house_01.usdc", 3) + [
        {"category": "tree", "usd": "a/veg/Black_Oak.usd"}] * 5
    doc = C.record(cfg, pl, "urban_v3")
    assert C.used_types(doc, "structure") == ["house_01"]
    assert C.used_types(doc, "vegetation") == ["Black_Oak"]


def test_census_ignores_props(cfg):
    pl = _house("a/house_01.usdc") + [
        {"category": "bench", "usd": "a/Bench_01.usd"},
        {"category": "car", "usd": "a/Car_01.usd"}]
    doc = C.record(cfg, pl, "urban_v3")
    assert [r["type"] for r in doc["assets"]] == ["house_01"]


def test_census_admits_what_it_could_not_place_in_a_pool(cfg):
    """Dropping an unrecognised asset would make the census quietly wrong in
    the one direction that matters — it would under-report the bake."""
    doc = C.record(cfg, _house("a/Mystery_Building.usd"), "urban_v3")
    assert "Mystery_Building" in doc["unknown"]
    assert "Mystery_Building" in C.used_types(doc)


def test_used_types_is_bake_order_most_placed_first(cfg):
    pl = _house("a/house_01.usdc", 2) + _house("a/Down/BG_Building_A.usd", 9)
    doc = C.record(cfg, pl, "urban_v3")
    assert C.used_types(doc, "structure") == ["BG_Building_A", "house_01"]


def test_census_round_trips_through_a_file(cfg, tmp_path):
    doc = C.record(cfg, _house("a/house_01.usdc", 4), "urban_v3")
    path = str(tmp_path / "sub" / "census.json")
    C.write(path, doc)
    assert C.read(path) == doc


def test_reading_a_missing_census_is_not_an_error(tmp_path):
    assert C.read(str(tmp_path / "nope.json")) == {}
    assert C.read("") == {}


def test_census_records_what_would_reproduce_it(cfg):
    """A census with no seed or region cannot be checked against the scene it
    claims to describe, and a library outlives the session that baked it."""
    doc = C.record(cfg, _house("a/house_01.usdc"), "urban_v3")
    assert doc["seed"] == 42
    assert doc["region_m"] == [500, 500]
    assert doc["asset_pack"] == "test_pack"
    assert doc["recorded_at"]


# --------------------------------------------------------------------------
# Diversity — "is the same building standing within sight", as a number
# --------------------------------------------------------------------------

def _at(usd, pts):
    return [{"category": "house", "usd": usd, "x_m": x, "y_m": y}
            for x, y in pts]


def test_twin_distance_is_to_the_nearest_OTHER_copy():
    """Not to itself, which would make every distance 0 and every scene look
    perfect."""
    assert C._twin_distances([(0.0, 0.0), (30.0, 40.0)]) == [50.0, 50.0]


def test_a_lone_copy_has_no_twin_distance():
    assert C._twin_distances([(0.0, 0.0)]) == []
    assert C._twin_distances([]) == []


def test_twin_distance_takes_the_nearest_not_the_last():
    d = C._twin_distances([(0.0, 0.0), (10.0, 0.0), (500.0, 0.0)])
    assert d[0] == 10.0 and d[1] == 10.0 and d[2] == 490.0


def test_near_50m_counts_buildings_not_types(cfg):
    """Two towers 10 m apart is ONE defect a viewer sees twice, and the metric
    has to be per building or a type placed 25 times scores the same as one
    placed twice."""
    doc = C.record(cfg, _at("a/house_01.usdc", [(0, 0), (10, 0), (900, 900)]),
                   "urban_v3")
    d = doc["diversity"]
    assert d["buildings"] == 3
    # Two of the three are within 50 m of a twin; the far one is not.
    assert d["near_50m"] == pytest.approx(2 / 3, abs=1e-3)
    assert d["closest_twin_m"] == 10.0


def test_diversity_reports_what_the_pack_could_have_offered(cfg):
    """`types_used` alone cannot say whether a layout is diverse — 3 of 3 and
    3 of 80 are the same number and opposite verdicts."""
    doc = C.record(cfg, _at("a/house_01.usdc", [(0, 0)]), "urban_v3")
    # The fixture pack holds two intact buildings and one authored ruin.
    assert doc["diversity"]["types_available"] == 2
    assert doc["diversity"]["types_used"] == 1


def test_authored_ruins_are_not_counted_as_available(cfg):
    """`buildings.*` holds intact art and pre-authored ruins in the same
    pools. Counting the ruins would flatter every diversity number."""
    doc = C.record(cfg, _at("a/house_01.usdc", [(0, 0)]), "urban_v3")
    assert doc["diversity"]["types_available"] == 2      # not 3


def test_diversity_records_the_knobs_it_was_measured_under(cfg):
    """Two censuses are only comparable if each says what it was run with."""
    cfg["layout"]["districts"] = {"repeat_penalty": 2.2,
                                  "repeat_radius_m": 150.0}
    doc = C.record(cfg, _at("a/house_01.usdc", [(0, 0)]), "urban_v3")
    k = doc["diversity"]["knobs"]
    assert k["repeat_penalty"] == 2.2 and k["repeat_radius_m"] == 150.0


def test_per_type_twin_stats_reach_the_record(cfg):
    doc = C.record(cfg, _at("a/house_01.usdc", [(0, 0), (10, 0)]), "urban_v3")
    rec = next(r for r in doc["assets"] if r["type"] == "house_01")
    assert rec["nearest_twin_m"] == 10.0
    # The working list is scratch and must not survive into the file.
    assert "_twins" not in rec and "_pts" not in rec


def test_a_census_survives_placements_with_no_coordinates(cfg):
    """`used_by_scene`-style placement lists are not guaranteed to carry
    positions, and a census that raised on one would take the whole run."""
    doc = C.record(cfg, [{"category": "house", "usd": "a/house_01.usdc"}],
                   "urban_v3")
    assert doc["diversity"]["buildings"] == 1
    assert doc["diversity"]["closest_twin_m"] == 0.0


# --------------------------------------------------------------------------
# Bake ORDER — what makes a bake that is stopped early still useful
# --------------------------------------------------------------------------

class _Args:
    used_only = False
    only = ""
    census_only = False
    # Every selection flag `_select` reads needs a default here. Leaving one
    # out does not skip that branch, it raises AttributeError inside the
    # function under test -- which is how `--exclude` and then `--cells` each
    # broke all five census tests the moment they were added.
    exclude = ""
    cells = ""


def _items(names):
    from archetypes import plan as P
    from disaster import levels as L
    return [P.Item(n, L.STRUCTURE, n, "library", ["pristine"]) for n in names]


def _plan(monkeypatch, names):
    from archetypes import bake_cli as B
    from archetypes import plan as P
    monkeypatch.setattr(P, "build_plan",
                        lambda *a, **k: _items(names))
    return B


def test_census_puts_every_used_type_before_every_unused_one(monkeypatch, cfg):
    """The whole point of the ordering. A bake stopped by the disk or by the
    morning must have finished the archetypes a scene actually references."""
    B = _plan(monkeypatch, ["stock_a", "placed_twice", "stock_b",
                            "placed_often"])
    doc = C.record(cfg, _house("a/placed_often.usd", 9)
                   + _house("a/placed_twice.usd", 2), "urban_v3")
    _only, items = B._select(cfg, "earthquake", _Args(), doc)
    got = [i.type for i in items]
    assert got[:2] == ["placed_often", "placed_twice"]
    assert set(got[2:]) == {"stock_a", "stock_b"}


def test_census_is_an_order_not_a_filter_by_default(monkeypatch, cfg):
    """The rest of the pack is still worth baking — just second."""
    B = _plan(monkeypatch, ["stock_a", "placed"])
    doc = C.record(cfg, _house("a/placed.usd"), "urban_v3")
    _only, items = B._select(cfg, "earthquake", _Args(), doc)
    assert len(items) == 2


def test_census_only_narrows_to_what_the_scene_places(monkeypatch, cfg):
    B = _plan(monkeypatch, ["stock_a", "placed"])
    args = _Args()
    args.census_only = True
    doc = C.record(cfg, _house("a/placed.usd"), "urban_v3")
    _only, items = B._select(cfg, "earthquake", args, doc)
    assert [i.type for i in items] == ["placed"]


def test_a_census_run_always_passes_an_explicit_selection(monkeypatch, cfg):
    """`run(only=None)` means 'everything' and takes the PLAN's order, which
    would silently discard the census ordering computed just above."""
    B = _plan(monkeypatch, ["stock_a", "placed"])
    doc = C.record(cfg, _house("a/placed.usd"), "urban_v3")
    only, _items = B._select(cfg, "earthquake", _Args(), doc)
    assert only is not None and len(only) == 2


def test_no_census_still_means_everything(monkeypatch, cfg):
    B = _plan(monkeypatch, ["a", "b"])
    only, _items = B._select(cfg, "earthquake", _Args(), None)
    assert only is None


# --------------------------------------------------------------------------
# Preview framing — the arithmetic, without a viewport
# --------------------------------------------------------------------------

def test_a_bigger_subject_is_photographed_from_further_back():
    assert PV._fit_distance((80, 80, 80)) > PV._fit_distance((20, 20, 20))


def _in_frame(size, d, aim_drop=0.0, bearing=225.0, elev=22.0):
    """Every bbox corner inside the frustum at distance *d*. The check the
    solver is supposed to satisfy, done independently."""
    e, u, w = PV._basis(bearing, elev)
    hx, hy, hz = (0.5 * v for v in size)
    worst_h = worst_v = 0.0
    for sx in (-hx, hx):
        for sy in (-hy, hy):
            for sz in (-hz, hz):
                c = (sx, sy, sz + aim_drop)
                depth = d - sum(a * b for a, b in zip(c, e))
                if depth <= 0:
                    return False, 9e9, 9e9
                worst_h = max(worst_h, abs(sum(a * b for a, b in zip(c, u)))
                              / depth / PV._TAN_H)
                worst_v = max(worst_v, abs(sum(a * b for a, b in zip(c, w)))
                              / depth / PV._TAN_V)
    return True, worst_h, worst_v


@pytest.mark.parametrize("size", [
    (40.0, 14.0, 8.0),      # long low workshop — the case a sphere fit wastes
    (30.0, 30.0, 96.0),     # tower, bound vertically
    (20.0, 20.0, 20.0),
    (12.0, 9.0, 5.0),       # the smallest thing in the library
])
def test_every_bbox_corner_is_inside_the_frame(size):
    drop = 0.15 * size[2]
    d = PV._fit_distance(size, drop)
    ok, h, v = _in_frame(size, d, drop)
    assert ok and h <= 1.0 and v <= 1.0


@pytest.mark.parametrize("size", [
    (40.0, 14.0, 8.0), (30.0, 30.0, 96.0), (20.0, 20.0, 20.0),
])
def test_the_fit_is_TIGHT_not_merely_safe(size):
    """A frame the subject fills 40% of is a frame you cannot read damage off.
    One axis must be near its limit, or the solver is just backing away."""
    drop = 0.15 * size[2]
    d = PV._fit_distance(size, drop)
    _ok, h, v = _in_frame(size, d, drop)
    # `margin` is 1.06, so the binding axis should sit just inside 1/1.06.
    assert max(h, v) > 0.85


def test_the_exact_fit_beats_the_bounding_sphere_it_replaced(size=(40, 14, 8)):
    """The sphere is the box's DIAGONAL in every direction, so a long low
    building was photographed as if it were as tall as it is long."""
    import math
    drop = 0.15 * size[2]
    r = 0.5 * math.sqrt(sum(v * v for v in size)) + drop
    sphere = 1.12 * max(r / PV._TAN_H, r / PV._TAN_V)
    assert PV._fit_distance(size, drop) < 0.75 * sphere


def test_aiming_low_is_paid_for_not_ignored():
    """Aiming below centre without widening crops the top by exactly that
    much — on a tower, the part that says whether it is still standing."""
    assert PV._fit_distance((30, 30, 96), 20.0) > PV._fit_distance(
        (30, 30, 96), 0.0)


def test_the_plumb_basis_does_not_degenerate():
    """Looking straight down, the view direction is parallel to the world up
    and the cross product that builds the right vector collapses."""
    e, u, w = PV._basis(0.0, 90.0)
    assert e[2] == pytest.approx(1.0)
    for vec in (u, w):
        assert sum(c * c for c in vec) == pytest.approx(1.0, abs=1e-6)
    assert sum(a * b for a, b in zip(u, w)) == pytest.approx(0.0, abs=1e-6)


def test_the_vertical_half_angle_is_the_tighter_one():
    assert PV._TAN_V < PV._TAN_H


def test_preview_paths_are_relative_to_the_library():
    """`usd` is relative to the manifest so a library survives being moved or
    copied to Nucleus; a preview that was not would break on the same move."""
    d = PV.preview_dir("/some/lib/earthquake")
    assert not os.path.isabs(os.path.relpath(d, "/some/lib/earthquake"))
    assert os.path.basename(d) == PV.PREVIEW_DIR


def test_preview_has_no_module_level_kit_import():
    """`scene_gen` sits at the repo root because it is sim-agnostic. Every
    `omni` / `carb` / `pxr` import in `preview` must be inside a function, or
    importing the module on the host — which every test here does — dies."""
    import ast

    tree = ast.parse(open(PV.__file__).read())
    top = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))]
    names = {getattr(n, "module", None) or ""
             for n in top} | {a.name for n in top
                              for a in getattr(n, "names", ())}
    assert not {n.split(".")[0] for n in names if n} & {"omni", "carb", "pxr"}


# --------------------------------------------------------------------------
# Manifest — the two halves have to agree
# --------------------------------------------------------------------------

def test_merge_keeps_records_a_partial_bake_did_not_redo(tmp_path):
    path = str(tmp_path / "manifest.json")
    lib.write_manifest(path, [
        {"type": "a", "level": "cracked", "usd": "a_cracked.usd"},
        {"type": "b", "level": "cracked", "usd": "b_cracked.usd"}])
    lib.merge_manifest(path, [
        {"type": "a", "level": "cracked", "usd": "a_cracked.usd",
         "pipeline_version": "9"}])
    doc = lib.read_manifest(path)
    by = {r["type"]: r for r in doc["archetypes"]}
    assert by["a"]["pipeline_version"] == "9"
    assert "pipeline_version" not in by["b"]


def test_audit_reads_a_manifest_off_disk(tmp_path):
    """`version.py <manifest>` is the one-command answer to 'is this library
    current', so it has to work against what `write_manifest` produces."""
    path = str(tmp_path / "manifest.json")
    lib.write_manifest(path, [dict(V.stamp(), type="a", level="cracked",
                                   usd="a.usd"),
                              {"type": "b", "level": "cracked",
                               "usd": "b.usd"}])
    with open(path) as fh:
        res = V.audit(json.load(fh))
    assert len(res["current"]) == 1 and len(res["stale"]) == 1


# --------------------------------------------------------------------------
# Export — the paths only a DAMAGED archetype takes
# --------------------------------------------------------------------------

def test_portable_asset_keeps_a_bare_mdl_module_bare():
    """`disaster/bake._portable_asset` had no `from pxr import Sdf` while
    every other function in that module imports pxr locally, so all three of
    its `Sdf.AssetPath` calls raised `NameError`.

    It went unnoticed because `pristine` never reaches them: case 1 below is
    the bare `OmniPBR.mdl` that every `FractureCore_*` material carries, so
    only a rung that was actually fractured could trip it. A full bake would
    have exported nothing but pristine shells.
    """
    pxr = pytest.importorskip("pxr")
    from pxr import Sdf
    from disaster import bake as B

    got = B._portable_asset(Sdf.AssetPath("OmniPBR.mdl"), "/tmp/out")
    assert got is not None and got.path == "OmniPBR.mdl"


def test_portable_asset_leaves_nucleus_and_foreign_paths_alone():
    """Both return None so `anchor` handles them — and both are reached
    AFTER the bare-MDL branch, so they exercise the same missing import."""
    pytest.importorskip("pxr")
    from pxr import Sdf
    from disaster import bake as B

    assert B._portable_asset(Sdf.AssetPath("omniverse://h/a/b.png"), "/o") is None
    assert B._portable_asset(Sdf.AssetPath("/etc/passwd"), "/o") is None


def test_every_pxr_name_used_in_disaster_bake_is_imported():
    """The general form of the bug. This module deliberately has no
    module-level `pxr` import — it is read on the host, where Kit's USD build
    is not importable until Kit starts — so every function that touches a
    `pxr` name must import it itself, and a missing one is invisible until
    that branch runs inside a bake."""
    import ast

    from disaster import bake as B

    tree = ast.parse(open(B.__file__).read())
    pxr_names = {"Sdf", "Usd", "UsdGeom", "UsdShade", "UsdPhysics", "Gf", "Vt"}
    bad = []
    # TOP-LEVEL functions only. `ast.walk` reaches into nested defs, so a
    # closure's usage is attributed to the parent whose import is actually in
    # scope for it — checking nested defs separately reports `anchor` and
    # `one` inside `_rebuild_material`, which are perfectly fine.
    for fn in [n for n in tree.body
               if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]:
        imported = {a.asname or a.name
                    for n in ast.walk(fn) if isinstance(n, ast.ImportFrom)
                    and (n.module or "").startswith("pxr")
                    for a in n.names}
        used = {n.value.id for n in ast.walk(fn)
                if isinstance(n, ast.Attribute)
                and isinstance(n.value, ast.Name)
                and n.value.id in pxr_names}
        missing = used - imported
        if missing:
            bad.append(f"{fn.name}: {sorted(missing)}")
    assert bad == [], "pxr names used without a local import: " + "; ".join(bad)


# --------------------------------------------------------------------------
# Stage B switches — what a RUN can override that a config cannot
# --------------------------------------------------------------------------

def test_ruin_swaps_default_on(monkeypatch):
    from disaster import disaster_stage as DS
    monkeypatch.delenv("SCENE_RUIN_SWAPS", raising=False)
    assert DS._ruin_swaps_wanted({}) is True


def test_ruin_swaps_config_can_turn_them_off(monkeypatch):
    from disaster import disaster_stage as DS
    monkeypatch.delenv("SCENE_RUIN_SWAPS", raising=False)
    assert DS._ruin_swaps_wanted({"ruin_swaps": False}) is False


@pytest.mark.parametrize("val,want", [
    ("0", False), ("false", False), ("no", False), ("off", False),
    ("1", True), ("true", True),
])
def test_ruin_swaps_env_wins_over_config(monkeypatch, val, want):
    """The container bakes a fixed environment in at creation, so a run has to
    be able to override a config it cannot edit — the same rule
    `_archetypes_wanted` follows, and the reason `SCENE_ARCHETYPES=0` in
    `.env` silently disabled the library for a preset that wanted it on."""
    from disaster import disaster_stage as DS
    monkeypatch.setenv("SCENE_RUIN_SWAPS", val)
    assert DS._ruin_swaps_wanted({"ruin_swaps": not want}) is want


def test_an_empty_env_var_does_not_count_as_off(monkeypatch):
    """`docker-compose` passes `SCENE_RUIN_SWAPS=${SCENE_RUIN_SWAPS:-}`, so an
    unset knob arrives as the EMPTY STRING, not as absent. Treating that as
    'off' would disable swaps for every run on this container."""
    from disaster import disaster_stage as DS
    monkeypatch.setenv("SCENE_RUIN_SWAPS", "")
    assert DS._ruin_swaps_wanted({}) is True


# --- --cells: exact rungs, not a cross product -----------------------------

def _cell_args(cells):
    a = _Args()
    a.cells = cells
    return a


def test_cells_selects_exact_rungs_not_a_cross_product(monkeypatch, cfg):
    """The whole reason `--cells` exists. `--only A,B` crossed with a level
    filter bakes four rungs when the picker asked for two, and each extra one
    OVERWRITES a file the user may have hand-posed."""
    from archetypes import plan as P
    from disaster import levels as L
    ladder = ["pristine", "cracked", "soft_storey"]
    monkeypatch.setattr(P, "build_plan", lambda *a, **k: [
        P.Item("A", L.STRUCTURE, "A", "library", list(ladder)),
        P.Item("B", L.STRUCTURE, "B", "library", list(ladder))])
    from archetypes import bake_cli as B
    _only, items = B._select(cfg, "earthquake",
                             _cell_args("A:cracked,B:soft_storey"), {})
    got = {(i.type, lv) for i in items for lv in i.levels}
    assert got == {("A", "cracked"), ("B", "soft_storey")}


def test_cells_drops_types_it_did_not_name(monkeypatch, cfg):
    from archetypes import plan as P
    from disaster import levels as L
    monkeypatch.setattr(P, "build_plan", lambda *a, **k: [
        P.Item("A", L.STRUCTURE, "A", "library", ["pristine", "cracked"]),
        P.Item("B", L.STRUCTURE, "B", "library", ["pristine", "cracked"])])
    from archetypes import bake_cli as B
    _only, items = B._select(cfg, "earthquake", _cell_args("A:cracked"), {})
    assert [i.type for i in items] == ["A"]
    assert items[0].levels == ["cracked"]


def test_cells_returns_an_explicit_selection(monkeypatch, cfg):
    """`None` downstream means "everything", which would silently undo the
    trimming and re-cut the whole ladder."""
    from archetypes import plan as P
    from disaster import levels as L
    monkeypatch.setattr(P, "build_plan", lambda *a, **k: [
        P.Item("A", L.STRUCTURE, "A", "library", ["pristine", "cracked"])])
    from archetypes import bake_cli as B
    only, _items = B._select(cfg, "earthquake", _cell_args("A:cracked"), {})
    assert only is not None


# --- interior structure stays out of the rubble ----------------------------

def test_interior_prefix_agrees_between_the_two_modules():
    """`bake.py` names the merged mesh by matching this prefix and must stay
    importable without `pxr`, so it carries its own copy. If the two drift the
    interior silently goes back to being poured into `rubble_<core>`."""
    from disaster import bake as BK
    from disaster import mesh_damage as MD
    assert BK._INTERIOR_PREFIX == MD.INTERIOR_PREFIX


def test_cells_survives_the_rebuild_inside_run(monkeypatch, cfg):
    """`run` rebuilds the plan and filters it by (type, kind) only, so a level
    restriction has to be re-applied there or it is silently dropped and whole
    ladders get re-cut."""
    from archetypes import bake as A
    from archetypes import plan as P
    from disaster import levels as L
    items = [P.Item("A", L.STRUCTURE, "A", "library",
                    ["pristine", "cracked", "pancaked"]),
             P.Item("B", L.STRUCTURE, "B", "library",
                    ["pristine", "cracked", "pancaked"])]
    got = A._trim_to_cells(items, {("A", "cracked"), ("B", "pancaked")})
    assert {(i.type, lv) for i in got for lv in i.levels} == {
        ("A", "cracked"), ("B", "pancaked")}


def test_no_cells_leaves_the_ladder_alone(cfg):
    from archetypes import bake as A
    from archetypes import plan as P
    from disaster import levels as L
    items = [P.Item("A", L.STRUCTURE, "A", "library", ["pristine", "cracked"])]
    assert A._trim_to_cells(items, None) is items


def test_cells_string_parses_to_pairs():
    from archetypes.bake_cli import _cells_set
    assert _cells_set("A:cracked, B:soft_storey") == {
        ("A", "cracked"), ("B", "soft_storey")}
    assert _cells_set("") is None
