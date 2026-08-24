"""Stage B, step 5: reference the pre-baked archetype instead of damaging live.

The library is gitignored and takes an Isaac Sim session to bake, so these
tests plant a SYNTHETIC manifest and check the swap logic against it. That is
the part worth regressing on anyway — whether a scene picks the right archetype
and stops asking for live damage — rather than whether PhysX settled correctly.
"""

import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import compile_disaster as cd                                    # noqa: E402
import generate_scene                                            # noqa: E402
import scene_generator as sg                                     # noqa: E402
from archetypes import library as lib                            # noqa: E402
from archetypes import plan as P                                 # noqa: E402
from disaster import disaster_stage as ds                        # noqa: E402
from disaster import levels as L                                 # noqa: E402

PRESET, SEED, SEVERITY = "urban_small", 42, 0.8
DTYPE = "earthquake"


def _config():
    cfg = cd.load_scene_config(PRESET)
    spec = {"locale": "urban", "disaster-type": DTYPE, "severity": SEVERITY,
            "asset-pack": cfg.get("asset_pack"), "seed": SEED,
            "region_m": list(cfg["layout"]["region_m"])}
    import yaml
    base = yaml.safe_load(open(os.path.join(
        _SCENE_GEN, "config", "low_level", "default.yaml")))
    out = cd.compile_spec(spec, base)
    return sg.resolve_asset_pack(out)


@pytest.fixture(scope="module")
def config():
    return _config()


@pytest.fixture
def full_library(config, monkeypatch):
    """A manifest covering every planned (type, level) for this config."""
    recs = []
    for it in P.build_plan(config, DTYPE):
        for _t, lv in it.combos:
            recs.append({"type": it.type, "level": lv, "kind": it.kind,
                         "source": str(it.source),
                         "usd": lib.archetype_name(it.type, lv) + ".usd"})
    library = lib.Library(
        path=os.path.join(_SCENE_GEN, "assets", "archetypes", DTYPE,
                          lib.MANIFEST_NAME),
        doc={"disaster": DTYPE, "archetypes": recs})
    monkeypatch.setattr(ds, "_ARCH_CACHE", {DTYPE: library})
    return library


@pytest.fixture
def no_library(monkeypatch):
    monkeypatch.setattr(ds, "_ARCH_CACHE", {DTYPE: None})


def _build(config):
    resolver = sg._make_resolver(config)
    placements, layout, _ = generate_scene.build_scene(config, resolver)
    return placements


def _damaged(placements):
    return [p for p in placements
            if p.get("category") == "house" and p.get("_damage_level")
            and p["_damage_level"] != "pristine"]


# --------------------------------------------------------------------------

def test_without_a_library_nothing_breaks(config, no_library):
    """A fresh checkout has no library — it is gitignored and needs an Isaac
    session. The scene must still build, exactly as before Stage A existed."""
    placements = _build(config)
    assert placements
    hit = [p for p in placements if p.get("_archetype")]
    assert not hit, "referenced an archetype with no library loaded"


def test_with_a_library_buildings_reference_archetypes(config, full_library):
    placements = _build(config)
    hit = [p for p in placements if p.get("_archetype")]
    assert hit, "a full library was loaded and nothing referenced it"
    for p in hit:
        assert "assets/archetypes/" in p["usd"]
        assert p["usd"].startswith("airstack://scene_gen/")


def test_archetypes_replace_live_mesh_damage(config, full_library):
    """The payoff. A building with an archetype must NOT also be queued for
    live deformation — that is the per-scene physics Stage A abolishes."""
    placements = _build(config)
    for p in placements:
        if p.get("_archetype"):
            assert "_mesh_damage" not in p, (
                "building has both an archetype and a live-damage marker; "
                "it would be fractured at scene time for nothing")


def test_a_full_library_beats_the_max_buildings_budget(config, full_library):
    """Stage A's whole point: damage stops being rationed.

    Live mesh damage carries `fracture.max_buildings` (50-80) because each
    building costs 35-50 s. Referencing costs nothing, so every damaged
    building can have real geometry rather than the tilt-and-sink stand-in.
    """
    budget = ((config["disaster"].get("mesh_damage") or {})
              .get("fracture") or {}).get("max_buildings")
    assert budget, "this preset has no budget to beat"

    with_lib = [p for p in _build(config) if p.get("_archetype")]
    assert len(with_lib) > budget, (
        f"only {len(with_lib)} buildings got an archetype against a live "
        f"budget of {budget} — the library is not buying anything")


def test_archetype_geometry_is_taken_as_authored(config, full_library):
    """A baked archetype is re-centred, metres, Z-up, with its transform baked.

    Carrying the SOURCE asset's scale across the swap is how a 0.01-scaled
    Nucleus building comes back a hundred times too small.
    """
    for p in _build(config):
        if p.get("_archetype"):
            assert p["scale"] == 1.0
            assert p["axis_up"] == "Z"
            assert p["z_m"] == 0.0


def test_level_matches_the_field(config, full_library):
    """The referenced level must be the one the field and ladder agree on."""
    from disaster.field import make_damage_field
    dis = config["disaster"]
    region = tuple(float(v) for v in config["layout"]["region_m"])
    f = make_damage_field(dis.get("field"), (0.0, 0.0, region[0], region[1]))
    sev = float(dis["severity"])
    for p in _build(config):
        if not p.get("_archetype"):
            continue
        want = L.level_at(DTYPE, L.local_damage(f(p["x_m"], p["y_m"]), sev))
        assert p["_damage_level"] == want.name


def test_a_partial_library_falls_back_but_still_places(config, monkeypatch):
    """A bake that half-failed must still assemble, degrading down the ladder
    rather than leaving holes."""
    recs = []
    for it in P.build_plan(config, DTYPE):
        recs.append({"type": it.type, "level": "cracked", "kind": it.kind,
                     "source": str(it.source),
                     "usd": f"{it.type}_cracked.usd"})
    library = lib.Library(
        path=os.path.join(_SCENE_GEN, "assets", "archetypes", DTYPE,
                          lib.MANIFEST_NAME),
        doc={"disaster": DTYPE, "archetypes": recs})
    monkeypatch.setattr(ds, "_ARCH_CACHE", {DTYPE: library})
    placements = _build(config)
    hit = [p for p in placements if p.get("_archetype")]
    assert hit
    assert all("_cracked.usd" in p["usd"] for p in hit)
    assert library.misses, "fell back silently instead of recording it"
