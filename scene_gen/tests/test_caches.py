"""The two offline caches: asset measurements, and the two-tier scene cache.

Both are optimisations, so the tests are mostly about the ways an optimisation
can be WRONG rather than slow: serving a stale entry, keying on something that
does not determine the output, or keying on something that does not.
"""

import json
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import compile_disaster as cd                                    # noqa: E402
import measure_cache as mc                                       # noqa: E402
import scene_cache as sc                                         # noqa: E402
import scene_generator as sg                                     # noqa: E402


# --------------------------------------------------------------------------
# Measurement cache
# --------------------------------------------------------------------------

def test_round_trips_a_measurement(tmp_path):
    c = mc.MeasureCache(str(tmp_path / "m.json"), autosave=False)
    fp = {"sx": 1.0, "sy": 2.0, "sz": 3.0, "base": 0.0}
    c.put("/no/such.usd", "Z", fp)
    hit, got = c.get("/no/such.usd", "Z")
    assert hit and got == fp


def test_caches_a_failed_measurement_for_a_LOCAL_file(tmp_path):
    """For a local file, "could not measure" is a property of the ASSET —
    missing or broken. Stable, worth remembering, and self-healing because
    creating the file changes its stat token."""
    asset = tmp_path / "broken.usd"
    asset.write_text("")
    c = mc.MeasureCache(str(tmp_path / "m.json"), autosave=False)
    c.put(str(asset), "Z", None)
    hit, got = c.get(str(asset), "Z")
    assert hit and got is None


def test_does_NOT_cache_a_failed_measurement_for_an_UNREACHABLE_REMOTE(tmp_path):
    """THE BUG THIS TEST EXISTS FOR.

    For a remote asset, "could not measure" means "I could not reach Nucleus"
    — a property of the machine, not the asset. Caching it on a host that
    cannot reach the server made the CONTAINER, which can, skip measuring and
    pack the whole city against `fallback_sizes`: buildings overflowed their
    blocks and ground tiles were laid with gaps. 156 of 214 entries were
    poisoned this way.
    """
    c = mc.MeasureCache(str(tmp_path / "m.json"), autosave=False)
    c.put("omniverse://server/nope.usd", "Z", None)
    assert not c.get("omniverse://server/nope.usd", "Z")[0]
    assert len(c) == 0


def test_a_SUCCESSFUL_remote_measurement_is_still_cached(tmp_path):
    """Only failures are refused. A real measurement from a reachable server
    is exactly what this cache is for."""
    c = mc.MeasureCache(str(tmp_path / "m.json"), autosave=False)
    fp = {"sx": 12.0, "sy": 10.6, "sz": 6.4, "base": 0.0}
    c.put("omniverse://server/ok.usd", "Z", fp)
    hit, got = c.get("omniverse://server/ok.usd", "Z")
    assert hit and got == fp


def test_axis_up_is_part_of_the_key(tmp_path):
    c = mc.MeasureCache(str(tmp_path / "m.json"), autosave=False)
    c.put("/a.usd", "Z", {"sx": 1.0})
    assert not c.get("/a.usd", "Y")[0]


def test_a_changed_local_file_invalidates(tmp_path):
    """`prepare_assets.py` re-converting an asset at a new `target-size-m` must
    not leave the old footprint behind — that is how a manhole stayed 8 m."""
    asset = tmp_path / "a.usd"
    asset.write_text("one")
    c = mc.MeasureCache(str(tmp_path / "m.json"), autosave=False)
    c.put(str(asset), "Z", {"sx": 8.0})
    assert c.get(str(asset), "Z")[0]
    os.utime(asset, (0, 0))                      # same size, different mtime
    assert not c.get(str(asset), "Z")[0]


def test_survives_a_corrupt_file(tmp_path):
    """An optimisation must not be able to stop a scene from building."""
    p = tmp_path / "m.json"
    p.write_text("{ this is not json")
    assert len(mc.MeasureCache(str(p), autosave=False)) == 0


def test_persists_across_instances(tmp_path):
    p = str(tmp_path / "m.json")
    a = mc.MeasureCache(p, autosave=False)
    a.put("/a.usd", "Z", {"sx": 1.0})
    a.save()
    assert mc.MeasureCache(p, autosave=False).get("/a.usd", "Z")[0]


def test_resolver_takes_a_cache_and_a_cold_one_is_opt_out():
    """`MeasureCache` defines `__len__`, so an EMPTY cache is falsy — a
    `cache or None` here silently discarded the cache a cold run was handed."""
    cfg = {"asset_scale": 1.0, "fallback_sizes": {}, "measure_usds": True}
    empty = mc.MeasureCache(autosave=False)
    empty._data = {}
    assert not empty                                  # falsy, and that is fine
    assert sg._make_resolver(cfg, cache=empty)._cache is empty
    assert sg._make_resolver(cfg, cache=False)._cache is None


# --------------------------------------------------------------------------
# Scene cache
# --------------------------------------------------------------------------

@pytest.fixture(scope="module")
def cfgs():
    """The same preset at three severities."""
    import yaml
    base = yaml.safe_load(open(os.path.join(
        _SCENE_GEN, "config", "low_level", "default.yaml")))
    spec = {"locale": "urban", "disaster-type": "earthquake",
            "asset-pack": "urban_intact", "seed": 42, "region_m": [105, 105]}
    out = {}
    for sev in (0.0, 0.5, 0.9):
        s = dict(spec, severity=sev)
        out[sev] = sg.resolve_asset_pack(
            cd.compile_spec(s, yaml.safe_load(yaml.dump(base))))
    return out


def test_severities_share_one_pristine_key(cfgs):
    """THE POINT OF TWO TIERS. A locale and a seed fix the layout; severity
    only decides what happens to it. If a sweep produced several tier-1 keys,
    the scenes would not be comparable and the cache would be hiding it."""
    keys = {sc.pristine_key(c) for c in cfgs.values()}
    assert len(keys) == 1


def test_severities_get_distinct_disaster_keys(cfgs):
    keys = {sc.disaster_key(cfgs[s]) for s in (0.5, 0.9)}
    assert len(keys) == 2


def test_severity_zero_is_the_pristine_entry(cfgs):
    assert sc.is_pristine(cfgs[0.0])
    assert not sc.is_pristine(cfgs[0.9])
    c = sc.SceneCache("/tmp/nope")
    assert c.scene_usd(cfgs[0.0]).endswith(sc.PRISTINE_NAME)
    assert c.scene_usd(cfgs[0.9]).endswith(sc.SCENE_NAME)


def test_disaster_key_covers_the_whole_block(cfgs):
    """Not just type and severity. `urban_quake_tiny` overrides the field
    geometry, so two scenes can share a nominal severity and differ — serving
    one for the other would be the worst failure this cache could have."""
    a = cfgs[0.9]
    b = dict(a)
    b["disaster"] = dict(a["disaster"])
    b["disaster"]["field"] = dict(a["disaster"]["field"], radius_m=999.0)
    assert sc.disaster_key(a) != sc.disaster_key(b)


def test_pack_contents_not_just_its_name(cfgs):
    """Two configs naming `urban` are the same scene only if `urban` still
    means the same assets."""
    a = cfgs[0.0]
    b = dict(a)
    b["usds"] = dict(a["usds"])
    b["usds"]["buildings"] = {"intact": ["something_new.usd"]}
    assert sc.pristine_key(a) != sc.pristine_key(b)


def test_region_is_part_of_the_pristine_key(cfgs):
    a = cfgs[0.0]
    b = dict(a)
    b["layout"] = dict(a["layout"], region_m=[400, 400])
    assert sc.pristine_key(a) != sc.pristine_key(b)


def test_layout_round_trips(tmp_path, cfgs):
    c = sc.SceneCache(str(tmp_path))
    assert c.layout(cfgs[0.0]) == {}
    c.put_layout(cfgs[0.0], {"region": [0, 0, 1, 1], "blocks": [[0, 0, 1, 1]]})
    assert c.layout(cfgs[0.0])["blocks"] == [[0, 0, 1, 1]]
    # …and it is shared with every severity, because it is tier 1.
    assert c.layout(cfgs[0.9])["blocks"] == [[0, 0, 1, 1]]


def test_miss_then_hit(tmp_path, cfgs):
    c = sc.SceneCache(str(tmp_path))
    assert c.get(cfgs[0.9]) == ""
    p = c.reserve(cfgs[0.9])
    open(p, "w").write("")
    assert c.get(cfgs[0.9]) == p


def test_entries_lists_the_tree(tmp_path, cfgs):
    c = sc.SceneCache(str(tmp_path))
    c.put_layout(cfgs[0.0], {})
    for sev in (0.5, 0.9):
        open(c.reserve(cfgs[sev]), "w").write("")
    entries = c.entries()
    assert len(entries) == 1
    meta, kids = entries[0]
    assert meta["tier"] == "pristine" and len(kids) == 2
    assert {k["severity"] for k in kids} == {0.5, 0.9}


# --------------------------------------------------------------------------
# Guessed footprints must never reach the cache
#
# A guessed footprint is not a slightly-worse version of the same scene. Block
# sizing is driven by how big the buildings are, so a run that could not
# measure its assets produced a DIFFERENT LAYOUT — measured at 638 placements
# and 6 buildings against 784 and 4, same config, same seed. Nothing
# downstream can tell the two apart, which is why the cache has to.
# --------------------------------------------------------------------------

def test_the_resolver_records_which_assets_it_had_to_guess():
    r = sg.SizeResolver(asset_scale=1.0, fallback_sizes={"house": [4.0, 4.0]},
                        measure=False)
    assert r.fallbacks == set()
    r.get("/nonexistent/whatever.usd", "house")
    assert "/nonexistent/whatever.usd" in r.fallbacks


def test_a_measured_asset_is_not_recorded_as_a_guess(tmp_path):
    # `measure=False` means every asset is a guess by construction; the point
    # here is that the set tracks the FALLBACK branch and not every lookup.
    r = sg.SizeResolver(asset_scale=1.0, fallback_sizes={}, measure=False)
    r.get("/a.usd", "house")
    r.get("/a.usd", "house")
    assert r.fallbacks == {"/a.usd"}          # once per asset, not per lookup


def test_a_marked_entry_is_a_miss(tmp_path, cfgs):
    c = sc.SceneCache(str(tmp_path))
    p = c.reserve(cfgs[0.9])
    open(p, "w").write("")
    assert c.get(cfgs[0.9]) == p              # clean entry serves

    c.mark_footprint_fallback(cfgs[0.9], ["omniverse://x.usd"])
    assert c.get(cfgs[0.9]) == ""             # …and now it does not
    assert c.get(cfgs[0.9], allow_fallback=True) == p
    assert c.footprint_fallback(cfgs[0.9]) == ["omniverse://x.usd"]


def test_marking_survives_a_reread(tmp_path, cfgs):
    """The mark lives in meta.json, so a LATER process still refuses it.

    The failure this guards against is not one run serving itself a bad entry
    — it is the next run, days later, finding it and loading it silently.
    """
    c = sc.SceneCache(str(tmp_path))
    open(c.reserve(cfgs[0.9]), "w").write("")
    c.mark_footprint_fallback(cfgs[0.9], ["omniverse://x.usd"])
    assert sc.SceneCache(str(tmp_path)).get(cfgs[0.9]) == ""


def test_marking_does_not_clobber_the_rest_of_the_meta(tmp_path, cfgs):
    c = sc.SceneCache(str(tmp_path))
    open(c.reserve(cfgs[0.9]), "w").write("")
    c.mark_footprint_fallback(cfgs[0.9], ["omniverse://x.usd"])
    meta = json.load(open(os.path.join(c.scene_dir(cfgs[0.9]), sc.META_NAME)))
    assert meta["tier"] == "disaster" and meta["severity"] == 0.9


# --------------------------------------------------------------------------
# Write-through persistence
# --------------------------------------------------------------------------

def test_a_remote_measurement_is_flushed_immediately(tmp_path):
    """Because the process that makes it never exits normally.

    An Isaac launcher loops until the app closes and the documented iteration
    loop kills it with C-c, so `atexit` is not a path the expensive
    measurements can rely on. Remote successes are the ones a host process
    cannot reproduce at all, so they are the ones written through.
    """
    path = str(tmp_path / "m.json")
    c = mc.MeasureCache(path, autosave=False)
    c.put("omniverse://server/thing.usd", "Z", {"sx": 3.0, "sy": 4.0})
    # No save() call, no atexit — it is already on disk.
    assert mc.MeasureCache(path, autosave=False).get(
        "omniverse://server/thing.usd", "Z") == (True, {"sx": 3.0, "sy": 4.0})


def test_a_local_measurement_is_not_written_through(tmp_path):
    """A city with thousands of local props would write the file thousands of
    times, and a local measurement is cheap to redo. `atexit` covers those."""
    local = tmp_path / "asset.usd"
    local.write_text("x")
    path = str(tmp_path / "m.json")
    c = mc.MeasureCache(path, autosave=False)
    c.put(str(local), "Z", {"sx": 1.0, "sy": 1.0})
    assert not os.path.exists(path)
    assert c.save() is True                   # …but the flush still works


def test_an_unreachable_remote_asset_is_still_not_cached(tmp_path):
    """The write-through path must not resurrect the poisoning bug.

    `measure_cache`'s docstring records it: a host build cached 156 `None`s
    for Nucleus assets, and the container then trusted them and packed the
    whole city against 4x4 m guesses.
    """
    path = str(tmp_path / "m.json")
    c = mc.MeasureCache(path, autosave=False)
    c.put("omniverse://server/unreachable.usd", "Z", None)
    assert c.get("omniverse://server/unreachable.usd", "Z") == (False, None)
    assert not os.path.exists(path)


# --------------------------------------------------------------------------
# Acceptance: a host bake must be the SAME SCENE as an in-Kit generate
#
# This is the test that decides whether the bake cache can be used at all. It
# is host-runnable by comparing against a fixture captured from a real in-Kit
# run, because only Kit can reach Nucleus and therefore only Kit can produce
# the reference. Regenerate the fixture with:
#
#     python3 scene_gen/tools/load_bench.py --config urban_quake_tiny \
#         --emit-manifest scene_gen/tests/fixtures/in_kit_urban_quake_tiny_s42.json
#
# It SKIPS rather than fails when the fixture is absent, and when the host
# cannot measure the assets — a developer without Nucleus access should not see
# a red suite for a machine limitation. It fails only on the thing that
# matters: host and Kit disagreeing while both believed they could measure.
# --------------------------------------------------------------------------

_MANIFEST = os.path.join(_HERE, "fixtures", "in_kit_urban_quake_tiny_s42.json")


def test_a_host_build_matches_the_in_kit_reference():
    if not os.path.exists(_MANIFEST):
        pytest.skip(f"no in-Kit reference at {os.path.relpath(_MANIFEST, _SCENE_GEN)}"
                    " — capture one with load_bench --emit-manifest")
    with open(_MANIFEST) as fh:
        want = json.load(fh)

    import generate_scene as gs
    config = cd.load_scene_config(want.get("config", "urban_quake_tiny"))
    resolver = sg._make_resolver(config, cache=False)
    placements, _layout, _base = gs.build_scene(config, resolver)

    if resolver.fallbacks:
        pytest.skip(f"this machine could not measure {len(resolver.fallbacks)} "
                    f"asset(s) (Nucleus unreachable?), so a mismatch here would "
                    f"say nothing about the generator")

    got = {}
    for p in placements:
        got[p.get("category", "asset")] = got.get(p.get("category", "asset"), 0) + 1

    # Compare only the categories the reference recorded: the in-Kit line is
    # the base city, while `placements` here also carries what later stages
    # add. A category that IS in both must match exactly.
    for cat, n in (want.get("by_category") or {}).items():
        assert got.get(cat, 0) == n, (
            f"{cat}: host built {got.get(cat, 0)}, Kit built {n}. Footprints "
            f"drive block sizing, so this is a different LAYOUT, not a "
            f"different dressing.")
