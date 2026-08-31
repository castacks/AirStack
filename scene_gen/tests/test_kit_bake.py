#!/usr/bin/env python3
"""test_kit_bake.py — offline correctness for the GAC kit cache's KEYING
(asset+signature) and VINTAGE (fingerprint) safety net (`detail/kit_bake.py`,
2026-08-31 rework: `gac_fire.burn_gac`'s cache block now saves-on-slice
automatically — see that module's own docstring), plus a container-only
end-to-end proof that a real building's SECOND bake actually loads from the
cache its FIRST bake wrote.

    python3 scene_gen/tests/test_kit_bake.py        # host: keying/vintage only, no pxr needed
    pytest -q scene_gen/tests/test_kit_bake.py

    # in the container, against ONE real building (needs Nucleus + VTK):
    docker exec isaac-sim bash -c \\
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
       /isaac-sim/AirStack/scene_gen/tests/test_kit_bake.py --live SM_Building_02"

    # the same proof, but through the REAL gac_fire.burn_gac call site end
    # to end (slow -- see _live_two_run_proof_full's own docstring):
    docker exec isaac-sim bash -c \\
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
       /isaac-sim/AirStack/scene_gen/tests/test_kit_bake.py --live-full SM_Building_02 F1"

WHY A FAKE MANIFEST DIRECTORY, NOT A REAL SLICE, FOR THE UNIT TESTS
-----------------------------------------------------------------------
A real `slice_to_kit` needs the merged GAC mesh, which lives only on Nucleus
(no local mirror in this repo — see `test_quake_gac_bake.py`'s own docstring
for the identical constraint) and costs tens of seconds even for one
building. None of that is needed to prove the two things that changed
tonight are correct: `have_kit`/`load_kit`'s lookup (`_entry`) now requires
an EXACT `(asset, signature)` match, not `asset` alone, and `fingerprint()`
now covers `gac_fire.py` too, not just the two slicer files. Both are pure
`kits.json` bookkeeping, so every test below monkeypatches `kit_bake.KIT_DIR`
/`MANIFEST_PATH` onto a throwaway temp directory and writes synthetic
manifest rows by hand, exactly the shape `save_kit` would have written — the
"fake kit directory" the task brief asked for in place of a heavy live
slice.

The one thing this file CANNOT check offline is that `load_kit`'s actual
`pxr` reference-and-return path works, and that `save_kit`'s real rehome+
export round-trips a real building — that needs the real merged mesh. Two
container-only checks cover it, neither collected by pytest (their names do
not start with `test_`):

  * `_live_two_run_proof` (`--live NAME`) drives the slice/cache path
    DIRECTLY — `gac_fire.place_source` -> a live `slice_to_kit` -> `kb.
    save_kit` -> `kb.have_kit` -> `kb.load_kit` on a fresh stage, comparing
    piece counts — skipping `gac_fire.prepare()`'s soot-atlas bake and the
    full damage ladder, neither of which the cache touches. THIS IS WHERE A
    REAL BUG WAS CAUGHT (2026-08-31): the first version of `kit_bake.
    _place_source_for_bake` returned the prim CARRYING the scale/translate
    ops rather than a transform-less wrapper around it. Harmless-looking,
    but `gac_slice.window_centres` measures every window RELATIVE TO its
    `src` argument's own local frame (`root_inv = GetLocalToWorldTransform
    (root).GetInverse()`) — hand it the scaled prim itself and that inverse
    divides the scale straight back out, so the cached re-slice measured
    windows at ~100x their real size, missed `grid_for`'s confidence check,
    and fell back to a regular grid: 7 pieces where the live slice (seated
    the same way `gac_fire.place_source` does, correctly) cut 53, for the
    IDENTICAL region. Caught only by comparing REAL piece counts against a
    REAL building through the ACTUAL functions involved — a fake manifest
    directory cannot see it, because it never re-derives geometry, and a
    small synthetic mesh would not reproduce a confidence threshold tuned
    against a real merged asset's noise. `tools/bake_gac_kits.py._place_
    source` has the identical bug (see `_place_source_for_bake`'s docstring
    for the pointer) — out of scope to fix here.
  * `_live_two_run_proof_full` (`--live-full NAME LEVEL`) is the heavier,
    more literal version the task brief describes — the same building
    through `gac_fire.burn_gac` itself, TWICE, `use_baked_kit=True`,
    checking for the cache-hit log line and matching `n_pieces`. Measured
    at over 40 minutes for one `(building, level)` pair (`prepare()`'s
    soot-atlas bake dominates, unrelated to anything this file changed) and
    abandoned as a routine check for that reason — kept for when someone
    wants the fully end-to-end version regardless of cost.

Both grow the TRACKED `scene_gen/assets/kits/kits.json` exactly as real
usage would (that is the feature) — re-run `git status` on it afterward if
that matters for what you're doing next.
"""

import contextlib
import io
import os
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
for _p in (_HERE, _SG):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from detail import kit_bake as kb                              # noqa: E402

try:
    from pxr import Usd, UsdGeom                                # noqa: E402
    HAVE_USD = True
except Exception:                                                # pragma: no cover
    HAVE_USD = False


class _FakeManifest:
    """Point `kit_bake` at a throwaway `kits.json` for the lifetime of a
    `with` block, and restore the real paths afterward — a test that forgot
    to clean up must never touch the TRACKED manifest in
    `scene_gen/assets/kits/kits.json`."""

    def __enter__(self):
        self.tmp = tempfile.mkdtemp(prefix="kit_bake_test_")
        self._old_dir = kb.KIT_DIR
        self._old_manifest = kb.MANIFEST_PATH
        kb.KIT_DIR = self.tmp
        kb.MANIFEST_PATH = os.path.join(self.tmp, "kits.json")
        return self

    def __exit__(self, *exc):
        kb.KIT_DIR = self._old_dir
        kb.MANIFEST_PATH = self._old_manifest
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)
        return False

    def write(self, records):
        kb.write_manifest(records)

    def touch_usd(self, name):
        p = os.path.join(self.tmp, name + ".usd")
        with open(p, "w") as fh:
            fh.write("")
        return p


def _row(asset, signature, usd_path, fingerprint=None):
    return {"asset": asset, "signature": signature, "usd": usd_path,
           "fingerprint": fingerprint or kb.fingerprint(),
           "src_usd": "omniverse://fake", "grid": {"measured": False},
           "pieces": []}


# ---------------------------------------------------------------------------
# slice_signature — the KEYING half: hashes region.origin/top/sides, family,
# force_regular, style (see that function's own docstring for why each field
# is there and why offset/target are deliberately NOT).
# ---------------------------------------------------------------------------
def test_slice_signature_is_deterministic_and_hex():
    r = {"origin": 2, "top": 9, "sides": ("N", "S")}
    a = kb.slice_signature(region=r, family="01", force_regular=False, style="s")
    b = kb.slice_signature(region=dict(r), family="01", force_regular=False, style="s")
    assert a == b
    assert isinstance(a, str) and len(a) == 16
    int(a, 16)  # must parse as hex


def test_slice_signature_ignores_side_order():
    r1 = {"origin": 2, "top": 9, "sides": ("N", "S")}
    r2 = {"origin": 2, "top": 9, "sides": ("S", "N")}
    assert kb.slice_signature(region=r1, family="01") == \
        kb.slice_signature(region=r2, family="01")


def test_slice_signature_changes_with_each_field():
    base = dict(region={"origin": 2, "top": 9, "sides": ("N",)},
               family="01", force_regular=False, style="s")
    sig0 = kb.slice_signature(**base)

    def _mut(**over):
        kw = dict(base)
        kw.update(over)
        return kb.slice_signature(**kw)

    assert _mut(region={"origin": 3, "top": 9, "sides": ("N",)}) != sig0
    assert _mut(region={"origin": 2, "top": 10, "sides": ("N",)}) != sig0
    assert _mut(region={"origin": 2, "top": 9, "sides": ("S",)}) != sig0
    assert _mut(family="02") != sig0
    assert _mut(force_regular=True) != sig0
    assert _mut(style="other") != sig0


def test_slice_signature_of_no_region_is_a_real_hash_not_none():
    # `region=None` (the unconditional whole-building slice) still hashes to
    # a real value — it must never collide with the bare Python `None`
    # `have_kit`/`load_kit` use for "no signature was computed at all".
    sig = kb.slice_signature(region=None)
    assert sig is not None
    assert isinstance(sig, str) and len(sig) == 16


# ---------------------------------------------------------------------------
# have_kit / _entry — KEYING + VINTAGE against a fake manifest
# ---------------------------------------------------------------------------
def test_have_kit_hits_on_matching_signature_and_vintage():
    with _FakeManifest() as fm:
        usd = fm.touch_usd("Bldg")
        fm.write([_row("Bldg", "sigA", usd)])
        assert kb.have_kit("Bldg", "sigA") is True


def test_have_kit_misses_on_wrong_signature():
    with _FakeManifest() as fm:
        usd = fm.touch_usd("Bldg")
        fm.write([_row("Bldg", "sigA", usd)])
        # same building, a DIFFERENT fire plan's signature -> no hit, even
        # though a perfectly good (but wrongly ringed) bake exists on disk —
        # this is the exact scenario the KEYING fix exists for.
        assert kb.have_kit("Bldg", "sigB") is False
        # the legacy "no signature at all" default must not match either
        assert kb.have_kit("Bldg") is False


def test_have_kit_misses_on_stale_vintage():
    with _FakeManifest() as fm:
        usd = fm.touch_usd("Bldg")
        fm.write([_row("Bldg", "sigA", usd, fingerprint="deadbeefdeadbeef")])
        assert kb.have_kit("Bldg", "sigA") is False


def test_have_kit_misses_when_usd_file_is_missing():
    with _FakeManifest() as fm:
        fm.write([_row("Bldg", "sigA", os.path.join(fm.tmp, "nope.usd"))])
        assert kb.have_kit("Bldg", "sigA") is False


def test_have_kit_default_signature_matches_legacy_rows_with_no_signature_key():
    """`tools/bake_gac_kits.py` has never written a `"signature"` field —
    `have_kit(name)` (every existing caller's call shape: `quake_gac_probe.
    py`, that offline tool itself) must still see those rows after this
    change."""
    with _FakeManifest() as fm:
        usd = fm.touch_usd("Bldg")
        row = _row("Bldg", None, usd)
        del row["signature"]           # exactly what the offline baker writes
        fm.write([row])
        assert kb.have_kit("Bldg") is True
        assert kb.have_kit("Bldg", None) is True


def test_two_signatures_of_the_same_asset_coexist():
    """The exact scenario the KEYING fix exists for: two fire plans on one
    building, each cached under its own signature, neither clobbers or is
    served in place of the other."""
    with _FakeManifest() as fm:
        usd_a = fm.touch_usd("Bldg__sigA")
        usd_b = fm.touch_usd("Bldg__sigB")
        kb.merge_manifest([_row("Bldg", "sigA", usd_a)])
        kb.merge_manifest([_row("Bldg", "sigB", usd_b)])
        assert kb.have_kit("Bldg", "sigA") is True
        assert kb.have_kit("Bldg", "sigB") is True
        recs = kb.read_manifest()
        assert len(recs) == 2
        assert kb._entry("Bldg", "sigA")["usd"] == usd_a
        assert kb._entry("Bldg", "sigB")["usd"] == usd_b


def test_merge_manifest_replaces_only_the_matching_asset_and_signature():
    with _FakeManifest() as fm:
        usd_a1 = fm.touch_usd("Bldg__sigA")
        usd_b = fm.touch_usd("Other__sigB")
        kb.merge_manifest([_row("Bldg", "sigA", usd_a1)])
        kb.merge_manifest([_row("Other", "sigB", usd_b)])
        usd_a2 = fm.touch_usd("Bldg__sigA_v2")
        kb.merge_manifest([_row("Bldg", "sigA", usd_a2, fingerprint="newfp")])
        recs = kb.read_manifest()
        assert len(recs) == 2
        got = kb._entry("Bldg", "sigA")
        assert got["usd"] == usd_a2 and got["fingerprint"] == "newfp"
        assert kb._entry("Other", "sigB")["usd"] == usd_b


# ---------------------------------------------------------------------------
# fingerprint() — VINTAGE now covers gac_fire.py too
# ---------------------------------------------------------------------------
def test_fingerprint_hashes_gac_fire_py_too():
    """The whole point of widening the vintage hash: editing
    `disaster/gac_fire.py` (where `window_rects`/`_islands` — the fire-plan
    logic a `region=` cut is built from — actually live) must change
    `fingerprint()`, exactly like editing either slicer file already did.
    """
    assert os.path.exists(kb._GF_PATH)
    assert kb._GF_PATH.endswith(os.path.join("disaster", "gac_fire.py"))
    real_fp = kb.fingerprint()

    old_path = kb._GF_PATH
    tmp = tempfile.NamedTemporaryFile(suffix=".py", delete=False)
    try:
        with open(old_path, "rb") as fh:
            tmp.write(fh.read())
        tmp.write(b"\n# perturbed for test_fingerprint_hashes_gac_fire_py_too\n")
        tmp.close()
        kb._GF_PATH = tmp.name
        assert kb.fingerprint() != real_fp
    finally:
        kb._GF_PATH = old_path
        os.unlink(tmp.name)
    assert kb.fingerprint() == real_fp


def test_kit_filename_naming():
    assert kb._kit_filename("SM_Building_02", None) == "SM_Building_02.usd"
    sig = kb.slice_signature(region={"origin": 1, "top": 5, "sides": ("N",)})
    assert kb._kit_filename("SM_Building_02", sig) == \
        "SM_Building_02__{0}.usd".format(sig)


def test_save_kit_is_best_effort_and_never_raises():
    """A `save_kit` that fails (bad URL, no `pxr`, no VTK, whatever) must
    not take the caller's own bake down with it — `gac_fire.burn_gac` calls
    this AFTER its own live slice already produced good pieces. Runs with
    or without `pxr` installed: on a bare host the `ImportError` inside
    `_save_kit_unsafe` is caught the same as any other failure there."""
    with _FakeManifest() as fm:
        rec = kb.save_kit("NoSuchAsset", "sig",
                          "omniverse://does/not/exist.usd", 1.0, "style",
                          region={"origin": 0, "top": 1, "sides": ("N",)},
                          verbose=False)
        assert rec is None
        assert kb.read_manifest() == []


TESTS = [
    test_slice_signature_is_deterministic_and_hex,
    test_slice_signature_ignores_side_order,
    test_slice_signature_changes_with_each_field,
    test_slice_signature_of_no_region_is_a_real_hash_not_none,
    test_have_kit_hits_on_matching_signature_and_vintage,
    test_have_kit_misses_on_wrong_signature,
    test_have_kit_misses_on_stale_vintage,
    test_have_kit_misses_when_usd_file_is_missing,
    test_have_kit_default_signature_matches_legacy_rows_with_no_signature_key,
    test_two_signatures_of_the_same_asset_coexist,
    test_merge_manifest_replaces_only_the_matching_asset_and_signature,
    test_fingerprint_hashes_gac_fire_py_too,
    test_kit_filename_naming,
    test_save_kit_is_best_effort_and_never_raises,
]


def main(argv):
    if "--live" in argv:
        i = argv.index("--live")
        name = argv[i + 1] if len(argv) > i + 1 else "SM_Building_02"
        return _live_two_run_proof(name)
    if "--live-full" in argv:
        i = argv.index("--live-full")
        name = argv[i + 1] if len(argv) > i + 1 else "SM_Building_02"
        level = argv[i + 2] if len(argv) > i + 2 else "F1"
        return _live_two_run_proof_full(name, level)
    print("test_kit_bake  (pxr {0})".format(
        "present" if HAVE_USD else "ABSENT — save_kit still runs its "
        "best-effort/no-raise path, just never reaches pxr"))
    failed = 0
    for t in TESTS:
        try:
            t()
        except AssertionError as exc:
            failed += 1
            print("  {0:<62} FAIL  {1}".format(t.__name__, exc))
        except Exception as exc:
            failed += 1
            import traceback
            traceback.print_exc()
            print("  {0:<62} ERROR {1}".format(t.__name__, exc))
    print("\n{0}/{1} passed".format(len(TESTS) - failed, len(TESTS)))
    return 1 if failed else 0


def _live_two_run_proof(name, region=None, family="01", force_regular=False):
    """ONE real building, through the EXACT slice/cache path `gac_fire.
    burn_gac`'s cache block runs, without `prepare()`'s soot-atlas bake or
    the damage ladder (neither touches the cache — see the module docstring
    for why `_live_two_run_proof_full`, which does include them, is not the
    default): resolve `url`/`scale`/`style` the same way `gac_fire.prepare`
    does, seat the source with `gac_fire.place_source` (the LIVE seat — see
    `kit_bake._place_source_for_bake`'s docstring for why the SEATING
    function matters, not just the scale), slice it live, `save_kit` it
    under the resulting signature, then `have_kit`/`load_kit` it back on a
    FRESH stage and compare piece counts against the live slice. Needs
    Nucleus + VTK — container only, and never collected by pytest (this
    function's name does not start with `test_`).
    """
    import time

    from pxr import Sdf, Usd, UsdGeom
    from disaster import fracture, gac_fire as gf
    from detail import gac_storey_slice as gss, kit_bake as kb

    fracture.ensure_vtk(verbose=False)

    kind, asset = gf.split_kind(name, None)
    pack = gf.PACKS[kind]
    style = pack["style_prefix"] + asset
    url = gf.asset_url(asset, kind)
    scale = gf.asset_scale(url, pack["scale"], verbose=False)
    region = region or {"origin": 0, "top": 2, "sides": ("N",)}
    sig = kb.slice_signature(region=region, family=family,
                             force_regular=force_regular, style=style)
    print("[test_kit_bake] {0}: url={1} scale={2} style={3} signature={4}"
          .format(name, url, scale, style, sig))
    if kb.have_kit(name, sig):
        print("[test_kit_bake] NOTE: signature {0} was already cached from "
              "a prior run of this proof — this run checks live-vs-cached "
              "agreement, not that the cache starts empty".format(sig))

    def fresh_stage():
        st = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(st, 1.0)
        UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(st, "/W")
        st.SetDefaultPrim(st.GetPrimAtPath("/W"))
        UsdGeom.Xform.Define(st, "/W/cell")
        return st

    t0 = time.time()
    st1 = fresh_stage()
    src1 = gf.place_source(st1, "/W/cell", url, scale)
    pls1, _g1, _m1 = gss.slice_to_kit(st1, src1, "/W/cell", style,
                                      verbose=True, region=region,
                                      family=family,
                                      force_regular=force_regular)
    t_slice = time.time() - t0
    n1 = len(pls1)
    print("[test_kit_bake] LIVE SLICE: {0} piece(s) in {1:.1f}s"
          .format(n1, t_slice))
    if not n1:
        print("[test_kit_bake] LIVE PROOF FAILED: live slice produced "
              "nothing")
        return 1

    t1 = time.time()
    rec = kb.save_kit(name, sig, url, scale, style, region=region,
                      family=family, force_regular=force_regular,
                      verbose=True)
    t_save = time.time() - t1
    if rec is None:
        print("[test_kit_bake] LIVE PROOF FAILED: save_kit returned None")
        return 1
    if not kb.have_kit(name, sig):
        print("[test_kit_bake] LIVE PROOF FAILED: have_kit still False "
              "right after save_kit")
        return 1

    t2 = time.time()
    st2 = fresh_stage()
    pls2, _g2, _m2 = kb.load_kit(st2, "/W/cell", name, 1.0, sig)
    t_load = time.time() - t2
    n2 = len(pls2)

    ok = (n1 == n2)
    print("[test_kit_bake] LIVE PROOF {0}: {1} — live {2} piece(s) in "
          "{3:.1f}s, save {4:.1f}s, cache-load {5} piece(s) in {6:.2f}s"
          .format("OK" if ok else "MISMATCH", name, n1, t_slice, t_save,
                  n2, t_load))
    return 0 if ok else 1


def _live_two_run_proof_full(name, level):
    """ONE real building, through `gac_fire.burn_gac` TWICE with
    `use_baked_kit=True` on a bare USD stage (mirrors `tools/
    gac_burn_probe.py`'s own setup): the first run must slice live and save
    (`kb.save_kit`), the second must print the cache-hit line
    (`gac_fire.py`'s cache block: "loaded from kit cache") and produce the
    identical piece count. The literal shape the task brief asked for, kept
    for when someone wants the FULL ladder exercised too — measured at over
    40 minutes for one `(building, level)` pair (see the module docstring),
    so `_live_two_run_proof` above is the one worth running routinely. Needs
    Nucleus + VTK — container only, and never collected by pytest (this
    function's name does not start with `test_`).
    """
    import random
    import time

    import numpy as np
    from pxr import Usd, UsdGeom
    from disaster import fracture, gac_fire as gf, urban_fire as uf

    fracture.ensure_vtk(verbose=False)

    def _one_run(tag):
        st = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(st, 1.0)
        UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(st, "/W")
        st.SetDefaultPrim(st.GetPrimAtPath("/W"))
        UsdGeom.Scope.Define(st, "/W/bench")
        cell = "/W/bench/g0"
        UsdGeom.Xform.Define(st, cell)
        mats = uf.materials(st, "/W/bench")
        rng = random.Random(7)
        nrng = np.random.default_rng(7)
        buf = io.StringIO()
        t0 = time.time()
        with contextlib.redirect_stdout(buf):
            ctx = gf.burn_gac(st, cell, name, level, rng, nrng, mats, tag,
                              flow_root=None, mat_cache={}, ssf=1.0,
                              use_baked_kit=True, verbose=True)
        out = buf.getvalue()
        print(out)   # still show it live in the harness's own output
        return ctx, out, time.time() - t0

    ctx1, out1, s1 = _one_run("run1")
    n1 = ctx1["gac"]["n_pieces"]
    if "loaded from kit cache" in out1:
        print("[test_kit_bake] NOTE: first run already hit a pre-existing "
              "cache entry (a prior run left one behind) — the proof below "
              "is about run1 vs run2 agreeing, not about run1 being a miss")

    ctx2, out2, s2 = _one_run("run2")
    n2 = ctx2["gac"]["n_pieces"]

    ok = True
    if "loaded from kit cache" not in out2:
        print("[test_kit_bake] LIVE PROOF FAILED: second run did not print "
              "the cache-hit line")
        ok = False
    if n1 != n2:
        print("[test_kit_bake] LIVE PROOF FAILED: piece count changed "
              "{0} -> {1}".format(n1, n2))
        ok = False
    print("[test_kit_bake] LIVE PROOF {0}: {1} {2} — run1 {3} piece(s) in "
          "{4:.1f}s, run2 {5} piece(s) in {6:.1f}s (cache hit expected)"
          .format("OK" if ok else "FAILED", name, level, n1, s1, n2, s2))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
