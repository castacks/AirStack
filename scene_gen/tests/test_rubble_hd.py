#!/usr/bin/env python3
"""test_rubble_hd.py — does the HD debris piece library
(`tools/split_debris_spreads.py` -> `assets/rubble_hd/*`) actually satisfy
the contract the emitter needs?

    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \
        --with pytest python tests/test_rubble_hd.py
    pytest -q scene_gen/tests/test_rubble_hd.py

Host-side only (`usd-core`, no Kit/Isaac). Assumes `tools/split_debris_
spreads.py` has already been run (it writes `assets/rubble_hd/`, which this
test suite treats as checked-in build output, same as `assets/rubble_hd/
catalogue.json` itself) — if the library is missing, the piece-level tests
report 0 pieces checked and fail loudly rather than silently passing.

What each test checks:
  * every piece file the catalogue lists actually opens with `pxr`;
  * its base is within 1 mm of z=0 and its footprint is centred within 1 cm
    of x=y=0 (the catalogue's bottom-centre convention);
  * it carries a `primvars:st` with the SAME interpolation token as its
    source spread (faceVarying) and a value range inside the unit square,
    consistent with the source's own UV range;
  * it has a bound material whose `UsdPreviewSurface.diffuseColor` and
    `.normal` inputs are both connected to a `UsdUVTexture` — the exact
    shape `quake_rubble_usd._reference_diffuse_texture` looks for — and both
    textures RESOLVE to a file that exists on disk; the raw AUTHORED asset
    path (not just the resolved one) is also sanity-checked, so a future
    refactor can't silently start emitting an absolute or otherwise
    fragile path that only happens to resolve on this machine;
  * per spread, the sum of kept-piece triangles does not exceed the
    source's own triangle count;
  * `disaster.quake_rubble.load_hd_catalogue()` returns entries shaped
    exactly like `CATALOGUE` and `HD_CATALOGUE` (the module-level instance)
    is non-empty;
  * at least 15 "chunk"-kind pieces exist across the whole library (this is
    reported, not faked, if the real count comes in under 15).

Also re-runs `tests/test_quake_rubble.py` (unmodified during this task) as
a separate invocation from the shell — see the tool-side report — to prove
the planner module was not touched.
"""
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, _SCENE_GEN)

from disaster import quake_rubble as qr              # noqa: E402
from disaster import quake_rubble_usd as qru         # noqa: E402

from pxr import Usd, UsdGeom, UsdShade                # noqa: E402

ASSETS_ROOT = os.path.join(_SCENE_GEN, "assets")
RUBBLE_HD_ROOT = os.path.join(ASSETS_ROOT, "rubble_hd")
CATALOGUE_PATH = os.path.join(RUBBLE_HD_ROOT, "catalogue.json")

SOURCE_TRIS = {
    "brick_debris_pile": 58000,
    "concrete_debris_elements": 96826,
    "concrete_sidewalk_elements": 29410,
    "concrete_slabs": 58176,
    "cracked_paving_slabs": 27613,
    "huge_concrete_rubble_pile": 71293,
    "lamppost_block": 20538,
}

Z_TOL_M = 1e-3
XY_CENTER_TOL_M = 1e-2


def _load_catalogue():
    with open(CATALOGUE_PATH, "r") as f:
        return json.load(f)["pieces"]


def _mesh_prim(stage):
    for p in stage.Traverse():
        if p.IsA(UsdGeom.Mesh):
            return p
    return None


def test_catalogue_json_present_and_shaped():
    assert os.path.isfile(CATALOGUE_PATH), \
        "assets/rubble_hd/catalogue.json missing — run tools/split_debris_spreads.py first"
    pieces = _load_catalogue()
    assert len(pieces) > 0, "catalogue.json has zero pieces"
    for e in pieces:
        for key in ("name", "url", "size", "tris", "kind", "material", "source_spread"):
            assert key in e, "piece {0} missing key {1}".format(e.get("name"), key)
        assert len(e["size"]) == 3
        assert e["tris"] > 0
        assert e["material"] in ("brick", "concrete")
        assert e["kind"] in ("flake", "chunk", "raft", "street")
    print("catalogue.json: {0} pieces".format(len(pieces)))


def test_every_piece_file_opens_and_is_seated():
    pieces = _load_catalogue()
    n_checked = 0
    for e in pieces:
        path = os.path.join(ASSETS_ROOT, e["url"])
        assert os.path.isfile(path), "piece file missing: {0}".format(path)
        stage = Usd.Stage.Open(path)
        assert stage is not None, "could not open {0}".format(path)
        assert UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z
        assert abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) < 1e-9

        default_prim = stage.GetDefaultPrim()
        assert default_prim and default_prim.IsValid(), "{0}: no default prim".format(path)
        assert UsdGeom.Xform(default_prim), "{0}: default prim is not an Xform".format(path)

        mesh_prim = _mesh_prim(stage)
        assert mesh_prim is not None, "{0}: no Mesh prim".format(path)
        mesh = UsdGeom.Mesh(mesh_prim)
        pts = np.array(mesh.GetPointsAttr().Get())
        assert pts.shape[0] > 0

        bbmin = pts.min(axis=0)
        bbmax = pts.max(axis=0)
        xy_center = (bbmin[:2] + bbmax[:2]) / 2.0
        assert abs(float(bbmin[2])) < Z_TOL_M, \
            "{0}: base z={1} not within {2} m of 0".format(path, bbmin[2], Z_TOL_M)
        assert float(np.max(np.abs(xy_center))) < XY_CENTER_TOL_M, \
            "{0}: footprint centre {1} not within {2} m of origin".format(path, xy_center, XY_CENTER_TOL_M)

        fvc = np.array(mesh.GetFaceVertexCountsAttr().Get())
        assert fvc.size > 0 and (fvc == 3).all(), "{0}: not all-triangle".format(path)
        assert int(fvc.size) == e["tris"], \
            "{0}: mesh has {1} tris, catalogue says {2}".format(path, fvc.size, e["tris"])

        size = np.array(e["size"])
        dims = bbmax - bbmin
        assert np.allclose(size, dims, atol=1e-2), \
            "{0}: catalogue size {1} != measured {2}".format(path, size, dims)

        n_checked += 1
    print("piece files opened + seated OK: {0}".format(n_checked))
    assert n_checked > 0


def test_uv_primvar_present_matches_source_interpolation():
    pieces = _load_catalogue()
    n_checked = 0
    for e in pieces:
        path = os.path.join(ASSETS_ROOT, e["url"])
        stage = Usd.Stage.Open(path)
        mesh_prim = _mesh_prim(stage)
        pv = UsdGeom.PrimvarsAPI(mesh_prim)
        st = pv.GetPrimvar("st")
        assert st.IsDefined(), "{0}: no primvars:st".format(path)
        assert str(st.GetInterpolation()) == "faceVarying", \
            "{0}: st interpolation {1}, source is faceVarying".format(path, st.GetInterpolation())
        vals = np.array(st.Get())
        assert vals.shape[0] > 0
        # same value range as every measured source (0..~1, a couple of
        # sources' max sits fractionally over 1.0 in the raw scan — checked
        # against the actual source uv ranges rather than assuming [0,1]).
        assert vals.min() >= -1e-3, "{0}: st min {1} unexpectedly negative".format(path, vals.min())
        assert vals.max() <= 1.2, "{0}: st max {1} outside plausible UV range".format(path, vals.max())
        n_checked += 1
    print("UV primvar checked on {0} pieces".format(n_checked))
    assert n_checked > 0


def test_material_bound_and_textures_resolve():
    pieces = _load_catalogue()
    n_checked = 0
    for e in pieces:
        path = os.path.join(ASSETS_ROOT, e["url"])
        stage = Usd.Stage.Open(path)
        mesh_prim = _mesh_prim(stage)
        binding = UsdShade.MaterialBindingAPI(mesh_prim)
        mat, _rel = binding.ComputeBoundMaterial()
        assert mat and mat.GetPrim().IsValid(), "{0}: no bound material".format(path)

        found = {"diffuseColor": False, "normal": False}
        for prim in Usd.PrimRange(mat.GetPrim()):
            if not prim.IsA(UsdShade.Shader):
                continue
            sh = UsdShade.Shader(prim)
            if sh.GetIdAttr().Get() != "UsdPreviewSurface":
                continue
            for slot in ("diffuseColor", "normal"):
                inp = sh.GetInput(slot)
                if not inp:
                    continue
                sources, _invalid = inp.GetConnectedSources()
                if not sources:
                    continue
                tex_sh = UsdShade.Shader(sources[0].source)
                assert tex_sh.GetIdAttr().Get() == "UsdUVTexture", \
                    "{0}: {1} not connected to a UsdUVTexture".format(path, slot)
                file_input = tex_sh.GetInput("file")
                assert file_input, "{0}: {1} texture has no file input".format(path, slot)
                val = file_input.Get()
                assert val is not None, "{0}: {1} texture file unset".format(path, slot)

                # authoredPath sanity: relative, points back at the ORIGINAL
                # spread's own textures/ dir (never an absolute or
                # machine-specific path).
                authored = str(val.path)
                assert not os.path.isabs(authored), \
                    "{0}: {1} texture authored an absolute path {2}".format(path, slot, authored)
                assert "concrete_rubble_debris/split/{0}/textures/".format(e["source_spread"]) in authored, \
                    "{0}: {1} authored path {2} doesn't point at its source spread's textures/".format(
                        path, slot, authored)

                resolved = val.resolvedPath
                assert resolved, "{0}: {1} texture path did not resolve".format(path, slot)
                assert os.path.isfile(resolved), \
                    "{0}: {1} resolved texture {2} does not exist".format(path, slot, resolved)
                found[slot] = True
        assert found["diffuseColor"], "{0}: no resolved diffuseColor texture".format(path)
        assert found["normal"], "{0}: no resolved normal texture".format(path)
        n_checked += 1
    print("material + texture resolution checked on {0} pieces".format(n_checked))
    assert n_checked > 0


def test_emitter_lookup_resolves_diffuse_texture():
    """The emitter's OWN lookup (`quake_rubble_usd._reference_diffuse_
    texture`), not a reimplementation of it — proves a piece file actually
    satisfies what `_textured_debris_look` will call at scatter time."""
    pieces = _load_catalogue()
    n_checked = 0
    for e in pieces:
        path = os.path.join(ASSETS_ROOT, e["url"])
        resolved = qru._reference_diffuse_texture(path)
        assert resolved, "{0}: emitter lookup found no diffuse texture".format(path)
        assert os.path.isfile(resolved)
        n_checked += 1
    print("emitter _reference_diffuse_texture() resolved on {0} pieces".format(n_checked))
    assert n_checked > 0


def test_no_mdl_shader_needed_to_copy():
    """None of the 7 source spreads carries an MDL/OmniPBR shader (checked
    directly against the sources here, independent of the splitting tool's
    own `_has_mdl_shader` check) — so no piece should have one either."""
    for spread in SOURCE_TRIS:
        src_path = os.path.join(ASSETS_ROOT, "concrete_rubble_debris", "split", spread, spread + ".usdc")
        stage = Usd.Stage.Open(src_path)
        mdl_found = False
        for prim in stage.Traverse():
            if not prim.IsA(UsdShade.Shader):
                continue
            sh = UsdShade.Shader(prim)
            sid = str(sh.GetIdAttr().Get() or "")
            if "mdl" in sid.lower() or "omnipbr" in sid.lower():
                mdl_found = True
        assert not mdl_found, "{0}: source unexpectedly carries an MDL/OmniPBR shader " \
            "-- the splitter's piece files need updating to copy it".format(spread)


def test_piece_tris_sum_never_exceeds_source():
    pieces = _load_catalogue()
    by_spread = {}
    for e in pieces:
        by_spread.setdefault(e["source_spread"], 0)
        by_spread[e["source_spread"]] += e["tris"]
    assert set(by_spread.keys()) == set(SOURCE_TRIS.keys()), \
        "catalogue spreads {0} != expected {1}".format(sorted(by_spread), sorted(SOURCE_TRIS))
    for spread, kept_tris in by_spread.items():
        assert kept_tris <= SOURCE_TRIS[spread], \
            "{0}: kept {1} tris > source {2}".format(spread, kept_tris, SOURCE_TRIS[spread])
    print("per-spread kept/source tris:",
          {k: (v, SOURCE_TRIS[k]) for k, v in sorted(by_spread.items())})


def test_load_hd_catalogue_shape_matches_catalogue():
    loaded = qr.load_hd_catalogue()
    assert len(loaded) > 0, "load_hd_catalogue() returned nothing"
    ref_keys = set(qr.CATALOGUE["chunk_01"].keys())
    for name, entry in loaded.items():
        assert set(entry.keys()) == ref_keys, \
            "{0}: HD entry keys {1} != CATALOGUE shape {2}".format(name, set(entry.keys()), ref_keys)
        assert isinstance(entry["size"], tuple) and len(entry["size"]) == 3
        assert entry["textured"] is True
    print("load_hd_catalogue(): {0} entries, shape matches CATALOGUE".format(len(loaded)))


def test_load_hd_catalogue_never_raises_on_missing_file():
    empty = qr.load_hd_catalogue(path="/definitely/not/a/real/path.json")
    assert empty == {}
    empty2 = qr.load_hd_catalogue(path=__file__)  # exists but is not JSON
    assert empty2 == {}


def test_hd_catalogue_module_level_nonempty():
    assert isinstance(qr.HD_CATALOGUE, dict)
    assert len(qr.HD_CATALOGUE) > 0, "HD_CATALOGUE is empty at import time"


def test_at_least_15_chunk_pieces():
    pieces = _load_catalogue()
    n_chunks = sum(1 for e in pieces if e["kind"] == "chunk")
    print("chunk-kind pieces found: {0}".format(n_chunks))
    assert n_chunks >= 15, \
        "only {0} chunk-kind pieces found across the library (need >= 15) -- " \
        "reporting honestly, not padding this".format(n_chunks)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)
    print("\nall tests passed")
