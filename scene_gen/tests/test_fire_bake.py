#!/usr/bin/env python3
"""test_fire_bake.py — the per-building fire bake, checked without Isaac Sim.

    python3 scene_gen/tests/test_fire_bake.py           # host: schema only
    pytest -q scene_gen/tests/test_fire_bake.py

    # in the container, with pxr — runs the USD half too:
    docker exec isaac-sim bash -c \
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
       /isaac-sim/AirStack/scene_gen/tests/test_fire_bake.py"

    # and re-check bakes that already exist:
    docker exec isaac-sim bash -c \
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
       /isaac-sim/AirStack/scene_gen/tests/test_fire_bake.py \
       --verify /isaac-sim/.cache/fire_bakes"

WHAT THIS TESTS AND WHY EACH PART EXISTS
-----------------------------------------
**A. The sidecar schema (host, no USD).** A bake's `.usd` is geometry; its
`.json` is everything the assembly needs to put the FIRE back — the events,
the wall frames the Flow emitters are placed against, the mass boxes, the
settled top. If a field silently stops round-tripping the assembly renders a
correct-looking burnt building with the flames in the wrong place, or with no
flames at all, and nothing in either launcher says so. So every field
`urban_fire._flame_sources` and `_severity` read is asserted individually,
not just "the JSON parses".

**B. The MATERIAL TRAP (in-container, pxr only — no Kit).** This is the one
that has already cost this repo a whole bake: a sliced GAC piece, and every
`soot_plume.piece_material_like` copy of one, binds a material that lives
INSIDE the merged source's subtree. Export with that subtree dropped and
every piece renders WHITE with its geometry and its UVs perfectly intact
("this turned every sliced building white" — `tools/bake_gac_kits.py`). The
test builds exactly that shape offline — a material in its own
`Materials/*_Inst.usd`, referenced under `<cell>/src`, with a sooted copy
internally referencing it and overriding one texture — runs
`fire_bake.rehome_for_export`, drops the source, exports the ROOT LAYER, and
reopens the file COLD to check the surviving material still resolves to the
SOOT png. That is the whole failure, reproduced and closed, with no Nucleus
and no GPU.

**C. `strip_physics` and `verify_export`.** A settled bake keeps its physics
APIs; the assembly must be static. Applied schemas and `physics:*` attributes
are removed and the reopened file is checked for both.

Everything in A runs anywhere. B and C skip cleanly when `pxr` is absent,
which is what makes this file safe to run on the host.
"""

import json
import math
import os
import struct
import sys
import zlib

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
if _SG not in sys.path:
    sys.path.insert(0, _SG)

from disaster import fire_bake as fb                          # noqa: E402

try:
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade   # noqa: E402
    HAVE_USD = True
except Exception:                                             # pragma: no cover
    HAVE_USD = False


# ---------------------------------------------------------------------------
# A. the schema, host-side
# ---------------------------------------------------------------------------
def _fake_mass(tag="main", cx=0.0, cy=0.0):
    return {"tag": tag, "cx": cx, "cy": cy, "yaw": 0.0, "W": 24.0, "D": 16.0,
            "z0": 0.0, "top": 19.2, "levels": [0.0, 3.2, 6.4, 9.6, 12.8, 16.0],
            "module": 4.0,
            "spec": {"bands": [{"h": 3.2, "module": 4.0},
                               {"h": 1.1, "parapet": True}],
                     "family": "04", "wings": []}}


def _fake_event(eid, m, side="S", storey=2, state="flame", n_ops=3):
    ops = []
    for k in range(n_ops):
        u0 = 2.0 + 4.0 * k
        ops.append({
            "fr": (-12.0, -8.0, 0.0, 24.0, 3.2, -0.02, False),
            "ua": u0, "ub": u0 + 1.6, "va": 7.4, "vb": 9.2,
            "hua": u0 + 0.05, "hub": u0 + 1.55, "hva": 7.5, "hvb": 9.1,
            "out": -0.05, "side": side, "storey": storey, "mass": m["tag"],
            "span": (u0, u0 + 1.6, 7.4, 9.2), "m": m,
            "e": {"mass": m["tag"], "x": u0 - 10.0, "y": -8.0, "z": 6.4,
                  "side": side, "storey": storey, "name": "SM_wall_A",
                  "role": "wall", "dead": (k == 2)}})
    w_t = sum(o["span"][1] - o["span"][0] for o in ops)
    A_v = sum((o["span"][1] - o["span"][0]) * (o["span"][3] - o["span"][2])
              for o in ops)
    return {"id": eid, "mass": m["tag"], "side": side, "storey": storey,
            "ops": ops, "u0": ops[0]["span"][0], "u1": ops[-1]["span"][1],
            "z_sill": 7.4, "z_head": 9.2, "w_t": w_t, "h_eq": A_v / w_t,
            "A_v": A_v, "state": state, "tau": 900.0, "intensity": 0.8,
            "drift": 0.2, "top": False}


def _fake_fire(m):
    return {"origin": 2, "storeys": [2, 3, 4, 5], "top": 5,
            "sides": ("S", "E"), "n_storeys": len(m["levels"]),
            "mass": "main", "roof": True, "level": "F4",
            "state": "smoulder", "finish": "ash"}


def test_manifest():
    ents = fb.parse_manifest(
        "gac:SM_Building_02:F1 gac:SM_Building_09:F6:3 "
        "kit:commercial_mid:F5c::S,E kit:office_wide:F5c:1:S/E:99",
        base_seed=7)
    assert len(ents) == 4, ents
    assert ents[0] == {"kind": "gac", "name": "SM_Building_02", "level": "F1",
                       "origin": None, "sides": None, "seed": 7, "index": 0}
    assert ents[1]["origin"] == 3 and ents[1]["seed"] == 7 + 31
    assert ents[2]["sides"] == ("S", "E") and ents[2]["level"] == "F5c"
    # `/` is accepted for a shell that would rather not quote a comma
    assert ents[3]["sides"] == ("S", "E")
    # an explicit seed overrides the base+31*i default
    assert ents[3]["seed"] == 99 and ents[3]["origin"] == 1
    # THE LEVEL IS NOT UPPER-CASED. `F5c` is a real level in
    # `urban_fire.LEVELS`; upper-casing it makes an unknown one, which the
    # launcher then rejects fourteen seconds into the run.
    assert ents[2]["level"] == "F5c"
    # comments and blanks do not shift the seeds of what follows
    ents2 = fb.parse_manifest("# a comment\ngac:A:F1\n\ngac:B:F2\n", base_seed=0)
    assert [e["seed"] for e in ents2] == [0, 31], ents2
    # the stems are what `fire_bake.sh` predicts and the assembler globs
    assert fb.out_stem(ents[0]) == "gac_SM_Building_02_F1_s7"
    assert fb.bake_tag(ents[2]) == "k2"
    u, j = fb.out_paths(ents[0], "/tmp/x")
    assert u == "/tmp/x/gac_SM_Building_02_F1_s7.usd"
    assert j == "/tmp/x/gac_SM_Building_02_F1_s7.json"
    for bad in ("nope:X:F1", ":X:F1", "gac::F1"):
        try:
            fb.parse_entry(bad)
        except ValueError:
            continue
        raise AssertionError("{0!r} should not parse".format(bad))
    print("  manifest            OK  ({0} entries, seeds {1})".format(
        len(ents), [e["seed"] for e in ents]))


def test_event_round_trip():
    m = _fake_mass()
    events = [_fake_event(0, m, "S", 2, "flame"),
              _fake_event(1, m, "E", 4, "out", n_ops=2),
              _fake_event(2, m, "S", 5, "smoulder", n_ops=1)]
    fire = _fake_fire(m)
    doc = fb.sidecar({"kind": "gac", "name": "SM_Building_02", "level": "F4",
                      "seed": 7, "index": 0},
                     fire, {"main": m}, events,
                     [-12.0, -8.0, 0.0, 12.0, 8.0, 17.9], 17.9,
                     {"interior": [{"x": 1.0, "y": 2.0, "z": 12.0,
                                    "radius": 1.1, "scale": 1.2}],
                      "roof": [{"x": 0.5, "y": -0.5, "z": 17.3,
                                "radius": 1.8, "scale": 1.25}]},
                     ["fire events: 3 event(s)"], {"total_s": 1.0},
                     {"loose": 12}, usd="/tmp/x.usd")
    # it has to survive an actual file, not just a dict copy
    blob = json.loads(json.dumps(doc))
    masses = {k: fb.mass_from_json(v) for k, v in blob["masses"].items()}
    back = fb.events_from_json(blob["events"], masses)

    assert len(back) == len(events)
    for a, b in zip(events, back):
        for k in ("id", "side", "storey", "state", "mass", "top"):
            assert a[k] == b[k], (k, a[k], b[k])
        for k in fb._EV_SCALARS:
            assert abs(float(a[k]) - float(b[k])) < 1e-9, (k, a[k], b[k])
        assert len(a["ops"]) == len(b["ops"])
        for oa, ob in zip(a["ops"], b["ops"]):
            # THE SEVEN-TUPLE FRAME, `dw` INCLUDED. `_b_face_pt` re-centres a
            # `dw` frame on the piece; losing that bool puts every emitter
            # half a module along the wall.
            assert len(ob["fr"]) == 7 and ob["fr"][6] is oa["fr"][6]
            for i in range(6):
                assert abs(ob["fr"][i] - oa["fr"][i]) < 1e-9
            # everything `_flame_sources` actually reads
            for k in ("ua", "ub", "va", "vb", "hua", "hub", "hva", "hvb",
                      "out"):
                assert abs(float(oa[k]) - float(ob[k])) < 1e-9, k
            assert ob["side"] == oa["side"] and ob["storey"] == oa["storey"]
            # ...and what `_severity`/`_el_jitter` read off the element
            for k in ("mass", "x", "y", "z", "dead"):
                assert ob["e"][k] == oa["e"][k], (k, ob["e"][k], oa["e"][k])
            # the mass is stored ONCE and shared back onto every opening
            assert ob["m"] is masses["main"]
    # the mass round trip keeps what `soot_plume.parapet_height` sums
    mm = masses["main"]
    assert abs(mm["W"] - m["W"]) < 1e-9 and abs(mm["top"] - m["top"]) < 1e-9
    assert mm["levels"] == m["levels"]
    assert sum(b["h"] for b in mm["spec"]["bands"] if b.get("parapet")) == 1.1
    from disaster import soot_plume as spl
    assert abs(spl.parapet_height(mm) - 1.1) < 1e-9
    # and the fire plan, which is what `_severity` keys off
    f = blob["fire"]
    assert f["origin"] == 2 and f["storeys"] == [2, 3, 4, 5] and f["roof"]
    assert f["state"] == "smoulder" and f["n_storeys"] == 6
    print("  event round trip    OK  ({0} events, {1} openings)".format(
        len(back), sum(len(e["ops"]) for e in back)))


def test_translate():
    """The column offset moves the wall frames, NOT `e["x"]/["y"]`."""
    m = _fake_mass()
    events = [_fake_event(0, m)]
    blob = json.loads(json.dumps(fb.events_to_json(events)))
    masses = {"main": fb.mass_from_json(fb.mass_to_json(m))}
    back = fb.events_from_json(blob, masses)
    fr0 = tuple(back[0]["ops"][0]["fr"])
    ex0, ey0 = (back[0]["ops"][0]["e"]["x"], back[0]["ops"][0]["e"]["y"])
    fb.translate(masses, back, 250.0, -3.0)
    fr1 = back[0]["ops"][0]["fr"]
    assert abs(fr1[0] - (fr0[0] + 250.0)) < 1e-9
    assert abs(fr1[1] - (fr0[1] - 3.0)) < 1e-9
    assert fr1[2:] == fr0[2:], "translate must not touch yaw/size/dw"
    assert abs(masses["main"]["cx"] - (m["cx"] + 250.0)) < 1e-9
    assert abs(masses["main"]["cy"] - (m["cy"] - 3.0)) < 1e-9
    # `_el_jitter` hashes these; shifting them would re-roll every module's
    # severity wobble and the assembled flames would no longer agree with the
    # soot baked into the building's own textures.
    assert back[0]["ops"][0]["e"]["x"] == ex0
    assert back[0]["ops"][0]["e"]["y"] == ey0
    # AND THE EMITTER ACTUALLY MOVES BY THE COLUMN OFFSET. This is the whole
    # claim the assembly rests on: a Flow emitter is authored under
    # `/World/flow/emitters`, which does NOT inherit the column's transform,
    # so the ONLY thing that puts it on the right building is the translated
    # wall frame.
    from disaster import quake_flow as qf
    op0 = fb.events_from_json(blob, {"main": fb.mass_from_json(
        fb.mass_to_json(m))})[0]["ops"][0]
    p0 = qf._b_face_pt(op0["fr"], 3.0, 8.0, op0["out"])
    p1 = qf._b_face_pt(back[0]["ops"][0]["fr"], 3.0, 8.0,
                       back[0]["ops"][0]["out"])
    assert abs((p1[0] - p0[0]) - 250.0) < 1e-9, (p0, p1)
    assert abs((p1[1] - p0[1]) - (-3.0)) < 1e-9, (p0, p1)
    assert abs(p1[2] - p0[2]) < 1e-12, "a column offset must not move z"
    print("  translate           OK  (frames moved {0:+.0f},{1:+.0f} m, "
          "element coords held)".format(250.0, -3.0))


def test_emitter_geometry_survives():
    """A round-tripped opening puts the emitter in the SAME PLACE.

    Not a schema check: this runs `quake_flow._b_face_pt` and
    `urban_fire._severity` — the two functions `_flame_sources` uses to
    decide where a Flow source goes and how hard it burns — over the original
    records and over the deserialised ones, and compares the world points.
    """
    from disaster import quake_flow as qf
    from disaster import urban_fire as uf

    m = _fake_mass()
    events = [_fake_event(0, m, "S", 3, "flame")]
    fire = _fake_fire(m)
    masses = {"main": fb.mass_from_json(fb.mass_to_json(m))}
    back = fb.events_from_json(
        json.loads(json.dumps(fb.events_to_json(events))), masses)
    ctx_a = {"fire": fire}
    ctx_b = {"fire": dict(fire, sides=tuple(fire["sides"]))}
    for oa, ob in zip(events[0]["ops"], back[0]["ops"]):
        for frac in (0.1, 0.5, 0.9):
            u_a = oa["hua"] + frac * (oa["hub"] - oa["hua"])
            u_b = ob["hua"] + frac * (ob["hub"] - ob["hua"])
            v_a = oa["hva"] + 0.66 * (oa["hvb"] - oa["hva"])
            v_b = ob["hva"] + 0.66 * (ob["hvb"] - ob["hva"])
            pa = qf._b_face_pt(oa["fr"], u_a, v_a, oa["out"] + uf.FLAME_OUT)
            pb = qf._b_face_pt(ob["fr"], u_b, v_b, ob["out"] + uf.FLAME_OUT)
            assert max(abs(x - y) for x, y in zip(pa, pb)) < 1e-9, (pa, pb)
        assert (qf._outward(oa["m"], oa["side"])
                == qf._outward(ob["m"], ob["side"]))
        sa = uf._severity(ctx_a, oa["storey"], oa["e"]["mass"], oa["e"])
        sb = uf._severity(ctx_b, ob["storey"], ob["e"]["mass"], ob["e"])
        assert abs(sa - sb) < 1e-12, (sa, sb)
    print("  emitter geometry    OK  (world points and severities identical)")


def test_module_check():
    problems = fb.check(verbose=False)
    assert not problems, problems
    print("  fire_bake.check     OK")


# ---------------------------------------------------------------------------
# B/C. the USD half — needs `pxr` (the container's `tools/usd_python.sh`)
# ---------------------------------------------------------------------------
def _png(path, rgb=(128, 128, 128), w=4, h=4):
    """A real, valid PNG with no PIL — the bare-USD harness has numpy and
    omni.client but PIL is not guaranteed, and a texture that does not decode
    would make a resolution test pass for the wrong reason."""
    raw = b"".join(b"\x00" + bytes(rgb) * w for _ in range(h))

    def chunk(tag, data):
        body = tag + data
        return (struct.pack(">I", len(data)) + body
                + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF))

    blob = (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw))
            + chunk(b"IEND", b""))
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "wb") as fh:
        fh.write(blob)
    return path


def _quad(stage, path):
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr([(-1, 0, 0), (1, 0, 0), (1, 0, 2), (-1, 0, 2)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateExtentAttr([(-1, 0, 0), (1, 0, 2)])
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.faceVarying).Set([(0, 0), (1, 0), (1, 1), (0, 1)])
    return mesh


def _material_file(path, tex):
    """A material in its OWN `Materials/*_Inst.usd`, which is the shape
    `gac_slice._material_source` recognises — every GreatAmericanCity
    material is exactly this (measured, `tools/gac_mat_probe.py`)."""
    st = Usd.Stage.CreateNew(path)
    mat = UsdShade.Material.Define(st, "/M")
    sh = UsdShade.Shader.Define(st, "/M/PBRShader")
    sh.CreateIdAttr("UsdPreviewSurface")
    ts = UsdShade.Shader.Define(st, "/M/tex")
    ts.CreateIdAttr("UsdUVTexture")
    ts.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(tex))
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        ts.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    st.SetDefaultPrim(mat.GetPrim())
    st.GetRootLayer().Save()
    return path


def test_material_trap(tmp=None):
    """THE MATERIAL TRAP, reproduced and closed, offline.

    Builds the exact shape a sliced+sooted GAC building has — a material in
    its own file, referenced under `<cell>/src`, and a
    `soot_plume.piece_material_like` copy of it that internally references it
    and overrides one texture — then rehomes, drops the source, exports the
    ROOT LAYER and reopens COLD.
    """
    if not HAVE_USD:
        print("  material trap       SKIP (no pxr — run under usd_python.sh)")
        return
    import shutil
    import tempfile
    from disaster import soot_plume as spl

    tmp = tmp or tempfile.mkdtemp(prefix="fire_bake_test_")
    try:
        clean = _png(os.path.join(tmp, "textures", "clean.png"), (180, 170, 160))
        soot = _png(os.path.join(tmp, "textures", "soot.png"), (30, 27, 25))
        matf = _material_file(
            os.path.join(tmp, "Materials", "M_Test_Inst.usd"), clean)

        out = os.path.join(tmp, "bake.usd")
        stage = Usd.Stage.CreateNew(os.path.join(tmp, "_work.usd"))
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        stage.SetDefaultPrim(world.GetPrim())
        UsdGeom.Xform.Define(stage, Sdf.Path(fb.BAKE_ROOT))
        cell = fb.BAKE_ROOT + "/g0"
        UsdGeom.Xform.Define(stage, Sdf.Path(cell))

        # the merged source, with the material living inside it
        src_mat = stage.DefinePrim(Sdf.Path(cell + "/src/M"))
        assert src_mat.GetReferences().AddReference(matf, Sdf.Path("/M"))
        # the pre-slice soot bake's copy: the SOURCE material's whole network,
        # with only the diffuse map swapped
        soot_mat = spl.piece_material_like(
            stage, cell + "/SootLooks/m0", src_mat,
            Sdf.Path(cell + "/src/M/tex"), "file", soot)
        assert soot_mat is not None, "piece_material_like returned None"

        piece = _quad(stage, cell + "/pieces/p0")
        UsdShade.MaterialBindingAPI(piece.GetPrim()).Bind(soot_mat)
        clean_piece = _quad(stage, cell + "/pieces/p1")
        UsdShade.MaterialBindingAPI(clean_piece.GetPrim()).Bind(
            UsdShade.Material(src_mat))
        # and a settled body, so the physics strip has something to do
        UsdPhysics.CollisionAPI.Apply(piece.GetPrim())
        rb = UsdPhysics.RigidBodyAPI.Apply(piece.GetPrim())
        rb.CreateRigidBodyEnabledAttr(False)

        doomed = [cell + "/src"]
        # 1) the trap is DETECTED — the sooted copy's own path says nothing,
        #    its composition arcs say everything
        assert fb.depends_on(soot_mat.GetPrim(), doomed), \
            "depends_on missed the internal reference into the source"
        rel, name, tex = fb.local_texture_override(soot_mat.GetPrim())
        assert name == "file" and tex == soot, (rel, name, tex)

        # 2) rehomed and rebound
        st = fb.rehome_for_export(stage, fb.BAKE_ROOT, doomed,
                                  fb.BAKE_ROOT + "/Looks", verbose=False)
        assert st["failed"] == 0, st
        assert st["needed"] == 2 and st["rehomed"] == 2, st
        assert st["rebound"] == 2, st

        # 3) the source can now go
        stage.RemovePrim(Sdf.Path(cell + "/src"))
        n = fb.strip_physics(stage, root=None, remove_prims=(), verbose=False)
        assert n["schemas"] >= 2, n

        # 4) ROOT LAYER ONLY — never Flatten (see `fire_bake`'s docstring)
        stage.GetRootLayer().Export(out)

        # 5) reopen COLD and resolve every texture
        info = fb.verify_export(out, doomed=tuple(doomed),
                                expect_root=fb.BAKE_ROOT, check_remote=False,
                                verbose=False)
        assert info["ok"], info
        assert info["meshes"] == 2, info
        assert info["n_doomed_prims"] == 0 and info["n_doomed_arcs"] == 0, info
        assert info["n_physics_prims"] == 0, info
        assert info["n_textures_missing"] == 0, info
        # the assembler references the file with NO prim path, so the
        # defaultPrim has to be the root the baker promised
        assert info["default_prim"] == fb.DEFAULT_PRIM, info["default_prim"]
        assert info["has_root"], "the export lost /World/bake"

        cold = Usd.Stage.Open(out)
        got = {}
        for p in (cell + "/pieces/p0", cell + "/pieces/p1"):
            prim = cold.GetPrimAtPath(Sdf.Path(p))
            assert prim and prim.IsValid(), p
            mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            assert mat and mat.GetPrim().IsValid(), \
                "{0} lost its material — this is the WHITE BUILDING".format(p)
            _s, _i, t = spl.find_basecolor(mat.GetPrim())
            got[p] = t
            assert t and os.path.exists(t), (p, t)
        assert got[cell + "/pieces/p0"] == soot, got
        assert got[cell + "/pieces/p1"] == clean, got
        print("  material trap       OK  (sooted piece keeps {0}, clean piece "
              "keeps {1}, source dropped)".format(os.path.basename(soot),
                                                  os.path.basename(clean)))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_export_detects_a_broken_bake():
    """The verifier has to FAIL on the bad file, or it is decoration.

    Same stage as above with the rehome SKIPPED: the source is dropped while
    the sooted material still points into it. Cold, that is a piece with a
    dangling material — exactly the white building — and `verify_export` must
    say so rather than reporting a clean 2-mesh bake.
    """
    if not HAVE_USD:
        print("  broken-bake detect  SKIP (no pxr)")
        return
    import shutil
    import tempfile
    from disaster import soot_plume as spl

    tmp = tempfile.mkdtemp(prefix="fire_bake_bad_")
    try:
        clean = _png(os.path.join(tmp, "textures", "clean.png"))
        soot = _png(os.path.join(tmp, "textures", "soot.png"))
        matf = _material_file(
            os.path.join(tmp, "Materials", "M_Test_Inst.usd"), clean)
        out = os.path.join(tmp, "bad.usd")
        stage = Usd.Stage.CreateNew(os.path.join(tmp, "_work.usd"))
        world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        stage.SetDefaultPrim(world.GetPrim())
        cell = fb.BAKE_ROOT + "/g0"
        UsdGeom.Xform.Define(stage, Sdf.Path(fb.BAKE_ROOT))
        UsdGeom.Xform.Define(stage, Sdf.Path(cell))
        src_mat = stage.DefinePrim(Sdf.Path(cell + "/src/M"))
        src_mat.GetReferences().AddReference(matf, Sdf.Path("/M"))
        soot_mat = spl.piece_material_like(
            stage, cell + "/SootLooks/m0", src_mat,
            Sdf.Path(cell + "/src/M/tex"), "file", soot)
        piece = _quad(stage, cell + "/pieces/p0")
        UsdShade.MaterialBindingAPI(piece.GetPrim()).Bind(soot_mat)
        stage.RemovePrim(Sdf.Path(cell + "/src"))     # the mistake
        stage.GetRootLayer().Export(out)
        info = fb.verify_export(out, doomed=(cell + "/src",),
                                expect_root=fb.BAKE_ROOT, check_remote=False,
                                verbose=False)
        assert not info["ok"], \
            "verify_export passed a bake whose sooted material lost its " \
            "source — it would ship a white building: {0}".format(info)
        print("  broken-bake detect  OK  ({0} dangling arc(s) found)".format(
            info["n_doomed_arcs"]))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
# `--verify <usd or dir>` — re-check bakes that already exist
# ---------------------------------------------------------------------------
def verify_paths(spec):
    import glob as _g

    paths = []
    for item in [q.strip() for q in str(spec).split(",") if q.strip()]:
        if os.path.isdir(item):
            paths += sorted(_g.glob(os.path.join(item, "*.usd")))
        else:
            paths.append(item)
    if not paths:
        print("[test_fire_bake] nothing matched {0!r}".format(spec))
        return 1
    bad = 0
    for p in paths:
        js = os.path.splitext(p)[0] + ".json"
        doomed = ("/src",)
        if os.path.exists(js):
            try:
                doc = fb.read_sidecar(js)
                doomed = (doc.get("cell", fb.BAKE_ROOT) + "/src",)
                print("[sidecar] {0}: {1} event(s), {2} interior + {3} roof "
                      "seat(s), top_z {4}, src_kept={5}".format(
                          os.path.basename(js), len(doc.get("events") or []),
                          len((doc.get("seats") or {}).get("interior") or []),
                          len((doc.get("seats") or {}).get("roof") or []),
                          doc.get("top_z"), doc.get("src_kept")))
            except Exception as exc:
                print("[sidecar] {0} unreadable: {1}".format(js, exc))
                bad += 1
        else:
            print("[sidecar] MISSING for {0} — the assembly will place no "
                  "emitters on it".format(os.path.basename(p)))
            bad += 1
        info = fb.verify_export(p, doomed=doomed, expect_root=fb.BAKE_ROOT,
                                check_remote=True, verbose=True)
        bad += 0 if info.get("ok") else 1
    print("\n{0}/{1} bake(s) clean".format(len(paths) - bad, len(paths)))
    return 0 if bad == 0 else 1


TESTS = [test_manifest, test_event_round_trip, test_translate,
         test_emitter_geometry_survives, test_module_check,
         test_material_trap, test_export_detects_a_broken_bake]


def main(argv):
    if "--verify" in argv:
        i = argv.index("--verify")
        return verify_paths(argv[i + 1] if len(argv) > i + 1
                            else fb.DEFAULT_OUT_DIR)
    print("test_fire_bake  (pxr {0})".format(
        "present" if HAVE_USD else "ABSENT — USD tests skipped"))
    failed = 0
    for t in TESTS:
        try:
            t()
        except AssertionError as exc:
            failed += 1
            print("  {0:<19} FAIL  {1}".format(t.__name__, exc))
        except Exception as exc:
            failed += 1
            import traceback
            traceback.print_exc()
            print("  {0:<19} ERROR {1}".format(t.__name__, exc))
    print("\n{0}/{1} passed".format(len(TESTS) - failed, len(TESTS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
