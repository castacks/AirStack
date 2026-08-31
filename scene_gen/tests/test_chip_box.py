#!/usr/bin/env python3
"""test_chip_box.py — `fracture.chip_box` and the three places round 5 wires
it in, so the authored cuboids stop reading as gift boxes.

    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \
        --with vtk --with pytest python -m pytest tests/test_chip_box.py -q

Host-side (`usd-core` + `vtk`, no Kit/Isaac). The user's round-5 note is the
spec: "There's a lot of perfect rectangular debris. While they should look
rectangular, they shouldn't look perfect ... use VTK to cause chips ... random
from small to very large chips. Is it possible to make it look warped? bent?"
— and then "can we not have them be clean breaks? ... the actual breaking
should not be clean."

WHAT IS ACTUALLY PINNED HERE

  the helper       a chipped solid is still a SOLID (no open edge, no
                   degenerate face, every normal finite), loses 2-30 % of its
                   volume, and does so with a WIDE spread (both barely-nicked
                   and big-chunk-gone pieces exist in one population);
                   deterministic per seed; bends only where asked.
  the refusals     a non-solid, a 2-face scrap, a mesh with no rng — all pass
                   through untouched. This is the safety rule that keeps the
                   helper away from a clipped shell, which is the
                   `vtkStripper::GetPointCells` SIGSEGV in the round-4
                   catalogue.
  the escape       `QC_CHIP=0` reproduces the pre-round-5 box BYTE FOR BYTE
                   through `quake_rubble_usd._box`, points, counts, indices
                   and normals alike.
  the wiring       `_author_large` chips lintels/quoins/joists/column stubs
                   and keeps the bottom-centre origin contract; the beam
                   material falls back gracefully when the megascans pack is
                   not on disk; `quake_collapse._chip_prim` refuses anything
                   that is not one of our own small UV-less boxes.
"""
import os
import random
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import fracture as fr                 # noqa: E402
from disaster import quake_rubble_usd as qru        # noqa: E402

from pxr import Gf, Sdf, Usd, UsdGeom, Vt           # noqa: E402


# sizes that stand in for the real populations: `LINTEL_LONG_SIZE`,
# `LINTEL_QUOIN_SIZE`, `COLUMN_SIZE_XY/Z`, `JOIST_SIZE` (quake_rubble.py) and
# a dropped floor plate (`quake_flow.SLAB_T` on a 22 x 18 m mass).
KINDS = {
    "lintel": (1.80, 0.28, 0.26),
    "quoin":  (0.52, 0.38, 0.34),
    "column": (0.45, 0.45, 2.00),
    "joist":  (3.20, 0.10, 0.20),
    "slab":   (22.0, 18.0, 0.30),
}


def _spec(kind):
    kw = dict(qru._CHIP_KIND.get(kind, qru._CHIP_DEFAULT))
    kw.pop("seed", None)
    warp = kw.pop("warp_frac", 0.0)
    return kw, warp


def _chip(kind, seed):
    kw, warp = _spec(kind)
    sz = KINDS[kind]
    return fr.chip_box(sizes=sz, rng=random.Random(seed), bottom=True,
                       warp_m=warp * max(sz), **kw)


def _degenerate(v, f, tol=1e-12):
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    return int((np.linalg.norm(np.cross(b - a, c - a), axis=1) < tol).sum())


# ---------------------------------------------------------------------------
# the helper itself
# ---------------------------------------------------------------------------
def test_vtk_is_available():
    """Everything below is vacuous without it — fail loudly, not silently."""
    assert fr._vtk() is not None, "run with --with vtk"


def test_box_arrays_is_a_closed_outward_wound_solid():
    for seg in ((1, 1, 1), (4, 3, 1), (2, 2, 2)):
        v, f = fr.box_arrays(3.0, 2.0, 0.5, bottom=True, seg=seg)
        assert fr.open_edge_count(f) == 0, seg
        assert _degenerate(v, f) == 0, seg
        assert abs(fr.mesh_volume(v, f) - 3.0) < 1e-9, seg
        assert abs(float(v[:, 2].min())) < 1e-12          # bottom-centred
        # outward winding: the signed volume must be POSITIVE, or every
        # authored normal points into the piece.
        a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
        signed = float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum()) / 6.0
        assert signed > 0.0, (seg, signed)


def test_chipped_pieces_are_still_solids():
    """No open edge, no degenerate face, every face normal finite.

    An open piece is a hole in the render AND a bad convex hull for physics,
    which is why `chip_box` rejects a cut whose cap came back short rather
    than shipping it."""
    for kind in KINDS:
        for seed in range(12):
            v, f = _chip(kind, 100 + seed)
            assert len(f) >= 4, (kind, seed)
            assert fr.open_edge_count(f) == 0, (kind, seed)
            assert _degenerate(v, f) == 0, (kind, seed)
            assert np.isfinite(v).all(), (kind, seed)
            n = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
            ln = np.linalg.norm(n, axis=1)
            assert np.isfinite(ln).all() and (ln > 0).all(), (kind, seed)


def test_volume_loss_stays_inside_the_two_to_thirty_percent_band():
    """`min_loss`/`max_loss` are a CONTRACT, not a tendency.

    Under the floor the piece is still a gift box; over the ceiling the chip
    has become a bisection and the piece stops reading as the lintel/joist the
    planner sized."""
    for kind, sz in KINDS.items():
        v0, f0 = fr.box_arrays(*sz, bottom=True)
        vol0 = fr.mesh_volume(v0, f0)
        for seed in range(16):
            v, f = _chip(kind, 200 + seed)
            loss = 1.0 - fr.mesh_volume(v, f) / vol0
            # EXACT, not approximate: a final uniform scale pulls the piece
            # back into the band after the roughening pass has moved its
            # volume (see the note at the end of `chip_box`).
            assert 0.045 <= loss <= 0.275, (kind, seed, loss)


def test_chip_sizes_span_small_to_very_large():
    """The user asked for "random from small to very large chips".

    A UNIFORM depth draw gives every piece a similar bite and the population
    reads processed; `_chip_depth` is log-uniform with a big-chip tail, so one
    population has to contain both a barely-nicked piece and one that lost a
    real chunk."""
    for kind in ("column", "quoin", "joist", "slab"):
        v0, f0 = fr.box_arrays(*KINDS[kind], bottom=True)
        vol0 = fr.mesh_volume(v0, f0)
        losses = np.asarray([
            1.0 - fr.mesh_volume(*_chip(kind, 900 + seed)) / vol0
            for seed in range(40)])
        assert float(losses.min()) < 0.11, (kind, losses.round(3))
        assert float(losses.max()) > 0.22, (kind, losses.round(3))
        assert float(losses.max() - losses.min()) > 0.15, (kind,
                                                           losses.round(3))
        assert float(losses.std()) > 0.04, (kind, float(losses.std()))


def test_deterministic_per_seed_and_different_across_seeds():
    for kind in KINDS:
        a = _chip(kind, 4242)
        b = _chip(kind, 4242)
        assert np.array_equal(a[0], b[0]) and np.array_equal(a[1], b[1]), kind
        c = _chip(kind, 4243)
        assert not (a[0].shape == c[0].shape
                    and np.allclose(a[0], c[0])), kind


def test_refuses_non_solid_and_missing_rng_input():
    """PASS THROUGH, never cut. An open shell is exactly what segfaults
    `vtkStripper` (round-4 catalogue) and what `quake_sliced`'s standing rule
    forbids handing to this file at all."""
    v, f = fr.box_arrays(1.0, 1.0, 1.0)
    holed = f[:-2]                                    # two faces removed
    ov, of = fr.chip_box(v, holed, random.Random(1))
    assert np.array_equal(of, holed) and np.array_equal(ov, v)

    scrap = np.asarray([[0, 1, 2], [0, 2, 3]], dtype=np.int64)
    sv = np.asarray([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], dtype=float)
    ov, of = fr.chip_box(sv, scrap, random.Random(1))
    assert np.array_equal(of, scrap)

    ov, of = fr.chip_box(v, f, None)                  # no rng -> no chips
    assert np.array_equal(of, f) and np.array_equal(ov, v)


def test_qc_chip_0_returns_the_input_untouched(monkeypatch):
    monkeypatch.setenv("QC_CHIP", "0")
    assert not fr.chips_enabled()
    v, f = fr.chip_box(sizes=(1.0, 2.0, 0.3), rng=random.Random(3), bottom=True)
    assert len(v) == 8 and len(f) == 12
    plain_v, plain_f = fr.box_arrays(1.0, 2.0, 0.3, bottom=True)
    assert np.array_equal(v, plain_v) and np.array_equal(f, plain_f)


def _end_face_share(v, f, la, sign):
    """(largest coplanar end-facing face as a share of all end-facing area,
    how many distinct planes carry that area) at one end of a bar.

    A sawn end is (1.0, 1) — ONE plane, all of the area. A snapped end is
    several overlapping steps, so the share drops and the count rises. This is
    the measurable form of "the actual breaking should not be clean"."""
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    n = np.cross(b - a, c - a)
    area = np.linalg.norm(n, axis=1) / 2.0
    unit = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    sel = unit[:, la] * sign > 0.5
    if not sel.any():
        return 1.0, 0
    key = np.round(np.column_stack(
        [unit[sel], (a[sel] * unit[sel]).sum(1)]), 3)
    _u, inv = np.unique(key, axis=0, return_inverse=True)
    tot = np.zeros(len(_u))
    np.add.at(tot, np.asarray(inv).ravel(), area[sel])
    return float(tot.max() / area[sel].sum()), int((tot > 0.02 * area[sel].sum()).sum())


def test_a_plain_box_end_is_one_plane_and_a_chipped_bar_end_is_not():
    """"can we not have them be clean breaks? ... the actual breaking should
    not be clean."

    The plain box each of these replaces has exactly ONE face at each end
    carrying 100 % of the end-facing area — a saw cut. After the end steps,
    the biggest single plane must carry well under all of it, at both ends,
    on average."""
    sz = KINDS["joist"]
    v0, f0 = fr.box_arrays(*sz, bottom=True)
    assert _end_face_share(v0, f0, 0, +1.0) == (1.0, 1)
    assert _end_face_share(v0, f0, 0, -1.0) == (1.0, 1)

    kw, warp = _spec("joist")
    share, planes = [], []
    for seed in range(24):
        v, f = fr.chip_box(sizes=sz, rng=random.Random(500 + seed),
                           bottom=True, warp_m=warp * max(sz), **kw)
        for sign in (+1.0, -1.0):
            s_, p_ = _end_face_share(v, f, 0, sign)
            share.append(s_)
            planes.append(p_)
    # measured 0.29 / 5.9 planes / 0 flat ends over these seeds; the bounds
    # leave room for a tuning nudge but not for a regression to a saw cut.
    assert np.mean(share) < 0.55, np.mean(share)
    assert np.mean(planes) > 3.0, np.mean(planes)
    assert np.mean(np.asarray(share) > 0.95) < 0.15, np.mean(
        np.asarray(share) > 0.95)


def test_end_steps_are_extra_cuts_and_only_on_bars():
    """`ends` buys EXTRA cuts on a bar and nothing at all on a cube — a quoin
    is a spalled stone, not a snapped member, and forcing break steps onto one
    would just shave its corners twice."""
    cube = (0.5, 0.5, 0.5)
    a = fr.chip_box(sizes=cube, rng=random.Random(9), bottom=True,
                    chips=(3, 3), ends=0.0, rough_frac=0.0)
    b = fr.chip_box(sizes=cube, rng=random.Random(9), bottom=True,
                    chips=(3, 3), ends=1.0, rough_frac=0.0)
    assert np.array_equal(a[0], b[0]) and np.array_equal(a[1], b[1])

    bar = (3.2, 0.16, 0.22)
    c = fr.chip_box(sizes=bar, rng=random.Random(9), bottom=True,
                    chips=(3, 3), ends=0.0, rough_frac=0.0)
    d = fr.chip_box(sizes=bar, rng=random.Random(9), bottom=True,
                    chips=(3, 3), ends=1.0, rough_frac=0.0)
    assert len(d[1]) != len(c[1]) or not np.array_equal(d[0], c[0])


def test_warp_bends_the_middle_and_keeps_the_length():
    """A bow, not a fold: the mid-span moves, the two ends do not, and the
    piece is still as long as it was."""
    v, f = fr.box_arrays(3.2, 0.10, 0.20, bottom=True, seg=(6, 1, 1))
    w = fr.warp_mesh(v, random.Random(11), warp_m=0.06, twist_deg=6.0, axis=0)
    d = np.linalg.norm(w - v, axis=1)
    u = (v[:, 0] - v[:, 0].min()) / (v[:, 0].max() - v[:, 0].min())
    mid = d[(u > 0.35) & (u < 0.65)]
    end = d[(u < 0.05) | (u > 0.95)]
    assert mid.mean() > 3.0 * max(end.mean(), 1e-6), (mid.mean(), end.mean())
    assert abs((w[:, 0].max() - w[:, 0].min()) - 3.2) < 1e-9
    assert d.max() <= 0.06 * 1.3 + 1e-9, d.max()

    flat = fr.warp_mesh(v, random.Random(11), warp_m=0.0, twist_deg=0.0)
    assert np.array_equal(flat, v), "warp_m=0 must be a no-op"


def test_concrete_kinds_are_not_bent_and_timber_is():
    """Concrete does not bow. Only `joist` carries `warp_frac` in the table."""
    for kind in ("lintel", "quoin", "column", "slab"):
        assert "warp_frac" not in qru._CHIP_KIND.get(kind, qru._CHIP_DEFAULT), kind
    assert qru._CHIP_KIND["joist"].get("warp_frac", 0.0) > 0.0


def test_cost_per_piece_stays_a_handful_of_plane_cuts():
    """MEASURED IN PLANE CUTS, not milliseconds.

    On an idle machine a piece costs 4.4-8.7 ms (see the v9 README), which is
    the "<10 ms" the brief asked for — but this suite also runs beside a
    Blender CPU render, where the same code measures 44 ms, and a wall-clock
    assertion would just be a flaky one. So the budget is expressed against a
    single capped VTK plane clip timed in the SAME process: a piece is 2-6
    corner chips plus 2-4 end steps plus one subdivision, so ~10x one clip is
    the expected cost and anything past 25x is a real regression (a lost
    early-out, a subdivision that stopped respecting its budget) rather than a
    busy machine."""
    v, f = fr.box_arrays(1.8, 0.28, 0.26, bottom=True, seg=(4, 1, 1))
    fr._chip_clip(v, f, np.asarray([0.0, 0.0, -1.0]),
                  np.asarray([0.0, 0.0, 0.22]))            # warm VTK
    t0 = time.perf_counter()
    for _ in range(20):
        fr._chip_clip(v, f, np.asarray([0.0, 0.0, -1.0]),
                      np.asarray([0.0, 0.0, 0.22]))
    unit = (time.perf_counter() - t0) / 20.0
    assert unit > 0.0
    for kind in KINDS:
        _chip(kind, 1)
        t0 = time.perf_counter()
        for seed in range(8):
            _chip(kind, 300 + seed)
        per = (time.perf_counter() - t0) / 8.0
        assert per < 25.0 * unit, (kind, per / unit, 1000.0 * per)


# ---------------------------------------------------------------------------
# wiring 1 — quake_rubble_usd._box / _author_large
# ---------------------------------------------------------------------------
def _stage():
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    st.DefinePrim("/World", "Xform")
    st.DefinePrim("/World/B", "Xform")
    return st


def _mesh_arrays(st, path):
    m = UsdGeom.Mesh(st.GetPrimAtPath(path))
    return (list(m.GetPointsAttr().Get()),
            list(m.GetFaceVertexCountsAttr().Get()),
            list(m.GetFaceVertexIndicesAttr().Get()),
            list(m.GetNormalsAttr().Get() or ()))


def test_box_with_a_chip_spec_is_irregular_but_keeps_the_contract():
    st = _stage()
    spec = dict(qru._CHIP_KIND["lintel"])
    spec["seed"] = 12345
    qru._box(st, "/World/B/l", 1.8, 0.28, 0.26, chip=spec)
    pts, counts, idx, nrm = _mesh_arrays(st, "/World/B/l")
    assert len(pts) > 8 and set(counts) == {3}
    assert len(nrm) == 3 * len(counts)
    zs = [p[2] for p in pts]
    # the bottom-centre origin is what the planner places by (`_author_large`)
    assert abs(min(zs)) < 1e-6, min(zs)
    # ... and the piece stays within `max_grow` of the size it was asked for.
    # It is NOT clamped to it exactly: the roughening field pushes the bbox out
    # without costing volume, and paying for that in a fit-to-size cost a
    # 14 %-roughened lintel 48 % of its volume (see `chip_box`'s note).
    assert max(zs) <= 0.26 * 1.25 + 1e-4, max(zs)
    assert max(abs(p[0]) for p in pts) <= 0.9 * 1.25 + 1e-4
    ext = UsdGeom.Mesh(st.GetPrimAtPath("/World/B/l")).GetExtentAttr().Get()
    assert abs(ext[0][2]) < 1e-6 and ext[1][2] <= 0.26 * 1.25 + 1e-4
    st2 = UsdGeom.PrimvarsAPI(st.GetPrimAtPath("/World/B/l"))
    assert st2.HasPrimvar("st"), "the Blender preview needs a planar st"


def test_qc_chip_0_authors_the_old_box_byte_for_byte(monkeypatch):
    """The escape hatch has to be an EXACT revert, not an approximation."""
    st = _stage()
    qru._box(st, "/World/B/plain", 1.8, 0.28, 0.26)
    ref = _mesh_arrays(st, "/World/B/plain")

    monkeypatch.setenv("QC_CHIP", "0")
    entry = {"asset": None, "prim_path": None, "kind": "lintel",
             "pos": (1.0, 2.0, 3.0), "rot_deg": (0.0, 0.0, 12.0),
             "scale": 1.0, "size": (1.8, 0.28, 0.26), "bury": 0.2,
             "look": "stone"}
    assert qru._chip_spec("lintel", 7) is None
    qru._author_large(st, "/World/B", "t", entry, {}, {}, None, 0)
    got = _mesh_arrays(st, "/World/B/rubble_t_large_00")
    assert got == ref


def test_author_large_chips_every_authored_bar_kind():
    st = _stage()
    sizes = {"lintel": (1.8, 0.28, 0.26), "quoin": (0.5, 0.36, 0.32),
             "column": (0.45, 0.45, 2.0), "joist": (3.2, 0.1, 0.2)}
    for i, (kind, size) in enumerate(sorted(sizes.items())):
        entry = {"asset": None, "prim_path": None, "kind": kind,
                 "pos": (float(i) * 4.0, 0.0, 0.0), "rot_deg": (0, 0, 0),
                 "scale": 1.0, "size": size, "bury": 0.1, "look": None}
        out = qru._author_large(st, "/World/B", "t", entry, {}, {}, None, i)
        assert out
        pts, counts, _idx, _n = _mesh_arrays(st, out)
        assert len(pts) > 8, (kind, len(pts))
        assert set(counts) == {3}, kind
        assert abs(min(p[2] for p in pts)) < 1e-6, kind


def test_author_large_is_deterministic_and_position_keyed():
    """`_chip_seed` hashes the piece's own identity — no shared rng draw, so
    a piece's shape does not depend on how many were authored before it."""
    def one(stage, n, pos):
        entry = {"asset": None, "prim_path": None, "kind": "lintel",
                 "pos": pos, "rot_deg": (0, 0, 0), "scale": 1.0,
                 "size": (1.8, 0.28, 0.26), "bury": 0.1, "look": None}
        return qru._author_large(stage, "/World/B", "t", entry, {}, {}, None, n)

    a = _stage()
    b = _stage()
    pa = one(a, 0, (0.0, 0.0, 0.0))
    pb = one(b, 0, (0.0, 0.0, 0.0))
    assert _mesh_arrays(a, pa) == _mesh_arrays(b, pb)
    pc = one(a, 1, (5.0, 0.0, 0.0))
    assert _mesh_arrays(a, pa)[0] != _mesh_arrays(a, pc)[0]


def test_beam_look_falls_back_when_the_megascans_pack_is_absent(monkeypatch,
                                                                tmp_path):
    """The Damaged_Concrete_Floor import is another agent's in-flight work.
    Absent, the beam pieces keep their old material and STILL get chipped."""
    monkeypatch.setattr(qru, "_beam_tex_path", lambda: None)
    monkeypatch.setattr(qru, "_BEAM_MISSING_WARNED", [False])
    st = _stage()
    assert qru._beam_look(st, "/World/B", {}) is None
    entry = {"asset": None, "prim_path": None, "kind": "lintel",
             "pos": (0, 0, 0), "rot_deg": (0, 0, 0), "scale": 1.0,
             "size": (1.8, 0.28, 0.26), "bury": 0.1, "look": "stone"}
    out = qru._author_large(st, "/World/B", "t", entry, {}, {}, None, 0)
    pts, counts, _i, _n = _mesh_arrays(st, out)
    assert len(pts) > 8 and set(counts) == {3}
    from pxr import UsdShade
    bound = UsdShade.MaterialBindingAPI(
        st.GetPrimAtPath(out)).ComputeBoundMaterial()[0]
    assert str(bound.GetPath()).endswith("rubble_stone")


def test_beam_look_binds_the_pack_when_it_is_there(monkeypatch, tmp_path):
    fake = tmp_path / "T_fake_2K_B.png"
    fake.write_bytes(b"\x89PNG\r\n\x1a\n")
    monkeypatch.setattr(qru, "_beam_tex_path", lambda: str(fake))
    st = _stage()
    entry = {"asset": None, "prim_path": None, "kind": "column",
             "pos": (0, 0, 0), "rot_deg": (0, 0, 0), "scale": 1.0,
             "size": (0.45, 0.45, 2.0), "bury": 0.1, "look": "concrete"}
    out = qru._author_large(st, "/World/B", "t", entry, {}, {}, None, 0)
    from pxr import UsdShade
    bound = UsdShade.MaterialBindingAPI(
        st.GetPrimAtPath(out)).ComputeBoundMaterial()[0]
    assert str(bound.GetPath()) == "/World/B/QuakeLooks/rubble_beam"
    sh = UsdShade.Shader.Get(st, Sdf.Path(str(bound.GetPath()) + "/Shader"))
    assert sh.GetInput("diffuse_texture").Get() is not None


def test_timber_joist_and_rebar_keep_their_own_look(monkeypatch, tmp_path):
    fake = tmp_path / "T_fake_2K_B.png"
    fake.write_bytes(b"\x89PNG\r\n\x1a\n")
    monkeypatch.setattr(qru, "_beam_tex_path", lambda: str(fake))
    st = _stage()
    entry = {"asset": None, "prim_path": None, "kind": "joist",
             "pos": (0, 0, 0), "rot_deg": (0, 0, 0), "scale": 1.0,
             "size": (3.2, 0.1, 0.2), "bury": 0.1, "look": "timber"}
    out = qru._author_large(st, "/World/B", "t", entry, {}, {}, None, 0)
    from pxr import UsdShade
    bound = UsdShade.MaterialBindingAPI(
        st.GetPrimAtPath(out)).ComputeBoundMaterial()[0]
    assert str(bound.GetPath()).endswith("rubble_timber")


# ---------------------------------------------------------------------------
# wiring 2/3 — quake_collapse._chip_prim / _chip_pieces
# ---------------------------------------------------------------------------
def _qc():
    from disaster import quake_collapse as qc
    return qc


def _plain_box(st, path, sx, sy, sz, with_st=False, quads=True):
    """`quake_flow._box`'s shape: centred points, quads, no UVs."""
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    P = Gf.Vec3f
    pts = [P(-hx, -hy, -hz), P(hx, -hy, -hz), P(hx, hy, -hz), P(-hx, hy, -hz),
           P(-hx, -hy, hz), P(hx, -hy, hz), P(hx, hy, hz), P(-hx, hy, hz)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    m = UsdGeom.Mesh.Define(st, Sdf.Path(path))
    m.CreatePointsAttr(Vt.Vec3fArray(pts))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 6))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
    if with_st:
        pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray,
            UsdGeom.Tokens.faceVarying)
        pv.Set(Vt.Vec2fArray([Gf.Vec2f(0.0, 0.0)] * 24))
    return m


def test_chip_prim_tessellates_and_chips_a_dropped_floor_plate():
    qc = _qc()
    st = _stage()
    _plain_box(st, "/World/B/slab", 22.0, 18.0, 0.30)
    assert qc._chip_prim(st, "/World/B/slab", qc._CHIP_SLAB, tessellate=True)
    m = UsdGeom.Mesh(st.GetPrimAtPath("/World/B/slab"))
    pts = np.asarray([[p[0], p[1], p[2]] for p in m.GetPointsAttr().Get()])
    counts = list(m.GetFaceVertexCountsAttr().Get())
    assert set(counts) == {3} and len(counts) > 12
    # still the same plate, still centred on its own local origin
    ext = pts.max(0) - pts.min(0)
    assert 18.0 < ext[0] <= 22.0 * 1.26 and 14.0 < ext[1] <= 18.0 * 1.26
    c = 0.5 * (pts.max(0) + pts.min(0))
    assert np.abs(c).max() < 1e-4, c
    # ... and no longer a rectangle: the top face is no longer one flat plane
    # spanning the full plan
    assert float(pts[:, 2].std()) > 0.0


def test_chip_prim_refuses_uv_carrying_and_oversized_meshes():
    """The guards are the safety rule, not decoration: a mesh with real UVs is
    kit/sliced art (chipping would drop the cladding) and a big mesh is not one
    of our boxes at all."""
    qc = _qc()
    st = _stage()
    _plain_box(st, "/World/B/kit", 3.0, 0.3, 3.0, with_st=True)
    assert not qc._chip_prim(st, "/World/B/kit", qc._CHIP_PRISM)

    v, f = fr.box_arrays(3.0, 3.0, 3.0, seg=(6, 6, 6))
    assert len(f) > qc.CHIP_MAX_FACES
    m = UsdGeom.Mesh.Define(st, Sdf.Path("/World/B/big"))
    m.CreatePointsAttr(Vt.Vec3fArray(
        [Gf.Vec3f(*[float(q) for q in p]) for p in v]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(f)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for i in f.ravel()]))
    assert not qc._chip_prim(st, "/World/B/big", qc._CHIP_PRISM)

    assert not qc._chip_prim(st, "/World/B/nope", qc._CHIP_PRISM)
    assert qc._chip_ok([5], [0, 1, 2, 3, 4], 8) is None      # a pentagon
    assert qc._chip_ok([3], [0, 1, 99], 8) is None           # out of range


def test_chip_prim_chips_a_break_box_cell_in_place():
    qc = _qc()
    st = _stage()
    v, f = fr.box_arrays(1.1, 0.42, 0.09, seg=(2, 1, 1))
    m = UsdGeom.Mesh.Define(st, Sdf.Path("/World/B/frag"))
    m.CreatePointsAttr(Vt.Vec3fArray(
        [Gf.Vec3f(*[float(q) for q in p]) for p in v]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(f)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for i in f.ravel()]))
    before = fr.mesh_volume(v, f)
    assert qc._chip_prim(st, "/World/B/frag", qc._CHIP_PRISM)
    mm = UsdGeom.Mesh(st.GetPrimAtPath("/World/B/frag"))
    nv = np.asarray([[p[0], p[1], p[2]] for p in mm.GetPointsAttr().Get()])
    idx = np.asarray(list(mm.GetFaceVertexIndicesAttr().Get()),
                     dtype=np.int64).reshape(-1, 3)
    assert fr.open_edge_count(idx) == 0
    after = fr.mesh_volume(nv, idx)
    assert 0.045 <= 1.0 - after / before <= 0.275, after / before


def test_chip_pieces_is_a_no_op_under_qc_chip_0_and_rubble_v1(monkeypatch):
    qc = _qc()
    from disaster import quake_flow as qf
    st = _stage()
    _plain_box(st, "/World/B/s", 6.0, 4.0, 0.3)
    ctx = {"stage": st, "parent": "/World/B", "mats": {}}
    ref = _mesh_arrays(st, "/World/B/s")

    monkeypatch.setenv("QC_CHIP", "0")
    assert qc._chip_pieces(ctx, ["/World/B/s"], qc._CHIP_SLAB,
                           tessellate=True) == 0
    assert _mesh_arrays(st, "/World/B/s") == ref

    monkeypatch.delenv("QC_CHIP")
    monkeypatch.setattr(qf, "_RUBBLE_MODE", "v1")
    assert qc._chip_pieces(ctx, ["/World/B/s"], qc._CHIP_SLAB,
                           tessellate=True) == 0
    assert _mesh_arrays(st, "/World/B/s") == ref

    monkeypatch.setattr(qf, "_RUBBLE_MODE", "v2")
    assert qc._chip_pieces(ctx, ["/World/B/s"], qc._CHIP_SLAB,
                           tessellate=True) == 1
    assert _mesh_arrays(st, "/World/B/s") != ref


def test_chip_pieces_binds_the_beam_scan_and_honours_beam_keep(monkeypatch,
                                                               tmp_path):
    """Round-5 addendum: broken concrete gets the Damaged_Concrete_Floor scan.

    ... but NOT at the cost of `_break_box`'s own variety: the 45 % of a broken
    deck's cells it deliberately puts on the mortar-dust tint are named in
    `beam_keep` and stay where they are."""
    qc = _qc()
    from pxr import UsdShade
    fake = tmp_path / "T_fake_2K_B.png"
    fake.write_bytes(b"\x89PNG\r\n\x1a\n")
    monkeypatch.setattr(qru, "_beam_tex_path", lambda: str(fake))
    st = _stage()
    keep = qru._box_material_for(st, "/World/B", "brick_dusty", None)
    UsdShade.Material.Define(st, Sdf.Path("/World/B/QuakeLooks/a_brick_dusty"))
    for name in ("a", "b"):
        _plain_box(st, "/World/B/" + name, 1.2, 0.5, 0.12)
    qru._bind(st, "/World/B/b",
              UsdShade.Material.Get(st,
                                    Sdf.Path("/World/B/QuakeLooks/a_brick_dusty")))
    ctx = {"stage": st, "parent": "/World/B", "mats": {}}
    assert qc._chip_pieces(ctx, ["/World/B/a", "/World/B/b"], qc._CHIP_PRISM,
                           beam=True, beam_keep=("a_brick_dusty",)) == 2
    got = {}
    for name in ("a", "b"):
        m = UsdShade.MaterialBindingAPI(
            st.GetPrimAtPath("/World/B/" + name)).ComputeBoundMaterial()[0]
        got[name] = m.GetPath().name
    assert got["a"] == "rubble_beam", got
    assert got["b"] == "a_brick_dusty", got
    assert keep is not None


def test_chip_pieces_leaves_materials_alone_without_beam():
    qc = _qc()
    from pxr import UsdShade
    st = _stage()
    _plain_box(st, "/World/B/p", 1.2, 0.5, 0.12)
    ctx = {"stage": st, "parent": "/World/B", "mats": {}}
    assert qc._chip_pieces(ctx, ["/World/B/p"], qc._CHIP_PRISM) == 1
    m = UsdShade.MaterialBindingAPI(
        st.GetPrimAtPath("/World/B/p")).ComputeBoundMaterial()[0]
    assert not m or not m.GetPrim().IsValid() or m.GetPath().name != "rubble_beam"
    assert UsdGeom.PrimvarsAPI(
        st.GetPrimAtPath("/World/B/p")).HasPrimvar("st")


def test_chip_pieces_never_raises_on_a_bad_path():
    qc = _qc()
    st = _stage()
    assert qc._chip_pieces({"stage": st, "parent": "/World/B"},
                           [None, "", "/nope"], qc._CHIP_PRISM) == 0


# ---------------------------------------------------------------------------
# round-5 follow-up (2026-08-31) proof lines — "no positive log evidence
# that chips fire during a real bake". `quake_rubble_usd.author()` now
# prints one `[chip]` line per large-element population per building; this
# checks the functions that produce them are wired up and callable, and
# that the line only claims a chip/dress pass ran when it actually did.
# ---------------------------------------------------------------------------
def test_author_prints_a_chip_proof_line_for_authored_boxes(capsys):
    st = _stage()
    parent = "/World/B"
    plan = {"mound": None, "apron": None, "instances": {},
            "large": [{"asset": None, "prim_path": None, "kind": "column",
                      "pos": (0.0, 0.0, 0.0), "rot_deg": (0.0, 0.0, 0.0),
                      "scale": 1.0, "size": (0.4, 0.4, 2.5), "bury": 0.0,
                      "look": "concrete"}]}
    old = os.environ.get("QC_CHIP")
    try:
        os.environ["QC_CHIP"] = "1"
        out = qru.author(st, parent, plan, tag="proofbox")
        assert len(out["large"]) == 1
        printed = capsys.readouterr().out
        assert "[chip] rubble large boxes (proofbox): 1 chipped, 0 " \
               "passed-through (vtk=True)" in printed, printed

        os.environ["QC_CHIP"] = "0"
        st2 = _stage()
        qru.author(st2, parent, plan, tag="proofbox2")
        printed2 = capsys.readouterr().out
        assert "[chip] rubble large boxes (proofbox2): 0 chipped, 1 " \
               "passed-through (vtk=False)" in printed2, printed2
    finally:
        if old is None:
            os.environ.pop("QC_CHIP", None)
        else:
            os.environ["QC_CHIP"] = old


def test_author_prints_a_dress_proof_line_for_laid_panels(capsys):
    st = _stage()
    parent = "/World/B"
    panel_path = parent + "/Panel"
    _plain_box(st, panel_path, 3.0, 0.3, 3.0)
    plan = {"mound": None, "apron": None, "instances": {},
            "large": [{"asset": None, "prim_path": panel_path, "kind": "panel",
                      "pos": (1.0, 1.0, 0.2), "rot_deg": (0.0, 0.0, 10.0),
                      "scale": 1.0, "size": (3.0, 0.3, 3.0), "bury": 0.05}]}
    old = os.environ.get("LAY_PANEL_DRESS")
    try:
        os.environ["LAY_PANEL_DRESS"] = "1"
        out = qru.author(st, parent, plan, tag="proofpanel")
        assert out["large"] == [panel_path]
        printed = capsys.readouterr().out
        assert "[chip] laid panels (proofpanel): 1 dressed " \
               "(sink+tilt+edge-chunks)" in printed, printed
        # the edge-chunk count named in the line matches what actually
        # landed in `out["static"]`
        n_extra = len(out["static"]) - len(out["large"])
        assert "{0} edge chunk(s)".format(n_extra) in printed, (n_extra, printed)
        assert n_extra > 0

        os.environ["LAY_PANEL_DRESS"] = "0"
        st2 = _stage()
        _plain_box(st2, panel_path, 3.0, 0.3, 3.0)
        qru.author(st2, parent, plan, tag="proofpanel2")
        printed2 = capsys.readouterr().out
        assert "[chip] laid panels (proofpanel2): 0 dressed " \
               "(sink+tilt+edge-chunks), 1 passed-through " \
               "(LAY_PANEL_DRESS=False), 0 edge chunk(s)" in printed2, printed2
    finally:
        if old is None:
            os.environ.pop("LAY_PANEL_DRESS", None)
        else:
            os.environ["LAY_PANEL_DRESS"] = old


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
