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


def test_volume_loss_stays_inside_the_band_the_spec_asked_for():
    """`min_loss`/`max_loss` are a CONTRACT, not a tendency.

    Under the floor the piece is still a gift box; over the ceiling the chip
    has become a bisection and the piece stops reading as the lintel/joist the
    planner sized. Round 6 raised the ceiling from 0.27 to 0.36 because a
    piece that loses a third of its section HAS lost about a third of its
    volume — the old ceiling made the final uniform scale inflate a properly
    bitten piece to hide the loss. The band is read FROM the spec, so this
    tracks the table instead of restating it."""
    for kind, sz in KINDS.items():
        spec = qru._CHIP_KIND.get(kind, qru._CHIP_DEFAULT)
        # SLACK FOR THE WARP. `chip_box` applies `warp_mesh` AFTER the
        # exact-volume clamp on purpose (bowing a board costs volume the
        # clamp would otherwise charge it for twice — see the note at the end
        # of `chip_box`), so a bent kind can land a couple of points outside.
        slack = 0.05 if spec.get("warp_frac") else 0.005
        lo = float(spec.get("min_loss", 0.05)) - slack
        hi = float(spec.get("max_loss", 0.27)) + slack
        v0, f0 = fr.box_arrays(*sz, bottom=True)
        vol0 = fr.mesh_volume(v0, f0)
        for seed in range(16):
            v, f = _chip(kind, 200 + seed)
            loss = 1.0 - fr.mesh_volume(v, f) / vol0
            # EXACT, not approximate: a final uniform scale pulls the piece
            # back into the band after the roughening pass has moved its
            # volume (see the note at the end of `chip_box`).
            assert lo <= loss <= hi, (kind, seed, loss)


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
        assert float(losses.min()) < 0.16, (kind, losses.round(3))
        assert float(losses.max()) > 0.28, (kind, losses.round(3))
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
        assert not qru._CHIP_KIND.get(kind, qru._CHIP_DEFAULT).get(
            "warp_frac"), kind
    assert qru._CHIP_KIND["joist"].get("warp_frac", 0.0) > 0.0


def test_cost_per_piece_stays_a_handful_of_plane_cuts():
    """MEASURED IN PLANE CUTS, not milliseconds.

    On an idle machine a piece costs 4.4-8.7 ms (see the v9 README), which is
    the "<10 ms" the brief asked for — but this suite also runs beside a
    Blender CPU render, where the same code measures 44 ms, and a wall-clock
    assertion would just be a flaky one. So the budget is expressed against a
    single capped VTK plane clip timed in the SAME process: a piece is 2-6
    corner chips plus 2-4 end steps plus one subdivision.

    ROUND 6 RAISED THE CEILING FROM 25x TO 70x, and that is the honest price
    of the fix: a piece now also pays for 1-3 large section-sized bites, an
    anisotropic refinement to `gouge_budget` (760-900 triangles against round
    5's 66-256) and 2-6 gouge passes over every vertex. Measured across the
    five kinds it lands at 12-30x. Past 70x is a real regression — a lost
    early-out, or a refinement that stopped respecting its budget."""
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
        assert per < 70.0 * unit, (kind, per / unit, 1000.0 * per)


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
    # `max_grow` (0.35) plus the few per cent the final exact-volume clamp
    # may add on top of it — both documented in `chip_box`.
    assert max(zs) <= 0.26 * 1.42 + 1e-4, max(zs)
    assert max(abs(p[0]) for p in pts) <= 0.9 * 1.42 + 1e-4
    ext = UsdGeom.Mesh(st.GetPrimAtPath("/World/B/l")).GetExtentAttr().Get()
    assert abs(ext[0][2]) < 1e-6 and ext[1][2] <= 0.26 * 1.42 + 1e-4
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
    assert 18.0 < ext[0] <= 22.0 * 1.42 and 14.0 < ext[1] <= 18.0 * 1.42
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
    band = (float(qc._CHIP_PRISM["min_loss"]) - 0.005,
            float(qc._CHIP_PRISM["max_loss"]) + 0.005)
    assert band[0] <= 1.0 - after / before <= band[1], after / before


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


# ---------------------------------------------------------------------------
# ROUND 6 — the shaft, the scallop, and the census
# ---------------------------------------------------------------------------
# WHAT ROUND 5 GOT WRONG AND WHY NO TEST CAUGHT IT. Every assertion above this
# line measures VOLUME. The user's complaint is about SHAPE, and the two came
# apart completely: on a 0.40 x 0.40 x 3.50 m column the round-5 table lost
# 7-21 % of the volume and left the middle of the piece a perfect prism,
# because a plane clip can only remove material at the extreme corner along
# its own normal and every corner of that column is at one of its two ends.
#
# So the round-6 acceptance test is not a volume band. It is the CROSS-SECTION
# FILL PROFILE — how much of the nominal section survives at each of 40
# stations along the piece — measured by sampling points against the closed
# surface (`vtkSelectEnclosedPoints`). The measurement lives in
# `tools/pillar_break_bench.py` so the test and the render bench cannot drift:
# the number asserted here is literally the number printed under the PNGs.

sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..", "tools")))
import pillar_break_bench as bench                   # noqa: E402


def _profile(kind, seed):
    v, f = _chip(kind, seed)
    return bench.section_profile(v, f, KINDS[kind])[0]


def test_the_middle_of_a_column_is_damaged_and_not_only_its_two_ends():
    """THE round-6 test, and the one that would have caught round 5.

    Round 5 on this exact column: 22/42/61 % at the bottom, 72-92 % at the
    top, and 85-99 % at EVERY station in between — a bevelled prism. The bar
    is set where the eye is: a shaft that never drops below ~0.90 of its
    section reads as a cuboid at 7 m, whatever it lost at its ends."""
    worst = []
    for seed in range(14):
        p = _profile("column", 700 + seed)
        k = max(1, int(round(len(p) * 0.10)))          # drop the end stations
        worst.append(float(p[k:-k].min()))
    worst = np.asarray(worst)
    # EVERY piece has a real bite somewhere along its shaft. The ceiling is
    # 0.92 and not lower on purpose: "random from small to very large chips"
    # means the population must CONTAIN a lightly-nicked piece, and the
    # measured spread over 14 seeds is 0.19-0.91 with a median near 0.56.
    # What round 5 did — every piece between 0.85 and 0.99 — is the failure.
    assert worst.max() <= 0.92, worst.round(2)
    # ... the typical piece loses a fifth of its section mid-shaft ...
    assert float(np.median(worst)) <= 0.80, worst.round(2)
    # ... and the tail loses a third or more.
    assert worst.min() <= 0.66, worst.round(2)
    assert float(worst.max() - worst.min()) > 0.35, worst.round(2)


def test_a_plate_loses_its_edge_runs_not_only_its_four_corners():
    """The stop-sign case. Round 5 cut a slab's four corners off and left
    every edge between them a ruler-straight line — which is the flat grey
    plate lying on the rubble mound in `eq500_v3/b0_apartment_DG5_obl.png`.

    Measured along the plate's LONG axis: a piece whose section only dips at
    its two ends has had its corners cut and nothing else."""
    mids = []
    for seed in range(14):
        p = _profile("lintel", 800 + seed)
        k = max(1, int(round(len(p) * 0.15)))
        mids.append(float(p[k:-k].min()))
    mids = np.asarray(mids)
    assert mids.max() <= 0.93, mids.round(2)
    assert float(np.median(mids)) <= 0.88, mids.round(2)


def test_no_piece_is_left_with_all_six_faces_pristine():
    """"They shouldn't look perfect" made countable. A face still carrying
    >90 % of its nominal area, flat and in-plane, is a face that reads as
    cast — and six of them is a cuboid."""
    for kind in ("column", "lintel", "quoin", "joist", "slab"):
        for seed in range(10):
            v, f = _chip(kind, 400 + seed)
            _faces, pristine = bench.face_report(v, f, KINDS[kind])
            assert pristine <= 2, (kind, seed, pristine, _faces)


def test_gouges_and_bites_off_reproduce_the_round_five_shape_exactly():
    """The new mechanisms are OPT-IN. A caller that does not set them — every
    non-quake user of `fracture` — gets the old arrays back byte for byte."""
    sz = (0.45, 0.45, 2.0)
    old = dict(chips=(2, 6), depth_frac=(0.025, 0.15), ends=0.55,
               rough_frac=0.10)
    a = fr.chip_box(sizes=sz, rng=random.Random(5), bottom=True, **old)
    b = fr.chip_box(sizes=sz, rng=random.Random(5), bottom=True,
                    bites=(0, 0), gouges=(0, 0), rough_lam_frac=None, **old)
    assert np.array_equal(a[0], b[0]) and np.array_equal(a[1], b[1])


def test_subdivide_long_edges_refines_only_the_long_ones_and_stays_closed():
    """A T-junction or an open edge here would put the fan caps back into
    every later cut — the round-3 defect the whole solidify pass exists for."""
    def max_edge(v, f):
        e = np.vstack([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
        return float(np.linalg.norm(v[e[:, 0]] - v[e[:, 1]], axis=1).max())

    for sz in ((0.4, 0.4, 3.5), (2.5, 1.6, 0.12)):
        v, f = fr.box_arrays(*sz, bottom=True)
        area = fr._tri_area(v, f)
        tgt = float(np.clip(np.sqrt(2.0 * area / 760.0), 0.02, 1.0))
        nv, nf = fr.subdivide_long_edges(v, f, tgt, budget=760)
        # WATERTIGHT. A T-junction or an open edge here would put the fan caps
        # back into every later cut.
        assert fr.open_edge_count(nf) == 0, sz
        assert _degenerate(nv, nf) == 0, sz
        assert abs(fr.mesh_volume(nv, nf) - fr.mesh_volume(v, f)) < 1e-9, sz
        # IT USES THE BUDGET. `subdivide_to_budget`'s uniform 4-to-1 split can
        # only land on 12/48/192/768..., so at 760 it stops at 192 triangles
        # and a 0.88 m longest edge — and a 0.3 m scallop has nothing to
        # displace at that spacing. This is the whole reason the gouge pass
        # needed a second refiner.
        uv, uf = fr.subdivide_to_budget(v, f, budget=760)
        assert len(nf) <= 760, (sz, len(nf))
        assert len(nf) > 2 * len(uf), (sz, len(nf), len(uf))
        assert max_edge(nv, nf) < 0.75 * max_edge(uv, uf), sz
    # and it never overruns a budget, however fine the target
    v, f = fr.box_arrays(0.4, 0.4, 3.5, bottom=True)
    for budget in (60, 200, 760):
        _v, _f = fr.subdivide_long_edges(v, f, 0.02, budget=budget)
        assert len(_f) <= budget, (budget, len(_f))
        assert fr.open_edge_count(_f) == 0, budget


def test_a_gouge_never_opens_inverts_or_escapes_the_piece():
    """Points only, so topology cannot change — but the displacement must not
    push a vertex through the far face either, which would invert the solid
    and give physics a hull with negative volume."""
    v, f = fr.box_arrays(0.4, 0.4, 3.5, bottom=True)
    v, f = fr.subdivide_long_edges(v, f, 0.13, budget=900)
    v0 = fr.mesh_volume(v, f)
    ext0 = v.max(0) - v.min(0)
    for seed in range(20):
        g = fr.gouge_arrays(v, random.Random(seed), 4, piece_vol=v0)
        assert g.shape == v.shape and np.isfinite(g).all(), seed
        assert fr.open_edge_count(f) == 0, seed
        vol = fr.mesh_volume(g, f)
        assert 0.30 * v0 < vol < v0 + 1e-9, (seed, vol / v0)
        # never grows: a gouge only ever pushes material IN
        assert ((g.max(0) - g.min(0)) <= ext0 + 1e-9).all(), seed


def test_gouge_depth_spans_small_to_very_large():
    """`gouge_big_p` is the tail the user asked for. Over a population, the
    deepest bite has to be several times the shallowest."""
    v, f = fr.box_arrays(0.4, 0.4, 3.5, bottom=True)
    v, f = fr.subdivide_long_edges(v, f, 0.13, budget=900)
    v0 = fr.mesh_volume(v, f)
    loss = np.asarray([
        1.0 - fr.mesh_volume(fr.gouge_arrays(v, random.Random(s), 3,
                                             piece_vol=v0), f) / v0
        for s in range(40)])
    # `gouge_vol_frac` caps the TOTAL a gouge pass may take, so the ceiling
    # here is a budget rather than a free draw; the point of the assertion is
    # the SPREAD underneath it.
    assert loss.min() < 0.04, loss.round(3)
    assert loss.max() > 0.10, loss.round(3)
    assert loss.max() > 4.0 * max(loss.min(), 1e-3), loss.round(3)


def test_every_spec_table_entry_asks_for_the_round_six_mechanisms():
    """A kind that forgets `gouges` is a kind that ships perfect cuboids —
    which is exactly how round 5 shipped with the wiring 'done'."""
    qc = _qc()
    tables = [("qru." + k, s) for k, s in qru._CHIP_KIND.items()]
    tables.append(("qru._CHIP_DEFAULT", qru._CHIP_DEFAULT))
    tables += [("qc." + n, getattr(qc, n))
               for n in ("_CHIP_PLANK", "_CHIP_PRISM", "_CHIP_SLAB")]
    for name, spec in tables:
        assert int(spec.get("gouges", (0, 0))[1]) >= 2, name
        assert int(spec.get("bites", (0, 0))[1]) >= 1, name
        assert spec.get("rough_lam_frac"), name
        assert float(spec.get("max_loss", 0.27)) >= 0.30, name


def test_spec_for_shape_routes_a_plate_a_bar_and_a_board():
    """The helper a `quake_flow` emitter needs so its one-line wiring does not
    have to know the tables. See its docstring for the census of every
    population that is STILL unchipped."""
    qc = _qc()
    assert qc.spec_for_shape((2.5, 1.6, 0.12)) is qc._CHIP_SLAB
    assert qc.spec_for_shape((1.8, 0.28, 0.26)) is qc._CHIP_PRISM
    assert qc.spec_for_shape((0.45, 0.45, 2.0)) is qc._CHIP_PRISM
    assert qc.spec_for_shape((3.2, 0.10, 0.20), timber=True) is qc._CHIP_PLANK


def test_a_quake_flow_shaped_plain_box_is_accepted_by_the_chip_round_trip():
    """PROOF THAT THE MISSING WIRING IS ONLY WIRING. Every unchipped
    population listed in `spec_for_shape`'s docstring is a `quake_flow._box`:
    a CENTRED 6-quad box with no `st`. `_chip_prim` must accept exactly that
    shape, or adding the call would silently no-op."""
    qc = _qc()
    st = _stage()
    from pxr import Gf, Sdf, UsdGeom, Vt
    hx, hy, hz = 0.9, 0.14, 0.13                      # a `_p_lintels` bar
    P = Gf.Vec3f
    pts = [P(-hx, -hy, -hz), P(hx, -hy, -hz), P(hx, hy, -hz), P(-hx, hy, -hz),
           P(-hx, -hy, hz), P(hx, -hy, hz), P(hx, hy, hz), P(-hx, hy, hz)]
    m = UsdGeom.Mesh.Define(st, Sdf.Path("/World/B/lintel_00"))
    m.CreatePointsAttr(Vt.Vec3fArray(pts))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 6))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(
        [i for q in [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
                     (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)] for i in q]))
    spec = qc.spec_for_shape((2 * hx, 2 * hy, 2 * hz))
    assert qc._chip_prim(st, "/World/B/lintel_00", spec, tessellate=True)
    out = UsdGeom.Mesh(st.GetPrimAtPath("/World/B/lintel_00"))
    v = np.asarray([[p[0], p[1], p[2]] for p in out.GetPointsAttr().Get()])
    cnt = np.asarray(out.GetFaceVertexCountsAttr().Get())
    assert (cnt == 3).all() and len(cnt) > 12
    _faces, pristine = bench.face_report(
        v, np.asarray(out.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3),
        (2 * hx, 2 * hy, 2 * hz))
    assert pristine <= 2, (pristine, _faces)


# ---------------------------------------------------------------------------
# ROUND 6b — wiring 4/4: the `quake_flow` cuboid populations
# ---------------------------------------------------------------------------
# `fracture.chip_box` existed for a whole round while `grep -c chip
# quake_flow.py` was 1 — and that hit was a comment. The lintel bars, the
# spall bands, the interior litter, the lifted paving slabs and the shattered
# column chunks all shipped as perfect rectangles because nothing called the
# helper on them. These tests pin the round trip for EVERY shape now wired, so
# a future emitter cannot quietly go back to shipping a cuboid.
#
# SHAPES ARE READ FROM THE EMITTERS' OWN DRAW RANGES, not invented here:
# `_p_lintels` (three branches), `_disturb_interior`'s litter, `_d_chunk`,
# `_d_face_band`, `_b_crumbs`, `_buckled_pavement`, `_c_kerb`, and
# `fit_interior`'s column at `COLUMN_W`.

QF_SHAPES = {
    # name              (sx, sy, sz)          expect a gouge pass?
    "lintel_run":       ((1.80, 0.26, 0.24), True),
    "quoin":            ((0.48, 0.35, 0.31), True),
    "coping":           ((0.90, 0.38, 0.28), True),
    "litter":           ((0.45, 0.30, 0.20), True),
    "chunk":            ((0.60, 0.45, 0.30), True),
    "spall_band":       ((0.90, 0.11, 0.65), True),
    "crumb":            ((0.18, 0.12, 0.10), False),   # under CHIP_TINY_M
    "pavement_slab":    ((2.10, 1.60, 0.14), True),
    "kerb":             ((1.20, 0.32, 0.18), True),
    "fit_column":       ((0.45, 0.45, 2.80), True),
}


def test_spec_for_shape_size_ladder_keeps_the_small_populations_affordable():
    """THE COST CONTROL, and without it this wiring is unaffordable.

    `_disturb_interior` authors `W*D/100*9` litter boxes PER STOREY and
    `_b_crumbs` 5-12 per loss patch; at the full 760-triangle gouge budget one
    building would pay several hundred thousand triangles for pieces a few
    pixels across. The ladder has to actually bite."""
    qc = _qc()
    big = qc.spec_for_shape((1.80, 0.26, 0.24))
    small = qc.spec_for_shape((0.60, 0.45, 0.30))
    tiny = qc.spec_for_shape((0.18, 0.12, 0.10))
    assert big["gouges"][1] >= 2 and big["gouge_budget"] >= 700
    assert 0 < small["gouges"][1] < big["gouges"][1]
    assert small["gouge_budget"] < big["gouge_budget"]
    # a tiny piece gets SILHOUETTE ONLY — a bite, no gouge, no roughening
    assert tiny["gouges"] == (0, 0) and tiny["bites"][1] >= 1
    assert not tiny["rough_frac"]
    # ... and the tiers still route by shape, not only by size
    assert qc.spec_for_shape((2.10, 1.60, 0.14))["ends"] == \
        qc._CHIP_SLAB["ends"]
    assert qc.spec_for_shape((3.2, 0.10, 0.20), timber=True)["warp_frac"] > 0


def test_every_wired_quake_flow_shape_round_trips_through_chip_prim():
    """One round trip per emitter shape. The acceptance criterion is the same
    one the pillar bench uses: the piece must come back a closed solid that is
    no longer a cuboid."""
    qc = _qc()
    st = _stage()
    for name, (sz, gouged) in QF_SHAPES.items():
        path = "/World/B/qf_" + name
        _plain_box(st, path, *sz)
        spec = qc.spec_for_shape(sz)
        assert qc._chip_prim(st, path, spec, tessellate=True), name
        m = UsdGeom.Mesh(st.GetPrimAtPath(path))
        v = np.asarray([[p[0], p[1], p[2]]
                        for p in m.GetPointsAttr().Get()], dtype=float)
        cnt = np.asarray(m.GetFaceVertexCountsAttr().Get())
        f = np.asarray(m.GetFaceVertexIndicesAttr().Get(),
                       dtype=np.int64).reshape(-1, 3)
        assert (cnt == 3).all(), name
        assert fr.open_edge_count(f) == 0, name          # still a solid
        assert np.isfinite(v).all(), name
        _faces, pristine = bench.face_report(v, f, sz)
        assert pristine <= 2, (name, pristine, _faces)
        # the ladder actually applied: a tiny piece stays cheap
        assert len(f) <= 900, (name, len(f))
        if not gouged:
            assert len(f) <= 120, (name, len(f))


def test_the_refusal_ladder_still_protects_kit_and_sliced_art():
    """The wiring must never reach anything that is not one of our own boxes.
    A UV-carrying mesh is kit/sliced art (chipping drops the cladding) and an
    open shell is the `vtkStripper::GetPointCells` SIGSEGV in the round-4
    catalogue — both have to PASS THROUGH, not be handled."""
    qc = _qc()
    st = _stage()
    _plain_box(st, "/World/B/uv", 1.2, 0.3, 0.25, with_st=True)
    assert not qc._chip_prim(st, "/World/B/uv", qc.spec_for_shape(
        (1.2, 0.3, 0.25)))
    from pxr import Sdf, UsdGeom, Vt
    op = UsdGeom.Mesh.Define(st, Sdf.Path("/World/B/open"))
    op.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0),
                                       Gf.Vec3f(1, 1, 0), Gf.Vec3f(0, 1, 0)]))
    op.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    op.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    assert not qc._chip_prim(st, "/World/B/open",
                             qc.spec_for_shape((1.0, 1.0, 0.01)))


def _qf():
    from disaster import quake_flow as qf
    return qf


def test_chip_authored_routes_a_mixed_bag_and_tallies_for_the_proof_line():
    """`_chip_authored` is what every `quake_flow` call site uses. It must
    measure each piece itself (one call site holds a lintel, a quoin and a
    coping stone), tally into `ctx`, and never raise on a bad path."""
    qf = _qf()
    st = _stage()
    paths = []
    for name, (sz, _g) in QF_SHAPES.items():
        p = "/World/B/mixed_" + name
        _plain_box(st, p, *sz)
        paths.append(p)
    _plain_box(st, "/World/B/mixed_uv", 1.2, 0.3, 0.25, with_st=True)
    ctx = {"stage": st, "parent": "/World/B", "mats": {}}
    n = qf._chip_authored(ctx, paths + ["/World/B/mixed_uv", None, "/nope"],
                          why="test")
    assert n == len(paths), (n, len(paths))
    assert ctx["_chip_n"] == n
    assert ctx["_chip_m"] == 1, ctx["_chip_m"]       # the UV mesh passed
    assert ctx["_chip_by"]["test"] == (n, 1)


def test_chip_authored_is_a_no_op_under_qc_chip_0(monkeypatch):
    """The escape hatch reaches the new wiring too: nothing is touched and
    nothing is counted as chipped."""
    qf = _qf()
    st = _stage()
    _plain_box(st, "/World/B/off", 1.8, 0.26, 0.24)
    before = list(UsdGeom.Mesh(
        st.GetPrimAtPath("/World/B/off")).GetPointsAttr().Get())
    monkeypatch.setenv("QC_CHIP", "0")
    ctx = {"stage": st, "parent": "/World/B", "mats": {}}
    assert qf._chip_authored(ctx, ["/World/B/off"]) == 0
    after = list(UsdGeom.Mesh(
        st.GetPrimAtPath("/World/B/off")).GetPointsAttr().Get())
    assert before == after


def test_the_proof_line_is_printed_and_kept_in_the_notes(capsys):
    """A scene bake log has to SHOW the chips fired. Round 5 shipped with the
    wiring believed done and no positive evidence anywhere that it ran, which
    is how three review rounds went by with the pillars still cuboids."""
    qf = _qf()
    st = _stage()
    _plain_box(st, "/World/B/pl", 1.8, 0.26, 0.24)
    ctx = {"stage": st, "parent": "/World/B", "mats": {}, "notes": []}
    qf._chip_authored(ctx, ["/World/B/pl"], why="lintel")
    qf._chip_report(ctx)
    out = capsys.readouterr().out
    assert "[chip] quake_flow:" in out and "chipped" in out and "passed" in out
    assert "lintel" in out
    assert any("[chip] quake_flow:" in q for q in ctx["notes"])


def test_the_skip_decisions_are_recorded_where_they_were_made():
    """Ground, soil, sheet metal, a surviving shear core and an INTACT roof
    are deliberately not chipped. The reasoning has to survive next to the
    code, or the next round re-litigates it from scratch (or, worse, wires
    them).

    LIVE REVIEW ROUND: the fissure's MOUND (the earthen ridge `_c_geom_mesh`
    authors, called from `_c_fissure_trace` / `_c_fissures`) stays on this
    list — still soil, still never chipped. Its new cracked-asphalt BAND
    (`_c_fissure_pave`) does NOT: "make it irregular cracked shapes, not
    just rectangles" / "make the asphalt/ground actually look cracked near
    the fissure" (user) is cast pavement, not ground relief, and is
    deliberately chipped — checked below, SEPARATELY, so this test still
    fails if that call ever migrates onto the mound path instead of staying
    where it belongs. The proximity check reads off `_c_geom_mesh`, not
    `_c_fissures`, because `_c_fissures` no longer authors any geometry
    itself — it only loops corners and delegates."""
    import inspect
    qf = _qf()
    src = inspect.getsource(qf)
    for name in ("_c_clods", "_c_geom_mesh", "_c_overturn_ground",
                 "r_signage_fail"):
        i = src.index("def " + name + "(")
        assert "NOT CHIPPED" in src[max(0, i - 900):i], name
    for name in ("_shaft", "_roof_box"):
        fn = getattr(qf, name)
        assert "NOT CHIPPED" in (fn.__doc__ or ""), name
    # NONE of the ground-relief/mound path grew a chip call. `_c_fissures`
    # and `_c_fissure_trace` both only ever call INTO `_c_geom_mesh` (the
    # mound mesh) and `_c_fissure_pave` (the asphalt band) — neither
    # authors a box of its own any more — so this still means what it
    # always meant: the ground relief itself is never chipped.
    for name in ("_c_clods", "_c_fissures", "_c_fissure_trace",
                 "_c_geom_mesh", "_c_overturn_ground",
                 "r_signage_fail", "_shaft", "_roof_box"):
        body = inspect.getsource(getattr(qf, name))
        assert "_chip_authored(" not in body, name

    # THE ONE DELIBERATE EXCEPTION beside the mound: `_c_fissure_pave`'s
    # cracked-asphalt band IS cast pavement, so it DOES chip — pinned here
    # rather than folded into the "never chip" list above, so the test
    # still means something in both directions: a chip call that goes
    # missing here is caught exactly as one that appears above is.
    pave_body = inspect.getsource(qf._c_fissure_pave)
    i = pave_body.index("_chip_authored(")
    rationale = pave_body[max(0, i - 300):i]
    assert "CHIP" in rationale, (
        "no rationale comment immediately above _c_fissure_pave's chip call")
    assert "pavement" in rationale.lower()
