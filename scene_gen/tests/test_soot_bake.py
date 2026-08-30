#!/usr/bin/env python3
"""test_soot_bake.py — does the soot land on the right TEXEL of a kit
module's OWN base-colour atlas, not just the right crop of the skin?

    python3 scene_gen/tests/test_soot_bake.py
    pytest -q scene_gen/tests/test_soot_bake.py

WHY THIS EXISTS
---------------
`disaster/soot_bake.py` replaces "crop the skin behind a module and stretch
it over the module's whole base map" with "rasterise the module's own
triangles into texel space, recover each texel's local 3D position, sample
the skin there, composite through the module's own UVs." That is only
correct if the maths behind `uv_position_map` (fan triangulation, the
vertex/varying/faceVarying UV lookup, the row-0-is-v=1 flip, the
seam/wrap handling) and `bake_module` (the xform's row-vector convention,
`sample_skin`'s column-wrap/row-clamp addressing, the exact
`soot_plume.merge_rgb` compositing) all agree with each other and with
`soot_plume`'s own conventions. Every test below builds a tiny SYNTHETIC
skin and mesh by hand — no USD, no Kit, no Isaac Sim — so the exact texel a
marker lands on can be predicted and checked directly:

  * (a) a single UV island covering a sub-rectangle of the map: a soot
    marker on the wall lands in the sub-rectangle's own sub-region, not
    smeared over the whole map;
  * (b) two non-adjacent islands (a wall split into a bottom and top
    module): each marker lands ONLY in its own module's island;
  * (c) the same island expressed as faceVarying + indexed UVs (two
    triangles) instead of per-vertex UVs, to check the face-vertex-slot
    lookup;
  * (d) a non-identity xform (translate + rotate 90 degrees onto the E
    face): the marker still lands where `side_u` says the E face is;
  * (e) rasterisation performance: 1300 triangles into 512x512 in well
    under 2 s;
  * (f) two fan triangles covering a full 0..1 UV square tile every texel
    between them (no crack down the shared diagonal).

It runs host-side in well under a minute.
"""

import math
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import soot_bake as sb            # noqa: E402


# ---------------------------------------------------------------------------
# A synthetic skin and mass, per the task brief
# ---------------------------------------------------------------------------
def _sk(h=300, w=1000, ppm=10.0, per=100.0, H=30.0, z0=0.0):
    rgba = np.zeros((h, w, 4), dtype=np.float32)
    offsets = {"S": 0.0, "E": 30.0, "N": 50.0, "W": 80.0}
    return {"rgba": rgba, "ppm": ppm, "per": per, "H": H, "z0": z0,
            "offsets": offsets}


def _mass():
    return dict(W=30.0, D=20.0, cx=15.0, cy=10.0, yaw=0.0)


def _mark(sk, side, u0, u1, z0, z1):
    """Paint a fully-opaque, pure-black soot marker into the skin over
    world span `u0..u1` (metres along `side`) and `z0..z1` (world height).
    Pure black + alpha=1 means a fully-covered composite equals the marker
    colour exactly (`merge_rgb`'s formula reduces to the crop colour at
    alpha=1), and alpha=0 elsewhere means an untouched composite equals the
    base exactly -- both make the assertions below exact equalities rather
    than fuzzy comparisons."""
    h, w = sk["rgba"].shape[:2]
    ppm, H, z00 = sk["ppm"], sk["H"], sk["z0"]
    off = sk["offsets"][side]
    c0 = int(math.floor((off + u0) * ppm))
    c1 = int(math.ceil((off + u1) * ppm))
    r0 = int(math.floor((H - (z1 - z00)) * ppm))
    r1 = int(math.ceil((H - (z0 - z00)) * ppm))
    sk["rgba"][r0:r1, c0:c1, 0:3] = 0.0
    sk["rgba"][r0:r1, c0:c1, 3] = 1.0


def _uv_rect_to_texels(u0, u1, v0, v1, px):
    c0 = int(math.floor(u0 * px))
    c1 = int(math.ceil(u1 * px))
    r0 = int(math.floor((1.0 - v1) * px))
    r1 = int(math.ceil((1.0 - v0) * px))
    return r0, r1, c0, c1


def _changed_mask(out, base_val):
    return np.any(np.abs(out - base_val) > 1e-4, axis=-1)


def _assert_rect(out, base_val, mask, rect, px, tol=1):
    """Every covered texel strictly inside `rect` (shrunk by `tol`) is
    marked; every covered texel strictly outside it (grown by `tol`) is
    untouched base; every UNCOVERED texel anywhere is untouched base."""
    r0, r1, c0, c1 = rect
    changed = _changed_mask(out, base_val)
    rr, cc = np.mgrid[0:px, 0:px]
    inside = ((rr >= r0 + tol) & (rr < r1 - tol)
              & (cc >= c0 + tol) & (cc < c1 - tol))
    outside = ((rr < r0 - tol) | (rr >= r1 + tol)
               | (cc < c0 - tol) | (cc >= c1 + tol))
    assert inside.any(), "test rect too small for the tolerance"
    assert changed[inside].all(), "marker did not fully cover its own texels"
    assert not changed[outside & mask].any(), \
        "soot leaked outside the expected UV rectangle"
    assert not changed[~mask].any(), "uncovered texels were not left as base"


def _assert_marker_in_island(changed, island_rect, marker_rect, tol=1):
    """Like `_assert_rect`, but the "must stay unmarked" check is scoped to
    ONE island's own rectangle rather than the whole canvas -- needed when a
    SECOND, unrelated island elsewhere on the same map is legitimately
    marked too (the two-islands test)."""
    ir0, ir1, ic0, ic1 = island_rect
    mr0, mr1, mc0, mc1 = marker_rect
    rr, cc = np.mgrid[0:changed.shape[0], 0:changed.shape[1]]
    in_island = (rr >= ir0) & (rr < ir1) & (cc >= ic0) & (cc < ic1)
    inside = ((rr >= mr0 + tol) & (rr < mr1 - tol)
              & (cc >= mc0 + tol) & (cc < mc1 - tol))
    near_marker = ((rr >= mr0 - tol) & (rr < mr1 + tol)
                   & (cc >= mc0 - tol) & (cc < mc1 + tol))
    assert inside.any(), "test rect too small for the tolerance"
    assert changed[inside].all(), "marker did not fully cover its own texels"
    assert not changed[in_island & ~near_marker].any(), \
        "soot leaked elsewhere within the same island"


_BASE_VAL = 0.5
# Deliberately coarser than BAKE_PX for the boundary-exactness tests: the
# synthetic skin's own resolution is ppm=10 (0.1 m/px), and `sample_skin`
# is BILINEAR, so a soot edge fades over about one skin pixel of world
# distance. At BAKE_PX=512 that fade band is several output texels wide
# (0.1 m is a bigger fraction of a small local feature once magnified to
# 512 px) and would blow the "one texel" tolerance below; at this coarser
# working resolution one output texel already covers about as much world
# distance as one skin pixel, so the fade stays inside the tolerance.
_PX = 64


def _base(hb=8, wb=8):
    return np.full((hb, wb, 3), _BASE_VAL, dtype=np.float32)


# ---------------------------------------------------------------------------
# (a) one UV island, a sub-rectangle of the map
# ---------------------------------------------------------------------------
def test_single_island_quad():
    sk = _sk()
    m = _mass()
    # a wall point at world x=1..2 (side S, u = x directly -- side_u's own
    # convention with cx=15, W=30) lands on this quad's local x=1..2
    _mark(sk, "S", 1.0, 2.0, 2.0, 3.0)

    points = np.array([[0, 0, 0], [4, 0, 0], [4, 0, 3], [0, 0, 3]],
                      dtype=np.float64)
    fvc = [4]
    fvi = [0, 1, 2, 3]
    uv = np.array([[0, 0], [0.56, 0], [0.56, 0.42], [0, 0.42]],
                  dtype=np.float64)

    pos, mask = sb.uv_position_map(points, fvc, fvi, uv, "vertex", px=_PX)
    assert mask.sum() > 0

    xform = np.eye(4)
    out = sb.bake_module(sk, "S", m, xform, pos, mask, _base(), px=_PX)

    # x 1..2 of a 0..4 span is 1/4..2/4 of the UV width 0.56; z 2..3 of a
    # 0..3 span is the top third of the UV height 0.42
    rect = _uv_rect_to_texels(0.25 * 0.56, 0.50 * 0.56,
                              2.0 / 3.0 * 0.42, 0.42, _PX)
    _assert_rect(out, _BASE_VAL, mask, rect, _PX)

    # nothing outside the island's own 0.56 x 0.42 rectangle was touched
    island_rect = _uv_rect_to_texels(0.0, 0.56, 0.0, 0.42, _PX)
    ir0, ir1, ic0, ic1 = island_rect
    rr, cc = np.mgrid[0:_PX, 0:_PX]
    off_island = (rr < ir0) | (rr >= ir1) | (cc < ic0) | (cc >= ic1)
    changed = _changed_mask(out, _BASE_VAL)
    assert not changed[off_island].any()
    assert not mask[off_island].any()


# ---------------------------------------------------------------------------
# (b) two non-adjacent islands (bottom half / top half of one wall)
# ---------------------------------------------------------------------------
def test_two_islands_do_not_bleed():
    sk = _sk()
    m = _mass()
    # top-half marker: z 2..3 (inside the top module's z 1.5..3 span)
    _mark(sk, "S", 1.0, 2.0, 2.0, 3.0)
    # bottom-half marker: z 0..0.5 (inside the bottom module's z 0..1.5 span)
    _mark(sk, "S", 1.0, 2.0, 0.0, 0.5)

    points = np.array([
        [0, 0, 0.0], [4, 0, 0.0], [4, 0, 1.5], [0, 0, 1.5],   # bottom half
        [0, 0, 1.5], [4, 0, 1.5], [4, 0, 3.0], [0, 0, 3.0],   # top half
    ], dtype=np.float64)
    fvc = [4, 4]
    fvi = [0, 1, 2, 3, 4, 5, 6, 7]
    uv = np.array([
        [0.6, 0.5], [0.9, 0.5], [0.9, 0.7], [0.6, 0.7],       # bottom island
        [0.1, 0.1], [0.4, 0.1], [0.4, 0.3], [0.1, 0.3],       # top island
    ], dtype=np.float64)

    pos, mask = sb.uv_position_map(points, fvc, fvi, uv, "vertex", px=_PX)
    xform = np.eye(4)
    out = sb.bake_module(sk, "S", m, xform, pos, mask, _base(), px=_PX)
    changed = _changed_mask(out, _BASE_VAL)

    # top island: x fraction 1/4..2/4 of [0.1,0.4]; z fraction (2-1.5)/1.5
    # .. (3-1.5)/1.5 of [0.1,0.3]
    top_rect = _uv_rect_to_texels(
        0.1 + 0.25 * 0.3, 0.1 + 0.50 * 0.3,
        0.1 + (0.5 / 1.5) * 0.2, 0.1 + (1.5 / 1.5) * 0.2, _PX)
    # bottom island: same x fraction of [0.6,0.9]; z fraction 0..(0.5/1.5)
    # of [0.5,0.7]
    bot_rect = _uv_rect_to_texels(
        0.6 + 0.25 * 0.3, 0.6 + 0.50 * 0.3,
        0.5 + 0.0, 0.5 + (0.5 / 1.5) * 0.2, _PX)

    top_full = _uv_rect_to_texels(0.1, 0.4, 0.1, 0.3, _PX)
    bot_full = _uv_rect_to_texels(0.6, 0.9, 0.5, 0.7, _PX)

    # each marker fully covers its own expected sub-region and leaks
    # nowhere else WITHIN its own island
    _assert_marker_in_island(changed, top_full, top_rect)
    _assert_marker_in_island(changed, bot_full, bot_rect)

    # and nothing at all changed outside either island (uncovered texels,
    # or covered texels of the wall that belong to neither module)
    rr, cc = np.mgrid[0:_PX, 0:_PX]
    tf0, tf1, tc0f, tc1f = top_full
    bf0, bf1, bc0f, bc1f = bot_full
    in_either_island = (((rr >= tf0) & (rr < tf1) & (cc >= tc0f) & (cc < tc1f))
                         | ((rr >= bf0) & (rr < bf1) & (cc >= bc0f) & (cc < bc1f)))
    assert not changed[~in_either_island].any(), \
        "soot appeared outside both modules' own UV islands"


# ---------------------------------------------------------------------------
# (c) faceVarying + indexed UVs (two triangles, not one quad face)
# ---------------------------------------------------------------------------
def test_facevarying_indexed_uvs():
    sk = _sk()
    m = _mass()
    _mark(sk, "S", 1.0, 2.0, 2.0, 3.0)

    points = np.array([[0, 0, 0], [4, 0, 0], [4, 0, 3], [0, 0, 3]],
                      dtype=np.float64)
    fvc = [3, 3]
    fvi = [0, 1, 2, 0, 2, 3]
    uv_pool = np.array([[0, 0], [0.56, 0], [0.56, 0.42], [0, 0.42]],
                       dtype=np.float64)
    uv_indices = [0, 1, 2, 0, 2, 3]

    pos, mask = sb.uv_position_map(points, fvc, fvi, uv_pool, "faceVarying",
                                    uv_indices=uv_indices, px=_PX)
    assert mask.sum() > 0

    xform = np.eye(4)
    out = sb.bake_module(sk, "S", m, xform, pos, mask, _base(), px=_PX)

    rect = _uv_rect_to_texels(0.25 * 0.56, 0.50 * 0.56,
                              2.0 / 3.0 * 0.42, 0.42, _PX)
    _assert_rect(out, _BASE_VAL, mask, rect, _PX)


# ---------------------------------------------------------------------------
# (d) a non-identity xform: translate + rotate 90 deg onto the E face
# ---------------------------------------------------------------------------
def test_non_identity_xform_lands_on_e_face():
    sk = _sk()
    m = _mass()
    # E's own u runs with world y directly here (cy=10=D/2), same
    # simplification side S got from cx=15=W/2 -- side_u(m,"E",30,y) == y
    _mark(sk, "E", 1.0, 2.0, 2.0, 3.0)

    # local mesh: same flat quad as (a), lying in the x-z plane at y=0
    points = np.array([[0, 0, 0], [4, 0, 0], [4, 0, 3], [0, 0, 3]],
                      dtype=np.float64)
    fvc = [4]
    fvi = [0, 1, 2, 3]
    uv = np.array([[0, 0], [0.56, 0], [0.56, 0.42], [0, 0.42]],
                  dtype=np.float64)
    pos, mask = sb.uv_position_map(points, fvc, fvi, uv, "vertex", px=_PX)

    # rotate +90 deg about z (row-vector convention: local (x,0,z) -> world
    # (0, x, z)) then translate to the E face plane (world x = cx+W/2 = 30)
    xform = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [30.0, 0.0, 0.0, 1.0],
    ])
    # sanity: local x=1 -> world (30, 1, z); side_u(m,"E",30,1) should be 1
    from disaster import quake_flow as qf   # noqa: E402  (pure placement math)
    lx, ly = qf._to_local(m, 30.0, 1.0)
    assert abs((ly + m["D"] / 2.0) - 1.0) < 1e-9

    out = sb.bake_module(sk, "E", m, xform, pos, mask, _base(), px=_PX)
    rect = _uv_rect_to_texels(0.25 * 0.56, 0.50 * 0.56,
                              2.0 / 3.0 * 0.42, 0.42, _PX)
    _assert_rect(out, _BASE_VAL, mask, rect, _PX)


# ---------------------------------------------------------------------------
# (e) rasterisation performance
# ---------------------------------------------------------------------------
def test_rasterisation_performance():
    rng = np.random.default_rng(0)
    T = 1300
    fvc = [3] * T
    fvi = list(range(3 * T))
    # small islands, like a real kit atlas -- not 1300 triangles each
    # covering the whole canvas
    centers = rng.uniform(0.05, 0.95, size=(T, 2))
    offs = rng.uniform(-0.02, 0.02, size=(T, 3, 2))
    uv = (centers[:, None, :] + offs).reshape(-1, 2)
    points = rng.uniform(-1.0, 1.0, size=(3 * T, 3))

    t0 = time.perf_counter()
    pos, mask = sb.uv_position_map(points, fvc, fvi, uv, "vertex",
                                    px=sb.BAKE_PX)
    dt = time.perf_counter() - t0
    print(f"uv_position_map: {T} triangles -> {sb.BAKE_PX}x{sb.BAKE_PX}: "
          f"{dt:.3f} s")
    assert dt < 2.0, f"rasterisation took {dt:.3f} s, budget is 2 s"
    assert mask.sum() > 0


# ---------------------------------------------------------------------------
# (f) coverage sanity: no crack down the shared diagonal
# ---------------------------------------------------------------------------
def test_full_square_quad_has_no_cracks():
    points = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
                      dtype=np.float64)
    fvc = [4]
    fvi = [0, 1, 2, 3]
    uv = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float64)

    pos, mask = sb.uv_position_map(points, fvc, fvi, uv, "vertex", px=_PX)
    coverage = float(mask.mean())
    print(f"full-square coverage: {coverage * 100.0:.2f}%")
    assert coverage >= 0.99, coverage


# ---------------------------------------------------------------------------
# triangles() itself: fan arity and face_ids restriction
# ---------------------------------------------------------------------------
def test_triangulate_fan_and_face_ids():
    fvc = [4, 5, 3]          # quad, pentagon, triangle
    fvi = list(range(4 + 5 + 3))
    tri, tri_face, tri_slot = sb.triangles(fvc, fvi)
    assert tri.shape == (2 + 3 + 1, 3)
    assert list(tri_face) == [0, 0, 1, 1, 1, 2]
    # slot 0 of face 1 (the pentagon) starts right after the quad's 4 slots
    assert tri_slot[2].tolist() == [4, 5, 6]

    tri2, tri_face2, _ = sb.triangles(fvc, fvi, face_ids=[1])
    assert tri_face2.tolist() == [1, 1, 1]
    assert tri2.shape == (3, 3)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)

    # debug artefact: the baked map of test (a), for a reviewer to look at
    try:
        from PIL import Image

        out_dir = ("/tmp/claude-1000/-home-krrishjain-SEI-COA-disaster-dataset"
                   "/27af4e96-2b10-45f3-a7bf-2bee5e437ff2/scratchpad/agent_bake")
        os.makedirs(out_dir, exist_ok=True)

        sk = _sk()
        m = _mass()
        _mark(sk, "S", 1.0, 2.0, 2.0, 3.0)
        points = np.array([[0, 0, 0], [4, 0, 0], [4, 0, 3], [0, 0, 3]],
                          dtype=np.float64)
        uv = np.array([[0, 0], [0.56, 0], [0.56, 0.42], [0, 0.42]],
                      dtype=np.float64)
        pos, mask = sb.uv_position_map(points, [4], [0, 1, 2, 3], uv,
                                       "vertex", px=_PX)
        out = sb.bake_module(sk, "S", m, np.eye(4), pos, mask, _base(),
                             px=_PX)
        png = (np.clip(out, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)
        Image.fromarray(png, mode="RGB").save(os.path.join(out_dir,
                                                             "bake_a.png"))
        print("wrote " + os.path.join(out_dir, "bake_a.png"))
    except Exception as exc:  # pragma: no cover -- debug helper only
        print("debug PNG skipped: " + repr(exc))
