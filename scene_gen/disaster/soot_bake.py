"""soot_bake — bake the perimeter soot skin THROUGH a kit module's OWN UVs,
not through a stretch of whatever the skin happened to have behind it.

WHY THIS EXISTS. Until now the soot reached a placed module by cropping
`soot_plume.skin()`'s canvas to the module's measured span (`piece_crop`)
and STRETCHING that crop corner-to-corner over the module's whole
base-colour texture (`soot_plume.merge_piece`). That assumes a module's UVs
address its own base map corner to corner, uniformly. A probe of the real
kit meshes (`tools/pack_structure_probe.py`, 2026-08-30) says that
assumption is false: the base-colour maps are UV ATLASES with islands — one
wall face uses only u 0.00..0.56, v 0.00..0.42 of its map; another spans
v 0.03..0.73 with the module's OWN bottom vertices sitting at v~0.44 and its
top at v~0.55 — non-monotone, and nowhere near corner-to-corner. Stretching
a crop over the WHOLE map under that layout puts soot on texels the module
never draws from at all, and skips texels it does. The fix has to go
through the mesh's own triangles and UVs, texel by texel, the way any
texture baker does it.

THE PIPELINE, per module TYPE (not per placed instance):

 1. `triangles` fans every polygon face (arity 3, 4, or n) into triangles,
    carrying both the POINT index of each corner (for `vertex`/`varying`
    primvars) and its FACE-VERTEX SLOT — the corner's position in the flat
    `face_vertex_indices` array — because `faceVarying` primvars (most
    kit meshes' UVs) are indexed by slot, not by point.
 2. `uv_position_map` rasterises those triangles into a `px` x `px` texel
    grid IN THE MODULE'S OWN UV SPACE and, for every covered texel,
    barycentrically interpolates the triangle's 3 corner LOCAL positions
    into it. The output (`pos`, `mask`) depends only on the MESH (its
    points, topology and UV primvar) — never on where an instance of that
    mesh is placed in the world. `kit_substitute` (the caller) caches this
    pair once per (mesh, UV set) and reuses it for every placed instance of
    that piece, which is where the real cost of this design is paid down:
    one rasterisation per KIT PIECE TYPE, not one per placement.
 3. `bake_module` takes that cached `(pos, mask)`, ONE PLACEMENT's `xform`
    and `side`/mass, transforms the covered texels' local positions to
    world, samples the skin there (`sample_skin`, which is where
    `soot_plume.side_u` and the skin's own column-wrap/row-clamp addressing
    live), and composites over the base map with the exact per-pixel
    formula `soot_plume.merge_rgb` uses (desaturate the base by
    `DESAT * alpha`, then lerp to the soot colour by `alpha`) — so a baked
    module and a `piece_crop`-merged one that happen to agree on geometry
    read identically.

ROW 0 IS v = 1, matching every other image convention already in this
codebase (`soot_plume.skin`'s own docstring: "Row 0 of every array here is
the TOP of the wall"). A UV's v axis runs the OPPOSITE way (v = 0 at the
texture's bottom-left origin, the OpenGL/USD `st` convention), so texel
(r, c) covers u in [c/px, (c+1)/px) and v in [1 - (r+1)/px, 1 - r/px) — the
row must be FLIPPED out of v, not read off it directly, or every bake would
come out upside down against `sample_skin`'s own row convention.

WHY A TEXEL IS COVERED BY ITS CENTRE, WITH AN EPSILON. Two fan triangles
from one quad share an exact diagonal edge in UV space; testing "is the
texel centre inside" with a small NEGATIVE tolerance on the barycentric
weights means a centre sitting exactly on that shared edge (floating-point
noise either way) satisfies BOTH triangles' inside test, so it is always
claimed by whichever triangle rasterises second — never by neither. A
strict `>= 0` test with no slack is exactly the case that starts leaving
one-pixel gaps down internal seams, which is the "cracks" `test_soot_bake`
checks for on a full 0..1 quad (>= 99 % coverage from two triangles).

UV WRAPPING, AND WHY 1.0 DOES NOT FOLD TO 0.0. UVs outside [0, 1) wrap
(u mod 1, v mod 1) before rasterising, because an atlas island can
legitimately be authored at, say, u in [3.00, 3.56] (the same island tiled
three times over) and wrapping brings it back to the canvas this function
actually owns. But an exact 1.0 is the FAR edge of a face whose texture
fills its whole UV space corner to corner — the ordinary, non-atlas case,
and the shape `test_full_square_quad_has_no_cracks` exercises — not the
same column as u = 0. Folding a literal 1.0 down to 0.0 (naive
`u - floor(u)`) collapses that corner onto its neighbour and rasterises a
zero-area triangle; `_wrap01` instead wraps into (0, 1], so 1.0 (and 2.0,
3.0, ...) stays put and only a GENUINE excursion outside the unit interval
gets folded. Separately, a triangle whose (correctly-wrapped) corners still
end up more than half the canvas apart in u or v is treated as a SEAM
STRADDLER — a small triangle that happens to sit across the u=0≡1 (or
v=0≡1) join, authored with continuous, un-wrapped UVs on either side
(0.98 and 1.02, not 0.98 and 0.02) — and is rasterised with its ORIGINAL,
un-wrapped corner values instead (which are naturally close together,
because the wrap step is what would have broken them apart), with the
resulting texel row/column simply taken modulo `px` at the end. This is
rare on these meshes (per the probe, islands sit well inside their own
map), so the handling is kept to exactly that: no general multi-period
tiling within one triangle is supported, and none is needed here.

WHY NEAREST (LINSPACE-INDEX) UPSAMPLE OF THE BASE MAP. These kit base-colour
maps are often as small as 128 px; the soot is a physically-derived field at
whatever resolution `sk["ppm"]` gives it, frequently finer. `bake_module`
upsamples the base to the working `px` with the same `np.linspace(...).
astype(int)` nearest-index trick `soot_plume.merge_rgb` already uses for
the same reason stated there: a small base map must not be allowed to blur
out a soot pattern with real detail in it, and nobody is looking closely
enough at brick-texture aliasing under a scorch mark to need bilinear here.

PERFORMANCE. `uv_position_map` rasterises PER TRIANGLE — a Python loop over
triangles, each one vectorised over its own texel bounding box with numpy
(meshgrid + barycentric weights in one shot, no per-texel Python). On kit
meshes an atlas island is a small fraction of the canvas, so a triangle's
bounding box is a small fraction of `px` x `px`; 1300 triangles into a
512 x 512 canvas run in well under 2 s on a laptop CPU (measured in
`test_soot_bake.py`). A single legitimately huge triangle (a bounding box
approaching the full canvas, as in the full-square-quad test) is still just
one more iteration of the same loop and stays cheap.
"""

import math

import numpy as np

from .soot_plume import DESAT, side_u

BAKE_PX = 512


# ---------------------------------------------------------------------------
# Fan triangulation
# ---------------------------------------------------------------------------
def triangles(face_vertex_counts, face_vertex_indices, face_ids=None):
    """Fan-triangulate polygon faces (arity 3, 4, or an n-gon fans from its
    own first corner). Returns `(tri, tri_face, tri_slot)`:

      tri       (T, 3) int  -- POINT index of each triangle's 3 corners
      tri_face  (T,)   int  -- source face index of each triangle (the true
                               index into `face_vertex_counts`, even when
                               `face_ids` restricts which faces are visited)
      tri_slot  (T, 3) int  -- FACE-VERTEX SLOT (the corner's position in
                               the flat `face_vertex_indices`) of each
                               corner -- what a `faceVarying` UV primvar is
                               indexed by.
    """
    counts = np.asarray(face_vertex_counts, dtype=np.int64)
    idx = np.asarray(face_vertex_indices, dtype=np.int64)
    if counts.size == 0:
        z3 = np.zeros((0, 3), dtype=np.int64)
        return z3, np.zeros((0,), dtype=np.int64), z3.copy()
    starts = np.concatenate(([0], np.cumsum(counts)[:-1]))
    faces = range(len(counts)) if face_ids is None else list(face_ids)

    tri, tri_face, tri_slot = [], [], []
    for f in faces:
        n = int(counts[f])
        if n < 3:
            continue
        s = int(starts[f])
        for i in range(1, n - 1):
            tri.append((int(idx[s]), int(idx[s + i]), int(idx[s + i + 1])))
            tri_slot.append((s, s + i, s + i + 1))
            tri_face.append(f)

    if not tri:
        z3 = np.zeros((0, 3), dtype=np.int64)
        return z3, np.zeros((0,), dtype=np.int64), z3.copy()
    return (np.asarray(tri, dtype=np.int64),
            np.asarray(tri_face, dtype=np.int64),
            np.asarray(tri_slot, dtype=np.int64))


# ---------------------------------------------------------------------------
# UV lookup and wrapping
# ---------------------------------------------------------------------------
def _corner_uv(tri, tri_slot, uv, uv_interp, uv_indices):
    """(T, 3, 2) UV per triangle corner, by the USD-primvar lookup rule:
    `vertex`/`varying` index by POINT, `faceVarying` by FACE-VERTEX SLOT;
    either way `uv[uv_indices[...]]` when an index array is given, else
    `uv[...]` directly."""
    uv = np.asarray(uv, dtype=np.float64)
    key = tri_slot if uv_interp == "faceVarying" else tri
    if uv_indices is not None:
        ui = np.asarray(uv_indices, dtype=np.int64)
        return uv[ui[key]]
    return uv[key]


def _wrap01(x):
    """Wrap into (0, 1], not [0, 1): an authored UV of exactly 1.0 (or 2.0,
    ...) is the FAR edge of a face whose texture fills its own UV space
    corner to corner, not the same column as u/v = 0 -- see the module
    docstring's "UV WRAPPING" note. Folding it to 0.0 collapses a
    full-extent triangle onto a point."""
    w = np.mod(x, 1.0)
    return np.where((w == 0.0) & (x != 0.0), 1.0, w)


# ---------------------------------------------------------------------------
# The position map
# ---------------------------------------------------------------------------
def uv_position_map(points, face_vertex_counts, face_vertex_indices, uv,
                     uv_interp, uv_indices=None, face_ids=None, px=BAKE_PX):
    """Rasterise the mesh's (selected) triangles into UV space. Returns
    `(pos, mask)`: `pos` (px, px, 3) float32, the LOCAL 3D position whose
    barycentric combination of triangle corners lands in each texel (zero
    where uncovered); `mask` (px, px) bool, which texels a triangle actually
    covers. Texel (r, c) is u in [c/px, (c+1)/px), v in
    [1 - (r+1)/px, 1 - r/px) -- row 0 is v = 1 (the module docstring's
    "ROW 0 IS v = 1"). Later triangles overwrite earlier ones."""
    if uv_interp not in ("vertex", "varying", "faceVarying"):
        raise ValueError(f"unknown uv_interp {uv_interp!r}")

    points = np.asarray(points, dtype=np.float64)
    tri, _tri_face, tri_slot = triangles(face_vertex_counts,
                                         face_vertex_indices, face_ids)
    pos = np.zeros((px, px, 3), dtype=np.float32)
    mask = np.zeros((px, px), dtype=bool)
    if tri.shape[0] == 0:
        return pos, mask

    tri_uv = _corner_uv(tri, tri_slot, uv, uv_interp, uv_indices)  # (T,3,2)
    eps = 1e-6

    for t in range(tri.shape[0]):
        raw = tri_uv[t]                      # (3, 2), exactly as authored
        wrapped = _wrap01(raw)
        span = wrapped.max(axis=0) - wrapped.min(axis=0)
        # seam straddler (docstring): fall back to the raw, un-wrapped
        # corners, which a well-formed mesh authors as continuous across
        # the join; texels are wrapped modulo px below regardless.
        use = raw if (span[0] > 0.5 or span[1] > 0.5) else wrapped

        xA, yA = float(use[0, 0]) * px, (1.0 - float(use[0, 1])) * px
        xB, yB = float(use[1, 0]) * px, (1.0 - float(use[1, 1])) * px
        xC, yC = float(use[2, 0]) * px, (1.0 - float(use[2, 1])) * px

        denom = (yB - yC) * (xA - xC) + (xC - xB) * (yA - yC)
        if abs(denom) < 1e-9:
            continue  # zero UV area

        cmin = int(math.floor(min(xA, xB, xC)))
        cmax = int(math.ceil(max(xA, xB, xC)))
        rmin = int(math.floor(min(yA, yB, yC)))
        rmax = int(math.ceil(max(yA, yB, yC)))
        # never chase more than one period's worth of texels even on a
        # pathological input -- see the docstring's scope note
        cmax = min(cmax, cmin + px)
        rmax = min(rmax, rmin + px)
        if cmax <= cmin or rmax <= rmin:
            continue

        cc, rr = np.meshgrid(np.arange(cmin, cmax), np.arange(rmin, rmax))
        pc = cc.astype(np.float64) + 0.5
        pr = rr.astype(np.float64) + 0.5

        wA = ((yB - yC) * (pc - xC) + (xC - xB) * (pr - yC)) / denom
        wB = ((yC - yA) * (pc - xC) + (xA - xC) * (pr - yC)) / denom
        wC = 1.0 - wA - wB
        inside = (wA >= -eps) & (wB >= -eps) & (wC >= -eps)
        if not np.any(inside):
            continue

        vA, vB, vC = points[tri[t, 0]], points[tri[t, 1]], points[tri[t, 2]]
        interp = (wA[inside][:, None] * vA + wB[inside][:, None] * vB
                  + wC[inside][:, None] * vC)

        ri = rr[inside] % px
        ci = cc[inside] % px
        pos[ri, ci] = interp.astype(np.float32)
        mask[ri, ci] = True

    return pos, mask


# ---------------------------------------------------------------------------
# Skin sampling and compositing
# ---------------------------------------------------------------------------
def sample_skin(sk, side, m, world_pts, glass=False):
    """Bilinear sample of `sk["rgba"]` at the skin coordinates of
    `world_pts` (P, 3) on `side` of mass `m`: column
    `((offsets[side] + side_u(m, side, wx, wy)) * ppm) mod w`, row
    `(H - (wz - z0)) * ppm` clamped to [0, h-1] -- `soot_plume.skin`'s own
    addressing (`piece_crop` does the same thing per-module-span rather
    than per-point). Column wraps round the perimeter seam; row clamps,
    because there is no wall to sample above the parapet or below the
    plinth. Returns (P, 4) float32. `glass` is reserved -- `sk["rgba"]`
    already carries glass-vs-masonry hardening from `skin()`'s own
    `glass=` argument, so there is nothing left for this function to do
    with it."""
    rgba = sk["rgba"]
    ppm, per, H, z0 = sk["ppm"], sk["per"], sk["H"], sk["z0"]
    h, w = rgba.shape[0], rgba.shape[1]
    off = sk["offsets"][side]

    wp = np.asarray(world_pts, dtype=np.float64).reshape(-1, 3)
    # `side_u` vectorised (it is `quake_flow._to_local` + one affine per
    # side): a per-point Python call was ~0.4 s per 768 px subset
    ang = math.radians(-float(m["yaw"]))
    ca, sa = math.cos(ang), math.sin(ang)
    dx, dy = wp[:, 0] - float(m["cx"]), wp[:, 1] - float(m["cy"])
    lx = dx * ca - dy * sa
    ly = dx * sa + dy * ca
    W, D = float(m["W"]), float(m["D"])
    u = {"S": lx + W / 2.0, "E": ly + D / 2.0,
         "N": W / 2.0 - lx, "W": D / 2.0 - ly}[side]
    col = (off + u) * ppm
    row = (H - (wp[:, 2] - z0)) * ppm

    col0 = np.floor(col).astype(np.int64)
    col1 = col0 + 1
    fc = (col - col0).astype(np.float32)
    c0 = np.mod(col0, w)
    c1 = np.mod(col1, w)

    row_c = np.clip(row, 0.0, h - 1)
    row0 = np.floor(row_c).astype(np.int64)
    row1 = np.minimum(row0 + 1, h - 1)
    fr = (row_c - row0).astype(np.float32)

    p00 = rgba[row0, c0]
    p10 = rgba[row0, c1]
    p01 = rgba[row1, c0]
    p11 = rgba[row1, c1]
    top = p00 * (1.0 - fc)[:, None] + p10 * fc[:, None]
    bot = p01 * (1.0 - fc)[:, None] + p11 * fc[:, None]
    out = top * (1.0 - fr)[:, None] + bot * fr[:, None]
    return out.astype(np.float32)


def bake_module(sk, side, m, xform, pos, mask, base_rgb, px=BAKE_PX,
                desat=None, sampler=None):
    """Composite one placed module's soot through its OWN UVs. `pos`/`mask`
    come from `uv_position_map` on the module's mesh alone (no placement in
    them -- the caller caches this pair per mesh, not per instance). `xform`
    is a 4x4 numpy array, USD/`Gf.Matrix4d` convention: ROW-major, points as
    ROW vectors, translation in the LAST ROW --
    `world = pos_local @ xform[:3, :3] + xform[3, :3]`. `base_rgb` (Hb x Wb
    x 3, 0..1; a 2-D grey map is expanded) is upsampled to `px` x `px`
    nearest (see the module docstring). Every covered texel is composited
    exactly as `soot_plume.merge_rgb` does per pixel: desaturate the base by
    `DESAT * alpha` (or `desat` when given), then lerp to the soot colour by
    `alpha`. Uncovered texels keep the (upsampled) base unchanged. Returns
    (px, px, 3) float32 in 0..1."""
    xform = np.asarray(xform, dtype=np.float64)
    pos = np.asarray(pos, dtype=np.float64)
    d = DESAT if desat is None else float(desat)

    base = np.asarray(base_rgb, dtype=np.float32)
    if base.ndim == 2:
        base = np.repeat(base[..., None], 3, axis=2)
    base = base[..., :3]
    bh, bw = base.shape[0], base.shape[1]
    byi = np.linspace(0, bh - 1, px).astype(int)
    bxi = np.linspace(0, bw - 1, px).astype(int)
    out = base[byi][:, bxi].astype(np.float32).copy()

    if not np.any(mask):
        return out

    flat_pos = pos[mask]                                # (K, 3) local
    world = flat_pos @ xform[:3, :3] + xform[3, :3]      # (K, 3)
    # `sampler(sk, side, m, world)` replaces the per-side lookup for a piece
    # that faces several elevations at once (a merged region-cut block)
    rgba = (sampler or sample_skin)(sk, side, m, world)  # (K, 4)

    a = rgba[:, 3:4]
    b = out[mask]
    grey = b.mean(axis=1, keepdims=True)
    desat_b = b * (1.0 - d * a) + grey * (d * a)
    comp = desat_b * (1.0 - a) + rgba[:, :3] * a
    out[mask] = np.clip(comp, 0.0, 1.0).astype(np.float32)
    return out
