"""tex_compress -- write soot bake textures as GPU block-compressed DDS
(BC1/DXT1) instead of raw PNG, so a per-piece soot map costs ~8x less VRAM
once it lands on the GPU.

WHY THIS EXISTS. `tools/bake_vram_census.py` measured the per-piece soot
maps (`urban_fire._bind_soot`'s `sootbake_*.png`, `gac_fire.bake_atlases`'s
`gacsoot_*.png`) as the single largest, least-shareable item in a fire
bake's own VRAM budget -- 7.93 GB deduplicated across a 48-building corpus
(`city_138`), 88% of the corpus's total deduplicated texture VRAM, because
(unlike the kit's own BaseColor/Normal/ORM atlases, which every building of
the same style shares) every soot composite is content-hashed and therefore
unique to its own building. That number assumes each PNG decodes to a raw
RGBA8 texture with a full mip pyramid in VRAM (`w*h*4*1.333` bytes -- the
4/3 factor is the standard geometric-series cost of a complete mip chain,
the same assumption `bake_vram_census.py` already uses). A GPU-native
block-compressed format never decodes to that: the texture UNIT samples the
compressed blocks directly, so what is IN VRAM is the compressed size, not
the decoded one.

WHY BC1 AND NOT BC7/BC3. Every soot bake this module writes is a fully
OPAQUE RGB composite -- `soot_bake.bake_module` and `gac_fire.bake_atlases`
both hand back an `(px, px, 3)` array with no alpha channel; the soot's own
alpha is already consumed by the composite before it is ever written to
disk. BC1 (DXT1's four-colour "opaque" mode) is the smallest block format
that carries RGB with no wasted alpha bits: 0.5 bytes/texel vs. BC7's 1 or
BC3's 1 (BC3 spends a whole second block on alpha this content never has).
That is 8x smaller than the RGBA8+mips baseline (4 * 1.333 = 5.333
bytes/texel), a bigger win than BC7 (4x) or BC3 (4x) would give the exact
same content, at a quality this codebase already accepts a JPEG-grade loss
budget for (the burn/scorch/ash maps this repo ships are themselves lossy
"Burn_Char_Ref.png"-style photographic textures).

`.dds` is a documented Omniverse USD/MDL texture format (`docs.omniverse.
nvidia.com/usd/latest/common/formats.html`, and `.../extensions/latest/
common/formats.html`, both list `.dds` beside `.png`/`.jpg`/`.exr` as an
accepted MDL material texture). Nothing else in this repo's bake pipeline
is extension-specific: `soot_plume.piece_material_like`/`piece_material`
just `Sdf.AssetPath(str(tex_path))` whatever string they are handed,
`fire_bake.local_texture_override`/`rehome_for_export`/`verify_export`
resolve a texture with `os.path.exists`/Nucleus `stat`, never a suffix
check. Pointing a material's base-colour input at a `.dds` instead of a
`.png` needs no other repo change (verified by grep -- see the
`build-urban-fire-scenes` skill's VRAM-pass notes for the exact greps run).

WHY A HAND-ROLLED ENCODER. No BC-capable image codec is present anywhere
this pipeline's actual runtime can reach: Kit's own embedded
`python3.11` has no pip access (`pip install` inside it -- and inside this
repo's own host venv -- both refuse as "externally managed" with no
persistent venv this pipeline could rely on at bake time). `.dds` files DO
exist inside the container (`find /isaac-sim -iname '*.dds'`, 2026-09-01) --
but every one of them is either Kit's own `omni.kit.window.usd_paths`
sample/demo asset (`lenna*.dds`, shipped specifically to exercise DDS
loading in the USD asset browser -- itself confirmation the renderer reads
DDS for real) or an unrelated PX4/Gazebo ocean texture; every ACTUAL scene
texture this bake pipeline references (every kit/GAC BaseColor/Normal/ORM
map on Nucleus) is `.png`, so there is no encoder anywhere already on this
machine to reuse. `encode_bc1`/`decode_bc1` below are pure
NumPy -- no new dependency anywhere the bake driver runs -- implementing
the standard two-pass DXT1 "range fit then cluster fit" compressor: PCA
endpoints from the block's own extreme-projected pixels, one closed-form
least-squares re-fit of the endpoints against the resulting index
assignment (the same two-step iteration every production BC1 encoder from
`squish` onward uses, capped at one refinement pass here for speed).
Round-tripped against the real soot corpus in `tools/soot_dds_probe.py`:
mean PSNR in the high 30s to low 40s dB per tile (see that tool's own
output for the exact numbers on this repo's corpus) -- well clear of
visible banding on a soot gradient, and confirmed by eye in the paired
before/after PNG that tool also writes.

Both write sites (`urban_fire._bind_soot`, `gac_fire.bake_atlases`) call
`save_soot_texture` instead of `PIL.Image.save` directly; it is the one
place the `SOOT_TEX_COMPRESS` env gate lives, and the one place a caller
needs to change if the extension ever needs to change again.
"""

import math
import os
import struct

import numpy as np

# ---------------------------------------------------------------------------
# env gate -- default ON (2026-09-01 VRAM pass); SOOT_TEX_COMPRESS=0 (or
# "false"/"no"/"") recovers the exact old PNG path for a bisect or an A/B.
# ---------------------------------------------------------------------------
def compress_enabled():
    v = os.environ.get("SOOT_TEX_COMPRESS")
    if v is None:
        return True
    return v.strip().lower() not in ("0", "false", "no", "")


# ---------------------------------------------------------------------------
# RGB565 <-> RGB8, GPU bit-replication convention (matches hardware BC1
# decode exactly, not a rounded division -- `(c5 << 3) | (c5 >> 2)` etc.)
# ---------------------------------------------------------------------------
def _pack565(rgb):
    r = np.clip(np.round(np.asarray(rgb[..., 0], dtype=np.float64) * 31.0 / 255.0), 0, 31).astype(np.uint32)
    g = np.clip(np.round(np.asarray(rgb[..., 1], dtype=np.float64) * 63.0 / 255.0), 0, 63).astype(np.uint32)
    b = np.clip(np.round(np.asarray(rgb[..., 2], dtype=np.float64) * 31.0 / 255.0), 0, 31).astype(np.uint32)
    return ((r << 11) | (g << 5) | b).astype(np.uint32)


def _unpack565(v):
    v = np.asarray(v, dtype=np.uint32)
    r5 = (v >> 11) & 0x1F
    g6 = (v >> 5) & 0x3F
    b5 = v & 0x1F
    r8 = (r5 << 3) | (r5 >> 2)
    g8 = (g6 << 2) | (g6 >> 4)
    b8 = (b5 << 3) | (b5 >> 2)
    return np.stack([r8, g8, b8], axis=-1).astype(np.float64)


def _quantize_order(c0, c1):
    """565-quantize a candidate endpoint pair and force `q0 > q1` (raw
    uint16 order) so the block always decodes in DXT1's four-colour OPAQUE
    mode -- the three-colour/one-bit-alpha mode (`q0 <= q1`) is never what
    this content wants, since none of it carries alpha. Nudges a
    degenerate equal-after-quantization pair apart by one 565 step so the
    two endpoints are never literally the same code (a flat block still
    encodes fine; both palette entries just sit one LSB apart)."""
    q0 = _pack565(c0)
    q1 = _pack565(c1)
    swap = q0 <= q1
    q0s = np.where(swap, q1, q0)
    q1s = np.where(swap, q0, q1)
    eq = q0s == q1s
    q1s = np.where(eq & (q1s > 0), q1s - 1, q1s)
    eq2 = q0s == q1s
    q0s = np.where(eq2, np.minimum(q0s + 1, 0xFFFF), q0s)
    return q0s.astype(np.uint16), q1s.astype(np.uint16)


# fraction toward c1 for each of the 4 DXT1 opaque-mode palette entries,
# indices [c0, c1, (2c0+c1)/3, (c0+2c1)/3]
_T = np.array([0.0, 1.0, 1.0 / 3.0, 2.0 / 3.0])


def _palette(col0, col1):
    col2 = np.round((2.0 * col0 + col1) / 3.0)
    col3 = np.round((col0 + 2.0 * col1) / 3.0)
    return np.stack([col0, col1, col2, col3], axis=1)          # (n, 4, 3)


def _assign(blocks, col0, col1):
    pal = _palette(col0, col1)                                  # (n, 4, 3)
    d = ((blocks[:, :, None, :] - pal[:, None, :, :]) ** 2).sum(-1)  # (n,16,4)
    return np.argmin(d, axis=-1)                                 # (n, 16)


def _pca_endpoints(blocks):
    """Initial (c0, c1) per block: the two pixels furthest apart along the
    block's own dominant colour axis (power iteration on the 3x3
    covariance -- 4x4=16 samples never needs more than a handful of
    iterations to converge). A flat block's covariance is ~0, the
    iteration leaves `v` at 0, and both endpoints land near the block's
    own mean -- harmless, since a flat block's 4 palette entries all but
    coincide anyway."""
    mean = blocks.mean(axis=1, keepdims=True)
    centered = blocks - mean
    cov = np.einsum("bpi,bpj->bij", centered, centered)
    v = np.ones((blocks.shape[0], 3), dtype=np.float64)
    for _ in range(8):
        v = np.einsum("bij,bj->bi", cov, v)
        n = np.linalg.norm(v, axis=1, keepdims=True)
        n = np.where(n < 1e-12, 1.0, n)
        v = v / n
    proj = np.einsum("bpi,bi->bp", centered, v)
    ar = np.arange(blocks.shape[0])
    c0 = blocks[ar, np.argmax(proj, axis=1)]
    c1 = blocks[ar, np.argmin(proj, axis=1)]
    return c0, c1


def _refine_endpoints(blocks, idx, fallback0, fallback1):
    """Closed-form least-squares re-fit of the two endpoints against a
    FIXED index assignment -- the "cluster fit" pass. Minimises
    `sum_i || p_i - ((1-t_i) c0 + t_i c1) ||^2` per channel; `t_i` is the
    palette fraction the pixel's own (already-assigned) index implies.
    Falls back to the input endpoints on a degenerate (near-singular)
    block, e.g. every pixel landed on the same index."""
    t = _T[idx]                                                  # (n, 16)
    one_m_t = 1.0 - t
    a = (one_m_t ** 2).sum(axis=1)
    b = (one_m_t * t).sum(axis=1)
    c = (t ** 2).sum(axis=1)
    det = a * c - b * b
    ok = det > 1e-6
    safe_det = np.where(ok, det, 1.0)
    d = np.einsum("np,npc->nc", one_m_t, blocks)
    e = np.einsum("np,npc->nc", t, blocks)
    c0 = (c[:, None] * d - b[:, None] * e) / safe_det[:, None]
    c1 = (a[:, None] * e - b[:, None] * d) / safe_det[:, None]
    c0 = np.where(ok[:, None], c0, fallback0)
    c1 = np.where(ok[:, None], c1, fallback1)
    return np.clip(c0, 0.0, 255.0), np.clip(c1, 0.0, 255.0)


def _pack_blocks(q0, q1, idx):
    n = q0.shape[0]
    colors = np.empty((n, 2), dtype="<u2")
    colors[:, 0] = q0
    colors[:, 1] = q1
    packed_idx = np.zeros(n, dtype=np.uint32)
    for i in range(16):
        packed_idx |= (idx[:, i].astype(np.uint32) << (2 * i))
    out = np.empty((n, 8), dtype=np.uint8)
    out[:, 0:4] = colors.view(np.uint8).reshape(n, 4)
    out[:, 4:8] = packed_idx.astype("<u4").view(np.uint8).reshape(n, 4)
    return out.tobytes()


def _pad_mult4(img):
    h, w = img.shape[:2]
    ph = max(4, ((h + 3) // 4) * 4)
    pw = max(4, ((w + 3) // 4) * 4)
    if ph == h and pw == w:
        return img
    return np.pad(img, ((0, ph - h), (0, pw - w), (0, 0)), mode="edge")


def encode_bc1(img_u8):
    """`(w, h)`-agnostic (pads to a multiple of 4, minimum 4x4, by edge
    replication) BC1 block-compress of an `(h, w, 3)` RGB array. Returns
    `(block_bytes, nbx, nby)` -- `nbx*nby*8` is the exact compressed size,
    the number every VRAM projection in this module's docstring uses."""
    img = _pad_mult4(np.asarray(img_u8, dtype=np.float64))
    h, w = img.shape[:2]
    nby, nbx = h // 4, w // 4
    blocks = (img.reshape(nby, 4, nbx, 4, 3)
                 .transpose(0, 2, 1, 3, 4)
                 .reshape(nby * nbx, 16, 3))
    c0, c1 = _pca_endpoints(blocks)
    q0, q1 = _quantize_order(c0, c1)
    col0, col1 = _unpack565(q0), _unpack565(q1)
    idx = _assign(blocks, col0, col1)
    c0r, c1r = _refine_endpoints(blocks, idx, col0, col1)
    q0f, q1f = _quantize_order(c0r, c1r)
    col0f, col1f = _unpack565(q0f), _unpack565(q1f)
    idxf = _assign(blocks, col0f, col1f)
    return _pack_blocks(q0f, q1f, idxf), nbx, nby


def decode_bc1(data, nbx, nby):
    """Inverse of `encode_bc1` -- returns an `(nby*4, nbx*4, 3)` uint8
    array (the padded size; callers crop to the real w/h themselves). Used
    for round-trip quality checks and before/after preview PNGs, never on
    the bake's own hot path."""
    n = nbx * nby
    arr = np.frombuffer(data, dtype=np.uint8).reshape(n, 8)
    colors = arr[:, 0:4].copy().view("<u2").reshape(n, 2)
    q0, q1 = colors[:, 0], colors[:, 1]
    idxb = arr[:, 4:8].copy().view("<u4").reshape(n)
    idx = np.zeros((n, 16), dtype=np.uint8)
    for i in range(16):
        idx[:, i] = (idxb >> (2 * i)) & 0x3
    col0, col1 = _unpack565(q0), _unpack565(q1)
    pal = _palette(col0, col1)                                    # (n, 4, 3)
    pixels = pal[np.arange(n)[:, None], idx]                       # (n,16,3)
    blocks = pixels.reshape(nby, nbx, 4, 4, 3)
    img = blocks.transpose(0, 2, 1, 3, 4).reshape(nby * 4, nbx * 4, 3)
    return np.clip(img, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# mip chain (box filter, all the way to 1x1 -- matches the 4/3 "full
# pyramid" VRAM factor `tools/bake_vram_census.py` already assumes for the
# uncompressed case, so the compressed projection is directly comparable)
# ---------------------------------------------------------------------------
def _mip_chain(img_u8):
    levels = [np.asarray(img_u8, dtype=np.float64)]
    h, w = levels[0].shape[:2]
    while h > 1 or w > 1:
        cur = levels[-1]
        ch, cw = cur.shape[:2]
        ph, pw = ch + (ch % 2), cw + (cw % 2)
        if ph != ch or pw != cw:
            cur = np.pad(cur, ((0, ph - ch), (0, pw - cw), (0, 0)), mode="edge")
        nxt = cur.reshape(ph // 2, 2, pw // 2, 2, 3).mean(axis=(1, 3))
        levels.append(nxt)
        h, w = nxt.shape[:2]
    return levels


# ---------------------------------------------------------------------------
# DDS container (BC1/DXT1, header per the standard DDS_HEADER/
# DDS_PIXELFORMAT layout -- no extension block, nothing DX10-specific)
# ---------------------------------------------------------------------------
_DDS_MAGIC = b"DDS "


def _dds_header(width, height, n_mips, top_size):
    flags = 0x1 | 0x2 | 0x4 | 0x1000 | 0x80000                # CAPS|H|W|PF|LINEARSIZE
    caps = 0x1000                                              # DDSCAPS_TEXTURE
    if n_mips > 1:
        flags |= 0x20000                                       # DDSD_MIPMAPCOUNT
        caps |= 0x8 | 0x400000                                 # COMPLEX | MIPMAP
    hdr = struct.pack("<I", 124)
    hdr += struct.pack("<I", flags)
    hdr += struct.pack("<I", height)
    hdr += struct.pack("<I", width)
    hdr += struct.pack("<I", top_size)
    hdr += struct.pack("<I", 0)                                 # depth
    hdr += struct.pack("<I", n_mips)
    hdr += struct.pack("<11I", *([0] * 11))                     # reserved1
    hdr += struct.pack("<I", 32)                                # pf.dwSize
    hdr += struct.pack("<I", 0x4)                               # pf.DDPF_FOURCC
    hdr += b"DXT1"
    hdr += struct.pack("<5I", 0, 0, 0, 0, 0)                    # bitcount+masks
    hdr += struct.pack("<I", caps)
    hdr += struct.pack("<3I", 0, 0, 0)                          # caps2..4
    hdr += struct.pack("<I", 0)                                 # reserved2
    assert len(hdr) == 124
    return _DDS_MAGIC + hdr


def build_dds(rgb_u8, mips=True):
    """A complete BC1 `.dds` file: the header plus one block-compressed
    mip level per `_mip_chain` entry (or just the base level when
    `mips=False`), base level first."""
    rgb_u8 = np.asarray(rgb_u8, dtype=np.uint8)
    if rgb_u8.ndim == 2:
        rgb_u8 = np.repeat(rgb_u8[..., None], 3, axis=2)
    h0, w0 = rgb_u8.shape[:2]
    levels = _mip_chain(rgb_u8) if mips else [rgb_u8.astype(np.float64)]
    payload = bytearray()
    top_size = None
    for lvl in levels:
        data, nbx, nby = encode_bc1(lvl)
        if top_size is None:
            top_size = nbx * nby * 8
        payload += data
    return _dds_header(w0, h0, len(levels), top_size) + bytes(payload)


def read_dds_bc1(path):
    """`(rgb_u8, width, height)` of a BC1 DDS's TOP mip level only --
    verification/preview use, not the bake's own hot path."""
    with open(path, "rb") as fh:
        data = fh.read()
    if data[:4] != _DDS_MAGIC:
        raise ValueError("not a DDS file: %r" % (path,))
    height = struct.unpack_from("<I", data, 4 + 8)[0]
    width = struct.unpack_from("<I", data, 4 + 12)[0]
    nbx = max(1, (width + 3) // 4)
    nby = max(1, (height + 3) // 4)
    top = data[128:128 + nbx * nby * 8]
    img = decode_bc1(top, nbx, nby)
    return img[:height, :width], width, height


# ---------------------------------------------------------------------------
# The one call site both writers use
# ---------------------------------------------------------------------------
def save_soot_texture(rgb, path_stub, compress=None, mips=True):
    """Write a baked soot RGB composite (float 0..1 or uint8, `(h,w,3)`) to
    `path_stub + ".dds"` (compressed, the default) or `path_stub + ".png"`
    (`SOOT_TEX_COMPRESS=0`, or `compress=False`) and return the path
    actually written -- the caller's `Sdf.AssetPath` and its own dedup-by-
    name check both use the RETURNED path, since the extension (and
    therefore the on-disk name) changes with the gate. Idempotent, and
    writes atomically (`os.replace` from a same-directory temp file) so a
    reader can never observe a partially-written file -- several `Kit`
    processes write into the same `OUT_DIR` across a city bake."""
    arr = np.asarray(rgb)
    if arr.dtype != np.uint8:
        arr = (np.clip(arr, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)
    on = compress_enabled() if compress is None else bool(compress)
    if not on:
        from PIL import Image
        path = path_stub + ".png"
        if not os.path.exists(path):
            Image.fromarray(arr).save(path)
        return path
    path = path_stub + ".dds"
    if not os.path.exists(path):
        data = build_dds(arr, mips=mips)
        tmp = "{0}.tmp{1}".format(path, os.getpid())
        with open(tmp, "wb") as fh:
            fh.write(data)
        os.replace(tmp, path)
    return path
