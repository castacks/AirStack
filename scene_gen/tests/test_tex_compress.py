#!/usr/bin/env python3
"""test_tex_compress.py -- does the BC1 DDS encoder round-trip real-looking
soot content without visible banding, and does `save_soot_texture` actually
gate on `SOOT_TEX_COMPRESS` the way the urban-fire VRAM pass needs it to?

    python3 scene_gen/tests/test_tex_compress.py
    pytest -q scene_gen/tests/test_tex_compress.py

No `pxr`, no Kit -- pure NumPy/PIL, runs host-side in well under a second.
"""
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import tex_compress as tc            # noqa: E402


def _psnr(a, b):
    a = a.astype(np.float64)
    b = b.astype(np.float64)
    mse = float(np.mean((a - b) ** 2))
    if mse <= 1e-12:
        return 99.0
    return 10.0 * np.log10((255.0 ** 2) / mse)


def _gradient_soot(h=256, w=256, seed=0):
    """A smooth vertical soot-style darkening (the case most exposed to
    BC1 banding) over a mildly noisy brick-ish base -- closer to a real
    `sootbake_*` composite (a photographic kit base map, which has real
    spatial correlation within a 4x4 block, darkened by a smooth soot
    field) than flat colour or literal per-pixel white noise, which is an
    adversarial case no block compressor -- BC1 or otherwise -- handles
    well and no real bake ever produces. The 3x3 box blur is what supplies
    that correlation."""
    rng = np.random.default_rng(seed)
    noise = 16 * rng.standard_normal((h, w, 3))
    # cheap separable box blur (no scipy dependency) for spatial
    # correlation within a 4x4 block, the way a real photographic base
    # texture has and literal per-pixel noise does not
    blurred = np.empty_like(noise)
    for c in range(3):
        ch = noise[:, :, c]
        ch = (ch + np.roll(ch, 1, axis=0) + np.roll(ch, -1, axis=0)) / 3.0
        ch = (ch + np.roll(ch, 1, axis=1) + np.roll(ch, -1, axis=1)) / 3.0
        blurred[:, :, c] = ch
    base = np.clip(140 + blurred, 0, 255)
    row = np.linspace(0.0, 1.0, h)[:, None]
    dark = 1.0 - 0.85 * (row ** 1.5)
    out = base * dark[:, :, None]
    return np.clip(out, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------
def test_encode_decode_roundtrip_gradient():
    img = _gradient_soot()
    data, nbx, nby = tc.encode_bc1(img)
    assert len(data) == nbx * nby * 8
    dec = tc.decode_bc1(data, nbx, nby)[:img.shape[0], :img.shape[1]]
    psnr = _psnr(img, dec)
    # BC1 (4:1 colour, no alpha) on a soft, noisy gradient: real-world BC1
    # encoders land ~30-45 dB on photographic content. A regression that
    # broke the endpoint math (e.g. the row/col transpose, or the 565
    # rounding) drops this into the teens -- assert well above that.
    assert psnr > 28.0, "BC1 round-trip too lossy: %.1f dB" % psnr


def test_flat_block_is_near_exact():
    img = np.full((16, 16, 3), 91, dtype=np.uint8)
    data, nbx, nby = tc.encode_bc1(img)
    dec = tc.decode_bc1(data, nbx, nby)
    # every pixel in a flat block should land within one 565 quantization
    # step of the source -- practically exact for a mid-grey value
    assert np.max(np.abs(dec.astype(int) - 91)) <= 4


def test_non_multiple_of_4_dims_padded_and_cropped():
    img = _gradient_soot(h=130, w=67, seed=3)
    data, nbx, nby = tc.encode_bc1(img)
    assert nbx == (67 + 3) // 4 and nby == (130 + 3) // 4
    dec = tc.decode_bc1(data, nbx, nby)
    assert dec.shape[0] >= 130 and dec.shape[1] >= 67
    cropped = dec[:130, :67]
    assert cropped.shape == img.shape
    assert _psnr(img, cropped) > 25.0


def test_dds_header_round_trips_top_mip(tmp_path=None):
    import tempfile
    img = _gradient_soot(h=64, w=64, seed=1)
    blob = tc.build_dds(img, mips=True)
    assert blob[:4] == b"DDS "
    d = tempfile.mkdtemp()
    path = os.path.join(d, "t.dds")
    with open(path, "wb") as fh:
        fh.write(blob)
    dec, w, h = tc.read_dds_bc1(path)
    assert (w, h) == (64, 64)
    assert dec.shape == (64, 64, 3)
    assert _psnr(img, dec) > 28.0


def test_mip_chain_reaches_1x1_and_shrinks_each_level():
    levels = tc._mip_chain(_gradient_soot(h=32, w=32, seed=2))
    dims = [(int(lv.shape[1]), int(lv.shape[0])) for lv in levels]
    assert dims[0] == (32, 32)
    assert dims[-1] == (1, 1)
    for (w0, h0), (w1, h1) in zip(dims, dims[1:]):
        assert w1 <= w0 and h1 <= h0


def test_compression_ratio_is_8x_vs_rgba8_plus_mips():
    # BC1 base level: 0.5 bytes/texel. The uncompressed baseline this
    # whole VRAM pass measures against (`bake_vram_census.py`) is RGBA8
    # with a full mip pyramid: 4 bytes/texel * 4/3. Both scale by the same
    # 4/3 pyramid factor when mips are included, so the ratio is exactly
    # 8x at any resolution -- assert it structurally, not against a
    # hard-coded byte count that would drift if BAKE_PX ever changes.
    w = h = 256
    data, nbx, nby = tc.encode_bc1(np.zeros((h, w, 3), dtype=np.uint8))
    bc1_base = len(data)
    rgba8_base = w * h * 4
    assert abs((rgba8_base / bc1_base) - 8.0) < 1e-9


# ---------------------------------------------------------------------------
def test_save_soot_texture_env_gate(monkeypatch, tmp_path):
    img = _gradient_soot(h=32, w=32, seed=5).astype(np.float32) / 255.0
    stub = str(tmp_path / "sootbake_deadbeef")

    monkeypatch.setenv("SOOT_TEX_COMPRESS", "1")
    p_on = tc.save_soot_texture(img, stub)
    assert p_on.endswith(".dds") and os.path.exists(p_on)

    monkeypatch.setenv("SOOT_TEX_COMPRESS", "0")
    p_off = tc.save_soot_texture(img, stub)
    assert p_off.endswith(".png") and os.path.exists(p_off)
    assert p_on != p_off        # the two gates never collide on one path


def test_save_soot_texture_default_is_compressed(monkeypatch, tmp_path):
    monkeypatch.delenv("SOOT_TEX_COMPRESS", raising=False)
    img = _gradient_soot(h=16, w=16, seed=6).astype(np.float32) / 255.0
    p = tc.save_soot_texture(img, str(tmp_path / "sootbake_cafef00d"))
    assert p.endswith(".dds")


def test_save_soot_texture_is_idempotent(monkeypatch, tmp_path):
    monkeypatch.setenv("SOOT_TEX_COMPRESS", "1")
    img = _gradient_soot(h=16, w=16, seed=7).astype(np.float32) / 255.0
    stub = str(tmp_path / "sootbake_idem")
    p1 = tc.save_soot_texture(img, stub)
    mtime1 = os.path.getmtime(p1)
    p2 = tc.save_soot_texture(img, stub)
    assert p1 == p2
    assert os.path.getmtime(p2) == mtime1   # not rewritten the second time


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
