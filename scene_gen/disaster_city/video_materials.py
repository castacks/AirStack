"""Tileable building materials cut from the drone video -> data/recon/materials/<name>.png + materials.json.

    ~/.venvs/recon/bin/python video_materials.py

Each photo material is a quad on a frame of a named clip (frame n of the clip sampled every 4 s
and scaled to 960 px wide, 1-based; pixel corners TL TR BR BL, picked by eye on a near-frontal view), rectified,
de-lit (divided by its own large-scale blur, so the sun gradient does not repeat on every tile)
and made seamless (cross-faded with its half-period roll). `tile_m` is how many metres one
tile covers, estimated from the surface's known size in the frame. Materials with no clean
patch in the footage (thin galvanised members, grating) are procedural in colours sampled
from the video. build_hero.py maps them on primvar `st` (metres).
"""
import json, subprocess, tempfile
import cv2, imageio_ffmpeg, numpy as np
from _paths import DATA, R

OUT = R / "materials"; OUT.mkdir(exist_ok=True)
CLIPS = DATA / "mocaps/clips"
N = 512
# name: (clip, frame n at 1/4 fps, quad, tile_m, roughness, metallic, note, mean sRGB brightness: the patch's colour is
# kept but its brightness set to this, so a wall filmed in shade does not bake the shade into its albedo; an RGB sets the mean colour)
PHOTO = {
    "concrete":   ("A03", 5, [(25, 135), (200, 135), (200, 345), (25, 345)], 3.5, 0.9, 0.0, "B01's cracked block wall, first floor, xlo half of the ylo face", 0.72),
    "steel_dark": ("A02", 8, [(60, 210), (560, 210), (560, 590), (60, 590)], 1.2, 0.7, 0.3, "the rusted I-beam web under B01's wing frame", 0.25),
    "stucco":     ("B04", 2, [(690, 352), (810, 352), (810, 398), (690, 398)], 3.0, 0.9, 0.0, "tan stucco of the hip-roofed building across the street (in shade: skylight turns it pink, so its colour is set)", (0.74, 0.66, 0.53)),
    # ground detail (ground_texture.py bakes their fine structure into the super-resolved ortho; colour stays the ortho's)
    "ground_concrete": ("B10", 5, [(40, 520), (900, 520), (900, 700), (40, 700)], 3.0, 0.9, 0.0, "the concrete apron beside B01", 0.60),
    "ground_dirt":     ("B07", 3, [(330, 345), (720, 345), (720, 450), (330, 450)], 2.0, 0.95, 0.0, "the sand round the dumpster", 0.62),
    "roof_brown": ("B04", 2, [(660, 310), (900, 310), (900, 327), (660, 327)], 3.0, 0.85, 0.0, "the same building's brown roof", 0.40),
}
# name: (base RGB sampled off the frames, noise amplitude, pattern, tile_m, roughness, metallic)
PROC = {
    "steel":     ((0.55, 0.57, 0.58), 0.05, "streak", 1.0, 0.5, 0.6),     # galvanised columns / rails, A03_05
    "grating":   ((0.45, 0.46, 0.47), 0.04, "grid", 0.5, 0.6, 0.5),       # stair treads and decks (A03): read solid at any distance
    "grating_open": ((0.45, 0.46, 0.47), 0.04, "bars", 0.5, 0.6, 0.5),    # the block's floor and roof, seen from inside (B08h): see-through
    "wood":      ((0.55, 0.42, 0.28), 0.08, "streak", 1.0, 0.9, 0.0),
    "rust":      ((0.45, 0.26, 0.16), 0.08, "blotch", 1.5, 0.8, 0.2),
    "tank_white": ((0.85, 0.85, 0.83), 0.03, "blotch", 3.0, 0.5, 0.1),
    "ground_asphalt": ((0.24, 0.24, 0.25), 0.12, "speckle", 1.0, 0.9, 0.0),  # aggregate speckle (the video's road is too oblique: it streaks)
    "ground_grass": ((0.36, 0.42, 0.22), 0.10, "blades", 1.0, 0.95, 0.0),  # no close lawn in the footage: blades in its colour
    "tank_black": ((0.10, 0.10, 0.11), 0.03, "blotch", 3.0, 0.5, 0.2),
    "metal_ribbed": ((0.48, 0.48, 0.47), 0.03, "ribs", 2.0, 0.6, 0.4),   # drill tower cladding, A04 frame 5
    "metal_grey":   ((0.42, 0.42, 0.43), 0.03, "blotch", 3.0, 0.6, 0.4),  # its flat faces
    "panel_dark":   ((0.28, 0.29, 0.29), 0.02, "blotch", 1.5, 0.6, 0.3),  # its closed window panels
    "yellow":       ((0.84, 0.77, 0.56), 0.06, "blotch", 1.5, 0.5, 0.1),  # B01's roof crane, faded cream (B09)
    "sign":         ((0.92, 0.92, 0.90), 0.01, "blotch", 1.0, 0.5, 0.0),  # fallback under the photo-baked "133" sign
}

def frame(clip, n):
    src = next(CLIPS.glob(f"{clip}_*.mp4"))
    with tempfile.TemporaryDirectory() as t:
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-v", "error", "-i", str(src), "-vf", "fps=1/4,scale=960:-2",
                        "-q:v", "2", f"{t}/%02d.jpg"], check=True)
        return cv2.imread(f"{t}/{n:02d}.jpg")

def seamless(img):
    h, w = img.shape[:2]
    ramp = lambda n: np.minimum(np.arange(n) + 0.5, n - np.arange(n) - 0.5) / (n / 2)       # 0 at the edges, 1 mid
    m = np.clip(np.outer(ramp(h), ramp(w)) * 2.5, 0, 1)[..., None]
    return img * m + np.roll(img, (h // 2, w // 2), (0, 1)) * (1 - m)

def delight(img):
    lo = cv2.GaussianBlur(img, (0, 0), N / 5)
    return np.clip(img / np.maximum(lo, 1e-3) * lo.reshape(-1, 3).mean(0), 0, 1)

rng = np.random.default_rng(0)
def noise(scale):
    n = cv2.resize(rng.standard_normal((N // scale, N // scale)).astype(np.float32), (N, N), interpolation=cv2.INTER_CUBIC)
    return seamless(n[..., None])[..., 0]

lib = {}
for name, (clip, n, quad, tile, rough, metal, note, bright) in PHOTO.items():
    img = frame(clip, n).astype(np.float32) / 255
    H = cv2.getPerspectiveTransform(np.float32(quad), np.float32([(0, 0), (N, 0), (N, N), (0, N)]))
    tex = seamless(delight(cv2.warpPerspective(img, H, (N, N), flags=cv2.INTER_CUBIC)))
    tex = tex.mean((0, 1)) + 0.6 * (tex - tex.mean((0, 1)))          # softer detail: less visible repetition
    tex = tex * (bright / tex.mean() if np.isscalar(bright) else np.array(bright[::-1]) / tex.reshape(-1, 3).mean(0))
    cv2.imwrite(str(OUT / f"{name}.png"), (np.clip(tex, 0, 1) * 255).astype(np.uint8))
    lib[name] = {"tile_m": tile, "rough": rough, "metal": metal, "source": f"{clip} frame {n} (1/4 fps): {note}"}
for name, (rgb, amp, pat, tile, rough, metal) in PROC.items():
    n = noise(64) * 0.6 + noise(8) * 0.4
    if pat == "streak": n = n * 0.5 + cv2.blur(noise(4), (1, 41)) * 0.8                  # vertical streaks
    if pat == "ribs":                                                         # light vertical ribs, uneven spacing
        g = np.zeros(N, np.float32)
        for x in np.cumsum(rng.uniform(40, 80, 12)).astype(int) % N: g[x:x + int(rng.uniform(6, 12))] = 3
        n = n + g[None, :]
    if pat == "speckle":                                                       # 1-3 px light and dark stones
        g = np.zeros((N, N), np.float32); k = 9000
        g[rng.integers(0, N, k), rng.integers(0, N, k)] = rng.choice([-2.5, 2.5], k)
        n = n * 0.3 + cv2.GaussianBlur(g, (3, 3), 0.8) * 1.5
    if pat == "blades":                                                        # short dark/light strokes, random tilt
        g = np.zeros((N, N), np.float32)
        for _ in range(6000):
            x, y, l, t = rng.integers(0, N), rng.integers(0, N), rng.integers(4, 14), rng.uniform(-0.5, 0.5)
            cv2.line(g, (int(x), int(y)), (int(x + l * np.sin(t)), int(y - l * np.cos(t))), float(rng.choice([-1.5, 1.5])), 1)
        n = n * 0.4 + seamless(cv2.GaussianBlur(g, (3, 3), 0)[..., None])[..., 0]
    if pat == "grid":
        g = np.zeros((N, N), np.float32); g[:, ::N // 16] = -3; g[::N // 16, :] = -3; n = n + cv2.blur(g, (5, 5))
    tex = np.array(rgb[::-1], np.float32)[None, None] * (1 + amp * n[..., None] * 3)
    tex = (np.clip(tex, 0, 1) * 255).astype(np.uint8)
    if pat == "bars":                                                         # press-locked grating: bearing bars every 3 cm,
        a_ = np.zeros((N, N), np.uint8); p1, p2 = N * 0.03 / tile, N * 0.10 / tile   # cross bars every 10 cm; alpha 0 between
        for x in np.arange(0, N, p1): a_[:, int(x):int(x) + max(2, int(p1 * 0.2))] = 255
        for y in np.arange(0, N, p2): a_[int(y):int(y) + max(2, int(p1 * 0.2)), :] = 255
        tex = np.dstack([tex, a_])
    cv2.imwrite(str(OUT / f"{name}.png"), tex)
    lib[name] = {"tile_m": tile, "rough": rough, "metal": metal, "source": f"procedural, colour off the video ({pat})", **({"cutout": True} if pat == "bars" else {})}
# decals: one image spanning each face of the part once (build_hero.py), cut straight from a frame -- the "133" sign
DECALS = {"sign_133": ("b01/images/A/A03_005.jpg", [(752, 608), (978, 610), (978, 750), (752, 750)], "B01's 133 sign, the white plate only")}
for name, (img_rel, quad, note) in DECALS.items():
    im = cv2.imread(str(R / img_rel)); w_, h_ = 512, round(512 * (quad[2][1] - quad[1][1]) / (quad[1][0] - quad[0][0]))
    Hm = cv2.getPerspectiveTransform(np.float32(quad), np.float32([(0, 0), (w_, 0), (w_, h_), (0, h_)]))
    cv2.imwrite(str(OUT / f"{name}.png"), cv2.warpPerspective(im, Hm, (w_, h_), flags=cv2.INTER_CUBIC))
    lib[name] = {"tile_m": 1.0, "rough": 0.5, "metal": 0.0, "decal": True, "aspect": round(w_ / h_, 3), "source": f"{img_rel}: {note}"}
json.dump(lib, open(OUT / "materials.json", "w"), indent=1)
sheet = np.concatenate([cv2.resize(cv2.imread(str(OUT / f"{k}.png")), (256, 256)) for k in lib], 1)
for i, k in enumerate(lib): cv2.putText(sheet, k, (i * 256 + 6, 20), 0, 0.6, (0, 255, 255), 2)
cv2.imwrite(str(OUT / "00_sheet.jpg"), sheet)
print(len(lib), "materials ->", OUT)
