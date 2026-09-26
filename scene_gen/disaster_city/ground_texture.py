"""Super-resolve the site ortho for the ground and bake real surface detail in -> data/recon/ground_tex/ortho.<UDIM>.jpg

    ~/.venvs/recon/bin/python ground_texture.py [--grid 8]

The Google tiles hold ~15.5 cm per texel, so re-rendering the ortho finer (it is 12.5 cm) adds
nothing. Instead:
 1. Real-ESRGAN x4 (spandrel, ~/.cache/sr/RealESRGAN_x4plus.pth) on the 12.5 cm ortho, tiled with
    overlap -> 3.1 cm/px. It sharpens edges (kerbs, markings, roof lines) but paints surfaces flat;
 2. so each surface class gets the fine structure of a real material cut from the drone video
    (video_materials.py: ground_asphalt / ground_concrete / ground_dirt, ground_grass procedural),
    tiled at its physical size and applied as a multiplicative high-pass -- the colour stays the
    ortho's, only texture at < ~0.5 m is added. Classes per pixel from the ortho's own colour (soft
    masks): grass = excess green, asphalt = dark and grey, concrete = bright and grey, dirt = warm rest, asphalt = cool rest;
 3. cut into GRID x GRID UDIM tiles (1001 + col + 10 * row, row 0 = south) that ground.py maps with
    st = 8 x the old whole-ortho UVs. 17920 px on a side does not fit one texture.
"""
import argparse, json, os
import cv2, numpy as np, spandrel, torch
from _paths import R

ap = argparse.ArgumentParser(); ap.add_argument("--grid", type=int, default=8); a = ap.parse_args()
OUT = R / "ground_tex"; OUT.mkdir(exist_ok=True)
geo = json.load(open(R / "ortho_site.json")); ortho = cv2.imread(str(R / "ortho_site.png"))
S, T, PAD = 4, 256, 16; H0, W0 = ortho.shape[:2]; m_px = geo["m_per_px"] / S
net = spandrel.ModelLoader().load_from_file(os.path.expanduser("~/.cache/sr/RealESRGAN_x4plus.pth")).model.eval().cuda().half()

# material detail: tileable, high-passed to zero mean (relative contrast), at the SR pixel size
LIB = json.load(open(R / "materials/materials.json"))
def detail(name):
    t = cv2.cvtColor(cv2.imread(str(R / "materials" / f"{name}.png")), cv2.COLOR_BGR2GRAY).astype(np.float32)
    px = max(8, int(round(LIB[name]["tile_m"] / m_px))); t = cv2.resize(t, (px, px), interpolation=cv2.INTER_AREA)
    lo = cv2.GaussianBlur(np.tile(t, (3, 3)), (0, 0), px / 6)[px:2 * px, px:2 * px]          # periodic blur
    return (t - lo) / (t.mean() + 1e-6)
DET = {k: detail(f"ground_{k}") for k in ("asphalt", "concrete", "dirt", "grass")}
GAIN = {"asphalt": 1.0, "concrete": 0.8, "dirt": 1.0, "grass": 1.2}

# soft class masks from the ortho's colour (computed at 12.5 cm, upsampled per tile)
o = ortho.astype(np.float32) / 255; b, g, r = o[..., 0], o[..., 1], o[..., 2]
v, sat = o.max(-1), o.max(-1) - o.min(-1); exg = 2 * g - r - b
M = {"grass": np.clip((exg - 0.012) / 0.05, 0, 1)}                 # winter grass is pale: a low excess-green threshold
rest = 1 - M["grass"]
M["asphalt"] = rest * np.clip((0.42 - v) / 0.12, 0, 1) * np.clip((0.14 - sat) / 0.08, 0, 1)
M["concrete"] = rest * (1 - M["asphalt"]) * np.clip((v - 0.62) / 0.1, 0, 1) * np.clip((0.16 - sat) / 0.08, 0, 1)
warm = np.clip((r - b - 0.02) / 0.05, 0, 1)                                   # sand and soil are warm; shadowed asphalt is blue
left = np.clip(rest - M["asphalt"] - M["concrete"], 0, 1)
M["dirt"], M["asphalt"] = left * warm, M["asphalt"] + left * (1 - warm)
M = {k: cv2.GaussianBlur(m, (0, 0), 1.0) for k, m in M.items()}

G = a.grid; tw, th = W0 // G, H0 // G
for row in range(G):                           # row 0 = the south (bottom) strip of the ortho
    for col in range(G):
        y0, x0 = H0 - (row + 1) * th, col * tw
        ya, yb, xa, xb = max(0, y0 - PAD), min(H0, y0 + th + PAD), max(0, x0 - PAD), min(W0, x0 + tw + PAD)
        src = ortho[ya:yb, xa:xb]
        out = np.zeros(((yb - ya) * S, (xb - xa) * S, 3), np.float32)
        for ty in range(0, src.shape[0], T - 2 * PAD):                                     # SR in overlapping tiles
            for tx in range(0, src.shape[1], T - 2 * PAD):
                c = src[ty:ty + T, tx:tx + T]
                with torch.inference_mode():
                    s = net(torch.from_numpy(c[..., ::-1].copy()).permute(2, 0, 1)[None].cuda().half() / 255)[0]
                s = s.permute(1, 2, 0).float().clamp(0, 1).cpu().numpy()[..., ::-1]
                iy, ix = (PAD if ty else 0), (PAD if tx else 0)
                out[(ty + iy) * S:(ty + c.shape[0]) * S, (tx + ix) * S:(tx + c.shape[1]) * S] = s[iy * S:, ix * S:]
        out = out[(y0 - ya) * S:(y0 - ya + th) * S, (x0 - xa) * S:(x0 - xa + tw) * S]
        hh, ww = out.shape[:2]; mod = np.zeros((hh, ww), np.float32)
        gy, gx = np.mgrid[0:hh, 0:ww]; gy += y0 * S; gx += x0 * S                          # global SR pixel -> wraps each detail tile
        for k, d in DET.items():
            mk = cv2.resize(M[k][y0:y0 + th, x0:x0 + tw], (ww, hh), interpolation=cv2.INTER_LINEAR)
            mod += mk * GAIN[k] * d[gy % d.shape[0], gx % d.shape[1]]
        out = np.clip(out * (1 + mod[..., None]), 0, 1)
        cv2.imwrite(str(OUT / f"ortho.{1001 + col + 10 * row}.jpg"), (out * 255).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 92])
    print(f"row {row + 1}/{G}", flush=True)
json.dump({"grid": G, "m_per_px": m_px, "tile_px": [tw * S, th * S], "x0": geo["x0"], "y1": geo["y1"],
           "size_m": [W0 * geo["m_per_px"], H0 * geo["m_per_px"]]}, open(OUT / "ground_tex.json", "w"), indent=1)
print(f"{G * G} tiles of {tw * S} px ({m_px * 100:.1f} cm/px) -> {OUT}")
