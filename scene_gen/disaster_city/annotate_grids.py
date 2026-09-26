"""Annotate video_compare.py 2x2 grids for review: a labelled reference grid on every pane and a compass.

    ~/.venvs/recon/bin/python annotate_grids.py GRID_DIR [--cols 8 --rows 6]

Reads GRID_DIR/*.jpg (tiles | summer over video | autumn, each pane under a 50 px label bar) and
GRID_DIR/cams.json, writes GRID_DIR/annotated/<name>.jpg:
  * a grid on each pane, columns A.. left to right, rows 1.. top to bottom, each cell labelled in its
    corner ("C4"), so a spot can be named as e.g. "top-right C4" -- the same cell is the same pixel
    region in all four panes (they share one camera);
  * a compass gizmo per pane: N / E / S / W as seen from that camera (the heading of its forward
    axis on the ground, world x = east, y = north), drawn as a top-down rose rotated so "up" in the
    rose is the direction the camera faces; its compass bearing (clockwise from north) and downward tilt are printed under it.
"""
import argparse, json, math
from pathlib import Path
import cv2, numpy as np

ap = argparse.ArgumentParser(); ap.add_argument("dir"); ap.add_argument("--cols", type=int, default=8); ap.add_argument("--rows", type=int, default=6)
a = ap.parse_args(); d = Path(a.dir); out = d / "annotated"; out.mkdir(exist_ok=True)
cams = {c["name"]: c for c in json.load(open(d / "cams.json"))}
BAR, GAP = 50, 10

def compass(img, cx, cy, r, heading_deg, tilt_deg):
    cv2.circle(img, (cx, cy), r + 10, (0, 0, 0), -1); cv2.circle(img, (cx, cy), r + 10, (255, 255, 255), 2)
    for lab, bearing in (("N", 90), ("E", 0), ("S", 270), ("W", 180)):          # world angle, CCW from east
        rel = math.radians(bearing - heading_deg)                              # CCW from the camera's forward
        v = np.array([-math.sin(rel), -math.cos(rel)])                         # forward = up on the image, CCW = left
        tip = (int(cx + v[0] * r), int(cy + v[1] * r))
        col = (60, 60, 255) if lab == "N" else (255, 255, 255)
        cv2.arrowedLine(img, (cx, cy), tip, col, 4 if lab == "N" else 2, tipLength=0.25)
        tp = (int(cx + v[0] * (r + 26)) - 10, int(cy + v[1] * (r + 26)) + 10)
        cv2.putText(img, lab, tp, 0, 0.9, (0, 0, 0), 5, cv2.LINE_AA); cv2.putText(img, lab, tp, 0, 0.9, col, 2, cv2.LINE_AA)
    bearing = (90 - heading_deg) % 360                                          # compass bearing: clockwise from north
    txt = f"facing {bearing:.0f} ({'N NE E SE S SW W NW'.split()[int((bearing + 22.5) // 45) % 8]}), {tilt_deg:.0f} down"
    (tw, th), _ = cv2.getTextSize(txt, 0, 0.6, 2); x1 = cx + r + 12                    # right-aligned under the rose, inside the pane
    cv2.rectangle(img, (x1 - tw - 12, cy + r + 44), (x1, cy + r + 50 + th + 8), (0, 0, 0), -1)
    cv2.putText(img, txt, (x1 - tw - 6, cy + r + 50 + th), 0, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

def grid(img, x0, y0, w, h):
    over = img.copy(); cw, ch = w / a.cols, h / a.rows
    for i in range(1, a.cols): cv2.line(over, (int(x0 + i * cw), y0), (int(x0 + i * cw), y0 + h), (0, 255, 255), 2)
    for j in range(1, a.rows): cv2.line(over, (x0, int(y0 + j * ch)), (x0 + w, int(y0 + j * ch)), (0, 255, 255), 2)
    cv2.addWeighted(over, 0.45, img, 0.55, 0, img)
    for i in range(a.cols):
        for j in range(a.rows):
            lab = f"{chr(65 + i)}{j + 1}"; p = (int(x0 + i * cw) + 6, int(y0 + j * ch) + 26)
            cv2.putText(img, lab, p, 0, 0.75, (0, 0, 0), 4, cv2.LINE_AA); cv2.putText(img, lab, p, 0, 0.75, (0, 255, 255), 2, cv2.LINE_AA)

n = 0
for f in sorted(d.glob("*.jpg")):
    if f.name.startswith("00_") or f.stem not in cams: continue
    img = cv2.imread(str(f)); H, W = img.shape[:2]
    pw, ph = (W - GAP) // 2, (H - GAP) // 2                                   # a pane incl. its label bar
    R = np.array(cams[f.stem]["R_wc"]); fwd = R[:, 2]
    heading = math.degrees(math.atan2(fwd[1], fwd[0])); tilt = math.degrees(math.asin(-fwd[2]))
    for px in (0, pw + GAP):
        for py in (0, ph + GAP):
            grid(img, px, py + BAR, pw, ph - BAR)
            compass(img, px + pw - 95, py + BAR + 95, 60, heading, tilt)
    cv2.imwrite(str(out / f.name), img, [cv2.IMWRITE_JPEG_QUALITY, 90]); n += 1
print(f"{n} grids annotated -> {out}")
