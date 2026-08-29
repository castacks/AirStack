#!/usr/bin/env python3
"""texture_depth — a depth/height field for a building façade PHOTOGRAPH,
and the window grid that falls out of it.

WHAT THIS IS FOR, AND WHAT IT IS NOT FOR
-----------------------------------------
The 112 standalone building assets are photographs on flat solids. The obvious
use for a depth map is to DISPLACE the surface, and that route is closed here:
`facade_displace_bench_launch_script.py` established that OmniPBR has no
displacement input in this build (`grep -c displacement OmniPBR.mdl` is 0),
that OmniSurface has the inputs but RTX Real-Time does not tessellate for
them, and that a normal map does nothing at a silhouette. A height map cannot
push a single vertex in this renderer.

So the depth field is used for the thing it CAN do: decide where to put real
geometry. `detail/greeble.py` already builds window reveals as solid pockets,
and its one remaining problem is that the bands and piers are laid on an
ASSUMED 3.6 m grid, so they land wherever they land and double up with the
windows painted into the photograph. Reading the grid out of the photo makes
the geometry and the texture agree, which is the whole complaint.

TWO ESTIMATORS, AND WHY BOTH ARE HERE
--------------------------------------
`--method ai` runs Depth Anything V2 (small), a monocular depth model, over
the texture. It is the general answer and it is genuinely good at "which parts
of this image are further away".

`--method classical` does no learning at all: a façade photo taken square-on
is a lit wall with dark rectangles in it, so luminance IS depth to within a
constant, and a bilateral filter keeps the window edges crisp while flattening
the brick noise. On this content it is competitive with the model and it costs
milliseconds instead of a GPU.

Which to trust is an empirical question and the point of running both — the
window GRID they imply is compared directly by `--grid`, because that is the
output that actually drives geometry.

    /isaac-sim/python.sh scene_gen/tools/texture_depth.py IN.jpg OUT_DIR \\
        --method ai --grid
"""

import argparse
import json
import os


def classical_depth(img):
    """Luminance-as-depth, edge-preserving. `img` is HxWx3 uint8."""
    import cv2
    import numpy as np
    g = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # BILATERAL, NOT GAUSSIAN. A façade photo is brick and render noise over a
    # few big flat regions; a Gaussian blurs the window edge, which is the one
    # feature the whole thing rests on. Bilateral flattens the noise and keeps
    # the edge.
    s = cv2.bilateralFilter(g, 11, 60, 60).astype(np.float32) / 255.0
    # windows are DARK and RECESSED, wall is light and proud, so depth is the
    # luminance itself, contrast-stretched off its own percentiles rather than
    # min/max (one specular highlight otherwise sets the whole scale)
    lo, hi = np.percentile(s, 2.0), np.percentile(s, 98.0)
    return np.clip((s - lo) / max(hi - lo, 1e-6), 0.0, 1.0)


def ai_depth(img):
    """Depth Anything V2 (small) — relative inverse depth, normalised."""
    import numpy as np
    import torch
    from PIL import Image
    from transformers import pipeline
    dev = 0 if torch.cuda.is_available() else -1
    pipe = pipeline("depth-estimation",
                    model="depth-anything/Depth-Anything-V2-Small-hf",
                    device=dev)
    out = pipe(Image.fromarray(img))
    d = np.asarray(out["predicted_depth"] if "predicted_depth" in out
                   else out["depth"], dtype=np.float32)
    if d.ndim == 3:
        d = d[0]
    import cv2
    d = cv2.resize(d, (img.shape[1], img.shape[0]),
                   interpolation=cv2.INTER_CUBIC)
    lo, hi = np.percentile(d, 2.0), np.percentile(d, 98.0)
    return np.clip((d - lo) / max(hi - lo, 1e-6), 0.0, 1.0)


def normal_from_height(h, strength=8.0):
    import numpy as np
    gy, gx = np.gradient(h * strength)
    n = np.dstack([-gx, -gy, np.ones_like(h)])
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    return n * 0.5 + 0.5


def window_grid(depth, min_frac=0.02):
    """Bay and storey lines read out of the depth field.

    THE GRID IS THE DELIVERABLE, NOT THE DEPTH. A façade is a periodic
    structure, so the recessed (dark, low-depth) pixels project onto the two
    axes as a comb: the troughs of the column projection are the window
    columns and the troughs of the row projection are the window rows.
    Reading it this way needs no template and no assumption about bay size —
    which is the point, since guessing 3.6 m is what put the greebles out of
    step with the photograph in the first place.

    Returns {"cols": [(u0,u1)...], "rows": [(v0,v1)...]} in 0..1 texture
    coordinates.
    """
    import numpy as np
    H, W = depth.shape
    # recessed = below the halfway point between the wall and the glass
    thr = 0.5 * (np.percentile(depth, 15.0) + np.percentile(depth, 85.0))
    rec = (depth < thr).astype(np.float32)

    def _runs(profile, n, min_len):
        # a column/row is "window" where more than a third of it is recessed
        on = profile > 0.33
        out, s = [], None
        for i, v in enumerate(on):
            if v and s is None:
                s = i
            elif not v and s is not None:
                if i - s >= min_len:
                    out.append((s / float(n), i / float(n)))
                s = None
        if s is not None and n - s >= min_len:
            out.append((s / float(n), 1.0))
        return out

    cols = _runs(rec.mean(axis=0), W, max(3, int(W * min_frac)))
    rows = _runs(rec.mean(axis=1), H, max(3, int(H * min_frac)))
    return {"cols": cols, "rows": rows}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("outdir")
    ap.add_argument("--method", default="classical",
                    choices=("classical", "ai", "both"))
    ap.add_argument("--grid", action="store_true")
    a = ap.parse_args()

    import cv2
    import numpy as np
    from PIL import Image

    os.makedirs(a.outdir, exist_ok=True)
    img = np.asarray(Image.open(a.src).convert("RGB"))
    stem = os.path.splitext(os.path.basename(a.src))[0]
    report = {"src": a.src, "size": list(img.shape[:2])}

    methods = ("classical", "ai") if a.method == "both" else (a.method,)
    for m in methods:
        import time
        t0 = time.time()
        d = classical_depth(img) if m == "classical" else ai_depth(img)
        dt = time.time() - t0
        base = os.path.join(a.outdir, "{0}_{1}".format(stem, m))
        Image.fromarray((d * 255).astype("uint8")).save(base + "_depth.png")
        n = normal_from_height(d)
        Image.fromarray((n * 255).astype("uint8")).save(base + "_normal.png")
        rec = {"sec": round(dt, 3),
               "depth_p5": round(float(np.percentile(d, 5)), 3),
               "depth_p95": round(float(np.percentile(d, 95)), 3)}
        if a.grid:
            g = window_grid(d)
            rec["cols"] = len(g["cols"])
            rec["rows"] = len(g["rows"])
            rec["grid"] = g
            json.dump(g, open(base + "_grid.json", "w"))
        report[m] = rec
        print("{0:<10} {1:6.2f} s  depth p5/p95 {2:.2f}/{3:.2f}{4}".format(
            m, dt, rec["depth_p5"], rec["depth_p95"],
            "  grid {0} col x {1} row".format(rec.get("cols"), rec.get("rows"))
            if a.grid else ""))
    json.dump(report, open(os.path.join(a.outdir, stem + "_depth.json"), "w"),
              indent=1)


if __name__ == "__main__":
    main()
