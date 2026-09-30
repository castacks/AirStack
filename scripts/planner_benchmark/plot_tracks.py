#!/usr/bin/env python3
"""Trajectory maps for EVERY scenario of a benchmark, one panel per planner arm.

    python3 scripts/planner_benchmark/plot_tracks.py --bench bench/wide900 \
        --arms mtl_info_aware_r800,tigris_det_60dps,mtl_plain_r800

Each figure shows the scenario's prior (grey, darker = more belief) with, per arm, the flown
path (dark line), where the camera looked (the boresight ground points, coloured dots), the
home (square) and the arm's residual (``--metric``, default ``residual_hw``, lower is better).
The panel with the lowest residual is marked "best".

Writes ``<bench>/track_maps/<id>.png`` for every scenario that has a result for every listed
arm, plus ``<bench>/track_maps/all_tracks.pdf`` with all of them. The PDF is sorted by
the first arm's advantage over the second (``--sort adv``, default: where the second arm
wins most comes first) or by scenario id (``--sort id``). Tracks come from ``results.jsonl``,
decimated to about 25 m, so no track files need to be kept.
Needs numpy and matplotlib.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from priors import rasterize  # noqa: E402

LOOK_COLOR = "#2a78c2"   # boresight ground points
PATH_COLOR = "#1b1b1b"   # aircraft path
HOME_COLOR = "#e8a33a"


def load(bench: Path):
    idx = json.loads((bench / "index.json").read_text())
    scen = {r["id"]: r for r in idx["scenarios"]}
    res: dict[str, dict] = {}
    for line in (bench / "results.jsonl").read_text().splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        if "error" in r:
            continue
        res.setdefault(r["id"], {})[r["arm"]] = r  # last one wins
    return scen, res


def prior_of(sc: dict, res_m: float):
    a = sc["mission"]["area"]
    size = float(a["size_m"])
    cn, ce = a.get("center_ned", [0.0, 0.0])
    area = (cn - size / 2, cn + size / 2, ce - size / 2, ce + size / 2)
    bel = sc["airstack"]["belief"]
    return rasterize(bel["bumps"], float(bel.get("base_uncertainty", 0.0)), area, res=res_m,
                     cap=float(bel.get("belief_cap", 0.85)))


def figure(sc: dict, meta: dict, recs: dict, arms: list[str], metric: str, res_m: float):
    import matplotlib.pyplot as plt
    na, ea, V = prior_of(sc, res_m)
    ext = [ea[0], ea[-1], na[0], na[-1]]
    vals = {a: float(recs[a].get(metric, recs[a]["residual"])) for a in arms}
    best = min(vals.values())
    fig, axes = plt.subplots(1, len(arms), figsize=(4.3 * len(arms), 4.9), dpi=110)
    axes = np.atleast_1d(axes)
    homes = [ag["home_ned"] for ag in sc["team"]["agents"]]
    for ax, arm in zip(axes, arms):
        rr = recs[arm]
        ax.imshow(V, origin="lower", extent=ext, cmap="Greys", vmin=0, vmax=max(V.max(), 1e-12),
                  interpolation="bilinear")
        for t in (rr.get("tracks") or [rr.get("track")]):
            if not t:
                continue
            T = np.asarray(t, float)
            ok = np.isfinite(T[:, 2]) & np.isfinite(T[:, 3])
            ax.scatter(T[ok, 3], T[ok, 2], s=3, color=LOOK_COLOR, alpha=0.35, lw=0, zorder=2)
            ax.plot(T[:, 1], T[:, 0], "-", color="white", lw=2.6, zorder=3)  # halo for contrast
            ax.plot(T[:, 1], T[:, 0], "-", color=PATH_COLOR, lw=1.2, zorder=4)
        for h in homes:
            ax.plot([h[1]], [h[0]], "s", color=HOME_COLOR, ms=7, mec="black", mew=0.8, zorder=5)
        flown = rr.get("flown_m", 0.0) / max(rr.get("budget_m", 1.0), 1.0)
        tag = "  best" if vals[arm] <= best + 1e-9 else ""
        ax.set_title(f"{arm}\nresidual {vals[arm]:.3f}{tag}   (flew {100 * flown:.0f} % of budget)",
                     fontsize=9, fontweight="bold" if tag else "normal")
        ax.set_xlim(ext[0], ext[1]); ax.set_ylim(ext[2], ext[3])
        ax.set_xticks([]); ax.set_yticks([])
        for s in ax.spines.values():
            s.set_color("#999999")
    hdr = (f"{meta['id']}   family {meta['family']}   budget {meta['budget_s']:.0f} s   "
           f"altitude {meta['altitude_m']:.0f} m   home {meta['home']}   "
           f"agents {meta.get('team_size', 1)}   beta {sc['sensor']['detection']['beta']:.0f} m")
    fig.suptitle(hdr, fontsize=9.5)
    fig.text(0.5, 0.01, "grey: prior (darker = more belief)   line: aircraft path   "
             "blue dots: camera ground points   square: home   north up, east right, "
             f"{sc['mission']['area']['size_m']:.0f} m square", ha="center", fontsize=8, color="#444444")
    fig.tight_layout(rect=(0, 0.035, 1, 0.94))
    return fig


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bench", type=Path, required=True)
    ap.add_argument("--arms", required=True, help="comma-separated, one panel each (2-4)")
    ap.add_argument("--metric", default="residual_hw", help="residual_hw (default) or residual")
    ap.add_argument("--sort", choices=["adv", "id"], default="adv")
    ap.add_argument("--res", type=float, default=20.0, help="prior raster for drawing [m]")
    ap.add_argument("--out", type=Path, default=None, help="default <bench>/track_maps")
    ap.add_argument("--no-pdf", action="store_true")
    a = ap.parse_args(argv)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    arms = [x.strip() for x in a.arms.split(",") if x.strip()]
    scen, res = load(a.bench)
    out = a.out or a.bench / "track_maps"
    out.mkdir(parents=True, exist_ok=True)
    ids = [sid for sid in scen if all(x in res.get(sid, {}) for x in arms)]
    if not ids:
        raise SystemExit("no scenario has a result for every listed arm")
    m = a.metric

    def val(sid, arm):
        r = res[sid][arm]
        return float(r.get(m, r["residual"]))

    if a.sort == "adv" and len(arms) >= 2:
        ids.sort(key=lambda s: val(s, arms[0]) - val(s, arms[1]), reverse=True)
    else:
        ids.sort()
    pdf = None if a.no_pdf else PdfPages(out / "all_tracks.pdf")
    for k, sid in enumerate(ids):
        sc = json.loads((a.bench / "scenarios" / f"{sid}.json").read_text())
        fig = figure(sc, scen[sid], res[sid], arms, m, a.res)
        fig.savefig(out / f"{sid}.png")
        if pdf is not None:
            pdf.savefig(fig)
        plt.close(fig)
        if (k + 1) % 20 == 0:
            print(f"[plot_tracks] {k + 1}/{len(ids)}", flush=True)
    if pdf is not None:
        pdf.close()
    print(f"[plot_tracks] {len(ids)} scenarios -> {out}/<id>.png" + ("" if a.no_pdf else f" and {out / 'all_tracks.pdf'}"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
