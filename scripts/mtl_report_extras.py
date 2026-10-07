#!/usr/bin/env python3
"""Extra figures for an MTL run report: gimbal trajectory + detection reach.

    python3 scripts/mtl_report_extras.py --run-dir runs/<run_id>

Reads the per-robot ``telemetry.csv`` / ``track.json`` / ``scenario.json``, the
run's ``ground_truth.json`` and the team ``detection.json`` (written by
``analyze_mtl_run.py``), writes PNGs to ``<run-dir>/report_figures/`` and
injects them as a section at the end of ``<run-dir>/report.html`` (between
``<!-- mtl-extras:begin -->`` / ``<!-- mtl-extras:end -->``, replaced on re-run;
``analyze_mtl_run.py`` rewrites report.html, so re-run this after it).

Figures
  1. detection reach map: flown tracks, the hard sensor-reach corridor
     (horizontal distance <= sqrt(beta^2 - h^2) from the track), the ground that
     was actually seen (independent re-score: P_miss < 0.1), and a line from
     every target to the aircraft position at the look that found it;
  2. per-target distances: closest horizontal approach of any track vs the
     horizontal / slant range at the detecting look;
  3. the sensor model P(z | r) with the single-look and hard range limits, over
     the slant ranges at which the targets were actually found;
  4. gimbal trajectory per agent: boresight ground-point trace on the map, and
     the cross-track angle (planned roll + phi at the same arc length, commanded,
     measured) and boresight slant range vs time.

It also re-computes the residual belief mass independently (numpy, raster
``--res`` m, same gates and sigmoid as mtl_metrics_logger.detection) and prints
it next to detection.json's value as a cross-check. Needs numpy + matplotlib
(scipy optional: the reach corridor is skipped without it).
"""

from __future__ import annotations

import argparse
import base64
import csv
import json
import math
import re
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

# the report's own series colours (report.py --s1/--s2/--s3) and ink tokens
SER = ["#2a78d6", "#eb6834", "#1baf7a"]
INK, INK2, MUTED, GRID, AXIS, REF = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#898781"
BEGIN, END = "<!-- mtl-extras:begin -->", "<!-- mtl-extras:end -->"

plt.rcParams.update({
    "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb",
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.spines.top": False,
    "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11, "axes.titleweight": "bold",
    "axes.titlecolor": INK, "legend.frameon": False, "legend.fontsize": 9, "lines.linewidth": 1.6,
})


# --------------------------------------------------------------------------- #
# data
# --------------------------------------------------------------------------- #
def read_telemetry(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    keys = ["t", "x_world", "y_world", "z_world", "yaw", "cmd_roll", "cmd_pitch", "cmd_yaw",
            "meas_roll", "meas_pitch", "meas_yaw", "gimbal_measured", "progress_m", "slant_m"]

    def num(v):
        try:
            return float(v)
        except (TypeError, ValueError):
            return math.nan
    out = {k: np.array([num(r.get(k)) for r in rows]) for k in keys}
    order = np.argsort(out["t"], kind="stable")
    return {k: v[order] for k, v in out.items()}


def boresight(tel: dict[str, np.ndarray], ground_z: float = 0.0):
    """Same convention as detection.boresight_ground_point: pitch > 0 looks down (earth frame)."""
    use_meas = (tel["gimbal_measured"] != 0) & np.isfinite(tel["meas_pitch"])
    pitch = np.where(use_meas, tel["meas_pitch"], tel["cmd_pitch"])
    yaw = np.where(use_meas, tel["meas_yaw"], tel["cmd_yaw"])
    h = tel["z_world"] - ground_z
    bz = -np.sin(pitch)
    ok = (bz < -1e-6) & (h > 0)
    s = np.where(ok, h / np.where(ok, -bz, 1.0), np.nan)
    gx = tel["x_world"] + s * np.cos(yaw) * np.cos(pitch)
    gy = tel["y_world"] + s * np.sin(yaw) * np.cos(pitch)
    return gx, gy, s


def prior_raster(sc: dict, res: float):
    area, bel = sc["mission"]["area"], sc["airstack"]["belief"]
    half = float(area["size_m"]) / 2.0
    cn, ce = (float(v) for v in area.get("center_ned", (0.0, 0.0)))
    ys = np.arange(cn - half, cn + half + 1e-9, res)   # north = world y
    xs = np.arange(ce - half, ce + half + 1e-9, res)   # east  = world x
    g = np.zeros((len(ys), len(xs)))
    for b in bel["bumps"]:
        g += float(b.get("amplitude", 0.4)) * np.exp(-0.5 * ((ys[:, None] - b["n"]) / b["sigma_n"]) ** 2) \
            * np.exp(-0.5 * ((xs[None, :] - b["e"]) / b["sigma_e"]) ** 2)
    g = np.minimum(g, float(bel.get("belief_cap", 0.85)))
    floor = float(bel.get("base_uncertainty", 0.0))
    if floor > 0:
        g = np.maximum(g, floor)
    return xs, ys, g / g.sum()


def miss_raster(xs, ys, looks, det, tan_half):
    """Per-pixel miss product (1 = never seen); same disc gate, range gate and dt/dt_ref weight as the scorer."""
    a, b, c, beta, pout, dtr = (det["a"], det["b"], det["c"], det["beta"], det["p_out_of_range"], det["dt_ref_s"])
    res = xs[1] - xs[0]
    M = np.ones((len(ys), len(xs)))
    for px, py, pz, gx, gy, s, w in looks:
        if not (np.isfinite(s) and s <= beta and w > 0):
            continue
        r = s * tan_half
        j0, j1 = max(0, int(np.ceil((gx - r - xs[0]) / res))), min(len(xs) - 1, int(np.floor((gx + r - xs[0]) / res)))
        i0, i1 = max(0, int(np.ceil((gy - r - ys[0]) / res))), min(len(ys) - 1, int(np.floor((gy + r - ys[0]) / res)))
        if i0 > i1 or j0 > j1:
            continue
        X, Y = xs[j0:j1 + 1][None, :], ys[i0:i1 + 1][:, None]
        inside = (X - gx) ** 2 + (Y - gy) ** 2 <= r * r
        d3 = np.sqrt((X - px) ** 2 + (Y - py) ** 2 + pz ** 2)
        q = np.where(d3 > beta, 1.0 - pout, 1.0 - 1.0 / (a + np.exp(b * (d3 - c))))
        M[i0:i1 + 1, j0:j1 + 1] *= np.where(inside, np.maximum(q, 0.0) ** w, 1.0)
    return M


# --------------------------------------------------------------------------- #
# figures
# --------------------------------------------------------------------------- #
def _png(fig, path: Path) -> Path:
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    return path


def _map_axes(ax, xs, ys, prior):
    ext = [xs[0], xs[-1], ys[0], ys[-1]]
    ax.imshow(prior, origin="lower", extent=ext, cmap="Greys", alpha=0.55, interpolation="bilinear",
              vmin=0, vmax=prior.max() * 1.15)
    ax.set_xlim(ext[0], ext[1]); ax.set_ylim(ext[2], ext[3]); ax.set_aspect("equal")
    ax.set_xlabel("x = east [m]"); ax.set_ylabel("y = north [m]")
    ax.grid(False)


def fig_reach_map(path, xs, ys, prior, seen, corridor, agents, tel, det_rows, reach_h):
    fig, ax = plt.subplots(figsize=(9.2, 9.2))
    _map_axes(ax, xs, ys, prior)
    ext = [xs[0], xs[-1], ys[0], ys[-1]]
    handles = []
    if corridor is not None:
        ax.contourf(xs, ys, corridor.astype(float), levels=[0.5, 1.5], colors=["#c3c2b7"], alpha=0.35)
        ax.contour(xs, ys, corridor.astype(float), levels=[0.5], colors=[MUTED], linewidths=1.0, linestyles="--")
        handles.append(Patch(facecolor="#c3c2b7", alpha=0.5, edgecolor=MUTED, linestyle="--",
                             label=f"hard reach: ≤ {reach_h:.0f} m horizontal from the track (slant ≤ β)"))
    ax.contourf(xs, ys, seen.astype(float), levels=[0.5, 1.5], colors=["#2a78d6"], alpha=0.16)
    ax.contour(xs, ys, seen.astype(float), levels=[0.5], colors=["#2a78d6"], linewidths=1.0)
    handles.append(Patch(facecolor="#2a78d6", alpha=0.25, edgecolor="#2a78d6",
                         label="ground actually seen (P_miss < 0.1, from the measured gimbal)"))
    for k, name in enumerate(agents):
        T = tel[name]
        ax.plot(T["x_world"], T["y_world"], color=SER[k % 3], lw=2.2, solid_capstyle="round", zorder=4)
        ax.plot(T["x_world"][0], T["y_world"][0], "s", ms=7, color=SER[k % 3], mec="white", mew=1.5, zorder=5)
        handles.append(Line2D([], [], color=SER[k % 3], lw=2.2, label=f"{name} flown track"))
    for d in det_rows:
        col = SER[agents.index(d["agent"]) % 3] if d["agent"] in agents else MUTED
        if d["pos"] is not None:
            ax.plot([d["x"], d["pos"][0]], [d["y"], d["pos"][1]], color=col, lw=0.9, alpha=0.75, zorder=3)
            ax.plot(*d["pos"], "o", ms=3, color=col, zorder=4)
        ax.plot(d["x"], d["y"], "o", ms=8, color=col if d["found"] else "#d03b3b", mec="white", mew=1.5, zorder=6)
    handles.append(Line2D([], [], color=INK2, lw=0.9, marker="o", ms=7, mfc=INK2, mec="white",
                          label="target —— aircraft position at the look that found it (colour = finder)"))
    ax.set_title("Detection reach around the flown tracks")
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.07), ncol=1, fontsize=9)
    ax.set_xlim(ext[0], ext[1]); ax.set_ylim(ext[2], ext[3])
    return _png(fig, path)


def fig_target_distances(path, det_rows, reach_h, single_h, h):
    rows = sorted(det_rows, key=lambda d: d["closest_h"])
    x = np.arange(len(rows))
    fig, ax = plt.subplots(figsize=(10.5, 4.2))
    ax.axhline(reach_h, color=REF, ls="--", lw=1.2)
    ax.text(len(rows) - 0.5, reach_h + 12, f"hard limit {reach_h:.0f} m (slant = β)", ha="right", color=INK2, fontsize=9)
    ax.axhline(single_h, color=REF, ls=":", lw=1.2)
    ax.text(len(rows) - 0.5, single_h - 38, f"one look ≥ 0.9 inside {single_h:.0f} m", ha="right", color=INK2, fontsize=9)
    for k, d in enumerate(rows):
        if d["det_h"] is not None:
            ax.plot([k, k], [d["closest_h"], d["det_h"]], color=GRID, lw=2, zorder=1)
    ax.plot(x, [d["closest_h"] for d in rows], "o", ms=7, color=SER[0], mec="white", mew=1.2, zorder=3,
            label="closest approach of any flown track (horizontal)")
    ax.plot(x, [np.nan if d["det_h"] is None else d["det_h"] for d in rows], "D", ms=6, color=SER[1], mec="white",
            mew=1.2, zorder=3, label="horizontal distance at the detecting look")
    ax.set_xticks(x, [str(d["index"]) for d in rows], fontsize=7)
    ax.set_xlabel("target index (sorted by closest approach)")
    ax.set_ylabel("horizontal distance [m]")
    ax.set_ylim(0, max(reach_h * 1.08, 100))
    ax.set_title(f"How far each target was from the aircraft (altitude {h:.0f} m)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2)
    return _png(fig, path)


def fig_sensor(path, det, det_rows, h, single_r):
    a, b, c, beta, pout, thr = det["a"], det["b"], det["c"], det["beta"], det["p_out_of_range"], det["threshold"]
    r = np.linspace(h, beta * 1.12, 800)
    p = np.where(r > beta, pout, 1.0 / (a + np.exp(b * (r - c))))
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 5.6), sharex=True, gridspec_kw={"height_ratios": [3, 2]})
    ax1.plot(r, p, color=SER[0], lw=2)
    ax1.axhline(thr, color=REF, ls=":", lw=1.1); ax1.text(r[0] + 5, thr + 0.02, f"threshold {thr}", color=INK2, fontsize=9)
    for xv, lab, ls in ((single_r, f"single look ≥ {thr}: r ≤ {single_r:.0f} m", ":"), (beta, f"β = {beta:.0f} m (P → {pout:g})", "--")):
        ax1.axvline(xv, color=REF, ls=ls, lw=1.1)
        ax1.text(xv - 6, 0.08 if ls == "--" else 0.22, lab, rotation=90, ha="right", va="bottom", color=INK2, fontsize=9)
    ax1.set_ylim(-0.02, 1.02); ax1.set_ylabel("P(detect) per look")
    ax1.set_title(f"Sensor model per {det['dt_ref_s']:g} s look: 1 / (a + exp(b (r − c))), a={a:g} b={b:g} c={c:g}")
    rs = [d["det_slant"] for d in det_rows if d["det_slant"] is not None]
    ax2.hist(rs, bins=np.arange(h, beta * 1.12, 25), color=SER[1], edgecolor="#fcfcfb", linewidth=1.5)
    ax2.axvline(beta, color=REF, ls="--", lw=1.1)
    ax2.set_ylabel("targets"); ax2.set_xlabel("3-D slant range [m]")
    ax2.set_title("Slant range at the look that found each target", fontsize=10)
    fig.tight_layout()
    return _png(fig, path)


def fig_gimbal_map(path, xs, ys, prior, agents, tel, bores):
    n = len(agents)
    fig, axs = plt.subplots(1, n, figsize=(5.2 * n, 5.6), squeeze=False)
    for k, name in enumerate(agents):
        ax = axs[0][k]
        _map_axes(ax, xs, ys, prior)
        T, (gx, gy, s) = tel[name], bores[name]
        t = T["t"]
        ok = np.isfinite(gx)
        pts = np.c_[gx, gy][ok]
        seg = np.stack([pts[:-1], pts[1:]], axis=1)
        lc = LineCollection(seg, cmap="viridis", linewidths=0.6, alpha=0.9)
        lc.set_array(t[ok][:-1]); ax.add_collection(lc)
        ax.plot(T["x_world"], T["y_world"], color=SER[k % 3], lw=2.4, zorder=4)
        ax.set_title(f"{name}: boresight ground trace")
        if k:
            ax.set_ylabel("")
        fig.colorbar(lc, ax=ax, fraction=0.046, pad=0.02, label="time [s]")
    fig.suptitle("Gimbal trajectory on the ground (where the camera axis hit the ground; track in the agent colour)",
                 fontsize=11, color=INK, fontweight="bold")
    fig.tight_layout()
    return _png(fig, path)


def lateral_offset(x, y, gx, gy, yaw=None):
    """Signed cross-track offset of the boresight ground point (left of the heading = +)."""
    if yaw is None:
        yaw = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))
    return -(gx - x) * np.sin(yaw) + (gy - y) * np.cos(yaw)


def fig_gimbal_series(path, agents, tel, bores, planned, beta, roll_lim, zoom):
    n = len(agents)
    fig, axs = plt.subplots(n, 3, figsize=(15, 3.0 * n), squeeze=False,
                            gridspec_kw={"width_ratios": [3, 1.5, 1.5]})
    for k, name in enumerate(agents):
        T, (gx, gy, s) = tel[name], bores[name]
        t, x, y = T["t"], T["x_world"], T["y_world"]
        lat = lateral_offset(x, y, gx, gy)
        pl = planned.get(name)
        plat = pslant = None
        if pl is not None and np.isfinite(T["progress_m"]).any():
            plat = np.interp(T["progress_m"], pl["arc"], pl["lat"])
            pslant = np.interp(T["progress_m"], pl["arc"], pl["slant"])
        for col, (t0, t1) in enumerate(((t[0], t[-1]), zoom)):
            ax = axs[k][col]
            m = (t >= t0) & (t <= t1)
            if plat is not None:
                ax.plot(t[m], plat[m], color=MUTED, lw=1.3, label="planned (same arc length)")
            ax.plot(t[m], lat[m], color=SER[k % 3], lw=1.1, label="flown (measured gimbal)")
            ax.axhline(0, color=AXIS, lw=1)
            ax.set_xlim(t0, t1); ax.set_ylim(-1000, 1000)
            ax.set_ylabel(f"{name}\nlateral offset [m]" if col == 0 else "")
            if k == 0:
                ax.set_title("Boresight ground point, cross-track offset (left +)" if col == 0
                             else f"zoom {t0:.0f}\u2013{t1:.0f} s")
            if k == n - 1:
                ax.set_xlabel("time [s]")
        ax = axs[k][2]
        t0, t1 = zoom
        m = (t >= t0) & (t <= t1)
        ax.axhline(beta, color=REF, ls="--", lw=1.1)
        if pslant is not None:
            ax.plot(t[m], pslant[m], color=MUTED, lw=1.3)
        ax.plot(t[m], s[m], color=SER[k % 3], lw=1.1)
        over = s > beta
        mm = m & over
        ax.plot(t[mm], s[mm], ".", ms=3, color="#d03b3b")
        sat = np.abs(np.degrees(T["meas_roll"])) >= roll_lim - 0.2
        ax.text(0.02, 0.03, f"whole flight: look point beyond \u03b2 {100 * np.mean(over):.1f} % (dropped)\n"
                f"gimbal roll on the \u00b1{roll_lim:.0f}\u00b0 stop {100 * np.mean(sat):.1f} %\n"
                f"max slant flown {np.nanmax(s):.0f} m"
                + ("" if pslant is None else f", planned {np.nanmax(pslant):.0f} m"),
                transform=ax.transAxes, va="bottom", fontsize=8, color=INK2)
        ax.set_xlim(t0, t1); ax.set_ylim(0, max(1000, np.nanmax(s) * 1.05))
        ax.set_ylabel("boresight slant [m]")
        if k == 0:
            ax.set_title(f"Boresight slant range (dashed: \u03b2 = {beta:.0f} m)")
        if k == n - 1:
            ax.set_xlabel("time [s]")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.legend(handles=[Line2D([], [], color=MUTED, lw=1.5, label="planned (track.json, same arc length)"),
                        Line2D([], [], color=INK2, lw=1.5, label="flown, measured gimbal (agent colour)"),
                        Line2D([], [], color="#d03b3b", marker=".", ls="", ms=8, label="look point beyond β")],
               loc="lower center", ncol=3)
    return _png(fig, path)


# --------------------------------------------------------------------------- #
def inject(report: Path, figs: list[tuple[str, Path, str]], notes: list[str]) -> None:
    html = report.read_text(encoding="utf-8")
    html = re.sub(re.escape(BEGIN) + r".*?" + re.escape(END), "", html, flags=re.S)
    parts = [BEGIN, '<section style="max-width:1180px;margin:0 auto;padding:0 16px 48px">',
             '<div class="card" style="margin-bottom:12px"><h2>Detection reach and gimbal trajectory</h2>']
    parts += [f'<p class="notes">{n}</p>' for n in notes]
    parts.append("</div>")
    for title, p, caption in figs:
        b64 = base64.b64encode(p.read_bytes()).decode()
        parts.append(f'<div class="card" style="margin-bottom:12px"><h2>{title}</h2>'
                     f'<img alt="{title}" src="data:image/png;base64,{b64}" '
                     f'style="width:100%;height:auto;border-radius:6px;background:#fcfcfb">'
                     f'<p class="notes">{caption}</p></div>')
    parts += ["</section>", END]
    html = html.replace("</body>", "\n".join(parts) + "\n</body>", 1)
    report.write_text(html, encoding="utf-8")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-dir", type=Path, required=True)
    ap.add_argument("--res", type=float, default=10.0, help="raster for the seen region / residual check [m]")
    ap.add_argument("--zoom", type=float, nargs=2, default=None, help="time window of the zoomed gimbal plot [s]")
    ap.add_argument("--no-inject", action="store_true", help="only write the PNGs")
    args = ap.parse_args(argv)
    run = args.run_dir.resolve()
    agents = sorted(d.name for d in run.iterdir() if d.is_dir() and (d / "telemetry.csv").is_file())
    sc = json.loads(next(run / a / "scenario.json" for a in agents if (run / a / "scenario.json").is_file()).read_text())
    gt = json.loads((run / "ground_truth.json").read_text())
    dj = json.loads((run / "detection.json").read_text()) if (run / "detection.json").is_file() else {"targets": []}
    det = sc["sensor"]["detection"]
    det = {"a": det.get("a", 1.1), "b": det.get("b", 0.1), "c": det.get("c", 61.0), "beta": det.get("beta", 61.0),
           "p_out_of_range": det.get("p_out_of_range", 1e-6), "threshold": det.get("threshold", 0.9),
           "dt_ref_s": det.get("dt_ref_s", 0.1)}
    tan_half = math.tan(math.radians(float(sc["sensor"]["fov_deg"])) / 2.0)
    h = float(sc["aircraft"]["altitude_m"])
    beta = det["beta"]
    reach_h = math.sqrt(max(beta ** 2 - h ** 2, 0.0))
    # single look reaches the threshold where 1/(a+exp(b(r-c))) = thr
    arg = 1.0 / det["threshold"] - det["a"]
    single_r = min(beta, det["c"] + math.log(arg) / det["b"]) if arg > 0 else float("nan")
    single_h = math.sqrt(max(single_r ** 2 - h ** 2, 0.0)) if math.isfinite(single_r) else float("nan")

    tel = {a: read_telemetry(run / a / "telemetry.csv") for a in agents}
    bores = {a: boresight(tel[a]) for a in agents}
    planned = {}
    for a in agents:
        tp = run / a / "track.json"
        if tp.is_file():
            tr = json.loads(tp.read_text())
            s = tr["samples"]
            hx, hy, hz = tr.get("home_enu", [0.0, 0.0, 0.0])
            if all(k in s for k in ("bx_map", "by_map", "x_map", "y_map", "arc")):
                px, py = np.array(s["x_map"]) + hx, np.array(s["y_map"]) + hy
                bx, by = np.array(s["bx_map"]) + hx, np.array(s["by_map"]) + hy
                yaw = np.array(s["yaw_enu"]) if "yaw_enu" in s else None
                pz = np.array(s.get("z_map", [h] * len(px))) + hz - (np.array(s["bz_map"]) + hz if "bz_map" in s else 0.0)
                planned[a] = {"arc": np.array(s["arc"]), "lat": lateral_offset(px, py, bx, by, yaw),
                              "slant": np.sqrt((bx - px) ** 2 + (by - py) ** 2 + pz ** 2)}

    # independent residual re-score + seen region
    xs, ys, prior = prior_raster(sc, args.res)
    looks = []
    for a in agents:
        T, (gx, gy, s) = tel[a], bores[a]
        t = T["t"]
        dt = np.diff(t, append=t[-1] + (np.median(np.diff(t)) if len(t) > 1 else det["dt_ref_s"]))
        looks.append(np.c_[T["x_world"], T["y_world"], T["z_world"], gx, gy, s, dt / det["dt_ref_s"]])
    M = miss_raster(xs, ys, np.vstack(looks), det, tan_half)
    resid = float((prior * M).sum())
    seen = M < 0.1
    corridor = None
    try:
        from scipy.ndimage import distance_transform_edt
        trk = np.zeros_like(M, dtype=bool)
        for a in agents:
            j = np.clip(np.round((tel[a]["x_world"] - xs[0]) / args.res).astype(int), 0, len(xs) - 1)
            i = np.clip(np.round((tel[a]["y_world"] - ys[0]) / args.res).astype(int), 0, len(ys) - 1)
            trk[i, j] = True
        corridor = distance_transform_edt(~trk) * args.res <= reach_h
    except ImportError:
        pass

    # per-target geometry
    all_xy = np.vstack([np.c_[tel[a]["x_world"], tel[a]["y_world"]] for a in agents])
    by_idx = {t["index"]: t for t in dj.get("targets", [])}
    det_rows = []
    for k, tg in enumerate(gt["targets"]):
        x, y = float(tg["e"]), float(tg["n"])
        rec = by_idx.get(tg.get("index", k), {})
        ag, td = rec.get("responsible_agent"), rec.get("detection_time_s")
        pos = det_h = det_slant = None
        if ag in tel and td is not None:
            T = tel[ag]
            q = int(np.clip(np.searchsorted(T["t"], td + T["t"][0]), 0, len(T["t"]) - 1))
            pos = (float(T["x_world"][q]), float(T["y_world"][q]))
            det_h = math.hypot(x - pos[0], y - pos[1])
            det_slant = math.sqrt(det_h ** 2 + float(T["z_world"][q]) ** 2)
        det_rows.append({"index": tg.get("index", k), "x": x, "y": y, "agent": ag, "found": bool(rec.get("detected")),
                         "pos": pos, "det_h": det_h, "det_slant": det_slant,
                         "closest_h": float(np.min(np.hypot(all_xy[:, 0] - x, all_xy[:, 1] - y)))})

    out = run / "report_figures"
    out.mkdir(exist_ok=True)
    zoom = tuple(args.zoom) if args.zoom else (60.0, 120.0)
    roll_lim = float((sc.get("airstack") or {}).get("sim_gimbal", {}).get("roll_limit_deg", [-80, 80])[1])
    figs = [
        ("Detection reach around the flown tracks",
         fig_reach_map(out / "detection_reach_map.png", xs, ys, prior, seen, corridor, agents, tel, det_rows, reach_h),
         f"Grey: everything within {reach_h:.0f} m horizontally of a flown track = slant ≤ β = {beta:.0f} m at {h:.0f} m "
         f"altitude, the most any look could reach. Blue: the ground the measured gimbal actually covered well "
         f"(miss probability &lt; 0.1). Each target is joined to the aircraft position at the look that pushed it over "
         f"{det['threshold']}."),
        ("Per-target distances", fig_target_distances(out / "target_distances.png", det_rows, reach_h, single_h, h),
         f"A single {det['dt_ref_s']:g} s look already gives P ≥ {det['threshold']} out to {single_r:.0f} m slant "
         f"({single_h:.0f} m horizontal); a few looks reach β."),
        ("Sensor model", fig_sensor(out / "sensor_model.png", det, det_rows, h, single_r),
         "Per-look detection probability vs 3-D range, as scored (mtl_metrics_logger.detection.DetectionModel)."),
        ("Gimbal trajectory — ground trace", fig_gimbal_map(out / "gimbal_ground_trace.png", xs, ys, prior, agents, tel, bores),
         "The measured boresight ground point over time (viridis = time); the zig-zag is the cross-track sweep."),
        ("Gimbal trajectory — sweep and slant range",
         fig_gimbal_series(out / "gimbal_sweep.png", agents, tel, bores, planned, beta, roll_lim, zoom),
         "Where the camera axis hit the ground, as a signed cross-track distance from the aircraft (left +): "
         "planned (track.json boresight schedule, compared at the same arc length because the aircraft flew "
         "slightly faster than planned) vs flown (measured gimbal). Right: slant range to the boresight point; "
         "red = beyond β, where the scorer drops the whole look."),
    ]
    rep_resid = (dj.get("summary") or {}).get("residual_belief_mass")
    notes = [
        f"Sensor in this run (scenario.json): c = {det['c']:g} m, β = {beta:g} m, a = {det['a']:g}, b = {det['b']:g}, "
        f"FOV {sc['sensor']['fov_deg']:g}°, altitude {h:.0f} m. Hard reach = β slant = {reach_h:.0f} m horizontal; "
        f"one look alone reaches P ≥ {det['threshold']} within {single_r:.0f} m slant ({single_h:.0f} m horizontal).",
        f"Independent re-score of the residual belief on a {args.res:g} m raster: {resid:.5f} "
        f"(report: {rep_resid}). Ground seen with P_miss &lt; 0.1: {100 * seen.mean():.1f} % of the area.",
    ]
    print("\n".join(re.sub("&lt;", "<", n) for n in notes))
    if not args.no_inject and (run / "report.html").is_file():
        inject(run / "report.html", figs, notes)
        print(f"injected {len(figs)} figures into {run / 'report.html'}")
    print(f"wrote {out}/*.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
