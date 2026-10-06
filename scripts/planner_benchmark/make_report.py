#!/usr/bin/env python3
"""Turn a benchmark run into a self-contained HTML report (and a flat CSV).

    python3 scripts/planner_benchmark/make_report.py --bench bench/wide \\
        [--focus mtl_info_aware,tigris_sweep] [--out bench/wide/report.html]

    # primary-arm mode: one subject arm against several opponents
    python3 scripts/planner_benchmark/make_report.py --bench bench/wide900 \\
        --focus mtl_curve --against tigris_det_60dps,mtl_info_aware_r800 \\
        --arms mtl_curve,tigris_det_60dps,mtl_info_aware_r800 --out bench/wide900/report_curve.html \\
        --csv bench/wide900/results_curve.csv

Reads ``<bench>/index.json`` (scenario factors + prior descriptors) and ``<bench>/results.jsonl``
(one line per scenario x arm from run_benchmark.py). Writes:

* ``report.html``: click any scenario (table row, scatter dot, map card) to see every arm's trajectory on its prior; leaderboard, head-to-head of the two focus arms (win rate, "decisive" wins
  by more than --decisive in residual), where the first focus arm wins most (by prior family,
  budget, altitude, home, and by binned prior descriptors), a family x budget heat map, the
  anytime curves, a scatter of every scenario, maps of the most and least decisive scenarios
  (prior + both tracks), and the full table. Opens offline; no network needed.
* ``results_wide.csv`` (``--csv``): one row per scenario, every arm's residual and every descriptor,
  for your own analysis (pandas, a spreadsheet, ...).

``--focus A,B`` compares A against B. ``--focus A --against B,C`` (primary-arm mode) makes A the
subject and repeats the head-to-head sections (tiles, advantage by factor and descriptor quartile,
family x budget heat map, scatter, most / least decisive maps) against EACH opponent; the
leaderboard, anytime curves and full table always show every arm.

Needs numpy; matplotlib (optional) draws the scenario maps.
"""

from __future__ import annotations

import argparse
import base64
import csv
import io
import json
import math
import statistics as st
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from priors import rasterize  # noqa: E402

DESCRIPTORS = [("effective_area_km2", "Effective prior area [km²]"),
               ("budget_over_spread", "Budget ÷ prior spread"),
               ("mass_within_half_budget", "Prior mass within half the budget of home"),
               ("mean_dist_from_home_m", "Mean prior distance from home [m]"),
               ("peaks_25pct", "Peaks (blocks > 25 % of max, connected)"),
               ("gini", "Concentration (Gini of cell mass)"),
               ("cells_for_90pct", "Cells holding 90 % of the mass")]
FACTORS = [("family", "Prior family"), ("budget_s", "Budget [s]"), ("altitude_m", "Altitude [m]"), ("home", "Home"),
           ("team_size", "Agents")]


def load(bench: Path):
    idx = json.loads((bench / "index.json").read_text())
    scen = {r["id"]: r for r in idx["scenarios"]}
    res = defaultdict(dict)
    errors = []
    for line in (bench / "results.jsonl").read_text().splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        if "error" in r:
            errors.append(r)
            continue
        res[r["id"]][r["arm"]] = r  # last one wins
    arms_meta = json.loads((bench / "arms.json").read_text()) if (bench / "arms.json").is_file() else {}
    return idx, scen, res, errors, arms_meta


def quantile_bins(vals, k=4):
    qs = np.quantile(vals, np.linspace(0, 1, k + 1))
    qs = np.unique(qs)
    return qs


def fmt_bin(lo, hi):
    def f(v):
        return f"{v:.3g}" if abs(v) < 1000 else f"{v:.0f}"
    return f"{f(lo)}–{f(hi)}"


def group_stats(rows, key_fn, A, B, decisive):
    g = defaultdict(list)
    for r in rows:
        g[key_fn(r)].append(r)
    out = []
    for k, v in g.items():
        d = [x[B] - x[A] for x in v]  # > 0: A searched more
        out.append({"key": k, "n": len(v), "a": st.mean(x[A] for x in v), "b": st.mean(x[B] for x in v),
                    "adv": st.mean(d), "win": sum(x > 1e-4 for x in d) / len(v),
                    "decisive": sum(x > decisive for x in d) / len(v),
                    "rel": st.mean((x[B] - x[A]) / x[B] for x in v if x[B] > 1e-3) if any(x[B] > 1e-3 for x in v) else 0.0})
    return out


def scenario_png(sc_path: Path, recs: dict, arms: list[str], colors: list[str], res=25.0):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return None
    sc = json.loads(sc_path.read_text())
    a = sc["mission"]["area"]
    size = float(a["size_m"])
    cn, ce = a.get("center_ned", [0.0, 0.0])
    area = (cn - size / 2, cn + size / 2, ce - size / 2, ce + size / 2)
    bel = sc["airstack"]["belief"]
    na, ea, V = rasterize(bel["bumps"], float(bel.get("base_uncertainty", 0.0)), area, res=res,
                          cap=float(bel.get("belief_cap", 0.85)))
    fig, ax = plt.subplots(figsize=(3.2, 3.2), dpi=90)
    ax.imshow(V, origin="lower", extent=[ea[0], ea[-1], na[0], na[-1]], cmap="magma", interpolation="bilinear")
    for arm, col, ls in zip(arms, colors, ["-", "--", ":"]):
        rr = recs.get(arm) or {}
        for t in (rr.get("tracks") or [rr.get("track")]):
            if t:
                ax.plot([p[1] for p in t], [p[0] for p in t], ls, color=col, lw=1.6)
    h = sc["team"]["agents"][0]["home_ned"]
    ax.plot([h[1]], [h[0]], "s", color="white", ms=5, mec="black")
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_xlim(ea[0], ea[-1]); ax.set_ylim(na[0], na[-1])
    fig.tight_layout(pad=0.1)
    buf = io.BytesIO()
    fig.savefig(buf, format="png")
    plt.close(fig)
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()


def _png_gray(img: np.ndarray) -> str:
    """8-bit grayscale PNG as a data URI, without matplotlib (row 0 = top)."""
    import struct
    import zlib
    h, w = img.shape
    raw = b"".join(b"\x00" + img[i].astype(np.uint8).tobytes() for i in range(h))

    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))
    return "data:image/png;base64," + base64.b64encode(png).decode()


def _rdp(pts: np.ndarray, tol: float) -> list[int]:
    """Indices kept by Ramer-Douglas-Peucker on an (N, 2) polyline (iterative)."""
    n = len(pts)
    if n < 3:
        return list(range(n))
    keep = np.zeros(n, bool)
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = pts[i], pts[j]
        ab = b - a
        L = float(np.hypot(*ab))
        seg = pts[i + 1:j] - a
        d = np.abs(ab[0] * seg[:, 1] - ab[1] * seg[:, 0]) / L if L > 1e-9 else np.hypot(seg[:, 0], seg[:, 1])
        k = int(np.argmax(d))
        if d[k] > tol:
            m = i + 1 + k
            keep[m] = True
            stack += [(i, m), (m, j)]
    return list(np.nonzero(keep)[0])


def scene_payload(sc_path: Path, recs: dict, arms: list[str], metric: str, px: int = 125) -> dict | None:
    """What the report's scene viewer draws for one scenario: the prior as a small grey PNG
    (darker = more belief), the area, homes, and per arm the simplified path (4 m RDP), the
    camera ground points (every other decimated sample), the score and budget use."""
    if not sc_path.is_file():
        return None
    sc = json.loads(sc_path.read_text())
    a = sc["mission"]["area"]
    size = float(a["size_m"])
    cn, ce = a.get("center_ned", [0.0, 0.0])
    area = (cn - size / 2, cn + size / 2, ce - size / 2, ce + size / 2)
    bel = sc["airstack"]["belief"]
    na, ea, V = rasterize(bel["bumps"], float(bel.get("base_uncertainty", 0.0)), area, res=size / px,
                          cap=float(bel.get("belief_cap", 0.85)))
    v = V / max(float(V.max()), 1e-12)
    img = np.clip(255.0 * (1.0 - v ** 0.8), 0, 255)[::-1]  # north up
    out = {"area": [area[0], area[1], area[2], area[3]], "prior": _png_gray(img),
           "homes": [[round(float(x), 1) for x in ag["home_ned"]] for ag in sc["team"]["agents"]],
           "beta": float(sc["sensor"]["detection"]["beta"]), "arms": {}}
    for arm in arms:
        r = recs.get(arm)
        if not r:
            continue
        paths, looks = [], []
        for t in (r.get("tracks") or [r.get("track")]):
            if not t:
                continue
            T = np.asarray(t, float)
            idx = _rdp(T[:, :2], 4.0)
            paths.append([int(round(x)) for k in idx for x in (T[k, 0], T[k, 1])])
            L = T[::2, 2:4]
            L = L[np.isfinite(L).all(axis=1)]
            looks.append([int(round(x)) for x in L.ravel()])
        out["arms"][arm] = {"v": float(r.get(metric, r["residual"])), "planned": float(r["residual"]),
                            "flown": float(r.get("flown_m", 0.0)), "budget": float(r.get("budget_m", 0.0)),
                            "paths": paths, "looks": looks}
    return out


def focus_analysis(rows, A, B, decisive, n_maps):
    """Head to head of A against B over the analysed rows: win / decisive rates, the mean
    advantage (B's residual - A's) by factor and descriptor quartile, family x budget, the most
    and least decisive scenarios."""
    d = [r[B] - r[A] for r in rows]
    by = {}
    for key, lab in FACTORS:
        by[key] = {"label": lab, "groups": sorted(group_stats(rows, lambda r, k=key: r[k], A, B, decisive),
                                                   key=lambda g: -g["adv"])}
    for key, lab in DESCRIPTORS:
        vals = np.array([r[key] for r in rows], float)
        qs = quantile_bins(vals, 4)
        if len(qs) < 2:
            continue

        def binner(r, qs=qs, key=key):
            i = int(np.clip(np.searchsorted(qs, r[key], side="right") - 1, 0, len(qs) - 2))
            return f"{i}:{fmt_bin(qs[i], qs[i + 1])}"
        gs = group_stats(rows, binner, A, B, decisive)
        gs.sort(key=lambda g: int(g["key"].split(":")[0]))
        for g in gs:
            g["key"] = g["key"].split(":", 1)[1]
        by[key] = {"label": lab, "groups": gs, "binned": True}
    # family x budget
    fams = sorted({r["family"] for r in rows})
    buds = sorted({r["budget_s"] for r in rows})
    heat = []
    for fm in fams:
        row = []
        for bu in buds:
            v = [r[B] - r[A] for r in rows if r["family"] == fm and r["budget_s"] == bu]
            row.append({"adv": st.mean(v) if v else None, "n": len(v)})
        heat.append(row)
    top = sorted(rows, key=lambda r: -(r[B] - r[A]))
    return {"A": A, "B": B, "n": len(rows), "adv_mean": st.mean(d), "adv_median": st.median(d),
             "win": sum(x > 1e-4 for x in d) / len(d), "loss": sum(x < -1e-4 for x in d) / len(d),
             "decisive": sum(x > decisive for x in d) / len(d),
             "decisive_loss": sum(x < -decisive for x in d) / len(d),
             "rel": st.mean((r[B] - r[A]) / r[B] for r in rows if r[B] > 1e-3),
             "by": by, "heat": {"fams": fams, "buds": buds, "cells": heat},
             "top": [r["id"] for r in top[:n_maps]], "bottom": [r["id"] for r in top[::-1][:max(4, n_maps // 2)]]}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bench", type=Path, required=True)
    ap.add_argument("--focus", default="mtl_info_aware,tigris_sweep",
                    help="A,B: where does A beat B; or just A together with --against")
    ap.add_argument("--against", default=None,
                    help="primary-arm mode: comma-separated opponents of the --focus arm, one head-to-head each")
    ap.add_argument("--csv", type=Path, default=None, help="CSV path (default <bench>/results_wide.csv)")
    ap.add_argument("--decisive", type=float, default=0.05, help="residual margin that counts as decisive")
    ap.add_argument("--metric", default="residual_hw",
                    help="residual_hw (plan flown through the common slew-limited gimbal, default) or residual (as planned)")
    ap.add_argument("--arms", default=None,
                    help="comma-separated arms to include (default: every arm in results.jsonl); "
                         "a scenario is analysed only if all of them have a result")
    ap.add_argument("--no-viewer", action="store_true",
                    help="leave out the click-a-scenario trajectory viewer (smaller file)")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--maps", type=int, default=12, help="scenario maps per side (most / least decisive)")
    ap.add_argument("--min-residual", type=float, default=0.01,
                    help="drop scenarios where EVERY arm is below this (saturated: all planners cleared the prior)")
    a = ap.parse_args(argv)
    idx, scen, res, errors, arms_meta = load(a.bench)
    fa = [x.strip() for x in a.focus.split(",") if x.strip()]
    if a.against:
        if len(fa) != 1:
            raise SystemExit("--against needs a single --focus arm")
        A, opps = fa[0], [x.strip() for x in a.against.split(",") if x.strip()]
    else:
        if len(fa) != 2:
            raise SystemExit("--focus needs A,B (or one arm plus --against)")
        A, opps = fa[0], [fa[1]]
    B = opps[0]
    arm_names = sorted({arm for v in res.values() for arm in v})
    if a.arms:
        want = [x.strip() for x in a.arms.split(",") if x.strip()]
        missing = [x for x in want + [A] + opps if x not in arm_names]
        if missing:
            raise SystemExit(f"--arms: no results for {missing}")
        arm_names = sorted(set(want) | {A} | set(opps))
    head = [A] + [x for x in opps if x != A]
    order = [x for x in head if x in arm_names] + [x for x in arm_names if x not in head]

    # complete rows: every arm present
    rows = []
    for sid, rec in res.items():
        if sid not in scen or any(x not in rec for x in order):
            continue
        if any(a.metric not in rec[x] for x in order):
            continue
        r = dict(scen[sid])
        r.setdefault("team_size", 1)
        for arm in order:
            r[arm] = rec[arm][a.metric]
            r[arm + "__planned"] = rec[arm]["residual"]
            r[arm + "__hw"] = rec[arm].get("residual_hw")
            r[arm + "__nonudge"] = rec[arm].get("residual_hw_nonudge")
            r[arm + "__openloop"] = rec[arm].get("residual_hw_openloop")
            r[arm + "__flown"] = rec[arm]["flown_m"]
            r[arm + "__budget"] = rec[arm].get("budget_m") or r["budget_m"]
            r[arm + "__s"] = rec[arm].get("plan_s")
            r[arm + "__curve"] = rec[arm].get("curve_hw" if a.metric == "residual_hw" else "curve")
        ia = next(((rec.get(x) or {}).get("info_aware") for x in ["mtl_info_aware"] + order
                   if (rec.get(x) or {}).get("info_aware")), None) or {}
        r["ia_chosen"] = (ia.get("chosen") or "").split("+")[0].split("~")[0]
        rows.append(r)
    if not rows:
        raise SystemExit("no scenario has results for every arm yet")
    rows.sort(key=lambda r: r["id"])
    n_complete = len(rows)
    all_rows = list(rows)
    saturated = [r["id"] for r in rows if max(r[x] for x in order) < a.min_residual]
    rows = [r for r in rows if max(r[x] for x in order) >= a.min_residual]
    if not rows:
        raise SystemExit("every complete scenario is saturated (all arms below --min-residual)")

    # CSV
    out_csv = a.csv or (a.bench / "results_wide.csv")
    cols = ["id", "family", "budget_s", "altitude_m", "home", "team_size", "n_bumps", "n_cells"] + [d for d, _ in DESCRIPTORS] + \
        ["entropy", "spread_m", "cells_for_50pct"] + order + [f"{x}__planned" for x in order] + \
        [f"{x}__hw" for x in order] + [f"{x}__nonudge" for x in order] + [f"{x}__openloop" for x in order] + [f"{x}__flown" for x in order] + \
        [f"{x}__s" for x in order] + ["ia_chosen"]
    with out_csv.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for r in all_rows:
            w.writerow([r.get(c) for c in cols])

    # leaderboard
    board = []
    for arm in order:
        v = [r[arm] for r in rows]
        ranks = []
        for r in rows:
            srt = sorted(order, key=lambda x: r[x])
            ranks.append(srt.index(arm) + 1)
        board.append({"arm": arm, "mean": st.mean(v), "median": st.median(v), "rank": st.mean(ranks),
                      "best": sum(1 for r in rows if min(order, key=lambda x: r[x]) == arm),
                      "flown_frac": st.mean(r[arm + "__flown"] / r[arm + "__budget"] for r in rows),
                      "plan_s": st.mean((r[arm + "__s"] or 0) for r in rows),
                      "plan_s_median": st.median((r[arm + "__s"] or 0) for r in rows),
                      "plan_s_max": max((r[arm + "__s"] or 0) for r in rows),
                      "planned": st.mean(r[arm + "__planned"] for r in rows),
                      "openloop": (st.mean(r[arm + "__openloop"] for r in rows)
                                   if all(r[arm + "__openloop"] is not None for r in rows) else None)})
    pair = {}
    for x in order:
        for y in order:
            if x != y:
                pair[x + "|" + y] = sum(1 for r in rows if r[x] < r[y] - 1e-4) / len(rows)

    # focus analysis: the subject A against every opponent
    focuses = [focus_analysis(rows, A, Bx, a.decisive, a.maps) for Bx in opps if A in order and Bx in order]
    focus = focuses[0] if focuses else None
    # anytime curves
    curves = {}
    for arm in order:
        cs = [r[arm + "__curve"] for r in rows if r.get(arm + "__curve")]
        if cs:
            curves[arm] = [st.mean(c[i] for c in cs) for i in range(len(cs[0]))]

    # maps
    # per opponent: the subject solid in the first colour, the opponent dashed in its own colour
    # (the same colour the arm has everywhere else in the report)
    maps = {}
    pal = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
    for fc in focuses:
        cB = pal[order.index(fc["B"]) % len(pal)]
        fc["maps"] = {}
        for sid in fc["top"] + fc["bottom"]:
            img = scenario_png(a.bench / "scenarios" / f"{sid}.json", res[sid], [A, fc["B"]], [pal[0], cB])
            if img:
                fc["maps"][sid] = img

    scenes = {}
    if not a.no_viewer:
        for r in rows:
            sp = scene_payload(a.bench / "scenarios" / f"{r['id']}.json", res[r["id"]], order, a.metric)
            if sp:
                scenes[r["id"]] = sp

    slim_rows = [{k: r[k] for k in ["id", "family", "budget_s", "altitude_m", "home", "n_cells", "ia_chosen"]
                  + [d for d, _ in DESCRIPTORS] + order} for r in rows]
    for sr, r in zip(slim_rows, rows):
        sr["params"] = r.get("params")
    data = {"bench": str(a.bench), "n_scen": len(idx["scenarios"]), "n_rows": len(rows), "order": order,
            "n_complete": n_complete, "n_saturated": len(saturated), "min_residual": a.min_residual,
            "metric": a.metric,
            "board": board, "pair": pair, "focus": focus, "focuses": focuses, "curves": curves, "rows": slim_rows,
            "errors": [{"id": e["id"], "arm": e["arm"], "error": e["error"][:300]} for e in errors[:50]],
            "n_errors": len(errors), "decisive": a.decisive, "maps": maps, "scenes": scenes,
            "arms": arms_meta.get("arms", []), "spec": idx.get("spec", {})}
    html = TEMPLATE.replace("__DATA__", json.dumps(data, default=float).replace("</", "<\\/"))
    out = a.out or (a.bench / "report.html")
    out.write_text(html, encoding="utf-8")
    print(f"[make_report] {n_complete} complete scenarios ({len(saturated)} saturated, left out), {len(errors)} errors "
          f"-> {out} (+ {out_csv.name}, all complete scenarios)")
    for fc in focuses:
        print(f"  {A} vs {fc['B']}: better in {100 * fc['win']:.0f} %, worse in {100 * fc['loss']:.0f} %, "
              f"decisively better (> {a.decisive}) in {100 * fc['decisive']:.0f} % (decisively worse in "
              f"{100 * fc['decisive_loss']:.0f} %); mean advantage {fc['adv_mean']:+.4f} (mean per-scenario "
              f"reduction of {fc['B']}'s residual: {100 * fc['rel']:+.0f} %)")
    return 0


TEMPLATE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Planner Benchmark Report</title>
<style>
/* one reading column; figures full width; every table scrolls inside its own box */
:root{--bg:#f6f6f3;--surface:#fff;--ink:#141613;--ink2:#4b5049;--muted:#80867d;--rule:#dcdfd7;--grid:#ebece6;
 --s1:#2a78d6;--s2:#eb6834;--s3:#1baf7a;--s4:#eda100;--pos:#256abf;--neg:#d0473e;--mid:#f0efec;
 --body:system-ui,-apple-system,"Segoe UI",sans-serif;--mono:ui-monospace,"SF Mono",Menlo,monospace}
@media (prefers-color-scheme:dark){:root{--bg:#131512;--surface:#1b1e1a;--ink:#f1f2ed;--ink2:#c0c4ba;--muted:#8b9187;--rule:#30342e;
 --grid:#262a25;--s1:#3987e5;--s2:#d95926;--s3:#199e70;--s4:#c98500;--pos:#5598e7;--neg:#e66767;--mid:#383835;color-scheme:dark}}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 var(--body)}
.wrap{max-width:1080px;margin:0 auto;padding:28px 18px 60px}
h1{font-size:28px;margin:0 0 6px}h2{font-size:18px;margin:0 0 6px}h3{font-size:14px;margin:0 0 6px;color:var(--ink2)}
.sub{color:var(--ink2);margin:0}.eyebrow{font:12px var(--mono);letter-spacing:.08em;text-transform:uppercase;color:var(--muted);margin:0 0 6px}
section{margin-top:34px;display:grid;gap:10px}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:10px}
.tile{background:var(--surface);border:1px solid var(--rule);border-radius:8px;padding:12px 14px}
.tile .v{font:500 24px var(--mono)}.tile .l{font-size:13px;color:var(--ink2)}
.card{background:var(--surface);border:1px solid var(--rule);border-radius:8px;padding:14px;min-width:0}
.grid2{display:grid;grid-template-columns:repeat(auto-fit,minmax(460px,1fr));gap:12px}
.tw{overflow:auto;border:1px solid var(--rule);border-radius:8px;background:var(--surface)}
table{border-collapse:collapse;width:100%;font-size:13px;font-variant-numeric:tabular-nums}
th,td{padding:6px 9px;border-bottom:1px solid var(--grid);text-align:right;white-space:nowrap}
th:first-child,td:first-child{text-align:left}th{position:sticky;top:0;background:var(--surface);color:var(--ink2);cursor:pointer}
.note{color:var(--ink2);font-size:13px;max-width:78ch;margin:0}
svg{display:block;width:100%;height:auto}svg text{fill:var(--ink2);font:11px var(--mono)}
.legend{display:flex;flex-wrap:wrap;gap:12px;font-size:12px;color:var(--ink2)}.legend i{display:inline-block;width:11px;height:11px;border-radius:2px;margin-right:5px;vertical-align:-1px}
.maps{display:grid;grid-template-columns:repeat(auto-fill,minmax(230px,1fr));gap:10px}
.map{background:var(--surface);border:1px solid var(--rule);border-radius:8px;padding:8px;font-size:12px}
.map img{width:100%;border-radius:4px}.map b{font-family:var(--mono)}
select{font:inherit;padding:4px 8px;border-radius:6px;border:1px solid var(--rule);background:var(--surface);color:var(--ink)}
#tip{position:fixed;pointer-events:none;display:none;background:var(--surface);border:1px solid var(--rule);border-radius:6px;padding:7px 9px;font-size:12px;box-shadow:0 4px 14px rgba(0,0,0,.15);max-width:280px;z-index:9}
.pos{color:var(--pos)}.neg{color:var(--neg)}
/* scene viewer */
:root{--halo:#ffffff;--prior-filter:none}
@media (prefers-color-scheme:dark){:root{--halo:#0c0d0b;--prior-filter:invert(1)}}
.clickable{cursor:pointer}tr.clickable:hover td{background:var(--mid)}.map.clickable:hover{border-color:var(--muted)}
dialog#scene{border:1px solid var(--rule);border-radius:10px;background:var(--surface);color:var(--ink);padding:0;width:min(1100px,calc(100vw - 32px));max-height:calc(100vh - 32px)}
dialog#scene::backdrop{background:rgba(0,0,0,.45)}
.sv-head{display:flex;flex-wrap:wrap;align-items:center;gap:8px 14px;padding:12px 16px;border-bottom:1px solid var(--rule);position:sticky;top:0;background:var(--surface);z-index:2}
.sv-head h2{margin:0;font:600 16px var(--mono)}.sv-head .facts{color:var(--ink2);font-size:13px}.sv-head .sp{flex:1}
.sv-body{padding:12px 16px 16px;display:grid;gap:12px}
.sv-ctrl{display:flex;flex-wrap:wrap;gap:8px 16px;font-size:13px;color:var(--ink2);align-items:center}
.sv-ctrl label{display:inline-flex;align-items:center;gap:6px;cursor:pointer}
.sv-maps{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr))}
.sv-maps.overlay{grid-template-columns:minmax(0,640px);justify-content:center}
.sv-map{min-width:0}.sv-map h3{margin:0 0 4px;font:12px var(--mono);color:var(--ink2)}
.sv-map svg{border:1px solid var(--rule);border-radius:6px;background:#fff;touch-action:none}
.sv-map image{filter:var(--prior-filter)}
button{font:inherit;font-size:13px;padding:4px 10px;border-radius:6px;border:1px solid var(--rule);background:var(--surface);color:var(--ink);cursor:pointer}
button:hover{border-color:var(--muted)}
.swatch{display:inline-block;width:26px;vertical-align:middle}.swatch svg{display:inline-block;width:26px;height:6px}
.toc{display:flex;flex-wrap:wrap;gap:6px 14px;margin-top:16px;font-size:13px;color:var(--ink2)}.toc a{color:var(--s1)}
.h2h{scroll-margin-top:12px}
</style></head><body><div class="wrap" id="app"></div><div id="tip"></div><dialog id="scene" aria-label="Scenario trajectories"></dialog>
<script>
const D=__DATA__;
const app=document.getElementById("app"),tip=document.getElementById("tip");
const NS="http://www.w3.org/2000/svg",css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
function el(t,a,p,txt){const e=document.createElement(t);for(const k in (a||{}))e.setAttribute(k,a[k]);if(txt!=null)e.textContent=txt;if(p)p.appendChild(e);return e;}
function s(t,a,p){const e=document.createElementNS(NS,t);for(const k in a)e.setAttribute(k,a[k]);if(p)p.appendChild(e);return e;}
const f3=v=>v==null?"—":(+v).toFixed(3),f4=v=>v==null?"—":(+v).toFixed(4),pct=v=>(100*v).toFixed(0)+" %",sg=v=>(v>=0?"+":"−")+Math.abs(v).toFixed(3);
function showTip(e,h){tip.innerHTML=h;tip.style.display="block";tip.style.left=Math.min(e.clientX+14,innerWidth-290)+"px";tip.style.top=Math.min(e.clientY+14,innerHeight-120)+"px";}
function hide(){tip.style.display="none";}
const COL=["--s1","--s2","--s3","--s4"],col=a=>css(COL[Math.max(0,D.order.indexOf(a))%COL.length]);
function table(parent,head,rows,cls,onRow){const tw=el("div",{class:"tw"},parent);const t=el("table",{},tw);const tr=el("tr",{},el("thead",{},t));
 head.forEach((h,i)=>{const th=el("th",{},tr,h);th.onclick=()=>sortBy(i);});const tb=el("tbody",{},t);let dir=1;
 function fill(rs){tb.innerHTML="";if(onRow)onRow.order=rs.map(r=>r[0]);rs.forEach(r=>{const x=el("tr",{},tb);if(onRow&&D.scenes[r[0]]){x.className="clickable";x.onclick=()=>onRow(r[0],onRow.order);}r.forEach((v,j)=>{const td=el("td",{},x);if(v&&v.html!=null){td.innerHTML=v.html}else td.textContent=v;});});}
 function sortBy(i){dir=-dir;const key=r=>{const v=r[i];const n=parseFloat(v&&v.sort!=null?v.sort:v);return isNaN(n)?String(v&&v.sort!=null?v.sort:v):n;};rows.sort((a,b)=>key(a)>key(b)?dir:key(a)<key(b)?-dir:0);fill(rows);}
 fill(rows);return t;}

// header
el("p",{class:"eyebrow"},app,"AirStack planner benchmark");
el("h1",{},app,(D.focuses&&D.focuses.length>1?D.focuses[0].A+" against "+D.focuses.map(f=>f.B).join(" and ")+": r":"R")+"esidual belief across "+D.n_rows+" scenarios");
el("p",{class:"sub"},app,(D.metric==="residual_hw"?"Residual belief = P(target missed), lower is better, of each plan flown through the SAME slew-limited gimbal (120°/s per axis, ±80° roll, the follower's pointing law); ":"Planned residual belief = P(target missed), lower is better; ")+"every plan scored with the same model. "+
 D.n_complete+" of "+D.n_scen+" scenarios have results for every arm; "+D.n_saturated+" of them are left out because every arm cleared the prior (all residuals below "+D.min_residual+")"+
 (D.n_errors?"; "+D.n_errors+" failed plans (listed at the end)":"")+". Source: "+D.bench);

// primary-arm mode: D.focuses = one head-to-head per opponent of the subject arm (old --focus A,B: one)
const FS=D.focuses||(D.focus?[D.focus]:[]),F=FS[0]||null,SUBJ=F?F.A:null;
const DASH=["","8 5","2 4","10 4 2 4","1 3"],dashOf=a=>DASH[Math.max(0,D.order.indexOf(a))%DASH.length];
function swatch(parent,a){const sw=el("span",{class:"swatch"},parent);const svg=s("svg",{viewBox:"0 0 26 6",width:26,height:6},sw);
 s("line",{x1:1,x2:25,y1:3,y2:3,stroke:col(a),"stroke-width":2.5,"stroke-dasharray":dashOf(a)},svg);return sw;}
const byId=Object.fromEntries(D.rows.map(r=>[r.id,r]));
if(FS.length>1){const nav=el("nav",{class:"toc"},app);el("span",{},nav,"Head to head:");
 FS.forEach((fc,i)=>{const x=el("a",{href:"#h2h"+i},nav,fc.A+" vs "+fc.B);});el("a",{href:"#board"},nav,"leaderboard");el("a",{href:"#anytime"},nav,"anytime");el("a",{href:"#all"},nav,"all scenarios");}

// head-to-head tiles, one row per opponent (summary up front)
if(F){const sec=el("section",{},app);el("p",{class:"eyebrow"},sec,FS.length>1?"Subject: "+SUBJ+" against "+FS.length+" opponents":"Head to head");
 FS.forEach((fc,i)=>{if(FS.length>1)el("h3",{},sec,"vs "+fc.B);else el("h2",{},sec,fc.A+" vs "+fc.B);tiles(sec,fc);});}
function tiles(sec,fc){const t=el("div",{class:"tiles"},sec);
 [[pct(fc.win),fc.A+" leaves less belief unsearched than "+fc.B],[pct(fc.decisive),"… by more than "+D.decisive+" (decisive)"],
  [pct(fc.loss),fc.B+" better ("+pct(fc.decisive_loss)+" decisively)"],[sg(-fc.adv_mean),"mean residual change, "+fc.A+" − "+fc.B],
  [pct(fc.rel),"mean relative reduction of "+fc.B+"'s residual"]].forEach(([v,l])=>{const x=el("div",{class:"tile"},t);el("div",{class:"v"},x,v);el("div",{class:"l"},x,l);});}

// leaderboard (every arm)
{const sec=el("section",{id:"board"},app);el("p",{class:"eyebrow"},sec,"Leaderboard");el("h2",{},sec,"Every arm");
 const anyOL=D.board.some(b=>b.openloop!=null);
 table(sec,["arm","mean residual","median","as planned"].concat(anyOL?["open-loop gimbal"]:[]).concat(["mean rank","best in","flown / budget","plan time mean / median / max [s]"]).concat(D.order.map(o=>"beats "+o)),
  D.board.map(b=>[{html:swatch(document.createElement("span"),b.arm).outerHTML+" "+b.arm,sort:b.arm},f4(b.mean),f4(b.median),f4(b.planned)].concat(anyOL?[b.openloop==null?"—":f4(b.openloop)]:[])
   .concat([b.rank.toFixed(2),b.best,pct(b.flown_frac),b.plan_s.toFixed(1)+" / "+(b.plan_s_median!=null?b.plan_s_median.toFixed(1):"—")+" / "+(b.plan_s_max!=null?b.plan_s_max.toFixed(0):"—")])
   .concat(D.order.map(o=>o===b.arm?"—":pct(D.pair[b.arm+"|"+o])))));
 el("p",{class:"note"},sec,"Mean residual = "+(D.metric==="residual_hw"?"through the common gimbal (the headline, same law for every arm)":D.metric)+". As planned = each planner's own boresight. Open-loop gimbal (where recorded) = the follower's open_loop law, which replays the planned cross-track angle instead of aiming: a secondary number only. Beats X = share of scenarios this arm leaves less residual than X (by more than 1e-4).");
 const arms=el("details",{},sec);el("summary",{},arms,"Arm definitions");el("pre",{style:"white-space:pre-wrap;font:12px var(--mono);color:var(--ink2)"},arms,JSON.stringify(D.arms.filter(x=>D.order.includes(x.name)),null,1));}

// per opponent: where A wins, heat map, scatter, maps
FS.forEach((fc,fi)=>renderFocus(fc,fi));
function renderFocus(fc,fi){
 const top=el("section",{id:"h2h"+fi,class:"h2h"},app);el("p",{class:"eyebrow"},top,(FS.length>1?"Head to head "+(fi+1)+" / "+FS.length+" · ":"")+"Where "+fc.A+" wins");
 el("h2",{},top,fc.A+" vs "+fc.B+": mean advantage (residual of "+fc.B+" − residual of "+fc.A+"), by factor");
 if(FS.length>1)tiles(top,fc);
 el("p",{class:"note"},top,"Bars to the right: "+fc.A+" searched more. Label: n, win rate, decisive rate (> "+D.decisive+"). Descriptor bins are quartiles.");
 const g=el("div",{class:"grid2"},top);
 Object.entries(fc.by).forEach(([k,v])=>{const c=el("div",{class:"card"},g);el("h3",{},c,v.label);advBars(c,v.groups,fc);});
 // heat map
 {const sec=el("section",{},app);el("h2",{},sec,"Prior family × budget: mean advantage of "+fc.A+" over "+fc.B);
  const H=fc.heat,W=680,cw=(W-170)/H.buds.length,ch=26,svg=s("svg",{viewBox:"0 0 "+W+" "+(H.fams.length*ch+40)},el("div",{class:"card"},sec));
  const m=Math.max(0.02,...H.cells.flat().filter(c=>c.adv!=null).map(c=>Math.abs(c.adv)));
  function mix(a,b,t){const p=x=>[1,3,5].map(i=>parseInt(x.slice(i,i+2),16));const A=p(a),B=p(b);return "rgb("+A.map((v,i)=>Math.round(v+(B[i]-v)*t)).join(",")+")";}
  H.buds.forEach((b,j)=>s("text",{x:170+j*cw+cw/2,y:14,"text-anchor":"middle"},svg).textContent=b+" s");
  H.fams.forEach((f,i)=>{s("text",{x:162,y:40+i*ch-8,"text-anchor":"end"},svg).textContent=f;
   H.cells[i].forEach((c,j)=>{const x=170+j*cw,y=22+i*ch;const t=c.adv==null?0:Math.min(1,Math.abs(c.adv)/m);
    const r=s("rect",{x:x+1,y:y+1,width:cw-2,height:ch-2,rx:3,fill:c.adv==null?css("--grid"):mix(css("--mid"),css(c.adv>=0?"--pos":"--neg"),t)},svg);
    if(c.adv!=null){s("text",{x:x+cw/2,y:y+17,"text-anchor":"middle",style:"fill:"+(t>0.55?"#fff":css("--ink"))},svg).textContent=sg(c.adv);}
    r.addEventListener("mousemove",e=>showTip(e,f+", "+H.buds[j]+" s<br>n "+c.n+(c.adv!=null?"<br>advantage "+sg(c.adv):"")));r.addEventListener("mouseleave",hide);});});
  el("p",{class:"note"},sec,"Blue: "+fc.A+" leaves less belief; red: "+fc.B+" does. Grey: no scenario in that cell.");}
 // scatter
 {const sec=el("section",{},app);el("h2",{},sec,fc.A+" vs "+fc.B+", one dot per scenario");
  const c=el("div",{class:"card"},sec);const selw=el("div",{},c);el("span",{style:"font-size:13px;color:var(--ink2);margin-right:8px"},selw,"Highlight family");
  const sel=el("select",{"aria-label":"highlight prior family"},selw);["all"].concat(fc.heat.fams).forEach(f=>el("option",{value:f},sel,f));
  const W=560,H=460,L=46,R=12,T=10,B=36,svg=s("svg",{viewBox:"0 0 "+W+" "+H,class:"scatter"},c);
  const mx=Math.min(1,Math.max(...D.rows.map(r=>Math.max(r[fc.A],r[fc.B])))*1.05),X=v=>L+(W-L-R)*v/mx,Y=v=>H-B-(H-T-B)*v/mx;
  function draw(){svg.innerHTML="";for(let v=0;v<=mx+1e-9;v+=0.1){s("line",{x1:X(0),x2:X(mx),y1:Y(v),y2:Y(v),stroke:css("--grid")},svg);s("line",{y1:Y(0),y2:Y(mx),x1:X(v),x2:X(v),stroke:css("--grid")},svg);
    s("text",{x:L-6,y:Y(v)+4,"text-anchor":"end"},svg).textContent=v.toFixed(1);s("text",{x:X(v),y:H-B+15,"text-anchor":"middle"},svg).textContent=v.toFixed(1);}
   s("line",{x1:X(0),y1:Y(0),x2:X(mx),y2:Y(mx),stroke:css("--muted"),"stroke-dasharray":"4 4"},svg);
   s("text",{x:(L+W)/2,y:H-4,"text-anchor":"middle"},svg).textContent=fc.B+" residual";
   s("text",{x:12,y:(T+H-B)/2,"text-anchor":"middle",transform:"rotate(-90 12 "+((T+H-B)/2)+")"},svg).textContent=fc.A+" residual";
   const hl=sel.value;D.rows.forEach(r=>{const on=hl==="all"||r.family===hl;
    const p=s("circle",{cx:X(r[fc.B]),cy:Y(r[fc.A]),r:on?4:3,fill:on?col(fc.A):css("--muted"),"fill-opacity":on?.85:.25,stroke:css("--surface"),"stroke-width":1,"data-id":r.id},svg);
    if(D.scenes[r.id]){p.classList.add("clickable");p.addEventListener("click",()=>{hide();openScene(r.id,NAV(fc));});}
    p.addEventListener("mousemove",e=>showTip(e,"<b>"+r.id+"</b><br>"+fc.A+" "+f4(r[fc.A])+"<br>"+fc.B+" "+f4(r[fc.B])+"<br>budget "+r.budget_s+" s, alt "+r.altitude_m+" m, home "+r.home));p.addEventListener("mouseleave",hide);});}
  sel.onchange=draw;draw();el("p",{class:"note"},sec,"Below the diagonal: "+fc.A+" searched more. Click a dot to open the scene.");}
 // maps
 const MP=fc.maps||{};
 if(Object.keys(MP).length){const sec=el("section",{},app);el("h2",{},sec,"Most decisive wins for "+fc.A+" over "+fc.B);
  const lg=el("div",{class:"legend"},sec);[fc.A,fc.B].forEach(a=>{const sp=el("span",{},lg);swatch(sp,a);sp.appendChild(document.createTextNode(" "+a));});
  el("p",{class:"note"},sec,"Prior (bright = more belief), "+fc.A+" track solid, "+fc.B+" dashed, home = white square. Click a card to open the scene viewer.");
  function grid(ids){const g=el("div",{class:"maps"},sec);ids.forEach(id=>{if(!MP[id])return;const r=byId[id];const m=el("div",{class:"map"},g);el("img",{src:MP[id],alt:id},m);if(D.scenes[id]){m.classList.add("clickable");m.onclick=()=>openScene(id,ids);}
   const t=el("div",{},m);t.innerHTML="<b>"+id+"</b><br>"+fc.A+" "+f3(r[fc.A])+" · "+fc.B+" "+f3(r[fc.B])+"<br>budget "+r.budget_s+" s · alt "+r.altitude_m+" m · home "+r.home;});}
  grid(fc.top);el("h2",{},sec,"Where "+fc.B+" does best against "+fc.A);grid(fc.bottom);}
}
function advBars(parent,groups,fc){const W=520,rowH=26,L=150,R=150,H=groups.length*rowH+24;const svg=s("svg",{viewBox:"0 0 "+W+" "+H},parent);
 const m=Math.max(0.02,...groups.map(g=>Math.abs(g.adv)));const X=v=>L+(W-L-R)*(v+m)/(2*m);
 s("line",{x1:X(0),x2:X(0),y1:4,y2:H-18,stroke:css("--muted")},svg);
 groups.forEach((gr,i)=>{const y=8+i*rowH;const x0=X(0),x1=X(gr.adv);
  s("text",{x:L-8,y:y+13,"text-anchor":"end"},svg).textContent=String(gr.key).slice(0,20);
  const r=s("rect",{x:Math.min(x0,x1),y:y+2,width:Math.max(1,Math.abs(x1-x0)),height:rowH-8,rx:3,fill:css(gr.adv>=0?"--pos":"--neg")},svg);
  s("text",{x:W-R+8,y:y+13},svg).textContent=sg(gr.adv)+"  n"+gr.n+" "+pct(gr.win)+"/"+pct(gr.decisive);
  r.addEventListener("mousemove",e=>showTip(e,"<b>"+gr.key+"</b><br>n "+gr.n+"<br>"+fc.A+" "+f4(gr.a)+"<br>"+fc.B+" "+f4(gr.b)+"<br>advantage "+sg(gr.adv)+" ("+pct(gr.rel)+" relative)<br>wins "+pct(gr.win)+", decisive "+pct(gr.decisive)));r.addEventListener("mouseleave",hide);});
 s("text",{x:X(-m),y:H-4},svg).textContent=sg(-m);s("text",{x:X(m),y:H-4,"text-anchor":"end"},svg).textContent=sg(m);}

// anytime curves (every arm)
{const sec=el("section",{id:"anytime"},app);el("h2",{},sec,"Anytime: mean residual vs fraction of the flown distance");
 const c=el("div",{class:"card"},sec);const lg=el("div",{class:"legend"},c);
 D.order.forEach(a=>{const sp=el("span",{},lg);swatch(sp,a);sp.appendChild(document.createTextNode(" "+a));});
 const W=640,H=280,L=46,R=12,T=10,B=30,svg=s("svg",{viewBox:"0 0 "+W+" "+H},c);
 const X=f=>L+(W-L-R)*f,Y=v=>H-B-(H-T-B)*v;
 for(let v=0;v<=1.001;v+=0.2){s("line",{x1:L,x2:W-R,y1:Y(v),y2:Y(v),stroke:css("--grid")},svg);s("text",{x:L-6,y:Y(v)+4,"text-anchor":"end"},svg).textContent=v.toFixed(1);
  s("text",{x:X(v),y:H-10,"text-anchor":"middle"},svg).textContent=(100*v).toFixed(0)+"%";}
 Object.entries(D.curves).forEach(([a,cv])=>{const pts=[[0,1]].concat(cv.map((v,i)=>[0.05*(i+1),v]));
  const pl=s("polyline",{points:pts.map(p=>X(p[0])+","+Y(p[1])).join(" "),fill:"none",stroke:col(a),"stroke-width":a===SUBJ?2.6:2,"stroke-dasharray":dashOf(a)},svg);
  pl.addEventListener("mousemove",e=>showTip(e,"<b>"+a+"</b><br>final "+f4(cv[cv.length-1])));pl.addEventListener("mouseleave",hide);});}

// full table (every arm; one advantage column per opponent)
{const sec=el("section",{id:"all"},app);const d=el("details",{open:""},sec);el("summary",{},d,"All scenarios ("+D.rows.length+"): click a header to sort, click a row to see its trajectories");
 const extra=["effective_area_km2","budget_over_spread","mass_within_half_budget","peaks_25pct","gini"];
 const advCell=(r,fc)=>({html:'<span class="'+((r[fc.B]-r[fc.A])>=0?"pos":"neg")+'">'+sg(r[fc.B]-r[fc.A])+"</span>",sort:r[fc.B]-r[fc.A]});
 table(d,["id","family","budget [s]","alt [m]","home"].concat(D.order).concat(FS.map(fc=>FS.length>1?"adv vs "+fc.B:"adv")).concat(extra).concat(["ia chose"]),
  D.rows.map(r=>[r.id,r.family,r.budget_s,r.altitude_m,r.home].concat(D.order.map(a=>f4(r[a]))).concat(FS.map(fc=>advCell(r,fc)))
   .concat(extra.map(k=>typeof r[k]==="number"?(+r[k]).toFixed(3):r[k])).concat([r.ia_chosen||""])),null,openScene);
 if(FS.length)el("p",{class:"note"},d,"adv = opponent's residual − "+SUBJ+"'s (positive, blue: "+SUBJ+" searched more).");}

// ---------- scene viewer: click a scenario anywhere to see every arm's trajectory on its prior
function NAV(fc){fc=fc||F;const ids=D.rows.map(r=>r.id).filter(id=>D.scenes[id]);if(fc)ids.sort((a,b)=>{const ra=byId[a],rb=byId[b];return (rb[fc.B]-rb[fc.A])-(ra[fc.B]-ra[fc.A]);});return ids;}
const SV={on:null,looks:true,overlay:true,nav:[],id:null};
const dlg=document.getElementById("scene");
dlg.addEventListener("click",e=>{if(e.target===dlg)dlg.close();});
dlg.addEventListener("close",()=>{hide();});
document.addEventListener("keydown",e=>{if(!dlg.open)return;if(e.key==="ArrowRight"||e.key==="ArrowLeft"){e.preventDefault();step(e.key==="ArrowRight"?1:-1);}});
function step(d){const i=SV.nav.indexOf(SV.id);if(i<0||!SV.nav.length)return;renderScene(SV.nav[(i+d+SV.nav.length)%SV.nav.length]);}
function openScene(id,nav){SV.nav=(nav&&nav.length?nav:NAV()).filter(x=>D.scenes[x]);if(!SV.on){SV.on={};const dflt=F?[F.A].concat(FS.map(fc=>fc.B)):D.order.slice(0,2);D.order.forEach(a=>SV.on[a]=dflt.includes(a));}
 renderScene(id);if(!dlg.open)dlg.showModal();}
function renderScene(id){SV.id=id;const S=D.scenes[id],r=byId[id];if(!S||!r)return;dlg.innerHTML="";
 const hd=el("div",{class:"sv-head"},dlg);el("h2",{},hd,id);
 el("span",{class:"facts"},hd,r.family+" · budget "+r.budget_s+" s · altitude "+r.altitude_m+" m · home "+r.home+" · beta "+S.beta+" m");
 el("span",{class:"sp"},hd);const i=SV.nav.indexOf(id);
 el("span",{class:"facts"},hd,(i>=0?(i+1)+" / "+SV.nav.length:""));
 const pv=el("button",{type:"button","aria-label":"previous scenario"},hd,"← prev");pv.onclick=()=>step(-1);
 const nx=el("button",{type:"button","aria-label":"next scenario"},hd,"next →");nx.onclick=()=>step(1);
 const cl=el("button",{type:"button","aria-label":"close"},hd,"close");cl.onclick=()=>dlg.close();
 const body=el("div",{class:"sv-body"},dlg);
 // controls
 const ct=el("div",{class:"sv-ctrl"},body);
 const arms=D.order.filter(a=>S.arms[a]);const best=Math.min(...arms.map(a=>S.arms[a].v));
 arms.forEach((a,k)=>{const lb=el("label",{},ct);const cb=el("input",{type:"checkbox"},lb);cb.checked=!!SV.on[a];cb.onchange=()=>{SV.on[a]=cb.checked;renderScene(id);};
  swatch(lb,a);
  lb.appendChild(document.createTextNode(a+"  "+f3(S.arms[a].v)+(S.arms[a].v<=best+1e-9?"  (best)":"")));});
 {const lb=el("label",{},ct);const cb=el("input",{type:"checkbox"},lb);cb.checked=SV.looks;cb.onchange=()=>{SV.looks=cb.checked;renderScene(id);};lb.appendChild(document.createTextNode("camera ground points"));}
 {const lb=el("label",{},ct);const cb=el("input",{type:"checkbox"},lb);cb.checked=!SV.overlay;cb.onchange=()=>{SV.overlay=!cb.checked;renderScene(id);};lb.appendChild(document.createTextNode("side by side"));}
 const shown=arms.filter(a=>SV.on[a]);
 const wrap=el("div",{class:"sv-maps"+(SV.overlay?" overlay":"")},body);
 if(!shown.length)el("p",{class:"note"},wrap,"Tick an arm above to draw it.");
 if(SV.overlay&&shown.length)drawScene(wrap,S,shown,"all selected arms, overlaid");
 else shown.forEach(a=>drawScene(wrap,S,[a],a+" · residual "+f4(S.arms[a].v)));
 el("p",{class:"note"},body,"Grey: prior ("+(matchMedia("(prefers-color-scheme: dark)").matches?"brighter":"darker")+" = more belief). Lines: aircraft path; dots: where the camera looked (every ~50 m). Square: home. North up, east right. Hover a path for the distance flown there; ← / → step through scenarios; Esc closes.");
 // numbers
 table(body,["arm",D.metric==="residual_hw"?"residual (through gimbal)":"residual","as planned","flown / budget","flown [km]"],
  arms.map(a=>{const x=S.arms[a];return [a,f4(x.v),f4(x.planned),x.budget?pct(x.flown/x.budget):"—",(x.flown/1000).toFixed(1)];}));}
function drawScene(parent,S,arms,title){const box=el("div",{class:"sv-map"},parent);el("h3",{},box,title);
 const [n0,n1,e0,e1]=S.area,W=e1-e0,H=n1-n0,X=e=>e-e0,Y=n=>n1-n;
 const svg=s("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"trajectories of "+arms.join(", ")},box);
 s("image",{href:S.prior,x:0,y:0,width:W,height:H,preserveAspectRatio:"none"},svg);
 const verts=[];
 arms.forEach(a=>{const A=S.arms[a],c=col(a),k=D.order.indexOf(a);
  if(SV.looks)A.looks.forEach(L=>{const g=s("g",{fill:c,"fill-opacity":0.35},svg);for(let i=0;i+1<L.length;i+=2)s("circle",{cx:X(L[i+1]),cy:Y(L[i]),r:W/260},g);});});
 arms.forEach(a=>{const A=S.arms[a],c=col(a),k=D.order.indexOf(a);
  A.paths.forEach(P=>{const pts=[];let d=0;for(let i=0;i+1<P.length;i+=2){if(i)d+=Math.hypot(P[i]-P[i-2],P[i+1]-P[i-1]);pts.push(X(P[i+1])+","+Y(P[i]));verts.push([X(P[i+1]),Y(P[i]),a,d]);}
   const pl=pts.join(" ");
   s("polyline",{points:pl,fill:"none",stroke:css("--halo"),"stroke-width":4.5,"stroke-linejoin":"round","vector-effect":"non-scaling-stroke","stroke-opacity":0.9},svg);
   s("polyline",{points:pl,fill:"none",stroke:c,"stroke-width":2,"stroke-linejoin":"round","stroke-dasharray":dashOf(a),"vector-effect":"non-scaling-stroke"},svg);});});
 S.homes.forEach(h=>s("rect",{x:X(h[1])-W/70,y:Y(h[0])-W/70,width:W/35,height:W/35,fill:css("--s4"),stroke:"#000","stroke-width":1,"vector-effect":"non-scaling-stroke"},svg));
 const dot=s("circle",{r:W/110,fill:"none",stroke:css("--ink"),"stroke-width":2,"vector-effect":"non-scaling-stroke",visibility:"hidden"},svg);
 svg.addEventListener("pointermove",e=>{const b=svg.getBoundingClientRect(),px=(e.clientX-b.left)/b.width*W,py=(e.clientY-b.top)/b.height*H;
  let bi=-1,bd=1e18;verts.forEach((v,i)=>{const q=(v[0]-px)**2+(v[1]-py)**2;if(q<bd){bd=q;bi=i;}});
  if(bi<0||Math.sqrt(bd)>W/25){dot.setAttribute("visibility","hidden");hide();return;}
  const v=verts[bi],A=S.arms[v[2]];dot.setAttribute("cx",v[0]);dot.setAttribute("cy",v[1]);dot.setAttribute("visibility","visible");
  showTip(e,"<b>"+v[2]+"</b><br>"+(v[3]/1000).toFixed(2)+" km flown here"+(A.budget?" ("+pct(v[3]/A.budget)+" of budget)":"")+"<br>N "+Math.round(n1-v[1])+" m, E "+Math.round(e0+v[0])+" m");});
 svg.addEventListener("pointerleave",()=>{dot.setAttribute("visibility","hidden");hide();});}
if(D.errors.length){const sec=el("section",{},app);el("h2",{},sec,"Failed plans ("+D.n_errors+")");table(sec,["scenario","arm","error"],D.errors.map(e=>[e.id,e.arm,e.error]));}
</script></body></html>
"""

if __name__ == "__main__":
    sys.exit(main())
