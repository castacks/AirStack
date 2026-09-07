#!/usr/bin/env python3
"""Which measurable feature of an asset predicts how long baking it takes?

    AirStack/.venv/bin/python scene_gen/tools/bake_corr_page.py \\
        -o scene_gen/_bakelab/bake_corr.html

Reads `_bakelab/bake_rungs.csv` (one row per baked archetype) joined to
`_bakelab/asset_properties.json` (what the source asset is), and plots every
candidate feature against wall-clock bake time.

SPEARMAN, NOT PEARSON. These relationships are monotonic but not linear —
cell count grows with the 2/3 power of volume, time grows with cells — and a
Pearson r on raw values would be dominated by `office_tower` sitting three
orders of magnitude out on its own. Rank correlation asks the question that
actually matters here ("does more of this mean more time?") and is unmoved by
the outlier.

THE PHASE SPLIT IS THE POINT. Correlating against TOTAL time hides the finding:
the fracture and the settle have different drivers, and in this pack those
drivers are anti-correlated, so the two effects cancel and the total looks like
noise. Every chart here is therefore drawn per phase as well as in total.
"""
from __future__ import annotations

import argparse
import csv
import html
import json
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)

#: Validated categorical slots 1-3 (dataviz reference palette), light / dark.
#: `validate_palette.js --pairs all` passes in both modes; the light aqua sits
#: at 2.74:1 on the light surface, which the relief rule covers with the direct
#: labels on every bar and the table at the foot of the page.
SERIES = [("build", "#2a78d6", "#3987e5"),
          ("settle", "#eb6834", "#d95926"),
          ("export", "#1baf7a", "#199e70")]

FEATURES = [
    ("P_points",  "source points",       "known"),
    ("P_faces",   "source triangles",    "known"),
    ("P_meshes",  "source meshes",       "known"),
    ("P_materials", "source materials",  "known"),
    ("P_textures", "source textures",    "known"),
    ("P_file_mb", "source file MB",      "known"),
    ("volume_m3", "bbox volume",         "known"),
    ("P_z_m",     "height",              "known"),
    ("P_x_m",     "width",               "known"),
    ("P_y_m",     "depth",               "known"),
    ("frag_cells", "voronoi cells",      "produced"),
    ("frag_loose", "loose fragments",    "produced"),
    ("frag_anchored", "anchored fragments", "produced"),
    ("thickened", "meshes thickened",    "produced"),
    ("settle_bodies", "physx bodies",    "produced"),
    ("settle_steps_used", "settle steps", "produced"),
    ("out_meshes", "exported meshes",    "produced"),
    ("usd_mb",    "output MB",           "produced"),
]


# --------------------------------------------------------------------------- #
# data
# --------------------------------------------------------------------------- #
def load():
    props = {p["type"]: p for p in json.load(
        open(os.path.join(_SCENE_GEN, "_bakelab", "asset_properties.json")))}
    rows = [r for r in csv.DictReader(
        open(os.path.join(_SCENE_GEN, "_bakelab", "bake_rungs.csv")))
        if r["outcome"] == "baked" and r["level"] != "pristine" and r["seconds"]]
    for r in rows:
        p = props.get(r["type"], {})
        v = (p.get("x_m") or 0) * (p.get("y_m") or 0) * (p.get("z_m") or 0)
        r["volume_m3"] = v or ""
        for k in ("points", "faces", "materials", "textures", "file_mb",
                  "x_m", "y_m", "z_m", "meshes"):
            r["P_" + k] = p.get(k, "")
    return rows


def col(rows, k):
    return np.array([float(r[k]) if r.get(k) not in (None, "", "None")
                     else np.nan for r in rows])


def spearman(x, y):
    m = ~(np.isnan(x) | np.isnan(y))
    x, y = x[m], y[m]
    if len(x) < 8 or np.std(x) == 0:
        return None
    rx = np.argsort(np.argsort(x)).astype(float)
    ry = np.argsort(np.argsort(y)).astype(float)
    return float(np.corrcoef(rx, ry)[0, 1])


# --------------------------------------------------------------------------- #
# svg helpers
# --------------------------------------------------------------------------- #
def esc(s):
    return html.escape(str(s))


def fmt(v):
    a = abs(v)
    if a >= 1e6:
        return f"{v/1e6:.1f}M"
    if a >= 1e3:
        return f"{v/1e3:.0f}k"
    if a >= 10:
        return f"{v:.0f}"
    if a >= 1:
        return f"{v:.1f}"
    return f"{v:.2g}"


def corr_bars(feats, W=760):
    """Grouped horizontal bars: rho against each phase, one group per feature.

    A DIVERGING AXIS because the sign is the finding — source points speeds
    nothing up, but it genuinely anti-correlates with settle time, and a
    magnitude-only chart would hide the cancellation that makes the total
    read as noise.
    """
    rowh, gap, pad_l, pad_t = 46, 10, 178, 44
    H = pad_t + len(feats) * (rowh + gap) + 48
    x0, x1 = pad_l, W - 54
    mid = (x0 + x1) / 2
    half = (x1 - x0) / 2
    sx = lambda r: mid + r * half

    p = [f'<svg viewBox="0 0 {W} {H}" role="img" class="chart" '
         f'aria-label="Rank correlation of each feature with bake time, by phase">']
    # ROW BANDS FIRST, then the grid over them. Drawn the other way round the
    # bands punch the vertical gridlines into dashes and the chart reads as if
    # something failed to render.
    for i in range(len(feats)):
        y = pad_t + i * (rowh + gap)
        p.append(f'<rect class="rowbg" x="{x0}" y="{y-3}" width="{x1-x0}" '
                 f'height="{rowh}" rx="2"/>')
    for r in (-1.0, -0.5, 0.5, 1.0):
        p.append(f'<line class="grid" x1="{sx(r):.1f}" y1="{pad_t-8}" '
                 f'x2="{sx(r):.1f}" y2="{H-30}"/>')
    p.append(f'<line class="ax" x1="{mid}" y1="{pad_t-8}" x2="{mid}" y2="{H-30}"/>')
    for r in (-1.0, -0.5, 0.0, 0.5, 1.0):
        p.append(f'<text class="tick" x="{sx(r):.1f}" y="{H-12}" '
                 f'text-anchor="middle">{r:+.1f}</text>'.replace("+0.0", "0"))

    bh = (rowh - 6) / 3.0
    seen_groups = set()
    for i, f in enumerate(feats):
        y = pad_t + i * (rowh + gap)
        # A LABELLED DIVIDER WHERE THE KIND CHANGES. The argument on this page
        # is entirely about which side of that line a feature sits on — a
        # feature the bake PRODUCES cannot be used to predict the bake — and a
        # single ordered list gives the reader no way to see the boundary.
        if f["group"] not in seen_groups:
            seen_groups.add(f["group"])
            lab = ("KNOWN BEFORE THE BAKE" if f["group"] == "known"
                   else "PRODUCED BY THE BAKE")
            p.append(f'<text class="glab" x="{pad_l-12}" y="{y-9}" '
                     f'text-anchor="end">{lab}</text>')
            p.append(f'<line class="gdiv" x1="{x0}" y1="{y-13}" '
                     f'x2="{x1}" y2="{y-13}"/>')
        p.append(f'<text class="flab" x="{pad_l-12}" y="{y+rowh/2+4}" '
                 f'text-anchor="end">{esc(f["label"])}</text>')
        for j, (name, _lc, _dc) in enumerate(SERIES):
            v = f[name]
            yy = y + j * bh + 1
            a, b = (mid, sx(v)) if v >= 0 else (sx(v), mid)
            w = max(abs(b - a), 1.2)
            p.append(f'<rect class="bar s{j}" x="{a:.1f}" y="{yy:.1f}" '
                     f'width="{w:.1f}" height="{bh-2:.1f}" rx="2">'
                     f'<title>{esc(f["label"])} vs {name}: rho {v:+.2f}</title></rect>')
            lx = (b + 5) if v >= 0 else (a - 5)
            anc = "start" if v >= 0 else "end"
            p.append(f'<text class="val" x="{lx:.1f}" y="{yy+bh/2+2:.1f}" '
                     f'text-anchor="{anc}">{v:+.2f}</text>')
    p.append("</svg>")
    return "".join(p)


def scatter(xs, ys, xlabel, ylabel, rho, tips=None, logx=True, W=330, H=250):
    """One feature against bake seconds. Log-log, with a power-law fit.

    Every dot carries an SVG `<title>`, which is the whole hover layer: a
    native tooltip needs no script, survives the artifact sandbox, and is
    reachable by the accessibility tree, which a custom div tooltip is not.
    """
    m = ~(np.isnan(xs) | np.isnan(ys)) & (ys > 0)
    if logx:
        m &= xs > 0
    x, y = xs[m], ys[m]
    lab = [t for t, keep in zip(tips or [], m) if keep] if tips else None
    if len(x) < 3:
        return '<div class="nodata">no data</div>'
    lx = np.log10(x) if logx else x
    ly = np.log10(y)
    pad_l, pad_b, pad_t, pad_r = 44, 32, 12, 10
    x0, x1 = pad_l, W - pad_r
    y0, y1 = H - pad_b, pad_t
    lo_x, hi_x = float(lx.min()), float(lx.max())
    lo_y, hi_y = float(ly.min()), float(ly.max())
    if hi_x - lo_x < 1e-9:
        lo_x, hi_x = lo_x - 0.5, hi_x + 0.5
    if hi_y - lo_y < 1e-9:
        lo_y, hi_y = lo_y - 0.5, hi_y + 0.5
    px = lambda v: x0 + (v - lo_x) / (hi_x - lo_x) * (x1 - x0)
    py = lambda v: y0 + (v - lo_y) / (hi_y - lo_y) * (y1 - y0)

    p = [f'<svg viewBox="0 0 {W} {H}" role="img" class="chart sc" '
         f'aria-label="{esc(xlabel)} against bake seconds, rho {rho:+.2f}">']
    for k in range(4):
        t = lo_y + (hi_y - lo_y) * k / 3.0
        p.append(f'<line class="grid" x1="{x0}" y1="{py(t):.1f}" '
                 f'x2="{x1}" y2="{py(t):.1f}"/>')
        p.append(f'<text class="tick" x="{x0-6}" y="{py(t)+3:.1f}" '
                 f'text-anchor="end">{fmt(10**t)}</text>')
    for k in range(3):
        t = lo_x + (hi_x - lo_x) * k / 2.0
        p.append(f'<text class="tick" x="{px(t):.1f}" y="{H-12}" '
                 f'text-anchor="middle">{fmt(10**t if logx else t)}</text>')
    # power-law fit in log space
    if len(x) > 4:
        b, a = np.polyfit(lx, ly, 1)
        p.append(f'<line class="fit" x1="{px(lo_x):.1f}" y1="{py(a+b*lo_x):.1f}" '
                 f'x2="{px(hi_x):.1f}" y2="{py(a+b*hi_x):.1f}"/>')
    for i, (xv, yv) in enumerate(zip(lx, ly)):
        tip = (f'<title>{esc(lab[i])} — {esc(xlabel)} {fmt(10**xv if logx else xv)}'
               f', {fmt(10**yv)} s</title>') if lab else ""
        p.append(f'<circle class="pt" cx="{px(xv):.1f}" cy="{py(yv):.1f}" r="4">'
                 f'{tip}</circle>' if tip else
                 f'<circle class="pt" cx="{px(xv):.1f}" cy="{py(yv):.1f}" r="4"/>')
    p.append(f'<text class="axlab" x="{(x0+x1)/2:.0f}" y="{H-1}" '
             f'text-anchor="middle">{esc(xlabel)}</text>')
    p.append(f'<text class="axlab" transform="translate(11,{(y0+y1)/2:.0f}) '
             f'rotate(-90)" text-anchor="middle">{esc(ylabel)}</text>')
    p.append("</svg>")
    return "".join(p)


CSS = """
:root{
  --ground:#F2F4F6; --surface:#FFFFFF; --surface-2:#E9EDF1; --line:#D3DAE1;
  --ink:#151A20; --ink-2:#4B5765; --ink-3:#77838F; --accent-ink:#1B486C;
  --s0:#2a78d6; --s1:#eb6834; --s2:#1baf7a;
  --grid:#E3E8ED; --fit:#8A97A4; --pt:#25527A; --rowbg:#F7F9FB;
  --shadow:0 1px 2px rgba(21,26,32,.06),0 8px 24px -12px rgba(21,26,32,.18);
}
@media (prefers-color-scheme: dark){
  :root:not([data-theme="light"]){
    --ground:#0E1216; --surface:#161C23; --surface-2:#1E262F; --line:#2C3742;
    --ink:#E6EBF0; --ink-2:#A8B4C0; --ink-3:#7C8894; --accent-ink:#9CC6E6;
    --s0:#3987e5; --s1:#d95926; --s2:#199e70;
    --grid:#242E38; --fit:#6C7A88; --pt:#7FB2DC; --rowbg:#1A222A;
    --shadow:0 1px 2px rgba(0,0,0,.4),0 10px 28px -14px rgba(0,0,0,.7);
  }
}
:root[data-theme="dark"]{
  --ground:#0E1216; --surface:#161C23; --surface-2:#1E262F; --line:#2C3742;
  --ink:#E6EBF0; --ink-2:#A8B4C0; --ink-3:#7C8894; --accent-ink:#9CC6E6;
  --s0:#3987e5; --s1:#d95926; --s2:#199e70;
  --grid:#242E38; --fit:#6C7A88; --pt:#7FB2DC; --rowbg:#1A222A;
  --shadow:0 1px 2px rgba(0,0,0,.4),0 10px 28px -14px rgba(0,0,0,.7);
}
*{box-sizing:border-box}
body{margin:0;background:var(--ground);color:var(--ink);
  font-family:"Source Sans 3",system-ui,sans-serif;font-size:16px;line-height:1.55}
.wrap{max-width:1080px;margin:0 auto;padding:40px 24px 90px;
  display:flex;flex-direction:column;gap:44px}
h1,h2{font-family:Archivo,system-ui,sans-serif;margin:0;text-wrap:balance;
  letter-spacing:-.015em}
h1{font-size:clamp(30px,4.4vw,44px);font-weight:700;line-height:1.06}
h2{font-size:22px;font-weight:650}
p{margin:0}
.eyebrow{font-family:"IBM Plex Mono",monospace;font-size:11px;letter-spacing:.16em;
  text-transform:uppercase;color:var(--ink-3)}
.lede{color:var(--ink-2);max-width:68ch;font-size:17px}
header.top{display:flex;flex-direction:column;gap:14px;
  border-bottom:1px solid var(--line);padding-bottom:28px}
section{display:flex;flex-direction:column;gap:16px}
.section-head{display:flex;flex-direction:column;gap:6px}
.card{background:var(--surface);border:1px solid var(--line);border-radius:3px;
  padding:18px;box-shadow:var(--shadow);overflow-x:auto}
.chart{display:block;width:100%;height:auto;font-family:"IBM Plex Mono",monospace}
.chart .grid{stroke:var(--grid);stroke-width:1}
.chart .ax{stroke:var(--ink-3);stroke-width:1.5}
.chart .rowbg{fill:var(--rowbg)}
.chart .tick{fill:var(--ink-3);font-size:10px}
.chart .flab{fill:var(--ink);font-size:12.5px}
.chart .val{fill:var(--ink-2);font-size:10px;font-variant-numeric:tabular-nums}
.chart .axlab{fill:var(--ink-3);font-size:10.5px}
.chart .glab{fill:var(--ink-3);font-size:9.5px;letter-spacing:.12em}
.chart .gdiv{stroke:var(--line);stroke-width:1}
.chart .bar{shape-rendering:crispEdges}
.chart .bar.s0{fill:var(--s0)} .chart .bar.s1{fill:var(--s1)} .chart .bar.s2{fill:var(--s2)}
.chart .fit{stroke:var(--fit);stroke-width:2;stroke-dasharray:5 4}
.chart .pt{fill:var(--pt);fill-opacity:.62;stroke:var(--surface);stroke-width:1.2}
.legend{display:flex;gap:18px;flex-wrap:wrap;align-items:center;
  font-family:"IBM Plex Mono",monospace;font-size:12px;color:var(--ink-2)}
.legend i{display:inline-block;width:11px;height:11px;border-radius:2px;
  margin-right:6px;vertical-align:-1px}
.grid3{display:grid;gap:14px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr))}
.panel{background:var(--surface);border:1px solid var(--line);border-radius:3px;
  padding:12px 12px 6px;box-shadow:var(--shadow)}
.panel h3{margin:0 0 2px;font-family:"IBM Plex Mono",monospace;font-size:12.5px;
  font-weight:600;color:var(--ink);letter-spacing:.02em}
.panel .rho{font-family:"IBM Plex Mono",monospace;font-size:11px;color:var(--ink-3);
  font-variant-numeric:tabular-nums;display:flex;flex-wrap:wrap;gap:2px 10px;
  margin-bottom:2px}
.panel .rho b{color:var(--ink-2);font-weight:600}
table{border-collapse:collapse;width:100%;font-size:13.5px;
  font-variant-numeric:tabular-nums}
th,td{padding:7px 11px;text-align:left;border-bottom:1px solid var(--line);white-space:nowrap}
thead th{background:var(--surface-2);font-family:"IBM Plex Mono",monospace;
  font-size:11px;letter-spacing:.07em;text-transform:uppercase;color:var(--ink-2)}
td.n,th.n{text-align:right}
tbody tr:hover{background:var(--surface-2)}
.tag{font-family:"IBM Plex Mono",monospace;font-size:10.5px;padding:1px 7px;
  border:1px solid var(--line);border-radius:2px;color:var(--ink-3)}
.notes{display:grid;gap:14px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr))}
.note{background:var(--surface);border:1px solid var(--line);
  border-left:3px solid var(--s0);border-radius:3px;padding:14px 16px;
  display:flex;flex-direction:column;gap:6px}
.note p{font-size:14px;color:var(--ink-2)}
.note b{font-size:14px}
code{font-family:"IBM Plex Mono",monospace;font-size:.92em;
  background:var(--surface-2);padding:1px 4px;border-radius:2px}
footer{border-top:1px solid var(--line);padding-top:18px;font-size:13px;color:var(--ink-3)}
:focus-visible{outline:2px solid var(--s0);outline-offset:2px}
@media (prefers-reduced-motion:reduce){*{animation:none!important;transition:none!important}}
"""


def build(out: str) -> str:
    rows = load()
    secs = col(rows, "seconds")
    tips = [f'{r["type"]} {r["level"]}' for r in rows]
    phases = {n: col(rows, f"t_{n}_s") for n, _l, _d in SERIES}

    feats = []
    for c, label, group in FEATURES:
        v = col(rows, c)
        rt = spearman(v, secs)
        if rt is None:
            continue
        f = {"col": c, "label": label, "group": group, "total": rt, "x": v}
        for n, _l, _d in SERIES:
            f[n] = spearman(v, phases[n]) or 0.0
        feats.append(f)

    known = sorted([f for f in feats if f["group"] == "known"],
                   key=lambda f: -abs(f["total"]))
    produced = sorted([f for f in feats if f["group"] == "produced"],
                      key=lambda f: -abs(f["total"]))

    legend = "".join(
        f'<span><i style="background:var(--s{j})"></i>{esc(n)}</span>'
        for j, (n, _l, _d) in enumerate(SERIES))

    def panels(fs, n=6):
        out = []
        for f in fs[:n]:
            out.append(
                f'<div class="panel"><h3>{esc(f["label"])}</h3>'
                f'<div class="rho"><b>total {f["total"]:+.2f}</b>'
                f'<span>build {f["build"]:+.2f}</span>'
                f'<span>settle {f["settle"]:+.2f}</span>'
                f'<span>export {f["export"]:+.2f}</span></div>'
                + scatter(f["x"], secs, f["label"], "bake seconds",
                          f["total"], tips=tips)
                + "</div>")
        return "".join(out)

    trows = "".join(
        f'<tr><td>{esc(f["label"])}</td><td><span class="tag">'
        f'{"known up front" if f["group"]=="known" else "bake output"}</span></td>'
        f'<td class="n">{f["total"]:+.2f}</td><td class="n">{f["build"]:+.2f}</td>'
        f'<td class="n">{f["settle"]:+.2f}</td><td class="n">{f["export"]:+.2f}</td></tr>'
        for f in sorted(feats, key=lambda f: -abs(f["total"])))

    doc = f"""<title>Bake Time Drivers</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@650;700&family=IBM+Plex+Mono:wght@400;600&family=Source+Sans+3:wght@400;600&display=swap">
<style>{CSS}</style>
<div class="wrap">
<header class="top">
  <span class="eyebrow">Stage A · earthquake · {len(rows)} baked archetypes</span>
  <h1>Nothing you know in advance predicts a bake</h1>
  <p class="lede">Spearman rank correlation between every measurable feature of
  an asset and the wall-clock time to bake one damaged rung of it. The headline
  is a negative result: the best feature knowable <em>before</em> the bake —
  bounding-box volume — reaches only <strong>+0.66</strong>, and source point
  count, the thing that looks most like "how heavy is this mesh", reaches
  <strong>+0.08</strong>. The reason is in the second chart.</p>
</header>

<section>
  <div class="section-head"><span class="eyebrow">The finding</span>
  <h2>The phases have opposite drivers, so the total cancels</h2>
  <p class="lede">Source point count drives the fracture (<strong>+0.51</strong>)
  and the export (<strong>+0.62</strong>) but runs <em>backwards</em> against the
  settle (<strong>&minus;0.21</strong>) — because in this pack the low-poly
  assets are the big ones. The <code>selected_citydemo</code> towers are 70&ndash;300
  points and 100,000&nbsp;m&sup3;; the standalone houses are 60,000 points and
  700&nbsp;m&sup3;. Points and volume correlate at &minus;0.29, points and cell
  count at &minus;0.41. Sum the phases and the two effects annihilate.</p></div>
  <div class="card">
    <div class="legend">{legend}<span style="color:var(--ink-3)">rank correlation, &minus;1 to +1</span></div>
    {corr_bars(known + produced)}
  </div>
</section>

<section>
  <div class="section-head"><span class="eyebrow">Known before you bake</span>
  <h2>What you could predict from</h2>
  <p class="lede">Log&ndash;log, one dot per baked archetype, dashed line a
  power-law fit. Ordered by |rho| against total time.</p></div>
  <div class="grid3">{panels(known, 6)}</div>
</section>

<section>
  <div class="section-head"><span class="eyebrow">Produced by the bake</span>
  <h2>What the bake itself generates</h2>
  <p class="lede">These are outcomes, not inputs — you cannot use them to
  schedule work. They are here because they say <em>where</em> the time goes:
  loose fragments against settle time is the strongest relationship on the
  page at <strong>+0.89</strong>, and output size against export time is
  <strong>+0.90</strong>.</p></div>
  <div class="grid3">{panels(produced, 6)}</div>
</section>

<section>
  <div class="section-head"><span class="eyebrow">What follows</span>
  <h2>Three things this changes</h2></div>
  <div class="notes">
    <div class="note"><b>Schedule on volume, not on poly count</b>
    <p>Bounding-box volume at +0.66 is the only usable ex-ante predictor, and it
    works because it proxies cell count (rho +0.87), which drives the settle —
    the larger half of the clock.</p></div>
    <div class="note"><b>Decimation is still the right lever, but only for two phases</b>
    <p>Source triangles drive fracture (+0.54) and export (+0.61) and nothing
    else. Cutting them attacks 55% of the wall clock and almost all of the
    disk; it will not touch the settle at all.</p></div>
    <div class="note"><b>Anchored fragments are nearly free to simulate, not to write</b>
    <p>Anchored count correlates +0.00 with settle — PhysX never sees them, as
    designed — but +0.22 with export. They cost disk and write time, not
    solver time.</p></div>
  </div>
</section>

<section>
  <div class="section-head"><span class="eyebrow">Table view</span>
  <h2>Every feature</h2></div>
  <div class="card"><table><thead><tr><th>feature</th><th>kind</th>
    <th class="n">total</th><th class="n">build</th><th class="n">settle</th>
    <th class="n">export</th></tr></thead><tbody>{trows}</tbody></table></div>
</section>

<footer>Spearman rank correlation over {len(rows)} baked damaged archetypes
(<code>pristine</code> excluded — it is a re-export, not a bake). Source:
<code>scene_gen/_bakelab/bake_rungs.csv</code> joined to
<code>asset_properties.json</code>; regenerate with
<code>scene_gen/tools/bake_corr_page.py</code>.</footer>
</div>
"""
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    open(out, "w").write(doc)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("-o", "--out",
                    default=os.path.join(_SCENE_GEN, "_bakelab", "bake_corr.html"))
    args = ap.parse_args()
    out = build(os.path.abspath(args.out))
    print(f"{out}  ({os.path.getsize(out)/1e3:.0f} KB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
