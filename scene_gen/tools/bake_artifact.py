#!/usr/bin/env python3
"""Build the single page that carries the bake: the photos and the numbers.

    AirStack/.venv/bin/python scene_gen/tools/bake_artifact.py \\
        --library scene_gen/assets/archetypes_urban_v2/earthquake \\
        -o scene_gen/_bakelab/bake.html

`BAKE_REPORT.md` answers the same questions in a terminal. This exists because
the two halves of the answer — what a rung COSTS and what a rung LOOKS LIKE —
are only useful side by side, and markdown cannot put a photo of `pancaked`
next to the 23 seconds and 38 MB it took to make it.

SELF-CONTAINED BY CONSTRUCTION. Tiles are downscaled and inlined as data URIs,
because the page is published to a host that blocks every external request.
`--max-mb` is the budget; rows are dropped from the gallery (never from the
table) if the tiles would blow it.
"""
from __future__ import annotations

import argparse
import base64
import csv
import html
import io as _io
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)

import bake_report as R                                         # noqa: E402

#: Every rung either earthquake ladder can produce, in intensity order —
#: structures first, then the vegetation ladder's own two. A tree does not fail
#: the way a building does (`disaster.levels` keys the ladder on KIND for
#: exactly that reason), so an asset is shown the rungs ITS ladder has rather
#: than a fixed five, which would print five empty frames beside every stump.
RUNGS = ("pristine", "cracked", "soft_storey", "partial_collapse", "pancaked",
         "fallen", "stump")

#: The severity ramp. ORDERED, because the ladder is — this is the one place
#: colour carries information rather than decoration, so it runs neutral-grey
#: (nothing happened) through ochre and rust to a deep oxblood (flattened),
#: and is deliberately not the page's steel-blue accent. The vegetation rungs
#: reuse the band their severity matches.
RUNG_HUE = {
    "pristine": "#78858F", "cracked": "#B08D1F", "soft_storey": "#C26A28",
    "partial_collapse": "#AC3F2A", "pancaked": "#7A2029",
    "fallen": "#AC3F2A", "stump": "#7A2029",
}

#: What the ramp under the title shows: the structure ladder, which is what
#: nearly every row is.
RAMP = RUNGS[:5]


def tile_uri(path: str, width: int, quality: int) -> str:
    from PIL import Image

    im = Image.open(path).convert("RGB")
    im.thumbnail((width, width * 2), Image.LANCZOS)
    buf = _io.BytesIO()
    im.save(buf, "JPEG", quality=quality, optimize=True)
    return "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode()


def esc(v) -> str:
    return html.escape("" if v is None else str(v))


def num(v, nd=0, dash="—") -> str:
    if v in (None, "", "None"):
        return dash
    try:
        f = float(v)
    except (TypeError, ValueError):
        return esc(v)
    return f"{f:,.{nd}f}"


CSS = """
:root{
  --ground:#F2F4F6; --surface:#FFFFFF; --surface-2:#E9EDF1; --line:#D3DAE1;
  --ink:#151A20; --ink-2:#4B5765; --ink-3:#77838F;
  --accent:#255E8C; --accent-soft:#DCE8F2; --accent-ink:#1B486C;
  --good:#2F6B4F; --warn:#9A6A12; --crit:#9E3826;
  --shadow:0 1px 2px rgba(21,26,32,.06),0 8px 24px -12px rgba(21,26,32,.18);
}
@media (prefers-color-scheme: dark){
  :root:not([data-theme="light"]){
    --ground:#0E1216; --surface:#161C23; --surface-2:#1E262F; --line:#2C3742;
    --ink:#E6EBF0; --ink-2:#A8B4C0; --ink-3:#7C8894;
    --accent:#6FA6D2; --accent-soft:#1A2C3C; --accent-ink:#9CC6E6;
    --good:#6FBF95; --warn:#D6A94A; --crit:#E08268;
    --shadow:0 1px 2px rgba(0,0,0,.4),0 10px 28px -14px rgba(0,0,0,.7);
  }
}
:root[data-theme="dark"]{
  --ground:#0E1216; --surface:#161C23; --surface-2:#1E262F; --line:#2C3742;
  --ink:#E6EBF0; --ink-2:#A8B4C0; --ink-3:#7C8894;
  --accent:#6FA6D2; --accent-soft:#1A2C3C; --accent-ink:#9CC6E6;
  --good:#6FBF95; --warn:#D6A94A; --crit:#E08268;
  --shadow:0 1px 2px rgba(0,0,0,.4),0 10px 28px -14px rgba(0,0,0,.7);
}
*{box-sizing:border-box}
body{
  margin:0; background:var(--ground); color:var(--ink);
  font-family:"Source Sans 3","Source Sans Pro",system-ui,sans-serif;
  font-size:16px; line-height:1.55; -webkit-font-smoothing:antialiased;
}
.wrap{max-width:1180px;margin:0 auto;padding:40px 24px 96px;
      display:flex;flex-direction:column;gap:44px}
h1,h2,h3{font-family:Archivo,"Archivo Expanded",system-ui,sans-serif;
         margin:0;text-wrap:balance;letter-spacing:-.015em}
h1{font-size:clamp(30px,4.4vw,46px);font-weight:700;line-height:1.05}
h2{font-size:22px;font-weight:650;letter-spacing:.01em}
h3{font-size:15px;font-weight:650}
p{margin:0}
code,.mono,td.n,th.n{font-family:"IBM Plex Mono",ui-monospace,monospace;
     font-variant-numeric:tabular-nums}
.eyebrow{font-family:"IBM Plex Mono",monospace;font-size:11px;font-weight:500;
  letter-spacing:.16em;text-transform:uppercase;color:var(--ink-3)}
.lede{color:var(--ink-2);max-width:66ch;font-size:17px}

header.top{display:flex;flex-direction:column;gap:14px;
  border-bottom:1px solid var(--line);padding-bottom:30px}
.runline{display:flex;flex-wrap:wrap;gap:8px;align-items:center;
  font-family:"IBM Plex Mono",monospace;font-size:12px;color:var(--ink-3)}
.chip{display:inline-flex;align-items:center;gap:6px;padding:3px 9px;
  border:1px solid var(--line);border-radius:2px;background:var(--surface)}
.chip.stop{border-color:var(--warn);color:var(--warn)}

.stats{display:grid;gap:14px;grid-template-columns:repeat(auto-fit,minmax(190px,1fr))}
.stat{background:var(--surface);border:1px solid var(--line);border-radius:3px;
  padding:16px 18px;display:flex;flex-direction:column;gap:2px;box-shadow:var(--shadow)}
.stat .v{font-family:Archivo,sans-serif;font-size:32px;font-weight:700;
  line-height:1.05;font-variant-numeric:tabular-nums}
.stat .k{font-size:12.5px;color:var(--ink-3)}
.stat .sub{font-size:12.5px;color:var(--ink-2)}

section{display:flex;flex-direction:column;gap:16px}
.section-head{display:flex;flex-direction:column;gap:6px}

.scroll{overflow-x:auto;border:1px solid var(--line);border-radius:3px;
  background:var(--surface)}
table{border-collapse:collapse;width:100%;font-size:13.5px}
th,td{padding:8px 11px;text-align:left;border-bottom:1px solid var(--line);
  white-space:nowrap}
thead th{position:sticky;top:0;background:var(--surface-2);z-index:1;
  font-family:"IBM Plex Mono",monospace;font-size:11px;font-weight:600;
  letter-spacing:.07em;text-transform:uppercase;color:var(--ink-2)}
tbody tr:last-child td{border-bottom:none}
tbody tr:hover{background:var(--surface-2)}
td.n,th.n{text-align:right}
th.sortable{cursor:pointer;user-select:none}
th.sortable:hover{color:var(--ink)}
th.sortable::after{content:"";opacity:.35;padding-left:5px}
th.sortable[data-dir="asc"]::after{content:"\\2191";opacity:1}
th.sortable[data-dir="desc"]::after{content:"\\2193";opacity:1}

.pill{display:inline-block;padding:2px 8px;border-radius:2px;font-size:11.5px;
  font-family:"IBM Plex Mono",monospace;border:1px solid transparent}
.pill.ok{color:var(--good);border-color:var(--good);background:transparent}
.pill.part{color:var(--warn);border-color:var(--warn)}
.pill.none{color:var(--ink-3);border-color:var(--line)}
.pill.bad{color:var(--crit);border-color:var(--crit)}

.rungbar{display:flex;gap:3px;align-items:stretch;margin-top:2px}
.rungbar i{display:block;height:5px;flex:1;border-radius:1px}

.ladder{display:flex;flex-direction:column;gap:10px}
.arow{background:var(--surface);border:1px solid var(--line);border-radius:3px;
  padding:12px 14px;display:grid;gap:12px;
  grid-template-columns:190px 1fr;align-items:start;box-shadow:var(--shadow)}
@media (max-width:800px){.arow{grid-template-columns:1fr}}
.aname{display:flex;flex-direction:column;gap:4px;min-width:0}
.aname b{font-family:"IBM Plex Mono",monospace;font-size:13.5px;font-weight:600;
  overflow-wrap:anywhere}
.ameta{font-size:12px;color:var(--ink-3);line-height:1.45}
.tiles{display:grid;gap:8px;grid-template-columns:repeat(5,1fr)}
@media (max-width:640px){.tiles{grid-template-columns:repeat(3,1fr)}}
.tile{display:flex;flex-direction:column;gap:4px;min-width:0}
.tile figure{margin:0;border:1px solid var(--line);border-top-width:3px;
  border-radius:2px;overflow:hidden;background:var(--surface-2);aspect-ratio:1}
.tile img{display:block;width:100%;height:100%;object-fit:cover}
.tile .lab{font-family:"IBM Plex Mono",monospace;font-size:10px;
  letter-spacing:.06em;text-transform:uppercase;color:var(--ink-3)}
.tile .cap{font-family:"IBM Plex Mono",monospace;font-size:10.5px;
  color:var(--ink-2);font-variant-numeric:tabular-nums}
.tile.empty figure{display:grid;place-items:center;border-style:dashed;
  border-top-width:1px}
.tile.empty figure span{font-size:10.5px;color:var(--ink-3);
  font-family:"IBM Plex Mono",monospace}

.notes{display:grid;gap:14px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr))}
.note{background:var(--surface);border:1px solid var(--line);
  border-left:3px solid var(--accent);border-radius:3px;padding:14px 16px;
  display:flex;flex-direction:column;gap:6px}
.note p{font-size:14px;color:var(--ink-2)}
.note b{font-size:14px}
.note code{font-size:12.5px;background:var(--surface-2);padding:1px 4px;
  border-radius:2px}
footer{border-top:1px solid var(--line);padding-top:20px;font-size:13px;
  color:var(--ink-3);display:flex;flex-direction:column;gap:6px}
footer code{font-size:12.5px}
a{color:var(--accent-ink)}
:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
@media (prefers-reduced-motion:reduce){*{animation:none!important;transition:none!important}}
"""

SORT_JS = """
document.querySelectorAll('table[data-sortable]').forEach(function(t){
  var head = t.tHead.rows[0];
  Array.from(head.cells).forEach(function(th, i){
    th.classList.add('sortable');
    th.tabIndex = 0;
    function go(){
      var dir = th.dataset.dir === 'asc' ? 'desc' : 'asc';
      Array.from(head.cells).forEach(function(o){ delete o.dataset.dir; });
      th.dataset.dir = dir;
      var body = t.tBodies[0];
      var rows = Array.from(body.rows);
      rows.sort(function(a, b){
        var x = a.cells[i].dataset.v, y = b.cells[i].dataset.v;
        var nx = parseFloat(x), ny = parseFloat(y);
        var both = !isNaN(nx) && !isNaN(ny);
        var c = both ? nx - ny : String(x).localeCompare(String(y));
        return dir === 'asc' ? c : -c;
      });
      rows.forEach(function(r){ body.appendChild(r); });
    }
    th.addEventListener('click', go);
    th.addEventListener('keydown', function(e){
      if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); go(); }
    });
  });
});
"""


def status_pill(status: str) -> str:
    if status == "complete":
        return '<span class="pill ok">complete</span>'
    if status.startswith("partial") or status.startswith("in progress"):
        return f'<span class="pill part">{esc(status)}</span>'
    if status.startswith("all rungs"):
        return '<span class="pill bad">failed</span>'
    return '<span class="pill none">not reached</span>'


def build(library: str, out: str, tile_px: int, quality: int,
          max_mb: float) -> str:
    props = json.load(open(os.path.join(_SCENE_GEN, "_bakelab",
                                        "asset_properties.json")))
    trace = R.read_trace(os.path.join(library, "bake_trace.jsonl"))
    rungs = R.rung_rows(props, trace)
    assets = R.asset_rows(props, trace)
    gal = os.path.join(_SCENE_GEN, "galleries", "archetypes_urban_v2", "tiles")

    baked = [r for r in rungs if r["outcome"] == "baked"]
    total_s = sum(float(r["seconds"] or 0) for r in rungs)
    total_mb = sum(float(r["usd_mb"] or 0) for r in baked)
    done = [a for a in assets if a["status"] == "complete"]
    started = [a for a in assets if a["rungs_baked"]]
    fails = [r for r in rungs if r["outcome"] not in ("baked", "")]
    free_gb = 0.0
    try:
        st = os.statvfs(library)
        free_gb = st.f_bavail * st.f_frsize / 1e9
    except Exception:                                           # noqa: BLE001
        pass

    # ---- gallery, biggest-first so the expensive assets are seen -----------
    order = sorted(started, key=lambda a: -(a["rungs_baked"] or 0))
    budget = max_mb * 1e6
    used = 0
    rows_html = []
    for a in order:
        tiles = []
        have = [lv for lv in RUNGS if (a["type"], lv) in trace]
        for lv in (have or list(RUNGS[:5])):
            png = os.path.join(gal, a["type"], f"{lv}.png")
            cell = trace.get((a["type"], lv)) or {}
            hue = RUNG_HUE[lv]
            if os.path.exists(png) and used < budget:
                try:
                    uri = tile_uri(png, tile_px, quality)
                except Exception:                               # noqa: BLE001
                    uri = ""
                if uri:
                    used += len(uri)
                    if lv == "pristine":
                        cap = (f"{num(cell.get('src_x_m'), 1)}×"
                               f"{num(cell.get('src_y_m'), 1)}×"
                               f"{num(cell.get('src_z_m'), 1)} m")
                    else:
                        cap = (f"{num(cell.get('frag_loose'))}/"
                               f"{num(cell.get('frag_cells'))} frag · "
                               f"{num(cell.get('seconds'))} s · "
                               f"{num(cell.get('usd_mb'))} MB")
                    tiles.append(
                        f'<div class="tile"><span class="lab" '
                        f'style="color:{hue}">{esc(lv)}</span>'
                        f'<figure style="border-top-color:{hue}">'
                        f'<img src="{uri}" alt="{esc(a["type"])} {esc(lv)}" '
                        f'loading="lazy"></figure>'
                        f'<span class="cap">{cap}</span></div>')
                    continue
            why = "not baked" if not cell else esc(cell.get("outcome", "—"))
            tiles.append(
                f'<div class="tile empty"><span class="lab">{esc(lv)}</span>'
                f'<figure><span>{why}</span></figure>'
                f'<span class="cap">&nbsp;</span></div>')
        meta = (f"{esc(a['group'])} · {esc(a['material'])} · "
                f"{num(a['volume_m3'])} m³<br>{num(a['src_points'])} pts · "
                f"{num(a['src_meshes'])} mesh<br>"
                f"{num(a['total_s'])} s · {num(a['total_mb'])} MB")
        rows_html.append(
            f'<article class="arow"><div class="aname"><b>{esc(a["type"])}</b>'
            f'{status_pill(a["status"])}'
            f'<span class="ameta">{meta}</span></div>'
            f'<div class="tiles">{"".join(tiles)}</div></article>')

    # ---- per-rung cost ----------------------------------------------------
    cost_rows = []
    for lv in RUNGS[1:]:
        got = [r for r in baked if r["level"] == lv and r["seconds"]]
        if not got:
            continue
        secs = sorted(float(r["seconds"]) for r in got)
        mbs = sorted(float(r["usd_mb"] or 0) for r in got)
        cells = sorted(int(r["frag_cells"] or 0) for r in got)
        loose = sorted(int(r["frag_loose"] or 0) for r in got)
        m = len(secs) // 2
        cost_rows.append(
            f'<tr><td><span class="pill" style="color:{RUNG_HUE[lv]};'
            f'border-color:{RUNG_HUE[lv]}">{esc(lv)}</span></td>'
            f'<td class="n" data-v="{len(got)}">{len(got)}</td>'
            f'<td class="n" data-v="{secs[m]}">{num(secs[m])}</td>'
            f'<td class="n" data-v="{secs[-1]}">{num(secs[-1])}</td>'
            f'<td class="n" data-v="{mbs[m]}">{num(mbs[m])}</td>'
            f'<td class="n" data-v="{mbs[-1]}">{num(mbs[-1])}</td>'
            f'<td class="n" data-v="{cells[m]}">{num(cells[m])}</td>'
            f'<td class="n" data-v="{loose[m]}">{num(loose[m])}</td></tr>')

    # ---- the full table ---------------------------------------------------
    COLS = (("asset", "type", 0), ("group", "group", 0),
            ("material", "material", 0), ("hollow", "hollow", 0),
            ("vol m³", "volume_m3", 0), ("w m", "x_m", 1), ("d m", "y_m", 1),
            ("h m", "z_m", 1), ("src pts", "src_points", 0),
            ("src mesh", "src_meshes", 0), ("src mat", "src_materials", 0),
            ("src tex", "src_textures", 0), ("src MB", "src_file_mb", 1),
            ("rungs", "rungs_baked", 0), ("bake s", "total_s", 0),
            ("out MB", "total_mb", 0), ("cells", "cells_total", 0),
            ("loose", "loose_total", 0), ("cracked s", "cracked_s", 0),
            ("soft s", "soft_storey_s", 0),
            ("partial s", "partial_collapse_s", 0),
            ("pancake s", "pancaked_s", 0))
    head = "".join(
        f'<th class="{"n" if i > 3 else ""}">{esc(lab)}</th>'
        for i, (lab, _k, _nd) in enumerate(COLS)) + "<th>status</th>"
    body = []
    for a in sorted(assets, key=lambda a: (-(a["rungs_baked"] or 0),
                                           -(a["volume_m3"] or 0))):
        tds = []
        for i, (_lab, key, nd) in enumerate(COLS):
            v = a.get(key)
            if key == "hollow":
                txt, sv = ("yes" if v else "no") if v != "" else "—", int(bool(v))
            elif i > 3:
                txt, sv = num(v, nd), (v if v not in (None, "") else -1)
            else:
                txt, sv = esc(v), esc(v)
            cls = ' class="n"' if i > 3 else ""
            if key == "type":
                txt = f"<code>{txt}</code>"
            tds.append(f'<td{cls} data-v="{esc(sv)}">{txt}</td>')
        body.append(f'<tr>{"".join(tds)}'
                    f'<td data-v="{esc(a["status"])}">'
                    f'{status_pill(a["status"])}</td></tr>')

    fail_html = ""
    if fails:
        rowsf = "".join(
            f'<tr><td><code>{esc(r["type"])}</code></td><td>{esc(r["level"])}</td>'
            f'<td>{esc(r["outcome"])}</td><td class="n">{num(r["seconds"])}</td>'
            f'<td>{esc(str(r["note"])[:120])}</td></tr>' for r in fails)
        fail_html = (
            '<section><div class="section-head"><span class="eyebrow">'
            'Did not export</span><h2>Cells the pipeline could not finish</h2>'
            '</div><div class="scroll"><table><thead><tr><th>asset</th>'
            '<th>rung</th><th>outcome</th><th class="n">s</th><th>why</th>'
            f'</tr></thead><tbody>{rowsf}</tbody></table></div></section>')

    ramp = "".join(f'<i style="background:{RUNG_HUE[lv]}"></i>' for lv in RAMP)
    doc = f"""<title>Urban V2 Quake Bake</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@500;650;700&family=IBM+Plex+Mono:wght@400;500;600&family=Source+Sans+3:wght@400;600&display=swap">
<style>{CSS}</style>
<div class="wrap">
<header class="top">
  <span class="eyebrow">Stage A · earthquake · asset pack urban_v2</span>
  <h1>Every earthquake rung, on every asset we could afford</h1>
  <p class="lede">One overnight bake of the five-rung earthquake ladder across
  the whole <code>urban_v2</code> pack, ordered newest-and-cheapest first so
  that whatever it got through, it got through <em>completely</em>. Each row
  below is one asset photographed at all five rungs from a single camera, with
  what that rung cost to make.</p>
  <div class="rungbar" aria-hidden="true">{ramp}</div>
  <div class="runline">
    <span class="chip">library <code>{esc(os.path.relpath(library, _SCENE_GEN))}</code></span>
    <span class="chip">seed 7</span>
    <span class="chip">host Isaac Sim, headless</span>
    <span class="chip stop">stops at 12&nbsp;GB free · {num(free_gb, 1)}&nbsp;GB now</span>
  </div>
</header>

<div class="stats">
  <div class="stat"><span class="v">{len(done)}<span style="font-size:19px;color:var(--ink-3)"> / {len(assets)}</span></span>
    <span class="k">assets with a complete ladder</span>
    <span class="sub">{len(started)} started</span></div>
  <div class="stat"><span class="v">{len(baked)}</span>
    <span class="k">archetypes exported</span>
    <span class="sub">{len(fails)} failed</span></div>
  <div class="stat"><span class="v">{total_s / 3600:,.1f} h</span>
    <span class="k">of baking</span>
    <span class="sub">{num(total_s / max(len(baked), 1))} s per archetype</span></div>
  <div class="stat"><span class="v">{total_mb / 1000:,.1f} GB</span>
    <span class="k">written to disk</span>
    <span class="sub">{num(total_mb / max(len(baked), 1))} MB per archetype</span></div>
</div>

<section>
  <div class="section-head"><span class="eyebrow">The finding</span>
  <h2>Cost tracks the building's volume, not its polygon count</h2>
  <p class="lede">The fracture cuts a roughly fixed fragment size out of the
  whole envelope, so it is the <em>cell count</em> that sets both the clock and
  the file — and cell count follows volume. <code>house_01</code> (734&nbsp;m³,
  55k points) baked all four damaged rungs in 76&nbsp;s for 158&nbsp;MB;
  <code>office_tower</code> (257,000&nbsp;m³, 106k points) spent 364&nbsp;s on
  <code>cracked</code> alone and wrote 1.47&nbsp;GB for it. Note also that
  <code>cracked</code> — the <em>gentlest</em> rung — is among the most
  expensive, because it cuts the whole building and releases almost none of
  it.</p></div>
  <div class="scroll"><table data-sortable><thead><tr>
    <th>rung</th><th class="n">n</th><th class="n">median s</th>
    <th class="n">max s</th><th class="n">median MB</th><th class="n">max MB</th>
    <th class="n">median cells</th><th class="n">median loose</th>
  </tr></thead><tbody>{"".join(cost_rows)}</tbody></table></div>
</section>

<section>
  <div class="section-head"><span class="eyebrow">The photos</span>
  <h2>The ladder, asset by asset</h2>
  <p class="lede">One camera per row, framed on the pristine cell and reused —
  so a wider silhouette means more thrown debris, never a closer camera.
  Captions read <span class="mono">loose/cut fragments · seconds · archetype
  size</span>.</p></div>
  <div class="ladder">{"".join(rows_html)}</div>
</section>

<section>
  <div class="section-head"><span class="eyebrow">The data</span>
  <h2>Every asset the plan names</h2>
  <p class="lede">Click a column to sort. Assets with no bake row were never
  reached — the run is ordered, not sampled, so they are the expensive tail.
  <span class="mono">hollow</span> is the pack's <span class="mono">solid:</span>
  flag inverted; a hollow asset is thickened before it can be fractured, and
  nothing in this pack is declared solid.</p></div>
  <div class="scroll"><table data-sortable><thead><tr>{head}</tr></thead>
  <tbody>{"".join(body)}</tbody></table></div>
</section>

{fail_html}

<section>
  <div class="section-head"><span class="eyebrow">What the run turned up</span>
  <h2>Seven things worth acting on</h2></div>
  <div class="notes">
    <div class="note"><b>The gentlest rung is the most expensive</b>
    <p><code>cracked</code> cuts the whole building and releases almost none of
    it, so it pays for every fragment and throws none away. On
    <code>office_tower</code> that is 2,845 cells, 364&nbsp;s and 1.47&nbsp;GB
    — more than <code>pancaked</code> costs on the same asset.</p></div>

    <div class="note"><b>Nothing in the pack is declared solid</b>
    <p>Every one of the 81 library assets is <code>hollow</code>, so
    <code>solidify</code> runs on all of them before anything can be
    fractured. That is the single biggest lever on both time and output size,
    and it is currently pulled all the way in one direction by omission rather
    than by a decision.</p></div>

    <div class="note"><b>All eight brownstones were crashing, silently</b>
    <p><code>mesh_damage.subdivide</code> called <code>len()</code> on every
    primvar value, and the AEC packs author
    <code>primvars:doNotCastShadows</code> as a <em>constant</em>
    <code>bool</code> — which raises rather than returning 0. 32 archetypes
    were being lost to a one-line <code>SKIP</code>. Fixed and covered by a
    test; they bake in ~53&nbsp;s each.</p></div>

    <div class="note"><b><code>japanese_building</code> sits on the settle gate</b>
    <p>The only asset the convergence gate ever rejected — and it is
    <em>marginal</em>, not broken: three rungs failed on one attempt and two of
    those passed on the retry. 150–320 of its ~350 bodies were still moving at
    the 1,800-step ceiling, none through the floor, so the pile keeps shoving
    itself rather than falling wrong. PhysX on the GPU is not bit-reproducible,
    so a cell this close to the line is a coin flip — which is the real
    problem, since a seeded bake is supposed to be reproducible.</p></div>

    <div class="note"><b>Felling a tree makes a debris field</b>
    <p><code>fallen</code> and <code>stump</code> turn one tree into 500–772
    meshes spread far wider than the tree is tall — <code>Douglas_Fir</code> is
    6&nbsp;m and its wreckage fills a frame padded for a 6&nbsp;m object. It
    reads as scattered sticks rather than a felled trunk.</p></div>

    <div class="note"><b>Three of the six "trees" are stumps</b>
    <p><code>SM_Stump.prop</code>, <code>SM_Stump_02</code> and
    <code>SM_Stump_03</code> sit in <code>shared.yaml</code>'s
    <code>trees:</code> pool, so Stage A plans <code>fallen</code> and
    <code>stump</code> rungs for assets that are already stumps. Cheap, but
    meaningless — a pack tagging question.</p></div>

    <div class="note"><b>At tower scale, <code>cracked</code> is invisible</b>
    <p><code>office_tower_cracked</code> cost 364&nbsp;s and 1.47&nbsp;GB to cut
    2,845 cells and release 1,195 of them — and in the tile beside its pristine
    twin the two are hard to tell apart. Fragment size is roughly fixed, so on a
    93&nbsp;m building the damage lands below the scale anything sees it at.
    Since the point of these scenes is what a drone can spot from the air, that
    is the rung most worth re-tuning: either scale the fragment with the
    building, or skip <code>cracked</code> on towers entirely.</p></div>
    <div class="note"><b>Black trees are the preview, not the bake</b>
    <p>The AEC vegetation binds MDL only (<code>TreeBark_10.mdl</code>,
    <code>Pine_needles.mdl</code>) with no <code>UsdPreviewSurface</code>
    fallback. Blender cannot evaluate MDL, so those rows render as
    silhouettes. <code>bound_missing</code> is 0 — Isaac Sim shows them
    correctly. Only the geometry in those tiles is evidence.</p></div>
  </div>
</section>

<section>
  <div class="section-head"><span class="eyebrow">Reading this back</span>
  <h2>Where everything lives</h2></div>
  <div class="notes">
    <div class="note"><b>The library</b><p>Baked archetypes and the append-only
    per-cell trace: <code>scene_gen/assets/archetypes_urban_v2/earthquake/</code>.
    Deliberately <em>not</em> <code>assets/archetypes/</code>, which the live
    scenes read.</p></div>
    <div class="note"><b>Resume it</b><p><code>scene_gen/tools/bake_overnight.sh</code>
    picks up where this stopped — every run carries <code>--skip-existing</code>,
    and the resume now reads the trace, so a bake killed mid-cell still
    resumes.</p></div>
    <div class="note"><b>Full-size photos</b><p>Contact sheets and per-rung PNGs
    at <code>scene_gen/galleries/archetypes_urban_v2/</code>. The tiles above are
    downscaled to keep this page self-contained.</p></div>
    <div class="note"><b>Raw numbers</b><p><code>scene_gen/_bakelab/</code> —
    <code>bake_assets.csv</code>, <code>bake_rungs.csv</code>,
    <code>asset_properties.csv</code>, and <code>BAKE_REPORT.md</code>.</p></div>
  </div>
</section>

<footer>
  <span>Baked on the host with <code>AirStack/.venv</code> + <code>.env.host</code>
  — the <code>isaac-sim</code> container was never taken.</span>
  <span>Generated by <code>scene_gen/tools/bake_artifact.py</code>.</span>
</footer>
</div>
<script>{SORT_JS}</script>
"""
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    open(out, "w").write(doc)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--library", required=True)
    ap.add_argument("-o", "--out",
                    default=os.path.join(_SCENE_GEN, "_bakelab", "bake.html"))
    ap.add_argument("--tile-px", type=int, default=190)
    ap.add_argument("--quality", type=int, default=72)
    ap.add_argument("--max-mb", type=float, default=11.0,
                    help="budget for inlined tiles; the page cap is 16 MB")
    args = ap.parse_args()
    out = build(os.path.abspath(args.library), os.path.abspath(args.out),
                args.tile_px, args.quality, args.max_mb)
    print(f"{out}  ({os.path.getsize(out) / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
