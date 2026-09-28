#!/usr/bin/env python3
"""Post-flight analysis of a TIGRIS search run (works on MTL runs too).

    python3 scripts/analyze_tigris_run.py --run-dir runs/latest
    python3 scripts/analyze_tigris_run.py --run-dir runs/<mtl_run> --rewards-only   # score an MTL run with both rewards

Two parts:

1. **Identical team analysis to MTL.** Exactly what ``scripts/analyze_mtl_run.py`` does,
   through the same ``mtl_metrics_logger`` code: ``<run-dir>/{telemetry.csv,
   detection.json, residual_belief.csv, report.html}`` with the residual belief mass
   (flown and planned), targets found, time to detect, cell coverage and distance.
   ``--rewards-only`` skips it (the files of the run are left untouched).

2. **Both TIGRIS rewards, always** (whatever reward the planner optimised):
   * ``original`` - TIGRIS's entropy reward on the per-cell presence belief (tpr/fpr
     Bayes update, Rs/Rf weights), one update per cell per ``edge_m`` of flown path;
   * ``matched``  - the searched belief mass = 1 - residual belief, on the planning grid,
     plus the logger's own residual curve (2 m raster) as the reference;
   each for the FLOWN telemetry and for the PLANNED track (track.json). Writes
   ``<run-dir>/tigris_rewards.json``, ``tigris_rewards.csv`` (flown),
   ``tigris_rewards_planned.csv`` and ``tigris_report.html`` (charts + the replan table
   from ``<robot>/tigris_replans.json``).

The reward parameters (edge_m = extend_dist_m, grid_res_m, use_entropy, rs, rf,
initial_confidence) are read from the run's ``tigris_replans.json`` when there is one,
otherwise the TIGRIS defaults are used, so an MTL run and a TIGRIS run with default
parameters are scored identically. Stdlib only (plus the repo's own modules).
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
for sub in ("robot/ros_ws/src/behavior/mtl_metrics_logger",
            "robot/ros_ws/src/global/planners/tigris_search_planner",
            "scripts"):
    sys.path.insert(0, str(REPO / sub))

from mtl_metrics_logger.analysis import planned_looks_from_track, write_run_outputs  # noqa: E402
from mtl_metrics_logger.report import read_telemetry_csv  # noqa: E402
from tigris_search_planner.rewards import (Detection, Grid, RewardParams, look_from_gimbal,  # noqa: E402
                                           look_from_points, score_looks)

import analyze_mtl_run  # noqa: E402  (load_run: same input resolution as the MTL analysis)

DEFAULT_CONFIG = REPO / "stacks/tigris_search/config"
DEFAULTS = {"extend_dist_m": 60.0, "grid_res_m": 4.0, "use_entropy": True, "rs": 2.0, "rf": 1.0,
            "initial_confidence": 0.01}


# --------------------------------------------------------------------------- inputs
def agent_dirs(run_dir: Path) -> list[Path]:
    return sorted(d for d in run_dir.iterdir() if d.is_dir() and
                  ((d / "telemetry.csv").is_file() or (d / "track.json").is_file()))


def load_replans(run_dir: Path) -> dict[str, dict]:
    out = {}
    for d in agent_dirs(run_dir):
        p = d / "tigris_replans.json"
        if p.is_file():
            out[d.name] = json.loads(p.read_text(encoding="utf-8"))
    return out


def reward_params(replans: dict[str, dict], args) -> dict:
    prm = dict(DEFAULTS)
    for rp in replans.values():
        p = rp.get("params") or {}
        for k in DEFAULTS:
            if k in p:
                prm[k] = p[k]
        break
    if args.edge_m is not None:
        prm["extend_dist_m"] = args.edge_m
    if args.grid_res_m is not None:
        prm["grid_res_m"] = args.grid_res_m
    return prm


def find_scenario(run_dir: Path, explicit: Path | None) -> Path:
    cands = [explicit, *(d / "scenario.json" for d in agent_dirs(run_dir)), run_dir / "scenario.json",
             DEFAULT_CONFIG / "scenario.json", REPO / "stacks/mtl_search/config/scenario.json"]
    for c in cands:
        if c is not None and c.is_file():
            return c
    raise SystemExit("cannot find scenario.json (pass --scenario)")


# --------------------------------------------------------------------------- rewards
def flown_samples(run_dir: Path, sc: dict, det: Detection) -> list[tuple]:
    fov = math.radians(float(sc["sensor"]["fov_deg"]))
    samples = []
    for d in agent_dirs(run_dir):
        tel = d / "telemetry.csv"
        if not tel.is_file():
            continue
        rows = [r for r in read_telemetry_csv(tel) if r.get("agent") in (None, "", d.name)
                and r.get("t") is not None and r.get("x_world") is not None]
        rows.sort(key=lambda r: r["t"])
        arc, last, last_t = 0.0, None, None
        for r in rows:
            p = (r["x_world"], r["y_world"], r["z_world"])
            if last is not None:
                arc += math.hypot(p[0] - last[0], p[1] - last[1])
            dt = 0.0 if last_t is None else max(r["t"] - last_t, 0.0)
            last, last_t = p, r["t"]
            measured = r.get("gimbal_measured", 1) and r.get("meas_pitch") is not None and r.get("meas_yaw") is not None
            pitch = r.get("meas_pitch") if measured else r.get("cmd_pitch")
            yaw = r.get("meas_yaw") if measured else r.get("cmd_yaw")
            look = look_from_gimbal(p[0], p[1], p[2], pitch, yaw, fov, det, dt / det.dt_ref) if dt > 0 else None
            samples.append((r["t"], arc, look, d.name))
    samples.sort(key=lambda s: s[0])
    return samples


def planned_samples(run_dir: Path, sc: dict, det: Detection) -> list[tuple]:
    fov = math.radians(float(sc["sensor"]["fov_deg"]))
    samples = []
    for d in agent_dirs(run_dir):
        tp = d / "track.json"
        if not tp.is_file():
            continue
        tr = json.loads(tp.read_text(encoding="utf-8"))
        looks = planned_looks_from_track(tr)
        arcs = tr["samples"].get("arc") or []
        t = looks["t"]
        for k in range(len(t)):
            dt = (t[k] - t[k - 1]) if k else 0.0
            look = look_from_points(looks["pos"][k], looks["bore"][k], fov, det, dt / det.dt_ref) if dt > 0 else None
            samples.append((t[k], arcs[k] if k < len(arcs) else 0.0, look, d.name))
    samples.sort(key=lambda s: s[0])
    return samples


def decimate(curve: dict, keep: int = 600) -> dict:
    n = len(curve["t_s"])
    if n <= keep:
        return curve
    step = n / keep
    idx = sorted({int(i * step) for i in range(keep)} | {n - 1})
    return {k: [v[i] for i in idx] for k, v in curve.items()}


# --------------------------------------------------------------------------- report
CSS = """
.viz-root{color-scheme:light;--surface-1:#fcfcfb;--surface-2:#f4f3ef;--text-primary:#0b0b0b;
--text-secondary:#52514e;--text-muted:#7c7b76;--grid:#e6e5e0;--series-1:#2a78d6;--series-2:#eb6834;
--series-3:#1baf7a;--border:#dddcd6}
@media (prefers-color-scheme:dark){:root:where(:not([data-theme="light"])) .viz-root{color-scheme:dark;
--surface-1:#1a1a19;--surface-2:#242422;--text-primary:#fff;--text-secondary:#c3c2b7;--text-muted:#8f8e86;
--grid:#34332f;--series-1:#3987e5;--series-2:#d95926;--series-3:#199e70;--border:#3a3935}}
:root[data-theme="dark"] .viz-root{color-scheme:dark;--surface-1:#1a1a19;--surface-2:#242422;--text-primary:#fff;
--text-secondary:#c3c2b7;--text-muted:#8f8e86;--grid:#34332f;--series-1:#3987e5;--series-2:#d95926;
--series-3:#199e70;--border:#3a3935}
body{margin:0;background:var(--surface-1)}
.viz-root{background:var(--surface-1);color:var(--text-primary);font:14px/1.45 system-ui,-apple-system,Segoe UI,sans-serif;
max-width:1100px;margin:0 auto;padding:24px 16px}
h1{font-size:22px;margin:0 0 4px}h2{font-size:16px;margin:28px 0 8px}.sub{color:var(--text-secondary);margin:0 0 16px}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(155px,1fr));gap:12px}
.tile{background:var(--surface-2);border-radius:8px;padding:12px}.tile .v{font-size:22px;font-weight:600}
.tile .l{color:var(--text-secondary);font-size:12px}
.chart{position:relative;background:var(--surface-2);border-radius:8px;padding:8px 8px 4px}
.legend{display:flex;gap:16px;flex-wrap:wrap;color:var(--text-secondary);font-size:12px;margin:4px 8px 8px}
.legend span{display:inline-flex;align-items:center;gap:6px}
.sw{display:inline-block;width:18px;height:0;border-top:2px solid}
svg text{fill:var(--text-muted);font-size:11px}
.tip{position:absolute;pointer-events:none;background:var(--surface-1);border:1px solid var(--border);border-radius:6px;
padding:6px 8px;font-size:12px;display:none;white-space:nowrap;box-shadow:0 2px 8px rgba(0,0,0,.15)}
table{border-collapse:collapse;width:100%;font-size:12px}th,td{padding:4px 6px;border-bottom:1px solid var(--border);text-align:right}
th{color:var(--text-secondary);font-weight:600}td:first-child,th:first-child{text-align:left}
.wrap{overflow-x:auto}.note{color:var(--text-secondary);font-size:12px}
"""

JS = r"""
function drawChart(el){
  const d=JSON.parse(el.dataset.chart);const W=el.clientWidth-16,H=260,m={l:56,r:12,t:10,b:28};
  const ns='http://www.w3.org/2000/svg';const svg=document.createElementNS(ns,'svg');
  svg.setAttribute('width',W);svg.setAttribute('height',H);el.insertBefore(svg,el.querySelector('.tip'));
  let xmax=0,ymax=0;d.series.forEach(s=>{s.x.forEach(v=>xmax=Math.max(xmax,v));s.y.forEach(v=>ymax=Math.max(ymax,v))});
  xmax=xmax||1;ymax=(ymax||1)*1.08;const X=v=>m.l+(W-m.l-m.r)*v/xmax,Y=v=>H-m.b-(H-m.t-m.b)*v/ymax;
  const mk=(t,a)=>{const e=document.createElementNS(ns,t);for(const k in a)e.setAttribute(k,a[k]);svg.appendChild(e);return e};
  for(let i=0;i<=4;i++){const v=ymax*i/4;mk('line',{x1:m.l,x2:W-m.r,y1:Y(v),y2:Y(v),stroke:'var(--grid)'});
    const t=mk('text',{x:m.l-6,y:Y(v)+4,'text-anchor':'end'});t.textContent=v<10?v.toFixed(3):v.toFixed(0)}
  for(let i=0;i<=5;i++){const v=xmax*i/5;const t=mk('text',{x:X(v),y:H-8,'text-anchor':'middle'});t.textContent=v.toFixed(0)+' s'}
  const used=[];
  d.series.forEach(s=>{const p=s.x.map((v,i)=>(i?'L':'M')+X(v).toFixed(1)+' '+Y(s.y[i]).toFixed(1)).join('');
    mk('path',{d:p,fill:'none',stroke:s.color,'stroke-width':2,'stroke-dasharray':s.dash||'','stroke-linejoin':'round'});
    const n=s.x.length-1;if(n<0)return;const lx=Math.min(X(s.x[n])+4,W-m.r-2);let ly=Y(s.y[n])-6;
    while(used.some(u=>Math.abs(u.x-lx)<160&&Math.abs(u.y-ly)<13))ly+=13;used.push({x:lx,y:ly});
    const t=mk('text',{x:lx,y:ly,'text-anchor':'end'});t.textContent=s.label});
  const cross=mk('line',{y1:m.t,y2:H-m.b,stroke:'var(--text-muted)','stroke-width':1,visibility:'hidden'});
  const tip=el.querySelector('.tip');
  svg.addEventListener('pointermove',ev=>{const r=svg.getBoundingClientRect();const px=ev.clientX-r.left;
    const xv=Math.max(0,Math.min(xmax,(px-m.l)/(W-m.l-m.r)*xmax));cross.setAttribute('x1',X(xv));cross.setAttribute('x2',X(xv));
    cross.setAttribute('visibility','visible');tip.replaceChildren();const h=document.createElement('div');
    h.textContent='t = '+xv.toFixed(1)+' s';h.style.color='var(--text-secondary)';tip.appendChild(h);
    d.series.forEach(s=>{let i=0;while(i<s.x.length-1&&s.x[i+1]<=xv)i++;const row=document.createElement('div');
      const b=document.createElement('b');b.textContent=(s.y[i]??0).toFixed(d.digits);row.appendChild(b);
      row.appendChild(document.createTextNode('  '+s.label));tip.appendChild(row)});
    tip.style.display='block';tip.style.left=Math.min(px+14,W-190)+'px';tip.style.top='12px'});
  svg.addEventListener('pointerleave',()=>{tip.style.display='none';cross.setAttribute('visibility','hidden')});
}
document.querySelectorAll('.chart[data-chart]').forEach(drawChart);
"""


def chart_block(title: str, series: list[dict], digits: int, note: str) -> str:
    payload = html.escape(json.dumps({"series": series, "digits": digits}), quote=True)
    legend = "".join(f'<span><i class="sw" style="border-top-color:{s["color"]};'
                     f'{"border-top-style:dashed;" if s.get("dash") else ""}"></i>{html.escape(s["label"])}</span>'
                     for s in series)
    return (f'<h2>{html.escape(title)}</h2><div class="legend">{legend}</div>'
            f'<div class="chart" data-chart="{payload}"><div class="tip"></div></div>'
            f'<p class="note">{html.escape(note)}</p>')


def write_report(path: Path, run_name: str, data: dict) -> None:
    fl, pl = data["flown"], data["planned"]
    s = data["summary"]

    def ser(label, curve, key, color, dash=None):
        c = decimate(curve)
        return {"label": label, "x": c["t_s"], "y": c[key], "color": color, "dash": dash}

    orig = []
    matched = []
    if fl["t_s"]:
        orig.append(ser("flown", fl, "original", "var(--series-1)"))
        matched.append(ser("flown (planning grid)", fl, "matched", "var(--series-1)"))
    if pl["t_s"]:
        orig.append(ser("planned", pl, "original", "var(--series-2)", "6 4"))
        matched.append(ser("planned (planning grid)", pl, "matched", "var(--series-2)", "6 4"))
    ev = data.get("evaluator_flown")
    if ev and ev["t_s"]:
        matched.append({"label": "flown (logger, 2 m raster)", "x": decimate(ev)["t_s"],
                        "y": decimate(ev)["searched"], "color": "var(--series-3)"})

    def tile(v, label):
        return f'<div class="tile"><div class="v">{html.escape(v)}</div><div class="l">{html.escape(label)}</div></div>'

    fmt = lambda v, d=4: "n/a" if v is None else f"{v:.{d}f}"  # noqa: E731
    tiles = "".join([
        tile(fmt(s.get("residual_belief_mass")), "residual belief mass, flown (lower is better)"),
        tile(fmt(s.get("planned_residual_belief_mass")), "residual belief mass, planned"),
        tile(fmt(s.get("flown_original"), 1), "TIGRIS original reward, flown"),
        tile(fmt(s.get("planned_original"), 1), "TIGRIS original reward, planned"),
        tile(fmt(s.get("flown_matched")), "matched reward (searched mass), flown"),
        tile(fmt(s.get("planned_matched")), "matched reward (searched mass), planned"),
    ])
    rows = []
    for agent, rp in data.get("replans", {}).items():
        for r in rp.get("replans", []):
            rows.append("<tr>" + "".join(f"<td>{html.escape(str(v))}</td>" for v in (
                agent, r.get("index"), r.get("trigger"), f'{r.get("progress_arc_m", 0):.1f}',
                f'{r.get("commit_arc_m", 0):.1f}', f'{r.get("budget_left_m", 0):.1f}', f'{r.get("planning_s", 0):.2f}',
                r.get("iterations"), r.get("tree_size"), "yes" if r.get("improved") else "no",
                f'{r.get("segment_length_m", 0):.1f}', f'{(r.get("segment_reward") or {}).get("original", 0):.2f}',
                f'{(r.get("segment_reward") or {}).get("matched", 0):.4f}',
                f'{r.get("residual_mass_after_flown", 0):.4f}')) + "</tr>")
    table = ""
    if rows:
        head = "".join(f"<th>{h}</th>" for h in ("agent", "solve", "trigger", "progress m", "commit m", "budget left m",
                                                 "planning s", "iterations", "tree", "new path", "segment m",
                                                 "reward original", "reward matched", "residual after flown"))
        table = (f'<h2>Replans (receding horizon)</h2><div class="wrap"><table><thead><tr>{head}</tr></thead>'
                 f'<tbody>{"".join(rows)}</tbody></table></div>'
                 '<p class="note">Segment rewards are computed on the planner\'s belief at that solve (after the flown '
                 'and committed looks); "residual after flown" is the planner\'s residual belief mass on its grid.</p>')
    p = data["params"]
    body = (f'<div class="viz-root"><h1>TIGRIS rewards — {html.escape(run_name)}</h1>'
            f'<p class="sub">planner: {html.escape(data.get("planner", "?"))} · reward optimised: '
            f'{html.escape(str(data.get("reward_mode", "n/a")))} · pass length {p["extend_dist_m"]:g} m · grid '
            f'{p["grid_res_m"]:g} m · Rs {p["rs"]:g} / Rf {p["rf"]:g} · entropy {p["use_entropy"]}</p>'
            f'<div class="tiles">{tiles}</div>'
            + chart_block("Original TIGRIS reward (entropy reduction) vs time", orig, 1,
                          "Per-cell presence belief, tpr/fpr Bayes update with the scenario sigmoid, one update per cell per "
                          f"{p['extend_dist_m']:g} m pass at the best range of the pass; steps at the end of each pass.")
            + chart_block("Matched reward (searched belief mass = 1 - residual) vs time", matched, 4,
                          "Every look multiplies the belief by (1 - P(r))^(dt/dt_ref). The logger curve is the "
                          "authoritative residual metric on the 2 m prior raster; the grid curves are the planner's model.")
            + table + "<p class='note'>Full detection report (map, targets, prior vs residual): report.html.</p></div>")
    doc = (f'<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" '
           f'content="width=device-width,initial-scale=1"><title>TIGRIS rewards</title><style>{CSS}</style></head>'
           f'<body>{body}<script>{JS}</script></body></html>')
    path.write_text(doc, encoding="utf-8")


def write_csv(path: Path, curve: dict) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "reward_original", "reward_matched"])
        for t, o, m in zip(curve["t_s"], curve["original"], curve["matched"]):
            w.writerow([t, o, m])


# --------------------------------------------------------------------------- main
def team_analysis(run_dir: Path, out: Path, args) -> dict:
    sc, gt, rows, planned, png, sc_path, gt_path = analyze_mtl_run.load_run(run_dir, args.scenario, args.ground_truth)
    res = write_run_outputs(
        out, scenario=sc, ground_truth=gt, rows_by_agent=rows, planned_by_agent=planned,
        title=f"TIGRIS search — {run_dir.name}",
        subtitle=f"scenario {sc['mission']['name']} · {len(rows)} agent(s): {', '.join(sorted(rows))} · "
                 f"fused on one timeline",
        belief_png=png,
        extra={"run_id": run_dir.name, "inputs": {"scenario": str(sc_path), "ground_truth": str(gt_path)}})
    s = res["summary"]
    mttd = s["mean_time_to_discovery_s"]
    resid, planned_resid = s.get("residual_belief_mass"), s.get("planned_residual_belief_mass")
    print(f"[analyze_tigris_run] {run_dir.name}: residual belief "
          f"{'n/a' if resid is None else f'{resid:.4f}'} = P(target missed), lower is better"
          f"{'' if planned_resid is None else f' (planned {planned_resid:.4f})'}; "
          f"{100 * (s.get('searched_belief_fraction') or 0):.1f} % of the prior searched")
    print(f"  {s['targets_detected']}/{s['targets_total']} targets found"
          f"{'' if mttd is None else f', mean time to discovery {mttd:.1f} s'}; "
          f"valid cells reached {s['cells_covered']}/{s['cells_total']} "
          f"({100 * s['belief_mass_fraction']:.1f} % of their mass) over {s['total_path_length_m'] / 1000:.2f} km")
    return s


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-dir", type=Path, default=REPO / "runs" / "latest")
    ap.add_argument("--out-dir", type=Path, default=None, help="default: the run dir itself")
    ap.add_argument("--scenario", type=Path, default=None)
    ap.add_argument("--ground-truth", type=Path, default=None)
    ap.add_argument("--rewards-only", action="store_true", help="skip the team analysis (part 1)")
    ap.add_argument("--edge-m", type=float, default=None, help="original-reward pass length (default: extend_dist_m)")
    ap.add_argument("--grid-res-m", type=float, default=None)
    args = ap.parse_args(argv)

    run_dir = args.run_dir.resolve()
    if not run_dir.is_dir():
        raise SystemExit(f"run dir not found: {args.run_dir}")
    out = (args.out_dir or run_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)

    summary = {}
    if not args.rewards_only:
        summary = team_analysis(run_dir, out, args)
    elif (out / "detection.json").is_file():
        summary = json.loads((out / "detection.json").read_text()).get("summary", {})

    sc = json.loads(find_scenario(run_dir, args.scenario).read_text(encoding="utf-8"))
    replans = load_replans(run_dir)
    prm = reward_params(replans, args)
    det = Detection.from_scenario(sc)
    rp = RewardParams(use_entropy=bool(prm["use_entropy"]), rs=float(prm["rs"]), rf=float(prm["rf"]),
                      initial_confidence=float(prm["initial_confidence"]))
    grid = Grid.from_scenario(sc, res=float(prm["grid_res_m"]), initial_confidence=rp.initial_confidence)
    edge = float(prm["extend_dist_m"])
    flown = score_looks(grid, det, rp, flown_samples(run_dir, sc, det), edge).as_dict()
    planned = score_looks(grid, det, rp, planned_samples(run_dir, sc, det), edge).as_dict()

    evaluator = None
    det_json = out / "detection.json"
    if det_json.is_file():
        curves = json.loads(det_json.read_text()).get("curves", {})
        t, r = curves.get("t_s") or [], curves.get("residual_mass") or []
        pts = [(a, 1.0 - b) for a, b in zip(t, r) if b is not None]
        if pts:
            evaluator = {"t_s": [a for a, _ in pts], "searched": [round(b, 6) for _, b in pts]}

    planner = "tigris" if replans else "unknown (no tigris_replans.json: MTL or other)"
    mode = next((rp_["params"].get("reward_mode") for rp_ in replans.values() if rp_.get("params")), None)
    summ = {
        "residual_belief_mass": summary.get("residual_belief_mass"),
        "planned_residual_belief_mass": summary.get("planned_residual_belief_mass"),
        "flown_original": flown["original"][-1] if flown["original"] else None,
        "flown_matched": flown["matched"][-1] if flown["matched"] else None,
        "planned_original": planned["original"][-1] if planned["original"] else None,
        "planned_matched": planned["matched"][-1] if planned["matched"] else None,
        "solves": sum(len(r.get("replans", [])) for r in replans.values()),
    }
    data = {"schema": "tigris.rewards/1", "run_id": run_dir.name, "planner": planner, "reward_mode": mode,
            "params": prm, "summary": summ, "flown": flown, "planned": planned,
            "evaluator_flown": evaluator, "replans": replans}
    (out / "tigris_rewards.json").write_text(json.dumps(data, indent=1), encoding="utf-8")
    write_csv(out / "tigris_rewards.csv", flown)
    write_csv(out / "tigris_rewards_planned.csv", planned)
    write_report(out / "tigris_report.html", run_dir.name, data)
    f = lambda v, d=4: "n/a" if v is None else f"{v:.{d}f}"  # noqa: E731
    print(f"  TIGRIS rewards: original {f(summ['flown_original'], 1)} flown / {f(summ['planned_original'], 1)} planned;"
          f" matched (searched mass) {f(summ['flown_matched'])} flown / {f(summ['planned_matched'])} planned"
          + (f"; {summ['solves']} TIGRIS solves" if summ["solves"] else ""))
    for name in ("telemetry.csv", "detection.json", "residual_belief.csv", "report.html", "tigris_rewards.json",
                 "tigris_rewards.csv", "tigris_rewards_planned.csv", "tigris_report.html"):
        if (out / name).is_file():
            print(f"  wrote {out / name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
