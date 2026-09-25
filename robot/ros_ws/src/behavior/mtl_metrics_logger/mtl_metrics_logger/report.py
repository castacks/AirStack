"""Run outputs for MTL search flights: telemetry CSV, detection JSON, HTML report.

Pure Python (stdlib only) so the same code writes the per-robot files inside the
robot container (``mtl_metrics_logger``) and the fused team files on the host
(``scripts/analyze_mtl_run.py``).

The HTML report is one self-contained file (inline CSS/JS/SVG, the belief
texture embedded as a data URI): open it straight from the run folder.
"""

from __future__ import annotations

import base64
import csv
import html
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

__all__ = ["TELEMETRY_COLUMNS", "write_telemetry_csv", "read_telemetry_csv", "write_json",
           "build_report_data", "render_report_html", "write_report"]

TELEMETRY_COLUMNS = [
    "t", "agent", "state",
    "x_map", "y_map", "z_map", "x_world", "y_world", "z_world", "n", "e", "d",
    "yaw", "vx", "vy", "vz", "speed",
    "cmd_roll", "cmd_pitch", "cmd_yaw", "meas_roll", "meas_pitch", "meas_yaw", "gimbal_measured",
    "bore_x_world", "bore_y_world", "slant_m", "footprint_r_m",
    "progress_m", "remaining_m", "xte_m", "carrot_x_map", "carrot_y_map", "carrot_z_map",
    "aim_x_world", "aim_y_world", "pointing_error_m",
    "stamp",  # absolute ROS time [s] (sim time in Isaac): aligns robots / bags
]


def _fmt(v: Any) -> Any:
    if isinstance(v, float):
        if math.isnan(v) or math.isinf(v):
            return ""
        return f"{v:.4f}"
    return "" if v is None else v


def write_telemetry_csv(path: str | Path, rows: Iterable[Mapping[str, Any]],
                        columns: Sequence[str] = TELEMETRY_COLUMNS) -> int:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    n = 0
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(columns)
        for r in rows:
            w.writerow([_fmt(r.get(c)) for c in columns])
            n += 1
    return n


def read_telemetry_csv(path: str | Path) -> list[dict[str, Any]]:
    out = []
    with Path(path).open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            row: dict[str, Any] = {}
            for k, v in r.items():
                if k in ("agent", "state"):
                    row[k] = v
                elif v == "" or v is None:
                    row[k] = None
                else:
                    try:
                        row[k] = float(v)
                    except ValueError:
                        row[k] = v
            out.append(row)
    return out


def write_json(path: str | Path, data: Any) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=1, default=_json_default) + "\n", encoding="utf-8")


def _json_default(o: Any) -> Any:
    if isinstance(o, set):
        return sorted(o)
    if isinstance(o, float) and (math.isnan(o) or math.isinf(o)):
        return None
    raise TypeError(f"not JSON serialisable: {type(o).__name__}")


def _thin(points: Sequence[Sequence[float]], step_m: float = 1.5) -> list[list[float]]:
    out: list[list[float]] = []
    for p in points:
        if p is None or any(v is None for v in p[:2]):
            continue
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= step_m:
            out.append([round(p[0], 2), round(p[1], 2)])
    if points and out and points[-1] is not None and list(points[-1][:2]) != out[-1]:
        out.append([round(points[-1][0], 2), round(points[-1][1], 2)])
    return out


def _thin_series(t: Sequence[float], ys: Sequence[Sequence[Any]], max_points: int = 600):
    n = len(t)
    if n <= max_points:
        idx = list(range(n))
    else:
        stride = n / max_points
        idx = sorted({int(k * stride) for k in range(max_points)} | {n - 1})
    return [round(t[i], 2) for i in idx], [[None if y[i] is None else round(float(y[i]), 4) for i in idx]
                                            for y in ys]


def build_report_data(*, title: str, subtitle: str, summary: Mapping[str, Any],
                      targets: Sequence[Mapping[str, Any]], area: Mapping[str, float],
                      cells: Sequence[Mapping[str, Any]], agents: Sequence[Mapping[str, Any]],
                      curves: Mapping[str, Sequence[float]], planned_mass: float | None,
                      per_agent: Mapping[str, Mapping[str, Sequence[Any]]],
                      belief_png: bytes | None = None, notes: Sequence[str] = ()) -> dict[str, Any]:
    """Assemble the report payload (world ENU coordinates throughout).

    ``agents``: ``[{"name", "home": [x, y], "planned": [[x, y]...], "flown": [[x, y]...],
    "bore": [[x, y]...]}]``. ``curves``: team ``t``, ``detected``, ``distance``, ``mass``.
    ``per_agent[name]``: ``t``, ``xte``, ``pointing_error``, ``cmd_pitch``, ``meas_pitch``.
    """
    t, (det, dist, mass) = _thin_series(curves.get("t", []),
                                        [curves.get("detected", []), curves.get("distance", []),
                                         curves.get("mass", [])])
    pa = {}
    for name, s in per_agent.items():
        tt, (xte, perr, cp, mp) = _thin_series(s.get("t", []), [s.get("xte", []), s.get("pointing_error", []),
                                                                 s.get("cmd_pitch", []), s.get("meas_pitch", [])])
        pa[name] = {"t": tt, "xte": xte, "pointing_error": perr,
                    "cmd_pitch_deg": [None if v is None else round(math.degrees(v), 2) for v in cp],
                    "meas_pitch_deg": [None if v is None else round(math.degrees(v), 2) for v in mp]}
    return {
        "title": title, "subtitle": subtitle, "summary": dict(summary),
        "planned_mass": planned_mass, "targets": list(targets), "area": dict(area),
        "cells": list(cells),
        "agents": [{"name": a["name"], "home": a.get("home"),
                    "planned": _thin(a.get("planned", []), 3.0), "flown": _thin(a.get("flown", []), 1.5),
                    "bore": _thin(a.get("bore", []), 4.0)} for a in agents],
        "curves": {"t": t, "detected": det, "distance": dist, "mass": mass},
        "per_agent": pa,
        "belief_png": ("data:image/png;base64," + base64.b64encode(belief_png).decode()) if belief_png else None,
        "notes": list(notes),
    }


def write_report(path: str | Path, data: Mapping[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render_report_html(data), encoding="utf-8")


def render_report_html(data: Mapping[str, Any]) -> str:
    payload = json.dumps(data, default=_json_default, separators=(",", ":")).replace("</", "<\\/")
    return (_TEMPLATE.replace("__TITLE__", html.escape(str(data.get("title", "MTL run"))))
            .replace("__PAYLOAD__", payload))


_TEMPLATE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>__TITLE__</title>
<style>
:root{color-scheme:light;--page:#f9f9f7;--surface:#fcfcfb;--ink:#0b0b0b;--ink2:#52514e;--muted:#898781;
--grid:#e1e0d9;--axis:#c3c2b7;--ring:rgba(11,11,11,.10);--s1:#2a78d6;--s2:#eb6834;--s3:#1baf7a;
--good:#0ca30c;--critical:#d03b3b;--ref:#898781}
@media (prefers-color-scheme:dark){:root:where(:not([data-theme="light"])){color-scheme:dark;--page:#0d0d0d;
--surface:#1a1a19;--ink:#fff;--ink2:#c3c2b7;--grid:#2c2c2a;--axis:#383835;--ring:rgba(255,255,255,.10);
--s1:#3987e5;--s2:#d95926;--s3:#199e70}}
:root[data-theme="dark"]{color-scheme:dark;--page:#0d0d0d;--surface:#1a1a19;--ink:#fff;--ink2:#c3c2b7;
--grid:#2c2c2a;--axis:#383835;--ring:rgba(255,255,255,.10);--s1:#3987e5;--s2:#d95926;--s3:#199e70}
*{box-sizing:border-box}body{margin:0;background:var(--page);color:var(--ink);
font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:1180px;margin:0 auto;padding:24px 16px 48px}
h1{font-size:22px;margin:0 0 2px}h2{font-size:15px;margin:0 0 10px}.sub{color:var(--ink2);margin:0 0 18px}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:12px;margin-bottom:16px}
.tile,.card{background:var(--surface);border:1px solid var(--ring);border-radius:10px;padding:14px}
.tile .v{font-size:26px;font-weight:600}.tile .l{color:var(--ink2);font-size:12px}
.grid2{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:12px;margin-bottom:12px}
.card{min-width:0}svg{display:block;width:100%;height:auto;overflow:visible}
.legend{display:flex;flex-wrap:wrap;gap:12px;color:var(--ink2);font-size:12px;margin:6px 0 0}
.legend span{display:inline-flex;align-items:center;gap:6px}.key{width:16px;height:0;border-top:2px solid}
.key.dash{border-top-style:dashed}.dot{width:9px;height:9px;border-radius:50%}
table{border-collapse:collapse;width:100%;font-size:12.5px;font-variant-numeric:tabular-nums}
th,td{text-align:left;padding:5px 8px;border-bottom:1px solid var(--grid)}th{color:var(--ink2);font-weight:600}
.tw{overflow-x:auto}.ok{color:var(--good);font-weight:600}.no{color:var(--critical);font-weight:600}
#tip{position:fixed;pointer-events:none;background:var(--surface);color:var(--ink);border:1px solid var(--ring);
border-radius:8px;padding:8px 10px;font-size:12px;box-shadow:0 4px 14px rgba(0,0,0,.12);display:none;z-index:9;
max-width:260px}#tip b{font-size:13px}#tip .r{display:flex;align-items:center;gap:6px}
.notes{color:var(--ink2);font-size:12px}.muted{color:var(--muted)}
@media (max-width:520px){.grid2{grid-template-columns:1fr}}
</style></head><body><main id="app"></main><div id="tip"></div>
<script id="payload" type="application/json">__PAYLOAD__</script>
<script>
(function(){
"use strict";
const D=JSON.parse(document.getElementById("payload").textContent);
const app=document.getElementById("app"),tip=document.getElementById("tip");
const NS="http://www.w3.org/2000/svg";
const SER=["var(--s1)","var(--s2)","var(--s3)"];
function el(tag,attrs,parent,ns){const e=ns?document.createElementNS(NS,tag):document.createElement(tag);
 for(const k in (attrs||{})){if(k==="text")e.textContent=attrs[k];else e.setAttribute(k,attrs[k]);}
 if(parent)parent.appendChild(e);return e;}
function s(tag,attrs,parent){return el(tag,attrs,parent,true);}
function fmt(v,d){if(v===null||v===undefined||Number.isNaN(v))return "—";return Number(v).toFixed(d===undefined?1:d);}
function showTip(ev,rows,title){tip.textContent="";if(title)el("div",{text:title,class:"muted"},tip);
 rows.forEach(r=>{const d=el("div",{class:"r"},tip);if(r.color)el("span",{class:"key",style:"border-color:"+r.color},d);
 el("b",{text:r.value},d);el("span",{text:" "+r.label,class:"muted"},d);});
 tip.style.display="block";const x=Math.min(ev.clientX+14,innerWidth-270),y=Math.min(ev.clientY+14,innerHeight-120);
 tip.style.left=x+"px";tip.style.top=y+"px";}
function hideTip(){tip.style.display="none";}
const agents=D.agents.map(a=>a.name);const colorOf=n=>SER[Math.max(0,agents.indexOf(n))%SER.length];

// ---------------- header + KPI tiles ----------------
el("h1",{text:D.title},app);el("p",{class:"sub",text:D.subtitle},app);
const S=D.summary,tiles=el("div",{class:"tiles"},app);
function tile(v,l){const t=el("div",{class:"tile"},tiles);el("div",{class:"v",text:v},t);el("div",{class:"l",text:l},t);}
tile(S.targets_detected+" / "+S.targets_total,"targets found (P_det ≥ "+S.detection_threshold+")");
tile(S.mean_time_to_discovery_s==null?"—":fmt(S.mean_time_to_discovery_s,0)+" s","mean time to discovery");
tile(fmt(S.belief_mass_per_km,0),"belief mass per km flown");
const realized=S.belief_mass_covered, planned=D.planned_mass;
tile(planned?fmt(100*realized/planned,0)+" %":fmt(100*S.belief_mass_fraction,0)+" %",
     planned?"realized / planned coverage (mass)":"belief mass covered");
tile(fmt(S.total_path_length_m/1000,2)+" km","team track flown");

// ---------------- map ----------------
const mapCard=el("div",{class:"card"},el("div",{class:"grid2"},app));
el("h2",{text:"Search area — planned vs flown, targets"},mapCard);
(function(){const A=D.area,W=560,pad=8;
 // Extent = search area UNION every planned/flown point and home (tracks may leave the
 // area, e.g. the planner's straight run-out past the last cluster), plus a margin.
 const E={x0:A.x_min,x1:A.x_max,y0:A.y_min,y1:A.y_max};
 D.agents.forEach(a=>[].concat(a.planned||[],a.flown||[],a.home?[a.home]:[]).forEach(p=>{
  E.x0=Math.min(E.x0,p[0]);E.x1=Math.max(E.x1,p[0]);E.y0=Math.min(E.y0,p[1]);E.y1=Math.max(E.y1,p[1]);}));
 const mg=0.02*Math.max(E.x1-E.x0,E.y1-E.y0);E.x0-=mg;E.x1+=mg;E.y0-=mg;E.y1+=mg;
 const H=Math.round(Math.max(320,Math.min(900,2*pad+(E.y1-E.y0)*(W-2*pad)/(E.x1-E.x0))));
 const sc=Math.min((W-2*pad)/(E.x1-E.x0),(H-2*pad)/(E.y1-E.y0));
 const ox=(W-2*pad-(E.x1-E.x0)*sc)/2,oy=(H-2*pad-(E.y1-E.y0)*sc)/2;
 const X=x=>pad+ox+(x-E.x0)*sc,Y=y=>H-pad-oy-(y-E.y0)*sc;
 const svg=s("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"map of the search area",style:"overflow:hidden"},mapCard);
 if(D.belief_png&&A.belief){const b=A.belief;s("image",{href:D.belief_png,x:X(b.x_min),y:Y(b.y_max),width:(b.x_max-b.x_min)*sc,
  height:(b.y_max-b.y_min)*sc,preserveAspectRatio:"none",opacity:"0.85"},svg);}
 if(A.box){const b=A.box;s("rect",{x:X(b.x_min),y:Y(b.y_max),width:(b.x_max-b.x_min)*sc,height:(b.y_max-b.y_min)*sc,
  fill:"none",stroke:"#f2c200","stroke-width":1.5},svg);}
 const cs=(D.area.cell_size||20)*sc;
 D.cells.forEach(c=>{if(!c.covered)return;s("rect",{x:X(c.x)-cs/2+1,y:Y(c.y)-cs/2+1,width:cs-2,height:cs-2,fill:"none",
  stroke:"var(--ink)","stroke-opacity":.35,"stroke-width":1},svg);});
 D.agents.forEach((a,i)=>{const col=SER[i%3];
  const line=(pts,attrs)=>{if(pts.length<2)return;s("polyline",Object.assign({points:pts.map(p=>X(p[0])+","+Y(p[1])).join(" "),
   fill:"none",stroke:col,"stroke-width":2,"stroke-linejoin":"round","stroke-linecap":"round"},attrs),svg);};
  line(a.planned,{"stroke-dasharray":"5 4","stroke-opacity":.9,"stroke-width":1.5});
  line(a.flown,{"stroke-width":2});
  if(a.home){s("rect",{x:X(a.home[0])-5,y:Y(a.home[1])-5,width:10,height:10,rx:2,fill:col,stroke:"var(--surface)","stroke-width":2},svg);}
 });
 D.targets.forEach(t=>{const cx=X(t.x),cy=Y(t.y);const g=s("g",{tabindex:0},svg);
  if(t.detected){s("circle",{cx,cy,r:6,fill:"var(--good)",stroke:"var(--surface)","stroke-width":2},g);}
  else{s("circle",{cx,cy,r:5.5,fill:"var(--surface)",stroke:"var(--critical)","stroke-width":2},g);
   s("path",{d:"M"+(cx-3)+" "+(cy-3)+"L"+(cx+3)+" "+(cy+3)+"M"+(cx+3)+" "+(cy-3)+"L"+(cx-3)+" "+(cy+3),stroke:"var(--critical)","stroke-width":1.6},g);}
  s("circle",{cx,cy,r:13,fill:"transparent"},g);
  const rows=[{value:fmt(t.detection_prob,3),label:"P_det"},{value:t.detected?fmt(t.detection_time_s,1)+" s":"not found",label:"discovery"}];
  if(t.responsible_agent)rows.push({value:t.responsible_agent,label:"found by",color:colorOf(t.responsible_agent)});
  const h=ev=>showTip(ev,rows,"target "+t.index);g.addEventListener("pointermove",h);g.addEventListener("focus",e=>{const r=g.getBoundingClientRect();h({clientX:r.right,clientY:r.top});});
  g.addEventListener("pointerleave",hideTip);g.addEventListener("blur",hideTip);});
 const lg=el("div",{class:"legend"},mapCard);
 D.agents.forEach((a,i)=>{const sp=el("span",{},lg);el("span",{class:"key",style:"border-color:"+SER[i%3]},sp);el("span",{text:a.name+" flown"},sp);});
 const sp=el("span",{},lg);el("span",{class:"key dash",style:"border-color:var(--muted)"},sp);el("span",{text:"planned"},sp);
 const f=el("span",{},lg);el("span",{class:"dot",style:"background:var(--good)"},f);el("span",{text:"✓ found"},f);
 const m=el("span",{},lg);el("span",{class:"dot",style:"border:2px solid var(--critical)"},m);el("span",{text:"✕ missed"},m);
})();

// ---------------- line charts ----------------
function lineChart(parent,title,t,series,opts){opts=opts||{};const card=el("div",{class:"card"},parent);el("h2",{text:title},card);
 const W=520,H=230,L=46,R=12,T=10,B=30;const svg=s("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":title},card);
 const xs=opts.x||t;const xmin=Math.min(...xs),xmax=Math.max(...xs,xmin+1e-9);
 let ymax=opts.ymax;if(ymax===undefined){ymax=0;series.forEach(se=>se.y.forEach(v=>{if(v!=null&&v>ymax)ymax=v;}));if(opts.ref)ymax=Math.max(ymax,opts.ref.value);ymax=ymax>0?ymax*1.08:1;}
 const ymin=opts.ymin||0;const X=x=>L+(x-xmin)/(xmax-xmin)*(W-L-R),Y=y=>H-B-(y-ymin)/(ymax-ymin)*(H-T-B);
 for(let k=0;k<=4;k++){const v=ymin+(ymax-ymin)*k/4,y=Y(v);s("line",{x1:L,x2:W-R,y1:y,y2:y,stroke:k?"var(--grid)":"var(--axis)","stroke-width":1},svg);
  s("text",{x:L-6,y:y+4,"text-anchor":"end","font-size":11,fill:"var(--muted)",text:fmt(v,ymax<10?1:0)},svg);}
 for(let k=0;k<=4;k++){const v=xmin+(xmax-xmin)*k/4;s("text",{x:X(v),y:H-B+16,"text-anchor":"middle","font-size":11,fill:"var(--muted)",text:fmt(v,0)},svg);}
 s("text",{x:W-R,y:H-2,"text-anchor":"end","font-size":11,fill:"var(--muted)",text:opts.xlabel||"time [s]"},svg);
 if(opts.ref){const y=Y(opts.ref.value);s("line",{x1:L,x2:W-R,y1:y,y2:y,stroke:"var(--ref)","stroke-dasharray":"4 4","stroke-width":1.2},svg);
  s("text",{x:W-R,y:y-5,"text-anchor":"end","font-size":11,fill:"var(--ink2)",text:opts.ref.label},svg);}
 series.forEach(se=>{let d="",pen=false;se.y.forEach((v,i)=>{if(v==null){pen=false;return;}
  if(opts.step&&pen){d+="L"+X(xs[i])+" "+Y(se.y[i-1]);}d+=(pen?"L":"M")+X(xs[i])+" "+Y(v);pen=true;});
  s("path",{d,fill:"none",stroke:se.color,"stroke-width":2,"stroke-linejoin":"round","stroke-dasharray":se.dash||""},svg);});
 const hair=s("line",{y1:T,y2:H-B,stroke:"var(--axis)","stroke-width":1,visibility:"hidden"},svg);
 const hit=s("rect",{x:L,y:T,width:W-L-R,height:H-T-B,fill:"transparent"},svg);
 hit.addEventListener("pointermove",ev=>{const r=svg.getBoundingClientRect();const px=(ev.clientX-r.left)/r.width*W;
  const xv=xmin+(px-L)/(W-L-R)*(xmax-xmin);let bi=0,bd=Infinity;xs.forEach((x,i)=>{const d=Math.abs(x-xv);if(d<bd){bd=d;bi=i;}});
  hair.setAttribute("x1",X(xs[bi]));hair.setAttribute("x2",X(xs[bi]));hair.setAttribute("visibility","visible");
  showTip(ev,series.map(se=>({value:fmt(se.y[bi],opts.digits===undefined?1:opts.digits),label:se.name,color:se.color})),
   (opts.xlabel||"t")+" = "+fmt(xs[bi],1));});
 hit.addEventListener("pointerleave",()=>{hair.setAttribute("visibility","hidden");hideTip();});
 if(series.length>1||opts.ref){const lg=el("div",{class:"legend"},card);series.forEach(se=>{const sp=el("span",{},lg);
  el("span",{class:"key"+(se.dash?" dash":""),style:"border-color:"+se.color},sp);el("span",{text:se.name},sp);});}
 return card;}
const C=D.curves,g1=el("div",{class:"grid2"},app);
lineChart(g1,"Targets found over time (team)",C.t,[{name:"targets found",y:C.detected,color:"var(--s1)"}],{step:true,digits:0,ymax:Math.max(1,S.targets_total)});
lineChart(g1,"Belief mass swept vs distance flown",C.t,[{name:"covered mass",y:C.mass,color:"var(--s1)"}],
 {x:C.distance,xlabel:"team distance [m]",ref:D.planned_mass?{value:D.planned_mass,label:"planned "+fmt(D.planned_mass,0)}:null,digits:0});
const PA=D.per_agent,names=Object.keys(PA),g2=el("div",{class:"grid2"},app);
if(names.length){
 // one shared x (the first agent's t) keeps the crosshair honest; agents are sampled on the team timeline upstream
 const tt=PA[names[0]].t;
 lineChart(g2,"Cross-track error",tt,names.map(n=>({name:n,y:PA[n].xte,color:colorOf(n)})),{digits:2});
 lineChart(g2,"Pointing error (boresight vs scheduled point)",tt,names.map(n=>({name:n,y:PA[n].pointing_error,color:colorOf(n)})),{digits:2});
 const g3=el("div",{class:"grid2"},app);
 names.forEach(n=>lineChart(g3,"Gimbal pitch — "+n+" [deg]",PA[n].t,[{name:"commanded",y:PA[n].cmd_pitch_deg,color:"var(--s1)"},
  {name:"measured",y:PA[n].meas_pitch_deg,color:"var(--s2)",dash:"5 3"}],{digits:1,ymin:-20,ymax:110}));}

// ---------------- tables ----------------
const tc=el("div",{class:"card"},app);el("h2",{text:"Targets"},tc);const tw=el("div",{class:"tw"},tc);const tb=el("table",{},tw);
const hr=el("tr",{},el("thead",{},tb));["#","x [m]","y [m]","P_det","found","discovery [s]","first seen [s]","found by","observed by","looks","min range [m]"].forEach(h=>el("th",{text:h},hr));
const body=el("tbody",{},tb);D.targets.forEach(t=>{const r=el("tr",{},body);
 [t.index,fmt(t.x,1),fmt(t.y,1),fmt(t.detection_prob,3)].forEach(v=>el("td",{text:String(v)},r));
 el("td",{text:t.detected?"✓ yes":"✕ no",class:t.detected?"ok":"no"},r);
 [fmt(t.detection_time_s,1),fmt(t.first_seen_s,1),t.responsible_agent||"—",(t.observed_by||[]).join(", ")||"—",t.observations,fmt(t.min_range_m,1)].forEach(v=>el("td",{text:String(v)},r));});
const ac=el("div",{class:"card",style:"margin-top:12px"},app);el("h2",{text:"Summary"},ac);const at=el("table",{},el("div",{class:"tw"},ac));
Object.keys(S).forEach(k=>{const r=el("tr",{},at);el("th",{text:k},r);const v=S[k];el("td",{text:typeof v==="object"&&v!==null?JSON.stringify(v):String(v)},r);});
if(D.notes&&D.notes.length){const n=el("div",{class:"card notes",style:"margin-top:12px"},app);D.notes.forEach(x=>el("p",{text:x},n));}
})();
</script></body></html>
"""
