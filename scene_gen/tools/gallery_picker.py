#!/usr/bin/env python3
"""gallery_picker — click the rungs you want to hand-edit, then open them.

    AirStack/.venv/bin/python scene_gen/tools/gallery_picker.py
    # then open http://localhost:8765

Serves the archetype library as a clickable grid: a ROW per asset, a COLUMN
per damage rung, the bake's own preview in every cell. Click cells to select
them; the button at the bottom hands the selection to `tools/edit_usds.py`
inside the container, which is the same flow as typing the file list by hand —
just without typing thirteen paths.

WHY A BROWSER AND NOT A WINDOW
------------------------------
`tkinter` is available and there is a display, so a native window was possible.
A grid of ~250 previews is the deciding factor: HTML scrolls, reflows and
scales it for free, where a Tk canvas needs all of that written by hand. It
also works over an SSH tunnel, which a Tk window does not.

WHAT THE BADGES MEAN
--------------------
    (no badge)   straight bake output
    edited       `hand_edited_at` is set — you have posed this one
    subst        `substituted_from` is set — it is a copy of another rung

Those come from the manifest, so the picker never disagrees with the library
about what has been touched.

SAFE WHILE A BAKE RUNS. It only ever READS the manifest and the preview PNGs.
The one action it takes is launching the editor, and `edit_usds.py` writes
`.orig.usd` backups before anything else.
"""

from __future__ import annotations

import argparse
import html
import io
import json
import os
import subprocess
import sys
import urllib.parse
from http.server import BaseHTTPRequestHandler, HTTPServer

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from archetypes import library as lib                          # noqa: E402
from disaster import levels as L                               # noqa: E402

#: Container path for the repo, for the launch command.
CONTAINER_REPO = "/isaac-sim/AirStack"

STATE = {"lib_dir": "", "thumbs": {}, "config": ""}


def _rungs(doc):
    seen = {str(r.get("level")) for r in doc.get("archetypes", ())}
    try:
        order = list(L.bake_levels(str(doc.get("disaster") or "none"),
                                   L.STRUCTURE))
    except Exception:                                            # noqa: BLE001
        order = []
    out = [x for x in order if x in seen]
    return out + sorted(seen - set(out))


def _model():
    d = STATE["lib_dir"]
    doc = lib.read_manifest(os.path.join(d, lib.MANIFEST_NAME))
    by = {}
    for r in doc.get("archetypes", ()):
        if r.get("kind") != "structure":
            continue
        by.setdefault(str(r["type"]), {})[str(r["level"])] = r

    def used(t):
        for r in by[t].values():
            if r.get("used_by"):
                return sum(int(v) for v in r["used_by"].values())
        return 0

    types = sorted(by, key=lambda t: (-used(t), t))
    return doc, by, types, _rungs(doc), used


def _stamp(rel_png):
    """`mtime_size` for a preview, or "" if it is not there.

    This is the cache key, and it has to be: previews are REGENERATED all the
    time — `tools/repreview.py` rewrites them after every hand-edit, material
    change or substitution, to the same filename. Keyed on the name alone the
    picker served the pre-edit thumbnail for the rest of the session and the
    grid quietly stopped matching the library.
    """
    p = os.path.join(STATE["lib_dir"], rel_png)
    try:
        st = os.stat(p)
        return f"{int(st.st_mtime)}_{st.st_size}"
    except OSError:
        return ""


def _thumb(rel_png, w=260):
    """A cached JPEG thumbnail, invalidated when the PNG changes.

    The previews are 1280x720 PNGs and a page of 250 of them is ~400 MB over
    the wire; this makes it ~4 MB. The cache is keyed on the file's
    mtime+size so a re-rendered preview replaces its thumbnail.
    """
    stamp = _stamp(rel_png)
    if not stamp:
        return None
    key = (rel_png, w, stamp)
    if key in STATE["thumbs"]:
        return STATE["thumbs"][key]
    from PIL import Image
    p = os.path.join(STATE["lib_dir"], rel_png)
    im = Image.open(p).convert("RGB")
    im.thumbnail((w, w), Image.LANCZOS)
    buf = io.BytesIO()
    im.save(buf, "JPEG", quality=78)
    # Drop any older entry for the same preview so the cache cannot grow
    # without bound across a long editing session.
    for k in [k for k in STATE["thumbs"] if k[0] == rel_png and k != key]:
        STATE["thumbs"].pop(k, None)
    STATE["thumbs"][key] = buf.getvalue()
    return STATE["thumbs"][key]


def _page():
    doc, by, types, rungs, used = _model()
    n = sum(len(v) for v in by.values())
    rows = []
    for t in types:
        cells = []
        for lv in rungs:
            rec = by[t].get(lv)
            if not rec:
                cells.append(
                    f'<td class="cell miss" data-type="{html.escape(t)}" '
                    f'data-level="{html.escape(lv)}" onclick="tog(this)">'
                    f'not baked</td>')
                continue
            png = (rec.get("preview") or {}).get("obl") or ""
            badge = ""
            if rec.get("substituted_from"):
                badge = '<span class="b subst">subst</span>'
            elif rec.get("hand_edited_at"):
                badge = '<span class="b edited">edited</span>'
            fn = html.escape(rec.get("usd", ""))
            # `?v=` is the file's mtime+size: same URL for unchanged
            # previews (so the browser keeps caching), a new URL the moment
            # one is re-rendered.
            img = (f'<img loading="lazy" src="/thumb?p={urllib.parse.quote(png)}'
                   f'&v={_stamp(png)}">'
                   if png else '<div class="noimg">no preview</div>')
            cells.append(
                f'<td class="cell" data-usd="{fn}" '
                f'data-type="{html.escape(t)}" data-level="{html.escape(lv)}" '
                f'onclick="tog(this)">'
                f'{img}{badge}'
                f'<div class="cap">{rec.get("usd_mb", 0):.0f} MB</div></td>')
        u = used(t)
        rows.append(f'<tr><th>{html.escape(t)}'
                    f'<div class="sub">{"placed " + str(u) + "x" if u else "stock"}'
                    f'</div></th>{"".join(cells)}</tr>')
    head = "".join(f"<th>{html.escape(r)}</th>" for r in rungs)
    return f"""<!doctype html><meta charset="utf-8">
<title>archetype picker</title>
<style>
 body{{background:#111;color:#ddd;font:13px/1.4 system-ui,sans-serif;margin:0;padding:12px 12px 90px}}
 h1{{font-size:18px;margin:0 0 2px}} .meta{{color:#888;margin-bottom:12px}}
 table{{border-collapse:separate;border-spacing:4px}}
 th{{color:#ccc;font-weight:600;text-align:left;vertical-align:middle;max-width:190px}}
 th .sub{{color:#7a7;font-weight:400;font-size:11px}}
 thead th{{color:#e8a860;text-transform:uppercase;font-size:11px}}
 .cell{{position:relative;background:#1b1b1a;border:2px solid #262625;border-radius:4px;
        cursor:pointer;padding:0;line-height:0}}
 .cell img{{display:block;border-radius:2px}}
 .cell.sel{{border-color:#4da3ff;box-shadow:0 0 0 2px #4da3ff55}}
 .cell.miss{{color:#a55;font-size:11px;line-height:1.4;padding:18px 10px;cursor:pointer;text-align:center}}
 .noimg{{color:#777;font-size:11px;line-height:1.4;padding:26px 10px;text-align:center}}
 .cap{{position:absolute;left:4px;bottom:3px;color:#cfcfcf;font-size:10px;
       line-height:1;text-shadow:0 1px 2px #000}}
 .b{{position:absolute;right:4px;top:4px;font-size:10px;padding:1px 5px;border-radius:3px;line-height:1.5}}
 .edited{{background:#2f6b34;color:#dfd}} .subst{{background:#6b4a2f;color:#fed}}
 #bar{{position:fixed;left:0;right:0;bottom:0;background:#1a1a19;border-top:1px solid #333;
       padding:10px 14px;display:flex;gap:14px;align-items:center}}
 button{{background:#2b6cb0;color:#fff;border:0;border-radius:4px;padding:8px 14px;
         font-size:13px;cursor:pointer}} button:disabled{{background:#333;color:#777}}
 #cmd{{color:#8a8;font-family:ui-monospace,monospace;font-size:11px;overflow:auto;white-space:nowrap;flex:1}}
</style>
<h1>archetype picker</h1>
<div class="meta">{n} archetypes · {len(types)} types · {html.escape(str(doc.get('asset_pack','')))} /
 {html.escape(str(doc.get('disaster','')))} — click cells to select</div>
<table><thead><tr><th></th>{head}</tr></thead><tbody>{''.join(rows)}</tbody></table>
<div id="bar">
  <button id="go" disabled onclick="go()">Open 0 in Isaac</button>
  <button id="rb" disabled onclick="rebake()">Rebake 0</button>
  <button onclick="clr()">Clear</button>
  <label id="expl" title="Split every mesh into its disconnected pieces so
they can be posed one at a time; they are merged back on save. Off opens the
merged meshes, which is lighter in the viewport.">
    <input type="checkbox" id="explode" checked> explode pieces</label>
  <button id="sb" style="display:none" onclick="subst()">Substitute</button>
  <button id="swap" style="display:none" onclick="swap()">&#8646;</button>
  <span id="stxt" style="color:#e8a860"></span>
  <div id="cmd"></div>
</div>
<script>
// Selection is keyed on (type, level), not on the filename: a rung the settle
// REJECTED has no file and no manifest record, and substituting into one is
// exactly what it is for. Map keeps click order, which is what decides
// destination vs source below.
const sel=new Map();
const key=td=>td.dataset.type+'\u001f'+td.dataset.level;
let flip=false;
function tog(td){{const k=key(td);
  if(sel.has(k)){{sel.delete(k);td.classList.remove('sel')}}
  else{{sel.set(k,{{t:td.dataset.type,l:td.dataset.level,u:td.dataset.usd||''}});
        td.classList.add('sel')}}
  upd()}}
function clr(){{sel.clear();flip=false;
  document.querySelectorAll('.cell.sel').forEach(e=>e.classList.remove('sel'));upd()}}
function pair(){{ // exactly two rungs of the SAME asset
  if(sel.size!==2)return null;
  const v=[...sel.values()];
  if(v[0].t!==v[1].t)return null;
  return flip?[v[1],v[0]]:[v[0],v[1]];}}
function upd(){{
  const b=document.getElementById('go');
  const withFile=[...sel.values()].filter(x=>x.u);
  b.textContent='Open '+withFile.length+' in Isaac'; b.disabled=!withFile.length;
  const r=document.getElementById('rb');
  r.textContent='Rebake '+sel.size; r.disabled=sel.size===0;
  const p=pair(), sb=document.getElementById('sb'), st=document.getElementById('stxt');
  if(p){{sb.style.display='inline'; document.getElementById('swap').style.display='inline';
    st.textContent=p[0].l+'  \u2190  '+p[1].l+'   ('+p[0].t+')';}}
  else{{sb.style.display='none'; document.getElementById('swap').style.display='none';
    st.textContent=sel.size===2?'pick two rungs of the SAME asset':'';}}
  document.getElementById('cmd').textContent='';}}
function swap(){{flip=!flip;upd();}}
function go(){{const names=[...sel.values()].filter(x=>x.u).map(x=>x.u);
  fetch('/open',{{method:'POST',body:JSON.stringify(
    {{names:names, explode:document.getElementById('explode').checked}})}})
  .then(r=>r.text()).then(t=>{{document.getElementById('cmd').textContent=t;}})}}
function rebake(){{
  if(!confirm('Rebake '+sel.size+' rung(s)? This OVERWRITES them, including '
      +'any hand edits or substitutions.')) return;
  document.getElementById('cmd').textContent='launching rebake\u2026';
  fetch('/rebake',{{method:'POST',body:JSON.stringify(
    {{cells:[...sel.values()].map(x=>[x.t,x.l])}})}})
  .then(r=>r.text()).then(t=>{{document.getElementById('cmd').textContent=t;}})}}
function subst(){{const p=pair(); if(!p)return;
  if(!confirm('Replace '+p[0].t+' '+p[0].l+' with a copy of '+p[1].l
      +'?  The destination is overwritten (a backup is kept '
      +'the first time).'))return;
  document.getElementById('cmd').textContent='substituting\u2026';
  fetch('/substitute',{{method:'POST',body:JSON.stringify(
    {{type:p[0].t, dest:p[0].l, src:p[1].l}})}})
  .then(r=>r.text()).then(t=>{{document.getElementById('cmd').textContent=t;}})}}
</script>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="text/plain; charset=utf-8"):
        b = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        if ctype.startswith("text/html"):
            self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        if u.path == "/":
            return self._send(200, _page(), "text/html; charset=utf-8")
        if u.path == "/thumb":
            q = urllib.parse.parse_qs(u.query).get("p", [""])[0]
            data = _thumb(q)
            if data is None:
                return self._send(404, "no such preview")
            return self._send(200, data, "image/jpeg")
        return self._send(404, "not found")

    def _substitute(self, body):
        """Stand one rung in for another, then re-shoot the destination.

        Runs `tools/substitute_rung.py` on the HOST -- it is pure file copy
        plus a manifest edit and needs no Kit -- and only then asks Kit for a
        new preview, because a substituted rung whose picture is not re-taken
        shows the geometry it replaced and looks like the swap did nothing.
        """
        t = str(body.get("type") or "")
        dest = str(body.get("dest") or "")
        src = str(body.get("src") or "")
        if not (t and dest and src) or dest == src:
            return self._send(400, "need a type and two different rungs")

        root = os.path.relpath(os.path.dirname(STATE["lib_dir"]), _SCENE_GEN)
        disaster = os.path.basename(STATE["lib_dir"])
        try:
            r = subprocess.run(
                [sys.executable, os.path.join("tools", "substitute_rung.py"),
                 t, dest, src, "--disaster", disaster, "--root", root],
                cwd=_SCENE_GEN, capture_output=True, text=True, timeout=600)
        except Exception as exc:                                 # noqa: BLE001
            return self._send(200, f"substitute failed to run: {exc}")
        out = (r.stdout or "") + (r.stderr or "")
        if r.returncode != 0:
            return self._send(200, f"substitute FAILED:\n{out.strip()}")

        usd = f"{root}/{disaster}/{t}_{dest}.usd"
        shot = (f'clear; cd {CONTAINER_REPO}/scene_gen && '
                f'/isaac-sim/python.sh tools/repreview.py {usd}')
        note = "preview re-shooting in the isaac-sim 'edit' window"
        try:
            subprocess.run(["docker", "exec", "isaac-sim", "tmux", "send-keys",
                            "-t", "isaac:edit", shot, "ENTER"],
                           check=True, capture_output=True, timeout=20)
        except Exception as exc:                                 # noqa: BLE001
            note = f"could not launch repreview ({exc}); run:\n{shot}"
        lines = [ln for ln in out.splitlines() if ln.startswith("[subst]")]
        return self._send(200, "\n".join(lines + [note]))

    def _rebake(self, names):
        """Re-cut exactly the selected rungs, in their own tmux window.

        *names* is a list of ``(type, level)`` pairs straight from the page.
        Deriving them from the filename is not possible -- both a type slug and
        a level name contain underscores (`SM_Building_24`,
        `partial_collapse`) -- and looking them up in the manifest excluded the
        rungs most worth re-cutting: one the settle REJECTED has no record.
        """
        cells = [f"{t}:{lv}" for t, lv in names]
        unknown = []
        if not cells:
            return self._send(200, "nothing to rebake")
        # THE ARCHETYPES ROOT, not the disaster directory. `library.disaster_dir`
        # appends the disaster name to whatever --out is given, so passing
        # `assets/archetypes/earthquake` writes to
        # `assets/archetypes/earthquake/earthquake` -- a bake that runs to
        # completion, reports success, and leaves the real library untouched.
        # Measured 2026-08-30: 19 archetypes and 1 GB into a stray nested dir.
        out = os.path.relpath(os.path.dirname(STATE["lib_dir"]), _SCENE_GEN)
        cmd = (f'clear; cd {CONTAINER_REPO}/scene_gen && '
               f'SCENE_ARCHETYPES=1 /isaac-sim/python.sh '
               f'archetypes/bake_cli.py --config {STATE["config"]} '
               f'--out {out} --cells {",".join(cells)}')
        try:
            # A SEPARATE WINDOW, never `isaac:0`. That is where a long bake
            # runs, and sending keys to it would either interleave with a
            # running bake or type a command into its stdin.
            subprocess.run(["docker", "exec", "isaac-sim", "tmux",
                            "new-window", "-d", "-t", "isaac", "-n", "rebake"],
                           capture_output=True, timeout=20)
            subprocess.run(["docker", "exec", "isaac-sim", "tmux",
                            "send-keys", "-t", "isaac:rebake", cmd, "ENTER"],
                           check=True, capture_output=True, timeout=20)
        except Exception as exc:                                 # noqa: BLE001
            return self._send(200, f"could not launch ({exc}); run it "
                                   f"yourself:\n{cmd}")
        warn = (f"  NOTE: {len(unknown)} not in manifest, skipped"
                if unknown else "")
        return self._send(200, f"rebaking {len(cells)} rung(s) in the "
                               f"isaac-sim 'rebake' window.{warn}\n"
                               f"If the main bake is still running they share "
                               f"one GPU and both write manifest.json.\n{cmd}")

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(n) or b"[]")
        except Exception:                                        # noqa: BLE001
            return self._send(400, "bad selection")
        # Tolerate the bare list the page used to post, so a stale tab still
        # works rather than silently opening nothing.
        explode = True
        cells = []
        if isinstance(body, dict):
            names = body.get("names") or []
            cells = body.get("cells") or []
            explode = bool(body.get("explode", True))
        else:
            names = body
        if not names and not cells and not isinstance(body, dict):
            return self._send(400, "nothing selected")
        route = urllib.parse.urlparse(self.path).path
        if route == "/substitute":
            return self._substitute(body)
        if route == "/rebake":
            return self._rebake(cells)
        if not names:
            return self._send(400, "nothing selected")
        rel = os.path.relpath(STATE["lib_dir"], _SCENE_GEN)
        args = " ".join(f"{rel}/{x}" for x in names)
        cmd = (f'clear; cd {CONTAINER_REPO}/scene_gen && '
               f'PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh '
               f'tools/edit_usds.py {args}'
               f'{"" if explode else " --no-explode"}')
        try:
            subprocess.run(["docker", "exec", "isaac-sim", "tmux",
                            "send-keys", "-t", "isaac:edit", cmd, "ENTER"],
                           check=True, capture_output=True, timeout=20)
        except Exception as exc:                                 # noqa: BLE001
            return self._send(200, f"could not launch ({exc}); run it yourself:"
                                   f"\n{cmd}")
        return self._send(200, f"launched {len(names)} archetype(s) in the "
                               f"isaac-sim 'edit' window"
                               f"{'' if explode else ' (meshes NOT exploded)'}"
                               f" — close that window "
                               f"when done and the split runs automatically")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("library", nargs="?",
                    default=os.path.join(_SCENE_GEN, "assets", "archetypes",
                                         "earthquake"))
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--config", default="urban_v3_quake",
                    help="scene config the Rebake button bakes against; it "
                         "decides the knobs a re-cut rung is cut with, so it "
                         "must be the one the library was baked from")
    args = ap.parse_args()
    STATE["lib_dir"] = os.path.abspath(args.library)
    STATE["config"] = args.config
    if not os.path.isfile(os.path.join(STATE["lib_dir"], lib.MANIFEST_NAME)):
        raise SystemExit(f"no manifest in {STATE['lib_dir']}")
    doc, by, types, rungs, _u = _model()
    print(f"[picker] {STATE['lib_dir']}")
    print(f"[picker] {len(types)} types x {len(rungs)} rungs")
    print(f"[picker] http://localhost:{args.port}  (Ctrl-C to stop)")
    HTTPServer(("0.0.0.0", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
