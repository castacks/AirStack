#!/usr/bin/env python
"""tear_edge_probe — WHICH BOUNDARY PIECES DID THE TEAR ACTUALLY REACH, and do
the fragments read as the wall they came off?

The offline answer to BOTH halves of the fire_dtc3 review (2026-08-30):

    "(1) the partial collapse still shows very straight cuts — see standing
     pieces .../pieces/pier_S_2_10_0119 and pier_S_3_09_0102 ...
     (2) the ragged break pieces that DO exist ... are a completely diff
     texture/color from the wall they extend"

Runs the real chain on a bare USD stage (no Kit, no Flow, no physics) — the
same `burn_gac` / `burn_building` calls `_break_material_probe.py` makes — and
then prints three tables:

  A. THE ELEMENT SPAN TABLE. Per shell element on a lost elevation inside the
     collapse band: its MEASURED footprint (`p["_size"]` for a sliced piece,
     `urban_building.PIECES` for a kit module) against what
     `fire_collapse.el_span` believes. These AGREE — `gac_slice.
     register_style` writes each sliced cell a `PIECES` row of
     `(sx, sy, sz, -sx/2, -sy/2, 0)` — and the table is here to keep it that
     way: a non-zero `dt` would mean `plan_edges` is reasoning about a
     footprint the piece does not have. What is NOT true of a sliced piece is
     that consecutive footprints BUTT (see table B): the footprint is the
     bbox of a REGION CUT and swallows sills, cornices and balcony returns,
     so the gap between two columns is 0.0-1.2 m and varies per storey.

  B. THE BOUNDARY CENSUS. Per (side, storey) of the lost region: the dead
     intervals, then every SURVIVING shell piece on that side/storey with its
     span, its `plan_edges` classes, and whether `_tear_perimeter` tore it.
     A live piece whose span BUTTS a dead interval and carries no class is a
     straight kit/slice seam left on the lip of the hole — the complaint.

  C. THE FRAGMENT MATERIAL CENSUS. Per `brk_*/frag_*`, what the prim binds
     and what its `core` subset binds, bucketed into facade / facade-tone /
     char, so "the fragments are a different colour from the wall" is a
     number and not an impression.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/tear_edge_probe.py gac:SM_Building_02:F5c 193"
    ... tear_edge_probe.py kit:apartment:F5c 7
    ... tear_edge_probe.py dtc:Amar_Tower:F5c 131

Entries are `kind:name:level[:sides]`, the shape `fire_bake.sh` uses.
"""
import random
import re
import sys
import time
import traceback
from collections import Counter

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade                        # noqa: E402
import scene_generator as sg                                  # noqa: E402
from detail import urban_building as ub                       # noqa: E402
from disaster import fracture, quake_flow as qf               # noqa: E402
from disaster import fire_collapse as fc, urban_fire as uf    # noqa: E402


def _sides(spec):
    if not spec:
        return None
    parts = spec.replace("/", ",").split(",")
    return tuple(q.strip().upper()[:1] for q in parts if q.strip()) or None


def run_kit(style, level, sides, seed):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/W")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/W"))
    UsdGeom.Scope.Define(stage, "/W/bench")
    cell = "/W/bench/k0"
    UsdGeom.Xform.Define(stage, cell)
    mats = uf.materials(stage, "/W/bench")
    pls = ub.build_building(style, 0.0, 0.0, 0.0, random.Random(seed + 7))
    sg.apply_placements(stage, pls, cell + "/parts", 1.0)
    ub.apply_glass_tint(stage, pls)
    specs = qf._mass_specs(style, 0.0, 0.0, 0.0)
    main_spec = max(specs, key=lambda q: len(q["levels"]))
    n_st = max(1, len(main_spec["levels"]))
    origin = max(0, min(n_st - 1, int(round(0.25 * (n_st - 1)))))
    ctx = uf.burn_building(stage, cell, style, pls, 0.0, 0.0, 0.0, level,
                           random.Random(seed), np.random.default_rng(seed),
                           mats, "k0", flow_root=None, origin=origin,
                           sides=sides, mat_cache={})
    return stage, cell, ctx


def run_sliced(qname, level, sides, seed):
    from disaster import gac_fire as gf
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/W")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/W"))
    UsdGeom.Scope.Define(stage, "/W/bench")
    cell = "/W/bench/g0"
    UsdGeom.Xform.Define(stage, cell)
    mats = uf.materials(stage, "/W/bench")
    ctx = gf.burn_gac(stage, cell, qname, level, random.Random(seed),
                      np.random.default_rng(seed), mats, "g0",
                      flow_root=None, mat_cache={}, ssf=1.0, sides=sides,
                      verbose=True)
    return stage, cell, ctx


# ---------------------------------------------------------------------------
def _measured_span(m, e):
    """(t0, t1) from the piece's own MEASURED extent, whatever kind it is.

    A sliced placement carries `_size` (the world bbox of the cut cell) and
    `x_m`/`y_m` at its CENTROID; a kit module is in `urban_building.PIECES`.
    This is deliberately NOT `fc.el_span` — it is what `el_span` is checked
    against.
    """
    p = e.get("p") or {}
    sz = p.get("_size")
    sd = e["side"]
    if sz:
        cx, cy = float(p.get("x_m", 0.0)), float(p.get("y_m", 0.0))
        hx, hy = 0.5 * float(sz[0]), 0.5 * float(sz[1])
        pts = [(cx - hx, cy - hy), (cx + hx, cy - hy),
               (cx + hx, cy + hy), (cx - hx, cy + hy)]
        ts = [fc.along_of(m, sd, *qf._to_local(m, wx, wy)) for wx, wy in pts]
        return min(ts), max(ts)
    return fc.el_span(m, e)


def _span_table(ctx, plan, m):
    print("[tear_probe] A. ELEMENT SPAN: measured vs fire_collapse.el_span "
          "(mass {0}, sides {1}, storeys {2}-{3})".format(
              plan["mass"], "/".join(plan["sides"]), plan["s0"],
              plan["top_storey"]))
    print("    {0:<26} {1:>3} {2:>8} {3:>8} {4:>8} {5:>8} {6:>7} {7:>7}".format(
        "element", "st", "meas_t0", "meas_t1", "el_t0", "el_t1", "dt0", "dt1"))
    bad = 0
    rows = 0
    for e in ctx["info"]["elements"]:
        if e["mass"] != plan["mass"] or e["role"] not in fc.SHELL_ROLES:
            continue
        if e["side"] not in plan["sides"]:
            continue
        if int(e["storey"]) not in set(plan["storeys"]):
            continue
        a0, a1 = _measured_span(m, e)
        b0, b1 = fc.el_span(m, e)
        d0, d1 = b0 - a0, b1 - a1
        if abs(d0) > 0.05 or abs(d1) > 0.05:
            bad += 1
        rows += 1
        if rows <= 40:
            print("    {0:<26} {1:>3} {2:8.2f} {3:8.2f} {4:8.2f} {5:8.2f} "
                  "{6:7.2f} {7:7.2f}{8}".format(
                      str(e.get("name"))[:26], e["storey"], a0, a1, b0, b1,
                      d0, d1, "   <-- WRONG" if abs(d0) > 0.05 or abs(d1) > 0.05
                      else ""))
    print("[tear_probe] {0}/{1} element(s) on a lost elevation have an "
          "el_span that does not match their measured footprint".format(bad, rows))


def _boundary_census(ctx, plan, m):
    jobs = {id(j["el"]): j for j in (plan.get("edges") or ())}
    dead = {}
    for e in plan["kill"]:
        dead.setdefault((e["side"], int(e["storey"])), []).append(
            _measured_span(m, e))
    print("[tear_probe] B. BOUNDARY CENSUS (measured spans; a live piece "
          "butting a dead interval with NO class is a straight seam)")
    n_missed = 0
    for sd in plan["sides"]:
        for s in plan["storeys"]:
            iv = sorted(dead.get((sd, s)) or [])
            live = [e for e in ctx["info"]["elements"]
                    if e["mass"] == plan["mass"] and e["side"] == sd
                    and int(e["storey"]) == s and e["role"] in fc.SHELL_ROLES
                    and id(e) not in set(id(q) for q in plan["kill"])]
            print("  {0} storey {1}: dead {2}".format(
                sd, s, " ".join("[%.1f,%.1f]" % q for q in iv) or "(none)"))
            for e in sorted(live, key=lambda q: _measured_span(m, q)[0]):
                t0, t1 = _measured_span(m, e)
                j = jobs.get(id(e))
                cls = ",".join(j["classes"]) if j else "-"
                state = ("torn" if (j and j.get("torn")) else
                         "FAILED" if (j and j.get("failed")) else
                         "dropped" if (j and j.get("dropped")) else "untouched")
                gap = min([abs(t1 - a) for a, _b in iv] +
                          [abs(t0 - b) for _a, b in iv] or [99.0]) if iv else 99.0
                flag = ""
                # 1.25 m, not `plan_edges`' own 0.6: the point of the flag is
                # to show a piece the tolerance MISSED, and the fire_dtc3 miss
                # (`pier_S_3_09_0102`, gap 0.98 m) is exactly one of those.
                # Fixed, not read off `fire_collapse`, so this probe reports
                # the same way against a pristine tree and a fixed one.
                if state == "untouched" and gap <= 1.25:
                    flag = "   <-- STRAIGHT SEAM ON THE HOLE"
                    n_missed += 1
                print("      {0:<26} [{1:6.2f},{2:6.2f}] gap {3:5.2f}  "
                      "{4:<22} {5}{6}".format(
                          str(e.get("name"))[:26], t0, t1, gap, cls, state, flag))
    print("[tear_probe] {0} surviving piece(s) butt the hole with no tear"
          .format(n_missed))
    return n_missed


_CHAR_NAMES = ("char_concrete", "soot", "soot_mid", "soot_light", "calcined",
               "burnt_metal", "void", "dark_concrete", "ash")


def _mat_kind(prim):
    m = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    if not m or not m.GetPrim().IsValid():
        return "(unbound)", "unbound"
    p = m.GetPrim()
    path, name = str(p.GetPath()), p.GetName()
    if "/FireLooks/tearmat_" in path or name.startswith("tearmat_"):
        return name, "parent_uv"        # the parent's own network, rehomed
    if "/FireLooks/tear_" in path or name.startswith("tear_"):
        return name, "facade_tone"
    if "/BurnLooks" in path or name in _CHAR_NAMES or name.startswith(
            ("Char_", "Scorch_", "Ash_")):
        return name, "char"
    if "/QuakeLooks/clad_" in path:
        return name, "clad_triplanar"
    if "/SootLooks/" in path or "/src/" in path or "/Looks/" in path:
        return name, "facade"
    return name, "other"


def _diffuse(stage, mpath):
    """(material name, diffuse source) BY PATH — a handle handed out inside a
    traversal expires, so never hold one."""
    mp = stage.GetPrimAtPath(mpath)
    if not mp or not mp.IsValid():
        return "(expired)", "(none)"
    for c in mp.GetChildren():
        sh = UsdShade.Shader(c)
        if not sh:
            continue
        for name in ("diffuseColor", "diffuse_color_constant", "base_color",
                     "diffuse_texture"):
            i = sh.GetInput(name)
            if i is None:
                continue
            try:
                if i.HasConnectedSource():
                    src = i.GetConnectedSource()[0].GetPrim()
                    f = UsdShade.Shader(src).GetInput("file")
                    v = f.Get() if f else None
                    return mp.GetName(), "tex:" + str(v).rsplit("/", 1)[-1].rstrip("@")
                v = i.Get()
            except Exception:
                continue
            if v is not None:
                return mp.GetName(), ("tex:" + str(v).rsplit("/", 1)[-1].rstrip("@")
                                      if name == "diffuse_texture"
                                      else "rgb:" + str(v))
    return mp.GetName(), "(no diffuse input)"


def _frag_dump(stage, cell, rx):
    """Every `brk_*` fragment matching `rx`, with what its prim binds. The
    row-5 review named one prim (`brk_g7_wall_E_4_06_0090/frag_001`), so the
    probe has to be able to answer about one prim."""
    print("[tear_probe] D. FRAGMENTS MATCHING %r" % rx.pattern)
    rows = []
    for p in Usd.PrimRange(stage.GetPrimAtPath(cell)):
        if not p.IsA(UsdGeom.Mesh):
            continue
        path = p.GetPath().pathString
        if "/brk_" not in path or not rx.search(path):
            continue
        mb = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
        rows.append((path, mb.GetPrim().GetPath().pathString
                     if (mb and mb.GetPrim().IsValid()) else None))
    for path, mp in rows:
        nm, df = _diffuse(stage, mp)
        print("    %-58s -> %-22s %s" % (path[-58:], nm, df))
    print("[tear_probe] %d fragment(s) matched" % len(rows))


def _frag_census(stage, cell, ctx):
    loose = set(ctx.get("loose") or ())
    static = set(ctx.get("static_extra") or ())
    rows, ex, uvbox, mats_seen = Counter(), {}, {}, set()
    st_stage = stage
    for p in Usd.PrimRange(stage.GetPrimAtPath(cell)):
        if not p.IsA(UsdGeom.Mesh):
            continue
        path = str(p.GetPath())
        if "/brk_" not in path:
            continue
        where = ("loose" if path in loose else
                 "static" if path in static else "unlisted")
        core = None
        for s in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(p)):
            if s.GetPrim().GetName() == "core":
                core = s.GetPrim()
        pname, pkind = _mat_kind(p)
        if where == "static":
            mb = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
            if mb and mb.GetPrim().IsValid():
                mats_seen.add(mb.GetPrim().GetPath().pathString)
        cname, ckind = (_mat_kind(core) if core is not None else ("-", "-"))
        # DOES IT HAVE UVs AT ALL? A raw `fracture._write_mesh` fragment has
        # none, so a UV-mapped atlas bound to it samples one texel; the
        # fire_dtc3 fix is exactly "the parent's UVs, carried".
        uv, box = "NO-UV", None
        for q in UsdGeom.PrimvarsAPI(p).GetPrimvars():
            if q.GetPrimvarName().startswith(("st", "uv", "UV")):
                v = q.Get()
                uv = "st[%d]" % (len(v) if v is not None else 0)
                if v:
                    us = [float(a[0]) for a in v]
                    vs = [float(a[1]) for a in v]
                    box = (min(us), max(us), min(vs), max(vs))
                break
        key = (where, pkind, ckind, uv.split("[")[0])
        if box is not None:
            b = uvbox.get(key)
            uvbox[key] = box if b is None else (
                min(b[0], box[0]), max(b[1], box[1]),
                min(b[2], box[2]), max(b[3], box[3]))
        rows[key] += 1
        ex.setdefault(key, (path, pname, cname))
    print("[tear_probe] C. FRAGMENT MATERIAL CENSUS")
    print("    {0:<9} {1:<16} {2:<12} {3:<6} {4:>6}   example  [prim-mat | "
          "core-mat]".format("where", "prim", "core", "uv", "n"))
    for key in sorted(rows, key=lambda k: (-rows[k], k)):
        path, pname, cname = ex[key]
        b = uvbox.get(key)
        print("    {0:<9} {1:<16} {2:<12} {3:<6} {4:>6}   {5}  [{6} | {7}]{8}"
              .format(key[0], key[1], key[2], key[3], rows[key],
                      path.rsplit("/", 2)[-1], pname, cname,
                      "" if b is None else
                      "  uv u[%.2f,%.2f] v[%.2f,%.2f]" % b))
    # DOES THE MATERIAL A FRAGMENT BINDS STILL RESOLVE A MAP? On the kit path
    # the piece's own material lives INSIDE the module `_break_split`
    # deactivates, so `fire_collapse.rehome_material` copies it out through an
    # internal reference first. If activation composed through that arc the
    # copy would be pruned and this would print "(none)" — it is the one claim
    # in that function's docstring a probe can actually check.
    print("[tear_probe] materials the STATIC fragments bind:")
    for mpath in sorted(mats_seen):
        nm, df = _diffuse(st_stage, mpath)
        print("    %-52s %-26s %s" % (mpath[-52:], nm, df))
    good = sum(n for k, n in rows.items()
               if k[0] == "static"
               and k[1] in ("facade", "facade_tone", "parent_uv"))
    tri = sum(n for k, n in rows.items()
              if k[0] == "static" and k[1] == "clad_triplanar")
    charred = sum(n for k, n in rows.items() if k[0] == "static" and k[1] == "char")
    tot = sum(n for k, n in rows.items() if k[0] == "static")
    print("[tear_probe] STATIC fragments: {0}/{1} carry the wall's own tone; "
          "{2} carry the clad TRIPLANAR of a unique atlas (a smear); {3} are "
          "charred whole".format(good, tot, tri, charred))


def main():
    spec = sys.argv[1] if len(sys.argv) > 1 else "gac:SM_Building_02:F5c"
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 193
    bits = spec.split(":")
    kind = bits[0]
    name = bits[1] if len(bits) > 1 else "SM_Building_02"
    level = bits[2] if len(bits) > 2 else "F5c"
    sides = _sides(bits[3]) if len(bits) > 3 else None
    frag_rx = re.compile(sys.argv[3]) if len(sys.argv) > 3 else None
    fracture.ensure_deps(verbose=False)
    fracture.ensure_vtk(verbose=False)
    t0 = time.time()
    print("=" * 78)
    print("[tear_probe] {0} seed={1}".format(spec, seed))
    try:
        if kind == "kit":
            stage, cell, ctx = run_kit(name, level, sides, seed)
        else:
            qname = name if kind == "gac" else "{0}:{1}".format(kind, name)
            stage, cell, ctx = run_sliced(qname, level, sides, seed)
    except Exception:
        traceback.print_exc()
        return 1
    for n in ctx["notes"]:
        print("   note:", n[:240])
    print("[tear_probe] {0:.0f}s".format(time.time() - t0))
    plan = ctx.get("partial_collapse")
    if not plan:
        print("[tear_probe] no partial-collapse plan on the ctx")
        return 1
    m = ctx["info"]["masses"].get(plan["mass"]) or ctx["info"]["masses"]["main"]
    _span_table(ctx, plan, m)
    _boundary_census(ctx, plan, m)
    _frag_census(stage, cell, ctx)
    if frag_rx is not None:
        _frag_dump(stage, cell, frag_rx)
    return 0


if __name__ == "__main__":
    sys.exit(main())
