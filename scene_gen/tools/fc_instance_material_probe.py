#!/usr/bin/env python
"""fc_instance_material_probe — WHICH ASSETS LOSE THEIR MATERIALS (or their
geometry) WHEN `apply_placements` MARKS THEM INSTANCEABLE.

    bash scene_gen/tools/usd_python.sh scene_gen/tools/fc_instance_material_probe.py \
        --tsv scene_gen/_scene_assets.tsv
    bash scene_gen/tools/usd_python.sh scene_gen/tools/fc_instance_material_probe.py \
        --usd omniverse://.../props_light_post/SM_lightpost_light_post_b.usd

THE TWO FAILURES THIS SEPARATES (user, 2026-08-31, on the live 500 m fire
city): "Some props like the street lights also look like they have no
texture. What happened to it? same with some benches." — and, earlier, "this
roof house is floating with no building near it".

`downtown_fire_500.yaml` sets `instance_placements: true` (the fix for the
composition OOM), so `scene_generator.apply_placements` calls
`SetInstanceable(True)` on every placement whose category no `prune_prims`
rule blocks. USD instancing moves a prim's DESCENDANTS into a shared
prototype and leaves the instance prim itself outside it. Two things break:

  1. GEOMETRY, when the referenced asset's ROOT PRIM IS ITSELF A GPRIM. The
     prototype then holds only the root's children (GeomSubsets, Looks) with
     no Mesh in it, and the asset renders NOTHING. -> `renders: GONE`.
  2. MATERIALS, when a mesh's binding RELATIONSHIP TARGETS A PRIM OUTSIDE
     THE PROTOTYPE. USD does not translate a relationship out of a prototype
     — the binding is simply dropped — and the mesh falls back to the
     renderer's default grey. -> `binds: LOST n/m`.

THE CONTROL IS THE POINT. Every asset is composed TWICE on the same scratch
stage — once plain, once `SetInstanceable(True)` — and the computed bound
material of every mesh is compared between the two. Nothing here is inferred
from the asset's shape; the difference is measured. An asset that comes back
`ok` is proof the mechanism is not what is wrong with it.

NO `SimulationApp`. Runs under `tools/usd_python.sh` (bare USD + the
Omniverse resolver), so it is safe beside a live Isaac session.
"""
import argparse
import collections
import os
import sys

from pxr import Usd, UsdGeom, UsdShade


def _meshes(stage, root):
    """(relative path -> prim) for every Mesh at or under `root`, walking
    into instance proxies so the instanced and plain arms are comparable."""
    out = {}
    base = str(root.GetPath())
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies(
            Usd.PrimIsActive & Usd.PrimIsDefined)):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        out[str(prim.GetPath())[len(base):] or "/"] = prim
    return out


def _bound(prim):
    try:
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    except Exception:
        return None
    if not mat or not mat.GetPrim().IsValid():
        return None
    return str(mat.GetPrim().GetPath())


def audit(usd):
    """One asset: `(verdict, detail)`."""
    src = Usd.Stage.Open(usd)
    if src is None:
        return "OPEN-FAILED", {}
    dp = src.GetDefaultPrim()
    root_type = str(dp.GetTypeName()) if dp else "(no default prim)"
    gprim_root = bool(dp and dp.IsA(UsdGeom.Gprim))

    st = Usd.Stage.CreateInMemory()
    a = st.DefinePrim("/W/plain")
    a.GetReferences().AddReference(usd)
    a.Load()
    b = st.DefinePrim("/W/inst")
    b.GetReferences().AddReference(usd)
    b.Load()
    b.SetInstanceable(True)

    ma, mb = _meshes(st, a), _meshes(st, b)
    lost_geom = sorted(set(ma) - set(mb))
    lost_bind, kept_bind, never = [], 0, 0
    for rel, prim in ma.items():
        want = _bound(prim)
        if want is None:
            never += 1
            continue
        other = mb.get(rel)
        got = _bound(other) if other is not None else None
        if got is None:
            lost_bind.append((rel, want))
        else:
            kept_bind += 1

    detail = {"root_type": root_type, "gprim_root": gprim_root,
              "n_mesh": len(ma), "n_mesh_inst": len(mb),
              "lost_geom": lost_geom, "lost_bind": lost_bind,
              "kept_bind": kept_bind, "unbound": never}
    if gprim_root or (ma and not mb):
        return "GEOMETRY-GONE", detail
    if lost_bind:
        return "MATERIALS-LOST", detail
    if lost_geom:
        return "GEOMETRY-PARTIAL", detail
    return "ok", detail


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--tsv", default="",
                    help="scene_gen/_scene_assets.tsv — audits every DISTINCT "
                         "usd in it, grouped by category")
    ap.add_argument("--usd", action="append", default=[])
    ap.add_argument("--only", default="",
                    help="comma list of categories to audit (tsv mode)")
    a = ap.parse_args(argv)

    jobs = [(None, u) for u in a.usd]
    if a.tsv:
        want = {c.strip() for c in a.only.split(",") if c.strip()}
        cats = collections.defaultdict(set)
        with open(a.tsv) as fh:
            next(fh, None)
            for line in fh:
                parts = line.rstrip("\n").split("\t")
                if len(parts) < 4:
                    continue
                _path, cat, _pack, usd = parts[0], parts[1], parts[2], parts[3]
                if want and cat not in want:
                    continue
                cats[cat].add(usd)
        seen = set()
        for cat in sorted(cats):
            for usd in sorted(cats[cat]):
                if usd in seen:
                    continue
                seen.add(usd)
                jobs.append((cat, usd))

    counts = collections.Counter()
    bad = []
    print("\n{0:<10} {1:<44} {2:<16} {3}".format(
        "verdict", "asset", "root", "detail"))
    print("-" * 118)
    for cat, usd in jobs:
        try:
            verdict, d = audit(usd)
        except Exception as exc:
            verdict, d = "ERROR", {"root_type": str(exc)[:60]}
        counts[verdict] += 1
        if verdict != "ok":
            bad.append((cat, usd, verdict, d))
        note = ""
        if d.get("lost_bind"):
            note = "binds LOST {0}/{1}".format(
                len(d["lost_bind"]), len(d["lost_bind"]) + d.get("kept_bind", 0))
        elif verdict == "GEOMETRY-GONE":
            note = "{0} mesh -> {1} in the prototype".format(
                d.get("n_mesh"), d.get("n_mesh_inst"))
        elif verdict == "ok":
            note = "{0} mesh, {1} bind(s) kept, {2} never bound".format(
                d.get("n_mesh"), d.get("kept_bind"), d.get("unbound"))
        print("{0:<10} {1:<44} {2:<16} {3}  [{4}]".format(
            verdict, usd.rsplit("/", 1)[-1][:44], d.get("root_type", "?"),
            note, cat or "-"))

    print("\n[probe] {0}".format(", ".join(
        "{0}={1}".format(k, v) for k, v in sorted(counts.items()))))
    if bad:
        print("[probe] AFFECTED CATEGORIES (what to un-instance):")
        by_cat = collections.defaultdict(list)
        for cat, usd, verdict, _d in bad:
            by_cat[cat].append((verdict, usd.rsplit("/", 1)[-1]))
        for cat in sorted(by_cat, key=str):
            print("  {0:<22} {1}".format(
                cat, ", ".join("{0} ({1})".format(u, v)
                               for v, u in sorted(by_cat[cat]))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
