#!/usr/bin/env python
"""gac_shell_bind_repair.py — rebind every unbound GAC quake shell mesh in an
ALREADY-BAKED per-building USD, on a COPY, without touching the original.

    python3 scene_gen/tools/gac_shell_bind_repair.py \
        --in-dir  .../pod_bakes_fresh/gac_quake \
        --manifest .../pod_bakes_fresh/gac_quake/gac_quake.json \
        --out-dir .../pod_bakes_fresh/gac_quake_repaired

Bare `usd-core` only — no VTK, no bpy, nothing that needs the sliced source.

THE BUG THIS PATCHES OVER
--------------------------
`detail/gac_storey_slice.py` (fixed the same day this tool was written; see
that file's `read_mesh`/`write_piece`) leaves every exterior-shell piece
(`wall_*`, `corner_*`, `core_x_*`, `pier_*`, `parapet_*`) with NO usable
material on 33/33 checked fresh bakes. Two things were actually true at once,
found by dumping raw `GeomSubset` bindings rather than trusting
`ComputeBoundMaterial()` alone:

  1. Some faces truly had `mats[mi] is None` at slice time (no material the
     old `read_mesh` could resolve at all, or nothing the old
     `ComputeBoundMaterial()` — allPurpose only — found when the source bound
     it under the `full` purpose instead, exactly the trap
     `tools/material_binding.py`'s docstring already documents for other
     packs). `write_piece` then SKIPPED the bind outright: no subset, no
     mesh-level default, nothing.

  2. Where a real material WAS harvested and a `GeomSubset` WAS created and
     bound, it *still* fails to resolve in the frozen per-building file. The
     relationship target is not dangling — the prim it points at
     (`/World/bake/Looks/M_Building_24_Metal_Inst_<hash>`, etc.) exists,
     `IsValid()` is True — but it is a bare, TYPELESS placeholder with no
     children and no shader network: a reference out to a per-material
     Nucleus asset (`GreatAmericanCity/.../Materials/M_Building_24_Metal_
     Inst.usd`) that composes down to nothing once the file is opened
     without that Nucleus mount. `ComputeBoundMaterial()` requires the
     resolved prim to actually BE a `UsdShadeMaterial`, so it reports
     invalid even though a naive `GetPrimAtPath(target).IsValid()` says the
     opposite. Measured on every one of the 33 manifest-live fresh bakes:
     `/World/bake/Looks` is 100% typeless stubs, 0 real materials, in every
     single file (see `looks_survey_all.txt` from this round's forensics).
     Meanwhile `/World/bake/QuakeLooks` (and the per-group
     `/World/bake/gN/QuakeLooks`) — materials `quake_sliced.py`'s OWN damage
     code authors directly with a real embedded `UsdPreviewSurface`/OmniPBR
     shader, never a reference — resolve fine (87/88, 107/108, 18/21 "real"
     children measured across the same files).

So "bind from that file's own `/World/bake/Looks`" (the naive reading of
"the source's real facade materials [are] unused" there) does NOT work: every
candidate in that scope is equally an unresolved stub, and rebinding a shell
to another one produces a relationship that still fails `ComputeBoundMaterial
()` and still renders as flat fallback. This tool instead redirects each
unbound shell to a REAL, already-embedded material FROM THE SAME FILE's own
`QuakeLooks` scope, chosen by a role heuristic built from the (broken)
target's own name where one is on record (a subset that WAS bound to
something GAC named `..._Glass_..._Inst_<hash>` gets the real `glass`
material, not the opaque facade one), falling back to a dominant-facade
default when no name survives to read.

WHAT THIS DOES NOT FIX
-----------------------
The root defect (harvesting + never-skipping) is fixed prospectively in
`detail/gac_storey_slice.py` for the NEXT bake. This tool only repairs bakes
that already exist. It also does not — cannot, offline — prove that the
underlying Nucleus reference is itself wrong or fixable; it only proves that
whatever it is, it does not survive a standalone per-building file, and
routes around that with material this repo already ships self-contained.
"""

import argparse
import json
import os
import shutil
import sys

sys.path.insert(0, os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..")))

from pxr import Sdf, Usd, UsdGeom, UsdShade  # noqa: E402

SHELL_NEEDLES = ("wall_", "corner_", "core_x", "pier_", "parapet_")

# role -> ordered candidate material names to look for in THIS file's own
# QuakeLooks scope, first one that resolves to a REAL (typed, non-empty)
# Material wins. "facade"/"core" are the two ROLE-FROM-PIECE-NAME defaults
# (§ `_piece_role`); "glass"/"metal"/"marble" are ROLE-FROM-BROKEN-NAME
# redirects (§ `_keyword_role`) used only when a subset's ORIGINAL (now
# unresolvable) target name says so.
ROLE_CANDIDATES = {
    "glass": ["glass"],
    "metal": ["plant_metal", "rebar"],
    "marble": ["plaster", "mortar"],
    "facade": ["dark_concrete", "plaster_dusty", "mortar", "plaster"],
    "core": ["dark_concrete", "mortar"],
}

# substrings looked for (case-insensitive) in a broken subset's own,
# unresolvable relationship-target path — e.g.
# ".../Looks/M_Building_24_Glass_Green_Inst_7101c938" — to redirect it to a
# more specific real role than the piece's own default.
KEYWORD_ROLE = (("glass", "glass"), ("metal", "metal"), ("marble", "marble"))


def _piece_role(mesh_name):
    """wall_/corner_/pier_/parapet_* -> "facade"; core_x_* -> "core"."""
    if mesh_name.startswith("core_x"):
        return "core"
    return "facade"


def _keyword_role(target_paths, default_role):
    for t in target_paths:
        low = str(t).lower()
        for needle, role in KEYWORD_ROLE:
            if needle in low:
                return role
    return default_role


def _is_real_material(prim):
    return bool(prim and prim.IsValid() and prim.IsA(UsdShade.Material)
                and len(prim.GetChildren()) > 0)


def _find_quake_looks(stage):
    """`/World/bake/QuakeLooks` if it exists; else the first prim anywhere
    in the stage literally named "QuakeLooks" (per-group scopes use the
    same name one level down, e.g. `/World/bake/g0/QuakeLooks`)."""
    p = stage.GetPrimAtPath("/World/bake/QuakeLooks")
    if p and p.IsValid():
        return p
    for pr in stage.Traverse():
        if pr.GetName() == "QuakeLooks":
            return pr
    return None


def _build_role_table(stage, verbose=False):
    """role -> resolved Sdf.Path of a REAL material in this file, or None."""
    scope = _find_quake_looks(stage)
    table = {}
    any_real = None
    if scope is not None:
        children = {c.GetName(): c for c in scope.GetChildren()}
        for role, candidates in ROLE_CANDIDATES.items():
            hit = None
            for name in candidates:
                c = children.get(name)
                if _is_real_material(c):
                    hit = c.GetPath()
                    break
            table[role] = hit
        for c in scope.GetChildren():
            if _is_real_material(c):
                any_real = c.GetPath()
                break
    else:
        table = {r: None for r in ROLE_CANDIDATES}
    # deterministic dominant-facade fallback: any role this file's own
    # QuakeLooks didn't have a dedicated candidate for still gets SOMETHING
    # real, rather than being left unresolved.
    for role in table:
        if table[role] is None and any_real is not None:
            if verbose:
                print("  [role-table] no dedicated candidate for {0!r}, "
                      "falling back to {1}".format(role, any_real))
            table[role] = any_real
    return table


def _bound_ok(prim):
    mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    return bool(mat and mat.GetPrim().IsValid())


def repair_stage(stage, role_table, verbose=False):
    """Mutates `stage` in place. Returns a stats dict."""
    stats = {"shell_meshes": 0, "already_bound": 0, "mesh_defaults_added": 0,
             "subsets_rebound": 0, "subsets_left": 0, "roles_used": {}}
    if not any(role_table.values()):
        stats["error"] = "no real material found anywhere in QuakeLooks"
        return stats

    for prim in list(stage.Traverse()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        name = prim.GetName()
        if not any(k in name for k in SHELL_NEEDLES):
            continue
        stats["shell_meshes"] += 1

        subsets = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
        mesh_ok = _bound_ok(prim)
        subsets_ok = subsets and all(_bound_ok(s.GetPrim()) for s in subsets)
        if mesh_ok and (not subsets or subsets_ok):
            stats["already_bound"] += 1
            continue

        default_role = _piece_role(name)
        default_path = role_table.get(default_role) or next(
            (p for p in role_table.values() if p), None)

        if not mesh_ok:
            target = stage.GetPrimAtPath(default_path)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(target))
            stats["mesh_defaults_added"] += 1
            stats["roles_used"][default_role] = stats["roles_used"].get(default_role, 0) + 1

        for sub in subsets:
            sp = sub.GetPrim()
            if _bound_ok(sp):
                stats["subsets_left"] += 1
                continue
            api = UsdShade.MaterialBindingAPI(sp)
            rel = api.GetDirectBindingRel()
            targets = [str(t) for t in (rel.GetTargets() if rel else [])]
            role = _keyword_role(targets, default_role)
            path = role_table.get(role) or default_path
            if not path:
                continue
            UsdShade.MaterialBindingAPI.Apply(sp).Bind(
                UsdShade.Material(stage.GetPrimAtPath(path)))
            stats["subsets_rebound"] += 1
            stats["roles_used"][role] = stats["roles_used"].get(role, 0) + 1
            if verbose:
                print("  [subset] {0} : {1} -> {2}".format(sp.GetPath(), role, path))
    return stats


def _load_manifest(manifest_path):
    with open(manifest_path) as fh:
        data = json.load(fh)
    return [e["usd"].rsplit("/", 1)[-1] for e in data]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in-dir", required=True, help="directory of fresh bakes")
    ap.add_argument("--manifest", default=None,
                    help="manifest JSON listing live stems (default: "
                         "<in-dir>/<in-dir-basename>.json)")
    ap.add_argument("--out-dir", required=True,
                    help="NEW directory for repaired copies -- must not be "
                         "the same as --in-dir")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)

    in_dir = os.path.abspath(args.in_dir)
    out_dir = os.path.abspath(args.out_dir)
    if in_dir == out_dir:
        print("--out-dir must differ from --in-dir (originals are never touched)",
              file=sys.stderr)
        return 2
    manifest = args.manifest or os.path.join(in_dir, os.path.basename(in_dir) + ".json")
    if not os.path.isfile(manifest):
        # fall back to whatever single *.json sits next to the manifest-style
        # naming used by this round's bakes (gac_quake.json)
        cands = [f for f in os.listdir(in_dir) if f.endswith(".json")
                 and not f.startswith("gac_SM_")]
        if len(cands) == 1:
            manifest = os.path.join(in_dir, cands[0])
        else:
            print("could not find a manifest JSON in {0} (looked for {1})".format(
                in_dir, manifest), file=sys.stderr)
            return 2

    stems = _load_manifest(manifest)
    os.makedirs(out_dir, exist_ok=True)

    totals = {"files": 0, "shell_meshes": 0, "already_bound": 0,
              "mesh_defaults_added": 0, "subsets_rebound": 0, "errors": []}
    print("{0:<38} {1:>6} {2:>7} {3:>8} {4:>9}".format(
        "file", "shell", "ok", "mesh+", "subset+"))
    print("-" * 76)
    for stem in stems:
        src = os.path.join(in_dir, stem)
        if not os.path.isfile(src):
            totals["errors"].append("{0}: missing from --in-dir".format(stem))
            continue
        dst = os.path.join(out_dir, stem)
        shutil.copyfile(src, dst)
        # copy a matching sidecar .json (per-piece manifest), if present --
        # untouched, informational only.
        json_side = src[:-4] + ".json" if src.endswith(".usd") else None
        if json_side and os.path.isfile(json_side):
            shutil.copyfile(json_side, dst[:-4] + ".json")

        stage = Usd.Stage.Open(dst)
        if stage is None:
            totals["errors"].append("{0}: failed to open".format(stem))
            continue
        role_table = _build_role_table(stage, verbose=args.verbose)
        stats = repair_stage(stage, role_table, verbose=args.verbose)
        if "error" in stats:
            totals["errors"].append("{0}: {1}".format(stem, stats["error"]))
            print("{0:<38} {1:>6}  !! {2}".format(stem, stats["shell_meshes"], stats["error"]))
            continue
        stage.GetRootLayer().Save()

        totals["files"] += 1
        totals["shell_meshes"] += stats["shell_meshes"]
        totals["already_bound"] += stats["already_bound"]
        totals["mesh_defaults_added"] += stats["mesh_defaults_added"]
        totals["subsets_rebound"] += stats["subsets_rebound"]
        print("{0:<38} {1:>6} {2:>7} {3:>8} {4:>9}".format(
            stem, stats["shell_meshes"], stats["already_bound"],
            stats["mesh_defaults_added"], stats["subsets_rebound"]))

    print("-" * 76)
    print("TOTAL  files={files} shell_meshes={shell_meshes} "
          "already_bound={already_bound} mesh_defaults_added={mesh_defaults_added} "
          "subsets_rebound={subsets_rebound}".format(**totals))
    if totals["errors"]:
        print("\n{0} error(s):".format(len(totals["errors"])))
        for e in totals["errors"]:
            print("  " + e)
    print("\nwrote repaired copies to {0}".format(out_dir))
    return 1 if totals["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
