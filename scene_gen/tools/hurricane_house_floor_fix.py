#!/usr/bin/env python3
"""hurricane_house_floor_fix.py — the REAL blue-floor fix, offline, bare pxr.

THE BUG H'S AUDIT MISSED. `build-hurricane-scenes/SKILL.md`'s bug catalogue
already records one floor-blue fix: the kit never bound its own floors at
MESH level, so RTX fell back to its default (blue) material, and
`bake_hurricane_archetypes_launch_script._bind_floors` /
`hurricane_house_pose_bake._bind_all_floors` fixed that by binding every
`house_floor*` MESH to a hardened `planks.wood_material` ("subfloor"). A
fresh audit re-confirms that mesh-level fix is clean (0 of 683 floor meshes
unbound at the time it was checked) — but MESH-level was never the whole
story.

Every `house_floor*` mesh also carries GeomSubsets named `Section0` /
`Section1`, and BOTH have their OWN direct material bindings, authored by
the original kit export and never cleared:

    Section0 -> /Baked/Looks/UnrealMaterial
    Section1 -> /Baked/Looks/Wood_01_White

A GeomSubset's own direct binding OVERRIDES the parent mesh's binding for
the faces in that subset (`UsdShade.MaterialBindingAPI`'s documented
resolution order) — so no matter how correctly `subfloor` is bound at mesh
level, `Section0`'s own faces still resolve to `UnrealMaterial` instead, and
`invisible while the roof is on top of it` is exactly why the original
mesh-level fix's own validation (`bake.validate`'s `bound_missing: 0` and
this file's own pre-existing bare-pxr audits) never caught it: those checks
ask "does the MESH have a binding," and it always did.

`UnrealMaterial` is a `UsdPreviewSurface` network whose 5 texture inputs
(diffuse/roughness/normal/occlusion) are all `omniverse://` paths under
`.../Muyang/ModularNeighborhood/Assets/Textures/Carpet_01_*.png`. This host
has no Omniverse Nucleus resolver at all (no `omni.client`, no network
egress — verified) so this tool CANNOT itself prove those specific files
are absent on the live Nucleus server; that measurement is the lead's own,
made on a Nucleus-connected host. What this tool DOES independently verify
by walking the whole library with bare pxr (see `census()`):

  * `UnrealMaterial` (this exact, bare name — NOT the numbered siblings
    `UnrealMaterial_1..11` the same files carry for windows/glass/doors/
    handles) is bound EXCLUSIVELY to `house_floor` GeomSubsets, 1,763 times
    across all 168 `house_*.usd` files in this library (canonical + the
    `_tornado` backups + the leftover `_h` build products + `swept`) — 0
    times on any other mesh category, in any file.
  * `Wood_01_White` — the OTHER subset override this same audit turned up,
    on `Section1` — is a DIFFERENT case: it is also bound to 177
    `house_wall` and 40 `house_roof`/`house_bay_roof` subsets across this
    library, none of which render blue or are reported broken anywhere in
    this session's work. There is no positive evidence this one is bad, so
    THIS TOOL DELIBERATELY LEAVES IT ALONE — clearing it would be a fix in
    search of a bug, not a fix for one. (Its validity, and every other
    material's, is still reported by `census()`'s "material validity"
    table, honestly labelled "nucleus_unverifiable" where that is the true
    epistemic state, rather than asserting it either way.)

THE FIX: for every `house_floor*` GeomSubset whose OWN direct binding names
`UnrealMaterial`, delete that binding (`MaterialBindingAPI.UnbindAllBindings`
on the SUBSET, not the mesh) rather than rebind it to a fresh copy of
`subfloor`. Deletion, not rebinding, because:

  1. It is the textbook USD idiom for "this subset does not override its
     parent" — an unauthored subset binding is DEFINED to fall through to
     the ancestor's resolved material, which is `subfloor` here by
     construction (verified: `ComputeBoundMaterial()` on the subset returns
     `/Baked/Looks/subfloor` immediately after `UnbindAllBindings()`, no
     second write needed).
  2. Rebinding would mean either duplicating `subfloor`'s prim into a new
     path per subset (extra prims, no benefit — the SAME material already
     covers the whole mesh once the override is gone) or pointing the
     subset at the mesh's OWN `subfloor` prim directly, which is exactly
     what deletion achieves for free via inheritance, with less code to get
     wrong.
  3. It also means fixing this bug can never conflict with a re-tune of
     `subfloor` itself (`harden_wood_material`, tile/tint/roughness) — there
     is only ever one authored copy per file, at mesh level, same as today.

After every subset in a file is cleared, the file's own `Looks/UnrealMaterial`
prim is deleted too if nothing in that file still targets it (checked by
`_material_ref_count`, both direct and collection-based bindings, any
purpose) — `UnrealMaterial` never appears at mesh level anywhere in this
library (measured), so in practice this always fires once a file has any
offending subset at all.

Usage:
    python3 scene_gen/tools/hurricane_house_floor_fix.py --census
    python3 scene_gen/tools/hurricane_house_floor_fix.py --fix
    python3 scene_gen/tools/hurricane_house_floor_fix.py --verify

Env:
    ARCH_DIR   archetype library (default scene_gen/assets/archetypes_hurricane)

Idempotent by construction: a second `--fix` run finds `subsets_cleared == 0`
for every file (nothing left bound to `UnrealMaterial`) and re-saves nothing
— see `test_hurricane_house_floor_materials.py`'s byte-reproducibility check.
"""
import argparse
import collections
import os
import sys

os.environ.setdefault("PXR_USDC_EMIT_DEPRECATION_WARNINGS", "0")

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _SCENE_GEN)

from pxr import Sdf, Usd, UsdGeom, UsdShade  # noqa: E402

from disaster import hurricane_flow as hf  # noqa: E402

# The one material name this fix targets. See the module docstring for the
# full measurement backing this — bound EXCLUSIVELY to house_floor subsets,
# 0 times anywhere else, across all 168 files in the library.
_TARGET_MATERIAL_NAME = "UnrealMaterial"


def _env(name, default):
    v = os.environ.get(name)
    return default if v is None or not v.strip() else v.strip()


ARCH_DIR = _env("ARCH_DIR", os.path.join(_SCENE_GEN, "assets",
                                         "archetypes_hurricane"))


def _house_files(arch_dir):
    return sorted(f for f in os.listdir(arch_dir)
                  if f.startswith("house_") and f.endswith(".usd")) \
        if os.path.isdir(arch_dir) else []


def _material_ref_count(stage, mat_path):
    """How many Mesh/GeomSubset prims in *stage* still target *mat_path*, at
    any binding purpose, direct or collection-based. Used to decide whether
    a now-possibly-orphaned Material prim is safe to delete outright."""
    n = 0
    for prim in stage.Traverse():
        if not (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Subset)):
            continue
        api = UsdShade.MaterialBindingAPI(prim)
        for purpose in (UsdShade.Tokens.allPurpose, UsdShade.Tokens.preview,
                       UsdShade.Tokens.full):
            rel = api.GetDirectBindingRel(purpose)
            if rel and rel.IsValid() and any(
                    t == mat_path for t in rel.GetTargets()):
                n += 1
        for rel in api.GetCollectionBindingRels():
            if rel and rel.IsValid() and any(
                    str(t).startswith(str(mat_path)) for t in rel.GetTargets()):
                n += 1
    return n


def fix_file(path, dry_run=False):
    """Clear every `house_floor*` GeomSubset's direct binding to
    `_TARGET_MATERIAL_NAME` in *path*, then delete that Material prim if
    nothing references it any more. Saves in place (crate) iff anything
    changed and `dry_run` is False.

    Never raises on a file with no floor meshes at all (`house_*_swept.usd`
    — measured, 1 of 168) or no offending subsets (idempotent — a second
    run finds `subsets_cleared == 0` and does not touch the file).
    """
    stage = Usd.Stage.Open(path)
    if not stage:
        return {"path": path, "error": "could not open"}

    n_floor_meshes = 0
    n_subsets_seen = 0
    cleared = 0

    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if hf._category_of(prim) != "house_floor":
            continue
        n_floor_meshes += 1
        for sub in prim.GetChildren():
            if not sub.IsA(UsdGeom.Subset):
                continue
            rel = sub.GetRelationship("material:binding")
            if not (rel and rel.IsValid()):
                continue
            targets = rel.GetTargets()
            if not targets:
                continue
            n_subsets_seen += 1
            if targets[0].name != _TARGET_MATERIAL_NAME:
                continue
            if not dry_run:
                UsdShade.MaterialBindingAPI(sub).UnbindAllBindings()
            cleared += 1

    removed_material = False
    if cleared and not dry_run:
        looks = None
        for prim in stage.Traverse():
            if prim.GetName() == "Looks":
                looks = prim
                break
        if looks is not None:
            mat_prim = looks.GetChild(_TARGET_MATERIAL_NAME)
            if mat_prim and mat_prim.IsValid():
                if _material_ref_count(stage, mat_prim.GetPath()) == 0:
                    stage.RemovePrim(mat_prim.GetPath())
                    removed_material = True
        stage.GetRootLayer().Save()

    return dict(path=path, floor_meshes=n_floor_meshes,
               subsets_seen=n_subsets_seen, subsets_cleared=cleared,
               material_removed=removed_material)


# ---------------------------------------------------------------------------
# validation -- callable from a pytest test with no CLI involved
# ---------------------------------------------------------------------------
def validate_bindings(path):
    """Walk every Mesh AND every GeomSubset in *path*; report any that binds
    NO material at all (the ORIGINAL blue-floor bug — RTX's own fallback)
    or that resolves to `_TARGET_MATERIAL_NAME` (this file's own fix
    target). Returns a dict with `unbound` and `unreal_material` lists of
    prim paths — both must be empty for a clean file.

    Deliberately does NOT fail a file for binding one of the OTHER
    Nucleus-hosted kit materials (`Wood_01_White`, `Stucco_01_Inst`, the
    `clad_*`/`shingle_*`/`Brick_*` family, `UnrealMaterial_1..11`) — this
    host has no Omniverse resolver and no network egress (verified), so it
    cannot tell a resolvable Nucleus texture from a missing one, and every
    one of those names is also used somewhere that is NOT reported broken
    (see the module docstring). Asserting "invalid" for all of them would
    be a false-positive audit, not a stronger one.
    """
    stage = Usd.Stage.Open(path)
    if not stage:
        return {"path": path, "error": "could not open"}
    unbound, bad = [], []
    for prim in stage.Traverse():
        if not (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Subset)):
            continue
        mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[:2]
        if mat is None or not mat.GetPrim().IsValid():
            unbound.append(prim.GetPath().pathString)
        elif mat.GetPrim().GetName() == _TARGET_MATERIAL_NAME:
            bad.append(prim.GetPath().pathString)
    return dict(path=path, unbound=unbound, unreal_material=bad)


# ---------------------------------------------------------------------------
# material validity census -- the table the final report asks for
# ---------------------------------------------------------------------------
def classify_material(mat_prim, repo_root):
    """(status, sample_paths) for one Material prim's texture inputs.

    status in {"no_textures", "local_ok", "local_missing",
    "nucleus_unverifiable"}. A material with ANY confirmed-missing LOCAL
    texture is "local_missing" regardless of what else it references —
    that is unambiguous evidence of breakage this host CAN see.
    "nucleus_unverifiable" means every texture is an `omniverse://` path
    this host has no resolver for (see `validate_bindings`'s docstring for
    why that is reported honestly rather than guessed at).
    """
    saw_nucleus = saw_local_ok = saw_local_missing = False
    samples = []
    n_tex = 0
    for shd in Usd.PrimRange(mat_prim):
        if shd.GetTypeName() != "Shader":
            continue
        sh = UsdShade.Shader(shd)
        for inp in sh.GetInputs():
            v = inp.Get()
            if not isinstance(v, Sdf.AssetPath):
                continue
            n_tex += 1
            raw = v.path
            samples.append(raw)
            if raw.startswith("omniverse:"):
                saw_nucleus = True
                continue
            cand = None
            for prefix in ("/isaac-sim/AirStack/", "airstack://"):
                if raw.startswith(prefix):
                    cand = os.path.join(repo_root, raw[len(prefix):])
                    break
            if cand is None and os.path.isabs(raw):
                cand = raw
            if cand and os.path.isfile(cand):
                saw_local_ok = True
            else:
                saw_local_missing = True
    if n_tex == 0:
        return "no_textures", samples
    if saw_local_missing:
        return "local_missing", samples
    if saw_nucleus and not saw_local_ok:
        return "nucleus_unverifiable", samples
    return "local_ok", samples


def census(arch_dir=None, repo_root=None):
    """Full library census: the GeomSubset override count (per the module
    docstring's measurement) and the material-validity table."""
    arch_dir = arch_dir or ARCH_DIR
    repo_root = repo_root or os.path.dirname(os.path.dirname(_SCENE_GEN))
    files = _house_files(arch_dir)
    n_files_bad = 0
    total_bad_subsets = 0
    total_subsets_any = 0
    non_floor_hits = collections.Counter()
    mat_status = {}
    for fn in files:
        path = os.path.join(arch_dir, fn)
        stage = Usd.Stage.Open(path)
        if not stage:
            continue
        file_bad = False
        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh):
                continue
            cat = hf._category_of(prim)
            for sub in prim.GetChildren():
                if not sub.IsA(UsdGeom.Subset):
                    continue
                rel = sub.GetRelationship("material:binding")
                if not (rel and rel.IsValid()):
                    continue
                targets = rel.GetTargets()
                if not targets:
                    continue
                total_subsets_any += 1
                if targets[0].name == _TARGET_MATERIAL_NAME:
                    if cat == "house_floor":
                        total_bad_subsets += 1
                        file_bad = True
                    else:
                        non_floor_hits[cat] += 1
            if prim.GetTypeName() == "":
                pass
        if file_bad:
            n_files_bad += 1
        for prim in stage.Traverse():
            if prim.GetTypeName() != "Material":
                continue
            name = prim.GetName()
            if name in mat_status:
                continue
            mat_status[name], _ = classify_material(prim, repo_root)
    return dict(n_files=len(files), n_files_bad=n_files_bad,
               total_bad_subsets=total_bad_subsets,
               total_subsets_any=total_subsets_any,
               non_floor_hits=dict(non_floor_hits),
               material_status=mat_status)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def cmd_census():
    rep = census()
    print("[hfloorfix] {0} house_*.usd files scanned".format(rep["n_files"]))
    print("[hfloorfix] {0} file(s) with >=1 house_floor subset bound to "
          "{1!r}".format(rep["n_files_bad"], _TARGET_MATERIAL_NAME))
    print("[hfloorfix] {0} total offending floor subset bindings ({1} "
          "subset bindings of any kind, any category, in the whole "
          "library)".format(rep["total_bad_subsets"], rep["total_subsets_any"]))
    print("[hfloorfix] {0!r} on NON-floor categories: {1}".format(
        _TARGET_MATERIAL_NAME, rep["non_floor_hits"] or "none (confirms "
        "floor-only)"))
    print("[hfloorfix] material validity (by distinct name across library):")
    by_status = collections.defaultdict(list)
    for name, status in rep["material_status"].items():
        by_status[status].append(name)
    for status in ("local_ok", "no_textures", "local_missing",
                  "nucleus_unverifiable"):
        names = sorted(by_status.get(status, []))
        if names:
            print("  {0:22s} {1:3d}  {2}".format(status, len(names), names))
    return 0


def cmd_fix(dry_run=False):
    files = _house_files(ARCH_DIR)
    if not files:
        print("[hfloorfix] no house_*.usd found under {0}".format(ARCH_DIR))
        return 1
    total_cleared = 0
    total_removed = 0
    per_class = collections.Counter()
    for fn in files:
        path = os.path.join(ARCH_DIR, fn)
        stats = fix_file(path, dry_run=dry_run)
        if "error" in stats:
            print("[hfloorfix] {0}: {1}".format(fn, stats["error"]))
            continue
        if stats["subsets_cleared"]:
            cls = ("_tornado" if fn.endswith("_tornado.usd") else
                   "_h" if fn.endswith("_h.usd") else
                   "swept" if "_swept" in fn else "canonical")
            per_class[cls] += stats["subsets_cleared"]
            total_cleared += stats["subsets_cleared"]
            if stats["material_removed"]:
                total_removed += 1
            print("[hfloorfix] {0:42s} cleared={1:3d} material_removed={2}"
                  .format(fn, stats["subsets_cleared"], stats["material_removed"]))
    print("[hfloorfix] {0} subset binding(s) cleared across {1} file(s) "
          "(by class: {2}); {3} orphaned Material prim(s) removed{4}"
          .format(total_cleared, len(files), dict(per_class), total_removed,
                  " [DRY RUN, nothing saved]" if dry_run else ""))
    return 0


def cmd_verify():
    files = _house_files(ARCH_DIR)
    bad_files = 0
    total_unbound = total_unreal = 0
    for fn in files:
        rep = validate_bindings(os.path.join(ARCH_DIR, fn))
        if "error" in rep:
            print("[hfloorfix][verify] {0}: {1}".format(fn, rep["error"]))
            continue
        if rep["unbound"] or rep["unreal_material"]:
            bad_files += 1
            total_unbound += len(rep["unbound"])
            total_unreal += len(rep["unreal_material"])
            print("[hfloorfix][verify] {0}: unbound={1} unreal_material={2}"
                  .format(fn, rep["unbound"], rep["unreal_material"]))
    print("[hfloorfix][verify] {0} file(s), {1} bad: {2} unbound prim(s), "
          "{3} UnrealMaterial binding(s)".format(
              len(files), bad_files, total_unbound, total_unreal))
    return 0 if (total_unbound == 0 and total_unreal == 0) else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--census", action="store_true")
    ap.add_argument("--fix", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--verify", action="store_true")
    args = ap.parse_args()
    rc = 0
    if args.census:
        rc = cmd_census() or rc
    if args.fix:
        rc = cmd_fix(dry_run=args.dry_run) or rc
    if args.verify:
        rc = cmd_verify() or rc
    if not (args.census or args.fix or args.verify):
        ap.print_help()
    return rc


if __name__ == "__main__":
    sys.exit(main())
