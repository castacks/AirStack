#!/usr/bin/env python
"""bake_gac_kits — slice a GreatAmericanCity building ONCE, offline, and save
the kit so nothing has to re-slice it again.

    ISAAC=/isaac-sim
    $ISAAC/AirStack/scene_gen/tools/usd_python.sh \\
        $ISAAC/AirStack/scene_gen/tools/bake_gac_kits.py --assets SM_Building_02
    $ISAAC/AirStack/scene_gen/tools/usd_python.sh \\
        $ISAAC/AirStack/scene_gen/tools/bake_gac_kits.py --assets SM_Building_02,SM_Building_09
    $ISAAC/AirStack/scene_gen/tools/usd_python.sh \\
        $ISAAC/AirStack/scene_gen/tools/bake_gac_kits.py --assets all

No `SimulationApp` — this is the bare-USD harness (`usd_python.sh`), the same
one `tools/piece_mat_probe.py` / `mat_carry_verify.py` / `ring_verify.py`
already run offline, safe beside a live Isaac Sim session because it never
touches Kit.

WHY THIS EXISTS
----------------
Every launch that puts a GAC building through the fire/quake ladder re-slices
it from the merged mesh: `gac_storey_slice.slice_to_kit` measures the window
grid, runs a VTK plane-clip stack per storey per ring cell, and writes a few
hundred to a couple thousand USD meshes. That is most of a bench's build time
(a 15-building bench: 31 minutes) and it is DETERMINISTIC for a given asset
plus slicer version — the same clip stack on the same mesh always produces
the same pieces. So: slice once, save the result, reference it at launch
(`detail/kit_bake.load_kit`) instead of re-cutting it.

THE MATERIAL TRAP — READ `gac_slice.rehome_materials` BEFORE CHANGING THIS
-----------------------------------------------------------------------------
`gac_storey_slice.write_piece` binds every subset to whatever material
`read_mesh` found bound on the SOURCE mesh — a prim living under the merged
building's own subtree. A live launcher gets away with this because
`slice_to_kit` only makes that subtree INVISIBLE, never detaches it, so the
binding keeps resolving all launch long. Export the sliced pieces to a
standalone file that way and every one of them references a material prim
that will not exist once this process exits — reopened cold, on a stage that
never touched the source, they render WHITE with their geometry and UVs
perfectly intact (this repo has already been burned exactly here once: "this
turned every sliced building white"). So, before the source is dropped, this
baker:

  1. slices onto an in-memory stage that still has the source referenced
     (`gac_storey_slice.slice_to_kit`, unmodified, called exactly the way a
     launcher calls it today);
  2. discovers every material the WRITTEN SUBSETS actually ended up bound to
     — by asking each subset, `UsdShade.MaterialBindingAPI.ComputeBoundMaterial`,
     not by reaching into slicer internals, so this needs nothing from
     `slice_to_kit` beyond what it already returns;
  3. REHOMES each one via `gac_slice.rehome_materials`, which either
     references the material's OWN Nucleus file directly (GAC's normal case —
     every material lives in its own `Materials/*.usd`, measured in
     `gac_slice._material_source`) or copies its composed spec when there is
     no such file;
  4. REBINDS every subset onto its rehomed material;
  5. removes the source subtree (`stage.RemovePrim`) — now safe, nothing
     left points into it;
  6. exports the ROOT LAYER ONLY (`stage.GetRootLayer().Export`, never
     `stage.Export()` / `stage.Flatten()`). The `freeze-disaster-dataset`
     skill has the cautionary tale: core USD's flatten cannot touch a kit
     mesh/subset/material carrying an `assetInfo` dict it cannot unpack, and
     GAC's own subtree is never even IN this stage's export — everything
     written here is either a freshly authored mesh (`write_piece`, no
     `assetInfo`) or a fresh reference to a material file, and root-layer
     export preserves exactly those without composing (or copying) a single
     spec out of the poisoned source. Nothing here needed Kit's exporter.

The output is intentionally still Nucleus-dependent for its materials (a
reference, not an embedded copy) — that is the SAME contract every other
GAC-derived asset in this pipeline already has (AEC brownstones, the objaverse
props), not a new one.

OUTPUT
------
`scene_gen/assets/kits/<asset>.usd` — one file per building; `*.usd` is
repo-gitignored (`scene_gen/**/*.usd`), same as every other bake in this
tree, so this cannot repeat the "cleaned out big files" history.
`scene_gen/assets/kits/kits.json` — the manifest `detail/kit_bake.load_kit`
reads, IS tracked (see that module's docstring for the shape and the
staleness fingerprint).
"""

import argparse
import os
import sys
import time

_SG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, _SG)

from detail import gac_slice as gsl                        # noqa: E402
from detail import gac_storey_slice as gss                 # noqa: E402
from detail import kit_bake as kb                           # noqa: E402

NUC = "omniverse://airlab-nucleus.andrew.cmu.edu:443/"
GAC_DIR = NUC + "Projects/SEI-COA/GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/"
# ALL 31 GAC BUILDINGS — the same enumeration `gac_kit_catalogue_launch_script`
# uses (`_GAC_ALL`, not re-derived here to avoid importing an Isaac-Sim-only
# launch script from a bare-USD tool): `SM_Building_<01..31>`, with `_06`
# alone carrying the `_Small` suffix.
_GAC_ALL = ["SM_Building_{0:02d}".format(k) for k in range(1, 32)]
_GAC_ALL[5] = "SM_Building_06_Small"
# GAC's own cm -> m convention — the same 0.01 every other caller in this repo
# hardcodes (`place_asset`, `place_source`, every `tools/*_verify.py` probe).
SCALE = 0.01


def _place_source(stage, holder, usd, scale=SCALE):
    """Reference `usd` under `holder/asset`, scaled — the same shape every
    other slicer probe in this repo sets up (see `tools/ring_verify.py` etc).
    Returns the referenced prim's path, or None if nothing actually composed
    (a broken Nucleus path still lets `AddReference` succeed, so the real
    test is whether anything has a world bound, not the return of that call).
    """
    from pxr import Sdf, Usd, UsdGeom

    UsdGeom.Xform.Define(stage, Sdf.Path(holder))
    kid = stage.DefinePrim(Sdf.Path(holder + "/asset"))
    kid.GetReferences().AddReference(usd)
    stage.Load(Sdf.Path(holder))
    if abs(scale - 1.0) > 1e-9:
        UsdGeom.Xformable(kid).AddScaleOp().Set((scale, scale, scale))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = cache.ComputeWorldBound(stage.GetPrimAtPath(holder)).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    return holder + "/asset"


def _rehome_piece_materials(stage, pieces_scope, dst_looks, verbose=True):
    """Rebind every GeomSubset under `pieces_scope` off the SOURCE's material
    prims and onto fresh ones under `dst_looks`. See the module docstring —
    this is step 2/3/4 of the material trap fix.

    Discovers what to rehome by asking the SUBSETS what they are bound to
    right now (`ComputeBoundMaterial`), rather than threading a materials
    list out of `slice_to_kit` — that keeps this baker decoupled from the
    slicer's internals, which matters while the other half of this change is
    still editing them.

    Returns `(new_mats, n_bound, n_subsets)`.
    """
    from pxr import Usd, UsdGeom, UsdShade

    root = stage.GetPrimAtPath(pieces_scope)
    mat_prims, targets = {}, []
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Subset):
            continue
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            continue
        key = str(mat.GetPrim().GetPath())
        mat_prims.setdefault(key, mat)
        targets.append((prim, key))
    new_mats = gsl.rehome_materials(stage, mat_prims, dst_looks, verbose=verbose)
    n_bound = 0
    for prim, key in targets:
        nm = new_mats.get(key)
        if nm is None:
            continue
        UsdShade.MaterialBindingAPI(prim).Bind(nm)
        n_bound += 1
    if verbose:
        print("[bake_gac_kits]   rehomed {0} material(s), rebound {1}/{2} "
              "subset(s)".format(len(new_mats), n_bound, len(targets)))
    return new_mats, n_bound, len(targets)


def bake_one(name, out_dir, usd=None, verbose=True):
    """Slice `name` once, rehome its materials, export it, and record it in
    `kits.json`. Returns the manifest record, or None if nothing composed or
    the slice was empty.
    """
    from pxr import Sdf, Usd, UsdGeom

    usd = usd or (GAC_DIR + name + ".usd")
    t0 = time.time()
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    kit = UsdGeom.Xform.Define(stage, Sdf.Path("/Kit"))
    stage.SetDefaultPrim(kit.GetPrim())

    src = _place_source(stage, "/Src", usd)
    if not src:
        print("[bake_gac_kits] {0}: nothing composed from {1}".format(name, usd))
        return None

    t_slice0 = time.time()
    # EXACTLY AS A LAUNCHER CALLS IT TODAY (`fire_pack_rows_launch_script.py`,
    # `gac_kit_catalogue_launch_script.py`): unmodified signature, unmodified
    # defaults bar the shared `CUT_OFFSET`. `cell="/Kit"` makes `slice_to_kit`
    # write pieces to `/Kit/pieces` — a ROOT-LEVEL scope, which is what lets
    # `/Kit` be this file's `defaultPrim` and be referenced as a whole later.
    pls, g, measured = gss.slice_to_kit(stage, src, "/Kit", name,
                                        offset=kb.CUT_OFFSET, verbose=verbose)
    slice_s = time.time() - t_slice0
    if not pls:
        print("[bake_gac_kits] {0}: sliced to nothing".format(name))
        return None

    new_mats, n_bound, n_subsets = _rehome_piece_materials(
        stage, "/Kit/pieces", "/Kit/Looks", verbose=verbose)
    if n_bound < n_subsets:
        print("[bake_gac_kits] WARNING {0}: only {1}/{2} subset(s) rehomed — "
              "{3} piece(s) will render untextured once the source is "
              "removed below".format(name, n_bound, n_subsets,
                                      n_subsets - n_bound))

    # NOW SAFE: every subset that had a resolvable material points at
    # `/Kit/Looks/*` instead of into `/Src`.
    stage.RemovePrim(Sdf.Path("/Src"))

    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, name + ".usd")
    # ROOT LAYER ONLY — see the module docstring for why this is not
    # `stage.Export()` / `stage.Flatten()`.
    stage.GetRootLayer().Export(out_path)
    mb = round(os.path.getsize(out_path) / 1e6, 3)
    bake_s = time.time() - t0

    pieces = []
    for p in pls:
        d = {k: v for k, v in p.items() if k != "prim_path"}
        d["name"] = p["prim_path"].rsplit("/", 1)[-1]
        pieces.append(d)

    record = {
        "asset": name,
        "usd": os.path.abspath(out_path),
        "src_usd": usd,
        "fingerprint": kb.fingerprint(),
        "grid": g,                       # already carries "measured"
        "pieces": pieces,
        "n_pieces": len(pieces),
        "materials": len(new_mats),
        "mb": mb,
        "slice_s": round(slice_s, 2),
        "bake_s": round(bake_s, 2),
    }
    kb.merge_manifest([record])
    if verbose:
        print("[bake_gac_kits] {0}: {1} piece(s), {2} material(s), {3:.3f} MB, "
              "grid {4} ({5:.2f} m storeys x{6}) — sliced {7:.1f}s, "
              "baked {8:.1f}s -> {9}".format(
                  name, len(pieces), len(new_mats), mb,
                  "measured" if measured else "regular",
                  g.get("storey_h", 0.0), len(g.get("storeys") or []),
                  slice_s, bake_s, out_path))
    return record


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--assets", default="SM_Building_02",
                    help="comma list of GAC assets, or 'all' for all 31 "
                         "(default: SM_Building_02)")
    ap.add_argument("--out-dir", default=kb.KIT_DIR)
    a = ap.parse_args()
    names = (list(_GAC_ALL) if a.assets.strip().lower() == "all" else
             [v.strip() for v in a.assets.split(",") if v.strip()])

    from disaster import fracture
    fracture.ensure_vtk(verbose=True)

    t0 = time.time()
    ok, failed = [], []
    for nm in names:
        try:
            r = bake_one(nm, a.out_dir)
            (ok if r else failed).append(nm)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[bake_gac_kits] FAILED {0}: {1}".format(nm, exc))
            failed.append(nm)

    print("\n" + "=" * 72)
    print("GAC KIT BAKE   {0}/{1} asset(s) -> {2}".format(
        len(ok), len(names), a.out_dir))
    if failed:
        print("  failed/empty: {0}".format(", ".join(failed)))
    print("  {0:.0f}s total, manifest: {1}".format(
        time.time() - t0, kb.MANIFEST_PATH))
    print("=" * 72 + "\n")


if __name__ == "__main__":
    main()
