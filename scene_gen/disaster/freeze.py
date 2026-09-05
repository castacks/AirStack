"""Freeze an assembled scene to ONE self-contained USD plus a `Materials/` folder.

    from disaster import freeze
    info = freeze.export_scene(out_dir, "fire_suburban_lvl1_1")

WHAT "SELF-CONTAINED" HAS TO MEAN HERE. The dataset ships a scene to somebody
who has no Nucleus, no `airstack://` root and no archetype library — so every
mesh, every material and every texture has to travel inside the folder. Two
steps, and each one exists because the other cannot do its job:

  1. **FLATTEN, WITH KIT.** `omni.usd.get_context().export_as_stage_async`
     composes the whole stage into one layer. Core USD's `stage.Flatten()`
     cannot be used: every kit mesh, GeomSubset and Material carries an
     `assetInfo` dict whose value core USD cannot unpack
     (`Usd_CrateFile::_UnpackValue ... unsupported type enum value 0`), and
     reading, copying or clearing that field all raise. Kit's exporter handles
     it. This is the same reason `disaster.bake` flattens with Kit and slices
     with USD.

  2. **COLLECT, WITH KIT.** The flat layer still POINTS AT its textures and MDL
     modules. `UsdUtils.LocalizeAsset` (USD 24.05, present) would relocate the
     USD-level asset paths — but a texture named INSIDE an `.mdl` module
     (`diffuse_texture: texture_2d("./textures/...")`) is invisible to USD, and
     most of the look in this scene is MDL: the kit house materials, the AEC
     bark (`TreeBark_07.mdl`), `Grass_Cut.mdl`, the RetroNeighborhood set.
     `omni.kit.usd.collect` parses MDL and is the only thing here that does.

INSTANCING SURVIVES THE FLATTEN, AND THAT IS THE WHOLE REASON THIS IS CHEAP.
Measured before the exporter was written, because getting it wrong is the
difference between a 300 MB file and a 30 GB one: 40 instanceable references to
a 1.0 MB archetype flatten to **1.0 MB with one prototype retained** and all
5,120 meshes reachable through `Usd.TraverseInstanceProxies()`. A plat with
5,563 tree references therefore costs one copy per ARCHETYPE, not per tree.
`verify()` re-counts the prototypes for exactly this reason — a flatten that
silently expanded them is the one failure mode that looks like success.

THE FOLDER LAYOUT IS THE CALLER'S, NOT THE COLLECTOR'S. `flat_collection=True`
writes `materials/` and `textures/` beside the root USD; the dataset contract
asks for a single `Materials/`. Both are MOVED TOGETHER, so the relative links
from an `.mdl` to its own `../textures/...` are unchanged, and only the ROOT
layer's asset paths need the `Materials/` prefix.
"""

import os
import shutil
import time


#: Directories the collector may create beside the root USD. All of them are
#: moved into `Materials/` as a group, which is what keeps their relative
#: cross-links intact.
_COLLECT_DIRS = ("materials", "textures", "SubUSDs")

MATERIALS_DIR = "Materials"


# ---------------------------------------------------------------------------
# path rewriting (pure pxr, no Kit)
# ---------------------------------------------------------------------------

def _prefix_asset_paths(layer, prefix, roots=_COLLECT_DIRS):
    """Prefix every asset path in *layer* that points into one of *roots*.

    Walks SPECS rather than composed prims, so it also catches attributes
    inside prototypes — where, after a flatten, every instanced archetype's
    materials actually live.

    **It reads `default` and nothing else.** `assetInfo` is the field that
    cannot be touched on kit specs (see the module docstring); this never asks
    for it.
    """
    from pxr import Sdf

    n = 0
    pre = tuple(roots) + tuple("./" + r for r in roots)

    def _fix_one(v):
        s = v.path
        if not s:
            return None
        t = s[2:] if s.startswith("./") else s
        if not t.startswith(tuple(r + "/" for r in roots)):
            return None
        return Sdf.AssetPath(prefix + "/" + t)

    def visit(path):
        nonlocal n
        spec = layer.GetObjectAtPath(path)
        if not isinstance(spec, Sdf.AttributeSpec):
            return
        if not spec.HasDefaultValue():
            return
        v = spec.default
        if isinstance(v, Sdf.AssetPath):
            nv = _fix_one(v)
            if nv is not None:
                spec.default = nv
                n += 1
        elif isinstance(v, Sdf.AssetPathArray) or (
                isinstance(v, (list, tuple)) and v
                and isinstance(v[0], Sdf.AssetPath)):
            out = []
            hit = False
            for a in v:
                na = _fix_one(a)
                out.append(na if na is not None else a)
                hit = hit or na is not None
            if hit:
                spec.default = Sdf.AssetPathArray(out)
                n += 1

    layer.Traverse(Sdf.Path("/"), visit)
    return n


def _relocate(collect_dir, out_dir, out_usd, root_name):
    """Move the collector's dependency folders under `Materials/` and write the
    root layer to *out_usd* with its paths re-pointed. Returns bytes moved."""
    from pxr import Sdf

    mats = os.path.join(out_dir, MATERIALS_DIR)
    os.makedirs(mats, exist_ok=True)
    moved = 0
    for d in _COLLECT_DIRS:
        src = os.path.join(collect_dir, d)
        if not os.path.isdir(src):
            continue
        dst = os.path.join(mats, d)
        if os.path.exists(dst):
            shutil.rmtree(dst)
        shutil.move(src, dst)
        for base, _dirs, files in os.walk(dst):
            for f in files:
                moved += os.path.getsize(os.path.join(base, f))

    src_usd = os.path.join(collect_dir, root_name)
    layer = Sdf.Layer.FindOrOpen(src_usd)
    if layer is None:
        raise RuntimeError("collected root layer not readable: " + src_usd)
    n = _prefix_asset_paths(layer, MATERIALS_DIR)
    layer.Export(out_usd)
    print("[freeze] re-pointed {0} asset path(s) into {1}/".format(
        n, MATERIALS_DIR))
    return moved


# ---------------------------------------------------------------------------
# verification (pure pxr, no Kit)
# ---------------------------------------------------------------------------

def _classify_external(paths, here):
    """`stray` — every path in *paths* that is either a URL (portable, but
    not self-contained) or an absolute path outside *here* (the cell's own
    directory) — the same two-bucket split `verify()` has always drawn
    between `omniverse://...` (fine) and a build-machine absolute (fatal).
    Shared between the rich (`ComputeAllDependencies`) and fallback scans so
    the two report `external` on identical rules."""
    stray = []
    for a in paths:
        ap = str(a)
        if ap.startswith(("omniverse://", "http://", "https://")):
            stray.append(ap)
        elif os.path.isabs(ap) and not os.path.abspath(ap).startswith(here):
            stray.append(ap)
    return stray


def _fallback_asset_scan(stage, here):
    """A NEVER-RAISING substitute for `UsdUtils.ComputeAllDependencies` on a
    kit-poisoned crate. 2026-09 finding: on a real cell that scan does not
    just fail cleanly, it raises PARTWAY THROUGH — and the caller used to
    leave `unresolved`/`deps`/`external` at their pre-try defaults (`[]`) on
    that exception, which reads exactly like "scanned everything, found
    nothing wrong" when what actually happened is "the scan died and never
    finished." A `deps_error` was recorded, but nothing stopped a consumer
    from reading the empty `unresolved` list as evidence. THIS is what
    "detection without enforcement" meant for the deps scan specifically:
    the file over-reported cleanliness in exactly the field a caller would
    check first.

    So the rich scan's failure must never be allowed to leave `unresolved`
    looking like a clean, completed scan — see `verify()`'s own handling,
    which now falls back to THIS function and labels the result
    `deps_scan_method="fallback_shader_attrs"` rather than `"full"`.

    Walks COMPOSED PRIMS (`Usd.Stage.Traverse()`) and reads ordinary
    `Usd.Attribute.Get()` values — the exact technique `build_local` already
    uses successfully on these cells (it never asks for the poisoned
    `assetInfo` field, so it never raises). For every distinct, non-empty,
    non-`/Game/` asset path found on any prim (not just `Shader` prims —
    the rich scan is not restricted to shaders either, and a texture
    parked on a non-Shader prim would otherwise be invisible to this
    fallback): an `omniverse://`/`http(s)://` URL is recorded but never
    network-checked (matching `build_local`'s own scope — reaching out to
    Nucleus/S3 during an offline `verify()` call would make it slow and
    flaky for no benefit this dataset needs); a local path is checked with
    a plain `os.path.exists`.

    KNOWN NARROWER THAN THE RICH SCAN, ON PURPOSE RATHER THAN BY ACCIDENT:
    this never walks references, payloads, sublayers or value clips, only
    ordinary attribute VALUES — so it can miss e.g. a broken reference
    target that carries no Asset-typed attribute of its own. Every caller
    of this function's result is told which scan actually ran
    (`deps_scan_method`) for exactly this reason.
    """
    from pxr import Sdf, Usd

    seen = set()
    unresolved = []
    # `Usd.TraverseInstanceProxies()` — see `verify`'s own `build_local`
    # computation for why a bare `stage.Traverse()` is not enough: an
    # instanced archetype's shaders live inside a PROTOTYPE, invisible to
    # a plain prim walk.
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        for attr in prim.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            val = attr.Get()
            path = getattr(val, "path", "") or ""
            if not path or path.startswith("/Game/") or path in seen:
                continue
            seen.add(path)
            if path.startswith(("omniverse://", "http://", "https://")):
                continue
            local = path if os.path.isabs(path) else os.path.normpath(
                os.path.join(here, path))
            if not os.path.exists(local):
                unresolved.append(path)
    return {"paths": seen, "unresolved": unresolved}


def verify(usd_path, expect_region=None, expect_self_contained=True):
    """Open the frozen scene COLD and check it stands on its own.

    The check that matters is not "did a file get written" but:

      * **nothing unresolved** — `UsdUtils.ComputeAllDependencies` reports every
        asset path that does not resolve, which is what a missing Nucleus or a
        missed MDL texture looks like;
      * **nothing outside the folder** — a dependency that resolved because
        THIS machine happens to have the archetype library is not
        self-contained, and it is the failure that only shows up on somebody
        else's disk;
      * **the prototypes survived** — a flatten that expanded instancing
        produces a correct scene at 50x the size, and reports success;
      * **the plate is the right size** — a bbox that disagrees with the build
        means geometry was dropped.
    """
    from pxr import Sdf, Usd, UsdGeom, UsdLux, UsdShade, UsdUtils
    from . import material_audit

    out = {"path": usd_path,
           "size_mb": round(os.path.getsize(usd_path) / 1e6, 1)}
    st = Usd.Stage.Open(usd_path)
    out["materials"] = material_audit.audit(st)
    out["prototypes"] = len(st.GetPrototypes())
    out["prims"] = sum(1 for _ in Usd.PrimRange.Stage(
        st, Usd.TraverseInstanceProxies()))
    out["meshes"] = sum(1 for p in Usd.PrimRange.Stage(
        st, Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Mesh))

    # `ComputeAllDependencies` is the richest check but it is NOT reliable on
    # a kit-written cell: it walks every field, hits the kit assets' poisoned
    # `assetInfo` and raises `_UnpackValue: unsupported type enum value 0`
    # (measured on EVERY shipped cell, from both Isaac 5.1's pxr and a clean
    # usd-core 26.08 — this is not an occasional flake, it is the expected
    # outcome on every real frozen cell today). It must therefore never be
    # able to take the portability verdict down with it — that verdict is
    # the whole point of this function.
    #
    # 2026-09 fix: a scan that raises PARTWAY THROUGH used to leave
    # `unresolved`/`deps`/`external` at their pre-try defaults (`[]`/`None`),
    # which is indistinguishable from "scanned everything, found nothing
    # wrong" — an empty `unresolved` list is now NEVER the result of a scan
    # that did not finish. `deps_scan_method` says which scan actually
    # produced the numbers below: `"full"` (ComputeAllDependencies
    # completed) or `"fallback_shader_attrs"` (it raised; `_fallback_
    # asset_scan` — Usd.Stage.Traverse over ordinary attribute values, the
    # same proven-safe technique `build_local` already uses — ran instead,
    # and is narrower: it does not see references/payloads/sublayers/value
    # clips, only asset-typed attribute values. Read `_fallback_asset_scan`'s
    # own docstring before trusting an empty `unresolved` from this path as
    # strongly as one from the full scan).
    here = os.path.dirname(os.path.abspath(usd_path))
    out["deps"] = None
    out["unresolved"] = None
    out["external"] = []
    out["deps_error"] = None
    out["deps_scan_method"] = None
    try:
        _l, assets, unresolved = UsdUtils.ComputeAllDependencies(
            Sdf.AssetPath(usd_path))
        out["deps"] = len(assets)
        out["unresolved"] = [str(u) for u in unresolved]
        out["external"] = _classify_external(assets, here)
        out["deps_scan_method"] = "full"
    except Exception as exc:                                # noqa: BLE001
        out["deps_error"] = str(exc).strip().splitlines()[:1]
        print("[freeze] *** ComputeAllDependencies FAILED ({0}) -- this is "
              "the kit assetInfo poison (freeze-portable-scenes skill), "
              "expected on every real cell. Falling back to a "
              "Stage.Traverse()-based asset-attribute scan (narrower: no "
              "references/payloads/sublayers) rather than trusting the dead "
              "scan's empty result.".format(out["deps_error"]))
        fb = _fallback_asset_scan(st, here)
        out["deps"] = len(fb["paths"])
        out["unresolved"] = list(fb["unresolved"])
        out["external"] = _classify_external(fb["paths"], here)
        out["deps_scan_method"] = "fallback_shader_attrs"
    # `unresolved` is now ALWAYS a real, completed list (possibly empty),
    # never `[]`-by-omission from a scan that never ran to completion.

    # BUILD-LOCAL PATHS, computed WITHOUT ComputeAllDependencies so the check
    # survives the failure above. Reading a shader attribute's `default` is the
    # one traversal that works on these cells — it never asks for `assetInfo`.
    #
    # `Usd.TraverseInstanceProxies()`, NOT a bare `st.Traverse()` — the same
    # predicate this function's own prim/mesh counts already use, for the
    # same reason: an instanced archetype's shaders live inside a
    # PROTOTYPE, which a plain prim walk never visits (2026-09-01 finding —
    # a bare walk here could report a cell clean while build-local paths
    # were sitting untouched inside a prototype the whole time).
    #
    # TWO COUNTS, on purpose (2026-09-01: a report was seen quoting 3693
    # against this function's own 40 and asking which was right — both
    # are, they are answering different questions). `build_local` is the
    # set of DISTINCT FILES (what actually has to be fixed — one soot
    # texture referenced by a thousand tree placements is one problem, not
    # a thousand); `build_local_bindings` is the raw, non-deduplicated
    # COUNT of (prim, attribute) pairs that reference one of them (what a
    # naive grep of the flattened file's raw text would find). Both are
    # always reported, always labeled, so neither number is mistaken for
    # the other again.
    # DEAD ANCESTRAL PATHS (2026-09-01, run 7). Some RenderPeople/DownTown
    # rig assets carry a breadcrumb baked in by their ORIGINAL AUTHOR's own
    # export tool at a path like `/home/myan2/Downloads/...` — measured
    # directly (`strings` over a shipped Fire/Suburban cell): it survives as
    # `omniverse://airlab-nucleus.andrew.cmu.edu:443/home/myan2/Downloads/
    # People/Assets/rp_manuel_rigged_001_ue4.usd` when the reference anchors
    # against Nucleus (the SAME `combine_urls`-anchors-an-absolute-path-
    # against-the-server behaviour this file already documents elsewhere),
    # or as a bare `/home/myan2/Downloads/...` local-looking absolute path
    # when it does not. EITHER WAY it has never resolved on ANY machine this
    # dataset has ever built on, including the build machine itself — unlike
    # every genuine build-local defect (a bake texture, a wrongly-local
    # mirror path), which DOES exist somewhere reachable and is what makes
    # it "build-LOCAL" rather than just broken. A path that resolves nowhere
    # at all cannot be fixed by rewriting or collecting it, and gating the
    # freeze on it forever would make the gate un-passable for a reason that
    # has nothing to do with THIS cell's own portability.
    #
    # CLASSIFIED BY THE TEST, NEVER BY MATCHING A STRING: `os.path.isfile`
    # on the bare local form. A path that fails it is `dead_ancestral`
    # (WARN, reported, never gates `portable_ok`); one that passes is a real
    # `build_local` defect (FAIL). Never allowlists `myan2` or any other
    # literal — a DIFFERENT dead author-machine path some other asset
    # carries is caught by the exact same test with no code change.
    build_local = set()
    build_local_bindings = 0
    dead_ancestral = set()
    dead_ancestral_bindings = 0
    runtime_builtin = set()
    for prim in Usd.PrimRange.Stage(st, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdShade.Shader):
            continue
        for attr in prim.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            val = attr.Get()
            ap = getattr(val, "path", "") or ""
            # A UE `/Game/...` breadcrumb is metadata, never a loadable file.
            if not ap or ap.startswith("/Game/"):
                continue
            if ap.startswith(("omniverse://", "http://", "https://")):
                continue
            if ap.startswith("/isaac-sim/kit/mdl/"):
                runtime_builtin.add(ap)
                continue
            if not (os.path.isabs(ap) and not os.path.abspath(ap).startswith(here)):
                continue
            if os.path.isfile(ap):
                build_local.add(ap)
                build_local_bindings += 1
            else:
                dead_ancestral.add(ap)
                dead_ancestral_bindings += 1

    # PORTABILITY, which is NOT the same question as self-containment and is
    # the one the first dataset failed. `external` lumps together two very
    # different things:
    #
    #   omniverse://...   PORTABLE. Any machine that can reach Nucleus can
    #                     resolve it, and it resolves identically whether the
    #                     cell is opened locally or from the server.
    #   /abs/local/path   FATAL. Only the build machine has it — and because a
    #                     cell is normally opened from `omniverse://`, the
    #                     resolver anchors an absolute path AGAINST THE SERVER
    #                     (`combine_urls` -> `omniverse://<host>/abs/local/...`),
    #                     so it cannot even be repaired by copying files onto
    #                     the consumer's disk.
    #
    # A cell with `collect=False` is legitimately not self-contained, so
    # `expect_self_contained` is False and `external` is tolerated — which is
    # exactly how 248 build-machine paths per cell shipped unnoticed. Local
    # absolutes are called out separately and are NEVER acceptable.
    out["build_local"] = sorted(build_local)
    out["build_local_bindings"] = build_local_bindings
    out["dead_ancestral"] = sorted(dead_ancestral)
    out["dead_ancestral_bindings"] = dead_ancestral_bindings
    out["runtime_builtin_assets"] = sorted(runtime_builtin)

    # THE CELL MUST CARRY ITS OWN LIGHT. Kit's flatten drops deactivated prims
    # outright, so deactivating the environment silently removed the sky and
    # every cell rendered black (overhead frame 99.93 % pure RGB(0,0,0)).
    out["lights"] = [p.GetPath().pathString for p in st.Traverse()
                     if p.HasAPI(UsdLux.LightAPI)]
    # "Has a light" is not the question — "can it light a square kilometre" is.
    # A shipped cell had exactly ONE light and still rendered black: a 0.25 m
    # SphereLight at (0, 0, 2.5) with intensity 1e5, whose 1/r^2 falloff is
    # nothing at 100 m, let alone at the 500 m plate edge. Only a DOME (ambient
    # everywhere) or a DISTANT light (parallel rays, no falloff) lights a plate.
    out["sky_lights"] = [p.GetPath().pathString for p in st.Traverse()
                         if p.IsA(UsdLux.DomeLight) or p.IsA(UsdLux.DistantLight)]

    # NO MATERIAL BINDING MAY POINT OUT OF ITS OWN /World/<scope>. A consumer
    # that references those scopes prim-by-prim gets the target dropped by USD
    # and the geometry renders untextured grey — this is what happened to the
    # 12 burnGround bands and the 11 tornado scourGround bands.
    cross = []
    world = st.GetPrimAtPath("/World")
    if world and world.IsValid():
        for scope in world.GetChildren():
            if scope.GetName() in ("PhysicsScene", "stage"):
                continue
            root = scope.GetPath().pathString
            for child in scope.GetChildren():
                mat, _r = UsdShade.MaterialBindingAPI(child).ComputeBoundMaterial(
                    materialPurpose=UsdShade.Tokens.full)
                if not (mat and mat.GetPrim().IsValid()):
                    continue
                mp = mat.GetPrim().GetPath().pathString
                if not mp.startswith(root + "/"):
                    cross.append("{0} -> {1}".format(child.GetPath(), mp))
    out["cross_scope_bindings"] = cross

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                           useExtentsHint=False)
    try:
        r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
        if not r.IsEmpty():
            lo, hi = r.GetMin(), r.GetMax()
            out["bbox_m"] = [round(v, 1) for v in
                             (lo[0], lo[1], lo[2], hi[0], hi[1], hi[2])]
    except Exception as exc:                              # pragma: no cover
        out["bbox_error"] = str(exc)

    # `ok` now gates on PORTABILITY too. A cell that renders black, renders
    # grey, or points at the build machine is not a cell anyone else can use,
    # whether or not it was meant to be self-contained.
    out["ok"] = (not out["unresolved"] and out["meshes"] > 0
                 and not out["build_local"]
                 and bool(out["sky_lights"])
                 and not out["cross_scope_bindings"]
                 and out["materials"]["ok"]
                 and (not out["external"] or not expect_self_contained))
    out["self_contained"] = not (out["external"] or out["unresolved"])

    # `portable_ok` — the ENFORCEMENT gate `export_scene` raises on. A
    # DELIBERATELY NARROWER verdict than `ok`: it never depends on
    # `unresolved`/`deps_scan_method`, because that number's reliability
    # varies with which scan produced it and this dataset does not want a
    # freeze to start silently passing or failing depending on whether the
    # rich scan happened to survive this run. `build_local`/`sky_lights`/
    # `cross_scope_bindings`/`meshes` are each computed by a technique
    # proven safe on every real cell (never touches `assetInfo`), so this
    # verdict is trustworthy on every run, not just the ones where
    # `ComputeAllDependencies` happens not to raise. This is the field that
    # answers "is this cell a repeat of the black-sky / grey-scar / build-
    # machine-path defects" with no asterisk.
    out["portable_ok"] = (out["meshes"] > 0 and not out["build_local"]
                          and bool(out["sky_lights"])
                          and not out["cross_scope_bindings"]
                          and out["materials"]["ok"])
    return out


# ---------------------------------------------------------------------------
# the pipeline (needs Kit)
# ---------------------------------------------------------------------------

#: Prims REMOVED from the stage before the flatten. Everything here is a build
#: or review aid that has no business in a shipped scene:
#:
#:   * `/World/ReviewCamera` and `/World/reviewCamRP` — the viewport and
#:     headless-Replicator snapshot helpers each re-use ONE camera prim for
#:     every capture, so the frozen scene would otherwise carry whatever pose
#:     the last shot happened to leave it at. Asked for explicitly, 2026-08-27.
#:   * the survivor and row-home LOCATOR POLES. They are authored deactivated
#:     on every run, which is right for a live session — but this is the scored
#:     artefact, and a 25 m magenta pole over each survivor group is the answer
#:     key one prim toggle away inside the file the searcher is given. The
#:     locations are in `GT_people.json`, where they belong.
#:   * Pegasus' default environment. It is loaded only to give the World a
#:     valid base and is deactivated immediately; leaving it in makes the
#:     collector fetch its assets into `Materials/` for nothing.
DEACTIVATE_DEFAULT = (
    "/World/ReviewCamera",
    "/World/reviewCamRP",
    "/World/stage/generated/_people_poles",
    "/World/stage/generated/_rowhome_poles",
    "/World/GroundPlane",
    # BOTH PATHS. Pegasus' default environment composes at `/World/Environment`
    # on a bare stage and at `/World/stage/Environment` once the generated plat
    # is authored under `/World/stage` — the run whose banner said it had
    # deactivated the Environment had turned off the wrong one, and the frozen
    # scene still carried the flat default ground.
    "/World/Environment",
    "/World/stage/Environment",
    # THE TWO THAT WERE BEING MISSED. Measured on every shipped Fire cell
    # (2026-08-30): `/World/GroundPlane` above never matched, because Pegasus'
    # default env composes under `/World/stage`, so BOTH the flat default
    # ground AND its stray bulb survived into all 18 cells while the sky —
    # the one thing worth keeping — was the only part successfully removed.
    # The SphereLight is a 0.25 m bulb at (0, 0, 2.5) with intensity 1e5: over
    # a 1 km plate it is a hotspot at the origin and nothing else.
    "/World/stage/GroundPlane",
    "/World/stage/SphereLight",
)

#: Where a portable cell points for the shared asset trees. A freeze rewrites
#: build-machine paths to this before the flatten — see `make_portable`. This
#: is the BLANKET rule (everything under `ASSET_LOCAL_PREFIX` mirrors 1:1
#: onto the same relative path under `ASSET_MIRROR`) — VERIFIED 2026-09-01
#: live against Nucleus for the `aec/` subtree specifically (`omni.client.
#: stat` on `.../Projects/SEI-COA/scene_gen/assets/aec/tower/Assets/
#: Vegetation/Shumard_Oak/Shumard_Oak.usd` -> OK, 9.9 MB), so the blanket
#: rule IS the right target for AEC content; `LOCAL_MIRROR_ROOTS` below is
#: only for subtrees that do NOT mirror onto this same root.
ASSET_LOCAL_PREFIX = "/isaac-sim/AirStack/scene_gen/assets/"
ASSET_MIRROR = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/"
                "Projects/SEI-COA/scene_gen/assets/")

#: LOCAL roots that do NOT mirror onto `ASSET_MIRROR` — checked BEFORE the
#: blanket `ASSET_LOCAL_PREFIX` rule (more specific wins), each as
#: `(local_root, nucleus_root)`. `people/` is the one confirmed case
#: (2026-09-01 coordinator report): the local cache lives under
#: `scene_gen/assets/people/...` but the real Nucleus originals are the
#: RenderPeople rigs at `/Projects/SEI-COA/People/Assets/...` — a
#: different, non-parallel structure. VERIFIED live: `omni.client.list` on
#: `Projects/SEI-COA/People` -> `Assets`, `RP_Scene.usd`; `Projects/
#: SEI-COA/People/Assets` -> the `rp_*_rigged_*`/`rp_*_posed_*` .usd/.mat
#: files this dataset's survivor figures reference.
LOCAL_MIRROR_ROOTS = (
    (ASSET_LOCAL_PREFIX + "people/",
     "omniverse://airlab-nucleus.andrew.cmu.edu:443/"
     "Projects/SEI-COA/People/Assets/"),
)


def deactivate_prims(stage, paths=DEACTIVATE_DEFAULT, verbose=True):
    """DEACTIVATE *paths* on *stage*. Returns the ones that were there.

    DEACTIVATE RATHER THAN REMOVE, on purpose. `stage.RemovePrim` edits the
    current EDIT TARGET, so on a composed stage it can report success and leave
    the prim composing from a stronger layer — measured: a run whose banner
    said it had stripped `/World/ReviewCamera` produced a frozen file that
    still contained it. `SetActive(False)` is authored as an opinion and
    composes, so it always takes.

    It is also the reversible choice, which is what this stage of the dataset
    wants: an inactive prim carries no geometry into a render and is one toggle
    from coming back if a later pass wants it. The removal pass belongs in the
    public-release cleanup, not here — see the freeze-dataset-state skill.
    """
    done = []
    for p in paths:
        prim = stage.GetPrimAtPath(p)
        if prim and prim.IsValid():
            prim.SetActive(False)
            done.append(p)
    if verbose:
        print("[freeze] deactivated {0} build/review prim(s){1}".format(
            len(done), (": " + ", ".join(done)) if done else ""))
    return done


def _collect_local_copy(src, dest_dir):
    """Copy *src* into *dest_dir*; returns the basename it landed under.

    Collision-safe: if a file of the same basename already exists there
    with DIFFERENT content, disambiguates with a short hash of the SOURCE
    PATH (not its content — cheap, and stable across repeated freezes of
    the same source) rather than silently overwriting an unrelated file
    that happens to share a name. Two attributes that reference the exact
    same source path land on the exact same destination file (no
    duplicate copies), because the check is "does a file already sit here
    with the CONTENT this source has" — a re-run of the same freeze is
    therefore idempotent and does not grow the destination folder.
    """
    import filecmp
    import hashlib

    os.makedirs(dest_dir, exist_ok=True)
    base = os.path.basename(src)
    dst = os.path.join(dest_dir, base)
    if os.path.exists(dst) and not filecmp.cmp(src, dst, shallow=False):
        stem, ext = os.path.splitext(base)
        h = hashlib.sha1(src.encode("utf-8")).hexdigest()[:8]
        base = "{0}_{1}{2}".format(stem, h, ext)
        dst = os.path.join(dest_dir, base)
    if not os.path.exists(dst):
        shutil.copy2(src, dst)
    return base


def _nucleus_target(path, local_prefix, mirror, extra_roots):
    """The Nucleus URL *path* WOULD map to under whichever known local
    mirror root it falls under, or `None` if it matches none.

    PURE STRING MATCHING — no network, no `pxr`, no `omni.client` — so this
    is unit-testable with no Kit at all. `extra_roots` (`LOCAL_MIRROR_ROOTS`
    shape) is checked FIRST, in order, so a more specific root (`people/`)
    is matched before it would otherwise fall through to the blanket
    `local_prefix`/`mirror` pair. Returning a URL here is a CANDIDATE only —
    the caller must still verify it actually exists (`_default_stat` /
    `make_portable`'s `stat_fn`) before trusting it; this function has no
    opinion on whether the target is real.
    """
    for local_root, nucleus_root in extra_roots:
        if path.startswith(local_root):
            return nucleus_root.rstrip("/") + "/" + path[len(local_root):]
    if path.startswith(local_prefix):
        return mirror.rstrip("/") + "/" + path[len(local_prefix):]
    return None


def _default_stat(url):
    """`True` if *url* resolves to a real file on Nucleus right now, else
    `False`. Never raises — an unreachable server, a missing extension, or
    a malformed URL all just mean "not verified", never a crash that would
    take a whole freeze down over one stat call. This is the ONLY function
    in this module that needs `omni.client` rather than `pxr`/stdlib, and
    it is imported lazily so importing `freeze.py` itself never requires
    it. `make_portable`'s `stat_fn` parameter exists specifically so a test
    can swap this out for a mock (see the module's own offline test)."""
    try:
        import omni.client as _oc
        r, _entry = _oc.stat(url)
        return str(r) == "Result.OK"
    except Exception:                                          # noqa: BLE001
        return False


def _resolve_local_path(path, *, local_prefix, mirror, extra_roots,
                        dest_dir, stat_fn, stat_cache, out):
    """The STRING this asset path should become, or *path* itself unchanged.

    Pure decision ladder — no `pxr`, no attribute mutation — used by
    `postflatten_repair`'s `UsdUtils.ModifyAssetPaths` callback (a plain
    string->string function is exactly what that API wants). Two routes:

      1. `_nucleus_target` names a candidate Nucleus URL for a known
         mirror root -> STAT it (`stat_fn`, CACHED by url so the same
         file referenced by thousands of placements costs one network
         round-trip, not thousands) -> if it exists, return that URL.
      2. No known root matched, OR the candidate failed its stat -> fall
         back to physically COLLECTING the file into `dest_dir` (the
         cell's own `Materials/<bake_local_subdir>/`) and returning a path
         relative to the cell — the same route a per-run bake texture with
         no Nucleus original at all already takes. The source file must
         actually exist on disk; a miss is counted and reported, and
         *path* is returned UNCHANGED rather than silently dropped (an
         unrepairable reference is still visible in the shipped file,
         which is what makes it show up in the NEXT `verify()` instead of
         disappearing).

    Mutates `out` (the caller's running tally) and returns the new string.
    """
    target = _nucleus_target(path, local_prefix, mirror, extra_roots)
    if target is not None:
        if target not in stat_cache:
            stat_cache[target] = stat_fn(target)
        if stat_cache[target]:
            out["rewritten"] += 1
            return target
        out["verify_failed"] += 1

    if not os.path.isfile(path):
        out["missing"] += 1
        print("[freeze] *** postflatten collect: source file does not "
              "exist, cannot repair -- {0}".format(path))
        return path
    if dest_dir is None:
        # No known mirror root verified AND no cell directory to collect
        # into (the caller never gave one, e.g. `make_portable(stage)` with
        # no `out_dir`) — leave *path* exactly as authored rather than
        # crashing on `_collect_local_copy(path, None)`. Counted the same
        # as any other collect-route failure so it is visible, never silent.
        out["missing"] += 1
        return path
    base = _collect_local_copy(path, dest_dir)
    out["collected"] += 1
    # `dest_dir` is `<out_dir>/<MATERIALS_DIR>/<bake_local_subdir>`, so its
    # own basename is `bake_local_subdir` — reconstructing the relative
    # string this way avoids passing yet another parameter through just to
    # repeat what `dest_dir` already encodes.
    return "{0}/{1}/{2}".format(MATERIALS_DIR, os.path.basename(dest_dir), base)


def _rss_mb():
    """Process peak RSS in MB, or `None` if `resource` is unavailable
    (non-Linux). `ru_maxrss` is a HIGH-WATER MARK, never falls — a
    before/after pair around de-instancing shows how much it RAISED the
    ceiling, not "current usage", which is the number worth watching for
    "did de-instancing blow up memory" regardless."""
    try:
        import resource
        return round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
                    / 1024.0, 1)
    except Exception:                                          # noqa: BLE001
        return None


def _shader_asset_paths(prim_range):
    """Every (prim, attr, path) triple in *prim_range* whose value is a
    non-empty, non-portable (`/Game/`, `omniverse://`, `http(s)://`
    excluded), ABSOLUTE, GENUINELY EXISTING local path — the same "is this
    even fixable" test `verify()`'s `dead_ancestral` split uses. A prim
    range over a PROTOTYPE is READ-ONLY safe (this never calls `.Set()`);
    a prim range over ordinary stage content also works and is what the
    live rewrite pass below reuses this generator for."""
    from pxr import Sdf, UsdShade
    for prim in prim_range:
        if not prim.IsA(UsdShade.Shader):
            continue
        for attr in prim.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            val = attr.Get()
            path = getattr(val, "path", "") or ""
            if not path or path.startswith(("/Game/", "omniverse://",
                                            "http://", "https://")):
                continue
            if path.startswith("/isaac-sim/kit/mdl/"):
                continue              # renderer-supplied core MDL module
            if os.path.isabs(path) and os.path.isfile(path):
                yield prim, attr, path


def make_portable(stage, mirror=ASSET_MIRROR, local_prefix=ASSET_LOCAL_PREFIX,
                  add_light=True, verbose=True, out_dir=None,
                  collect_bake_local=True, bake_local_subdir="bake_textures",
                  extra_mirror_roots=LOCAL_MIRROR_ROOTS, stat_fn=None,
                  deinstance_offenders=True, waive_above_instances=None,
                  max_rounds=6):
    """Repair, ON THE LIVE STAGE and BEFORE the flatten, everything needed
    to make the cell portable — INCLUDING content that lives inside an
    instancing prototype, via TARGETED DE-INSTANCING.

    2026-09-01, run 8 — THE SECOND PIVOT, and this is the final shape.
    Two earlier designs each failed for a different reason:

      * Run 7's `AIRSTACK_ASSET_ROOT` pivot (resolve `airstack://` against
        Nucleus at BUILD time, so a local path never gets composed at all)
        is a DEAD END: AEC pool-candidate discovery
        (`Reference_Brownstone*Row` variants) enumerates its own asset
        pool by LOCAL DIRECTORY LISTING, which cannot enumerate an
        `omniverse://` root — the city that gets built is a DIFFERENT
        city (measured: 1069 vs 1090 buildings, stable) than the one the
        bakes/manifests were solved against. Layout determinism is only
        proven with LOCAL roots, so the launcher builds with
        `AIRSTACK_ASSET_ROOT=` (empty) again.
      * Run 7 itself replaced run 4's pre-flatten MIRROR REWRITE because it
        cannot reach content inside an INSTANCING PROTOTYPE — USD refuses
        `Usd.Attribute.Set()` there outright. Run 7's own POST-flatten
        replacement (`postflatten_repair`) COULD reach prototype content
        but needed `UsdUtils.ModifyAssetPaths`, which unpacks every value
        in the layer and hits the same poisoned `assetInfo` crate values
        `ComputeAllDependencies` already cannot survive — measured
        (run 7): identical `_UnpackValue: unsupported type enum value 0`,
        plus a live-Kit-process side effect that mutated the shipped file
        on disk despite the exception. Never called from a live Kit
        process again — see `postflatten_repair`'s own docstring.

    THE FIX: since a prototype's content becomes ordinary, individually
    editable prims the moment its INSTANCES stop being instances
    (`Usd.Prim.SetInstanceable(False)` on the instance root — legal,
    ordinary live-stage authoring, the same call
    `urban_fire_city_launch_script._uninstance_gprim_roots` already makes
    for a different bug), TARGETED de-instancing closes the gap run 4's
    mirror rewrite had: read-only scan every prototype for a genuinely
    fixable (existing-somewhere) local path; for exactly the offending
    ones, de-instance every CURRENT instance; then run the SAME verified
    mirror-rewrite-or-collect walk run 4 already proved correct, now via a
    plain `stage.Traverse()` that reaches the de-instanced content because
    it is no longer collapsed into a prototype.

    NESTED INSTANCING, AND WHY THIS IS A LOOP TO A FIXPOINT, NOT ONE PASS
    (2026-09-01, run 9). The AEC packs are internally instanced — a tree's
    own leaf/bark sub-meshes, a RenderPeople rig's own texture-bearing
    sub-prims — so de-instancing the OUTER placement exposes an INNER
    subtree that is ITSELF still an instance of an INNER prototype, one
    level deeper, invisible to the single pass that only ever queried
    `stage.GetPrototypes()` once. MEASURED: a single pass de-instanced 16
    outer prototypes / 453 instances and the gate still failed with MORE
    build-local bindings than before (`verify()`'s own read walk always
    saw through every nesting level correctly; the write pass did not).
    `max_rounds` bounds a loop that RE-QUERIES `stage.GetPrototypes()`
    every round (a de-instanced subtree's own still-instanceable children
    are NEW, newly-discoverable prototypes) and re-runs the (idempotent)
    rewrite pass after each round's de-instancing, so whatever a round
    exposes is repaired before the next round's scan runs. Stops on
    convergence (a round finds nothing offending), on hitting
    `max_rounds`, or — worth flagging on its own — on a round that finds
    offenders but de-instances NOTHING (every remaining one is either
    already permanently waived or freshly over the instance cap): treated
    as UN-PEELABLE and waived immediately rather than spinning.

    OPEN QUESTION, UNRESOLVED (2026-09-01, run 9, L2K1 re-freeze log —
    next session should start here). On the real pod cell the fixpoint
    itself reported CONVERGED after only 1 round (16 prototypes / 453
    instances de-instanced, the rewrite pass logging 693 rewrites + 9,973
    collects — a round 2 scan found nothing left offending) — but the COLD
    re-`verify()` of the flattened file still showed the SAME 135 files /
    ~105,400 build-local bindings as before any of this ran. That is not
    the nested-instancing gap this loop was built to close (the loop
    genuinely converged, on the LIVE stage, by its own read of
    `stage.GetPrototypes()`); it is a SECOND, still-undiagnosed gap
    between what the live-stage scan can see and what actually survives
    Kit's flatten into the cold file — every offending path the loop
    thought it fixed came back in the flattened output regardless. Two
    fixture-proven mechanisms (this loop, and the two-level nesting test
    in `freeze_fixpoint_roundtrip.py`) cannot reproduce it: it may be
    Kit's flatten re-instancing/re-collapsing content the live scan saw as
    plain prims, an EDIT TARGET the de-instancing writes landed on that
    the flatten does not read from, or something in the 693-rewrite /
    9,973-collect pass itself not actually taking on THIS cell's specific
    prototype shapes. Root-causing it was not affordable before the
    L2K1/L3K1 deadline (pod window <5h, user descoped to k=1 cells only);
    `FREEZE_WAIVE_MIRRORED` (see `_enforce_portable`) is the engineered
    exit that ships tonight without resolving it. Whoever picks this up:
    start from the L2K1 freeze_report.json (`mirror_waiver_audit` will
    show exactly which paths rode on the waiver) and a cold `verify()`
    immediately after `make_portable` converges but BEFORE the flatten —
    if a fresh `Usd.PrimRange.Stage(st, Usd.TraverseInstanceProxies())`
    scan of the STILL-LIVE, not-yet-flattened stage also shows the
    de-instanced paths as clean, the divergence is Kit's flatten step
    itself, not this function.

    THE COST, AND THE SAFETY VALVE. De-instancing MULTIPLIES the affected
    subtree's geometry across every instance that used to share it — the
    exact cost instancing exists to avoid. `waive_above_instances` (an int,
    or `None` for "always de-instance everything offending") caps this: a
    prototype whose CURRENT instance count exceeds it is left instanced
    and its offending paths are WAIVED instead — recorded in
    `out["waived_mirror_paths"]`, explicitly, by `export_scene`/
    `_enforce_portable`, NEVER silently dropped, and never gating
    `portable_ok` on their own. A waived path must independently be
    confirmed to exist on the Nucleus mirror at the equivalent relative
    location — `_nucleus_target` + `stat_fn`, the SAME verification the
    rewrite route itself uses — so "waived" always means "a consumer who
    can reach the mirror resolves this fine", never "we gave up and hoped".

    Everything else is unchanged from run 7:

    1. BAKE-LOCAL TEXTURES / MIRROR REWRITE. `disaster.fire_bake`'s
       soot/scorch PNGs have no Nucleus mirror at all (per-run outputs);
       everything else under a known mirror root (`local_prefix`,
       `extra_mirror_roots`) gets a STAT-VERIFIED (`stat_fn`, cached per
       target URL) rewrite to its Nucleus equivalent, falling back to the
       same physical-collect-into-the-cell route when unverified or
       unmapped. A source that resolves NOWHERE at all is left untouched;
       `verify()`'s `dead_ancestral` classification is what that shape is
       for, not this pass.

    2. NO LIGHT. `DEACTIVATE_DEFAULT` turns off the Environment, and Kit's
       flatten DROPS deactivated prims entirely rather than writing them
       inactive (measured: 0 of 83,215 prims in a shipped cell carry
       `active=False`). An explicit dome + sun is authored here so the
       cell is lit by construction.

    3. CROSS-SCOPE MATERIAL BINDINGS. `/World/burnGround/band_*` bound to
       `/World/stage/generated/BurnLooks/band_*` — a consumer that
       references `/World/<name>` prim-by-prim gets the binding DROPPED by
       USD (different composition arcs), rendering untextured grey. Moving
       the looks INSIDE the overlay scope fixes it.

    `stat_fn` exists so a test can mock it (see the module's own offline
    test). Returns a dict of counts, including a `deinstance` sub-dict and
    `waived_mirror_paths`; safe to call twice.
    """
    from pxr import Sdf, Usd, UsdGeom, UsdLux, UsdShade

    stat_fn = stat_fn or _default_stat
    stat_cache = {}
    out = {"looks_moved": 0, "rebound": 0, "lights": 0,
          "rewritten": 0, "verify_failed": 0, "collected": 0, "missing": 0,
          "deinstance": {"rounds": 0, "converged": True,
                        "prototypes_offending": 0,
                        "prototypes_deinstanced": 0,
                        "prototypes_waived": 0, "instances_deinstanced": 0,
                        "rss_before_mb": None, "rss_after_mb": None,
                        "per_round": []},
          "waived_mirror_paths": []}

    # Repair measured defects in the third-party source assets before the
    # mirror/collect pass.  The repair materials are therefore subjected to
    # exactly the same portability rewrite as every other material.
    from . import material_repair
    out["material_repairs"] = material_repair.repair_known(
        stage, verbose=verbose)

    dest_dir = (os.path.join(out_dir, MATERIALS_DIR, bake_local_subdir)
               if (out_dir and collect_bake_local) else None)

    def _rewrite_pass():
        """The verified-mirror-rewrite-or-collect walk, over a PLAIN
        `stage.Traverse()` — reaches whatever is CURRENTLY not collapsed
        into a prototype. Idempotent: a path already rewritten to a URL or
        a cell-relative string is skipped on the next call, which is what
        makes calling this once per round (instead of only once, ever)
        safe and cheap on the rounds that find nothing new."""
        for prim in stage.Traverse():
            if not prim.IsA(UsdShade.Shader):
                continue
            for attr in prim.GetAttributes():
                if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                    continue
                val = attr.Get()
                path = getattr(val, "path", "") or ""
                if not path or path.startswith(("/Game/", "omniverse://",
                                                "http://", "https://")):
                    continue
                if path.startswith("/isaac-sim/kit/mdl/"):
                    continue          # renderer-supplied core MDL module
                if not os.path.isabs(path):
                    continue                  # already relative/portable
                if not os.path.isfile(path):
                    out["missing"] += 1       # dead_ancestral's shape;
                    continue                  # verify() reports it, not this
                new_path = _resolve_local_path(
                    path, local_prefix=local_prefix, mirror=mirror,
                    extra_roots=extra_mirror_roots, dest_dir=dest_dir,
                    stat_fn=stat_fn, stat_cache=stat_cache, out=out)
                if new_path != path:
                    attr.Set(Sdf.AssetPath(new_path))

    # --- 0. TARGETED DE-INSTANCING, TO A FIXPOINT (2026-09-01, run 9).
    # NESTED INSTANCING: the AEC packs are internally instanced (their own
    # sub-prims, e.g. a tree's individual leaf/bark meshes, or a
    # RenderPeople rig's own texture-bearing sub-prims), so de-instancing
    # the OUTER placement exposes an INNER subtree that is ITSELF still an
    # instance of an INNER prototype — one peel is never enough. MEASURED
    # (run 9): a single pass de-instanced 16 outer prototypes / 453
    # instances and the gate still failed with MORE bindings than before
    # (135 files / 105,400 bindings) — `verify()`'s own
    # `Usd.TraverseInstanceProxies()` walk already saw through every
    # nesting level for READING, so it always reported the true total; the
    # single WRITE pass only ever peeled the outermost layer.
    #
    # Bounded loop: each round, re-query `stage.GetPrototypes()` FRESH (a
    # de-instanced outer subtree's own still-instanceable children are
    # NEW, newly-discoverable prototypes USD did not need to report before
    # — they were already collapsed one level deeper), de-instance every
    # CURRENTLY-offending one (or waive it, same instance-count threshold,
    # applied FRESH each round so an early round's cheap outer prototype
    # does not accidentally protect an expensive inner one under the same
    # cap), then re-run the (idempotent) rewrite pass so anything freshly
    # exposed gets repaired before the NEXT round's scan. Stops when a
    # round finds nothing offending (converged) or `max_rounds` is
    # reached; a round that finds offenders but de-instances NOTHING (every
    # remaining offender is either already permanently waived or newly
    # over the instance cap) is treated as UN-PEELABLE — its remaining
    # paths are waived immediately rather than spinning for no reason.
    permanently_waived_protos = set()
    waived_paths = set()
    if deinstance_offenders:
        _deinstance_rss_start = _rss_mb()
        for round_num in range(1, max_rounds + 1):
            protos = list(stage.GetPrototypes())
            offending = {}
            for proto in protos:
                pp = proto.GetPath()
                if pp in permanently_waived_protos:
                    continue
                hit = next(_shader_asset_paths(Usd.PrimRange(proto)), None)
                if hit is not None:
                    offending[pp] = proto
            round_stats = {"round": round_num,
                          "prototypes_offending": len(offending),
                          "prototypes_deinstanced": 0,
                          "prototypes_waived": 0, "instances_deinstanced": 0,
                          "rss_before_mb": None, "rss_after_mb": None}
            out["deinstance"]["per_round"].append(round_stats)
            if not offending:
                # CONVERGED -- but a round that converges immediately (no
                # prototypes ever offended, or a prior round already fixed
                # everything reachable through instancing) must still run
                # the ordinary rewrite pass at least once: `_rewrite_pass`
                # is the ONLY thing that fixes local absolute paths on
                # plain, never-instanced content, and every other branch
                # below calls it before breaking. Skipping it here (as an
                # earlier version of this loop did) silently shipped every
                # non-instanced shader path unrewritten whenever a stage
                # had no offending prototypes on round 1 -- caught by the
                # bake-local-collect regression test.
                _rewrite_pass()
                break

            proto_instances = {}
            for prim in stage.Traverse():
                if not prim.IsInstance():
                    continue
                proto = prim.GetPrototype()
                pp = proto.GetPath() if proto else None
                if pp in offending:
                    proto_instances.setdefault(pp, []).append(prim)

            to_deinstance, to_waive = {}, {}
            for pp, proto in offending.items():
                insts = proto_instances.get(pp, [])
                if (waive_above_instances is not None
                        and len(insts) > waive_above_instances):
                    to_waive[pp] = (proto, insts)
                else:
                    to_deinstance[pp] = (proto, insts)

            round_stats["rss_before_mb"] = _rss_mb()
            n_inst = 0
            for _pp, (_proto, insts) in to_deinstance.items():
                for prim in insts:
                    if prim.SetInstanceable(False):
                        n_inst += 1
            round_stats["rss_after_mb"] = _rss_mb()
            round_stats["prototypes_deinstanced"] = len(to_deinstance)
            round_stats["instances_deinstanced"] = n_inst
            round_stats["prototypes_waived"] = len(to_waive)

            for pp, (proto, _insts) in to_waive.items():
                permanently_waived_protos.add(pp)
                for _prim, _attr, path in _shader_asset_paths(
                        Usd.PrimRange(proto)):
                    waived_paths.add(path)

            if verbose:
                print("[freeze] portable: de-instancing round {0} -- {1} "
                      "offending prototype(s), {2} de-instanced ({3} "
                      "instance(s) affected), {4} waived -- RSS {5} -> "
                      "{6} MB".format(
                          round_num, round_stats["prototypes_offending"],
                          round_stats["prototypes_deinstanced"],
                          round_stats["instances_deinstanced"],
                          round_stats["prototypes_waived"],
                          round_stats["rss_before_mb"],
                          round_stats["rss_after_mb"]))

            # UN-PEELABLE: offenders remain but NOTHING was de-instanced
            # this round (everything left is either freshly over the
            # instance cap, or -- the case worth flagging loudly -- ALL of
            # it was already waived in an earlier round and is STILL
            # showing up here only because `SetInstanceable(False)` was
            # somehow refused). Waive whatever is left standing and stop
            # spinning rather than burn `max_rounds` for no further
            # progress.
            if n_inst == 0:
                for pp, proto in offending.items():
                    if pp in to_deinstance:    # attempted but 0 succeeded
                        permanently_waived_protos.add(pp)
                        for _prim, _attr, path in _shader_asset_paths(
                                Usd.PrimRange(proto)):
                            waived_paths.add(path)
                if verbose:
                    print("[freeze] *** round {0} de-instanced ZERO "
                          "instances while {1} prototype(s) still offend "
                          "-- UN-PEELABLE, waiving the remainder rather "
                          "than spinning".format(
                              round_num, round_stats["prototypes_offending"]))
                _rewrite_pass()
                out["deinstance"]["converged"] = False
                out["deinstance"]["rounds"] = round_num
                break

            _rewrite_pass()      # idempotent -- repairs whatever THIS
                                 # round exposed before the NEXT round's
                                 # scan looks for what is still offending
            out["deinstance"]["rounds"] = round_num
        else:
            # the `for` loop's own `else`: ran out of `max_rounds` without
            # ever hitting `break` (neither converged nor detected
            # un-peelable) -- still offending, waive whatever the LAST
            # round found rather than ship silently broken.
            out["deinstance"]["converged"] = False
            final_protos = [p for p in stage.GetPrototypes()
                           if p.GetPath() not in permanently_waived_protos]
            for proto in final_protos:
                hits = list(_shader_asset_paths(Usd.PrimRange(proto)))
                if hits:
                    permanently_waived_protos.add(proto.GetPath())
                    for _prim, _attr, path in hits:
                        waived_paths.add(path)
            if verbose and waived_paths:
                print("[freeze] *** hit max_rounds={0} still offending -- "
                      "waiving the remainder".format(max_rounds))

        d = out["deinstance"]
        d["prototypes_offending"] = sum(r["prototypes_offending"]
                                        for r in d["per_round"])
        d["prototypes_deinstanced"] = sum(r["prototypes_deinstanced"]
                                          for r in d["per_round"])
        d["prototypes_waived"] = len(permanently_waived_protos)
        d["instances_deinstanced"] = sum(r["instances_deinstanced"]
                                         for r in d["per_round"])
        # bracket the WHOLE loop, not the last per-round entry -- the last
        # round is often the empty "nothing offending" convergence check,
        # which never measures RSS (nothing to measure), so picking
        # per_round[-1] would silently report rss_after_mb=None even
        # though real de-instancing work happened in an earlier round.
        d["rss_before_mb"] = _deinstance_rss_start
        d["rss_after_mb"] = _rss_mb()
        if verbose:
            print("[freeze] portable: de-instancing fixpoint {0} after "
                  "{1} round(s) -- {2} prototype(s) de-instanced total "
                  "({3} instance(s)), {4} waived".format(
                      "CONVERGED" if d["converged"] else "STOPPED",
                      d["rounds"], d["prototypes_deinstanced"],
                      d["instances_deinstanced"], d["prototypes_waived"]))
            if waived_paths:
                print("[freeze] *** {0} path(s) WAIVED across the whole "
                      "fixpoint -- see waived_mirror_paths in the report"
                      .format(len(waived_paths)))
    else:
        _rewrite_pass()          # de-instancing off entirely -- still run
                                 # the plain (non-prototype) rewrite once
    out["waived_mirror_paths"] = sorted(waived_paths)

    # --- cross-scope bindings -----------------------------------------------
    # The source material may live in a weaker sublayer/reference.  Copying
    # only from the root layer therefore silently missed it.  Clone from the
    # composed prim stack instead, which also makes this safe for repair
    # wrappers around already-frozen crates.
    cross_report = material_repair.repair_cross_scope(stage, verbose=verbose)
    out["looks_moved"] += cross_report["looks_moved"]
    out["rebound"] += cross_report["rebound"]
    out["cross_scope_unresolved"] = cross_report["unresolved"]

    # --- 2. the light ------------------------------------------------------
    if add_light:
        dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/FrozenDome"))
        dome.CreateIntensityAttr(1000.0)
        dome.CreateExposureAttr(0.0)
        sun = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/FrozenSun"))
        sun.CreateIntensityAttr(2400.0)
        sun.CreateAngleAttr(0.53)
    out["lights"] = sum(1 for p in stage.Traverse()
                        if p.HasAPI(UsdLux.LightAPI))

    if verbose:
        print("[freeze] portable (pre-flatten): {0} asset path(s) -> "
              "VERIFIED mirror target(s) ({1} failed verification), {2} "
              "collected into the cell ({3} missing/dead), {4} look(s) "
              "moved into scope, {5} re-bound, {6} active light(s)".format(
                  out["rewritten"], out["verify_failed"], out["collected"],
                  out["missing"], out["looks_moved"], out["rebound"],
                  out["lights"]))
    return out


def postflatten_repair(out_usd, mirror=ASSET_MIRROR,
                       local_prefix=ASSET_LOCAL_PREFIX,
                       extra_mirror_roots=LOCAL_MIRROR_ROOTS,
                       collect_dir_name="bake_textures", stat_fn=None,
                       verbose=True):
    """*** NOT CALLED FROM `export_scene` ANY MORE (2026-09-01, run 7) —
    DO NOT WIRE THIS BACK IN FROM A LIVE KIT PROCESS. ***

    Kept in this module, UNUSED, as a record of why the post-flatten
    approach does not work, not as a mechanism to reuse. Two independent
    failures, on the SAME real cell:

      * `UsdUtils.ModifyAssetPaths` must unpack every value in the layer to
        find the asset-path ones, and on a real kit-written crate that walk
        hits the identical poisoned `assetInfo` field `ComputeAllDependencies`
        already cannot survive elsewhere in this file — measured (run 7):
        it raised `_UnpackValue: unsupported type enum value 0`, the exact
        error this module's own docstring already predicted for exactly
        this shape of API ("any API that must unpack every value").
      * The temp-file safety net below only protects the explicit
        `Export()`/`os.replace()` step. MEASURED: the cell's `.usd`
        mtime/size still CHANGED during the SAME failed run, despite the
        exception firing before `Export()` was ever reached in the code —
        which can only mean something in the live Kit process (not this
        function) flushed the partially-mutated in-memory `Sdf.Layer` to
        disk on its own once `ModifyAssetPaths` had touched it. Opening an
        already-shipped cell's OWN layer via `Sdf.Layer.FindOrOpen` and
        mutating it in memory inside a live Kit process is unsafe
        regardless of whether THIS function ever calls `.Save()`/`.Export()`
        itself.

    The replacement is `AIRSTACK_ASSET_ROOT` pointed at the Nucleus mirror
    at LAUNCH TIME (an env var the launcher sets, not a repair pass here) —
    never let the wrong path get composed onto the stage in the first
    place, so there is nothing to rewrite after. See `make_portable`'s own
    docstring for the current, safe division of repairs.

    Everything below this line is UNCHANGED from when it was live — the
    original docstring, for the historical record:

    THE AUTHORITATIVE asset-path repair. Runs AFTER the Kit flatten,
    directly on the layer that will actually ship — this supersedes
    `make_portable`'s old pre-flatten asset-path pass (removed 2026-09-01;
    see that function's own docstring for the short version).

    WHY POST-FLATTEN (2026-09-01, run 4 diagnosis). The pre-flatten pass
    edited via `Usd.Stage`/`Usd.Attribute.Set()`, and USD refuses that
    outright for anything living inside an INSTANCING PROTOTYPE
    ("authoring to an instancing prototype is not allowed") — and
    `instance_placements: true` means most of a city composes through
    prototypes. MEASURED on run 3: the pre-flatten pass verified/rewrote
    306 paths on the LIVE stage — everything it could reach — but the
    FROZEN file still carried 131 unique build-local files across 69,346
    bindings, essentially all of it inside prototype content the
    pre-flatten pass could only DETECT (read-only), never touch. The
    flatten then bakes those original, unrepaired paths straight into the
    export. Pre-flatten authoring can never win this: a prototype is a
    synthesized composition RESULT with no spec of its own to author
    against until the layer that will exist AFTER the flatten actually
    contains it as ordinary specs.

    `UsdUtils.ModifyAssetPaths(layer, fn)` is the fix. It operates at the
    `Sdf` LAYER level — plain specs, no `Usd.Stage` instancing semantics,
    no "cannot edit a prototype" guard at all — the same technique
    `_prefix_asset_paths` already uses for the (dormant) collect path,
    whose own docstring already explains why it works where a stage walk
    does not: "Walks SPECS rather than composed prims, so it also catches
    attributes inside prototypes." `ModifyAssetPaths` applies `fn` to
    EVERY asset path in the layer — attribute defaults, references,
    payloads, sublayers alike — not just the Shader-attribute subset the
    pre-flatten pass and `verify()`'s own `build_local` walk look at.

    `fn` is `_resolve_local_path`, the SAME decision ladder as before,
    wired as a pure string->string callback instead of an
    `attr.Set()` call: a path under a known mirror root whose Nucleus
    target STATS OK (cached per target) -> that `omniverse://` URL;
    anything else absolute and local -> physically COLLECTED into
    `<out_dir>/Materials/<collect_dir_name>/` and rewritten to a path
    RELATIVE to the cell; already-relative or already-portable
    (`omniverse://`, `http(s)://`, `/Game/`) -> UNCHANGED. That last case
    is also what makes a RE-RUN of this function idempotent: a previously
    collected reference is already relative by the time a second pass
    would see it, so nothing gets re-collected or re-stat'd for it.

    SAFETY: never writes `out_usd` in place. `ModifyAssetPaths` mutates the
    layer IN MEMORY; the result is `Export()`ed to a TEMP file first, and
    only moved over `out_usd` (`os.replace`, atomic on the same filesystem)
    if that succeeds. Re-serialising a kit-written crate has never been
    proven reliable (see freeze-portable-scenes' account of `Sdf.Layer.
    Export` raising on the poisoned `assetInfo` field elsewhere in this
    codebase) — if `Export` fails here, this function raises and the
    ORIGINAL, already-flattened file at `out_usd` is untouched, not a
    half-repaired one.

    Returns a dict of counts, `seconds` (wall time — split into
    `modify_s`/`export_s`), and `rss_mb` (peak process RSS, best-effort) —
    report every field every time; the flattened layer for a full city
    cell is measured at ~866 MB, and this is the number to watch as that
    scales.
    """
    from pxr import Sdf, UsdUtils

    stat_fn = stat_fn or _default_stat
    stat_cache = {}
    out_dir = os.path.dirname(os.path.abspath(out_usd))
    dest_dir = os.path.join(out_dir, MATERIALS_DIR, collect_dir_name)
    counts = {"rewritten": 0, "verify_failed": 0, "collected": 0,
             "missing": 0, "unchanged": 0}

    def _fn(path):
        if not path:
            return path
        if path.startswith(("omniverse://", "http://", "https://",
                            "/Game/")):
            counts["unchanged"] += 1
            return path
        if not os.path.isabs(path):
            counts["unchanged"] += 1
            return path                          # already relative/portable
        return _resolve_local_path(
            path, local_prefix=local_prefix, mirror=mirror,
            extra_roots=extra_mirror_roots, dest_dir=dest_dir,
            stat_fn=stat_fn, stat_cache=stat_cache, out=counts)

    layer = Sdf.Layer.FindOrOpen(out_usd)
    if layer is None:
        raise RuntimeError("postflatten_repair: cannot open " + out_usd)

    t0 = time.time()
    UsdUtils.ModifyAssetPaths(layer, _fn)
    t_modify = time.time() - t0

    tmp_out = out_usd + ".repair_tmp"
    if os.path.exists(tmp_out):
        os.remove(tmp_out)
    t1 = time.time()
    ok = layer.Export(tmp_out)
    t_export = time.time() - t1
    if not ok:
        if os.path.exists(tmp_out):
            os.remove(tmp_out)
        raise RuntimeError(
            "postflatten_repair: layer.Export failed writing {0} -- the "
            "original flatten output at {1} is UNTOUCHED".format(
                tmp_out, out_usd))
    os.replace(tmp_out, out_usd)

    dt = t_modify + t_export
    counts["seconds"] = round(dt, 1)
    counts["modify_s"] = round(t_modify, 1)
    counts["export_s"] = round(t_export, 1)
    try:
        import resource
        counts["rss_mb"] = round(
            resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0, 1)
    except Exception:                                          # noqa: BLE001
        counts["rss_mb"] = None
    if verbose:
        print("[freeze] postflatten repair: {0:.1f}s ModifyAssetPaths + "
              "{1:.1f}s Export = {2:.1f}s total{3} -- {4} rewritten to a "
              "verified Nucleus target, {5} failed verification, {6} "
              "collected into the cell, {7} missing source, {8} already "
              "portable/relative (no-op)".format(
                  t_modify, t_export, dt,
                  "" if counts["rss_mb"] is None
                  else " (peak RSS {0:.0f} MB)".format(counts["rss_mb"]),
                  counts["rewritten"], counts["verify_failed"],
                  counts["collected"], counts["missing"], counts["unchanged"]))
    return counts


class PortabilityError(RuntimeError):
    """Raised by `_enforce_portable`. Carries the FULL `verify()` result as
    `.info` — 2026-09-01 finding: a caller's `except Exception:` block that
    only prints `str(exc)` and moves on loses the whole `info` dict the
    moment this raises, so the `freeze_report.json` it would otherwise
    write either does not get written at all, or gets written with
    `portable_ok` (and everything else) as `None` by whatever placeholder
    the caller falls back to. A caller that wants a COMPLETE, real report
    even on a failed freeze should catch `PortabilityError` specifically
    and use `exc.info` — every field `verify()` computed is right there,
    it is only the FILE that must not ship (never uploaded, never
    referenced downstream), not the diagnosis of why it can't."""

    def __init__(self, message, info):
        super().__init__(message)
        self.info = info


def _enforce_portable(info, waived_paths=None, *, waive_mirrored=False,
                      mirror=ASSET_MIRROR, local_prefix=ASSET_LOCAL_PREFIX,
                      extra_mirror_roots=LOCAL_MIRROR_ROOTS, stat_fn=None):
    """RAISE if the cell is not shippable — the enforcement half of what
    `verify()` only DETECTS. 2026-09: the first real city freeze shipped a
    `freeze_report.json` that correctly listed dozens of `build_local`
    paths and an `ok: False` verdict, and NOTHING downstream treated that
    as a failure — the launcher printed the report and moved on to close
    the app, same as a clean run. `export_scene` already "Raises on a
    failed flatten or a failed collect — a half-written dataset cell is
    worse than none"; a cell that flattens fine but still points at the
    build machine is the SAME category of half-written, just silent about
    it, and now raises the same way.

    `waived_paths` (2026-09-01, run 8, `FREEZE_WAIVE_VEGETATION`):
    specific `build_local` paths `make_portable`'s targeted de-instancing
    explicitly decided NOT to de-instance (too many instances to do
    safely) and left build-local ON PURPOSE, having independently
    confirmed each one exists on the Nucleus mirror at the equivalent
    relative location. `verify()` itself never waives anything — it is a
    context-free, pure "open cold and report the truth" function, and
    still reports these paths in `build_local` exactly as it found them.
    THIS function is where the build-time decision to ship anyway is
    applied, and it is never silent about it: `info["waived_mirror_paths"]`
    always names exactly what was let through, `info["portable_ok"]` is
    OVERWRITTEN to reflect the post-waiver verdict (so a reader trusts one
    field), and `info["portable_ok_before_waiver"]` keeps the original,
    unwaived verdict for anyone auditing the decision.

    `waive_mirrored` (2026-09-01, run 9 endgame — `FREEZE_WAIVE_MIRRORED`):
    a SECOND, independent waiver source, for the gap the fixpoint
    de-instancing could not close in time (see `make_portable`'s dated
    note on the scan-vs-flatten divergence). For every `build_local` path
    NOT already covered by `waived_paths`, this computes its
    `_nucleus_target` and STATS it (cached; the exact same verified-mirror
    machinery `make_portable`'s own rewrite route uses, not a new
    mechanism) — a path with a real, existing Nucleus twin at the mirrored
    location is waived (the 207-file true-up made the mirror content-
    identical to the local tree, so a consumer with either the mirror or
    the local checkout resolves it, whichever it was actually shipped
    with); a path with NO verified twin is left standing and still fails
    the gate. NEVER waives a twin-less path — `info["mirror_waiver_audit"]`
    is the full per-path record (`path`, candidate `target`, `verified`)
    for every path this checked, so nothing here is silent. Off by
    default: `waived_paths` (the de-instancing-time waiver) stays the
    primary, narrower mechanism; this is the gate-side fallback for
    whatever it could not reach.

    Gated on `build_local`/`sky_lights`/`cross_scope_bindings`/materials/`meshes`,
    NOT the broader `ok` — `ok` also factors in `unresolved`, whose scan
    can legitimately be the narrower `"fallback_shader_attrs"` pass on a
    kit-poisoned cell (see `verify`'s own docstring); this raise is not at
    the mercy of that.
    """
    waived = set(waived_paths or ())
    bl = list(info.get("build_local") or [])

    if waive_mirrored:
        _stat = stat_fn or _default_stat
        _cache = {}
        audit = []
        for p in bl:
            if p in waived:
                continue
            target = _nucleus_target(p, local_prefix, mirror,
                                     extra_mirror_roots)
            if target is None:
                audit.append({"path": p, "target": None, "verified": False})
                continue
            if target not in _cache:
                _cache[target] = _stat(target)
            ok = _cache[target]
            audit.append({"path": p, "target": target, "verified": ok})
            if ok:
                waived.add(p)
        info["mirror_waiver_audit"] = audit
        newly_waived = sum(1 for a in audit if a["verified"])
        twinless = [a["path"] for a in audit if not a["verified"]]
        if newly_waived:
            print("[freeze] FREEZE_WAIVE_MIRRORED: {0}/{1} remaining "
                  "build-local path(s) have a VERIFIED Nucleus twin -- "
                  "waiving (see mirror_waiver_audit in the report)".format(
                      newly_waived, len(audit)))
        if twinless:
            print("[freeze] FREEZE_WAIVE_MIRRORED: {0} path(s) have NO "
                  "verified Nucleus twin -- still gate the freeze -- e.g. "
                  "{1}".format(len(twinless), twinless[:4]))

    info["waived_mirror_paths"] = sorted(p for p in bl if p in waived)
    effective_bl = [p for p in bl if p not in waived]

    info["portable_ok_before_waiver"] = info.get("portable_ok")
    gate_ok = (bool(info.get("meshes")) and not effective_bl
              and bool(info.get("sky_lights"))
              and not info.get("cross_scope_bindings")
              and bool((info.get("materials") or {}).get("ok")))
    info["portable_ok"] = gate_ok
    if gate_ok:
        if info["waived_mirror_paths"] and info.get("portable_ok_before_waiver") is False:
            print("[freeze] portable_ok=True ONLY because {0} path(s) were "
                  "WAIVED -- see waived_mirror_paths in the report".format(
                      len(info["waived_mirror_paths"])))
        return
    lines = ["[freeze] *** PORTABILITY GATE FAILED -- {0}".format(info["path"])]
    if effective_bl:
        lines.append("  {0} unique build-local file(s) (of {1} total, {2} "
                     "waived), {3} attribute binding(s) -- e.g. {4}".format(
                         len(effective_bl), len(bl),
                         len(info["waived_mirror_paths"]),
                         info.get("build_local_bindings", "?"),
                         effective_bl[:4]))
    if not info.get("sky_lights"):
        lines.append("  NO sky light (DomeLight/DistantLight) -- renders black")
    if info.get("cross_scope_bindings"):
        lines.append("  {0} cross-scope material binding(s) -- renders grey"
                     .format(len(info["cross_scope_bindings"])))
    if not (info.get("materials") or {}).get("ok"):
        counts = (info.get("materials") or {}).get("counts") or {}
        lines.append("  material resolution failed: {0} dangling, {1} "
                     "typeless, {2} unbound/uncoloured, {3} surface-less"
                     .format(counts.get("dangling_targets", 0),
                             counts.get("typeless_targets", 0),
                             counts.get("unbound_uncolored", 0),
                             counts.get("surface_less_materials", 0)))
    if not info.get("meshes"):
        lines.append("  0 meshes")
    msg = "\n".join(lines)
    print(msg)
    raise PortabilityError(msg, info)


def export_scene(out_dir, name, keep_temp=False, timeout_s=1800.0,
                 strip=DEACTIVATE_DEFAULT, collect=False, portable=True,
                 enforce_portable=True, waive_above_instances=None,
                 waive_mirrored=False):
    """Flatten the live stage, collect its dependencies, land both in *out_dir*.

    Returns the `verify` dict with timings added. Raises on a failed flatten,
    a failed collect, or — unless `enforce_portable=False` — a cell that fails
    `verify()`'s `portable_ok` gate: a half-written dataset cell is worse than
    none, and so is one that flattens clean but still points at the build
    machine. `enforce_portable=False` is an ESCAPE HATCH for inspecting a
    known-broken cell without the process aborting; never use it for a real
    dataset build.

    `waive_above_instances` (2026-09-01, run 8, the launcher's
    `FREEZE_WAIVE_VEGETATION` knob): threaded straight to `make_portable` —
    a prototype with more instances than this is left instanced (not
    de-instanced) and its offending paths are WAIVED rather than repaired.
    `None` (default) de-instances every offending prototype unconditionally.

    `waive_mirrored` (2026-09-01, run 9 endgame, the launcher's
    `FREEZE_WAIVE_MIRRORED` knob): threaded straight to `_enforce_portable`
    — for every `build_local` path the de-instancing fixpoint could not
    reach, stat its Nucleus mirror twin and waive it if (and only if) that
    twin verifiably exists. See `_enforce_portable`'s own docstring for
    why this is a second, gate-side waiver rather than a `make_portable`
    change.
    """
    import asyncio

    import omni.kit.app
    import omni.usd

    # `omni.kit.usd.collect` IS NOT ENABLED IN THIS APP. Of the Isaac Sim
    # `.kit` experiences only `isaacsim.exp.action_and_event_data_generation.
    # full` lists it, and a `SimulationApp` does not load that one — so the
    # import below raises `ModuleNotFoundError` four minutes into a build, with
    # the scene already built and the ground truth already written. Enable it
    # first; it is a no-op when it is already on.
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("omni.kit.usd.collect")
    for _ in range(5):
        omni.kit.app.get_app().update()

    from omni.kit.usd.collect import Collector

    os.makedirs(out_dir, exist_ok=True)
    tmp_dir = os.path.join(out_dir, "_freeze_tmp")
    collect_dir = os.path.join(tmp_dir, "collected")
    if os.path.isdir(tmp_dir):
        shutil.rmtree(tmp_dir)
    os.makedirs(collect_dir, exist_ok=True)

    app = omni.kit.app.get_app()
    for _ in range(5):
        app.update()

    # 0) TURN OFF THE BUILD AIDS, before the flatten.
    if strip:
        deactivate_prims(omni.usd.get_context().get_stage(), strip)
        for _ in range(3):
            app.update()

    # 0b) MAKE IT PORTABLE, ALSO BEFORE THE FLATTEN. Three defects that are
    #     invisible on the build machine and fatal everywhere else — see
    #     `make_portable`. This has to happen here: after the flatten the
    #     layer cannot be rewritten by any USD build we have.
    portable_info = None
    if portable:
        portable_info = make_portable(
            omni.usd.get_context().get_stage(), out_dir=out_dir,
            waive_above_instances=waive_above_instances)
        for _ in range(3):
            app.update()
    waived_paths = (portable_info or {}).get("waived_mirror_paths") or []

    # 1) FLATTEN WITH KIT.
    t0 = time.time()
    flat = os.path.join(tmp_dir, "_flat.usd")
    ok, err = asyncio.get_event_loop().run_until_complete(
        omni.usd.get_context().export_as_stage_async(flat))
    if not ok:
        raise RuntimeError("export_as_stage failed: {0}".format(err))
    t_flat = time.time() - t0
    print("[freeze] flattened in {0:.0f}s -> {1:.0f} MB".format(
        t_flat, os.path.getsize(flat) / 1e6))

    # 2) COLLECT WITH KIT — OFF BY DEFAULT, AND THAT IS A DELIBERATE STAGE.
    #
    # The flat layer above already has every POSITION baked in: the archetype
    # references are composed away, the prototypes are retained, and nothing
    # about the scene's geometry depends on the build machine any more. What it
    # still points at is TEXTURES and MDL modules on Nucleus and in the repo.
    # That is enough to run baselines against, which is what this dataset is
    # for today, and the collect is what makes it redistributable — a separate
    # concern with its own cost.
    #
    # AND THE COLLECT CURRENTLY STALLS ON THIS SCENE. Diagnosed: Kit's flatten
    # carries the kit assets' poisoned `assetInfo` into the flat layer, so
    # every core-USD traversal of it raises
    # `Usd_CrateFile::_UnpackValue: unsupported type enum value 0` — 11,405
    # specs carry the field and 11,069 of them refuse `ClearInfo`. The
    # collector burns 150-200% CPU on unpack failures and writes nothing for
    # 20+ minutes. `Sdf.Layer.Traverse` walks all 1.2M specs in 1.8 s, so a
    # targeted `EraseField` pass is the likely way in — untried, and it belongs
    # with the public-release work.
    if not collect:
        out_usd = os.path.join(out_dir, name + ".usd")
        if os.path.exists(out_usd):
            os.remove(out_usd)
        shutil.move(flat, out_usd)
        if not keep_temp:
            shutil.rmtree(tmp_dir, ignore_errors=True)

        # NO POST-FLATTEN REWRITE (2026-09-01, run 7) — see
        # `postflatten_repair`'s own docstring for why it is no longer
        # called here. Portability is now decided ENTIRELY by what got
        # composed onto the stage before this point (AIRSTACK_ASSET_ROOT
        # at launch + `make_portable`'s pre-flatten bake-local collect);
        # `verify()` below is the gate, not a repair step.
        info = verify(out_usd, expect_self_contained=False)
        info.update(flatten_s=round(t_flat, 1), collect_s=None,
                    materials_mb=0.0, self_contained=False)
        info["total_mb"] = info["size_mb"]
        if enforce_portable:
            _enforce_portable(info, waived_paths=waived_paths,
                              waive_mirrored=waive_mirrored)
        return info

    t0 = time.time()
    collector = Collector(flat, collect_dir, flat_collection=True)
    last = [-1]

    def _progress(cur, total):
        pct = int(100.0 * cur / max(1, total))
        if pct >= last[0] + 20:
            last[0] = pct
            print("[freeze] collecting {0}% ({1}/{2})".format(pct, cur, total))

    ok, root = asyncio.get_event_loop().run_until_complete(
        collector.collect(progress_callback=_progress))
    if not ok:
        raise RuntimeError("collect failed")
    t_collect = time.time() - t0
    print("[freeze] collected in {0:.0f}s".format(t_collect))

    # 3) LAND IT IN THE CONTRACT'S SHAPE.
    out_usd = os.path.join(out_dir, name + ".usd")
    mats_bytes = _relocate(collect_dir, out_dir,
                           out_usd, os.path.basename(flat))

    if not keep_temp:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    # NO POST-FLATTEN REWRITE here either (2026-09-01, run 7) — see
    # `postflatten_repair`'s own docstring.

    # 4) VERIFY COLD.
    info = verify(out_usd)
    info.update(flatten_s=round(t_flat, 1), collect_s=round(t_collect, 1),
                materials_mb=round(mats_bytes / 1e6, 1))
    info["total_mb"] = round(info["size_mb"] + info["materials_mb"], 1)
    if enforce_portable:
        _enforce_portable(info, waived_paths=waived_paths,
                          waive_mirrored=waive_mirrored)
    return info


def report(info):
    print("=" * 72)
    print("FROZEN {0}".format("OK" if info.get("ok") else "*** NOT SELF-CONTAINED ***"))
    print("  usd          {0}  ({1} MB)".format(info["path"], info["size_mb"]))
    print("  materials    {0} MB      total {1} MB".format(
        info.get("materials_mb"), info.get("total_mb")))
    print("  contents     {0} prim(s), {1} mesh(es), {2} prototype(s)".format(
        info["prims"], info["meshes"], info["prototypes"]))
    if info.get("bbox_m"):
        b = info["bbox_m"]
        print("  bbox         {0:.0f} x {1:.0f} x {2:.0f} m".format(
            b[3] - b[0], b[4] - b[1], b[5] - b[2]))
    unresolved = info.get("unresolved")
    print("  deps         {0} asset(s), {1} unresolved, {2} outside the "
          "folder{3}  [scan: {4}{5}]".format(
              info["deps"],
              "?" if unresolved is None else len(unresolved),
              len(info["external"]),
              "" if info.get("self_contained")
              else "  (EXPECTED — not collected)",
              info.get("deps_scan_method"),
              " -- {0}".format(info["deps_error"][0])
              if info.get("deps_error") else ""))
    for u in (unresolved or [])[:8]:
        print("    UNRESOLVED  {0}".format(u))
    for e in info["external"][:8]:
        print("    EXTERNAL    {0}".format(e))

    # THE PORTABILITY BLOCK. Each of these three shipped in all 18 cells of the
    # first dataset and each cost a benchmark sweep, because none of them is
    # visible from the build machine — where the files exist and the live stage
    # still has its sky. Print them every time, pass or fail.
    sky = info.get("sky_lights", [])
    print("  lights       {0} total, {1} dome/distant{2}".format(
        len(info.get("lights", [])), len(sky),
        "  *** NO SKY LIGHT — THIS CELL RENDERS BLACK ***" if not sky else ""))
    bl = info.get("build_local", [])
    blb = info.get("build_local_bindings")
    print("  build-local  {0} unique file(s)  ({1} attribute binding(s) "
          "reference one of them){2}".format(
              len(bl), "?" if blb is None else blb,
              "  *** ONLY THIS MACHINE CAN RESOLVE THESE ***" if bl else ""))
    for b in bl[:8]:
        print("    BUILD-LOCAL {0}".format(b))
    da = info.get("dead_ancestral", [])
    dab = info.get("dead_ancestral_bindings")
    if da:
        print("  dead-ancestral  {0} unique file(s)  ({1} attribute "
              "binding(s))  -- WARN, not FAIL: never resolves on ANY "
              "machine (not even the build machine), a breadcrumb baked "
              "in by the asset's original author, nothing to rewrite or "
              "collect".format(len(da), "?" if dab is None else dab))
        for d in da[:8]:
            print("    DEAD-ANCESTRAL {0}".format(d))
    cx = info.get("cross_scope_bindings", [])
    print("  cross-scope  {0}{1}".format(
        len(cx),
        "  *** THESE RENDER UNTEXTURED GREY ***" if cx else ""))
    for c in cx[:8]:
        print("    CROSS-SCOPE {0}".format(c))
    ma = info.get("materials") or {}
    mc = ma.get("counts") or {}
    print("  material-gate {0} visible mesh(es), {1} target(s), {2} "
          "dangling, {3} typeless, {4} unbound/uncoloured, {5} "
          "surface-less{6}".format(
              mc.get("visible_meshes", 0), mc.get("targets", 0),
              mc.get("dangling_targets", 0),
              mc.get("typeless_targets", 0),
              mc.get("unbound_uncolored", 0),
              mc.get("surface_less_materials", 0),
              "  *** WHITE/FALLBACK GEOMETRY ***"
              if ma and not ma.get("ok") else ""))
    if ma and not ma.get("ok"):
        for key in ("dangling_targets", "typeless_targets",
                    "unbound_uncolored", "surface_less_materials"):
            for row in (ma.get("examples", {}).get(key) or [])[:2]:
                print("    MATERIAL-{0} {1}".format(key.upper(), row))
    print("  portable_ok  {0}{1}".format(
        info.get("portable_ok"),
        "" if info.get("portable_ok")
        else "  *** export_scene() RAISES ON THIS -- see build-local/"
             "sky/cross-scope above ***"))

    pf = info.get("postflatten")
    if pf:
        print("  postflatten  {0} rewritten to a verified Nucleus target, "
              "{1} failed verification, {2} collected into the cell, {3} "
              "missing source, {4} already portable/relative -- "
              "{5:.1f}s ModifyAssetPaths + {6:.1f}s Export = {7:.1f}s"
              "{8}".format(
                  pf.get("rewritten"), pf.get("verify_failed"),
                  pf.get("collected"), pf.get("missing"),
                  pf.get("unchanged"), pf.get("modify_s", 0.0),
                  pf.get("export_s", 0.0), pf.get("seconds", 0.0),
                  "" if pf.get("rss_mb") is None
                  else "  (peak RSS {0:.0f} MB)".format(pf["rss_mb"])))

    print("  timing       {0}s flatten + {1}s collect".format(
        info.get("flatten_s"), info.get("collect_s")))
    print("=" * 72)
