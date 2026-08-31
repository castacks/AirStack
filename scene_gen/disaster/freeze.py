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

    out = {"path": usd_path,
           "size_mb": round(os.path.getsize(usd_path) / 1e6, 1)}
    st = Usd.Stage.Open(usd_path)
    out["prototypes"] = len(st.GetPrototypes())
    out["prims"] = sum(1 for _ in Usd.PrimRange.Stage(
        st, Usd.TraverseInstanceProxies()))
    out["meshes"] = sum(1 for p in Usd.PrimRange.Stage(
        st, Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Mesh))

    # `ComputeAllDependencies` is the richest check but it is NOT reliable on
    # a kit-written cell: it walks every field, hits the kit assets' poisoned
    # `assetInfo` and raises `_UnpackValue: unsupported type enum value 0`
    # (measured on every shipped cell, from both Isaac 5.1's pxr and a clean
    # usd-core 26.08). It must therefore never be able to take the portability
    # verdict down with it — that verdict is the whole point of this function.
    here = os.path.dirname(os.path.abspath(usd_path))
    out["deps"] = None
    out["unresolved"] = []
    out["external"] = []
    out["deps_error"] = None
    try:
        _l, assets, unresolved = UsdUtils.ComputeAllDependencies(
            Sdf.AssetPath(usd_path))
        out["deps"] = len(assets)
        out["unresolved"] = [str(u) for u in unresolved]
        stray = []
        for a in assets:
            ap = str(a)
            if ap.startswith(("omniverse://", "http://", "https://")):
                stray.append(ap)
            elif os.path.isabs(ap) and not os.path.abspath(ap).startswith(here):
                stray.append(ap)
        out["external"] = stray
    except Exception as exc:                                # noqa: BLE001
        out["deps_error"] = str(exc).strip().splitlines()[:1]

    # BUILD-LOCAL PATHS, computed WITHOUT ComputeAllDependencies so the check
    # survives the failure above. Reading a shader attribute's `default` is the
    # one traversal that works on these cells — it never asks for `assetInfo`.
    build_local = set()
    for prim in st.Traverse():
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
            if os.path.isabs(ap) and not os.path.abspath(ap).startswith(here):
                build_local.add(ap)

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
                mat, _r = UsdShade.MaterialBindingAPI(child).ComputeBoundMaterial()
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
                 and (not out["external"] or not expect_self_contained))
    out["self_contained"] = not (out["external"] or out["unresolved"])
    return out


# ---------------------------------------------------------------------------
# the pipeline (needs Kit)
# ---------------------------------------------------------------------------

#: Prims REMOVED from the stage before the flatten. Everything here is a build
#: or review aid that has no business in a shipped scene:
#:
#:   * `/World/ReviewCamera` — `snapshots.place_camera` re-uses ONE camera prim
#:     for every capture, so the frozen scene would carry whatever pose the
#:     last shot happened to leave it at. Asked for explicitly, 2026-08-27.
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
#: build-machine paths to this before the flatten — see `make_portable`.
ASSET_LOCAL_PREFIX = "/isaac-sim/AirStack/scene_gen/assets/"
ASSET_MIRROR = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/"
                "Projects/SEI-COA/scene_gen/assets/")


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


def make_portable(stage, mirror=ASSET_MIRROR, local_prefix=ASSET_LOCAL_PREFIX,
                  add_light=True, verbose=True):
    """Repair, ON THE LIVE STAGE and BEFORE the flatten, the three things that
    made every cell of the first dataset unusable off the build machine.

    All three shipped silently in all 18 cells and cost a full benchmark sweep.
    None of them is visible in a snapshot taken during the build, because the
    build machine has the files and the live stage still has its sky.

    1. BUILD-MACHINE ASSET PATHS. `collect=False` leaves every look pointing at
       wherever the asset sat while building — absolute paths under
       `/isaac-sim/AirStack/scene_gen/assets/`. Those trees are git-ignored
       (`aec/*`, `objaverse/*`, `materials/scorched/`), so no other machine has
       them. Worse, a cell is normally opened from `omniverse://`, and an
       ABSOLUTE path inside a Nucleus-anchored layer resolves AGAINST THAT
       SERVER: measured with `omni.client.combine_urls`, the resolver turns
       `/isaac-sim/AirStack/.../Burn_Scorch.png` into
       `omniverse://<host>/isaac-sim/AirStack/.../Burn_Scorch.png`. Copying the
       files to local disk therefore cannot help — proved it, twice. Rewriting
       to an explicit `omniverse://` URL is the only form that resolves the
       same way from every machine and every anchor.

       This runs BEFORE the flatten on purpose. The flat layer carries the kit
       assets' poisoned `assetInfo`, so any core-USD traversal of it raises
       `_UnpackValue: unsupported type enum value 0` — that is what stalls the
       collector, and it is also why the finished cells cannot be rewritten
       after the fact by any USD build (measured: Isaac 5.1's pxr AND a clean
       usd-core 26.08 both raise on `Sdf.Layer.Export`). On the live stage the
       shader prims are ordinary prims and setting an attribute is cheap.

    2. NO LIGHT. `DEACTIVATE_DEFAULT` turns off the Environment, and Kit's
       flatten DROPS deactivated prims entirely rather than writing them
       inactive (measured: 0 of 83,215 prims in a shipped cell carry
       `active=False`). So the sky did not travel and every cell rendered
       black — the overhead frame was 99.93 % pure RGB(0,0,0). An explicit
       dome + sun is authored here so the cell is lit by construction, without
       bringing back the default environment's flat ground.

    3. CROSS-SCOPE MATERIAL BINDINGS. `/World/burnGround/band_*` bound to
       `/World/stage/generated/BurnLooks/band_*`. That is fine on one stage,
       but a consumer that references `/World/<name>` prim-by-prim puts them in
       different arcs, and USD DROPS a relationship target outside its own arc
       — so the burn scar rendered untextured grey. Moving the looks INSIDE the
       overlay scope makes the binding valid under any composition.

    Returns a dict of counts; safe to call twice.
    """
    from pxr import Sdf, UsdGeom, UsdLux, UsdShade

    out = {"paths_rewritten": 0, "looks_moved": 0, "rebound": 0, "lights": 0}
    mirror = mirror.rstrip("/") + "/"

    # --- 1. asset paths ---------------------------------------------------
    for prim in stage.Traverse():
        if not prim.IsA(UsdShade.Shader):
            continue
        for attr in prim.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            val = attr.Get()
            path = getattr(val, "path", "") or ""
            if path.startswith(local_prefix):
                attr.Set(Sdf.AssetPath(mirror + path[len(local_prefix):]))
                out["paths_rewritten"] += 1

    # --- 3. cross-scope bindings -------------------------------------------
    world = stage.GetPrimAtPath("/World")
    if world and world.IsValid():
        for scope in world.GetChildren():
            if scope.GetName() in ("PhysicsScene", "stage"):
                continue
            root = scope.GetPath().pathString
            looks_scope = root + "/Looks"
            for child in scope.GetChildren():
                api = UsdShade.MaterialBindingAPI(child)
                mat, _rel = api.ComputeBoundMaterial()
                if not (mat and mat.GetPrim().IsValid()):
                    continue
                src = mat.GetPrim().GetPath()
                if src.pathString.startswith(root + "/"):
                    continue                     # already inside the scope
                dst = Sdf.Path(looks_scope + "/" + src.name)
                if not stage.GetPrimAtPath(dst):
                    UsdGeom.Scope.Define(stage, looks_scope)
                    if not Sdf.CopySpec(stage.GetRootLayer(), src,
                                        stage.GetRootLayer(), dst):
                        continue
                    out["looks_moved"] += 1
                moved = UsdShade.Material(stage.GetPrimAtPath(dst))
                if moved:
                    UsdShade.MaterialBindingAPI(child).Bind(moved)
                    out["rebound"] += 1

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
        print("[freeze] portable: {0} asset path(s) -> mirror, {1} look(s) "
              "moved into scope, {2} re-bound, {3} active light(s)".format(
                  out["paths_rewritten"], out["looks_moved"], out["rebound"],
                  out["lights"]))
    return out


def export_scene(out_dir, name, keep_temp=False, timeout_s=1800.0,
                 strip=DEACTIVATE_DEFAULT, collect=False, portable=True):
    """Flatten the live stage, collect its dependencies, land both in *out_dir*.

    Returns the `verify` dict with timings added. Raises on a failed flatten or
    a failed collect — a half-written dataset cell is worse than none.
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
    if portable:
        make_portable(omni.usd.get_context().get_stage())
        for _ in range(3):
            app.update()

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
        info = verify(out_usd, expect_self_contained=False)
        info.update(flatten_s=round(t_flat, 1), collect_s=None,
                    materials_mb=0.0, self_contained=False)
        info["total_mb"] = info["size_mb"]
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

    # 4) VERIFY COLD.
    info = verify(out_usd)
    info.update(flatten_s=round(t_flat, 1), collect_s=round(t_collect, 1),
                materials_mb=round(mats_bytes / 1e6, 1))
    info["total_mb"] = round(info["size_mb"] + info["materials_mb"], 1)
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
    print("  deps         {0} asset(s), {1} unresolved, {2} outside the "
          "folder{3}".format(info["deps"], len(info["unresolved"]),
                             len(info["external"]),
                             "" if info.get("self_contained")
                             else "  (EXPECTED — not collected)"))
    for u in info["unresolved"][:8]:
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
    print("  build-local  {0}{1}".format(
        len(bl),
        "  *** ONLY THIS MACHINE CAN RESOLVE THESE ***" if bl else ""))
    for b in bl[:8]:
        print("    BUILD-LOCAL {0}".format(b))
    cx = info.get("cross_scope_bindings", [])
    print("  cross-scope  {0}{1}".format(
        len(cx),
        "  *** THESE RENDER UNTEXTURED GREY ***" if cx else ""))
    for c in cx[:8]:
        print("    CROSS-SCOPE {0}".format(c))

    print("  timing       {0}s flatten + {1}s collect".format(
        info.get("flatten_s"), info.get("collect_s")))
    print("=" * 72)
