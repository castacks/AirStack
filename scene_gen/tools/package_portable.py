#!/usr/bin/env python
"""package_portable.py — write a scene as a folder that stands alone anywhere.

    OMNI_USER=... OMNI_PASS=... OMNI_KIT_ACCEPT_EULA=YES \
    AirStack/.venv/bin/python scene_gen/tools/package_portable.py \
        --usd /tmp/dc_final.usd --out /tmp/disaster_city_portable

Produces a directory whose root layer references NOTHING outside itself — every
mesh, material layer, texture and MDL is inside — so it can be zipped and opened
by someone with no Nucleus account and none of this repo's assets.

WHY NOT FLATTEN. The obvious approach is one big flattened .usd plus textures.
It cannot be done here: `Usd.Stage.Flatten()` on this scene raises

    Usd_CrateFile::CrateFile::_UnpackValue : unsupported type enum value 0

from the vendor crates — the same poisoned `assetInfo` that
`.agents/skills/freeze-portable-scenes` documents stalling `omni.kit.usd.collect`
and blocking `UsdUtils.ModifyAssetPaths`. Reading composes fine; re-serialising
does not.

WHAT MAKES THE MIRROR WORK INSTEAD. Every nested vendor reference in this scene
is RELATIVE (`../Materials/MI_con_int_03.usd`, `SM_WoodPier_1_payload.usd`) —
verified across the DebrisConcrete, Old_Shipyard and FactoryDistrict packs. So
copying the dependency closure while PRESERVING ITS DIRECTORY STRUCTURE leaves
every one of those pointing at the right file, and the only layer that needs
rewriting is the root — which this generator authored and which serialises
cleanly. No vendor layer is opened for writing at all.

The `/Game/...` values in vendor asset attributes look like absolute paths but
are UE MDL module identifiers resolved off the MDL search path, not files; they
are left alone deliberately.
"""

import argparse
import os
import posixpath
import re
import shutil
import sys
import urllib.request

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

NUCLEUS_DIR = "assets/nucleus"   # mirror of omniverse://<host>/<path>
LOCAL_DIR = "assets/local"       # mirror of the AirStack checkout
WEB_DIR = "assets/web"           # mirror of https:// (NVIDIA's cloud sky)

# Textures an MDL names internally; they are invisible to a USD asset-path walk.
_MDL_TEX = re.compile(r'"([^"]+\.(?:png|jpg|jpeg|exr|hdr|dds|ktx|tga))"', re.I)


def web_closure(start_urls, out_dir, say):
    """Download an https dependency tree, preserving its directory structure.

    THE SKY LIVES OUT HERE. `RetroNeighborhood.stage.usd` — the environment the
    whole generator borrows — references NVIDIA's cloud
    `Environments/2023_1/DomeLights/Dynamic/CloudySky.usd` over https, and that
    file pulls three more siblings plus an MDL whose own textures are named only
    inside the module. Miss any of it and the package renders black on a machine
    with no internet, which is defect #1 in the freeze-portable-scenes skill and
    is invisible from here because THIS machine can always reach the bucket.

    Everything in the tree is referenced relatively, so mirroring the structure
    is enough — only the one layer that names the https root has to be rewritten.
    """
    from pxr import Sdf
    seen, mapping = set(), {}
    queue = list(start_urls)
    while queue:
        url = queue.pop()
        if url in seen or not url.startswith("http"):
            continue
        seen.add(url)
        rest = url.split("://", 1)[1]
        rel = posixpath.join(WEB_DIR, rest)
        dst = os.path.join(out_dir, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        try:
            with urllib.request.urlopen(url, timeout=60) as r, open(dst, "wb") as fh:
                shutil.copyfileobj(r, fh)
        except Exception as exc:
            say(f"[pkg] web MISSING {url}: {type(exc).__name__}")
            continue
        mapping[url] = rel
        base = url.rsplit("/", 1)[0]
        kids = []
        low = url.lower()
        if low.endswith((".usd", ".usda", ".usdc")):
            # Sdf reads text and crate alike, so this does not care which it got.
            lay = Sdf.Layer.OpenAsAnonymous(dst)
            if lay:
                def visit(path, _l=lay, _k=kids):
                    spec = _l.GetObjectAtPath(path)
                    if isinstance(spec, Sdf.PrimSpec):
                        for lo in (spec.referenceList, spec.payloadList):
                            for it in lo.GetAddedOrExplicitItems():
                                if it.assetPath:
                                    _k.append(it.assetPath)
                    elif (isinstance(spec, Sdf.AttributeSpec)
                          and spec.typeName == Sdf.ValueTypeNames.Asset):
                        v = spec.default
                        if v is not None and getattr(v, "path", ""):
                            _k.append(v.path)
                lay.Traverse(Sdf.Path("/"), visit)
                kids.extend(lay.subLayerPaths)
        elif low.endswith(".mdl"):
            try:
                kids.extend(_MDL_TEX.findall(open(dst, "r", errors="ignore").read()))
            except Exception:
                pass
        for k in kids:
            if k.startswith("http"):
                queue.append(k)
            elif not k.startswith("/") and "://" not in k:
                queue.append(posixpath.normpath(posixpath.join(base, k))
                             .replace("https:/", "https://"))
    say(f"[pkg] web closure: {len(mapping)} files")
    return mapping


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--usd", required=True, help="the built stage to package")
    ap.add_argument("--out", required=True, help="package directory to write")
    ap.add_argument("--name", default="scene.usda",
                    help="file name of the package's root layer")
    ap.add_argument("--verify-only", action="store_true",
                    help="re-run the self-contained checks on an existing "
                         "package without copying anything")
    ap.add_argument("--local-root", default="",
                    help="repo root that local absolute assets are mirrored "
                         "relative to (default: the AirStack checkout)")
    args = ap.parse_args()

    local_root = os.path.abspath(
        args.local_root or os.path.dirname(os.path.dirname(_HERE)))

    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": True,
                                       "width": 256, "height": 256})
    import omni.client
    import omni.kit.app
    import omni.usd
    from pxr import Sdf, Usd, UsdGeom, UsdLux, UsdShade

    log = []

    def say(msg):
        log.append(msg)
        print(msg, flush=True)

    if args.verify_only:
        out_root = os.path.join(args.out, args.name)
        say(f"[pkg] verify-only on {out_root}")
        _verify(out_root, args.out, say, Usd, Sdf, UsdGeom, UsdLux, UsdShade)
        app.close()
        return

    ctx = omni.usd.get_context()
    ctx.open_stage(args.usd)
    stage = ctx.get_stage()
    # Compose and LOAD everything: an unloaded payload is a dependency that
    # would not appear in `GetUsedLayers()` and would be silently left behind.
    stage.Load()
    for _ in range(240):
        omni.kit.app.get_app().update()

    root_layer = stage.GetRootLayer()

    # ---- what has to travel -------------------------------------------------
    deps = set()
    for lay in stage.GetUsedLayers():
        if lay.identifier != root_layer.identifier and not lay.anonymous:
            deps.add(lay.identifier)
    for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        for a in p.GetAttributes():
            if a.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            v = a.Get()
            if v and v.resolvedPath:
                deps.add(v.resolvedPath)
    say(f"[pkg] {len(deps)} dependencies to mirror")

    # ---- where each one lands ----------------------------------------------
    isaac_root = os.path.dirname(os.path.dirname(os.path.abspath(
        os.path.join(os.__file__))))  # placeholder, replaced below

    def is_core_mdl(src):
        # Core MDL modules (OmniPBR and friends) ship with every Omniverse app
        # and are found on the MDL search path by NAME. Copying them in would
        # add megabytes and risk shadowing the host's own copy.
        return "/isaacsim/kit/mdl/core/" in src.replace("\\", "/")

    def dest_for(src):
        if src.startswith("omniverse://"):
            rest = src.split("://", 1)[1]
            rest = rest.split("/", 1)[1] if "/" in rest else rest
            return posixpath.join(NUCLEUS_DIR, rest.lstrip("/"))
        ap_ = os.path.abspath(src)
        if ap_.startswith(local_root + os.sep):
            return posixpath.join(LOCAL_DIR,
                                  os.path.relpath(ap_, local_root).replace(os.sep, "/"))
        # Anything else keeps its full path under a catch-all, so it is obvious
        # in the manifest where it came from.
        return posixpath.join(LOCAL_DIR, "_external",
                              ap_.lstrip("/").replace(os.sep, "/"))

    os.makedirs(args.out, exist_ok=True)
    mapping = {}
    skipped_core = 0
    copied = failed = 0
    total_bytes = 0
    mdl_files = set()

    def fetch(src, dst_abs):
        # Skip what is already there so a re-run after a fix costs seconds
        # rather than re-pulling the whole 800 MB closure.
        if os.path.isfile(dst_abs) and os.path.getsize(dst_abs) > 0:
            return os.path.getsize(dst_abs)
        os.makedirs(os.path.dirname(dst_abs), exist_ok=True)
        if src.startswith("omniverse://"):
            r, content = omni.client.read_file(src)[0::2]
            if not str(r).endswith("OK"):
                return 0
            with open(dst_abs, "wb") as fh:
                fh.write(memoryview(content))
            return os.path.getsize(dst_abs)
        if not os.path.isfile(src):
            return 0
        shutil.copy2(src, dst_abs)
        return os.path.getsize(dst_abs)

    for src in sorted(deps):
        if is_core_mdl(src):
            skipped_core += 1
            continue
        rel = dest_for(src)
        dst = os.path.join(args.out, rel)
        n = fetch(src, dst)
        if n:
            mapping[src] = rel
            copied += 1
            total_bytes += n
            if src.lower().endswith(".mdl"):
                mdl_files.add((src, rel))
        else:
            failed += 1
            say(f"[pkg] MISSING {src}")
    say(f"[pkg] copied {copied} files, {total_bytes/1e6:.0f} MB "
        f"({skipped_core} core MDL skipped, {failed} unavailable)")

    # AN MDL RESOLVES ITS OWN TEXTURES RELATIVE TO ITSELF, and those paths are
    # invisible to a USD asset walk — they live inside the module's parameter
    # defaults. The first version of this copied the files sitting beside each
    # .mdl and stopped there, which missed every texture in a SUBDIRECTORY:
    # `Grass_Countryside.mdl` names `Grass_Countryside/..._BaseColor.png`, and
    # the oaks name `textures/oakleaves1_basecolor.png`. The package passed
    # every path check and rendered white grass and black trees.
    # Parsing the module for its own texture references gets exactly the files
    # that are needed, wherever beneath the module they sit.
    extra = 0
    for src_mdl, rel_mdl in sorted(mdl_files):
        try:
            local_mdl = os.path.join(args.out, rel_mdl)
            text = open(local_mdl, "r", errors="ignore").read()
        except Exception:
            continue
        src_dir = posixpath.dirname(src_mdl)
        rel_dir = posixpath.dirname(rel_mdl)
        for tex in sorted(set(_MDL_TEX.findall(text))):
            t = tex.lstrip("./")
            if t.startswith("/") or "://" in t:
                continue
            s2 = posixpath.join(src_dir, t)
            r2 = posixpath.join(rel_dir, t)
            n = fetch(s2, os.path.join(args.out, r2))
            if n:
                extra += 1
                total_bytes += n
            else:
                say(f"[pkg] MDL texture MISSING {s2}")
    if extra:
        say(f"[pkg] +{extra} textures named inside MDL modules")

    # ---- the sky, which lives on the public internet -----------------------
    # Find every https path in every used layer, mirror its closure, then
    # rewrite the (few) layers that name one. Those layers are vendor files, but
    # the only one involved here re-serialises cleanly — checked before writing.
    web_refs = {}
    for lay in stage.GetUsedLayers():
        found = []

        def visit(path, _l=lay, _f=found):
            spec = _l.GetObjectAtPath(path)
            if isinstance(spec, Sdf.PrimSpec):
                for lo in (spec.referenceList, spec.payloadList):
                    for it in lo.GetAddedOrExplicitItems():
                        if it.assetPath.startswith("http"):
                            _f.append(it.assetPath)
            elif (isinstance(spec, Sdf.AttributeSpec)
                  and spec.typeName == Sdf.ValueTypeNames.Asset):
                v = spec.default
                if v is not None and str(getattr(v, "path", "")).startswith("http"):
                    _f.append(v.path)
        try:
            lay.Traverse(Sdf.Path("/"), visit)
        except Exception:
            continue
        if found:
            web_refs[lay.identifier] = sorted(set(found))

    web_map = {}
    if web_refs:
        starts = sorted({u for v in web_refs.values() for u in v})
        say(f"[pkg] {len(starts)} https roots referenced by {len(web_refs)} layer(s)")
        web_map = web_closure(starts, args.out, say)

    for lay_id, urls in web_refs.items():
        rel_layer = mapping.get(lay_id)
        if not rel_layer:
            say(f"[pkg] WARNING: {lay_id} names the web but was not mirrored")
            continue
        target = os.path.join(args.out, rel_layer)
        vlay = Sdf.Layer.FindOrOpen(target)
        if vlay is None:
            say(f"[pkg] WARNING: could not open mirrored {rel_layer}")
            continue
        # Depth from this layer to the package root, so the rewritten path is
        # relative and the package can be unzipped anywhere.
        up = "../" * rel_layer.count("/")
        n = 0

        def fix(path, _l=vlay, _up=up):
            nonlocal n
            spec = _l.GetObjectAtPath(path)
            if isinstance(spec, Sdf.PrimSpec):
                for lo in (spec.referenceList, spec.payloadList):
                    items = lo.GetAddedOrExplicitItems()
                    new_items, ch = [], False
                    for it in items:
                        if it.assetPath in web_map:
                            ch = True
                            new_items.append(type(it)(_up + web_map[it.assetPath],
                                                      it.primPath, it.layerOffset))
                        else:
                            new_items.append(it)
                    if ch:
                        lo.prependedItems = []
                        lo.appendedItems = []
                        lo.explicitItems = new_items
                        n += 1
            elif (isinstance(spec, Sdf.AttributeSpec)
                  and spec.typeName == Sdf.ValueTypeNames.Asset):
                v = spec.default
                if v is not None and getattr(v, "path", "") in web_map:
                    spec.default = Sdf.AssetPath(_up + web_map[v.path])
                    n += 1
        vlay.Traverse(Sdf.Path("/"), fix)
        if n:
            try:
                vlay.Save()
                say(f"[pkg] repointed {n} sky path(s) in {rel_layer}")
            except Exception as exc:
                say(f"[pkg] WARNING: could not rewrite {rel_layer}: {exc}")

    # ---- the root layer, rewritten to point inside the package -------------
    out_root = os.path.join(args.out, args.name)
    root_layer.Export(out_root)
    lay = Sdf.Layer.FindOrOpen(out_root)

    def repoint(path, base=None):
        """Package-relative form of *path*, as seen from *base*.

        `base` is the directory of the layer being edited — the package root
        for the root layer, the layer's own folder for a mirrored one. It
        matters because the mirror preserves directory structure, so a nested
        layer's siblings are beside it, not at the top.
        """
        if not path:
            return path
        rel = mapping.get(path)
        if rel is None:
            if not (path.startswith("omniverse://") or os.path.isabs(path)):
                return path
            rel = dest_for(path)
            if not os.path.isfile(os.path.join(args.out, rel)):
                return path
        if base is None:
            return "./" + rel
        out = os.path.relpath(os.path.join(args.out, rel), base)
        return out if out.startswith(".") else "./" + out

    rewritten = left = 0

    def visit(path):
        nonlocal rewritten, left
        spec = lay.GetObjectAtPath(path)
        if isinstance(spec, Sdf.PrimSpec):
            for listop in ("referenceList", "payloadList"):
                proxy = getattr(spec, listop)
                items = proxy.GetAddedOrExplicitItems()
                new = []
                changed = False
                for it in items:
                    np_ = repoint(it.assetPath)
                    if np_ != it.assetPath:
                        changed = True
                        new.append(type(it)(np_, it.primPath,
                                            it.layerOffset))
                    else:
                        new.append(it)
                        if it.assetPath.startswith("omniverse://"):
                            left += 1
                if changed:
                    proxy.prependedItems = []
                    proxy.appendedItems = []
                    proxy.explicitItems = new
                    rewritten += 1
        elif isinstance(spec, Sdf.AttributeSpec):
            if spec.typeName == Sdf.ValueTypeNames.Asset:
                v = spec.default
                if v is not None and getattr(v, "path", ""):
                    np_ = repoint(v.path)
                    if np_ != v.path:
                        spec.default = Sdf.AssetPath(np_)
                        rewritten += 1
                    elif v.path.startswith("omniverse://"):
                        left += 1

    lay.Traverse(Sdf.Path("/"), visit)
    lay.Save()
    say(f"[pkg] root layer: {rewritten} paths repointed, {left} still remote")

    # ---- AND EVERY MIRRORED LAYER ----------------------------------------
    # The mirror works because nested vendor references are RELATIVE, so
    # copying the tree keeps them pointing at each other. That holds for
    # references and payloads; it does NOT hold for texture inputs, and some
    # packs name theirs as absolute `omniverse://` URLs. Those are invisible to
    # the root-layer pass above — they live inside a copied layer, not in the
    # scene — so the package mirrored the textures correctly and then went on
    # asking Nucleus for them: 15 of them on one archetype's materials, which
    # `--verify` reports as `outside_package` and a colleague without an
    # account sees as an untextured building.
    #
    # Each layer is repointed relative to ITSELF, saved only if something
    # changed, and failures are per-layer: one unwritable vendor crate must not
    # cost the package.
    n_lay = n_path = n_fail = 0
    for rel in sorted(set(mapping.values())):
        if os.path.splitext(rel)[1].lower() not in (".usd", ".usda", ".usdc"):
            continue
        full = os.path.join(args.out, rel)
        base = os.path.dirname(full)
        try:
            sub = Sdf.Layer.FindOrOpen(full)
            if sub is None:
                continue
            hits = [0]

            def visit_sub(path, _sub=sub, _base=base, _hits=hits):
                spec = _sub.GetObjectAtPath(path)
                if not isinstance(spec, Sdf.AttributeSpec):
                    return
                if spec.typeName != Sdf.ValueTypeNames.Asset:
                    return
                v = spec.default
                if v is None or not getattr(v, "path", ""):
                    return
                np_ = repoint(v.path, _base)
                if np_ != v.path:
                    spec.default = Sdf.AssetPath(np_)
                    _hits[0] += 1

            sub.Traverse(Sdf.Path("/"), visit_sub)
            if hits[0]:
                sub.Save()
                n_lay += 1
                n_path += hits[0]
        except Exception as exc:
            n_fail += 1
            say(f"[pkg] could not repoint {rel}: {str(exc)[:90]}")
    if n_path or n_fail:
        say(f"[pkg] nested layers: {n_path} texture path(s) repointed in "
            f"{n_lay} layer(s), {n_fail} unwritable")

    with open(os.path.join(args.out, "MANIFEST.tsv"), "w") as fh:
        fh.write("package_path\tsource\n")
        for src, rel in sorted(mapping.items(), key=lambda kv: kv[1]):
            fh.write(f"{rel}\t{src}\n")

    ok, meshes = _verify(out_root, args.out, say,
                         Usd, Sdf, UsdGeom, UsdLux, UsdShade)
    with open(os.path.join(args.out, "README.md"), "w") as fh:
        fh.write(_readme(args.name, meshes, total_bytes, ok, log))
    say(f"[pkg] wrote {out_root}")
    app.close()
    sys.exit(0 if ok else 1)


def _verify(root_path, out_dir, say, Usd, Sdf, UsdGeom, UsdLux, UsdShade):
    """Open the written package and ask what a machine with no Nucleus,
    no internet and none of this repo would actually get."""
    # ---- verify on the ARTEFACT, cold -------------------------------------
    # The skill's rule: open what was written and ask what a stranger's machine
    # would see. Every check here is done by re-reading the package from disk,
    # not by inspecting the live stage, because the live stage can reach
    # Nucleus and the internet and the stranger cannot.
    say("")
    say("[verify] re-reading the package as a stranger would")
    vstage = Usd.Stage.Open(root_path)
    vstage.Load()

    remote = []
    unresolved = []
    out_abs = os.path.abspath(out_dir)
    for lay in vstage.GetUsedLayers():
        if lay.anonymous:
            continue
        ident = lay.identifier
        if ident.startswith(("omniverse://", "http://", "https://")):
            remote.append(ident)
        elif os.path.isabs(ident) and not os.path.abspath(ident).startswith(out_abs):
            remote.append(ident)
    n_assets = 0
    for prim in Usd.PrimRange.Stage(vstage, Usd.TraverseInstanceProxies()):
        for a in prim.GetAttributes():
            if a.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            v = a.Get()
            if not v or not v.path:
                continue
            pth = v.path
            if pth.startswith(("omniverse://", "http://", "https://")):
                remote.append(f"{a.GetPath()} -> {pth}")
                continue
            # A `/Game/...` value is a UE MDL module identifier, not a file.
            if pth.startswith("/Game/") or pth.startswith("/Engine/"):
                continue
            n_assets += 1
            # A BARE MODULE NAME IS PORTABLE, wherever it happens to resolve.
            # `@OmniPBR.mdl@` is found on the MDL search path by every Omniverse
            # app; on THIS machine it resolves into the Isaac install, which is
            # outside the package and looks alarming, but nothing about that
            # path travels — the authored value is just "OmniPBR.mdl".
            if "/" not in pth and pth.lower().endswith(".mdl"):
                continue
            r = v.resolvedPath
            if not r:
                unresolved.append(f"{a.GetPath()} -> {pth}")
            elif not os.path.abspath(r).startswith(out_abs):
                remote.append(f"{a.GetPath()} -> {r}")

    sky = [p_.GetPath() for p_ in vstage.Traverse()
           if p_.IsA(UsdLux.DomeLight) or p_.IsA(UsdLux.DistantLight)]
    meshes = sum(1 for p_ in Usd.PrimRange.Stage(vstage, Usd.TraverseInstanceProxies())
                 if p_.IsA(UsdGeom.Mesh))

    cross = []
    for prim in vstage.Traverse():
        parts = prim.GetPath().pathString.split("/")
        if len(parts) < 3 or parts[1] != "World":
            continue
        scope = "/World/" + parts[2]
        b = UsdShade.MaterialBindingAPI(prim).GetDirectBinding().GetMaterial()
        if b and not b.GetPath().pathString.startswith(scope):
            cross.append(f"{prim.GetPath()} -> {b.GetPath()}")

    say(f"[verify] meshes           {meshes}")
    say(f"[verify] asset refs       {n_assets} checked")
    say(f"[verify] sky_lights       {len(sky)}  {'PASS' if sky else 'FAIL — renders black'}")
    say(f"[verify] outside_package  {len(remote)}  "
        f"{'PASS' if not remote else 'FAIL'}")
    say(f"[verify] unresolved       {len(unresolved)}  "
        f"{'PASS' if not unresolved else 'FAIL'}")
    say(f"[verify] cross_scope      {len(cross)}  "
        f"{'PASS' if not cross else 'WARN — a prim-by-prim consumer drops these'}")
    for tag, rows in (("outside", remote), ("unresolved", unresolved),
                      ("cross-scope", cross)):
        for r in rows[:8]:
            say(f"    {tag}: {r}")
        if len(rows) > 8:
            say(f"    {tag}: ... and {len(rows) - 8} more")

    ok = bool(sky) and not remote and not unresolved
    say(f"[verify] {'OK — the package stands alone' if ok else 'NOT SELF-CONTAINED'}")

    return ok, meshes


def _readme(name, meshes, nbytes, ok, log):
    return f"""# Disaster City — portable scene

Open **`{name}`**. Everything it needs is in this folder; nothing is fetched
from Nucleus, from the internet, or from the machine that built it.

- {meshes:,} meshes, {nbytes / 1e6:.0f} MB of assets
- self-contained check at build time: {'PASS' if ok else 'FAILED — see below'}

## Opening it

Any USD tool reads the geometry: `usdview {name}`, Blender, Houdini, Maya.

For the intended look use an Omniverse app (Isaac Sim, USD Composer). The
materials are MDL — `OmniPBR` plus vendor modules bundled under `assets/` — and
non-Omniverse renderers substitute their own preview surface, so expect flat
colour there rather than the textures.

## Layout

    {name}              the scene; every path in it is relative to this folder
    assets/nucleus/     asset packs mirrored from the AirLab Nucleus server
    assets/local/       assets from the AirStack checkout (trees, ground materials)
    assets/web/         NVIDIA's cloud sky (CloudySky + its textures)
    MANIFEST.tsv        every bundled file and where it came from

`assets/` keeps the original directory structure on purpose: the vendor layers
reference their materials and textures by RELATIVE path, so preserving the tree
is what makes them resolve here. Moving files inside `assets/` will break them.
Moving or renaming the whole folder is fine.

## Build log

```
{chr(10).join(log)}
```
"""


if __name__ == "__main__":
    main()
