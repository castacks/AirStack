#!/usr/bin/env python
"""
measure_fences.py — the REAL extents of every `lot_fences` module, from USD.

WHY IT EXISTS. `tools/fence_png.py` draws each module as its TRUE oriented
rectangle, and every judgement it makes — crossing, doubling, a gap between two
neighbours' runs — is a claim about two rectangles a few centimetres apart. Get
the rectangle wrong and the picture is a fiction that happens to look like a
plan. The asset-set comments are hand-written and drift; this reads the files.

NO SIMULATIONAPP, and that is the point of this rewrite. The previous version
booted a headless Kit app purely to get `pxr` and the `omniverse://` resolver on
the path. That is a GPU process on a machine that is usually already running a
sim, and it produced NOTHING a plain USD process cannot: the fence modules are
static meshes, so a bbox needs the resolver and the Nucleus credentials, not a
renderer. `tools/measure_fences.sh` wires `omni.usd.libs` (pxr),
`omni.usd_resolver` (the `omniverse://` AR plugin) and `omni.client.lib`
(libomniclient, which the resolver dlopens) onto a bare `python.sh` instead.
Runs in ~5 s against a live Nucleus, next to a running Isaac Sim, touching
neither.

FOUR THINGS ARE RECORDED per module, because they are four different claims and
the fence pass depends on all of them agreeing:

  raw        the composed stage's world bbox in stage units, with
             metersPerUnit and upAxis — what the file actually contains
  footprint  `scene_generator._measure_footprint` at the entry's own scale and
             axis-up — the SAME function the build calls, so this is what
             `suburb_scene` reads, not a re-derivation of it
  module     `suburb_scene._fence_module`'s (length, thickness, height,
             yaw_fix) — what `_fence_run` tiles with
  meshes     every Mesh prim's own bbox, so a module that is really a panel
             plus a post plus air shows up as more than one box

`pivot_along_m` is called out separately because it is the one number that can
open a seam without being visible in any of the four: it is how far the bbox
centre sits from the prim origin ALONG the run. `apply_placements` re-centres on
the bbox unless `raw_pivot` is set, so a non-zero value here is compensated —
but if a caller ever anchors a fence by its origin, that offset is the gap.

Output: JSON at OUT (default scene_gen/_plans/fence_modules_measured.json),
read by tools/fence_png.py. Keyed by BOTH the entry's `usd` string and the
resolved URL's basename, because those differ for `objaverse://` entries and
the consumer looks modules up by basename.

    scene_gen/tools/measure_fences.sh          # from the host, via docker exec
"""
import json
import os
import sys
import time
import traceback

T0 = time.time()
HERE = os.path.dirname(os.path.abspath(__file__))
SCENE_GEN = os.path.dirname(HERE)
OUT = os.environ.get("FENCE_MEASURE_OUT",
                     os.path.join(SCENE_GEN, "_plans", "fence_modules_measured.json"))
LOG = OUT + ".log"

sys.path.insert(0, SCENE_GEN)


def log(msg):
    line = "[measure_fences %5.1fs] %s" % (time.time() - T0, msg)
    print(line, flush=True)
    with open(LOG, "a") as fh:
        fh.write(line + "\n")


os.makedirs(os.path.dirname(OUT), exist_ok=True)
open(LOG, "w").close()

from pxr import Usd, UsdGeom                               # noqa: E402
import yaml                                                # noqa: E402

result = {"generated": time.strftime("%Y-%m-%d %H:%M:%S"),
          "tool": "measure_fences.py (plain pxr, no SimulationApp)",
          "modules": {}}


def bbox_of(prim, cache):
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    lo, hi = r.GetMin(), r.GetMax()
    return {"min": [float(lo[i]) for i in range(3)],
            "max": [float(hi[i]) for i in range(3)],
            "size": [float(hi[i] - lo[i]) for i in range(3)]}


def raw_measure(url):
    """The composed stage's own bbox, in its own units — no scale, no axis fix.

    Deliberately separate from `footprint` below: this is the file, that is the
    build's reading of the file, and a disagreement between them is the bug
    class this tool exists to catch.
    """
    stage = Usd.Stage.Open(url)
    if stage is None:
        return {"error": "Stage.Open returned None"}
    mpu = float(UsdGeom.GetStageMetersPerUnit(stage) or 1.0)
    up = str(UsdGeom.GetStageUpAxis(stage))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                              useExtentsHint=False)
    whole = bbox_of(stage.GetPseudoRoot(), cache)
    meshes, n_pts = [], 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Mesh":
            continue
        pts = prim.GetAttribute("points").Get()
        n = len(pts) if pts is not None else 0
        n_pts += n
        meshes.append({"path": str(prim.GetPath()), "points": n,
                       "bbox": bbox_of(prim, cache)})
    out = {"metersPerUnit": mpu, "upAxis": up, "bbox_stage_units": whole,
           "n_meshes": len(meshes), "points": n_pts, "meshes": meshes}
    if whole:
        out["size_m"] = [s * mpu for s in whole["size"]]
        out["base_m"] = whole["min"][1 if up == "Y" else 2] * mpu
    return out


def main():
    # THE RESOLVED CONFIG, exactly as the scene build sees it: `asset_root`
    # lives in shared.yaml and `resolve_asset_set` is what joins it onto the
    # relative Nucleus paths and expands objaverse:// to the local cache.
    import scene_generator as sg
    import suburb_scene as ss
    from compile_disaster import resolve_config_path, compile_spec, DEFAULT_BASE
    path = resolve_config_path(os.environ.get("FENCE_MEASURE_CONFIG",
                                              "suburb_net"))
    full = compile_spec(yaml.safe_load(open(path)),
                        yaml.safe_load(open(DEFAULT_BASE)))
    full = sg.resolve_asset_set(full, path)

    raw = ss._raw_pool(full, "lot_fences")
    pools = ss.AssetPools(full)
    urls = []
    for tag in ("low", "privacy"):
        for u in pools.load_tagged(raw, tag):
            if u not in urls:
                urls.append(u)
    log("%d lot_fences modules to measure" % len(urls))

    # THE BUILD'S OWN RESOLVER, measuring for real. `measure_usds` has to be on
    # — with it off `SizeResolver` hands back `fallback_sizes["fence"]`, which
    # is 4 x 4 x 3 m for every entry, and the previous run of this tool wrote
    # exactly that into the JSON and called it "measured". fence_png then drew
    # a 4 m-thick park railing.
    full["measure_usds"] = True
    resolver = sg._make_resolver(full)
    assert resolver.measure, "SizeResolver is not measuring — check measure_usds"

    for u in urls:
        sc, au = pools.scale_of(u), pools.axis_of(u)
        rec = {"url": u, "scale": sc, "axis_up": au,
               "yaw_offset_deg": pools.yaw_of(u)}
        log("measuring %s (scale %s, up %s)" % (os.path.basename(u), sc, au))
        try:
            rec["raw"] = raw_measure(u)
            log("  file: %s m, mpu %s, up %s, %d mesh(es), %d pts" % (
                ["%.3f" % v for v in (rec["raw"].get("size_m") or [])],
                rec["raw"].get("metersPerUnit"), rec["raw"].get("upAxis"),
                rec["raw"].get("n_meshes", 0), rec["raw"].get("points", 0)))
        except Exception:
            rec["raw"] = {"error": traceback.format_exc()}
            log("  raw FAILED: " + rec["raw"]["error"].splitlines()[-1])
        try:
            fp = resolver.get(u, "fence", scale=sc, axis_up=au)
            rec["footprint"] = {k: float(v) for k, v in dict(fp).items()}
            ln, th, h, fix = ss._fence_module(resolver, pools, u)
            rec["module"] = {"length_m": ln, "thickness_m": th, "height_m": h,
                             "yaw_fix_deg": fix,
                             # How far the bbox centre is from the prim origin
                             # ALONG the run. See the module docstring.
                             "pivot_along_m": (float(fp["cy"]) if fix == 90.0
                                               else float(fp["cx"]))}
            log("  pass reads: length %.3f  thick %.3f  height %.3f  turn %+.0f"
                % (ln, th, h, fix))
        except Exception:
            rec["footprint"] = {"error": traceback.format_exc()}
            log("  footprint FAILED: "
                + rec["footprint"]["error"].splitlines()[-1])
        # BOTH KEYS. `fence_png` looks a module up by the basename of the
        # RESOLVED path (`<uid>.usdc` for an objaverse entry), while a human
        # reading this file looks for the string in the asset set. Writing only
        # one of them is how the last version silently applied to the Nucleus
        # railing and to neither objaverse panel.
        result["modules"][u] = rec
        result["modules"][os.path.basename(u)] = rec

    with open(OUT, "w") as fh:
        json.dump(result, fh, indent=1)
    log("wrote %s" % OUT)


try:
    main()
except Exception:
    log("FAILED:\n" + traceback.format_exc())
    sys.exit(1)
