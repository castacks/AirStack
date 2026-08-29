#!/usr/bin/env python
"""
_o_arch_bench_launch_script.py — what one archetype COSTS, and what it LOOKS
like, merged vs unmerged. Agent O, round 3.

    LAUNCHER=_o_arch_bench_launch_script.py DONE_RE='ARCH LOAD BENCH DONE' \
    scene_gen/tools/eq_bench.sh O_load ARCH_NAMES=bld_commercial_DG5,bld_office_DG5 \
        ARCH_COPIES=20

For every name in `ARCH_NAMES` it builds a throwaway stage TWICE — once from
`ARCH_RAW_DIR` (the unmerged twin `BAKE_MERGE=both` writes) and once from
`ARCH_DIR` (merged) — references `ARCH_COPIES` copies onto a grid, and
measures the three numbers that decide whether a city loads in minutes or in
hours:

    compose_s      Usd reference + Load of N copies (no Hydra)
    firstframe_s   the first `app.update()` after that: Hydra sync + BLAS build
    fps            over `ARCH_FRAMES` further updates, at the viewport's own
                   resolution (printed, so the number is comparable)

and captures the SAME two cameras of copy 0 in both variants, so "the merge
did not change the look" is something to look at rather than to assert.

Env:
    ARCH_DIR      merged archetypes  (default omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/archetype)
    ARCH_RAW_DIR  unmerged twins     (default <ARCH_DIR>/_raw; "off" to skip)
    ARCH_NAMES    comma list of `bld_<style>_<level>` (no .usd)
    ARCH_COPIES   copies per stage   (default 20)
    ARCH_FRAMES   frames to average the frame time over (default 120)
    SNAP_DIR      captures
"""

import json
import math
import os
import sys
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={
    "headless": os.environ.get("ISAAC_SIM_HEADLESS", "false").strip().lower()
    in ("1", "true", "yes")})

from isaacsim.core.utils.extensions import enable_extension     # noqa: E402

enable_extension("omni.kit.window.script_editor")

import omni.kit.app                                             # noqa: E402
import omni.timeline                                            # noqa: E402
import omni.usd                                                 # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux                    # noqa: E402

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

ARCH_DIR = os.environ.get("ARCH_DIR") or os.path.join(
    _SCENE_GEN_DIR, "assets", "archetype")
ARCH_RAW_DIR = os.environ.get("ARCH_RAW_DIR", "").strip() or os.path.join(
    ARCH_DIR, "_raw")
NAMES = [q.strip() for q in os.environ.get("ARCH_NAMES", "").split(",") if q.strip()]
COPIES = int(os.environ.get("ARCH_COPIES") or "20")
FRAMES = int(os.environ.get("ARCH_FRAMES") or "120")
SNAP_DIR = os.environ.get("SNAP_DIR", "").strip()


def _snaps():
    import importlib.util as ilu
    sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
    spec = ilu.spec_from_file_location("snapshots", sp)
    mod = ilu.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _light_and_ground(stage):
    e = 900.0
    plane = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    plane.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, -e, 0),
                            Gf.Vec3f(e, e, 0), Gf.Vec3f(-e, e, 0)])
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    plane.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    plane.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.28, 0.25)])
    UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome")).CreateIntensityAttr(900.0)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(2200.0)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 30.0))


def _viewport_res():
    try:
        import omni.kit.viewport.utility as vp
        v = vp.get_active_viewport()
        return "{0}x{1}".format(*v.resolution)
    except Exception:
        return "?"


def run_one(usd, copies, pitch, label, name, capture):
    """Build a stage of `copies` references and time it. Returns a dict."""
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    _light_and_ground(stage)
    for _ in range(5):
        app.update()

    cols = max(1, int(round(math.sqrt(copies))))
    t0 = time.time()
    for i in range(copies):
        cx = (i % cols - (cols - 1) / 2.0) * pitch
        cy = (i // cols - (copies / cols - 1) / 2.0) * pitch
        h = UsdGeom.Xform.Define(stage, Sdf.Path("/World/b{0}".format(i))).GetPrim()
        h.GetReferences().AddReference(usd)
        h.Load()
        UsdGeom.Xformable(h).AddTranslateOp().Set(Gf.Vec3d(cx, cy, 0.0))
    n_prims = sum(1 for _ in stage.Traverse())
    compose_s = time.time() - t0

    t1 = time.time()
    app.update()
    first_s = time.time() - t1
    for _ in range(20):                       # let the renderer converge
        app.update()
    t2 = time.time()
    for _ in range(FRAMES):
        app.update()
    dt = time.time() - t2
    fps = FRAMES / dt if dt > 0 else 0.0

    out = {"file": os.path.basename(usd), "variant": label,
           "mb": round(os.path.getsize(usd) / 1e6, 2) if os.path.exists(usd) else None,
           "copies": copies, "prims": n_prims,
           "compose_s": round(compose_s, 2), "firstframe_s": round(first_s, 2),
           "fps": round(fps, 1), "frame_ms": round(1000.0 * dt / FRAMES, 1),
           "res": _viewport_res()}
    print("[oarch] {0:<30} {1:<8} {2:6.2f} MB  {3:7d} prims x{4}  "
          "compose {5:6.2f}s  first frame {6:6.2f}s  {7:5.1f} fps ({8})".format(
              out["file"], label, out["mb"] or 0.0, n_prims, copies,
              compose_s, first_s, fps, out["res"]), flush=True)

    if capture and SNAP_DIR:
        try:
            sn = _snaps()
            os.makedirs(SNAP_DIR, exist_ok=True)
            # copy 0's own frame: the grid is centred, so its cell centre is
            # deterministic and IDENTICAL between the two variants
            cx = (0 % cols - (cols - 1) / 2.0) * pitch
            cy = (0 // cols - (copies / cols - 1) / 2.0) * pitch
            d = pitch * 0.42
            sn.place_camera(stage, (cx - d, cy - d, pitch * 0.30), (cx, cy, 3.0))
            sn.snapshot(os.path.join(SNAP_DIR, "{0}_{1}_obl.png".format(name, label)))
            sn.place_camera(stage, (cx - pitch * 0.16, cy - pitch * 0.16, 6.0),
                            (cx, cy, 2.5), focal_mm=28.0)
            sn.snapshot(os.path.join(SNAP_DIR, "{0}_{1}_close.png".format(name, label)))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[oarch] capture FAILED: {0}".format(exc))
    return out


def main():
    omni.timeline.get_timeline_interface().stop()
    if not NAMES:
        print("[oarch] ARCH_NAMES is empty — nothing to do")
        print("ARCH LOAD BENCH DONE")
        simulation_app.close()
        return
    rows = []
    # WARM-UP, DISCARDED. The first stage in the process pays Kit's one-time
    # RTX shader compilation, and `raw` is always measured first — without
    # this the merge would look better than it is by a whole warm-up.
    try:
        warm = os.path.join(ARCH_DIR, NAMES[0] + ".usd")
        if os.path.exists(warm):
            run_one(warm, 2, 60.0, "warmup", "warmup", False)
    except Exception as exc:
        print("[oarch] warm-up: {0}".format(exc))
    for name in NAMES:
        merged = os.path.join(ARCH_DIR, name + ".usd")
        raw = None if ARCH_RAW_DIR.lower() in ("off", "none") else \
            os.path.join(ARCH_RAW_DIR, name + ".usd")
        # pitch from the merged file's own world bound, so both variants are
        # laid out and framed identically
        pitch = 60.0
        try:
            st = Usd.Stage.Open(merged)
            bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
            r = bc.ComputeWorldBound(st.GetPrimAtPath("/Baked")).ComputeAlignedRange()
            if not r.IsEmpty():
                lo, hi = r.GetMin(), r.GetMax()
                pitch = max(40.0, 1.35 * max(hi[0] - lo[0], hi[1] - lo[1]))
        except Exception as exc:
            print("[oarch] bound of {0}: {1}".format(name, exc))
        if raw and os.path.exists(raw):
            rows.append(run_one(raw, COPIES, pitch, "raw", name, True))
        if os.path.exists(merged):
            rows.append(run_one(merged, COPIES, pitch, "merged", name, True))
        else:
            print("[oarch] MISSING {0}".format(merged))

    print("\n" + "=" * 96)
    print("ARCHETYPE LOAD BENCH  ({0} copies each, {1} frames averaged)".format(
        COPIES, FRAMES))
    print("{0:<32} {1:<8} {2:>8} {3:>9} {4:>10} {5:>12} {6:>7}".format(
        "file", "variant", "MB", "prims", "compose s", "1st frame s", "fps"))
    for r in rows:
        print("{0:<32} {1:<8} {2:>8.2f} {3:>9d} {4:>10.2f} {5:>12.2f} {6:>7.1f}"
              .format(r["file"], r["variant"], r["mb"] or 0.0, r["prims"],
                      r["compose_s"], r["firstframe_s"], r["fps"]))
    if SNAP_DIR:
        try:
            with open(os.path.join(SNAP_DIR, "load_bench.json"), "w") as fh:
                json.dump(rows, fh, indent=1)
        except Exception as exc:
            print("[oarch] json: {0}".format(exc))
    print("ARCH LOAD BENCH DONE")
    print("=" * 96 + "\n")

    app = omni.kit.app.get_app()
    if (os.environ.get("KEEP_OPEN", "").strip() == "1"
            or os.environ.get("ISAAC_SIM_HEADLESS", "false").strip().lower()
            not in ("1", "true", "yes")):
        while simulation_app.is_running():
            app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
