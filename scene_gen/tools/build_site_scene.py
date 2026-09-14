#!/usr/bin/env python
"""build_site_scene.py — build a site-plan scene in Isaac Sim ON THE HOST, and capture it.

    OMNI_USER=... OMNI_PASS=... OMNI_KIT_ACCEPT_EULA=YES \
    AirStack/.venv/bin/python scene_gen/tools/build_site_scene.py \
        --config disaster_city --snap scene_gen/_plans/isaac_dc/ \
        --usd /tmp/disaster_city.usd

WHY NOT THE CONTAINER. `simulation/isaac-sim/launch_scripts/` is the normal way
to run a scene, and it cannot be used here: the `isaac-sim` container on this
machine is bind-mounted from a DIFFERENT checkout
(`/home/pranavkumara/Documents/AirStack`), so none of this repo's code exists
inside it, and it authenticates to Nucleus as `guest` so every library asset
comes back ACCESS_DENIED. The repo venv carries a full Isaac Sim pip install,
which has the same renderer and the same MDL support, and takes credentials
from the environment. Nothing shared is touched.

This is also the only way to SEE the ground materials. They are MDL, and the
offline preview (`tools/site_scene_png.py`, Blender/Cycles) renders an OmniPBR
ground as solid black — geometry is all it can check.
"""

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--config", default="disaster_city")
    ap.add_argument("--snap", required=True)
    ap.add_argument("--usd", default="")
    ap.add_argument("--res", type=int, default=1400)
    ap.add_argument("--sky", default="",
                    help="override the asset set's sky (HDR, or a stage USD "
                         "whose sky/light prims are borrowed)")
    args = ap.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": True,
                                       "width": args.res,
                                       "height": int(args.res * 0.62)})
    import omni.kit.app
    import omni.usd
    from pxr import Usd, UsdGeom

    import importlib.util as ilu
    def _load(mod):
        pp = os.path.normpath(os.path.join(
            _HERE, "..", "..", "simulation", "isaac-sim", "utils", mod + ".py"))
        sp2 = ilu.spec_from_file_location(mod, pp)
        m = ilu.module_from_spec(sp2)
        sp2.loader.exec_module(m)
        return m

    snaps = _load("snapshots")
    scene_prep = _load("scene_prep")

    from compile_disaster import load_scene_config
    from scene_generator import resolve_sky
    import suburb_scene

    cfg_path = os.path.join(os.path.dirname(_HERE), "config", "presets",
                            args.config + ".yaml")
    cfg = load_scene_config(cfg_path)

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    # THE SAME SKY THE SUBURBAN SCENES USE, and by the same route. The asset
    # set (`config/asset_sets/shared.yaml`) names
    # `RetroNeighborhood/RetroNeighborhood.stage.usd`, and `scene_prep.add_sky`
    # BORROWS that stage's root sky and light prims rather than lighting the
    # scene here. The first version of this file ignored all of that and bound
    # a Trainyard HDR to a hand-rolled dome — a sky nothing else in the
    # generator uses, which is why this scene did not look like the others.
    # Borrowed stages bring their own sun, so no DistantLight is added.
    sky = args.sky or resolve_sky(cfg)
    scene_prep.add_sky(stage, sky)
    print(f"[build] sky {sky or '(plain dome)'}", flush=True)

    # ---- PUT THE SUN WHERE THE SITE WANTS IT ------------------------------
    # The borrowed environment brings its own sun on a solar-position rig
    # (AxisNorth / AxisLatitude / AxisSHA / AxisDeclination), and it sits at
    # azimuth 242 deg — west-south-west, measured off its composed world
    # transform. Driving the rig's angles means solving for a date and hour;
    # replacing the light is exact and is what `freeze-portable-scenes` asks
    # for anyway ("author the light explicitly").
    #
    # The dome stays: it is the sky and the ambient fill. Only the directional
    # is swapped, at the same intensity so the exposure does not move.
    import math
    from pxr import Gf, UsdLux
    az = float(os.environ.get("SITE_SUN_AZ_DEG", cfg.get("site_sun_az_deg", 135.0)))
    el = float(os.environ.get("SITE_SUN_EL_DEG", cfg.get("site_sun_el_deg", 38.0)))
    keep = 5000.0
    _env_az, _env_dir = None, None
    # EVERY light, and say so. This loop used to run silently, and a silent
    # pass that finds nothing is indistinguishable from one that works: the
    # borrowed environment's own sun is at azimuth 242 (west-south-west), so
    # one missed light and the scene is lit from the wrong side while the line
    # below still prints the azimuth that was asked for. `TraverseInstanceProxies`
    # because `stage.Traverse()` does NOT descend into an instanced prim, and a
    # borrowed environment is exactly the kind of subtree that gets instanced.
    _seen = []
    for p in list(stage.Traverse(
            Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))):
        if not p.HasAPI(UsdLux.LightAPI):
            continue
        _lit = UsdLux.LightAPI(p)
        _int = _lit.GetIntensityAttr().Get() if _lit else None
        _kind = p.GetTypeName()
        _az = ""
        if p.IsA(UsdLux.DistantLight):
            _d = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()).TransformDir(
                    Gf.Vec3d(0, 0, -1)).GetNormalized()
            _az = (f" az={((math.degrees(math.atan2(-_d[0], -_d[1])) + 360) % 360):.0f}"
                   f" el={math.degrees(math.asin(max(-1.0, min(1.0, -_d[2])))):.0f}")
        _seen.append(f"{_kind} {p.GetPath()} intensity={_int}{_az}"
                     f"{' PROXY' if p.IsInstanceProxy() else ''}")
        if p.IsA(UsdLux.DistantLight):
            _env_az = (math.degrees(math.atan2(-_d[0], -_d[1])) + 360) % 360
            _env_dir = Gf.Vec3d(_d)
            if _int:
                keep = float(_int)
            if p.IsInstanceProxy():
                # Cannot edit inside an instance. Un-instance the ancestor that
                # made it a proxy, then the SetActive below is authorable.
                _anc = p
                while _anc and not _anc.IsInstance():
                    _anc = _anc.GetParent()
                if _anc:
                    _anc.SetInstanceable(False)
                    p = stage.GetPrimAtPath(p.GetPath())
            p.SetActive(False)
    print(f"[build] lights before the swap: {len(_seen)}", flush=True)
    for _l in _seen:
        print(f"[build]   {_l}", flush=True)
    # Azimuth is compass bearing: 0 = +Y north, 90 = +X east, so 135 = SE.
    a, e = math.radians(az), math.radians(el)
    sunpos = Gf.Vec3d(math.sin(a) * math.cos(e),
                      math.cos(a) * math.cos(e), math.sin(e)).GetNormalized()
    sun = UsdLux.DistantLight.Define(stage, "/World/SiteSun")
    sun.CreateIntensityAttr(keep)
    sun.CreateAngleAttr(0.53)
    # A DistantLight emits along its own -Z, so rotate -Z onto the direction
    # the light TRAVELS, which is the opposite of where the sun sits.
    rot = Gf.Rotation(Gf.Vec3d(0, 0, -1), -sunpos)
    UsdGeom.Xformable(sun.GetPrim()).AddTransformOp().Set(
        Gf.Matrix4d().SetRotate(rot))
    chk = UsdGeom.Xformable(sun.GetPrim()).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
    caz = (math.degrees(math.atan2(-chk[0], -chk[1])) + 360) % 360
    cel = math.degrees(math.asin(max(-1.0, min(1.0, -chk[2]))))
    print(f"[build] sun -> azimuth {caz:.0f} deg, elevation {cel:.0f} deg "
          f"(asked {az:.0f}/{el:.0f}), intensity {keep:.0f}", flush=True)

    # ---- AND TURN THE SKY TO MATCH ---------------------------------------
    # Switching the light off is only half of it. The borrowed environment is a
    # SOLAR-POSITION RIG — `sky/AxisNorth/AxisLatitude/AxisSHA/AxisDeclination`
    # carries a DistantLight, a visible SunSphere, a 10 km SkySphere painted by
    # an MDL sky (`Azimuth` 99.7, `Elevation` 24.4, `NorthOrientation` 108.4)
    # and the DomeLight that supplies most of the ambient. Deactivating the
    # light leaves all of THAT still pointing west-south-west, so the scene
    # carries a bright sky and a visible sun on one side while its shadows fall
    # for a sun on the other. Reported as "the sunlight is coming in from the
    # south west" — correctly, since the sky is what you actually look at.
    #
    # Rotating the environment's root about Z turns the sky, the sun sphere and
    # the dome together, so every cue agrees. The angle comes from the rig's own
    # MEASURED azimuth rather than from its `NorthOrientation`, because that
    # input is one term of several (latitude, hour angle, declination) and
    # solving the chain is how the first attempt got it wrong. Elevation is left
    # alone: tilting the rig would tip the horizon with it.
    if _env_az is not None:
        envp = stage.GetPrimAtPath("/World/Environment")
        if envp:
            turn = (_env_az - az) % 360.0
            # THE DOME DOES NOT TURN WITH IT. The dome is textured with the same
            # sunset sky, and it is what actually lights the ground: turning it
            # 107 degrees swung the warm quarter of that texture off the site and
            # the sand pads came out grey — measured against the aerial, which
            # shows warm tan. So the visible sky and sun sphere move, the fill
            # stays. Recorded as a WORLD matrix and restored afterwards rather
            # than counter-rotated by angle: the dome carries its own
            # `rotateZYX (270, 0, 108.36)` and composing a second rotation into
            # a ZYX triple is not addition.
            _domes = [(q, UsdGeom.Xformable(q).ComputeLocalToWorldTransform(
                          Usd.TimeCode.Default()))
                      for q in stage.Traverse() if q.IsA(UsdLux.DomeLight)]
            exf = UsdGeom.Xformable(envp)
            exf.ClearXformOpOrder()
            exf.AddRotateZOp().Set(float(turn))
            for q, w0 in _domes:
                pw = UsdGeom.Xformable(
                    q.GetParent()).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default())
                qxf = UsdGeom.Xformable(q)
                qxf.ClearXformOpOrder()
                qxf.AddTransformOp().Set(w0 * pw.GetInverse())
            # Verified on the recorded direction rather than by re-reading the
            # prim: the rig's light is DEACTIVATED by now, and an inactive prim
            # is pruned from the composed stage, so there is nothing left to
            # ask for a transform.
            after = Gf.Matrix4d().SetRotate(
                Gf.Rotation(Gf.Vec3d(0, 0, 1), turn)).TransformDir(
                    _env_dir).GetNormalized()
            aaz = (math.degrees(math.atan2(-after[0], -after[1])) + 360) % 360
            print(f"[build] sky turned {turn:.0f} deg about Z: its rig was at "
                  f"azimuth {_env_az:.0f}, now reads {aaz:.0f} "
                  f"(sun {caz:.0f}); {len(_domes)} dome(s) held in place",
                  flush=True)

    info = {}
    pl = suburb_scene.generate_suburb_on_stage(stage, cfg, info_out=info)

    # ---- REPAIR MDL MODULES BAKED WITH A CONTAINER PATH -------------------
    # Some assets were authored inside the isaac-sim container and carry
    # `info:mdl:sourceAsset = /isaac-sim/kit/mdl/core/Base/OmniPBR.mdl` — an
    # absolute path to a CORE module that exists on every Omniverse install but
    # never at that path here. The module then fails to load and the material
    # falls back to the error shader: MEASURED, the burnt-tree archetypes
    # rendered bright magenta with perfect geometry.
    #
    # A core module is found by NAME on the MDL search path, so dropping the
    # directory is the whole repair. Only unresolvable ABSOLUTE paths are
    # touched; anything that resolves is left exactly as authored.
    from pxr import Sdf as _Sdf
    _fixed = 0
    for _p in stage.Traverse():
        if _p.GetTypeName() != "Shader":
            continue
        _a = _p.GetAttribute("info:mdl:sourceAsset")
        if not _a:
            continue
        _v = _a.Get()
        if not _v or not _v.path:
            continue
        if _v.resolvedPath or not os.path.isabs(_v.path):
            continue
        _a.Set(_Sdf.AssetPath(os.path.basename(_v.path)))
        _fixed += 1
    if _fixed:
        print(f"[build] repaired {_fixed} MDL module path(s) baked to a "
              f"container location", flush=True)

    # ---- AND THE TEXTURES BAKED TO A CONTAINER LOCATION -------------------
    # Same cause, different attribute. The burnt-tree archetypes were baked
    # inside the isaac-sim container, so their soot shaders carry
    # `/isaac-sim/AirStack/scene_gen/assets/materials/scorched/scorch_*.png` —
    # a path that exists nowhere on this machine. The MDL repair above cannot
    # help: that fixes `info:mdl:sourceAsset` by NAME against the MDL search
    # path, and a texture has no search path to be found on.
    #
    # The files themselves are in this repo, at the same tail. So an
    # unresolvable absolute path containing `/scene_gen/` is re-anchored on the
    # repo root and checked on disk before it is written; anything that
    # resolves, or that does not exist here either, is left exactly as it is.
    # Caught by `package_portable --verify`, which reported nine of these as
    # unresolved — but they were equally broken in the live build, where the
    # only symptom is soot that never appears.
    _repo = os.path.dirname(_HERE)
    _tex = 0
    for _p in stage.Traverse(
            Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        for _a in _p.GetAttributes():
            if _a.GetTypeName() != _Sdf.ValueTypeNames.Asset:
                continue
            _v = _a.Get()
            if not _v or not _v.path:
                continue
            # `resolvedPath` IS NOT A TEST OF EXISTENCE. Kit's resolver hands
            # back a resolved string for an absolute path whose file is not
            # there, so `if _v.resolvedPath: continue` skipped every one of
            # these and the pass reported zero fixes while the packager was
            # still calling the same nine unresolved. Ask the filesystem.
            if _v.resolvedPath and os.path.isfile(_v.resolvedPath):
                continue
            if not os.path.isabs(_v.path) or "/scene_gen/" not in _v.path:
                continue
            _cand = os.path.join(
                _repo, _v.path.split("/scene_gen/", 1)[1].lstrip("/"))
            if os.path.isfile(_cand):
                _a.Set(_Sdf.AssetPath(_cand))
                _tex += 1
    if _tex:
        print(f"[build] re-anchored {_tex} texture path(s) baked to a "
              f"container location", flush=True)
    print(f"[build] {len(pl)} placements", flush=True)

    for _ in range(150):
        omni.kit.app.get_app().update()

    os.makedirs(args.snap, exist_ok=True)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    r = bc.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()
    cx, cy = 0.5 * (mn[0] + mx[0]), 0.5 * (mn[1] + mx[1])
    span = max(mx[0] - mn[0], mx[1] - mn[1])
    tall = max(mx[2], 1.0)
    print(f"[build] extent {mx[0]-mn[0]:.0f} x {mx[1]-mn[1]:.0f} m, "
          f"tallest {tall:.1f} m", flush=True)

    snaps.place_camera(stage, (cx, cy, tall + span / 1.1), (cx, cy, 0.0))
    snaps.snapshot(os.path.join(args.snap, "site_top.png"))
    for nm, (ax, ay) in (("e", (1.0, 0.0)), ("ne", (0.75, 0.75)),
                         ("s", (0.0, -1.0))):
        dd = span * 0.8
        snaps.place_camera(stage, (cx + ax * dd, cy + ay * dd,
                                   0.45 * dd + tall * 0.5),
                           (cx, cy, tall * 0.2))
        snaps.snapshot(os.path.join(args.snap, f"site_{nm}.png"))
    # Ground level: the only view where a 9 m asphalt tile and a 4 m rubble
    # tile look different from each other.
    snaps.place_camera(stage, (cx - span * 0.15, cy - span * 0.08, 9.0),
                       (cx + span * 0.3, cy + span * 0.04, 3.0))
    snaps.snapshot(os.path.join(args.snap, "site_ground.png"))
    print(f"[build] snapshots -> {args.snap}", flush=True)

    if args.usd:
        # THE ROOT LAYER, NOT A FLATTENED STAGE. `Usd.Stage.Export` flattens,
        # which pulls every referenced layer's values into one new crate file —
        # and the rubble_hd debris pieces contain a value type this USD build
        # can read but cannot re-pack ("Attempted to pack unsupported type
        # 'void'"), so the flatten aborts after the scene has already rendered.
        # Exporting the root layer keeps the references AS references, which
        # is both what the GUI wants and about three orders of magnitude
        # smaller; it needs Nucleus credentials at open time, which the opener
        # already passes.
        stage.GetRootLayer().Export(args.usd)
        print(f"[build] stage -> {args.usd}", flush=True)
    app.close()


if __name__ == "__main__":
    main()
