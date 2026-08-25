#!/usr/bin/env python
"""
Look at Stage C: the victim poses on their own, and victims in a real scene.

`scene_gen/targets.py` decides WHO is in a scene and where, and its host-side
tests pin the distribution — but a pose is joint angles, and joint angles are
only right or wrong in a picture. This bench is that picture. Two modes, both
writing PNGs through `utils/snapshots.py`:

    POSES=all        a row of one character per pose, on bare ground, one
                     oblique each plus a row shot. The fast mode: no city, no
                     damage, ~1 minute after the app is up.
    SCENE=<preset>   generate the preset, run Stage C on it, then photograph
                     the scene from above and each cohort close up.

USAGE (see .agents/skills/run-isaac-sim-launcher)

    ISAAC_SIM_SCRIPT_NAME=targets_showcase_launch_script.py ./airstack.sh up isaac-sim

    # or, against a container that is already up:
    docker exec isaac-sim tmux send-keys -t isaac 'clear; \\
      POSES=all SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/targets \\
      PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh \\
      /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/targets_showcase_launch_script.py \\
      --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts' ENTER

`SNAP_DIR` MUST sit under `/isaac-sim/.nvidia-omniverse/logs/` — that is the
only directory the host can read (`~/docker/isaac-sim/logs/`). Empty means no
captures and an otherwise identical run.

    POSES        `all`, a comma list of pose names, or empty to skip the row
    SCENE        a scene config name (e.g. urban_quake_tiny); empty to skip
    ASSET_CONFIG which config supplies the human assets for the pose row
    TARGET_SEED  re-roll the victims in SCENE mode
    TARGET_SEEDS comma list of targets.seed values, run one after another on
                 the SAME built city — the G3 three-seed run in one launch
    OCCUPANCY    night | day | commute, for SCENE mode
    COUNT_PER_KM2 override the population density, for SCENE mode
    MARKERS      0 to drop the coloured posts over each victim (SCENE mode)
    PROBE_INTERIORS  N candidate spots just inside a standing open-shell
                 building to test against the real geometry (SCENE mode, 0 off)
"""

import os
import sys

import carb
from isaacsim import SimulationApp

# Must be created before any omni imports.
simulation_app = SimulationApp(launch_config={"headless": False})

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

from scene_prep import add_colliders, add_sky, get_stage_meters_per_unit, ensure_scene_queries
from scene_generator import resolve_sky
from compile_disaster import load_scene_config
from disaster import kinds

import scene_generator as sg
import targets as T

# ----- CONFIGURATION -----
POSES = os.environ.get("POSES", "all")
SCENE = os.environ.get("SCENE", "")
ASSET_CONFIG = os.environ.get("ASSET_CONFIG", "urban")
SNAP_DIR = os.environ.get("SNAP_DIR", "")
TARGET_SEED = os.environ.get("TARGET_SEED", "")
#: A comma list of `targets.seed` values to run Stage C at, one after another,
#: ON THE SAME BUILT SCENE. G3 wants three seeds and the city is identical
#: across them by construction, so rebuilding it three times would pay Kit
#: startup and a full generate twice for nothing.
TARGET_SEEDS = [s.strip() for s in
                os.environ.get("TARGET_SEEDS", "").split(",") if s.strip()]
OCCUPANCY = os.environ.get("OCCUPANCY", "")
COUNT_PER_KM2 = os.environ.get("COUNT_PER_KM2", "")
MARKERS = os.environ.get("MARKERS", "1").lower() not in ("0", "false", "")
#: Probe the building-interior stratum: sample candidate spots just inside a
#: standing open-shell building and ask the REAL geometry whether a sightline
#: gets out. See `targets.interior_candidates` for why only the stage can
#: answer this. Off by default — it places nobody, it only measures.
PROBE_INTERIORS = int(os.environ.get("PROBE_INTERIORS", "0") or 0)
# -------------------------

#: Metres between characters in the pose row.
ROW_STEP = 3.0


def _snaps():
    """`utils/snapshots.py`, imported by path at the tail — see its docstring."""
    import importlib.util as ilu

    path = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
    spec = ilu.spec_from_file_location("snapshots", path)
    mod = ilu.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _ground(stage, half_x: float, half_y: float, ssf: float = 1.0):
    """A plain quad to stand on, so a pose is read against a surface."""
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    hx, hy = half_x * ssf, half_y * ssf
    mesh.CreatePointsAttr([Gf.Vec3f(-hx, -hy, 0.0), Gf.Vec3f(hx, -hy, 0.0),
                           Gf.Vec3f(hx, hy, 0.0), Gf.Vec3f(-hx, hy, 0.0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.32, 0.33, 0.30)])
    return mesh


def _pose_row(stage, config, ssf: float):
    """One character per pose, in a row, posed exactly as Stage C would pose it.

    The victims are hand-written rather than sampled so every pose appears
    once, but they go through `targets.to_placements` and `apply_placements`
    unchanged — the point is to photograph the REAL path, not a
    reimplementation of it that could be right while the path is wrong.
    """
    names = ([p for p in sg._HUMAN_POSES] if POSES.strip().lower() == "all"
             else [p.strip() for p in POSES.split(",") if p.strip()])
    unknown = [p for p in names if p not in sg._HUMAN_POSES]
    if unknown:
        print(f"[targets_bench] unknown pose(s) ignored: {unknown}")
        names = [p for p in names if p in sg._HUMAN_POSES]
    if not names:
        return {}, []

    victims = []
    for i, pose in enumerate(names):
        victims.append({
            "id": i, "cohort": "rubble_edge", "visibility": "open",
            "pose": pose, "lying": pose in T.LYING,
            # Facing +Y, i.e. toward the camera's south-west oblique, so a
            # front-on pose reads as a front-on pose.
            "x": i * ROW_STEP, "y": 0.0, "yaw_deg": 90.0, "damage": 0.0,
        })
    # …and one more of the same pose sunk into rubble, because `bury_frac` is
    # applied by the placer and is exactly the kind of thing that looks fine in
    # a unit test and puts a body underground in the viewport.
    victims.append({
        "id": len(victims), "cohort": "inside_rubble", "visibility": "occluded",
        "pose": "fetal", "lying": True, "bury_frac": 0.5,
        "x": len(names) * ROW_STEP, "y": 0.0, "yaw_deg": 90.0, "damage": 1.0,
    })
    names = names + ["fetal_buried"]

    resolver = sg._make_resolver(config)
    rng = T.rng_for(config)
    pls = T.to_placements(victims, config, resolver, rng)
    sg.apply_placements(stage, pls, parent_path="/World/poses",
                        scene_scale_factor=ssf, resolver=resolver)
    # The same settle Stage C runs, so the row shows what a scene shows.
    T.settle_on_surface(stage, victims, pls, ssf)
    print(f"[targets_bench] pose row: {len(pls)} character(s) — "
          + ", ".join(names))
    _report_contact(stage, names, victims, pls, ssf)
    return {n: (v["x"], v["y"]) for n, v in zip(names, victims)}, victims


def _report_contact(stage, names, victims, pls, ssf: float):
    """Print each posed character's world bbox against the ground plane.

    The eye cannot tell a body resting on the ground from one hovering 30 cm
    over it at an oblique angle — a floating `crouch` was read as fine in one
    capture and as broken in the next. `dz` is the gap (or, negative, the sink)
    in metres, and it is the number that says whether a pose is placed right.
    """
    from pxr import Usd, UsdGeom

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    print("[targets_bench] ground contact (dz>0 floats, dz<0 sinks):")
    for name, v, p in zip(names, victims, pls):
        prim = stage.GetPrimAtPath(p.get("prim_path", ""))
        if not prim or not prim.IsValid():
            continue
        rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng.IsEmpty():
            print(f"[targets_bench]   {name:<14} (no bounds)")
            continue
        lo, hi = rng.GetMin(), rng.GetMax()
        dz = float(lo[2]) / (ssf or 1.0)
        print(f"[targets_bench]   {name:<14} dz={dz:+.3f} m  "
              f"height={(float(hi[2]) - float(lo[2])) / (ssf or 1.0):.2f} m  "
              f"lying={v['lying']}")


def _scene(stage, ssf: float):
    """Generate SCENE, run Stage C on it, and hand back the victims."""
    config = load_scene_config(SCENE)
    if TARGET_SEED:
        config.setdefault("targets", {})["seed"] = int(TARGET_SEED)
    if OCCUPANCY:
        config.setdefault("targets", {})["occupancy"] = OCCUPANCY
    if COUNT_PER_KM2:
        # A 200 m preset holds 4 people at the shipped density, which is a
        # thin thing to photograph. Raising it here does not change what the
        # preset compiles to.
        config.setdefault("targets", {})["count_per_km2"] = float(COUNT_PER_KM2)

    from generate_scene import generate_scene_on_stage
    placements = generate_scene_on_stage(
        stage, config, parent_path="/World/stage/generated",
        scene_scale_factor=ssf)

    # COLLIDERS BEFORE STAGE C, exactly as the real launch scripts do — and
    # not a detail: `targets.settle_on_surface` raycasts down to find what a
    # victim is standing on, so without them every victim settles onto z=0
    # instead of onto the sidewalk or the rubble, and the sky check below has
    # nothing to hit either.
    # A PhysX scene has to exist before any of that means anything. The real
    # launch scripts get one from Pegasus/World; this bench builds a bare stage,
    # and without this line `add_colliders` authors colliders that no scene
    # query can see — every raycast returns nothing and the settle silently
    # falls back to z=0.
    if not stage.GetPrimAtPath(Sdf.Path("/World/physicsScene")).IsValid():
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
    gen = stage.GetPrimAtPath(Sdf.Path("/World/stage/generated"))
    if gen.IsValid():
        add_colliders(gen)
    for _ in range(10):
        omni.kit.app.get_app().update()

    # And PhysX only populates its query structures once the timeline has
    # stepped. A PhysicsScene plus colliders is NOT enough: two runs reported
    # "nothing under the first target" with both of those in place. Static
    # colliders, so nothing moves while it ticks. (`omni.timeline` is imported
    # at module scope on purpose: a function-local `import omni.timeline` makes
    # `omni` local to the whole function, and every earlier `omni.kit.app` line
    # in it dies with UnboundLocalError.)
    # Attach PhysX for queries and PROVE it (prints LIVE/BLIND).
    ensure_scene_queries(stage)
    # Left PLAYING rather than stopped, but THAT IS NOT THE FIX AND THIS BENCH
    # IS STILL BLIND. Measured 2026-08-25 on `urban_quake_tiny`, both ways
    # round: `add_colliders` authored every collider (the log lists them), the
    # PhysicsScene exists, the timeline has stepped — and every raycast in
    # `_sky_check` and in `findability.check_on_stage` still misses. A scene
    # query that hits nothing reads exactly like a scene with nothing in the
    # way, which is how this bench reported "24/24 interior candidates
    # reachable" from probes that never touched geometry. DO NOT BELIEVE A
    # RAYCAST NUMBER FROM THIS BENCH until that is fixed.
    #
    # Best remaining hypothesis, untested: a bare stage plus
    # `UsdPhysics.Scene.Define` never gets PhysX to ATTACH. The production
    # `scene_launch_script.py` goes through Pegasus/World, which builds a
    # SimulationContext, and its probes do hit — so the fix to try first is
    # standing up a SimulationContext here rather than defining the scene prim
    # by hand. Until then, in-sim findability evidence comes from
    # `scene_launch_script.py`, where `targets.place` prints the same report.

    add_sky(stage, resolve_sky(config))
    if PROBE_INTERIORS:
        _probe_interiors(stage, config, placements, ssf)

    # One built city, N populations. `targets.seed` is a separate RNG stream
    # from the scene's, so re-rolling it is the whole point of Stage C being
    # Stage C — the same baked city searched again with different people.
    victims = []
    for seed in (TARGET_SEEDS or [TARGET_SEED]):
        if seed:
            config.setdefault("targets", {})["seed"] = int(seed)
        if stage.GetPrimAtPath(Sdf.Path("/World/stage/targets")).IsValid():
            stage.RemovePrim(Sdf.Path("/World/stage/targets"))
            for _ in range(2):
                omni.kit.app.get_app().update()
        print(f"\n===== STAGE C  targets.seed={seed or '(default)'} =====")
        victims = kinds.get(config).place_targets(
            stage, config, placements=placements,
            parent_path="/World/stage/targets", scene_scale_factor=ssf)
        _sky_check(stage, victims, ssf)
    if MARKERS:
        _mark(stage, victims, ssf)
    return config, victims


def _probe_interiors(stage, config, placements, ssf: float) -> None:
    """Is "inside a standing building" a stratum? Ask the geometry.

    `targets.interior_candidates` samples spots just inside the facade of a
    standing OPEN-SHELL building — a person at a window. Whether any of them is
    findable is not answerable offline: the analytic model is one opaque box
    per standing building, so it calls every one of them buried by
    construction. Here there are real walls with real holes in them, so
    `findability.check_on_stage` gives a real answer.

    Nobody is placed. This prints a report and nothing else, which is the point
    — the stratum becomes a cohort only if the numbers say it should.
    """
    import findability
    import scene_generator
    import targets

    # Its own resolver, but a WARM one: `targets.place` has already measured
    # every asset in this scene through the shared `measure_cache`, so this is
    # a dictionary lookup rather than a second round of Nucleus round trips.
    # Passing None instead would fall back to the 12x12x12 m stand-in footprint
    # and put the candidates in the wrong place.
    resolver = scene_generator._make_resolver(config)
    sv = targets.survey_from_placements(
        placements, None, resolver,
        str((config.get("disaster") or {}).get("type", "none")))
    targets.mark_cut_geometry(stage, sv)
    cands = targets.interior_candidates(
        sv, targets.settings(config), targets.rng_for(config), PROBE_INTERIORS)
    if not cands:
        print("[targets_bench] interior probe: no standing open-shell "
              "building in this scene — nothing to sample")
        return
    rows = findability.check_on_stage(stage, cands, ssf)
    print(findability.format_report(rows, f"INTERIOR PROBE ({SCENE})"))
    ok = sum(1 for r in rows if r["verdict"] != "buried")
    print(f"[targets_bench] interior probe: {ok}/{len(rows)} candidates "
          f"reachable — {'a stratum' if ok else 'not a stratum'}")


def _sky_check(stage, victims, ssf: float, max_m: float = 400.0):
    """Raycast straight UP from every victim and report what is over them.

    This is the `visibility` label's reality check. The label is assigned from
    the cohort — a `street` casualty is `open`, a trapped one is `occluded` —
    and that is an assumption about geometry the sampler never sees. A drone
    looking down cannot see through a floor slab, so anything this hits means
    the label overstates what is findable from above.
    """
    try:
        import carb as _carb
        from omni.physx import get_physx_scene_query_interface
        sq = get_physx_scene_query_interface()
    except Exception as exc:                                     # noqa: BLE001
        print(f"[targets_bench] sky check unavailable: {exc}")
        return
    # Probe first: a raycast is only as good as the colliders under it, and a
    # scene with none reports "nothing overhead" for every victim including
    # the buried ones — which reads exactly like a clean bill of health.
    if victims:
        probe = sq.raycast_closest(
            _carb.Float3(victims[0]["x"] * ssf, victims[0]["y"] * ssf,
                         300.0 * ssf),
            _carb.Float3(0.0, 0.0, -1.0), 600.0 * ssf)
        if probe and probe.get("hit"):
            print(f"[targets_bench] collider probe: ground at "
                  f"{float(probe['position'][2]) / (ssf or 1.0):.3f} m "
                  f"({probe.get('collision', '?')})")
        else:
            print("[targets_bench] collider probe: NOTHING under the first "
                  "target — colliders are not on, so the sky check below and "
                  "the surface settle are both blind")
    # HIGH PROBE vs LOW PROBE, per victim. This is the measurement that
    # settles coasei-db's hazard: if a downward ray from 500 m over a collapsed
    # footprint stops tens of metres up while a ray from 4 m finds the rubble,
    # there is geometry hanging in the shell — a still-standing roof, or a
    # fragment reverted to its authored pose — and a victim settled against the
    # high hit would be buried in mid-air.
    print("[targets_bench] surface probes (high = from 500 m, low = from "
          "4 m):")
    for v in victims:
        hi = sq.raycast_closest(
            _carb.Float3(v["x"] * ssf, v["y"] * ssf, 500.0 * ssf),
            _carb.Float3(0.0, 0.0, -1.0), 1000.0 * ssf)
        lo = sq.raycast_closest(
            _carb.Float3(v["x"] * ssf, v["y"] * ssf, 4.0 * ssf),
            _carb.Float3(0.0, 0.0, -1.0), 8.0 * ssf)
        hz = (float(hi["position"][2]) / (ssf or 1.0)
              if hi and hi.get("hit") else None)
        lz = (float(lo["position"][2]) / (ssf or 1.0)
              if lo and lo.get("hit") else None)
        gap = "" if hz is None or lz is None else f"  gap={hz - lz:+.2f}"
        flag = "  <-- GHOST OVERHEAD" if (hz is not None and lz is not None
                                          and hz - lz > 2.0) else ""
        print(f"[targets_bench]   #{v['id']:<3} {v['cohort']:<14} "
              f"z={v.get('z', 0.0):+7.2f}  high="
              f"{'none' if hz is None else format(hz, '7.2f')}  low="
              f"{'none' if lz is None else format(lz, '7.2f')}{gap}{flag}")

    covered = []
    for v in victims:
        origin = _carb.Float3(v["x"] * ssf, v["y"] * ssf,
                              (float(v.get("z", 0.0)) + 0.4) * ssf)
        hit = sq.raycast_closest(origin, _carb.Float3(0.0, 0.0, 1.0),
                                 max_m * ssf)
        if hit and hit.get("hit"):
            covered.append((v, str(hit.get("collision", "?")),
                            float(hit["distance"]) / (ssf or 1.0)))
    print(f"[targets_bench] sky check: {len(covered)}/{len(victims)} target(s) "
          f"have something overhead")
    for v, path, dist in covered:
        print(f"[targets_bench]   #{v['id']:<3} {v['cohort']:<14} "
              f"{v['visibility']:<9} {dist:6.1f} m under {path}")


#: Cohort -> marker colour. Deliberately loud; these are not scenery.
_MARK_RGB = {
    "inside_rubble": (0.9, 0.1, 0.1),
    "exit_ring":     (1.0, 0.5, 0.0),
    "street":        (1.0, 0.9, 0.1),
    "open_space":    (0.1, 0.9, 0.2),
    "rubble_edge":   (0.2, 0.5, 1.0),
}


def _mark(stage, victims, ssf: float, height_m: float = 8.0):
    """A coloured post over every victim, so ONE top-down shot answers "did
    they land somewhere plausible" for all of them at once.

    A person is two or three pixels from 200 m up, and a close-up of each is
    a fight with the geometry in between — two of them came back as the inside
    of a rubble pile and as pure black. A post is visible from any altitude and
    says which cohort it is by colour.
    """
    from pxr import UsdGeom as _UG

    for v in victims:
        cyl = _UG.Cylinder.Define(
            stage, Sdf.Path(f"/World/stage/target_markers/m_{v['id']}"))
        cyl.CreateRadiusAttr(0.35 * ssf)
        cyl.CreateHeightAttr(height_m * ssf)
        cyl.CreateAxisAttr("Z")
        cyl.CreateDisplayColorAttr(
            [Gf.Vec3f(*_MARK_RGB.get(v["cohort"], (1.0, 1.0, 1.0)))])
        xf = _UG.Xformable(cyl.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(
            v["x"] * ssf, v["y"] * ssf,
            (float(v.get("z", 0.0)) + height_m / 2.0) * ssf))
    print(f"[targets_bench] marked {len(victims)} target(s) with posts")


class TargetsShowcaseApp:

    def __init__(self):
        usd_ctx = omni.usd.get_context()
        usd_ctx.new_stage()
        stage = usd_ctx.get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        stage.SetDefaultPrim(world.GetPrim())
        UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))
        _, ssf = get_stage_meters_per_unit(stage)

        row_points, row_victims = {}, []
        scene_config, scene_victims = None, []

        if POSES.strip():
            config = load_scene_config(ASSET_CONFIG)
            _ground(stage, 60.0, 20.0, ssf)
            row_points, row_victims = _pose_row(stage, config, ssf)
            add_sky(stage, resolve_sky(config))

        if SCENE.strip():
            scene_config, scene_victims = _scene(stage, ssf)

        for _ in range(20):
            omni.kit.app.get_app().update()

        if SNAP_DIR:
            try:
                snaps = _snaps()
                if row_points:
                    # Close enough to read a limb, and one shot of the whole
                    # row for the side-by-side.
                    snaps.views_around(stage, row_points, SNAP_DIR, ssf,
                                       top_h=6.0, obl_dist=5.0, obl_h=2.6,
                                       target_z=0.9)
                    span = (len(row_points) + 1) * ROW_STEP
                    snaps.overview(stage,
                                   ((len(row_points) - 1) * ROW_STEP / 2.0, 0.0),
                                   span, os.path.join(SNAP_DIR, "pose_row.png"))
                if scene_victims:
                    region = (scene_config.get("layout") or {}).get(
                        "region_m") or [400.0, 400.0]
                    snaps.overview(stage, (0.0, 0.0), float(max(region)),
                                   os.path.join(SNAP_DIR, "scene.png"), ssf)
                    # One victim per cohort, so every cohort is checked where
                    # it landed rather than in aggregate.
                    seen = {}
                    for v in scene_victims:
                        seen.setdefault(v["cohort"], (v["x"], v["y"]))
                    # PLUMB, and wide. An oblique close-up in a dense quake
                    # scene puts the camera inside a 15 m rubble pile or a
                    # facade — two runs came back as the inside of a rock and
                    # as pure black. Straight down from 50 m cannot, and the
                    # surroundings are what say whether a victim landed
                    # somewhere plausible.
                    for cohort, (vx, vy) in seen.items():
                        snaps.overview(stage, (vx, vy), 90.0,
                                       os.path.join(SNAP_DIR, f"{cohort}.png"),
                                       ssf)
                print(f"[targets_bench] snapshots -> {SNAP_DIR}")
            except Exception as exc:                             # noqa: BLE001
                print(f"[targets_bench] snapshots FAILED: {exc}")

        print("=" * 70)
        print("TARGETS SHOWCASE READY")
        print(f"  poses: {POSES or '(none)'}   scene: {SCENE or '(none)'}")
        print(f"  pose row: {len(row_victims)}   scene targets: {len(scene_victims)}")
        print(f"  snapshots: {SNAP_DIR or '(off — set SNAP_DIR)'}")
        print("=" * 70)

    def run(self):
        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            app.update()
        simulation_app.close()


def main():
    TargetsShowcaseApp().run()


if __name__ == "__main__":
    main()
