#!/usr/bin/env python
"""
fire_assembly — reference the per-building fire BAKES side by side as static
geometry, then put the smoke and the flames back.

    ISAAC_SIM_HEADLESS=false FA_BAKES=/isaac-sim/.cache/fire_bakes \
    FA_FLOW=1 KEEP_OPEN=1 SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/fire_row \
    PYTHONUNBUFFERED=1 PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
    /isaac-sim/python.sh \
    /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/fire_assembly_launch_script.py \
    --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts

The other half of `fire_bake_launch_script.py`. Each bake is a finished,
settled, physics-free building in its own `.usd` with a `.json` sidecar; this
references them at their column positions, adds the ground and the light the
benches use, and re-places the Flow emitters.

WHY THE SMOKE IS AUTHORED HERE AND NOT IN THE BAKE
---------------------------------------------------
The user's plan (2026-08-30): *"bake it then launch them together as
static"*. A Flow emitter is not geometry — it is a live simulation source
that has to exist on the stage that is being rendered, sharing the one
`FlowSimulate` layer the launcher authors. Baking it would freeze nothing
useful and would force the assembly to inherit each bake's own Flow stack.
So a bake carries **no Flow prims at all** and instead records its fire
EVENTS (`fire_bake.events_to_json`), which is exactly what
`urban_fire._flame_sources` — the function `r_flames` itself calls — needs to
put a sheet of sources across the head of the right window on the right
storey of the right elevation.

TWO THINGS THAT HAD TO CHANGE COMING BACK FROM A BAKE
------------------------------------------------------
1. **The column offset.** A bake is built at the ORIGIN and referenced at
   `x = column`, but an emitter is authored under `/World/flow/emitters`,
   which does NOT inherit the column's transform. `fire_bake.translate`
   moves each opening's wall FRAME (and the mass centres) by the column
   offset, so `_flame_sources` places world-correct emitters with no change
   to `urban_fire`.

2. **The smoke sits on what is LEFT of the building.** `_roof_plume` seats
   its sources at `m["top"] - 0.6` and `_interior_smoke` at
   `m["levels"][storey] + 0.4` — where the roof and the floors were
   PLANNED. After `fire_collapse` has dropped the top storeys into the shell
   those heights are metres above anything that still exists, and the plume
   hangs in clear air over the hole (user, 2026-08-30). The bake records its
   settled bbox and its interior/roof seats already clamped to it, and this
   script RE-CLAMPS against the bbox it measures on the referenced geometry —
   belt and braces, and it also catches a bake whose reference did not
   compose.

Env:
    FA_BAKES     comma list of baked `.usd` paths, OR a directory (its
                 `FA_GLOB` matches are used, sorted). Default
                 `/isaac-sim/.cache/fire_bakes`.
    FA_GLOB      glob inside a directory (default `*.usd`)
    FA_ORDER     comma list of stems to force the column order (default:
                 the sidecars' own `index`, then the file name)
    FA_SPACING   metres between columns (default 2.2 x the widest bake's
                 plan WIDTH, min 90 — `gac_fire_bench`'s own rule)
    FA_FLOW      1 (default) authors the Flow stack and the emitters
    FA_CELL_M    Flow density cell size, metres (default 0.12)
    FA_MAX_BLOCKS  the Flow block POOL (default 32768). THIS IS THE ONE THAT
                 SILENTLY LOSES THE SMOKE: `rtx/flow/maxBlocks` is a carb
                 setting, not the USD attribute, and when it runs out Flow
                 logs "Maximum Flow blocks ... in use" and every emitter
                 past the first few gets no voxels — a city that renders
                 with no smoke while reporting success.
    FA_EMITTERS  flame OPENINGS per building (default `urban_fire`'s 9)
    FA_SCALE     emission scale multiplier (default 1.0)
    FA_SEED      rng seed for the per-emitter jitter (default 7)
    SNAP_DIR     viewport captures, MUST be under
                 /isaac-sim/.nvidia-omniverse/logs/
    KEEP_OPEN    1 keeps the app up after the captures

Banner: `FIRE ASSEMBLY DONE`.
"""

import glob as _glob
import os
import random
import sys
import time

from isaacsim import SimulationApp


def _env(name, default=""):
    """The container exports every launcher knob as an EMPTY STRING, so
    `os.environ.get(name, default)` never reaches its default. Treat empty as
    absent."""
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
# FRACTIONAL CUTOUT OPACITY — `extra_args`, NOT `carb.settings`, and BOTH
# forms are required (the startup flag does not survive stage composition;
# the carb form alone is too late for startup). The bakes carry
# `soot_plume`'s baked atlases, `urban_fire._glass_pane`'s smoke deposits and
# the void tone, all fractional-cutout, which RTX discards unless this is on
# — the staining then renders as a hard binary stamp.
# `disaster/ground.py:KIT_ARGS` is the source of truth; a literal here
# because `scene_gen` is not on `sys.path` until after `SimulationApp`.
KIT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
            "--/rtx/pathtracing/fractionalCutoutOpacity=true"]

simulation_app = SimulationApp(launch_config={"headless": _HEADLESS,
                                              "extra_args": KIT_ARGS})

from isaacsim.core.utils.extensions import enable_extension    # noqa: E402

enable_extension("omni.kit.window.script_editor")
enable_extension("omni.flowusd")

import omni.kit.app                                            # noqa: E402
import omni.timeline                                           # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade        # noqa: E402

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from disaster import fire as fx                                # noqa: E402
from disaster import fire_bake as fb                           # noqa: E402
from disaster import soot_plume as spl                         # noqa: E402
from disaster import urban_fire as uf                          # noqa: E402

ASSEMBLY = "/World/assembly"

BAKES = _env("FA_BAKES", fb.DEFAULT_OUT_DIR)
GLOB = _env("FA_GLOB", "*.usd")
ORDER = [q.strip() for q in _env("FA_ORDER", "").split(",") if q.strip()]
FLOW = _env("FA_FLOW", "1") not in ("0", "false", "no")
# LOW-FIDELITY FIRE BY DEFAULT. The first row asked for 0.12 m cells and a
# 32,768-block pool for 82 emitters over 1.2 km: the pool alone is several
# GB of VRAM, the card was full (14.3 of 16.3 GB), textures stopped loading
# ("Insufficient VRAM, skipping TextureLoader") and no fire was visible. The
# user (2026-08-30): "make the fire lower fidelity ... ignore the smoke for
# now but the fire needs to be there". 0.3 m cells are 15x fewer cells per
# cubic metre; 6,144 blocks is a fifth of the pool; smoke emitters are off
# unless FA_SMOKE=1.
CELL_M = float(_env("FA_CELL_M", "0.3"))
MAX_BLOCKS = int(_env("FA_MAX_BLOCKS", "6144"))
MAX_EMITTERS = int(_env("FA_EMITTERS", str(6)))
# smoke back ON by default (user, second row: "add more fire and smoke to
# different buildings") — still the low-fidelity grid; VRAM is measured
SMOKE = _env("FA_SMOKE", "1") not in ("0", "false", "no")
SCALE = float(_env("FA_SCALE", "1.0"))
SEED = int(_env("FA_SEED", "7"))
SNAP_DIR = _env("SNAP_DIR", "")


# ---------------------------------------------------------------------------
# Finding the bakes
# ---------------------------------------------------------------------------
def vram_mb(tag):
    """Print and return the card's used VRAM (MiB) at a named stage.

    MEASURE AS YOU GO. The first assembled row filled a 16 GB card (14.3 GB)
    and the 1 km x 1 km scene this feeds has to fit a 5090 (32 GB) / RTX PRO
    5000 (48 GB) with every other component running — "at the current rate
    those would also have been exceeded" (user, 2026-08-30). Four readings
    per run — empty stage, geometry composed, Flow up, after the captures —
    give the per-building content cost and the Flow cost separately, which is
    what a projection needs.
    """
    import subprocess
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used,memory.total",
                              "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=10).stdout
        used, total = [float(x) for x in out.strip().split("\n")[0].split(",")[:2]]
    except Exception as exc:                        # no nvidia-smi in the image?
        print("[fa] VRAM {0}: unavailable ({1})".format(tag, exc))
        return None
    print("[fa] VRAM {0}: {1:.0f} / {2:.0f} MiB".format(tag, used, total))
    return used


def resolve_bakes(spec, pattern=GLOB):
    """`FA_BAKES` -> an ordered list of `(usd, json)` pairs.

    A directory is globbed; a comma list is taken as given. A `.usd` with no
    sidecar is still ASSEMBLED (its geometry is complete) but gets no
    emitters and says so — that is a bake whose export succeeded and whose
    sidecar write did not, and dropping the building would hide it.
    """
    out = []
    for item in [q.strip() for q in str(spec).split(",") if q.strip()]:
        if os.path.isdir(item):
            found = sorted(_glob.glob(os.path.join(item, pattern)))
        else:
            found = [item]
        for u in found:
            j = os.path.splitext(u)[0] + ".json"
            out.append((u, j if os.path.exists(j) else ""))
    seen, uniq = set(), []
    for u, j in out:
        if u in seen:
            continue
        seen.add(u)
        uniq.append((u, j))
    return uniq


def order_bakes(rows):
    """Column order: `FA_ORDER` first if given, then each sidecar's own
    `index` (the manifest position it was baked from), then the file name."""
    if ORDER:
        rank = {s: i for i, s in enumerate(ORDER)}
        return sorted(rows, key=lambda r: (
            rank.get(os.path.splitext(os.path.basename(r["usd"]))[0], 10 ** 6),
            r["doc"].get("index", 0) if r["doc"] else 0,
            os.path.basename(r["usd"])))
    return sorted(rows, key=lambda r: (
        r["doc"].get("index", 0) if r["doc"] else 0,
        os.path.basename(r["usd"])))


# ---------------------------------------------------------------------------
# Ground and light — the benches' own seat
# ---------------------------------------------------------------------------
def build_ground_and_light(stage, span):
    """Pavement-grey ground and a LOW warm key (25 deg).

    Copied from `urban_fire_bench` / `gac_fire_bench` on purpose: char is
    0.15 on screen and a spall scar 0.44, so under a flat overhead key the
    whole elevation crushes to black and none of the plume structure
    survives. A raking sun separates the tongues and the scars — and it is
    also what a drone flies in.
    """
    e = max(400.0, span * 1.4)
    ground = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    ground.CreatePointsAttr([Gf.Vec3f(-e, -e, 0.0), Gf.Vec3f(e, -e, 0.0),
                             Gf.Vec3f(e, e, 0.0), Gf.Vec3f(-e, e, 0.0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.30, 0.29)])
    ground.CreateExtentAttr([Gf.Vec3f(-e, -e, 0.0), Gf.Vec3f(e, e, 0.0)])
    try:
        mp = stage.DefinePrim(Sdf.Path("/World/Looks/pavement"))
        mp.GetReferences().AddReference(sg._join_asset_root(
            "airstack://scene_gen/assets/materials/megascans/Road_Asphalt.usda", ""))
        mp.Load()
        m = UsdShade.Material.Get(stage, "/World/Looks/pavement")
        if m:
            UsdShade.MaterialBindingAPI(ground.GetPrim()).Bind(m)
    except Exception as exc:
        print("[fa] ground material unavailable: {0}".format(exc))
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(700.0)
    dome.CreateColorAttr(Gf.Vec3f(0.72, 0.76, 0.86))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(3200.0)
    key.CreateAngleAttr(0.9)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.94, 0.86))
    key.AddRotateXYZOp().Set(Gf.Vec3f(-25.0, 0.0, 28.0))


def _bbox(stage, path):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                           useExtentsHint=False)
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return None
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    lo, hi = r.GetMin(), r.GetMax()
    return [float(lo[0]), float(lo[1]), float(lo[2]),
            float(hi[0]), float(hi[1]), float(hi[2])]


# ---------------------------------------------------------------------------
# The fire, put back
# ---------------------------------------------------------------------------
def _sphere_source(stage, path, seat, state, scale, vel, dx, dy, top_z):
    """One smoke-only `FlowEmitterSphere` at a recorded seat.

    The seat came out of the bake already clamped to the settled geometry;
    `top_z` here is what THIS stage measures on the referenced building, so a
    reference that composed differently (or not at all) cannot leave a plume
    in mid-air.
    """
    prim = fx._flow_create(stage, path, "FlowEmitterSphere")
    if not prim or not prim.IsValid():
        return 0
    z = float(seat["z"])
    if top_z is not None:
        z = min(z, float(top_z) - 0.4)
    fx._set(prim, "layer", Sdf.ValueTypeNames.Int, int(fx.FLOW_LAYER))
    fx._set(prim, "position", Sdf.ValueTypeNames.Float3,
            Gf.Vec3f(float(seat["x"]) + dx, float(seat["y"]) + dy, z))
    fx._set(prim, "radius", Sdf.ValueTypeNames.Float,
            float(seat.get("radius", 1.2)))
    fx._set(prim, "radiusIsWorldSpace", Sdf.ValueTypeNames.Bool, True)
    fx._set(prim, "coupleRateSmoke", Sdf.ValueTypeNames.Float, 2.0)
    fx._set(prim, "velocity", Sdf.ValueTypeNames.Float3, Gf.Vec3f(*vel))
    fx._set(prim, "velocityIsWorldSpace", Sdf.ValueTypeNames.Bool, True)
    fx.set_emission(prim, state, scale=float(seat.get("scale", 1.0)) * scale)
    return 1


def place_fire(stage, root, doc, masses, events, tag, rng, top_z,
               dx, dy, scale=1.0, max_emitters=9):
    """`urban_fire.r_flames`, re-run against a BAKED building.

    Same four parts, same budgets, same order — the difference is only where
    the inputs come from:

      1. FLAME events -> `FLAME_PER_OPENING` sheet sources across each
         opening's head, at most `max_emitters` openings. The opening records
         are the bake's own (`fire_bake.op_from_json`), so this is
         `urban_fire._flame_sources` verbatim, not a re-implementation.
      2. On an ACTIVE fire, smoke from the compartments that have already
         burnt OUT, highest storey first, `SMOKE_EXTRA_MAX` of them.
      3. On a burnt-out one, its SMOULDER events plus the interior smoke —
         which comes from the sidecar's recorded SEATS rather than from
         `ctx["fit"]`, because the fit-out is geometry inside a referenced
         file by now.
      4. The roof plume, gated on `fire["roof"]`, likewise from the seats.

    An event whose openings belong to a module a collapse killed is skipped:
    `e["dead"]` was serialised AFTER the ladder ran, so a flame never floats
    where a wall used to be.

    `_wall_vents` — `r_flames`' fallback for a band with no openings at all —
    is NOT reproduced: it walks `ctx["info"]["elements"]`, which a bake does
    not carry. A building that would have needed it says so and gets its
    roof/interior smoke only.
    """
    f = doc["fire"]
    state = f.get("state")
    if not state:
        return {"flame": 0, "smoke": 0, "interior": 0, "roof": 0,
                "note": "level {0} has no active state — no emitters"
                        .format(f.get("level"))}
    ctx = {"stage": stage, "rng": rng, "tag": tag, "notes": [],
           "flow_root": root, "info": {"masses": masses},
           "fire": {"origin": int(f["origin"]),
                    "storeys": [int(s) for s in f["storeys"]],
                    "top": int(f["top"]), "sides": tuple(f["sides"]),
                    "n_storeys": int(f["n_storeys"]), "mass": f["mass"],
                    "roof": bool(f["roof"]), "level": f["level"],
                    "state": state}}
    UsdGeom.Xform.Define(stage, Sdf.Path(root + "/emitters"))

    def live(ev):
        return all(not (o.get("e") or {}).get("dead") for o in ev["ops"])

    evs = [ev for ev in events if live(ev) and ev.get("ops")]
    is_flame = state == "flame"
    n_flame = n_open = 0
    # A TALL BUILDING BURNS IN MORE WINDOWS. Six openings read as a fire on
    # a 12-storey block and as a candle on a 37-storey tower ("the taller
    # buildings need more fire/flames, looks weird otherwise", user
    # 2026-08-30): from 12 storeys up the opening budget grows with the
    # height, capped at 16.
    n_st = int(f.get("n_storeys") or 0)
    max_open = (max(max_emitters, min(16, n_st // 2)) if n_st >= 12
                else max_emitters)
    for ev in [e for e in evs if e["state"] == "flame"]:
        for op in ev["ops"]:
            if n_open >= max_open:
                break
            n_flame += uf._flame_sources(
                ctx, root, op, "flame", scale,
                "e{0}_{1}".format(ev["id"], n_open), uf.FLAME_PER_OPENING)
            n_open += 1
    if state == "smoulder":
        # F4: the fire is DYING, not dead — a few openings are still alight
        # at reduced intensity, the rest smoulder. Without this an F4
        # building showed soot and a roof hole and no fire at all.
        for ev in [e for e in evs if e["state"] == "smoulder"]:
            for op in ev["ops"]:
                if n_open >= max(2, max_open // 2):
                    break
                n_flame += uf._flame_sources(
                    ctx, root, op, "flame", scale * 0.6,
                    "s{0}_{1}".format(ev["id"], n_open),
                    max(1, uf.FLAME_PER_OPENING - 1))
                n_open += 1
    note = ""
    if is_flame and n_open == 0:
        note = ("no live flame event with openings — `_wall_vents` is not "
                "available from a bake, so this building shows smoke only")

    n_smoke = 0
    if not SMOKE:
        # flames only (FA_SMOKE=0): no vent smoke, no interior or roof plumes
        return {"flame": n_flame, "smoke": 0, "interior": 0, "roof": 0,
                "openings": n_open, "state": state,
                "note": note or "smoke off (FA_SMOKE=0)"}
    if is_flame:
        out = sorted([e for e in evs if e["state"] == "out"],
                     key=lambda e: (-e["storey"], e["id"]))
        for ev in out[:uf.SMOKE_EXTRA_MAX]:
            op = ev["ops"][len(ev["ops"]) // 2]
            n_smoke += uf._flame_sources(ctx, root, op, "smoke", scale,
                                         "sm{0}".format(ev["id"]), 1)
    else:
        sm = [e for e in evs if e["state"] == "smoulder"]
        for ev in sm[:spl.SMOULDER_EVENTS_MAX]:
            op = ev["ops"][len(ev["ops"]) // 2]
            n_smoke += uf._flame_sources(ctx, root, op, state, scale,
                                         "sm{0}".format(ev["id"]), 1)

    seats = doc.get("seats") or {}
    n_int = 0
    if not is_flame:
        for k, seat in enumerate(seats.get("interior") or []):
            n_int += _sphere_source(
                stage, "{0}/emitters/{1}_int_{2}".format(root, tag, k),
                seat, state, scale, (0.0, 0.0, 1.6), dx, dy, top_z)
    n_roof = 0
    if f.get("roof"):
        for k, seat in enumerate(seats.get("roof") or []):
            n_roof += _sphere_source(
                stage, "{0}/emitters/{1}_roof{2}".format(root, tag, k),
                seat, "smoulder" if state == "flame" else state, scale,
                (0.6, 0.2, 3.2), dx, dy, top_z)
    return {"flame": n_flame, "smoke": n_smoke, "interior": n_int,
            "roof": n_roof, "openings": n_open, "state": state, "note": note}


# ---------------------------------------------------------------------------
def main():
    t0 = time.time()
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    usd_ctx = omni.usd.get_context()
    usd_ctx.new_stage()
    stage = usd_ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))
    UsdGeom.Xform.Define(stage, Sdf.Path(ASSEMBLY))
    _, ssf = get_stage_meters_per_unit(stage)

    vram = {"empty": vram_mb("empty stage")}
    pairs = resolve_bakes(BAKES)
    if not pairs:
        raise RuntimeError("FA_BAKES={0!r} matched no .usd".format(BAKES))
    rows = []
    for usd, js in pairs:
        doc = masses = events = None
        if js:
            try:
                doc, masses, events = fb.load_for_assembly(js)
            except Exception as exc:
                print("[fa] sidecar {0} unreadable ({1}) — geometry only"
                      .format(js, exc))
        rows.append({"usd": usd, "json": js, "doc": doc or {},
                     "masses": masses or {}, "events": events or []})
    rows = order_bakes(rows)

    # COLUMN PITCH FROM THE PLAN WIDTH, `gac_fire_bench`'s own rule: the row
    # faces the same way for every building, so it is the WIDTH that decides
    # how close two columns can stand. A bake with no sidecar contributes
    # nothing to the measurement and is spaced by whatever the rest need.
    widths = [abs(r["doc"]["bbox"][3] - r["doc"]["bbox"][0])
              for r in rows if (r["doc"].get("bbox"))]
    big = max(widths) if widths else 60.0
    spacing = float(_env("FA_SPACING", "0") or 0) or max(90.0, 2.2 * big)
    span = spacing * (len(rows) - 1) + 2.0 * big
    build_ground_and_light(stage, span)

    flow_root = None
    if FLOW:
        fx.setup_flow_stack(stage, density_cell_size_m=CELL_M,
                            max_blocks=MAX_BLOCKS, scene_scale_factor=ssf)
        flow_root = fx.FLOW_ROOT
        print("[fa] flow stack up at {0} ({1} m cells, {2} block pool)".format(
            flow_root, CELL_M, MAX_BLOCKS))

    # -- 1) reference every bake -------------------------------------------
    x0 = -0.5 * spacing * (len(rows) - 1)
    for i, r in enumerate(rows):
        x, y = x0 + i * spacing, 0.0
        r["x"], r["y"] = x, y
        holder = "{0}/b{1}".format(ASSEMBLY, i)
        xf = UsdGeom.Xform.Define(stage, Sdf.Path(holder))
        xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
        # REFERENCE ONTO A CHILD, not onto the Xform that carries the column
        # translate: the same idiom `gac_fire.place_source` and
        # `kit_bake.load_kit` use so a reference's own transform can never
        # compose with (or clobber) the one the caller authored.
        kid = stage.DefinePrim(Sdf.Path(holder + "/bake"))
        if not kid.GetReferences().AddReference(r["usd"]):
            print("[fa] FAILED to reference {0}".format(r["usd"]))
            continue
        stage.Load(Sdf.Path(holder + "/bake"))
        r["prim"] = holder
        tb = time.time()
        r["bbox"] = _bbox(stage, holder)
        r["load_s"] = time.time() - tb
        d = r["doc"]
        print("[fa] b{0} x={1:+8.1f}  {2:<28} {3:<4} {4}".format(
            i, x, d.get("name") or os.path.basename(r["usd"]),
            d.get("level") or "?",
            "bbox {0:.0f}x{1:.0f}x{2:.0f} m".format(
                r["bbox"][3] - r["bbox"][0], r["bbox"][4] - r["bbox"][1],
                r["bbox"][5] - r["bbox"][2]) if r["bbox"] else "EMPTY BBOX"))
        if d.get("src_kept"):
            print("[fa]     NOTE: this bake still carries its merged source "
                  "subtree (src_kept) — it composes and costs memory")
        for _ in range(2):
            omni.kit.app.get_app().update()
    for _ in range(10):
        omni.kit.app.get_app().update()
    vram["geometry"] = vram_mb("{0} bake(s) composed, no Flow".format(len(rows)))

    # -- 2) put the fire back ----------------------------------------------
    fires = []
    if FLOW and flow_root:
        for i, r in enumerate(rows):
            if not r.get("doc") or not r.get("prim"):
                continue
            # THE COLUMN OFFSET. `translate` moves the wall frames and the
            # mass centres, never `e["x"]/["y"]` — those are what
            # `_el_jitter` hashes for each module's stable severity wobble,
            # and shifting them would give the assembled building a
            # different flame-to-soot relationship than the one baked into
            # its own textures.
            fb.translate(r["masses"], r["events"], r["x"], r["y"])
            top_z = r["bbox"][5] if r.get("bbox") else r["doc"].get("top_z")
            res = place_fire(stage, flow_root, r["doc"], r["masses"],
                             r["events"], "a{0}".format(i),
                             random.Random(SEED + 31 * i), top_z,
                             r["x"], r["y"], scale=SCALE,
                             max_emitters=MAX_EMITTERS)
            res["i"] = i
            fires.append(res)
            print("[fa] b{0} fire: {1} flame source(s) over {2} opening(s), "
                  "{3} smoke, {4} interior, {5} roof  (state={6})".format(
                      i, res["flame"], res.get("openings", 0), res["smoke"],
                      res["interior"], res["roof"], res.get("state")))
            if res.get("note"):
                print("[fa]     " + res["note"])
        total = sum(f["flame"] + f["smoke"] + f["interior"] + f["roof"]
                    for f in fires)
        print("[fa] {0} Flow emitter(s) in all".format(total))
        # THE EMITTER BUDGET IS THE BUG THAT REPORTS SUCCESS. Flow's block
        # pool is finite and shared; past it, emitters get no voxels and the
        # scene renders with no smoke while every count above looks right.
        if total > 220:
            print("[fa] WARNING: {0} emitters is a lot for a {1} block pool "
                  "— if the row renders with smoke only on the first few "
                  "buildings, RAISE FA_MAX_BLOCKS or LOWER FA_EMITTERS, do "
                  "not go looking at the emitters".format(total, MAX_BLOCKS))
    for _ in range(10):
        omni.kit.app.get_app().update()

    # RE-ASSERT FRACTIONAL CUTOUT OPACITY, NOW THAT THE STAGE IS COMPOSED —
    # the second of the two required forms (see `KIT_ARGS` above). The
    # startup flag does not survive composition; the carb form alone is too
    # late for startup. Both launchers in this pipeline do both.
    try:
        for _ in range(30):                 # let Flow allocate its pool
            omni.kit.app.get_app().update()
        vram["flow"] = vram_mb("Flow up ({0} emitter(s))".format(
            sum(int(f.get("flame", 0)) + int(f.get("smoke", 0))
                + int(f.get("interior", 0)) + int(f.get("roof", 0))
                for f in fires)))
        import carb
        _s = carb.settings.get_settings()
        for _k in ("/rtx/raytracing/fractionalCutoutOpacity",
                   "/rtx/pathtracing/fractionalCutoutOpacity"):
            _s.set_bool(_k, True)
        print("[fa] fractionalCutoutOpacity re-asserted post-composition "
              "(raytracing={0}, pathtracing={1})".format(
                  _s.get("/rtx/raytracing/fractionalCutoutOpacity"),
                  _s.get("/rtx/pathtracing/fractionalCutoutOpacity")))
    except Exception as _exc:
        print("[fa] WARNING: could not re-assert fractionalCutoutOpacity "
              "({0}); the soot and the glass deposits will render as hard "
              "cutouts".format(_exc))
    for _ in range(4):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 78)
    print("FIRE ASSEMBLY   {0} bake(s), spacing {1:.0f} m, span {2:.0f} m"
          .format(len(rows), spacing, span))
    for i, r in enumerate(rows):
        d = r["doc"]
        b = r.get("bbox")
        print("  x={0:+8.1f}  {1:<10} {2:<24} {3:<4} {4:>6.1f} MB  "
              "{5} event(s)  {6}".format(
                  r.get("x", 0.0), d.get("kind", "?"),
                  d.get("name") or os.path.basename(r["usd"]),
                  d.get("level") or "?",
                  os.path.getsize(r["usd"]) / 1e6,
                  len(r["events"]),
                  "H {0:.0f} m".format(b[5] - b[2]) if b else "NO GEOMETRY"))
    print("  built in {0:.0f} s".format(time.time() - t0))
    print("=" * 78 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
            _spec2 = _ilu.spec_from_file_location("snapshots", _sp)
            _snaps = _ilu.module_from_spec(_spec2)
            _spec2.loader.exec_module(_snaps)
            os.makedirs(SNAP_DIR, exist_ok=True)
            # FLOW NEEDS TIME BEFORE IT IS PHOTOGRAPHED — the emitters inject
            # fuel per step, so a capture at t=0 is of an empty grid.
            if FLOW:
                timeline.play()
                for _ in range(300):
                    omni.kit.app.get_app().update()
            _snaps.overview(stage, (0.0, 0.0), span,
                            os.path.join(SNAP_DIR, "row.png"), ssf)
            tallest = max([(r["bbox"][5] - r["bbox"][2])
                           for r in rows if r.get("bbox")] or [30.0])
            _snaps.place_camera(
                stage, (-0.20 * span, -0.72 * span, 0.34 * span + tallest),
                (0.0, 0.0, tallest * 0.28))
            _snaps.snapshot(os.path.join(SNAP_DIR, "row_obl.png"))
            for i, r in enumerate(rows):
                b = r.get("bbox")
                if not b:
                    continue
                W, D, H = b[3] - b[0], b[4] - b[1], b[5] - b[2]
                name = "{0}_{1}_{2}".format(
                    i, (r["doc"].get("name")
                        or os.path.basename(r["usd"]).rsplit(".", 1)[0]),
                    r["doc"].get("level", ""))
                # TOP-VIEW HEIGHT FROM THE BUILDING'S OWN MEASURED SIZE: at
                # 18 mm on the 20.955 mm aperture, 0.5 x the horizontal FOV
                # is 1.164, so this is the standoff that fits the footprint
                # and clears the roof — `gac_fire_bench`'s own arithmetic.
                top_h = max(W, D) / 1.164 * 1.45 + H
                obl_dist = max(50.0, 1.3 * max(W, D, H))
                obl_h = max(18.0, 0.4 * H)
                _snaps.views_around(stage, {name: (r["x"], r["y"])}, SNAP_DIR,
                                    ssf, top_h=top_h, obl_dist=obl_dist,
                                    obl_h=obl_h)
            print("[fa] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[fa] snapshots FAILED: {0}".format(exc))

    # -- the budget --------------------------------------------------------
    vram["end"] = vram_mb("after captures")
    try:
        n_b = max(1, len(rows))
        base, geo = vram.get("empty"), vram.get("geometry")
        flow_v, end = vram.get("flow"), vram.get("end")
        if base is not None and geo is not None:
            per_b = (geo - base) / n_b
            flow_cost = (flow_v - geo) if flow_v is not None else None
            print("[fa] VRAM BUDGET: baseline {0:.0f} MiB | content {1:.0f} MiB "
                  "for {2} building(s) = {3:.0f} MiB/building | Flow {4} | "
                  "end {5:.0f} MiB".format(
                      base, geo - base, n_b, per_b,
                      "{0:.0f} MiB".format(flow_cost) if flow_cost is not None
                      else "off", end if end is not None else -1))
            for n_city, gpu, cap in ((40, "5090", 32768), (40, "RTX PRO 5000", 49152),
                                     (80, "5090", 32768), (80, "RTX PRO 5000", 49152)):
                proj = base + per_b * n_city + (flow_cost or 0)
                print("[fa]   projection: {0} damaged building(s) + this Flow "
                      "-> {1:.0f} MiB on a {2} ({3:.0%} of {4} MiB)".format(
                          n_city, proj, gpu, proj / cap, cap))
    except Exception as _exc:
        print("[fa] VRAM budget summary failed: {0}".format(_exc))

    print("FIRE ASSEMBLY DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception as _exc:
        import traceback
        traceback.print_exc()
        print("FIRE ASSEMBLY FAILED: {0}".format(_exc))
    keep = _env("KEEP_OPEN", "") == "1" or not _HEADLESS
    if keep:
        _app = omni.kit.app.get_app()
        omni.timeline.get_timeline_interface().play()
        while simulation_app.is_running():
            _app.update()
        omni.timeline.get_timeline_interface().stop()
    simulation_app.close()
