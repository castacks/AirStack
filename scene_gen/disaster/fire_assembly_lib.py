"""fire_assembly_lib — the parts of `fire_assembly_launch_script.py` that a
SECOND launcher needs, lifted out so it can import them.

WHY THIS MODULE EXISTS. `fire_assembly_launch_script.py` builds a
`SimulationApp` AT IMPORT (line 107 of the original) — `import
fire_assembly_launch_script` from another launcher would start a second Kit
app inside the first, which is the segfault `downtown_quake_launch_script.py`
already documents ("a second Kit app in one process is a segfault inside the
first second (measured)"). The city launcher
(`urban_fire_city_launch_script.py`, `urban_fire_city_plan.md` work item #7)
has to re-place the Flow emitters on a BAKE exactly the way the row launcher
does, so the alternative to this module is a second copy of `place_fire` —
the one function in the pipeline that must not drift, because it is the only
place `urban_fire._flame_sources` is called against a bake rather than
against a live building.

WHAT IS IN HERE, AND WHAT IS NOT. Everything here was MOVED, not rewritten:
`vram_mb`, `resolve_bakes`, `order_bakes`, `build_ground_and_light`,
`_sphere_source`, `place_fire`, and the fire-facing review-camera arithmetic
(`fire_view_params`) that used to live inline in the row launcher's capture
block. The three module-level knobs those functions used to close over —
`FA_GLOB`, `FA_ORDER`, `FA_SMOKE` — became ordinary parameters
(`pattern=`, `order=`, `smoke=`) and the log prefix became `prefix="fa"`, so
the row launcher passes its own env-derived values and gets byte-identical
behaviour. Nothing else changed; no logic was touched.

What is NOT here: `_env` (both launchers need it BEFORE `SimulationApp`,
i.e. before `scene_gen` is on `sys.path`), `KIT_ARGS` (same reason — it is a
literal in each launcher on purpose, see the row launcher's own comment) and
`_bbox` (three lines, and each launcher measures different prims).
"""

import glob as _glob
import math
import os

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

from . import fire as fx
from . import soot_plume as spl
from . import urban_fire as uf


# ---------------------------------------------------------------------------
# Finding the bakes
# ---------------------------------------------------------------------------
def vram_mb(tag, prefix="fa"):
    """Print and return the card's used VRAM (MiB) at a named stage.

    MEASURE AS YOU GO. The first assembled row filled a 16 GB card (14.3 GB)
    and the 1 km x 1 km scene this feeds has to fit a 5090 (32 GB) / RTX PRO
    5000 (48 GB) with every other component running — "at the current rate
    those would also have been exceeded" (user, 2026-08-30). Four readings
    per run — empty stage, geometry composed, Flow up, after the captures —
    give the per-building content cost and the Flow cost separately, which is
    what a projection needs.

    `prefix` only names the printing launcher (`fa` the row, `fc` the city);
    the row launcher's default keeps its lines exactly as they were.
    """
    import subprocess
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used,memory.total",
                              "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=10).stdout
        used, total = [float(x) for x in out.strip().split("\n")[0].split(",")[:2]]
    except Exception as exc:                        # no nvidia-smi in the image?
        print("[{0}] VRAM {1}: unavailable ({2})".format(prefix, tag, exc))
        return None
    print("[{0}] VRAM {1}: {2:.0f} / {3:.0f} MiB".format(prefix, tag, used, total))
    return used


def resolve_bakes(spec, pattern="*.usd"):
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


def order_bakes(rows, order=()):
    """Column order: `FA_ORDER` first if given, then each sidecar's own
    `index` (the manifest position it was baked from), then the file name."""
    if order:
        rank = {s: i for i, s in enumerate(order)}
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
def build_ground_and_light(stage, span, prefix="fa"):
    """Pavement-grey ground and a LOW warm key (25 deg).

    Copied from `urban_fire_bench` / `gac_fire_bench` on purpose: char is
    0.15 on screen and a spall scar 0.44, so under a flat overhead key the
    whole elevation crushes to black and none of the plume structure
    survives. A raking sun separates the tongues and the scars — and it is
    also what a drone flies in.

    NOT used by the CITY launcher: a generated city brings its own ground
    (`apply_ground_planes`) and its own sky (`add_sky`/`resolve_sky`), and a
    second 400 m quad at z=0 would z-fight the road surface.
    """
    import scene_generator as sg

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
        print("[{0}] ground material unavailable: {1}".format(prefix, exc))
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(700.0)
    dome.CreateColorAttr(Gf.Vec3f(0.72, 0.76, 0.86))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(3200.0)
    key.CreateAngleAttr(0.9)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.94, 0.86))
    key.AddRotateXYZOp().Set(Gf.Vec3f(-25.0, 0.0, 28.0))


def bbox(stage, path):
    """World-aligned `[x0, y0, z0, x1, y1, z1]` of `path`, or `None`.

    `useExtentsHint=False` on purpose — see `fix-floating-debris`: an
    extents hint is authored data and can be stale, which is exactly how
    airborne wood once audited as clean.
    """
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
               dx, dy, scale=1.0, max_emitters=9, smoke=True):
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

    `smoke` is the row launcher's `FA_SMOKE`, now an argument rather than a
    module global so the city launcher can pass its own.
    """
    f = doc["fire"]
    state = f.get("state")
    # A LEVEL WITH NO ACTIVE STATE CAN STILL SMOULDER. F1 has `ACTIVE=None`
    # by design, but on the GAC path `soot_plume.plan_events` now gives F1
    # one `smoulder` event (user, second row: "B0 has almost no damage") —
    # place a smoke wisp on it, no flames.
    wisp_only = False
    if not state:
        if any(ev.get("state") == "smoulder" and ev.get("ops")
               for ev in (events or [])):
            state, wisp_only = "smoulder", True
        else:
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
    if state == "smoulder" and not wisp_only:
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
    if not smoke:
        # flames only (FA_SMOKE=0): no vent smoke, no interior or roof plumes
        return {"flame": n_flame, "smoke": 0, "interior": 0, "roof": 0,
                "openings": n_open, "state": state,
                "note": note or "smoke off (FA_SMOKE=0)"}
    if wisp_only:
        # one wisp per smouldering window, half strength, nothing else
        n_smoke = 0
        for ev in [e for e in evs if e["state"] == "smoulder"][:2]:
            op = ev["ops"][len(ev["ops"]) // 2]
            n_smoke += uf._flame_sources(ctx, root, op, "smoulder", scale * 0.5,
                                         "wisp{0}".format(ev["id"]), 1)
        return {"flame": 0, "smoke": n_smoke, "interior": 0, "roof": 0,
                "openings": 0, "state": "wisp", "note": "F1 wisp"}
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
# THE REVIEW CAMERA FACES THE FIRE
# ---------------------------------------------------------------------------
def fire_view_params(doc, masses, box):
    """`views_around` arguments for one damaged building — was the arithmetic
    inline in `fire_assembly_launch_script`'s capture loop, moved here word
    for word so the city launcher frames a burning building the same way.

    `box` is the building's measured world bbox
    (`[x0, y0, z0, x1, y1, z1]`); `doc`/`masses` its sidecar and rehydrated
    mass boxes.

      * `top_h` — TOP-VIEW HEIGHT FROM THE BUILDING'S OWN MEASURED SIZE: at
        18 mm on the 20.955 mm aperture, 0.5 x the horizontal FOV is 1.164,
        so this is the standoff that fits the footprint and clears the roof
        — `gac_fire_bench`'s own arithmetic.
      * `azimuth_deg` — THE REVIEW CAMERA FACES THE FIRE. The default
        oblique looks from the south-west whatever burns; a building alight
        on E showed its blank back wall in every capture (fire_row3).
        Bearing = the burning sides' outward directions summed.
      * `aim_h` — the middle of the burning band, camera a little above it.
    """
    W, D, H = box[3] - box[0], box[4] - box[1], box[5] - box[2]
    top_h = max(W, D) / 1.164 * 1.45 + H
    obl_dist = max(50.0, 1.3 * max(W, D, H))
    obl_h = max(18.0, 0.4 * H)
    fd = (doc or {}).get("fire") or {}
    vec = {"E": (1, 0), "N": (0, 1), "W": (-1, 0), "S": (0, -1)}
    vx = sum(vec.get(sd, (0, 0))[0] for sd in (fd.get("sides") or []))
    vy = sum(vec.get(sd, (0, 0))[1] for sd in (fd.get("sides") or []))
    az = math.degrees(math.atan2(vy, vx)) if (vx or vy) else 225.0
    aim_h = 1.0
    try:
        lv = ((masses or {}).get(fd.get("mass") or "main")
              or list((masses or {}).values())[0])["levels"]
        sts = [int(q) for q in (fd.get("storeys") or [])]
        if sts:
            aim_h = 0.5 * (lv[min(sts[0], len(lv) - 1)]
                           + lv[min(sts[-1], len(lv) - 1)]) + 1.5
    except Exception:
        pass
    obl_h = max(obl_h, aim_h + 0.3 * obl_dist)
    return {"top_h": top_h, "obl_dist": obl_dist, "obl_h": obl_h,
            "azimuth_deg": az, "aim_h": aim_h}
