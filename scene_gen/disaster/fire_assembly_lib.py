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
import random

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade, Vt

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


# ---------------------------------------------------------------------------
# SCORCHED VEGETATION NEAR A BURNING BUILDING (2026-08-31)
# ---------------------------------------------------------------------------
# "if debris falls on trees, etc they can catch on fire so make them
# scorched... i don't really need debris there" (user, 2026-08-31) — no wood
# debris, no felling, no fracture: `vegetation.py`'s `burn_tree` pipeline
# (survey -> defoliate -> scorch_foliage/char_bole -> wood_debris/topple) is
# the WILDFIRE model, built for scenes where the tree IS the disaster and
# priced accordingly (per-tree MDL texture synthesis, new debris meshes). At
# CITY SCALE, with the tree only a bystander to a structure fire, the user's
# own instruction is the simpler bar: "a flat dark bind on leaf materials is
# acceptable". So this reuses exactly two things from `vegetation.py` — its
# `survey()` classification (which mesh/PointInstancer prototype is leaf vs.
# wood; it already knows a PointInstancer's material lives on its PROTOTYPE,
# which may be a subtree this pass would otherwise walk right past) and
# `_kind`/`_bound`'s MDL-aware material typing — and binds ONE flat charred
# material per role, built once for the whole city from `damage._pbr` with
# the same char/scorch RGB tones `urban_fire`'s own `_burn_set` draws from.
# Trees outside a fire's reach are never touched, so they "stay green" by
# construction (nothing here scans or rebinds them).
VEG_RADIUS_MULT = {"F2": 1.2, "F3": 1.3, "F4": 1.5, "F5": 1.7,
                   "F5c": 1.9, "F6": 2.1}
#: a squat building still reaches its own kerb trees — `1.2 x H` on an 6 m
#: two-storey shopfront is 7.2 m, inside a single street-tree bay.
VEG_RADIUS_FLOOR_M = 8.0


def veg_scorch_radius_m(level, height_m):
    """Vegetation scorch reach for one damaged building, in metres.

    0.0 below F2 — "level>=F2" per the brief: F0/F1 barely mark the shell,
    let alone anything standing near it, and F1 already carries its own
    wisp-only fire state (`fire_assembly_lib.place_fire`'s `wisp_only`).
    F2 upward scales `VEG_RADIUS_MULT` x the building's own MEASURED height
    (its composed bbox, so a partial collapse's shorter remaining shell
    reaches less far than its original height would have), floored at
    `VEG_RADIUS_FLOOR_M` so a short building is not given a radius smaller
    than the gap to its own street trees.
    """
    mult = VEG_RADIUS_MULT.get(level)
    if not mult:
        return 0.0
    return max(VEG_RADIUS_FLOOR_M, mult * max(0.0, float(height_m)))


def _dist_point_to_box_xy(x, y, box):
    """Nearest-point distance, in the XY plane, from `(x, y)` to an
    axis-aligned footprint `box = [x0, y0, z0, x1, y1, z1]` (this module's
    own `bbox()` shape) — 0.0 when the point is inside or on the footprint,
    so a tree standing inside a building's own settled street debris is
    never treated as "far from the fire"."""
    x0, y0, x1, y1 = float(box[0]), float(box[1]), float(box[3]), float(box[4])
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    return math.hypot(dx, dy)


def vegetation_scorch_targets(fuels, buildings):
    """`{prim_path: {"dist_m", "radius_m", "i", "level"}}` for every fuel
    placement within its nearest qualifying building's `veg_scorch_radius_m`.

    `fuels` is `fire.select_fuels(placements)`'s own `(x_m, y_m, prim_path)`
    triples (tree/plant/shrub/hedge/bush, matched by the SAME substring rule
    the wildfire spread solver uses, so "greenery" here means exactly what
    it means everywhere else in this codebase). `buildings` is
    `[{"i", "box", "level"}, ...]`, one entry per placed bake; an entry
    missing a box or level (no bake composed, or F0/F1) contributes nothing.

    A fuel in reach of more than one fire keeps the call with the larger
    MARGIN (radius minus distance) rather than the nearest fire outright —
    deterministic, order-independent, and biased toward whichever fire most
    clearly reaches it rather than whichever building merely happens to be
    closest.
    """
    out = {}
    for x, y, path in fuels:
        if not path:
            continue
        best = None
        for b in buildings:
            box = b.get("box")
            level = b.get("level")
            if not box or not level:
                continue
            h = float(box[5]) - float(box[2])
            r = veg_scorch_radius_m(level, h)
            if r <= 0.0:
                continue
            d = _dist_point_to_box_xy(float(x), float(y), box)
            if d > r:
                continue
            margin = r - d
            if best is None or margin > best[0]:
                best = (margin, d, r, b.get("i"), level)
        if best is not None:
            out[path] = {"dist_m": best[1], "radius_m": best[2],
                        "i": best[3], "level": best[4]}
    return out


def scorch_materials(stage, root):
    """The two flat charred materials every scorched tree binds — built ONCE
    for the whole city under `<root>/VegLooks`, so the prim cost of this pass
    is 2 materials total, not 2 per tree.

    `damage._pbr` with no texture (a flat OmniPBR, exactly the "flat dark
    bind" the brief allows) at `damage`'s own char/scorch RGB tones — the
    same tones `urban_fire._burn_set` draws its char/scorch/ash set from, so
    a scorched street tree reads as the same fire that damaged the building
    behind it rather than a separately-invented palette.
    """
    from . import damage

    leaf = damage._pbr(stage, root + "/VegLooks/leaf_char",
                       damage._CHAR_RGB, 0.95)
    trunk = damage._pbr(stage, root + "/VegLooks/trunk_char",
                        damage._SCORCH_RGB, 0.9)
    return {"leaf": leaf, "trunk": trunk}


def _bind_at_subset_granularity(prim, mat):
    """Bind `mat` at whatever granularity this prim's OWN material already
    lives at — its GeomSubsets if it has any, the prim itself otherwise.

    Mirrors `vegetation._bound`'s read side exactly, so an override binding
    authored here can never be masked by a more specific existing one — the
    same rule `urban_fire._bind_subsets` documents for a façade: "a whole-
    module bind is fatal ... the next pass's ordinary per-subset bind is
    then silently ignored". `_bound`/`_kind` are what CHOSE `mat` for this
    prim in the first place, so binding anywhere else would rebind the wrong
    granularity relative to what was just read.
    """
    subs = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
    targets = [s.GetPrim() for s in subs] or [prim]
    for t in targets:
        UsdShade.MaterialBindingAPI(t).Bind(mat)
    return len(targets)


def _bind_prototypes(stage, pi_path, mat):
    """Rebind every Mesh under a PointInstancer's PROTOTYPE subtree(s).

    The instancer prim itself carries no drawable geometry — the prototype
    does, and `vegetation.survey` already worked out which prototypes back
    THIS instancer's `leaf_pi`/`wood_pi` entry, so this only has to walk
    `GetPrototypesRel()` and bind what it finds.
    """
    pi_prim = stage.GetPrimAtPath(pi_path)
    if not pi_prim or not pi_prim.IsValid():
        return 0
    n = 0
    for t in UsdGeom.PointInstancer(pi_prim).GetPrototypesRel().GetTargets():
        proto = stage.GetPrimAtPath(t)
        if not proto or not proto.IsValid():
            continue
        for q in Usd.PrimRange(proto):
            if not q.IsA(UsdGeom.Mesh):
                continue
            n += _bind_at_subset_granularity(q, mat)
    return n


def apply_vegetation_scorch(stage, target_paths, root):
    """Bind the charred leaf/trunk materials onto every tree at `target_paths`.

    Runs `vegetation.survey` on each (the SAME classification the wildfire
    pipeline trusts: direct "bole" meshes plus `leaf_pi`/`wood_pi`
    PointInstancer entries), then rebinds bole meshes at their own subset
    granularity and instancer prototypes whole. `survey` already refuses an
    `IsInstance()` tree with a printed warning and an empty result, so a
    city built with `instanceable=true` street trees (it is not, today —
    `apply_placements` references each placement onto its own prim path) is
    a silent no-op here rather than a crash.
    """
    from . import vegetation as veg

    mats = scorch_materials(stage, root)
    n_leaf = n_trunk = n_trees = 0
    for path in target_paths:
        # AN INSTANCED TREE MUST BE DE-INSTANCED BEFORE THE REBIND. The 500 m
        # city now places vegetation `instanceable` (66,590 placements of 87
        # USDs OOM-killed composition twice, 2026-08-31), and USD forbids
        # edits inside an instance — `survey` would refuse it as a silent
        # no-op. Only the scorch TARGETS pay the de-instancing cost: a
        # handful of trees near the fires, while the green thousands stay
        # shared prototypes.
        _p = stage.GetPrimAtPath(path)
        if _p and _p.IsValid() and _p.IsInstanceable():
            _p.SetInstanceable(False)
        info = veg.survey(stage, path)
        touched = False
        for mesh_path, _mat_path, _tex, kind in info.get("bole", []):
            prim = stage.GetPrimAtPath(mesh_path)
            if not prim or not prim.IsValid():
                continue
            mat = mats["leaf"] if kind == "leaf" else mats["trunk"]
            _bind_at_subset_granularity(prim, mat)
            touched = True
            if kind == "leaf":
                n_leaf += 1
            else:
                n_trunk += 1
        for pi_path, _n in info.get("leaf_pi", []):
            k = _bind_prototypes(stage, pi_path, mats["leaf"])
            n_leaf += k
            touched = touched or k > 0
        for pi_path, _n in info.get("wood_pi", []):
            k = _bind_prototypes(stage, pi_path, mats["trunk"])
            n_trunk += k
            touched = touched or k > 0
        if touched:
            n_trees += 1
    return {"trees": n_trees, "leaf_binds": n_leaf, "trunk_binds": n_trunk}


def scorch_vegetation_pass(stage, placements, placed_rows, root):
    """The full scorched-vegetation pass, end to end.

    `placements` is the city's FULL placement list (`FireCityApp.placements`
    — `fire.select_fuels` does its own category filtering); `placed_rows` is
    `FireCityApp.placed`, each a bake row carrying `i`/`bbox`/`doc`.
    """
    buildings = [{"i": r.get("i"), "box": r.get("bbox"),
                 "level": ((r.get("doc") or {}).get("fire") or {})
                 .get("level")}
                for r in placed_rows]
    fuels = fx.select_fuels(placements)
    targets = vegetation_scorch_targets(fuels, buildings)
    stats = apply_vegetation_scorch(stage, list(targets.keys()), root)
    stats["fuels_total"] = len(fuels)
    stats["scorched"] = len(targets)
    stats["targets"] = targets
    return stats


# ---------------------------------------------------------------------------
# FIRE-SIDE DEBRIS APRON for a NON-COLLAPSE burning building (2026-08-31)
# ---------------------------------------------------------------------------
# "for the ones without a partial or full collapse, I want smaller debris
# particles on the sides that are on fire" (user, 2026-08-31) — F1..F5 only;
# F5c/F6 already drop an authored collapse heap (`fire_collapse`) and this
# apron is not meant to compete with (or duplicate) that rubble.
#
# ONE MESH PER BUILDING, NOT ONE PRIM PER LUMP. `quake_flow._a_lump` authors
# one `UsdGeom.Mesh` PER CALL — right for a collapse heap's few hundred
# chunks on ONE building, wrong here: with a global scatter over a whole
# city (up to `APRON_MAX_PER_SIDE` lumps per venting side, several sides per
# building, ~26 buildings) that idiom would cost hundreds of prims for
# geometry that never needs independent transforms or physics (the brief:
# "no debris needs authoring on them [trees]... Instanced or merged geometry
# preferred... keep the prim count bounded"). `_lump_points` reproduces
# `_a_lump`'s exact per-lump box/jitter/rotation math (so the LOOK is
# unchanged) but returns its 8 points already offset to a world seat instead
# of authoring a Mesh with a translate op, so `author_merged_lumps` can pack
# every lump for a building into ONE Mesh's points/faces — 1 prim (plus up
# to 2 material GeomSubsets) per building regardless of lump count.
APRON_LEVELS = ("F1", "F2", "F3", "F4", "F5")
APRON_DENSITY = {"F1": 4, "F2": 6, "F3": 9, "F4": 13, "F5": 17}
#: the wall length the base `APRON_DENSITY` count is tuned for
APRON_REF_SPAN_M = 14.0
#: bounds the merged mesh's triangle count even on a long block face
APRON_MAX_PER_SIDE = 40
#: a lump's nominal size, metres — "glass-scale" charred debris, not a
#: collapse chunk (`quake_flow._heap`'s own chunks run 1-3 m)
APRON_SIZE_M = (0.10, 0.28)
#: how far out from the wall face the scatter sits, onto the sidewalk
APRON_SETBACK_M = (0.25, 1.3)
#: kept clear of the corners (a fraction of the wall's own span), so a lump
#: never lands past the return wall this side does not own
APRON_INSET_FRAC = 0.08
#: ALWAYS THE DARK END — `urban_fire._debris_mat`'s own rule and ratio for
#: anything lying on the ground rather than on a visible elevation.
APRON_CHAR_P = 0.72


def apron_count(level, span_m):
    """How many small debris lumps one venting SIDE gets.

    Density scales with LEVEL (F1 a few wisps of char, F5 a real scatter)
    and with the wall's own length, referenced to `APRON_REF_SPAN_M` so a
    long block face is not as sparse as a narrow row-house front. Floored at
    2 (once a level qualifies at all, there is always SOMETHING to see) and
    capped at `APRON_MAX_PER_SIDE`.
    """
    base = APRON_DENSITY.get(level)
    if not base:
        return 0
    n = int(round(base * max(0.6, float(span_m) / APRON_REF_SPAN_M)))
    return max(2, min(APRON_MAX_PER_SIDE, n))


def apron_points_for_side(m, side, level, rng):
    """`[(x, y, size_m), ...]` world-space seats along one venting side's
    base wall line, offset outward onto the ground.

    `m` must already be in the building's CITY frame (see `world_masses` —
    `quake_flow._to_world`/`_outward`/`_p_wall_point` all read `m["cx"]`/
    `["cy"]`/`["yaw"]`, which are the bake-LOCAL origin until something
    applies the cell's placement). Z is deliberately not decided here: it is
    the caller's concern (the building's own MEASURED ground contact, e.g.
    this module's own `bbox()`), not this mass's analytic `z0`, because
    `fire_bake.place` never touches `z0` (a yaw about the vertical axis does
    not move height, and the city cell's own dz is applied by the holder's
    translate, not by the mass).
    """
    from . import quake_flow as qf

    span = qf._a_side_span(m, side)
    n = apron_count(level, span)
    if n <= 0:
        return []
    ox, oy = qf._outward(m, side)
    lo, hi = span * APRON_INSET_FRAC, span * (1.0 - APRON_INSET_FRAC)
    if hi <= lo:
        lo, hi = 0.0, span
    out = []
    for _ in range(n):
        t = rng.uniform(lo, hi)
        lx, ly = qf._p_wall_point(m, side, t)
        wx, wy = qf._to_world(m, lx, ly)
        d = rng.uniform(*APRON_SETBACK_M)
        s = rng.uniform(*APRON_SIZE_M)
        out.append((wx + ox * d, wy + oy * d, s))
    return out


def world_masses(masses, x, y, yaw_deg):
    """A DEEP COPY of `masses`, rotated+translated to its city cell.

    Exactly `fire_bake.place`'s own transform, run on a COPY: a caller that
    only needs a building's WORLD-SPACE wall lines (this apron pass) must
    not mutate the ORIGINAL `masses` dict, because `put_the_fire_back` (the
    city launcher's own step 4) later calls `fire_bake.place` on that same
    original once it decides the building's Flow emitter allocation —
    calling `place` on the same dict twice would rotate+translate it AGAIN,
    silently doubling the transform for whichever pass ran second.
    """
    import copy

    from . import fire_bake as fb

    out = copy.deepcopy(masses)
    fb.place(out, [], None, x, y, yaw_deg)
    return out


_LUMP_FACES = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
              (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))


def _lump_points(cx, cy, cz, s, rng, jitter=0.3):
    """One `quake_flow._a_lump`-shaped box's 8 points, already offset to a
    WORLD seat `(cx, cy, cz)`.

    Reproduces `_a_lump`'s box/jitter/three-axis-rotation math verbatim (the
    look must not change) but returns points instead of authoring a Mesh
    with a translate op, so `author_merged_lumps` can pack many lumps into
    one Mesh's own point array.
    """
    hx = s * 0.5
    hy = s * rng.uniform(0.45, 1.0) * 0.5
    hz = s * rng.uniform(0.35, 0.85) * 0.5
    ya, pa, ra = (rng.uniform(0.0, 2.0 * math.pi), rng.uniform(-0.9, 0.9),
                  rng.uniform(-0.9, 0.9))
    cya, sya = math.cos(ya), math.sin(ya)
    cp, sp = math.cos(pa), math.sin(pa)
    cr, sr = math.cos(ra), math.sin(ra)
    pts = []
    for dz in (-hz, hz):
        for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
            x = dx * (1.0 + rng.uniform(-jitter, jitter))
            y = dy * (1.0 + rng.uniform(-jitter, jitter))
            z = dz * (1.0 + rng.uniform(-jitter, jitter))
            y, z = y * cp - z * sp, y * sp + z * cp
            x, z = x * cr - z * sr, x * sr + z * cr
            x, y = x * cya - y * sya, x * sya + y * cya
            pts.append((cx + x, cy + y, cz + z))
    return pts


def author_merged_lumps(stage, path, seats, rng, mat_char=None,
                        mat_scorch=None, char_p=APRON_CHAR_P):
    """One Mesh prim carrying every lump in `seats` (`[(x, y, z, size), ...]`).

    Each lump is independently assigned char or scorch (weighted `char_p`,
    `urban_fire._debris_mat`'s own ratio) via TWO `materialBind` GeomSubsets
    — `UsdShade.Tokens.materialBind` / `UsdGeom.Tokens.partition` are BOTH
    required on `CreateGeomSubset`, or the subset is silently ignored by the
    renderer and the whole mesh falls back to one material (the exact trap
    `surge.py`'s `build_ponding` already documents and works around). Falls
    back to a WHOLE-mesh bind (whichever material was given) so a caller
    that wants the apron in one tone can pass only `mat_char`.

    Returns `(prim, n_lumps)`; `(None, 0)` for an empty scatter — no prim is
    authored for a building with nothing to place.
    """
    if not seats:
        return None, 0
    points, counts, indices = [], [], []
    char_faces, scorch_faces = [], []
    for x, y, z, s in seats:
        base = len(points)
        points.extend(_lump_points(x, y, z, s, rng))
        face_start = len(counts)
        for f in _LUMP_FACES:
            counts.append(4)
            indices.extend(base + k for k in f)
        faces = list(range(face_start, len(counts)))
        (char_faces if rng.random() < char_p else scorch_faces).extend(faces)
    me = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    me.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
    me.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    me.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
    me.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    lo = [min(p[k] for p in points) for k in range(3)]
    hi = [max(p[k] for p in points) for k in range(3)]
    me.CreateExtentAttr([Gf.Vec3f(*lo), Gf.Vec3f(*hi)])
    fallback = mat_char if mat_char is not None else mat_scorch
    if fallback is not None:
        UsdShade.MaterialBindingAPI.Apply(me.GetPrim()).Bind(fallback)
    if mat_char is not None and char_faces:
        sub = UsdGeom.Subset.CreateGeomSubset(
            me, "apronChar", UsdGeom.Tokens.face, Vt.IntArray(char_faces),
            UsdShade.Tokens.materialBind, UsdGeom.Tokens.partition)
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat_char)
    if mat_scorch is not None and scorch_faces:
        sub = UsdGeom.Subset.CreateGeomSubset(
            me, "apronScorch", UsdGeom.Tokens.face, Vt.IntArray(scorch_faces),
            UsdShade.Tokens.materialBind, UsdGeom.Tokens.partition)
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat_scorch)
    return me.GetPrim(), len(seats)


def apron_debris_materials(stage, root):
    """The two flat debris tones the apron draws from — built ONCE for the
    whole city under `<root>/DebrisLooks`. ALWAYS THE DARK END: same rule
    and RGB tones `urban_fire._debris_mat`/`damage._pbr` use for anything
    lying on the ground rather than on a visible elevation."""
    from . import damage

    char = damage._pbr(stage, root + "/DebrisLooks/apron_char",
                       damage._CHAR_RGB, 0.92)
    scorch = damage._pbr(stage, root + "/DebrisLooks/apron_scorch",
                         damage._SCORCH_RGB, 0.88)
    return {"char": char, "scorch": scorch}


def build_fire_apron(stage, root, r, rng):
    """The debris apron for one placed bake, or a no-op note for one that
    does not qualify.

    `r` is one `FireCityApp.placed` row (`i`/`stem`/`x`/`y`/`yaw`/`bbox`/
    `doc`/`masses`). Gated on `doc["fire"]["level"]` in `APRON_LEVELS`
    (F1..F5 — no F5c/F6, which already drop a collapse heap) and on having
    at least one recorded venting side (`doc["fire"]["sides"]`); a building
    failing either gate gets `"prim": None` and a `"note"` saying why.
    """
    doc = r.get("doc") or {}
    f = doc.get("fire") or {}
    level = f.get("level")
    if level not in APRON_LEVELS:
        return {"prim": None, "n": 0, "sides": (), "level": level,
                "note": "level {0!r} not F1-F5 (collapse or unburnt)"
                        .format(level)}
    sides = tuple(f.get("sides") or ())
    if not sides:
        return {"prim": None, "n": 0, "sides": (), "level": level,
                "note": "no venting sides recorded"}
    tag = f.get("mass") or "main"
    masses = r.get("masses") or {}
    world = world_masses(masses, float(r.get("x", 0.0)),
                         float(r.get("y", 0.0)), float(r.get("yaw", 0.0)))
    m = world.get(tag) or (list(world.values())[0] if world else None)
    if not m:
        return {"prim": None, "n": 0, "sides": sides, "level": level,
                "note": "mass {0!r} not found in this bake's sidecar"
                        .format(tag)}
    box = r.get("bbox")
    z0 = float(box[2]) if box else float(m.get("z0", 0.0))
    seats = []
    for side in sides:
        for x, y, s in apron_points_for_side(m, side, level, rng):
            seats.append((x, y, z0, s))
    mats = apron_debris_materials(stage, root)
    path = "{0}/apron/{1}".format(root, r.get("stem", "d{0}".format(r.get("i", 0))))
    prim, n = author_merged_lumps(stage, path, seats, rng,
                                  mat_char=mats["char"],
                                  mat_scorch=mats["scorch"])
    return {"prim": str(prim.GetPath()) if prim else None, "n": n,
            "sides": sides, "level": level}


def fire_apron_pass(stage, root, placed_rows, seed=7):
    """The full fire-side debris apron pass over every placed bake.

    One `build_fire_apron` call per row, with a per-building rng so the
    scatter is stable given a stable `FA_SEED` without sharing draws across
    buildings (the same discipline `place_fire`'s own per-building
    `random.Random(SEED + 31 * r["i"])` follows).
    """
    out = []
    for r in placed_rows:
        rng = random.Random(int(seed) + 97 * int(r.get("i") or 0))
        res = build_fire_apron(stage, root, r, rng)
        res["i"] = r.get("i")
        res["stem"] = r.get("stem")
        out.append(res)
    return out
