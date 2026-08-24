"""fire stage — a wildfire as NVIDIA Flow fire and smoke.

WHAT ISAAC SIM GIVES YOU, AND WHAT IT DOES NOT
----------------------------------------------
Flow (`omni.flowusd`) is a sparse-voxel gas solver with a real combustion model.
Inside its grid it genuinely simulates: an emitter injects fuel, fuel above
`ignitionTemp` burns, burning makes temperature and smoke, temperature makes
buoyancy, vorticity curls the plume. A plume evolves; it is not a looping
sprite.

What Flow does NOT do is spread between scene objects. It has no idea a tree is
a tree. An emitter is a geometric region that injects fuel into the grid, and
setting one tree alight will never set its neighbour alight.

NVIDIA's own fire event confirms this. `isaacsim.replicator.incident` creates
exactly one `FlowEmitterBox` per tagged prim and runs it through a hardcoded
three-state clock (idle -> ignition at +5 s -> flame), and its `FuelSource`
carries the comment "write a replacement fuel model that uses configurable fuel
and smoke sources". Its `pyro_nearby_radius` argument is threaded all the way
into the incident report and then never used to ignite anything.

So the spread model is ours. That is the right outcome for a dataset: the
ignition schedule is seeded and reproducible, and the burn state of every prop
at every timestep is known exactly rather than emerging from a solver.

THE SPREAD MODEL
----------------
Wind-driven elliptical growth — the Alexander / Van Wagner model that fire
behaviour analysts actually use. From the ignition point the front is an
ellipse that grows at `head_mps` downwind, `back_mps` upwind and `flank_mps`
sideways. Because every axis scales linearly with time, "when does the front
reach this tree" has a closed form (see `_ignition_time`) — no timestepping, no
cellular automaton, one quadratic per fuel.

Ember spotting is layered on top: a fraction of fuels ignite early, as if a
brand blew ahead of the front. That is what produces the ragged, non-convex
burn edge that a clean ellipse lacks.

BUDGET
------
Flow allocates 32x16x16-voxel blocks from a pool that defaults to 4096, so the
coverable volume is about `4096 * 8192 * densityCellSize**3` cubic metres —
worst case, since the grid is sparse and smoke does not fill a box. At NVIDIA's
warehouse value of 0.05 m that is a 16 m cube. A suburb needs 0.2-0.5 m and a
raised `max_blocks`.

The other lever is emitter count. A full suburb plants ~3,400 trees; every
active emitter allocates blocks. `emitter_spacing_m` decimates the fuel list on
a grid so the front stays continuous instead of patchy, `max_emitters` caps it
outright, and every emitter outside its burn window is left `enabled = False`
so it costs nothing.

UNITS
-----
Flow's shipped defaults come from USD Composer, which is centimetres, and this
is the single most reported Flow problem on the NVIDIA forums. Everything here
is authored in metres and multiplied by `scene_scale_factor` (= 1 /
metersPerUnit) on the way into USD, the same convention `apply_placements`
uses. Emitters live under an identity-transform scope so `position` and
`halfSize` share one frame.

USAGE
-----
    from disaster import fire
    driver = fire.apply_wildfire(stage, placements, config.get("fire", {}),
                                 scene_scale_factor=ssf)

Keep the returned driver alive — it holds the timeline subscription that
advances the burn.
"""

import math

from pxr import Gf, Sdf, Usd, UsdGeom

# Flow ties an emitter to a simulation by a shared integer. Mismatched layers
# is the single most common "nothing happens" cause; 3 is what the incident
# extension uses and there is no reason to differ.
FLOW_LAYER = 3

FLOW_ROOT = "/World/flow"

# Anything whose placement category matches one of these is fuel. Substring
# match, because the suburb names categories by role (`street_tree`, `yard_tree`)
# rather than from one enum.
DEFAULT_FUEL_CATEGORIES = ("tree", "plant", "shrub", "hedge", "bush")

DEFAULTS = {
    "enabled": True,
    "seed": 0,

    # Where it started and which way the wind pushes it, in the same metric
    # frame as the placements (region centred on the origin).
    "origin_m": [0.0, 0.0],
    "heading_deg": 45.0,

    # Rate of spread. 1.2 m/s downwind is a fast-moving crown fire; the 6:1
    # head-to-flank ratio is a typical wind-driven ellipse.
    "head_mps": 1.2,
    "flank_mps": 0.2,
    "back_mps": 0.06,

    # How far into the burn the scene is when the sim starts. The front keeps
    # advancing from there, so this sets how much of the region is already
    # alight at t=0. Give either the seconds or, more usefully, the fraction of
    # `duration_s` — the fraction wins when both are set, so retuning the
    # spread rate does not silently change how much of the scene opens alight.
    "start_offset_s": 0.0,
    "start_offset_frac": None,
    # Fuels the front reaches after this never ignite — it is what bounds the
    # burn to part of the region instead of eventually taking all of it.
    "duration_s": 600.0,

    # Per-fuel timing noise, so the front is a band rather than a hard line.
    "jitter_s": 8.0,
    # Embers blown ahead of the front.
    "spot_chance": 0.04,
    "spot_lead_s": 90.0,

    # How long a single fuel takes to go through its states.
    "ignition_s": 6.0,     # smoking, pre-flame
    "flame_s": 120.0,      # flaming
    "smoulder_s": 240.0,   # smoke only, decaying

    # Emitter budget — see BUDGET above.
    "emitter_spacing_m": 6.0,
    "max_emitters": 160,
    "fuel_categories": list(DEFAULT_FUEL_CATEGORIES),

    # Solver resolution. NVIDIA's warehouse fire uses 0.05; anything much
    # coarser than 0.1 dilutes the burn across the cell until the flame band
    # of the colormap is out of reach and only smoke renders.
    "density_cell_size_m": 0.1,
    # The block pool, applied as the `rtx/flow/maxBlocks` carb setting.
    #
    # STRAIGHT VRAM. A block is 32x16x16 voxels across several channels, so
    # the pool runs to gigabytes well before this number looks large — 65536
    # is enough to lose a 16 GB card on its own. 4x NVIDIA's 4096 default
    # covers the peak concurrent emitter count with room to spare; raise it
    # only against the "Maximum Flow blocks in use" warning, and watch
    # nvidia-smi when you do.
    "max_blocks": 16384,
    # Where the top of the flame colour ramp sits in temperature. 1.0 is
    # NVIDIA's value and is proven to render fire on the test bench; the
    # 0.35 this used to carry was invented to explain flameless smoke that
    # turned out to be the missing param prims. Lower it only if the flames
    # really are too dim, and against the bench rather than a full suburb.
    "colormap_x_max": 1.0,

    # POST-DISASTER SHAPE. `flaming_fraction` is how much of the burn is still
    # actually alight; everything else only ever smokes and smoulders.
    # `residual_smoke_frac` is the thin drift left over burnt ground, which
    # never switches off — it is what keeps the scene from going clean behind
    # the front.
    "flaming_fraction": 0.18,
    "intensity_range": [0.25, 1.0],
    "residual_smoke_frac": 0.12,

    # Emitter sphere radius bounds, metres.
    "emitter_radius_m": [0.7, 2.6],

    # Wood smoke, not the warehouse ramp's near-black soot.
    "smoke_rgb": [0.44, 0.42, 0.40],
    "smoke_opacity": 0.30,

    # Injection strength, straight from the incident extension's fuel model.
    "flame_fuel": 0.5,
    "flame_smoke": 2.0,
    "smoke_only": 2.0,
    "temperature": 2.0,

    # Wind carried into the plume, m/s. Derived from `heading_deg` unless set.
    "wind_mps": 4.0,

    # Label burning prims for synthetic-data ground truth.
    "semantics": True,
    "semantic_class": "fire",
}


# --------------------------------------------------------------------------
# Flow stack
# --------------------------------------------------------------------------

def _flow_create(stage, prim_path, type_name):
    """Create a Flow prim, preferring the command that builds its child params.

    `FlowCreatePrim` also defines the `advection`, `colormap`, `rayMarch` etc.
    sub-prims that carry most of the settings. Falling back to `DefinePrim`
    keeps this importable outside Kit (tests, tooling) where the command
    registry does not exist.
    """
    try:
        import omni.kit.commands
        ok, prim = omni.kit.commands.execute(
            "FlowCreatePrim", prim_path=Sdf.Path(prim_path), type_name=type_name)
        if ok and prim:
            return prim
    except Exception:
        pass
    return stage.DefinePrim(Sdf.Path(prim_path), type_name)


def _child(stage, path, type_name):
    """Get or DEFINE a Flow parameter sub-prim.

    `FlowCreatePrim` creates ONE prim. Its whole implementation is
    `stage.DefinePrim(prim_path, type_name)` — it does not author the
    `advection`, `colormap`, `rayMarch`, `renderSettings` children that carry
    almost every setting that matters. Reading them back with GetPrimAtPath
    returns invalid prims, so `combustionEnabled` was never enabled and the
    flame half of the colormap was never authored: Flow advected smoke and
    nothing ever burned.

    This is also why `isaacsim.replicator.incident` has
    `setup_pyro_settings()` commented out and loads
    `data/warehouse_fire_settings.usda` as a payload instead — that file
    declares the full hierarchy. The types below are taken from it.
    """
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim.IsValid():
        prim = stage.DefinePrim(Sdf.Path(path), type_name)
    return prim


def _set(prim, name, type_name, value):
    """Author *name* on *prim*, complaining loudly if the prim is not there.

    This used to `return` quietly on an invalid prim, which is how an entire
    Flow configuration went missing without a single line of output — see
    `_child` for what was actually wrong.
    """
    if prim is None or not prim.IsValid():
        print("[fire] WARNING: cannot author {0!r} — prim missing".format(name))
        return
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, type_name, custom=True)
    attr.Set(value)


def setup_flow_stack(stage, *, layer=FLOW_LAYER, density_cell_size_m=0.1,
                     max_blocks=16384, colormap_x_max=1.0,
                     smoke_rgb=(0.44, 0.42, 0.40), smoke_opacity=0.30,
                     scene_scale_factor=1.0, root=FLOW_ROOT):
    """Author the FlowSimulate / FlowOffscreen / FlowRender trio.

    All three plus every emitter must share `layer` or nothing simulates.

    The combustion and colormap values are the incident extension's warehouse
    fire (`pyro_demon/pyro_settings_setup.py`), which is already tuned for
    metres and Z-up. Only resolution and pool size are opened up here, because
    those are what a neighbourhood-sized fire actually needs to change.
    """
    ssf = float(scene_scale_factor)
    sim_path = root + "/flowSimulate"
    if stage.GetPrimAtPath(sim_path).IsValid():
        return stage.GetPrimAtPath(sim_path)

    UsdGeom.Scope.Define(stage, Sdf.Path(root))

    try:
        import omni.kit.commands
        for path in ("rtx/flow/enabled",
                     "rtx/flow/rayTracedReflectionsEnabled",
                     "rtx/flow/rayTracedTranslucencyEnabled",
                     "rtx/flow/pathTracingEnabled"):
            omni.kit.commands.execute("ChangeSetting", path=path, value=True)
        # THE BLOCK POOL LIVES HERE, not on flowRender/renderSettings.
        # Setting only the USD attribute leaves the pool at its 4096 default
        # and Flow logs "Maximum Flow blocks of 4096 in use" while every
        # emitter after the first few silently gets nothing — which reads as
        # "the fire is only in a couple of places".
        omni.kit.commands.execute("ChangeSetting", path="rtx/flow/maxBlocks",
                                  value=int(max_blocks))
    except Exception:
        pass

    sim = _flow_create(stage, sim_path, "FlowSimulate")
    _set(sim, "layer", Sdf.ValueTypeNames.Int, int(layer))
    _set(sim, "level", Sdf.ValueTypeNames.Int, 0)
    _set(sim, "autoCellSize", Sdf.ValueTypeNames.Bool, False)
    _set(sim, "densityCellSize", Sdf.ValueTypeNames.Float,
         float(density_cell_size_m) * ssf)
    _set(sim, "blockMinLifetime", Sdf.ValueTypeNames.UInt, 6)
    _set(sim, "stepsPerSecond", Sdf.ValueTypeNames.Float, 60.0)
    _set(sim, "timeScale", Sdf.ValueTypeNames.Float, 1.0)
    _set(sim, "maxStepsPerSimulate", Sdf.ValueTypeNames.UInt, 1)
    _set(sim, "velocitySubSteps", Sdf.ValueTypeNames.UInt, 1)
    _set(sim, "levelCount", Sdf.ValueTypeNames.Int, 1)
    _set(sim, "levelCellSizeMultiplier", Sdf.ValueTypeNames.Float, 0.5)
    # Colliders are off on the suburb preview path, so there is nothing for
    # smoke to collide with and this would only cost.
    _set(sim, "physicsCollisionEnabled", Sdf.ValueTypeNames.Bool, False)
    _set(sim, "simulateWhenPaused", Sdf.ValueTypeNames.Bool, False)

    adv = _child(stage, sim_path + "/advection",
                 "FlowAdvectionCombustionParams")
    _set(adv, "combustionEnabled", Sdf.ValueTypeNames.Bool, True)
    _set(adv, "enabled", Sdf.ValueTypeNames.Bool, True)
    _set(adv, "downsampleEnabled", Sdf.ValueTypeNames.Bool, True)
    _set(adv, "ignitionTemp", Sdf.ValueTypeNames.Float, 0.05)
    _set(adv, "burnPerTemp", Sdf.ValueTypeNames.Float, 4.0)
    _set(adv, "fuelPerBurn", Sdf.ValueTypeNames.Float, 0.25)
    _set(adv, "smokePerBurn", Sdf.ValueTypeNames.Float, 3.0)
    _set(adv, "tempPerBurn", Sdf.ValueTypeNames.Float, 5.0)
    _set(adv, "coolingRate", Sdf.ValueTypeNames.Float, 1.5)
    _set(adv, "buoyancyPerTemp", Sdf.ValueTypeNames.Float, 1.1)
    # A little lift from smoke itself, not only from heat — otherwise a
    # cold smouldering plume has nothing to raise it and pools at ground
    # level.
    _set(adv, "buoyancyPerSmoke", Sdf.ValueTypeNames.Float, 0.18)
    _set(adv, "buoyancyMaxSmoke", Sdf.ValueTypeNames.Float, 1.0)
    _set(adv, "divergencePerBurn", Sdf.ValueTypeNames.Float, 0.0)
    _set(adv, "gravity", Sdf.ValueTypeNames.Float3,
         Gf.Vec3f(0.0, 0.0, -9.81 * ssf))

    # SMOKE MUST DRIFT, NOT PUFF. NVIDIA's warehouse fire runs the smoke
    # channel at damping 0.30 / fade 0.65 — an aggressive decay tuned for a
    # small indoor fire, where the plume is supposed to die within a few metres
    # of the crate. Applied to a building it kills each puff before it can rise
    # and the result reads as a string of smoke BALLS rather than a column.
    #
    # This is also why the smoke looked better before the param prims were
    # fixed: those settings were silently never applied, so Flow used its own
    # much gentler defaults. Going back to a long-lived plume on purpose.
    channels = {
        "velocity":    (0.01, 0.01, 0.5),
        "divergence":  (0.01, 1.00, 0.5),
        "temperature": (0.00, 0.00, 0.9),
        "fuel":        (0.00, 0.00, 0.9),
        "burn":        (0.00, 0.00, 0.9),
        # 0.04/0.06 kept every puff alive almost indefinitely, which is what
        # sent the columns into the sky. Enough persistence to drift and
        # spread, not enough to climb forever.
        "smoke":       (0.14, 0.26, 0.9),
    }
    for name, (damping, fade, blend) in channels.items():
        ch = _child(stage, sim_path + "/advection/" + name,
                    "FlowAdvectionChannelParams")
        _set(ch, "damping", Sdf.ValueTypeNames.Float, damping)
        _set(ch, "fade", Sdf.ValueTypeNames.Float, fade)
        _set(ch, "secondOrderBlendFactor", Sdf.ValueTypeNames.Float, blend)
        _set(ch, "secondOrderBlendThreshold", Sdf.ValueTypeNames.Float, 0.001)

    vort = _child(stage, sim_path + "/vorticity", "FlowVorticityParams")
    _set(vort, "enabled", Sdf.ValueTypeNames.Bool, True)
    _set(vort, "forceScale", Sdf.ValueTypeNames.Float, 0.6)
    _set(vort, "velocityMask", Sdf.ValueTypeNames.Float, 1.0)
    _set(vort, "velocityLogScale", Sdf.ValueTypeNames.Float, 100.0)

    _set(_child(stage, sim_path + "/pressure", "FlowPressureParams"),
         "enabled", Sdf.ValueTypeNames.Bool, True)

    # Block allocation follows the smoke. Without neighbour allocation a fast
    # front outruns its own grid and the plume clips.
    summ = _child(stage, sim_path + "/summaryAllocate",
                  "FlowSummaryAllocateParams")
    _set(summ, "enabled", Sdf.ValueTypeNames.Bool, True)
    _set(summ, "enableNeighborAllocation", Sdf.ValueTypeNames.Bool, True)
    # Lower, so faint drifting smoke still gets blocks allocated to it
    # instead of being culled the moment it thins out.
    _set(summ, "smokeThreshold", Sdf.ValueTypeNames.Float, 0.004)
    _set(summ, "speedThreshold", Sdf.ValueTypeNames.Float, 1.0)
    _set(summ, "speedThresholdMinSmoke", Sdf.ValueTypeNames.Float, 0.0)

    off = _flow_create(stage, root + "/flowOffscreen", "FlowOffscreen")
    _set(off, "layer", Sdf.ValueTypeNames.Int, int(layer))
    _set(off, "level", Sdf.ValueTypeNames.Int, 0)

    shadow = _child(stage, root + "/flowOffscreen/shadow",
                    "FlowShadowParams")
    _set(shadow, "enabled", Sdf.ValueTypeNames.Bool, True)
    _set(shadow, "attenuation", Sdf.ValueTypeNames.Float, 100.0)
    _set(shadow, "coarsePropagate", Sdf.ValueTypeNames.Bool, True)
    _set(shadow, "lightDirection", Sdf.ValueTypeNames.Float3,
         Gf.Vec3f(-1.0, 1.0, 1.0))
    _set(shadow, "minIntensity", Sdf.ValueTypeNames.Float, 0.02)
    _set(shadow, "numSteps", Sdf.ValueTypeNames.UInt, 16)
    _set(shadow, "stepSizeScale", Sdf.ValueTypeNames.Float, 0.75)

    # Temperature -> colour. The values above 1.0 are deliberate and are what
    # drives bloom into a flame core; clamping them to 1 gives you grey soup.
    cmap = _child(stage, root + "/flowOffscreen/colormap",
                  "FlowRayMarchColormapParams")
    _set(cmap, "resolution", Sdf.ValueTypeNames.UInt, 32)
    _set(cmap, "colorScale", Sdf.ValueTypeNames.Float, 2.5)
    _set(cmap, "xPoints", Sdf.ValueTypeNames.FloatArray,
         [0.0, 0.05, 0.15, 0.6, 0.85, 1.0])
    # The dark end of NVIDIA's ramp is 0.036 grey at half opacity — warehouse
    # smoke, i.e. burning plastic, and it reads as near-black soot. Wood smoke
    # is far lighter and thinner, so the low three stops come from config. The
    # flame stops above stay verbatim: those are what bloom keys off.
    r, g, b = smoke_rgb
    a = float(smoke_opacity)
    _set(cmap, "rgbaPoints", Sdf.ValueTypeNames.Float4Array, [
        Gf.Vec4f(r * 0.55, g * 0.55, b * 0.55, a * 0.02),
        Gf.Vec4f(r, g, b, a),
        Gf.Vec4f(r, g, b, a),
        Gf.Vec4f(1.0, 0.1594, 0.0134, 0.8),
        Gf.Vec4f(13.53, 2.99, 0.12599, 0.8),
        Gf.Vec4f(78.0, 39.0, 6.1, 0.7),
    ])
    _set(cmap, "colorScalePoints", Sdf.ValueTypeNames.FloatArray,
         [1.0] * 6)

    ren = _flow_create(stage, root + "/flowRender", "FlowRender")
    _set(ren, "layer", Sdf.ValueTypeNames.Int, int(layer))
    _set(ren, "level", Sdf.ValueTypeNames.Int, 0)

    # Temperature -> colormap X. The colormap is only orange above x=0.6, and
    # a coarse cell averages combustion over a large volume, so temperature
    # never climbs that far and the fire renders as grey smoke. Pulling
    # colormapXMax down rescales the ramp so the flame band is reachable at
    # the temperatures a neighbourhood-scale grid actually produces.
    rm = _child(stage, root + "/flowRender/rayMarch", "FlowRayMarchParams")
    _set(rm, "colormapXMax", Sdf.ValueTypeNames.Float, float(colormap_x_max))
    _set(rm, "colormapXMin", Sdf.ValueTypeNames.Float, 0.0)
    _set(rm, "attenuation", Sdf.ValueTypeNames.Float, 22.0)
    _set(rm, "colorScale", Sdf.ValueTypeNames.Float, 1.0)
    _set(rm, "shadowFactor", Sdf.ValueTypeNames.Float, 1.0)
    _set(rm, "stepSizeScale", Sdf.ValueTypeNames.Float, 0.75)

    rs = _child(stage, root + "/flowRender/renderSettings",
                "FlowRenderSettingsParams")
    _set(rs, "flowEnabled", Sdf.ValueTypeNames.Bool, True)
    _set(rs, "compositeEnabled", Sdf.ValueTypeNames.Bool, True)
    _set(rs, "enableAutoApply", Sdf.ValueTypeNames.Bool, True)
    _set(rs, "maxBlocks", Sdf.ValueTypeNames.Int, int(max_blocks))
    _set(rs, "pathTracingEnabled", Sdf.ValueTypeNames.Bool, True)
    _set(rs, "rayTracedReflectionsEnabled", Sdf.ValueTypeNames.Bool, True)
    _set(rs, "rayTracedTranslucencyEnabled", Sdf.ValueTypeNames.Bool, True)

    return sim


# --------------------------------------------------------------------------
# Fuel selection
# --------------------------------------------------------------------------

def select_fuels(placements, categories=DEFAULT_FUEL_CATEGORIES,
                 require_prim_path=True):
    """Placements that can burn, as `(x_m, y_m, prim_path)` triples.

    Matched on substring so one entry covers `tree`, `street_tree` and
    `yard_tree` — the suburb names categories by role, not from one enum.

    `prim_path` is stamped onto a placement by `apply_placements`, i.e. only
    once it is on a stage. Emitters need it, so the default insists on it. The
    host-side planners (`tools/fire_png.py`) have positions but no stage, and
    pass `require_prim_path=False` to work from the geometry alone.
    """
    cats = tuple(c.lower() for c in categories)
    out = []
    for p in placements:
        cat = str(p.get("category", "")).lower()
        if not any(c in cat for c in cats):
            continue
        path = p.get("prim_path")
        if require_prim_path and not path:
            continue
        out.append((float(p["x_m"]), float(p["y_m"]), path))
    return out


def thin_by_spacing(fuels, spacing_m, max_count):
    """Decimate on a grid, then cap.

    Grid decimation rather than random sampling because a random subset of a
    dense treeline leaves holes the fire front visibly steps over. One emitter
    per cell keeps the front continuous at whatever density the budget allows.
    """
    if spacing_m <= 0.0:
        kept = list(fuels)
    else:
        seen = {}
        for f in fuels:
            key = (int(math.floor(f[0] / spacing_m)),
                   int(math.floor(f[1] / spacing_m)))
            seen.setdefault(key, f)
        kept = list(seen.values())
    kept.sort(key=lambda f: (f[0], f[1]))
    if max_count and len(kept) > max_count:
        # Even stride, so the cap thins the whole burn rather than truncating
        # one end of it.
        stride = len(kept) / float(max_count)
        kept = [kept[int(i * stride)] for i in range(max_count)]
    return kept


# --------------------------------------------------------------------------
# Spread
# --------------------------------------------------------------------------

# The elliptical spread solver now lives in `disaster.field`, where it also
# backs the `ellipse` damage-field kind — so "when did the front reach here"
# and "how hard was this spot hit" are answered by the same arithmetic.
from disaster.field import arrival_time as _ignition_time  # noqa: E402


def resolve_start_offset(cfg):
    """Seconds of burn already elapsed when the scene opens.

    `start_offset_frac` is the preferred spelling and overrides the absolute
    one: it stays meaningful when the spread rate is retuned, which the raw
    seconds do not.
    """
    frac = cfg.get("start_offset_frac")
    if frac is not None:
        return float(cfg["duration_s"]) * float(frac)
    return float(cfg.get("start_offset_s", 0.0))


def plan_ignition(fuels, cfg, rng):
    """Attach an ignition time to every fuel.

    Returns `[(x_m, y_m, prim_path, t_ignite_s, intensity, will_flame), ...]`
    sorted by time, with anything the front never reaches inside `duration_s`
    dropped. `t_ignite_s` is NEGATIVE for fuel the front had already passed
    when the scene opened — see the offset note below.
    """
    ox, oy = cfg["origin_m"]
    th = math.radians(float(cfg["heading_deg"]))
    cos_t, sin_t = math.cos(th), math.sin(th)

    head = float(cfg["head_mps"])
    flank = float(cfg["flank_mps"])
    back = float(cfg["back_mps"])
    duration = float(cfg["duration_s"])
    offset = resolve_start_offset(cfg)
    jitter = float(cfg["jitter_s"])
    spot_chance = float(cfg["spot_chance"])
    spot_lead = float(cfg["spot_lead_s"])
    flaming_fraction = float(cfg["flaming_fraction"])
    lo_i, hi_i = cfg["intensity_range"]
    lo_i, hi_i = float(lo_i), float(hi_i)

    out = []
    for x, y, path in fuels:
        dx, dy = x - ox, y - oy
        # Rotate into the wind frame: +u downwind, v across.
        u = dx * cos_t + dy * sin_t
        v = -dx * sin_t + dy * cos_t

        t = _ignition_time(u, v, head, flank, back)
        if not math.isfinite(t):
            continue
        if jitter > 0.0:
            t += rng.uniform(-jitter, jitter)
        # An ember landing ahead of the front. Only ever pulls a fuel earlier,
        # so it widens the burn rather than stalling it.
        if spot_chance > 0.0 and rng.random() < spot_chance:
            t -= rng.uniform(0.0, spot_lead)
        if t > duration:
            continue
        # NOT clamped at zero. A fuel the front passed before the scene opened
        # gets a NEGATIVE ignition time, which is what puts it into smoulder or
        # residual on frame one instead of re-igniting everything at t=0 —
        # clamping produced a whole plat lighting up simultaneously, which is
        # the opposite of a post-disaster scene.
        t = t - offset

        # WHAT MAKES IT A POST-DISASTER SCENE rather than a firestorm: only a
        # minority of fuels ever flame. The rest smoke, smoulder and go to
        # residual, which is what most of a burnt-over neighbourhood is doing.
        # Weighted toward the low end so the strong ones stand out.
        inten = lo_i + (hi_i - lo_i) * (rng.random() ** 1.8)
        flames = rng.random() < flaming_fraction
        if flames:
            inten = max(inten, 0.55 + 0.45 * rng.random())
        out.append((x, y, path, t, inten, flames))

    out.sort(key=lambda f: f[3])
    return out


# --------------------------------------------------------------------------
# Emitters
# --------------------------------------------------------------------------

def add_fire_emitter(stage, target_path, emitter_path, bbox_cache, *,
                     layer=FLOW_LAYER, wind=(0.0, 0.0, 0.0), cfg=None,
                     intensity=1.0, rng=None):
    """A `FlowEmitterSphere` sitting low in the target prim's bounding box.

    A SPHERE, not the box the incident extension uses. A box fitted to a whole
    tree injects fuel uniformly through a tall rectangular volume, and at any
    distance that is exactly what it looks like — a burning cuboid. NVIDIA gets
    away with it because their fuel really is a boxy warehouse crate.

    Sized off the horizontal footprint and dropped near the base, because in a
    burnt-over scene the fire is in the litter and the trunk, not hovering in
    the canopy. `intensity` scales the radius as well as the emission, so a
    remnant is a small ember and an active pocket is a proper fire rather than
    the same fire at lower opacity.
    """
    cfg = cfg or DEFAULTS
    target = stage.GetPrimAtPath(target_path)
    if not target.IsValid():
        return None

    box = bbox_cache.ComputeWorldBound(target).ComputeAlignedBox()
    if box.IsEmpty():
        return None
    lo, hi = box.GetMin(), box.GetMax()
    half_x = max(1e-3, (hi[0] - lo[0]) * 0.5)
    half_y = max(1e-3, (hi[1] - lo[1]) * 0.5)

    lo_r, hi_r = cfg.get("emitter_radius_m", [0.7, 2.6])
    radius = 0.42 * (half_x + half_y) * 0.5 * (0.55 + 0.45 * intensity)
    radius = max(float(lo_r), min(float(hi_r), radius))
    if rng is not None:
        radius *= rng.uniform(0.82, 1.22)

    # Low in the volume: base plus a little under one radius, so the sphere
    # straddles the ground rather than floating.
    cz = lo[2] + radius * 0.8
    cx = (hi[0] + lo[0]) * 0.5
    cy = (hi[1] + lo[1]) * 0.5
    if rng is not None:
        cx += rng.uniform(-half_x, half_x) * 0.3
        cy += rng.uniform(-half_y, half_y) * 0.3

    prim = _flow_create(stage, emitter_path, "FlowEmitterSphere")
    if not prim or not prim.IsValid():
        return None

    _set(prim, "layer", Sdf.ValueTypeNames.Int, int(layer))
    _set(prim, "position", Sdf.ValueTypeNames.Float3, Gf.Vec3f(cx, cy, cz))
    _set(prim, "radius", Sdf.ValueTypeNames.Float, float(radius))
    _set(prim, "radiusIsWorldSpace", Sdf.ValueTypeNames.Bool, True)
    _set(prim, "velocity", Sdf.ValueTypeNames.Float3,
         Gf.Vec3f(float(wind[0]), float(wind[1]), float(wind[2])))
    _set(prim, "velocityIsWorldSpace", Sdf.ValueTypeNames.Bool, True)
    _set(prim, "coupleRateFuel", Sdf.ValueTypeNames.Float, 2.0)
    _set(prim, "coupleRateSmoke", Sdf.ValueTypeNames.Float, 2.0)
    _set(prim, "coupleRateTemperature", Sdf.ValueTypeNames.Float, 2.0)
    _set(prim, "coupleRateVelocity", Sdf.ValueTypeNames.Float, 2.0)
    _set(prim, "temperature", Sdf.ValueTypeNames.Float,
         float(cfg.get("temperature", 2.0)) * (0.5 + 0.5 * intensity))
    _set(prim, "fuel", Sdf.ValueTypeNames.Float, 0.0)
    _set(prim, "smoke", Sdf.ValueTypeNames.Float, 0.0)
    _set(prim, "enabled", Sdf.ValueTypeNames.Bool, False)
    return prim


# --------------------------------------------------------------------------
# Emission per visual state
# --------------------------------------------------------------------------

# TEMPERATURE IS THE GLOW, not fuel. The colormap maps temperature to colour,
# so an emitter injecting heat renders bright whatever its fuel is set to —
# which is why a "smoke only" emitter came out as a glowing ball with no
# visible smoke. Zero the temperature AND its couple rate and the same emitter
# renders as plain grey smoke off the bottom of the ramp.
# Rebalanced after the first full scene: a burning house threw a column into
# the stratosphere while the burnt-out ones beside it showed nothing at all.
# The flaming states are cut hard and the cold ones raised, so the difference
# between "still alight" and "finished" is legible without either dominating.
STATE_EMISSION = {
    "flame":    {"fuel": 0.35, "smoke": 0.85, "temperature": 1.6,
                 "couple_temp": 2.0},
    "smoke":    {"fuel": 0.0, "smoke": 1.0, "temperature": 0.0,
                 "couple_temp": 0.0},
    # A COLLAPSED BUILDING SMOKES HARD AND LONG. A burnt-out shell is a deep
    # bed of smouldering timber with no flame above it — visually it is almost
    # all smoke, and these were reading as barely-there wisps next to the
    # houses still alight. The flaming values above are deliberately NOT
    # raised with them; those already look right.
    "smoulder": {"fuel": 0.0, "smoke": 1.60, "temperature": 0.03,
                 "couple_temp": 0.4},
    "residual": {"fuel": 0.0, "smoke": 1.30, "temperature": 0.0,
                 "couple_temp": 0.0},
}


def set_emission(prim, state, scale=1.0):
    """Author one emitter's fuel / smoke / temperature for a visual state."""
    e = STATE_EMISSION.get(state)
    if e is None:
        _set(prim, "enabled", Sdf.ValueTypeNames.Bool, False)
        return False
    _set(prim, "enabled", Sdf.ValueTypeNames.Bool, True)
    _set(prim, "fuel", Sdf.ValueTypeNames.Float, e["fuel"] * scale)
    _set(prim, "smoke", Sdf.ValueTypeNames.Float, e["smoke"] * scale)
    _set(prim, "temperature", Sdf.ValueTypeNames.Float, e["temperature"])
    _set(prim, "coupleRateTemperature", Sdf.ValueTypeNames.Float,
         e["couple_temp"])
    return True


def _plume_z(state, height):
    """How high above a building's base its emitters sit.

    SMOKE HUGS THE GROUND; ONLY FLAME CLIMBS. Every state was being spread up
    to the building's full height, but the smoke states only ever apply to a
    structure that has already collapsed — there is nothing left standing for
    the source to be part-way up, so the plume floated over the rubble. A
    smouldering debris bed emits from the debris, which is at ground level by
    definition.

    Flame keeps the full range, because a burning house really is alight from
    its floor to its roof line.
    """
    h = max(0.0, float(height))
    if state == "flame":
        # Flame climbs, but only as far as there is something to climb. Now
        # that damaged houses collapse outright there is usually nothing —
        # so this is capped at 2 m AND bounded by the building's real settled
        # height, whichever is lower. Uncapped, a house reduced to a low pile
        # still got flames at its original roof line, hanging in the air.
        return (0.30, max(0.7, min(2.0, h)))
    # Smoke never climbs on its own: a smouldering debris bed emits from the
    # debris, which is at ground level by definition.
    return (0.15, max(0.5, min(1.1, h)))


def add_structure_fire(stage, placements, state, root, tag, rng,
                       layer=FLOW_LAYER, wind=(1.2, 0.4, 0.0), max_emitters=6,
                       top_z=None, base_z=0.0, strength=1.0):
    """Fire ON a building rather than inside it.

    One big sphere at a building's centre is a fireball in a box — the flame
    sits in the middle and never touches the geometry. A real burning house
    shows fire at its SURFACES: breaking through the roof line, standing in the
    window and door openings, licking up the outside of the walls.

    So this puts several small emitters at the positions of the building's own
    modules, biased to the roof, which is both where a wildfire takes hold
    first and where it is visible from the air.

    Returns the number of emitters created.
    """
    if state not in STATE_EMISSION or not placements:
        return 0

    # THE FLOOR PLATE IS THE INTERIOR. `modular_house` tiles a house's inside
    # with `Floor_Quart_01` modules, so those — and only those — describe the
    # space within the walls. Averaging ALL modules skews the centre toward
    # whichever side carries more wall, porch or bay, and using wall modules
    # puts every plume on the perimeter by construction, because that is where
    # walls are.
    #
    # Callers that pass debris rather than modules get the same treatment on
    # whatever they passed, which is why this must be given the HOUSE, not the
    # rubble: a fragment cloud's centroid is wherever the pieces landed.
    floors = [p for p in placements
              if str(p.get("category", "")).endswith("_floor")]
    footprint = floors or placements

    xs = [float(p["x_m"]) for p in footprint]
    ys = [float(p["y_m"]) for p in footprint]
    cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
    # Half-extent of the floor plate, plus half a module so a single-tile
    # footprint still has somewhere to put emitters.
    half_x = max(1.2, (max(xs) - min(xs)) * 0.5 + 2.5)
    half_y = max(1.2, (max(ys) - min(ys)) * 0.5 + 2.5)

    # WHERE THE PLUME STARTS HAS TO COME FROM WHAT IS ACTUALLY THERE. Taking
    # it from the roof modules' authored z put emitters at the height the roof
    # USED to be — so a collapsed house had its smoke hanging in mid-air over
    # a pile of rubble. `top_z` lets the caller pass the building's real
    # settled height instead; the authored roof is only the fallback.
    if top_z is not None:
        top = float(top_z)
    else:
        roof_z = [float(p.get("z_m", 0.0)) for p in placements
                  if str(p.get("category", "")).endswith(("roof", "bay_roof"))]
        top = max(roof_z) if roof_z else 2.5

    picks = []
    for _ in range(max_emitters):
        # `half_x` IS already half the footprint, so the `* 2.0` here made the
        # radius up to 1.1 half-widths and pushed plumes back out through the
        # walls — which is why the smoke kept hugging the edges however much
        # the factor was reduced. 0.40 of a half-width sits well inside.
        r = 0.40 * math.sqrt(rng.random())
        a = rng.uniform(0.0, 2.0 * math.pi)
        picks.append({
            "x_m": cx + math.cos(a) * r * half_x,
            "y_m": cy + math.sin(a) * r * half_y,
            "z_m": float(base_z) + rng.uniform(*_plume_z(state, top - base_z)),
        })

    UsdGeom.Xform.Define(stage, Sdf.Path(root + "/emitters"))
    n = 0
    for i, p in enumerate(picks):
        path = "{0}/emitters/{1}_{2:02d}".format(root, tag, i)
        prim = _flow_create(stage, path, "FlowEmitterSphere")
        if not prim or not prim.IsValid():
            continue
        # Sit it just proud of the module so the plume wraps the geometry
        # instead of being buried inside it.
        _set(prim, "layer", Sdf.ValueTypeNames.Int, int(layer))
        _set(prim, "position", Sdf.ValueTypeNames.Float3, Gf.Vec3f(
            float(p["x_m"]), float(p["y_m"]), float(p["z_m"])))
        _set(prim, "radius", Sdf.ValueTypeNames.Float, rng.uniform(0.9, 1.7))
        _set(prim, "radiusIsWorldSpace", Sdf.ValueTypeNames.Bool, True)
        _set(prim, "coupleRateFuel", Sdf.ValueTypeNames.Float, 2.0)
        _set(prim, "coupleRateSmoke", Sdf.ValueTypeNames.Float, 2.0)
        _set(prim, "velocity", Sdf.ValueTypeNames.Float3, Gf.Vec3f(*wind))
        _set(prim, "velocityIsWorldSpace", Sdf.ValueTypeNames.Bool, True)
        set_emission(prim, state,
                     scale=float(strength) * rng.uniform(0.75, 1.15))
        n += 1
    return n


# --------------------------------------------------------------------------
# Burn state over time
# --------------------------------------------------------------------------

UNBURNT, IGNITING, FLAMING, SMOULDERING, RESIDUAL = range(5)

_STATE_NAMES = {
    UNBURNT: "unburnt", IGNITING: "igniting", FLAMING: "flaming",
    SMOULDERING: "smouldering", RESIDUAL: "residual",
}


class WildfireDriver:
    """Advances every emitter through its burn states as the timeline runs.

    POST-DISASTER, NOT A FIREFRONT. The terminal state is RESIDUAL, not "off":
    ground that the fire has finished with keeps issuing a thin drift of smoke
    indefinitely, which is what a burnt-over neighbourhood actually looks like
    an hour later. Switching burnt fuels off entirely — the previous behaviour
    — left most of the map completely clean and put every visible effect into
    a handful of roaring emitters.

    Intensity is per fuel and seeded. Only `flaming_fraction` of fuels ever
    reach FLAMING at all; the rest go straight from IGNITING to SMOULDERING, so
    the scene reads as mostly remnants with a few live pockets rather than as
    one uniform fire replicated N times.

    Attributes are written only on a state CHANGE. Keep a reference to this —
    dropping it unsubscribes the timeline callback and the burn freezes.
    """

    def __init__(self, stage, entries, cfg):
        self.stage = stage
        self.cfg = cfg
        # [prim, t_ignite, intensity, will_flame, state]
        self.entries = [[prim, t, inten, flames, UNBURNT]
                        for prim, t, inten, flames in entries]
        self.ignition_s = float(cfg["ignition_s"])
        self.flame_s = float(cfg["flame_s"])
        self.smoulder_s = float(cfg["smoulder_s"])
        self._sub = None
        self._labelled = set()

        import omni.timeline
        self.timeline = omni.timeline.get_timeline_interface()
        self._t0 = self.timeline.get_current_time()
        self._sub = (self.timeline.get_timeline_event_stream()
                     .create_subscription_to_pop(
                         self._on_timeline_event, name="WildfireDriver"))
        self.update(0.0)

    def state_at(self, elapsed, t_ignite, will_flame=True):
        d = elapsed - t_ignite
        if d < 0.0:
            return UNBURNT
        if d < self.ignition_s:
            return IGNITING
        flame = self.flame_s if will_flame else 0.0
        if d < self.ignition_s + flame:
            return FLAMING
        if d < self.ignition_s + flame + self.smoulder_s:
            return SMOULDERING
        return RESIDUAL

    def _apply(self, prim, state, intensity):
        cfg = self.cfg
        k = float(intensity)
        if state == UNBURNT:
            _set(prim, "enabled", Sdf.ValueTypeNames.Bool, False)
            _set(prim, "fuel", Sdf.ValueTypeNames.Float, 0.0)
            _set(prim, "smoke", Sdf.ValueTypeNames.Float, 0.0)
            return
        # Delegated so the non-flaming states zero TEMPERATURE as well as
        # fuel. Leaving heat on is what made smoke-only emitters render as
        # glowing balls — see STATE_EMISSION.
        vis = {IGNITING: "smoke", FLAMING: "flame",
               SMOULDERING: "smoulder"}.get(state, "residual")
        set_emission(prim, vis, scale=k)

    def update(self, elapsed):
        cfg = self.cfg
        label = bool(cfg.get("semantics")) and cfg.get("semantic_class")
        for entry in self.entries:
            prim, t_ig, inten, flames, prev = entry
            state = self.state_at(elapsed, t_ig, flames)
            if state == prev:
                continue
            entry[4] = state
            self._apply(prim, state, inten)
            if label and state in (IGNITING, FLAMING):
                self._label(prim, cfg["semantic_class"])

    def _label(self, emitter_prim, cls):
        """Tag the emitter as burning for synthetic-data ground truth.

        Best effort: Flow's volume is a render-time composite that Replicator's
        segmentation annotators do not see in 5.1, so this labels the emitter
        region rather than the visible flame. The FlowUSD writer that captures
        the volume itself is a 6.0 feature.
        """
        path = str(emitter_prim.GetPath())
        if path in self._labelled:
            return
        self._labelled.add(path)
        try:
            from isaacsim.core.utils.semantics import add_labels
            add_labels(emitter_prim, labels=[cls], instance_name="class")
        except Exception:
            try:
                from isaacsim.core.utils.semantics import add_update_semantics
                add_update_semantics(emitter_prim, cls)
            except Exception:
                pass

    def burn_report(self, elapsed):
        """Per-prim burn state at `elapsed`, for writing dataset labels."""
        out = {}
        for prim, t_ig, _inten, flames, _st in self.entries:
            out[str(prim.GetPath())] = _STATE_NAMES[
                self.state_at(elapsed, t_ig, flames)]
        return out

    def _on_timeline_event(self, event):
        import omni.timeline
        if event.type != omni.timeline.TimelineEventType.CURRENT_TIME_TICKED.value:
            return
        self.update(self.timeline.get_current_time() - self._t0)

    def destroy(self):
        self._sub = None


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------

def apply_wildfire(stage, placements, cfg=None, *, scene_scale_factor=1.0,
                   root=FLOW_ROOT, verbose=True):
    """Burn a scene. Returns a `WildfireDriver`, or None if disabled.

    `placements` is what `generate_suburb_on_stage` / `build_city` return —
    every entry already carries the `prim_path` this needs.

    THE RETURN VALUE MUST BE KEPT ALIVE. It owns the timeline subscription
    that advances the burn; drop it and the fire stops where it stands.
    """
    import random

    merged = dict(DEFAULTS)
    merged.update(cfg or {})
    if not merged.get("enabled", True):
        return None

    ssf = float(scene_scale_factor)
    rng = random.Random(merged["seed"])

    fuels = select_fuels(placements, merged["fuel_categories"])
    if not fuels:
        if verbose:
            cats = ", ".join(merged["fuel_categories"])
            loose = select_fuels(placements, merged["fuel_categories"],
                                 require_prim_path=False)
            if loose:
                print("[fire] {0} fuel placements matched but none carry a "
                      "prim_path — apply_wildfire must run AFTER "
                      "apply_placements".format(len(loose)))
            else:
                print("[fire] no fuel found — no placement category matched "
                      "{" + cats + "}")
        return None

    kept = thin_by_spacing(fuels, float(merged["emitter_spacing_m"]),
                           int(merged["max_emitters"]))
    planned = plan_ignition(kept, merged, rng)
    if not planned:
        if verbose:
            print("[fire] the front reaches no fuel — check origin_m, "
                  "heading_deg and duration_s")
        return None

    setup_flow_stack(stage, layer=FLOW_LAYER,
                     density_cell_size_m=float(merged["density_cell_size_m"]),
                     max_blocks=int(merged["max_blocks"]),
                     colormap_x_max=float(merged["colormap_x_max"]),
                     smoke_rgb=tuple(merged["smoke_rgb"]),
                     smoke_opacity=float(merged["smoke_opacity"]),
                     scene_scale_factor=ssf, root=root)

    emitters_root = root + "/emitters"
    UsdGeom.Xform.Define(stage, Sdf.Path(emitters_root))

    th = math.radians(float(merged["heading_deg"]))
    wind_speed = float(merged["wind_mps"]) * ssf
    wind = (math.cos(th) * wind_speed, math.sin(th) * wind_speed, 0.0)

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                   [UsdGeom.Tokens.default_],
                                   useExtentsHint=True)

    entries = []
    for i, (_x, _y, path, t_ig, inten, flames) in enumerate(planned):
        prim = add_fire_emitter(
            stage, path, "{0}/fire_{1:04d}".format(emitters_root, i),
            bbox_cache, layer=FLOW_LAYER, wind=wind, cfg=merged,
            intensity=inten, rng=rng)
        if prim is not None:
            entries.append((prim, t_ig, inten, flames))

    if not entries:
        if verbose:
            print("[fire] every fuel prim had an empty bounding box — "
                  "nothing to emit from")
        return None

    if verbose:
        last = planned[-1][3]
        n_flame = sum(1 for e in entries if e[3])
        print("[fire] {0} fuels -> {1} emitters ({2} ever flame, {3} smoulder "
              "only), front arrives over {4:.0f}s (cell {5} m, maxBlocks {6})"
              .format(len(fuels), len(entries), n_flame,
                      len(entries) - n_flame, last,
                      merged["density_cell_size_m"], merged["max_blocks"]))

    return WildfireDriver(stage, entries, merged)
