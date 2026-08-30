"""soot_plume — the soot on a façade grows OUT OF the openings the fire vented
through, and the flames come out of the same openings.

THE ONE IDEA
------------
A building on fire has a DISCRETE set of fire EVENTS: compartments that vented
through a contiguous run of openings on one elevation of one storey. Some of
them are alight NOW (those get NVIDIA Flow flames), some are smouldering
(smoke only), and some burnt out long ago and show NOTHING volumetric at all —
but every one of them left its mark on the wall above the openings it vented
through, and the soot on the building is the SUM of those marks.

`plan_events` draws that set ONCE per building. `urban_fire.r_flames` reads it
to decide where the emitters go, and `skin` reads it to decide where the soot
goes, so the flames and the stains agree by construction. The previous design
(`facade_bake.building_skin` + `urban_fire._flame_runs`) drew the two
independently — the skin scattered its own random plume sources across a
storey band while the flames picked their own openings — and the result was
soot smeared where no fire had ever vented and clean wall over windows with
flame coming out of them ("the pattern looks completely wrong", user
2026-08-30). Nothing from that design is reused here.

THE MODEL, and where every piece of it comes from
-------------------------------------------------
Per event, on the wall plane (u along the elevation, z up), in this order:

 1. HEAT RELEASE, ventilation-controlled:  Q = 1.5 A_v sqrt(h_eq)  [MW]
    (Kawagoe's inflow 0.5 A_v sqrt(h) kg/s of air x ~3 MJ per kg of air;
    EN 1991-1-2 Annex B eq. B.4 gives the same order for an ordinary opening
    factor). `intensity` scales it for a fuel-lean compartment.

 2. THE EXTERNAL FLAME — EN 1991-1-2:2002 Annex B, no forced draught:
        L_L = max(0, 1.9 (Q/w_t)^(2/3) - h_eq)     height above the head  (B.7)
        flame width = w_t, the run of openings                        (B.4.1(4))
        flame depth = 2/3 h_eq                                        (B.4.1(5))
        L_H = h_eq/3                 if h_eq <= 1.25 w_t  (ATTACHED)  (B.8)
            = 0.3 h_eq (h_eq/w_t)^0.54   otherwise       (PROJECTING) (B.9)
            = 0.6 h_eq (L_L/h_eq)^(1/3)  no wall above the window     (B.11)
        L_f = L_L + h_eq/2                                            (B.12)
        T_w = 520 / (1 - 0.4725 L_f w_t/Q) + T_0                      (B.14)
        T_z = (T_w - T_0)(1 - 0.4725 L_x w_t/Q) + T_0   along the axis (B.15)
    B.14/B.15 together put the flame TIP at T_0 + 520 K — Law's 540 C tip
    criterion, which is the same physics NFPA 921's "damage height tracks the
    visible flame height" rests on (Madrzykowski & Fleischmann 2012, within
    5 %; Gorbett et al., Fire Science Reviews 4:4, 2015).

 3. THE PLUME ABOVE THE TIP — Heskestad (SFPE Handbook, "Fire Plumes, Flame
    Height and Air Entrainment"): Gaussian across, half-width
    b = 0.12 (T_0/T_inf)^(1/2) (z - z_v) growing linearly with height from
    the virtual origin, axis temperature rise decaying as (z - z_v)^(-n).
    n = 5/3 for a free axisymmetric plume; Yokoi (1960, re-examined by
    Himoto et al. 2012) measured ~-1 in the wall-attached middle regime of a
    WINDOW plume. `PLUME_DECAY` sits between. THIS IS THE V-PATTERN: a plume
    that widens as it rises, cut by the wall (NFPA 921 §6.3.2.3, "plume-
    generated patterns"). The apex is at the opening because that is where
    the plume is narrowest, not because a shape was drawn there.

 4. DEPOSITION — thermophoresis, the mechanism Riahi & Beyler validated
    gravimetrically on a gypsum wall (Fire Safety Science 10:641-654, 2011):
        V_th = 0.55 nu/T . dT/dx             (their eq. 2; Talbot's coefficient)
        dT/dx = h_c (T_gas - T_wall) / k_air (their eq. 3)
        m = ∫ V_th C_s dt  ~=  V_th C_s tau  (their eq. 5, steady venting)
    C_s, the soot mass concentration in the plume, is tied to the temperature
    rise by the same dilution that sets it: C_s = Y_s rho c_p dT/(chi dH_c).
    NIST's laminar-channel measurements (Ko et al., "Quantifying
    Thermophoretic Deposition of Soot on Surfaces") give V_th ~ 0.45 mm/s at
    2.0e4 K/m — the formula above reproduces that with nu at the film
    temperature, which is the check that these constants are in the right
    universe.

 5. DARKNESS — Beer-Lambert through the deposit with the specific extinction
    of flame soot, sigma_s = 8.7 m^2/g (Mulholland & Croarkin 2000):
        alpha = 1 - exp(-sigma_s m)
    Deposits from overlapping events ADD in `m` BEFORE the exponential, so two
    plumes crossing come out darker than either alone, and a storey whose
    every window vented for an hour saturates to black. The "almost
    completely soot covered" state of a burnt-out block is the arithmetic's
    own answer for many long events, not a special case.

 6. WHAT IS NOT PHYSICS: three noise terms, each tied to a real mechanism and
    each MULTIPLICATIVE on `m`, so none of them can put soot where no plume
    reached — MEANDER (the plume axis wanders with height, so no two Vs are
    the same and neither edge of one is straight), a deposition MOTTLE
    (Riahi: "the physical properties of the surface change the temperature
    gradient" — a wall is not one substrate), and vertical STREAKING (the
    wall plume's own streamwise structure, and water runs after the fire).
    A fourth, on TONE rather than amount, keeps a saturated deposit from
    being one flat colour: soot thickness and ash content vary, so the
    colour of a black wall still varies (SOOT_DARK..SOOT_ASH).

WHAT IS DELIBERATELY LEFT OUT. Clean burn (soot oxidised off a surface hotter
than ~450-500 C, the Heskestad-flame-height clean zone in Riahi's own
turbulent tests) — that is an INTERIOR phenomenon on thin gypsum. An exterior
masonry reveal is massive, never reaches it in the time a compartment vents,
and is re-sooted by the cooling phase anyway; every photograph of a vented
window shows a black head, not a clean one. Forced draught (Annex B.4.2) —
windows on opposite walls of one compartment; this kit's compartments are not
modelled that finely and the no-draught branch is the conservative one.

THE SKIN, AND HOW IT REACHES A PRIM
-----------------------------------
`skin` rasterises every event onto ONE canvas unwrapped around the burning
mass's perimeter, S | E | N | W (u = 0 at the SW corner, running counter-
clockwise seen from above — the order `urban_fire._wall_run_frame` documents
each side's pieces run in). A plume near a corner therefore carries ROUND it
on the canvas exactly as far as its own Gaussian says, which is the physically
right amount, and the last seam (W back to S) closes because the lateral
distance is taken modulo the perimeter and the noise is spectral (periodic).

Each placed module then takes its OWN crop of that canvas (`piece_crop`,
addressed by the module's measured span along its side and its own z range)
and gets it merged into a copy of its OWN base-colour map (`merge_piece`),
bound on its own UVs. Continuity across module boundaries is automatic: every
crop comes out of one image and neighbours take neighbouring crops. The one
assumption in that step — that a module's UVs address its base map corner to
corner with image row 0 at the TOP of the face — is the same one every
previous per-prim bake in this dataset made and the previews it produced
landed the soot at the right storey; if a render ever shows the stain UNDER
the sills instead of over the heads, this is the line to flip.

Row 0 of every array here is the TOP of the wall (image convention).
"""

import hashlib
import math
import os

import numpy as np

# ---------------------------------------------------------------------------
# Physical constants — every one of them has a citation in the module docstring
# ---------------------------------------------------------------------------
T_AMB = 293.15          # K, ambient
SIGMA_S = 8.7           # m2/g, specific extinction of flame soot (Mulholland & Croarkin)
K_TH = 0.55             # Talbot thermophoretic coefficient (Riahi & Beyler eq. 2)
Y_SOOT = 0.10           # kg soot per kg fuel — post-flashover compartment (SFPE: 0.05-0.2)
DH_C = 20.0e3           # kJ/kg effective heat of combustion of building contents
CP = 1.0                # kJ/kg/K
CHI_CONV = 0.7          # convective fraction of Q
H_CONV = 12.0           # W/m2/K, façade wall in a buoyant plume (EC1 B.17 order)
WALL_WARM = 0.30        # a massive wall warms to this fraction of the gas rise
Q_PER_AV = 1.5          # MW per m2.m^0.5 — Kawagoe / EC1 B.4 order
FLAME_TIP_DT = 520.0    # K, EC1 B.14's tip (Law's 540 C)
PLUME_DECAY = 5.0 / 3.0  # axis dT ~ (z - z_v)^-n above the tip: Heskestad's free plume; Yokoi's wall regime is ~1
SPREAD = 0.19           # Heskestad b/(z - z_v) = 0.12 (T0/Tinf)^0.5 at Tinf ~ 2.6 T0 (~11 deg half-angle)
ROOT_WIDEN = 1.6        # stain width at the head / opening width (Madrzykowski & Fleischmann: damage up to 1.5x the fuel width)
NEUTRAL_FRAC = 0.15     # gas rise AT the neutral plane, as a share of T_w - T_0 — the
                        # value the in-opening ramp starts from and the downwash decays
                        # from, so there is no step there (pre-flashover smoke, hose)
DOWNWASH_REACH = 0.9    # ... decaying over this many h_eq below the neutral plane
NEUTRAL_WOBBLE = 0.30   # per-column wander of the neutral plane, in h_eq — what
                        # keeps the stain's lower edge from being a ruled line

# How long each ladder state has been venting, in seconds — the `tau` of
# Riahi & Beyler eq. 5. These are the design knobs for HOW MUCH wall a fire
# stains; the shape is the physics'. `urban_fire.LADDER`'s own `smoke_stain`
# `heavy` (0.45 / 1.0 / 1.25 / 1.4 / 1.4 / 2.0 up the ladder) MULTIPLIES
# them at rasterisation (`skin(duration_scale=heavy)`), so the effective
# venting is F1 ~4 min, F2 12, F3 25, F4 ~60, F5 ~90, F6 120.
DURATION_S = {"F1": 540.0, "F2": 720.0, "F3": 1200.0, "F4": 2600.0,
              "F5": 3900.0, "F6": 3600.0}
# One global gain on the deposited mass — the EFFECTIVE fraction of the
# steady design venting the literature constants describe. At 1.0 (EC1's
# T_w held for the whole `tau`, the plume axis on the wall) a 12-minute
# compartment fire blackened 10 m of wall above its window to alpha 0.9,
# which no photograph of a vented window shows: Annex B's flame temperature
# is a peak DESIGN value, the compartment holds it for a fraction of its
# venting time, and a window plume leans out from the wall by its own
# horizontal projection. 0.10 puts the reference single window (1 x 2 m,
# 12 min) at alpha ~1.0 on the reveal, ~0.55 four metres above the head and
# ~0.15 ten metres up, and lets an hour-long burnt-out storey saturate two
# to three storeys of wall — the calibration target, measured with
# `tools/soot_png.py`. It is the one knob to turn for "darker"/"lighter";
# never a physical constant.
DEPOSIT_GAIN = 0.10

# Noise — see docstring item 6. All multiplicative on the deposit.
MEANDER_AMP = 0.35      # axis wander, as a share of the local half-width
MEANDER_LEN_M = 3.0     # its vertical correlation length
MOTTLE = 0.30           # +- share on the deposit, ~1.5 m patches
MOTTLE_M = 1.5
STREAK = 0.28           # +- share on the deposit, vertically stretched
STREAK_STRETCH = 14.0
STREAK_M = 0.6          # lick width
TONE_NOISE_M = 0.8      # the colour of a saturated deposit varies at this scale

# The canvas
PX_PER_M = 40.0
MAX_W_PX, MAX_H_PX = 4096, 2048
MIN_PX = 48
PIECE_MAX_PX = 1024     # a merged per-prim map is worked at up to this size

# Colour, LINEAR albedo (see `urban_fire._FLAT` for why not screen grey).
SOOT_DARK = (0.030, 0.027, 0.025)
SOOT_ASH = (0.110, 0.105, 0.098)
ASH_LEVEL = {"scorch": 0.20, "char": 0.25, "ash": 0.55}
DESAT = 0.65            # how much a sooted surface loses its own colour

# Flames. `FLAME_BUDGET_OPENINGS` is how many openings may carry Flow flame
# sources on one building — the same 9 `urban_fire.r_flames` has always
# capped at (x FLAME_PER_OPENING sources each). Events that would take the
# count past it are planned as burnt OUT instead, which is what a fire that
# has moved on looks like anyway.
FLAME_BUDGET_OPENINGS = 9
SMOULDER_EVENTS_MAX = 3
RUN_LEN = (2, 4)        # openings per compartment event (a run of neighbours)

_HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(os.path.dirname(_HERE), "assets", "materials", "soot")
_RECIPE = "sootplume_v1"

_SIDES = ("S", "E", "N", "W")


# ---------------------------------------------------------------------------
# Air properties at a temperature (K) — power-law fits to the standard tables
# ---------------------------------------------------------------------------
def _nu(T):
    return 1.5e-5 * (T / T_AMB) ** 1.7


def _k_air(T):
    return 0.026 * (T / T_AMB) ** 0.8


def _rho(T):
    return 1.2 * T_AMB / T


# ---------------------------------------------------------------------------
# Geometry on the burning mass: sides, spans, the perimeter
# ---------------------------------------------------------------------------
def side_length(m, side):
    return float(m["W"]) if side in ("S", "N") else float(m["D"])


def perimeter_offsets(m):
    """Metres along the unwrapped perimeter at which each side STARTS."""
    W, D = float(m["W"]), float(m["D"])
    return {"S": 0.0, "E": W, "N": W + D, "W": 2.0 * W + D}, 2.0 * (W + D)


def side_u(m, side, wx, wy):
    """Distance along `side` (from its start corner, in the counter-clockwise
    traversal S->E->N->W) of the world point (wx, wy)."""
    from . import quake_flow as qf

    lx, ly = qf._to_local(m, wx, wy)
    W, D = float(m["W"]), float(m["D"])
    return {"S": lx + W / 2.0, "E": ly + D / 2.0,
            "N": W / 2.0 - lx, "W": D / 2.0 - ly}[side]


def piece_span(e, fe, m, side):
    """(u0, u1) of one placed module along its side, metres. Both ends are
    projected and ordered, so it does not matter which way the piece's own
    local +X runs relative to the side."""
    from . import quake_flow as qf

    ox, oy, yaw, width = fe[0], fe[1], fe[2], fe[3]
    ca, sa = math.cos(yaw), math.sin(yaw)
    if fe[6]:   # `dw` frame pivots at its centre
        p0 = (ox - ca * width / 2.0, oy - sa * width / 2.0)
        p1 = (ox + ca * width / 2.0, oy + sa * width / 2.0)
    else:
        p0 = (ox, oy)
        p1 = (ox + ca * width, oy + sa * width)
    a = side_u(m, side, p0[0], p0[1])
    b = side_u(m, side, p1[0], p1[1])
    return (min(a, b), max(a, b))


def opening_span(op, m, side):
    """(u0, u1, z_sill, z_head) of one opening record (the REVEAL, `h*`, when
    the table has it) along its side."""
    from . import quake_flow as qf

    fr = op["fr"]
    ua = op.get("hua", op["ua"])
    ub = op.get("hub", op["ub"])
    va = op.get("hva", op["va"])
    vb = op.get("hvb", op["vb"])
    x0, y0 = qf._face_xy(fr, ua)
    x1, y1 = qf._face_xy(fr, ub)
    a = side_u(m, side, x0, y0)
    b = side_u(m, side, x1, y1)
    return (min(a, b), max(a, b), float(min(va, vb)), float(max(va, vb)))


def parapet_height(m):
    spec = m.get("spec") or {}
    return float(sum(float(b.get("h", 0.0)) for b in spec.get("bands", [])
                     if b.get("parapet")))


# ---------------------------------------------------------------------------
# Openings per (mass, side, storey) — measured where the kit has a table,
# synthesised from the wall modules where it does not
# ---------------------------------------------------------------------------
def _reference_window(ctx, mtag):
    """(width, sill offset, head offset) of a typical measured window on
    this building — the median over every module of the burning mass that
    HAS a table — or None when the family has none (the glass towers)."""
    from . import quake_flow as qf

    key = "_soot_ref_window"
    if key in ctx:
        return ctx[key]
    ws, sills, heads = [], [], []
    for e in qf._els(ctx, mass=mtag, role=("wall", "corner")):
        for r in qf._G2_WIN_FACES.get(e["name"], ()):
            u0, u1, v0, v1 = (r[5], r[6], r[7], r[8]) if len(r) >= 9 else r[:4]
            ws.append(abs(u1 - u0))
            sills.append(min(v0, v1))
            heads.append(max(v0, v1))
    ref = None
    if ws:
        ref = (float(np.median(ws)), float(np.median(sills)),
               float(np.median(heads)))
    ctx[key] = ref
    return ref


def _virtual_openings(ctx, mtag, side, storey, only=None):
    """Opening-shaped records for wall modules with NO measured glazing
    table. Two cases:

      * a PUNCHED-WINDOW family where some modules are measured and others
        are not (`SM_MBuilding04_Facade_B` beside `_Facade_A`): the unmeasured
        module gets windows of the family's own median width, sill and head,
        at the pitch its width allows — because a façade module in such a
        family has windows in its art whether or not anyone measured them,
        and a burnt-out storey with clean bays where those modules stand is
        wrong (commercial_mid F6, soot_png 2026-08-30). Corner modules are
        left alone: they are solid returns in every family measured so far.
      * a family with NO table at all (the glass towers paint their mullions
        into the map): one spandrel vent per module, the middle 60 % of its
        width and the upper 55 % of its height.

    `only` restricts the walk to those element names.
    """
    from . import quake_flow as qf

    out = []
    m = ctx["info"]["masses"][mtag]
    ref = _reference_window(ctx, mtag)
    for e in qf._els(ctx, mass=mtag, role=("wall", "corner"), side=side,
                     storey=storey):
        if only is not None and e["name"] not in only:
            continue
        fr = qf._piece_frame(e)
        if fr is None:
            continue
        w, hh = fr[3], fr[4]
        if ref is not None:
            if e["role"] == "corner" or "corner" in e["name"].lower():
                continue
            rw, rs, rh = ref
            rs = min(rs, hh - 0.6)
            rh = max(rs + 0.5, min(rh, hh - 0.15))
            pitch = max(rw * 1.6, 1.2)
            k = max(1, int(w / pitch))
            for i in range(k):
                uc = w * (i + 0.5) / k
                rects = [(uc - rw / 2.0, uc + rw / 2.0)]
                for ua, ub in rects:
                    out.append({"fr": fr, "ua": ua, "ub": ub,
                                "va": e["z"] + rs, "vb": e["z"] + rh,
                                "out": -0.2, "e": e, "m": m, "side": side,
                                "storey": storey, "mass": mtag,
                                "virtual": True})
            continue
        # 0..w along the piece for BOTH frame kinds: `_face_xy` re-centres
        # a `dw` frame itself
        ua, ub = w * 0.20, w * 0.80
        out.append({"fr": fr, "ua": ua, "ub": ub,
                    "va": e["z"] + 0.30 * hh, "vb": e["z"] + 0.85 * hh,
                    "out": -0.05, "e": e, "m": m, "side": side,
                    "storey": storey, "mass": mtag, "virtual": True})
    return out


def openings(ctx, mtag, side, storey):
    """Every opening on (mass, side, storey), sorted along the side, each
    tagged with its own `span` = (u0, u1, z_sill, z_head)."""
    from . import quake_flow as qf

    fn = ctx.get("soot_openings")
    if fn is not None:
        # a caller with its own window table (a merged asset measured before
        # slicing, `disaster.gac_fire`) supplies records already carrying
        # `span` = (u0, u1, z_sill, z_head) along this side
        ops = [o for o in (fn(ctx, mtag, side, storey) or []) if o.get("span")]
        ops.sort(key=lambda o: o["span"][0])
        return ops
    m = ctx["info"]["masses"][mtag]
    ops = (list(qf._g2_openings(ctx, mass=mtag, sides=(side,),
                                storeys=(storey,)))
           + list(qf._g_shop_openings(ctx, mass=mtag, sides=(side,),
                                      storeys=(storey,))))
    measured = set(id(o["e"]) for o in ops)
    unmeasured = set(e["name"] for e in qf._els(
        ctx, mass=mtag, role=("wall", "corner"), side=side, storey=storey)
        if id(e) not in measured)
    if unmeasured:
        ops += _virtual_openings(ctx, mtag, side, storey, only=unmeasured)
    L = side_length(m, side)
    kept = []
    for op in ops:
        u0, u1, zs, zh = opening_span(op, m, side)
        # A corner or return module is assigned to the side whose wall line
        # it is nearest, and its openings face the OTHER way: projected
        # onto this side they collapse to a point, at or past the corner.
        # One such vent (u 26.0..26.0 on a 25 m side, highrise_step F3)
        # made a 0.3 m "window" with its own plume off the end of the wall.
        if (u1 - u0) < 0.25 or u1 < -0.3 or u0 > L + 0.3:
            continue
        op["span"] = (u0, u1, zs, zh)
        kept.append(op)
    kept.sort(key=lambda o: o["span"][0])
    return kept


# ---------------------------------------------------------------------------
# The events
# ---------------------------------------------------------------------------
def _make_event(ctx, ops, side, storey, state, tau, intensity, drift, eid):
    m = ctx["info"]["masses"][ctx["fire"]["mass"]]
    n = ctx["fire"]["n_storeys"]
    u0 = min(o["span"][0] for o in ops)
    u1 = max(o["span"][1] for o in ops)
    z_sill = min(o["span"][2] for o in ops)
    z_head = max(o["span"][3] for o in ops)
    w_t = sum(o["span"][1] - o["span"][0] for o in ops)
    A_v = sum((o["span"][1] - o["span"][0]) * (o["span"][3] - o["span"][2])
              for o in ops)
    h_eq = A_v / max(1e-6, w_t)
    top = storey >= n - 1 and parapet_height(m) < 0.6
    return {"id": int(eid), "mass": ctx["fire"]["mass"], "side": side,
            "storey": int(storey), "ops": list(ops), "u0": float(u0),
            "u1": float(u1), "z_sill": float(z_sill), "z_head": float(z_head),
            "w_t": float(max(0.3, w_t)), "h_eq": float(max(0.3, h_eq)),
            "A_v": float(max(0.1, A_v)), "state": state, "tau": float(tau),
            "intensity": float(intensity), "drift": float(drift),
            "top": bool(top)}


def _runs(ops, rng, share, run_len=RUN_LEN):
    """Split a storey's openings into contiguous compartment runs covering
    about `share` of them. `share` >= 1 covers every opening."""
    n = len(ops)
    if n == 0:
        return []
    if share >= 0.999:
        out, i = [], 0
        while i < n:
            ln = rng.randint(run_len[0], run_len[1])
            out.append(ops[i:i + ln])
            i += ln
        return out
    want = max(1, int(round(n * share)))
    out, used = [], set()
    tries = 0
    while sum(len(r) for r in out) < want and tries < 20:
        tries += 1
        ln = min(n, rng.randint(run_len[0], run_len[1]))
        start = rng.randint(0, max(0, n - ln))
        idx = [k for k in range(start, start + ln) if k not in used]
        if not idx:
            continue
        # keep only the contiguous block that starts at the first free index
        block = [idx[0]]
        for k in idx[1:]:
            if k == block[-1] + 1:
                block.append(k)
        out.append([ops[k] for k in block])
        used.update(block)
    return out


def event_seed(ctx):
    """A stable integer for this building's fire — from its tag, level,
    origin, sides, mass and position, never from the recipes' rng.

    WHY NOT `ctx["rng"]`. Every recipe in `urban_fire.LADDER` draws from the
    one `random.Random` the launcher hands in, in ladder order. Drawing the
    events from it too — before any recipe runs — shifted every draw that
    followed, so the SAME seed produced a different `fire_collapse`,
    `floor_burnthrough` and debris scatter than the run before ("for
    building 5 it looks like the roof is floating even though previously
    it wasn't", user 2026-08-30, dw_terrace F5 on UF_SEED=7). The events
    and the skin's noise now come from this hash, and the recipes see
    exactly the sequence they always did."""
    f = ctx["fire"]
    info = ctx["info"]
    key = "{0}|{1}|{2}|{3}|{4}|{5:.2f}|{6:.2f}".format(
        ctx.get("tag", ""), f.get("level"), f.get("origin"),
        "".join(f.get("sides", ())), f.get("mass"),
        float(info.get("x", 0.0)), float(info.get("y", 0.0)))
    return int(hashlib.md5(key.encode("utf-8")).hexdigest()[:12], 16)


def event_rng(ctx):
    import random as _random
    return _random.Random(event_seed(ctx))


def plan_events(ctx, severity, rng=None, max_active=FLAME_BUDGET_OPENINGS,
                heavy=1.0):
    """The building's fire events, from its `ctx["fire"]` plan.

    `severity(ctx, storey, mass)` is `urban_fire._severity`, passed in
    rather than imported so this module never depends on `urban_fire`.

    Per ladder level (the state names follow `fire.STATE_EMISSION`):

      F1  smoke damage. One or two small STAIN events on the origin storey of
          one side: short venting, never any flame — "the fire was next door
          or it was knocked down early".
      F2  compartment fire, ACTIVE. One or two FLAME events on the origin
          storey per venting side; if the band has a second storey, one
          small, young flame event on it — the leapfrog has just started.
      F3  fully involved and climbing. The lower storeys of the band are OUT
          (they burnt first and the fire has moved on: smoke only or nothing,
          but the longest venting, so the darkest wall); the top two storeys
          of the band are where the FLAME is now.
      F4+ burnt out. Every storey of the band has vented for a long time
          through most or all of its openings, all OUT; up to
          `SMOULDER_EVENTS_MAX` of the topmost become SMOULDER (Flow smoke).

    Flame events are capped at `max_active` openings in total, hottest storey
    first; the overflow is re-labelled OUT (it burnt out — the wall keeps its
    stain, the Flow budget keeps its ceiling). The first event drawn is
    ALWAYS in the compartment of origin, so the ladder's "clean below the
    origin" signature is anchored to a real vent.
    """
    rng = rng or event_rng(ctx)
    f = ctx["fire"]
    level = f["level"]
    if level not in DURATION_S:
        return []            # F0: nothing vented
    mtag = f["mass"]
    band = sorted(int(s) for s in f["storeys"])
    origin = int(f["origin"])
    sides = list(f["sides"])
    burnt_out = level in ("F4", "F5", "F6")
    tau0 = DURATION_S.get(level, DURATION_S["F3"]) * float(heavy)

    # one wind for the building, felt in-plane on each side
    wind = (rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0))
    side_dir = {"S": (1.0, 0.0), "E": (0.0, 1.0), "N": (-1.0, 0.0),
                "W": (0.0, -1.0)}

    def drift_for(side):
        dx, dy = side_dir[side]
        c = wind[0] * dx + wind[1] * dy
        return max(-0.35, min(0.35, 0.30 * c + rng.uniform(-0.06, 0.06)))

    groups = {}
    for side in sides:
        for s in band:
            ops = openings(ctx, mtag, side, s)
            if ops:
                groups[(side, s)] = ops
    if not groups:
        return []

    events = []
    eid = [0]

    def add(ops, side, s, state, tau, inten):
        if not ops:
            return
        eid[0] += 1
        events.append(_make_event(ctx, ops, side, s, state, tau, inten,
                                  drift_for(side), eid[0]))

    n = len(band)
    if level == "F1":
        side = sides[0]
        ops = groups.get((side, origin)) or next(iter(groups.values()))
        for r in _runs(ops, rng, 0.25 if len(ops) > 4 else 0.5,
                       run_len=(1, 2))[:2]:
            add(r, side, origin, "stain", tau0, 0.5)
    elif level == "F2":
        for side in sides:
            ops = groups.get((side, origin))
            if ops:
                for r in _runs(ops, rng, 0.45)[:2]:
                    add(r, side, origin, "flame", tau0, rng.uniform(0.8, 1.0))
            if n > 1 and (origin + 1) in band:
                ops = groups.get((side, origin + 1))
                if ops and rng.random() < 0.8:
                    for r in _runs(ops, rng, 0.2, run_len=(1, 3))[:1]:
                        add(r, side, origin + 1, "flame", tau0 * 0.45,
                            rng.uniform(0.6, 0.9))
    elif level == "F3":
        for side in sides:
            for s in band:
                ops = groups.get((side, s))
                if not ops:
                    continue
                d = s - origin
                sev = max(0.2, float(severity(ctx, s, mtag)))
                if d >= n - 2:
                    state, tau, share = "flame", tau0 * 0.6, 0.55
                else:
                    # burnt FIRST, so the whole floor plate went: every
                    # compartment on it vented (a clean bay through the
                    # middle of a fully involved storey read as two fires)
                    state, tau, share = "out", tau0 * (0.7 + 0.3 * sev), 1.0
                for r in _runs(ops, rng, share)[:(3 if share < 1 else 99)]:
                    add(r, side, s, state, tau, rng.uniform(0.7, 1.0) * sev)
    else:
        share = 0.8 if level == "F4" else 1.0
        for side in sides:
            for s in band:
                ops = groups.get((side, s))
                if not ops:
                    continue
                sev = max(0.35, float(severity(ctx, s, mtag)))
                for r in _runs(ops, rng, share, run_len=(2, 5)):
                    add(r, side, s, "out", tau0 * (0.6 + 0.4 * sev),
                        rng.uniform(0.75, 1.0) * sev)
        # the topmost few still smoke
        top_first = sorted(events, key=lambda ev: (-ev["storey"], ev["id"]))
        for ev in top_first[:SMOULDER_EVENTS_MAX]:
            ev["state"] = "smoulder"

    # the compartment of origin is always an event
    if not any(ev["storey"] == origin for ev in events):
        side = sides[0]
        ops = groups.get((side, origin))
        if ops:
            add(ops[:rng.randint(*RUN_LEN)], side, origin,
                "stain" if level == "F1" else
                ("flame" if not burnt_out else "out"), tau0, 0.9)

    # cap the flame sources at the Flow budget, hottest storey first
    flames = sorted([ev for ev in events if ev["state"] == "flame"],
                    key=lambda ev: (-ev["storey"], ev["id"]))
    used = 0
    for ev in flames:
        k = len(ev["ops"])
        if used + k <= max_active:
            used += k
        else:
            room = max_active - used
            if room > 0:
                # split: the first `room` openings stay alight
                keep = ev["ops"][:room]
                rest = ev["ops"][room:]
                ev.update(_make_event(ctx, keep, ev["side"], ev["storey"],
                                      "flame", ev["tau"], ev["intensity"],
                                      ev["drift"], ev["id"]))
                eid[0] += 1
                events.append(_make_event(ctx, rest, ev["side"], ev["storey"],
                                          "out", ev["tau"], ev["intensity"],
                                          ev["drift"], eid[0]))
                used = max_active
            else:
                ev["state"] = "out"
    events.sort(key=lambda ev: ev["id"])
    return events


def summarise(events):
    c = {}
    for ev in events:
        c[ev["state"]] = c.get(ev["state"], 0) + 1
    return "{0} event(s): ".format(len(events)) + ", ".join(
        "{0} {1}".format(v, k) for k, v in sorted(c.items()))


# ---------------------------------------------------------------------------
# One event's flame + plume, as a deposit (g/m2) on the canvas
# ---------------------------------------------------------------------------
def flame_geometry(ev):
    """EN 1991-1-2 Annex B, no forced draught — see the module docstring for
    the equation numbers. Returns a dict with Q [MW], L_L, L_H, L_f, T_w [K],
    z_v (the axis origin, mid-height of the opening) and z_tip."""
    w_t, h_eq, A_v = ev["w_t"], ev["h_eq"], ev["A_v"]
    Q = Q_PER_AV * A_v * math.sqrt(h_eq) * max(0.05, ev["intensity"])
    L_L = max(0.0, 1.9 * (Q / w_t) ** (2.0 / 3.0) - h_eq)
    attached = h_eq <= 1.25 * w_t
    if ev.get("top"):
        L_H = 0.6 * h_eq * (max(L_L, 0.01) / h_eq) ** (1.0 / 3.0)
    elif attached:
        L_H = h_eq / 3.0
    else:
        L_H = 0.3 * h_eq * (h_eq / w_t) ** 0.54
    L_f = L_L + h_eq / 2.0
    r = min(0.9, L_f * w_t / max(1e-6, Q))
    T_w = 520.0 / (1.0 - 0.4725 * r) + T_AMB
    z_v = ev["z_sill"] + h_eq / 2.0
    return {"Q": Q, "L_L": L_L, "L_H": L_H, "L_f": L_f, "T_w": T_w,
            "attached": attached, "z_v": z_v, "z_tip": ev["z_head"] + L_L,
            "r": r}


def _meander(nrng, n_rows, ppm):
    """A smooth, zero-mean wander along the rows, unit amplitude."""
    n = int(n_rows)
    x = nrng.normal(size=n)
    k = max(2, min(int(round(MEANDER_LEN_M * ppm)), max(2, n // 4)))
    ker = np.exp(-0.5 * (np.arange(-2 * k, 2 * k + 1) / float(k)) ** 2)
    ker /= ker.sum()
    pad = 2 * k
    y = np.convolve(np.pad(x, pad, mode="reflect"), ker, mode="same")[pad:pad + n]
    s = float(np.abs(y).max())
    return y / s if s > 1e-9 else y


def deposit_event(dep, ev, geo, ppm, per, H, z0, tau, nrng):
    """Add one event's thermophoretic deposit (g/m2) into `dep` (h x w,
    row 0 = top). The maths is the module docstring's items 1-4.

    ONE PLUME PER OPENING of the event, all sharing the event's own flame
    (Annex B is written for the run as a whole — `Q/w_t`, `L_L`, `T_w` are
    event-level — but a run of three windows two metres apart vents three
    sheets of flame, not one sheet centred on the gap between them). The
    three plumes then widen into each other with height, which is what
    gives a storey alight along its length the scalloped lower edge and
    merged upper wash every photograph of one shows.
    """
    off, _ = perimeter_offsets(ev["_m"])
    for op in ev["ops"]:
        u0, u1, zs, zh = op["span"]
        _deposit_plume(dep, ev, geo, off[ev["side"]] + 0.5 * (u0 + u1),
                       max(0.3, u1 - u0), max(0.3, zh - zs), zs, zh,
                       ppm, per, H, z0, tau, nrng)


def _deposit_plume(dep, ev, geo, u_c, w_t, h_eq, z_sill, z_head, ppm, per, H,
                   z0, tau, nrng):
    h, w = dep.shape
    z_tip, T_w = geo["z_tip"], geo["T_w"]
    z_v = z_sill + h_eq / 2.0
    dTw = T_w - T_AMB

    # rows of interest: from the downwash below the sill to the top
    z_lo = z_sill - (DOWNWASH_REACH * 1.5 + NEUTRAL_WOBBLE) * h_eq
    r_lo = int(max(0, math.floor((H - (z_lo - z0)) * ppm)))
    r_hi = min(h, r_lo + 1)
    r0 = 0
    rows = np.arange(r0, r_hi)
    z = z0 + H - (rows + 0.5) / ppm            # world height per row
    dz = z - z_v                               # height above the axis origin

    # axis temperature rise, per row
    dT = np.zeros_like(z)
    below_tip = dz <= (z_tip - z_v)
    # inside the flame: EC1 B.15 along the axis from the window
    lx = np.maximum(0.0, dz)
    dT_flame = dTw * (1.0 - 0.4725 * lx * ev["w_t"] / max(1e-6, geo["Q"]))
    dT[below_tip] = np.maximum(FLAME_TIP_DT, dT_flame[below_tip])
    # above the tip: the plume decay from the tip value
    above = ~below_tip
    ratio = np.maximum(1.0, dz[above] / max(0.3, z_tip - z_v))
    dT[above] = FLAME_TIP_DT * ratio ** (-PLUME_DECAY)
    # the opening itself carries the hot outflow
    dT[z < z_head] = dTw

    # wall contact: a flame that PROJECTS stands off the wall by L_H against
    # an out-of-plane half-thickness that starts at the flame depth and
    # grows with the plume
    thick = h_eq / 3.0 + SPREAD * np.maximum(0.0, dz)
    contact = np.exp(-math.log(2.0) * (geo["L_H"] / thick) ** 2)
    dT = dT * (0.55 + 0.45 * contact)

    # THE LOWER EDGE, and why it is not a step. Above the head the wall is in
    # the plume. Beside the opening (between the neutral plane and the head)
    # the jamb reveals see the outflow layer, which thickens towards the
    # head — a ramp, not a slab; below the neutral plane there is only the
    # downwash. The two meet at NEUTRAL_FRAC on both sides, and the neutral
    # plane itself wanders along the wall by NEUTRAL_WOBBLE h_eq, so the
    # stain's bottom is a ragged fringe at the sills rather than the ruled
    # line the first sim skins showed (uf_soot apartment_tall, 2026-08-30).
    # Lateral: the stain is only as wide as the OPENING down at the sill,
    # widens to ROOT_WIDEN x at the head, and fans at SPREAD above it — so
    # the piers between windows stay clean at sill level and blacken above.
    b_top = ROOT_WIDEN * w_t / 2.0 + SPREAD * max(0.0, float(z.max()) - z_head)
    # SIX half-widths, not three. The deposit at the axis of an hour-long
    # event is tens of g/m2, so the Gaussian tail at 3.2 b (0.08 % of the
    # axis) was still a visible stain — and the column window's edge showed
    # as a straight vertical cut through it (highrise_step F3 elevation,
    # 2026-08-30). At 6 b the tail is 1e-11 of the axis: nothing.
    reach = 6.0 * b_top + MEANDER_AMP * b_top
    axis0 = u_c + math.tan(ev["drift"]) * np.maximum(0.0, z - z_head)
    c_lo = int(math.floor((float(axis0.min()) - reach) * ppm))
    c_hi = int(math.ceil((float(axis0.max()) + reach) * ppm)) + 1
    cols = np.arange(c_lo, c_hi)
    u = (cols + 0.5) / ppm
    neutral = z_sill + h_eq / 3.0
    zn = neutral + NEUTRAL_WOBBLE * h_eq * _meander(nrng, len(cols), ppm)[None, :]
    zz = z[:, None]
    up = np.clip((zz - zn) / np.maximum(0.2, z_head - zn), 0.0, 1.0)
    ramp = NEUTRAL_FRAC + (1.0 - NEUTRAL_FRAC) * up * up * (3.0 - 2.0 * up)
    down = NEUTRAL_FRAC * np.exp(-np.maximum(0.0, zn - zz)
                                 / max(0.2, DOWNWASH_REACH * h_eq))
    vert = np.where(zz >= z_head, 1.0, np.where(zz >= zn, ramp, down))

    b = (w_t / 2.0 * (1.0 + (ROOT_WIDEN - 1.0) * up)
         + SPREAD * np.maximum(0.0, zz - z_head))
    axis = (axis0[:, None]
            + MEANDER_AMP * b * _meander(nrng, len(rows), ppm)[:, None])
    d = u[None, :] - axis
    d = (d + per / 2.0) % per - per / 2.0      # wrap round the perimeter
    # Gaussian across the PLUME; a flat-topped, steep-sided profile across
    # the OPENING rows below the head — the outflow there is a sheet the
    # width of the opening, and a Gaussian tail at flame temperature pushed
    # the stain's edge out further on exactly those rows (a comb along the
    # edge of the glass tower's stain, 2026-08-30)
    pw = np.where(zz >= z_head, 2.0, 4.0)
    prof = np.exp(-math.log(2.0) * (np.abs(d) / b) ** pw)
    dTg = dT[:, None] * vert * prof

    # thermophoresis (Riahi & Beyler eqs 2, 3, 5)
    Tg = T_AMB + dTg
    Tw = T_AMB + WALL_WARM * dTg
    Tf = 0.5 * (Tg + Tw)
    grad = H_CONV * (Tg - Tw) / _k_air(Tf)
    vth = K_TH * _nu(Tf) / Tf * grad                       # m/s
    cs = Y_SOOT * _rho(Tg) * CP * dTg / (CHI_CONV * DH_C)  # kg/m3
    m_g = vth * cs * tau * 1000.0 * DEPOSIT_GAIN           # g/m2

    if c_lo >= 0 and c_hi <= w:
        dep[rows[0]:rows[-1] + 1, c_lo:c_hi] += m_g.astype(dep.dtype)
    else:
        cc = cols % w
        np.add.at(dep, (rows[:, None], cc[None, :]), m_g.astype(dep.dtype))


# ---------------------------------------------------------------------------
# Noise
# ---------------------------------------------------------------------------
def _noise(nrng, h, w, beta=2.0, stretch_v=1.0, lo=None, hi=None):
    """Spectral 1/f^beta noise in 0..1, periodic in both axes."""
    f = np.fft.fft2(nrng.normal(size=(h, w)))
    fy = np.fft.fftfreq(h)[:, None] * float(stretch_v)
    fx = np.fft.fftfreq(w)[None, :]
    r = np.sqrt(fx * fx + fy * fy)
    r[0, 0] = 1e-6
    amp = r ** (-beta / 2.0)
    if lo is not None:
        amp = amp * (r >= lo)
    if hi is not None:
        amp = amp * (r <= hi)
    amp[0, 0] = 0.0
    a = np.real(np.fft.ifft2(f * amp))
    return ((a - a.min()) / (a.max() - a.min() + 1e-9)).astype(np.float32)


def _band(scale_m, ppm):
    """(lo, hi) cycles/px for features about `scale_m` across."""
    c = 1.0 / max(2.0, scale_m * ppm)
    return c * 0.35, c * 2.5


# ---------------------------------------------------------------------------
# The skin
# ---------------------------------------------------------------------------
def canvas_dims(m):
    W, D = float(m["W"]), float(m["D"])
    per = 2.0 * (W + D)
    H = float(m["top"] - m["z0"]) + parapet_height(m)
    ppm = min(PX_PER_M, MAX_W_PX / per, MAX_H_PX / max(0.5, H))
    w = max(MIN_PX, int(round(per * ppm)))
    h = max(MIN_PX, int(round(H * ppm)))
    return h, w, ppm, per, H


def skin(ctx, events, nrng, finish="char", glass=False, duration_scale=1.0):
    """The building's soot as RGBA, unwrapped S|E|N|W around the burning
    mass. Returns a dict: `rgba` (h x w x 4, float32, row 0 = top), `dep`
    (g/m2, after the multiplicative noise), `dep_raw` (before it — the pure
    physics, what a test compares), `ppm`, `per`, `H`, `z0`, `offsets`,
    `events`.

    `glass=True` hardens the alpha into a FILM (a curtain wall's soot is an
    opaque coat on glass or nothing, not a translucent wash) — the
    replacement for `r_smoke_stain`'s old flat-tone bind on towers.
    """
    mtag = ctx["fire"]["mass"]
    m = ctx["info"]["masses"][mtag]
    h, w, ppm, per, H = canvas_dims(m)
    z0 = float(m["z0"])
    dep = np.zeros((h, w), dtype=np.float32)
    for ev in events:
        ev["_m"] = m
        geo = flame_geometry(ev)
        ev["geo"] = geo
        deposit_event(dep, ev, geo, ppm, per, H, z0,
                      ev["tau"] * float(duration_scale), nrng)
        del ev["_m"]

    # multiplicative noise on the deposit (docstring item 6)
    lo, hi = _band(MOTTLE_M, ppm)
    mottle = _noise(nrng, h, w, beta=2.0, lo=lo, hi=hi)
    lo, hi = _band(STREAK_M, ppm)
    streak = _noise(nrng, h, w, beta=2.0, stretch_v=STREAK_STRETCH,
                    lo=lo, hi=hi * 2.0)
    gain = ((1.0 - MOTTLE + 2.0 * MOTTLE * mottle)
            * (1.0 - STREAK + 2.0 * STREAK * streak))
    dep_n = dep * gain
    alpha = 1.0 - np.exp(-SIGMA_S * dep_n)
    # even a saturated coat is not one flat sheet: thickness shows through
    alpha = alpha * (0.86 + 0.14 * streak)
    if glass:
        # a FILM: soot on glass is an opaque coat or nothing much. Hardened
        # over a wide range so the edge is a fringe rather than a cut, and
        # from a COARSER mottle than masonry gets (a threshold on fine noise
        # is grain; on 3 m noise it is a wavy edge), with no streaks — the
        # licks that read on masonry came out as hair on a hard-edged coat.
        lo, hi = _band(MOTTLE_M * 2.0, ppm)
        coarse = _noise(nrng, h, w, beta=2.4, lo=lo, hi=hi)
        # ...and hardened over a WIDE ramp on the deposit's log, not on
        # alpha: `1 - exp(-8.7 m)` saturates so fast that any ramp on it
        # collapsed the tail into a ~1 m cliff (the straight-edged blob on
        # highrise_step F3's elevation, 2026-08-30). Two decades of deposit
        # below the saturation point become the film's fringe.
        dg = dep * (1.0 - MOTTLE + 2.0 * MOTTLE * coarse)
        lg = np.log10(np.maximum(dg, 1e-6) * SIGMA_S)     # 0 at alpha ~ 0.63
        t = np.clip((lg + 2.0) / 2.3, 0.0, 1.0)             # 1e-2 .. 2 x sat
        alpha = (t * t * (3.0 - 2.0 * t)) * (0.92 + 0.08 * mottle)

    lo, hi = _band(TONE_NOISE_M, ppm)
    tone = _noise(nrng, h, w, beta=2.2, lo=lo, hi=hi)
    ash = ASH_LEVEL.get(finish, 0.25)
    t = np.clip(ash * (0.35 + 0.9 * tone), 0.0, 1.0)[..., None]
    dark = np.asarray(SOOT_DARK, dtype=np.float32)[None, None, :]
    ashc = np.asarray(SOOT_ASH, dtype=np.float32)[None, None, :]
    rgba = np.zeros((h, w, 4), dtype=np.float32)
    rgba[..., :3] = dark * (1.0 - t) + ashc * t
    rgba[..., 3] = np.clip(alpha, 0.0, 1.0)
    offsets, _ = perimeter_offsets(m)
    return {"rgba": rgba, "dep": dep_n.astype(np.float32),
            "dep_raw": dep, "ppm": ppm,
            "per": per, "H": H, "z0": z0, "offsets": offsets,
            "events": events, "mass": mtag}


# ---------------------------------------------------------------------------
# From the skin to one prim
# ---------------------------------------------------------------------------
def piece_crop(sk, side, u0, u1, za, zb):
    """The skin's pixels behind one module: `u0..u1` along `side` (metres),
    world heights `za..zb`. Columns wrap round the perimeter; rows clamp."""
    rgba, ppm, per, H, z0 = sk["rgba"], sk["ppm"], sk["per"], sk["H"], sk["z0"]
    h, w = rgba.shape[0], rgba.shape[1]
    off = sk["offsets"][side]
    c0 = int(math.floor((off + u0) * ppm))
    c1 = max(c0 + 1, int(math.ceil((off + u1) * ppm)))
    cols = np.arange(c0, c1) % w
    r0 = int(math.floor((H - (zb - z0)) * ppm))
    r1 = int(math.ceil((H - (za - z0)) * ppm))
    r0 = max(0, min(h - 1, r0))
    r1 = max(r0 + 1, min(h, r1))
    return rgba[r0:r1][:, cols]


def _smoothstep(a, b, x):
    t = np.clip((x - a) / max(1e-6, b - a), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def merge_rgb(base, crop):
    """`base` (Hb x Wb x 3, 0..1) with `crop` (RGBA) laid over it corner to
    corner, worked at whichever of the two is larger (a 128 px kit map must
    not be allowed to destroy a 2 cm/px stain)."""
    base = np.asarray(base, dtype=np.float32)
    if base.ndim == 2:
        base = np.repeat(base[..., None], 3, axis=2)
    base = base[..., :3]
    bh, bw = base.shape[0], base.shape[1]
    ch, cw = crop.shape[0], crop.shape[1]
    oh = max(bh, min(PIECE_MAX_PX, ch))
    ow = max(bw, min(PIECE_MAX_PX, cw))
    byi = np.linspace(0, bh - 1, oh).astype(int)
    bxi = np.linspace(0, bw - 1, ow).astype(int)
    b = base[byi][:, bxi]
    cyi = np.linspace(0, ch - 1, oh).astype(int)
    cxi = np.linspace(0, cw - 1, ow).astype(int)
    c = crop[cyi][:, cxi]
    a = c[..., 3:4]
    grey = b.mean(axis=2, keepdims=True)
    desat = b * (1.0 - DESAT * a) + grey * (DESAT * a)
    return np.clip(desat * (1.0 - a) + c[..., :3] * a, 0.0, 1.0)


def _read_rgb(url, max_px=2048):
    """A texture from Nucleus (through omni.client when it is importable) or
    from disk, as an H x W x 3 float array in 0..1, or None."""
    import io

    from PIL import Image

    data = None
    try:
        import omni.client
        res, _, content = omni.client.read_file(url)
        if str(res).endswith("OK"):
            data = bytes(memoryview(content))
    except Exception:
        data = None
    if data is None:
        p = url[7:] if url.startswith("file://") else url
        if not os.path.exists(p):
            return None
        with open(p, "rb") as fh:
            data = fh.read()
    try:
        im = Image.open(io.BytesIO(data)).convert("RGB")
    except Exception:
        return None
    if max(im.size) > max_px:
        s = max_px / float(max(im.size))
        im = im.resize((max(1, int(im.width * s)), max(1, int(im.height * s))))
    return np.asarray(im, dtype=np.float32) / 255.0


def merge_piece(base_url, crop, out_dir=None, key=""):
    """This prim's own base map with its crop of the skin merged in, as a
    cached PNG path (keyed on the base URL and the crop's own pixels, so the
    modules of a run that share a map but take different crops each bake
    once and clones of a building reuse the lot). None if the base cannot
    be read."""
    from PIL import Image

    a = np.asarray(crop, dtype=np.float32)
    ck = hashlib.md5(np.round(a, 3).tobytes()).hexdigest()[:16]
    kk = hashlib.md5("|".join((_RECIPE, str(base_url), ck, str(key)))
                     .encode("utf-8")).hexdigest()[:16]
    d = out_dir or OUT_DIR
    try:
        os.makedirs(d, exist_ok=True)
        gi = os.path.join(d, ".gitignore")
        if not os.path.exists(gi):
            with open(gi, "w") as fh:
                fh.write("*\n!.gitignore\n")
    except OSError:
        pass
    path = os.path.join(d, "sootpiece_{0}.png".format(kk))
    if os.path.exists(path):
        return path
    base = _read_rgb(base_url) if base_url else None
    if base is None:
        return None
    out = merge_rgb(base, a)
    Image.fromarray((out * 255.0 + 0.5).astype(np.uint8)).save(path)
    return path


def find_basecolor(mat_prim):
    """(shader prim path, input name, file) of the base-colour map a material
    network samples, or (None, None, None). Same heuristics as
    `damage._basecolor_texture`, but keeping WHERE the input lives so a copy
    of the material can override just that one input."""
    from pxr import Sdf, Usd, UsdShade

    if not mat_prim or not mat_prim.IsValid():
        return None, None, None
    fallback = None
    for p in Usd.PrimRange(mat_prim):
        sh = UsdShade.Shader(p)
        if not sh:
            continue
        for inp in sh.GetInputs():
            try:
                v = inp.Get()
            except Exception:
                continue
            if not isinstance(v, Sdf.AssetPath):
                continue
            f = v.resolvedPath or v.path or ""
            low = f.lower()
            if not low:
                continue
            if any(k in low for k in ("basecolor", "albedo", "diffuse", "_bc",
                                      "_col", "_d.", "basecolour")):
                return p.GetPath(), inp.GetBaseName(), f
            if fallback is None and not any(
                    k in low for k in ("normal", "_n.", "_nrm", "orm", "rough",
                                       "metal", "_r.", "_m.", "occlusion",
                                       "_ao", "height", "_h.", "emissive",
                                       "opacity", "mask", "_e.", ".mdl")):
                fallback = (p.GetPath(), inp.GetBaseName(), f)
    return fallback if fallback else (None, None, None)


def piece_material_like(stage, path, src_mat_prim, shader_path, input_name,
                        tex_path):
    """A COPY of the module's own material with only its base-colour map
    swapped for the merged one — normal, roughness, AO, metallic and every
    other input exactly as the kit authored them.

    An INTERNAL REFERENCE to the source material prim composes its whole
    network (and whatever that network references) under `path`; the one
    override authored on top is the texture input. This is what "translating
    to the actual materials" needs: a diffuse-only OmniPBR bound in place of
    the kit's own material made every sooted module a flat, differently-lit
    rectangle next to its untouched neighbour, soot or no soot (uf_soot
    commercial F4, 2026-08-30). Returns None if the source cannot be
    referenced (an instance proxy), so the caller can fall back to
    `piece_material`.
    """
    from pxr import Sdf, UsdShade

    if (not src_mat_prim or not src_mat_prim.IsValid()
            or src_mat_prim.IsInstanceProxy()
            or "__Prototype" in str(src_mat_prim.GetPath())):
        return None
    src = src_mat_prim.GetPath()
    dst = Sdf.Path(path)
    mat = UsdShade.Material.Define(stage, dst)
    prim = mat.GetPrim()
    if not prim.GetReferences().AddInternalReference(src):
        return None
    rel = Sdf.Path(shader_path).MakeRelativePath(src)
    if rel.isEmpty or str(rel) in (".", str(shader_path)):
        target = dst if Sdf.Path(shader_path) == src else None
        if target is None:
            return None
    else:
        target = dst.AppendPath(rel)
    sh_prim = stage.GetPrimAtPath(target)
    if not sh_prim or not sh_prim.IsValid():
        return None
    sh = UsdShade.Shader(sh_prim)
    inp = sh.GetInput(input_name) if sh else None
    if not inp:
        return None
    inp.Set(Sdf.AssetPath(str(tex_path)))
    return mat


def piece_material(stage, path, tex_path, roughness=0.85):
    """An OmniPBR with the merged map as its diffuse, on the module's OWN
    UVs — no `project_uvw`: a crop is already cut to this module's face, so
    its existing UVs address it, and leaving them alone is what keeps the
    kit's modelled relief sampling its texture the way the kit intended."""
    from pxr import Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(str(tex_path)))
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(float(roughness))
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(),
                                                        "out")
    mat.CreateVolumeOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def save_skin_png(sk, path, base_rgb=None):
    """The skin over a flat base (or `base_rgb`, an h x w x 3 array), for a
    preview. Returns `path`."""
    from PIL import Image

    rgba = sk["rgba"]
    h, w = rgba.shape[0], rgba.shape[1]
    if base_rgb is None:
        base_rgb = np.full((h, w, 3), 0.62, dtype=np.float32)
    out = merge_rgb(base_rgb, rgba)
    Image.fromarray((np.clip(out, 0, 1) * 255.0 + 0.5).astype(np.uint8)).save(path)
    return path
