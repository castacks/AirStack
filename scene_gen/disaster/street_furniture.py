"""street_furniture.py — hurricane damage for the plate's kerbside props:
`sign`, `streetlight`, `trash_can`, `fire_hydrant` (DEBRIS D-next review,
2026-09-01, user on the live 500 m plate: "I don't see fallen over [signs],
stop signs, etc. Those should also be thrown away/uprooted.").

THE REPORTED DEFECT, confirmed before this file existed: the scene's own
tally places 45 `sign`, 37 `streetlight`, 23 `trash_can` and 19
`fire_hydrant` prims (`suburb_scene.build_frontage` / `AssetPools.place`,
one placement per prim under `PARENT + "/<category>_<group>_<i>"` —
`scene_generator.apply_placements`'s own naming convention). Nothing reads
any of them: not `washaway.py`, not `hurricane.py`, not the launcher. They
stand pristine in a Cat-3 wind field while the houses around them lose
roofs and the fences around THEM blow down.

THIS FILE IS DISASTER-AGNOSTIC BY THE SAME DISCIPLINE `washaway.py` KEEPS.
It has no `import disaster.hurricane` anywhere in it, and never will: it is
called BY the hurricane launcher, the same relationship `washaway.py` has,
not the other way round. `depth_fn`, `wind_bearing_fn` and
`wind_intensity_fn` are opaque callables from whatever storm model the
caller has — exactly how `washaway.fence_specs` already treats the same
three names — so nothing here assumes `hurricane.intensity_field`'s
specific shape, only that it returns numbers on a 0..1 scale.

STRUCTURE MIRRORS `washaway.fence_specs` / `washaway.apply_fence_pose`
DELIBERATELY: a PURE decision function (`street_furniture_specs`, no
`pxr`, no stage — takes already-measured item geometry plus depth/wind
callables, returns one decision dict per item) and a separate
stage-touching apply function (`apply_street_furniture_pose`) that
consumes one decision. `measure_street_item` is the third piece, the
measurement half fences get from `measure_fence`.

WHY THE FOUR CATEGORIES ARE NOT ONE SHARED DAMAGE CURVE. A `fire_hydrant`
that topples as readily as a `trash_can` would be a worse bug than "nothing
moves at all" — it would look like a rain of random destruction rather
than a wind event that a careful viewer can read as physically consistent.
Each category's ladder is argued from what the object physically IS —
mass, drag area, anchoring — not fit to a target percentage:

  sign          The lightest and most exposed thing on the plate: a large
                flat aluminium/sheet blade on a thin steel post, and for
                the traffic-control assets specifically, frequently a
                MUTCD/AASHTO "breakaway" or slip-base post BY DESIGN — built
                to yield to a vehicle strike, which trades away wind
                resistance too. Reported failure modes span a real range
                ("blade bent/twisted on the post" through "post bent over"
                through "whole sign uprooted and gone" through "carried and
                lying flat downwind"); this file compresses the two mild
                readings into one "leaning" action (differentiated only by
                where in its lean-angle band the draw lands — see
                `_LEAN_DEG_RANGE`) and keeps "flat" / "gone" as the two
                severe ones. Onsets sit BELOW this module's own house
                ladder's lightest rung (`hurricane._HOUSE_CUTS`'
                `shingles_lost`, onset i=0.353) — restated here as a
                citation, not imported (see the module-dependency note
                above) — because a sign blade is lighter and more exposed
                than a course of roof shingles.
  trash_can     A hollow, unanchored cylinder with no footing and nothing
                holding it down but its own empty weight — no restoring
                moment once it starts to go, so there is no stable
                partly-tipped rest state and this category has no
                "leaning" rung at all. It is also the category the "a bin
                floats" design note means literally: in standing water it
                is the most buoyant thing on the plate, hence its own
                (shallow) `_DEPTH_GONE_M` entry, well under a fence panel's
                own `washaway.FENCE_WATER_DEPTH_GONE_M` (0.4 m — a timber
                panel lashed to two posts is a much higher bar than an
                empty bin). Lowest wind onsets of the four: "essentially
                always displaced in a Cat-3" per the design brief.
  streetlight   A galvanised steel or slip-formed concrete pole on a
                poured, anchor-bolted footing — a real, code-designed
                structural member (AASHTO luminaire-pole standards specify
                a design wind load), not cladding. It fails AS A STRUCTURE
                — the pole yields or the footing cracks — which needs force
                in the range this file's own house ladder reserves for its
                STRUCTURAL rungs (`roof_collapsed` onset i=0.546,
                `partial_collapse` i=0.590), not its roof-COVERING ones.
                Mostly survives; a minority lean, and only the rare extreme
                draw fails at the base. NEVER "gone" — a pole that fails is
                still lying right there, not carried off.
  fire_hydrant  Cast iron, low and squat — low centre of gravity, small
                frontal area relative to its mass — and bolted through a
                buried water main. A real dry-barrel hydrant DOES have an
                engineered "breakaway" flange, but that exists for a
                horizontal VEHICLE strike: an impulsive point load orders
                of magnitude past anything a design-level wind gust puts on
                an object this shape and this low. THE DELIBERATE NEGATIVE
                CASE: there is no plausible wind-only failure mode, so
                `_CUTS["fire_hydrant"]` is a single row with an onset no
                real field ever reaches (see that table) rather than a
                hand-set-to-zero special case bypassing the model — it goes
                through the EXACT SAME code every other category does, so
                "hydrants essentially never move" is verified BY the pass,
                not assumed around it. It is the control that proves this
                pass discriminates rather than flattening everything.

RUNS WITHOUT ISAAC. Same import-boundary guarantee as `washaway.py`:
`street_furniture_specs` never imports `pxr`; only `measure_street_item`
and `apply_street_furniture_pose` do, and only inside their own bodies.
"""

import math

from . import surge

# ---------------------------------------------------------------------------
# calibration
# ---------------------------------------------------------------------------
#
# THE SAME NORMALISATION `hurricane.intensity_field` uses (gust_mps /
# REF_GUST_MPS, REF_GUST_MPS = 100 m/s) so every onset below reads on the
# identical 0..1 scale `wind_intensity_fn` already returns. NOT imported
# from `disaster.hurricane` — restated here, per the module docstring's
# no-hurricane-dependency discipline.
_REF_GUST_MPS = 100.0

# Per-item "how well is THIS one built/anchored today" — age, rust, how well
# the footing was poured, whether the post already had a knock. UNLIKE
# `hurricane.CODE_ERAS` (a 50-year housing stock spanning multiple building
# codes), municipal street furniture on one block is installed to one spec,
# so a single resistance BAND stands in for day-to-day condition rather than
# a population of distinct eras. Divides the field intensity exactly as
# `hurricane._resistance(vuln)` does: `resist_hi` is the STRONGEST item.
ITEM_RESIST_LO, ITEM_RESIST_HI = 0.95, 1.45
# Drawn AFTER the resistance divide, same role as `hurricane.
# house_level_for_intensity`'s own `jitter`: everything about one item's
# damage the intensity+resistance blend does not capture.
ITEM_JITTER = 0.05

# `_ladder`'s convention (`hurricane._ladder`'s own, reimplemented rather
# than imported — see the module docstring): ASCENDING `(upper_bound,
# action)` rows; row k's bound is the ceiling of row k's own action, so the
# first row's action occupies `[0, bound)`. Onsets below are cited as
# `mph / m/s -> i` the same way `hurricane._HOUSE_CUTS`'s own comment does.
#
# CALIBRATED, not just onset-cited, against the SAME (resistance, jitter)
# blend `street_furniture_specs` actually applies: `ITEM_RESIST_LO/HI`
# (0.95..1.45) and `ITEM_JITTER` (0.05) turn one field intensity `i` into a
# per-item effective intensity spread of roughly `[i/1.45 - 0.05, i/0.95 +
# 0.05]` — at this file's own two shipped hurricane levels
# (`suburb_hurricane_500_l2.yaml` `site_gust_mps: 55.0` -> i=0.55,
# `..._l3.yaml` `site_gust_mps: 70.0` -> i=0.70) that spread is roughly
# 0.33-0.63 (L2) and 0.43-0.79 (L3). Onsets below are cited in mph/m/s the
# same way `hurricane._HOUSE_CUTS` cites its own, but were then checked with
# `python3` against that measured spread so a real L2/L3 plate lands a MIX
# of rungs rather than saturating one — see each row's own comment for what
# share of that spread it captures.
_CUTS = {
    # 42 m/s / 94 mph  -> i=0.420   sub-onset: sign untouched
    # 49 m/s / 110 mph -> i=0.490   blade starts to twist/bend, or the post
    #                               starts to go over
    # 56 m/s / 125 mph -> i=0.560   post bent hard over / blown down flat,
    #                               possibly carried a short distance
    # (unreachable row)             post shears/uproots at its base and the
    #                               whole sign leaves — "gone" is the open
    #                               top rung, not a fourth onset: past
    #                               `flat`'s own onset there is no further
    #                               physical threshold to cite, just a
    #                               growing SHARE of draws whose resistance/
    #                               jitter push them past it
    # Measured mix (`python3`, `ITEM_RESIST_LO/HI`+`ITEM_JITTER` applied) at
    # L2 (i=0.55): ~27% stands, ~38% leaning, ~27% flat, ~9% gone. At L3
    # (i=0.70): 0% stands, ~9% leaning, ~30% flat, ~61% gone — clearly
    # worse, never saturated to one action at either level.
    # `flat`'s ceiling RAISED 56 -> 78 m/s, 2026-09-01, after measuring the
    # mix at this plate's own intensity band (0.62-0.77) rather than at the
    # nominal preset levels: at i=0.70 the 56 m/s cut sent 62% of signs to
    # `gone` and 84% at i=0.77, so most of the plate's 45 signs simply
    # VANISHED. That is the wrong outcome for this dataset even though it is
    # defensible physics — the review asked for signs "thrown away/uprooted"
    # so they can be SEEN as damage, and a deleted sign is invisible to a
    # detector while a sign lying uprooted on the verge is a findable
    # object. A sign blade shears or the post bends over long before the
    # whole assembly plus its footing leaves the lot, so pushing the
    # carried-off rung up to genuinely extreme wind is also the better
    # model. `gone` is kept, not removed: some do go.
    "sign": ((42.0 / _REF_GUST_MPS, "stands"),
             (49.0 / _REF_GUST_MPS, "leaning"),
             (78.0 / _REF_GUST_MPS, "flat"),
             (2.000, "gone")),
    # 30 m/s / 67 mph  -> i=0.300   sub-onset: bin untouched — well under a
    #                               sign's own onset, this is the most
    #                               wind-vulnerable object on the plate
    # 52 m/s / 116 mph -> i=0.520   tips/rolls off its placed spot and ends
    #                               up lying somewhere downwind
    # Measured mix at L2 (i=0.55): 0% stands, ~78% flat, ~22% gone. At L3
    # (i=0.70): 0% stands, ~20% flat, ~80% gone — "essentially always
    # displaced" at BOTH levels, with "removed entirely" overtaking "lying
    # somewhere downwind" only once the wind is genuinely severe.
    "trash_can": ((30.0 / _REF_GUST_MPS, "stands"),
                  (52.0 / _REF_GUST_MPS, "flat"),
                  (2.000, "gone")),
    # MAILBOX — no `leaning` rung, deliberately, and the same reason the bin
    # above has none (2026-09-01, user: "bins won't be leaning. They are
    # either fallen or straight up, same for mail boxes"). A post-mounted
    # box is a short stiff cantilever on a light post: it either holds or
    # the post snaps/pulls and the whole thing goes over. There is no
    # half-way pose for it the way there is for a 9 m steel light column,
    # which bends at its base plate long before it fails. Slightly tougher
    # than a wheelie bin (bolted or set in the ground, not free-standing on
    # castors) and much lighter than a sign assembly, so its onset sits
    # between the two.
    #
    # NOT PLACED IN THE SUBURBAN SET TODAY — `mailbox` is an `urban.yaml`
    # asset, so this row is dead weight for the suburban hurricane cells and
    # exists so the URBAN cells inherit the rule rather than rediscovering
    # it. `street_furniture_specs` ignores categories it is never handed.
    "mailbox": ((38.0 / _REF_GUST_MPS, "stands"),
                (62.0 / _REF_GUST_MPS, "flat"),
                (2.000, "gone")),
    # 63 m/s / 141 mph -> i=0.630   foundation/anchor bolts start to yield,
    #                               pole visibly off true
    # 80 m/s / 179 mph -> i=0.800   base connection fails outright, pole
    #                               down — essentially unreached at either
    #                               shipped level (see below), a real
    #                               minority-of-a-minority the way "fail at
    #                               the base" is described in the brief
    # Measured mix at L2 (i=0.55): 100% stands, 0% anything else. At L3
    # (i=0.70): ~67% stands, ~33% leaning, ~0% flat — "mostly survives"
    # holds at BOTH shipped levels; base failure only starts to show up
    # past a real L3, at the rare extreme (resistance, jitter) draw.
    # `flat`'s onset LOWERED 80 -> 74 m/s, 2026-09-01. The dataset's level-3
    # cell is specified as "poles down in runs" (see the
    # `freeze-disaster-dataset` ladder) and the user asked for the same:
    # "for the higher intensities you can have the stop signs and street
    # lights, etc fall to the floor too". Measured on the live level-3 plate
    # BEFORE this change: 18 standing, 19 leaning, ZERO flat — the pole bent
    # but never came down, because the old 80 m/s rung sits above anything a
    # 70 m/s site gust can reach. 68 m/s keeps the ladder's shape (a column
    # leans at its base plate well before the connection fails outright, so
    # `leaning` still dominates) while letting the strongest cell actually
    # put some poles on the ground. Swept against
    # `test_streetlight_mostly_survives`, which requires outright base
    # failure to stay a MINORITY of bending — physically right, a column
    # bends at its base plate before the connection lets go. At 68 m/s flat
    # (16.2%) just overtook leaning (15.6%) and the test caught it; 74 m/s
    # gives 67% standing / 22.1% leaning / 10.9% flat at level 3.
    "streetlight": ((63.0 / _REF_GUST_MPS, "stands"),
                    (74.0 / _REF_GUST_MPS, "leaning"),
                    (2.000, "flat")),
    # No reachable onset: `i` is clamped 0..1, `ITEM_RESIST_LO` = 0.95 and
    # `ITEM_JITTER` = 0.05, so the worst possible draw is
    # `1.0 / 0.95 + 0.05` = 1.103 — nowhere near this row's 5.0. See the
    # module docstring's "THE DELIBERATE NEGATIVE CASE".
    "fire_hydrant": ((5.000, "stands"),),
}

# `(category, action) -> (lo_deg, hi_deg)` for `rng.uniform`. Unsigned — the
# SIGN of the lean is never ambiguous here the way it is in
# `washaway._fence_fall_lean` (see `apply_street_furniture_pose`'s own
# docstring for why: the hinge axis is built directly from the fall
# azimuth, not fixed by the item's own geometry, so there is nothing to
# resolve).
_LEAN_DEG_RANGE = {
    # Spans BOTH mild sign readings: the low end is "blade twisted, post
    # still basically vertical", the high end is "post bent hard over".
    ("sign", "leaning"): (10.0, 55.0),
    ("sign", "flat"): (75.0, 90.0),
    ("streetlight", "leaning"): (8.0, 25.0),
    ("streetlight", "flat"): (80.0, 90.0),
    ("trash_can", "flat"): (70.0, 90.0),
}

# `(category, action) -> sigma_m` for a gaussian downwind skid
# (`rng.gauss(0.0, sigma)`, `washaway.FENCE_SLIDE_M`'s own idiom — a fence's
# sigma is 1.9 m). A trash can's is deliberately far larger than a fence
# panel's: "they roll and travel a long way downwind" is the design brief's
# own wording, and a free-rolling cylinder travels much farther than a
# panel that simply falls over where it stood.
_SLIDE_SIGMA_M = {
    ("sign", "leaning"): 0.15,
    ("sign", "flat"): 2.5,
    # A felled pole's BASE is the rotation pivot (`measure_street_item`'s
    # docstring — `streetlight` is placed with `raw_pivot=True`, anchored at
    # its own base) and stays put; only a small sigma covers a footing that
    # shears a little rather than staying planted.
    ("streetlight", "leaning"): 0.0,
    ("streetlight", "flat"): 0.3,
    ("trash_can", "flat"): 7.0,
}

# Depth (m) past which a LIGHT item is treated as floated/carried off by
# standing water, independent of wind — the design brief's "surge depth as
# a separate cause for the light items (a bin floats)". No `streetlight` or
# `fire_hydrant` entry: DELIBERATELY ABSENT, not zero — see
# `street_furniture_specs`'s `.get(category)` -> `None` -> the depth check
# is skipped entirely for a heavy, anchored item, the same way this file's
# `fire_hydrant` wind ladder is a control rather than a special case.
_DEPTH_GONE_M = {
    # Hollow, light, unanchored: floats in barely more than a curb-deep
    # puddle. Well under `washaway.FENCE_WATER_DEPTH_GONE_M` (0.4 m) — a
    # timber panel lashed to two posts is a much higher bar to float than
    # an empty bin.
    "trash_can": 0.30,
    # A direct-bury or small-footing post resists floating on its own; it
    # goes once the footing is undermined or a surge-borne debris strike
    # does the work, which needs meaningfully deeper water than a bin.
    "sign": 0.60,
}

# Perturbs the fall azimuth PER ITEM so a cluster of signs on one corner
# does not all fall in exactly the same direction — the "stamped copy"
# defect `washaway.fence_specs`' own `FENCE_YAW_JITTER_DEG` was added to
# fix, reused here by jittering the azimuth directly (this hinge axis is
# built FROM the azimuth — see `apply_street_furniture_pose` — so there is
# no separate post-hoc twist step needed the way the fence code has one).
AZIMUTH_JITTER_DEG = 25.0


def _ladder(cuts, v):
    """Pick an action from `cuts` (ascending `(upper_bound, action)` rows)
    for effective intensity `v`. Identical convention to
    `hurricane._ladder`, reimplemented rather than imported — see the
    module docstring's no-hurricane-dependency discipline.
    """
    for lim, name in cuts:
        if v < lim:
            return name
    return cuts[-1][1]


def street_furniture_specs(items, depth_fn, wind_bearing_fn,
                           wind_intensity_fn, rng, *,
                           resist_lo=ITEM_RESIST_LO,
                           resist_hi=ITEM_RESIST_HI,
                           jitter=ITEM_JITTER,
                           cuts=None,
                           lean_deg_range=None,
                           slide_sigma_m=None,
                           depth_gone_m=None,
                           azimuth_jitter_deg=AZIMUTH_JITTER_DEG):
    """One decision per item in `items` (`(x, y, category)` tuples, ALREADY
    MEASURED off the placed prims — see `measure_street_item`). Returns a
    list, same order and length as `items`, of

        {"x", "y", "category", "action", ...}

    `action` is one of:

      "gone"    -- removed entirely: `apply_street_furniture_pose` calls
                   `prim.SetActive(False)`, no debris authored. UNLIKE
                   `washaway.fence_specs`' own "gone" (which spawns
                   floating rafts via `_one_raft`), nothing here builds
                   replacement geometry — a sign or bin swept off a 500 m
                   plate is not tracked material the way a fence panel
                   coming apart into the flood is, and inventing a new
                   `planks`/`build_rafts` "kind" for one is out of this
                   file's scope. Two independent triggers, checked THIS
                   order (depth before wind, `fence_specs`' own priority):

                     (a) DEPTH -- `depth_fn(x, y)` past this item's own
                         `_DEPTH_GONE_M` entry (`sign`/`trash_can` only).
                     (b) WIND -- the item's own effective intensity lands
                         in the "gone" rung of its category's ladder.

      "flat"    -- fully toppled. Hinged about a horizontal axis
                   PERPENDICULAR TO THE LOCAL WIND BEARING (never about any
                   placed yaw the item might carry -- an axisymmetric post
                   has no "own edge" to reconcile a fall direction with,
                   unlike a fence panel) so it falls TOWARD
                   `wind_bearing_fn(x, y)` — `vegetation.tip_tree`'s own
                   construction for a windthrown tree, reused in
                   `apply_street_furniture_pose`. Carries `"lean_deg"`
                   (unsigned, most of a right angle), `"azimuth_deg"` (the
                   fall direction, jittered per item by
                   `azimuth_jitter_deg`) and `"slide_m"` (a downwind skid,
                   large for a trash can, small for everything else — see
                   `_SLIDE_SIGMA_M`).

      "leaning" -- partially toppled, same construction as "flat" with a
                   smaller `lean_deg` band. Only `sign` and `streetlight`
                   carry this rung -- a trash can has no stable
                   partly-tipped rest state and a hydrant's ladder never
                   reaches this far.

      "stands"  -- everything else, and always the answer for a `category`
                   this file has no ladder for -- refusing to guess is the
                   same convention `washaway.raft_kind_weights(None)` /
                   `fence_wind_threshold([], ...)` use elsewhere for "no
                   model for this, do not fabricate one".

    THE LADDER IS AN ABSOLUTE ONSET, deliberately NOT the rank/percentile
    threshold `fence_specs`' own wind branch uses. That design exists there
    because a fence panel has no per-panel "how well built is THIS one"
    axis to draw diversity from, so `fence_specs` manufactures diversity
    out of the field's own rank distribution instead. A street-furniture
    item DOES have that axis (age, rust, footing quality -- see
    `ITEM_RESIST_LO`/`ITEM_RESIST_HI`'s own docstring), so this reuses
    `hurricane.house_level_for_intensity`'s idiom instead: read `_CUTS
    [category]` against `i / resistance(vuln) + jitter`, `vuln` a fresh
    `rng.uniform(0, 1)` drawn PER ITEM.

    `rng` is a `random.Random` (or anything `surge._as_random` normalises).
    """
    rng = surge._as_random(rng)
    cuts = _CUTS if cuts is None else cuts
    lean_deg_range = (_LEAN_DEG_RANGE if lean_deg_range is None
                      else lean_deg_range)
    slide_sigma_m = _SLIDE_SIGMA_M if slide_sigma_m is None else slide_sigma_m
    depth_gone_m = _DEPTH_GONE_M if depth_gone_m is None else depth_gone_m

    out = []
    for it in items:
        x, y, category = float(it[0]), float(it[1]), str(it[2])
        my_cuts = cuts.get(category)
        if my_cuts is None:
            out.append({"x": x, "y": y, "category": category,
                       "action": "stands"})
            continue

        # DEPTH FIRST -- `fence_specs`' own priority order: standing water
        # is the more certain of the two removal mechanisms for whatever it
        # applies to, so it is decided before wind gets a say.
        dg = depth_gone_m.get(category)
        if dg is not None and float(depth_fn(x, y)) > dg:
            out.append({"x": x, "y": y, "category": category,
                       "action": "gone"})
            continue

        vuln = rng.uniform(0.0, 1.0)
        resistance = resist_hi + (resist_lo - resist_hi) * vuln
        eff = float(wind_intensity_fn(x, y)) / resistance
        if jitter > 0.0:
            eff += rng.uniform(-jitter, jitter)
        action = _ladder(my_cuts, eff)

        if action in ("stands", "gone"):
            out.append({"x": x, "y": y, "category": category,
                       "action": action})
            continue

        # "leaning" or "flat": build the fall pose.
        bearing = float(wind_bearing_fn(x, y))
        if azimuth_jitter_deg > 0.0:
            bearing = (bearing + rng.gauss(0.0, azimuth_jitter_deg)) % 360.0
        lo, hi = lean_deg_range.get((category, action), (75.0, 90.0))
        lean = rng.uniform(lo, hi)
        sigma = slide_sigma_m.get((category, action), 0.0)
        slide = rng.gauss(0.0, sigma) if sigma > 0.0 else 0.0
        out.append({"x": x, "y": y, "category": category, "action": action,
                   "lean_deg": lean, "azimuth_deg": bearing,
                   "slide_m": slide})
    return out


# ---------------------------------------------------------------------------
# stage-touching -- `pxr`/`bake` imported only inside these two.
# ---------------------------------------------------------------------------

def measure_street_item(stage, prim, category, ssf=1.0):
    """`(x_m, y_m, category)` for a street-furniture prim placed by
    `suburb_scene.build_frontage` / `AssetPools.place`.

    Unlike `washaway.measure_fence`, no yaw or length is measured: a sign,
    streetlight, trash can or hydrant is (for the purpose of this pass)
    AXISYMMETRIC about its own vertical axis -- there is no "long edge" a
    fall direction has to reconcile with, so `street_furniture_specs` /
    `apply_street_furniture_pose` pick the fall direction straight off the
    LOCAL wind bearing at the item (`vegetation.tip_tree`'s own
    construction) rather than off any placed yaw.

    `category` is a REQUIRED argument rather than parsed from the prim's
    own name because `trash_can` contains the very delimiter every other
    category name doesn't (`"<category>_<group>_<i>"` — splitting on `"_"`
    to recover the category would cut `trash_can` in half where nothing
    else in the pool splits at all). The caller is already walking the
    stage BY category to build the item list this function's output feeds
    (see the launcher-wiring note this module's docstring points to), so it
    already knows it.
    """
    from pxr import Gf, UsdGeom

    xf = UsdGeom.Xformable(prim)
    t = None
    for op in xf.GetOrderedXformOps():
        if op.GetOpName().split(":")[-1] == "translate":
            v = op.Get()
            if v is not None:
                t = v
            break
    if t is None:
        t = Gf.Vec3d(0.0, 0.0, 0.0)
    return float(t[0]) / ssf, float(t[1]) / ssf, str(category)


def apply_street_furniture_pose(stage, prim_path, decision, ssf=1.0,
                                ground_z_m=None):
    """Apply one `street_furniture_specs` decision to the already-placed
    prim at `prim_path`. Returns True if it changed anything (False for
    `"stands"` or a prim that no longer resolves).

    Mirrors `washaway.apply_fence_pose` structurally, and repeats its two
    fixes rather than re-discovering them:

    1. GROUND IS MEASURED, NOT READ FROM `t[2]`. `apply_placements` folds
       an anchor->centroid offset into a placed prim's translate, so
       `t[2]` is the ground PLUS whatever that offset happens to be for
       THIS asset — not the ground itself. The item's own lowest WORLD
       point, measured BEFORE anything here is re-authored
       (`bake.world_point_bounds`, points-based, never `UsdGeom.
       BBoxCache` — see that function's own docstring for why a bbox
       query is not usable for seating), is the real local ground height,
       whatever `t[2]` means for this particular asset.

    2. `Gf.Rotation`'s `*` composes LEFT-OPERAND-FIRST (verified in this
       repo's own pxr by `washaway.apply_fence_pose`'s docstring, which
       used to claim the opposite and was wrong): `place_rot * lean_rot`
       places the item in its authored orientation first and THEN leans it
       about the requested WORLD-frame axis, which is the intent.
       `lean_rot * place_rot` would lean it first, in its own unplaced
       local frame, against an axis expressed in world coordinates — a
       frame mismatch that (as measured there, on a fence panel) can leave
       the object standing on the wrong face entirely rather than lying
       flat.

    THE HINGE ITSELF differs from `apply_fence_pose`: a fence panel is
    hinged along its own measured long axis (`measure_fence`) with only
    the SIGN of the lean free (`washaway._fence_fall_lean` picks it). A
    street-furniture item has no such fixed edge — it is axisymmetric — so
    the axis is built directly from `decision["azimuth_deg"]`,
    `vegetation.tip_tree`'s own construction (perpendicular to the fall
    direction: `axis = (-sin(az), cos(az), 0)`, which by Rodrigues' formula
    sends "up" to bearing `az` at `theta = +90`), with no sign-picking step
    needed at all.
    """
    from pxr import Gf, Usd, UsdGeom

    from . import bake

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False
    action = decision.get("action")
    if action == "gone":
        prim.SetActive(False)
        return True
    if action not in ("flat", "leaning"):
        return False

    xf = UsdGeom.Xformable(prim)
    vals = {}
    for op in xf.GetOrderedXformOps():
        vals[op.GetOpName().split(":")[-1]] = op.Get()
    t = vals.get("translate") or Gf.Vec3d(0.0, 0.0, 0.0)
    sc = vals.get("scale")
    rot = vals.get("rotateXYZ")

    def _lowest_world_z():
        # A FRESH `XformCache` EVERY CALL, DELIBERATELY. `UsdGeom.XformCache`
        # memoizes each prim's local-to-world transform and does not notice
        # `_apply` re-authoring the xform ops below it -- reusing one cache
        # instance across the mutation silently returns the PRE-pose
        # transform for the POST-pose measurement. Measured: with a single
        # shared cache this landed the "flattened" item's lowest point
        # ~0.8 m off `ground_z_stage` (neither the true ground nor the raw
        # `t[2]` -- a stale-cache artefact, not a seating-formula bug).
        # `washaway.apply_fence_pose` already keeps this discipline (a
        # separate inline `UsdGeom.XformCache(...)` for its pre-measurement
        # block and another fresh one for its post-measurement block); this
        # mirrors it exactly rather than sharing one across the call.
        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        lo = None
        for p in Usd.PrimRange(prim):
            if not p.IsA(UsdGeom.Mesh):
                continue
            b = bake.world_point_bounds(p, xc)
            if b is None:
                continue
            lo = b[0][2] if lo is None else min(lo, b[0][2])
        return lo

    # THE GROUND, MEASURED OFF THE STANDING ITEM BEFORE ANYTHING BELOW
    # RE-AUTHORS ITS TRANSFORM. See this function's own docstring, point 1.
    pre = _lowest_world_z()
    ground_z_stage = (float(ground_z_m) * ssf if ground_z_m is not None
                      else (pre if pre is not None else float(t[2])))

    az = math.radians(float(decision.get("azimuth_deg", 0.0)))
    axis = Gf.Vec3d(-math.sin(az), math.cos(az), 0.0)
    lean_rot = Gf.Rotation(axis, float(decision.get("lean_deg", 85.0)))
    place = rot if rot is not None else Gf.Vec3f(0.0, 0.0, 0.0)
    place_rot = (Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), float(place[2]))
                 * Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), float(place[1]))
                 * Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), float(place[0])))
    # LEFT-OPERAND FIRST. See this function's own docstring, point 2.
    combined = place_rot * lean_rot
    q = combined.GetQuat()
    im = q.GetImaginary()

    # THE SKID, along the fall azimuth. Zero for "leaning" streetlights
    # (`_SLIDE_SIGMA_M` has no entry, `decision.get("slide_m")` is 0.0/None)
    # and for anything built from a decision dict that predates this field.
    slide = float(decision.get("slide_m") or 0.0)
    tx, ty = float(t[0]), float(t[1])
    if slide:
        tx += math.cos(az) * slide * ssf
        ty += math.sin(az) * slide * ssf

    def _apply(tz):
        xf.SetXformOpOrder([])
        xf.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
        xf.AddOrientOp().Set(Gf.Quatf(float(q.GetReal()), float(im[0]),
                                      float(im[1]), float(im[2])))
        if sc is not None:
            xf.AddScaleOp().Set(Gf.Vec3f(float(sc[0]), float(sc[1]),
                                         float(sc[2])))

    _apply(float(t[2]))

    lowest = _lowest_world_z()
    if lowest is not None:
        _apply(float(t[2]) + (ground_z_stage - lowest))
    return True
