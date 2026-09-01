"""hurricane_flow — `wind_flow` with the ladder resolved at the BOTTOM and a
real heading instead of a shrug.

Why this is its own module and not a flag on `wind_flow`: the same argument
`wind_flow.py:3-8` makes about `damage_flow`. That header calls an
`if tornado:` running through the middle of `damage_building` "the kind of
change that quietly breaks the wildfire path six months from now" — and
`wind_flow.wreck_building` is exactly as fragile to an `if hurricane:` today.
A hurricane needs states `wind_flow` never had to resolve (see below) and a
directional bias `wind_flow` explicitly declined to guess at
(`wind_flow.py:216-224`); bolting both onto the tornado module would mean
every future tornado retune has to first prove it did not perturb a hurricane
scene it was never looking at.

THE LADDER IS EXTENDED AT THE BOTTOM, NOT THE TOP
--------------------------------------------------
`wind_flow.BREAK_PLAN`'s floor is `roof_stripped` — "covering AND sheathing
peeled off; structure and walls whole" — which is already fairly severe
damage. Two measurements say a hurricane almost never gets past that floor
and spends nearly all of its scenes BELOW it:

  * Marshall (2004), 8,119 post-Katrina residences: 90% lost <20% of roof
    cover; 10% lost most of the cover and/or some decking; 3 of 8,119
    (1 in 2,700) lost large sections of roof STRUCTURE.
  * For 116-135 mph gusts (~Cat 2-3 local): "less than 15 percent of homes
    sustained structural wind damage."

So `BREAK_PLAN` here inserts three new rungs — `shingles_lost`, `cover_lost`,
`deck_panels_lost` — strictly BELOW `roof_stripped`, and reuses `leveled` and
everything above it from `wind_flow` UNCHANGED (imported, not copied, so a
retune of the shared levels propagates). `swept` is not reachable at all; see
`wreck_building`'s guard.

THE PROGRESSION IS ROOF-DOWN — Marshall (2004) again: "dismantling of a
structure by wind usually develops first at roof level and progresses
downward with stronger wind velocities... because wind velocity increases
with height." Never author wall damage on a house whose roof is intact — the
three new states touch roof coverings and openings ONLY, never a wall.

THE NEW STATES ARE NEARLY FREE, AND FRACTURE WOULD BE WRONG FOR THEM
---------------------------------------------------------------------
`detail/modular_house.py` roofs a plan in per-block UNITS: one `Roof_01` per
fully-covered 10x10 block, one `Roof_Half_01` per leftover 5 m column
(`modular_house.py:721-727`), each landing as its OWN prim
(`apply_placements`, `scene_generator.py:4161`). Losing a course of shingles
does not turn a panel to rubble — it exposes what is under it cleanly gone or
cleanly there — so dropping a bay wholesale with `SetActive(False)`
(`strip_roof`) is both CHEAPER and MORE HONEST than fracturing a fraction of
it would be: `fracture.fracture_prim` would manufacture ragged broken edges
no gust below `roof_stripped` (which genuinely tears the deck apart) has
earned. Windows are the same story one rung up: a window is a whole WALL
PIECE with the pane baked into the mesh (`modular_house.WINDOWS_5`), so
"blown out" is a reference swap (`blow_out_windows`) to the kit's own
window-less panel — the same stand-in `damage._fragment_assets` already
hands to a fractured wall (`damage.py:271-277`) — not a break.

THE FAILURE IS ORIENTED, AND NOW IT CAN BE
-------------------------------------------
`wind_flow.py:216-224` picks its broken walls with a plain `rng.sample`
because, in its own words, "the honest model [is] a random draw over the
whole set... which at least does not encode the WRONG mechanism" — it has no
heading to work with. A hurricane's wind bearing is a known field
(`hurricane_wind_field.md` §2.3's `wind_bearing_deg`, the compass bearing the
wind blows TOWARD), and ASCE 7 Table 26.13-1 says the resulting damage is
ASYMMETRIC: a breached opening triples the internal pressure (GCpi
+-0.18 -> +-0.55) and the failure that follows shows up on the BREACH side,
not uniformly. `windward_walls` and `strip_roof`'s `seed_dir` both encode
that the wind hits one side of the house before any other — without editing
a line of `wind_flow.py` (see `_bias_wall_pool` for how `wreck_building`
gets `wind_flow`'s own hard-pick to respect the bearing anyway).

SWEPT IS SURGE, NEVER WIND — Roueche et al. on 3,016 hurricane homes: every
one of the 94 (3.1%) that suffered complete destruction was tied to storm
surge, not wind alone. EF FR12 DOD10 puts wind-only slab-sweeping at 200 mph
— above Cat 5 and unreachable at a suburban site. `wreck_building` refuses
`level="swept"` outright; `disaster.surge` owns any bare-slab state for this
disaster.

Like `wind_flow.py`, this module imports `pxr` only inside the functions
that need it — a bare `import disaster.hurricane_flow` must not require a
USD build on a host that does not have one.
"""

import math
import random
import re

from . import wind_flow

# ---------------------------------------------------------------------------
# The ladder
# ---------------------------------------------------------------------------
# Same 7-tuple SHAPE as `wind_flow.BREAK_PLAN`
# (n_walls, partial_p, seeds, cut_range, n_floors, roof_seeds, consume), kept
# uniform so any shared helper that reads `BREAK_PLAN[level]` generically
# still gets something well-formed. The three new rows are all-zero because
# none of them go through `wind_flow`'s fracture path at all — `wreck_building`
# dispatches them to `strip_roof` / `blow_out_windows` / `_blow_doors`
# instead, and the real tuning knobs for those three are `_ROOF_FRAC` below,
# not this table.
#
# `roof_stripped` through `leveled` are the SAME VALUES `wind_flow` uses,
# referenced rather than retyped, so a retune of the shared levels there
# propagates here automatically.
BREAK_PLAN = {
    "pristine":         wind_flow.BREAK_PLAN["pristine"],
    # Cat 1-ish. <20% of shingle cover gone (Marshall: 90% of homes land
    # here). No fracture, no windows, no doors -- see `_ROOF_FRAC`.
    "shingles_lost":    (0, 0.00, 0, (0.00, 0.00), 0, 0, 0.00),
    # Cat 1-2. >20% cover and the soffit/siding with it; this is where ASCE
    # 7's envelope-breach threshold sits (~96-97 mph for openings against
    # 97 mph for "cover_major"), so doors go here too.
    "cover_lost":       (0, 0.00, 0, (0.00, 0.00), 0, 0, 0.00),
    # Cat 2-3. Most bays gone, most windows with them. Still no fracture --
    # what is left standing is still whole modules, just fewer of them.
    "deck_panels_lost": (0, 0.00, 0, (0.00, 0.00), 0, 0, 0.00),
    # Cat 2-3+. NO LONGER `wind_flow.BREAK_PLAN["roof_stripped"]` (2026-08-31)
    # -- see the note above `_ROOF_FRAC`. It is now `strip_roof(frac=1.0)`
    # like the three rungs above it: every bay SetActive(False), no fracture,
    # no settle. All-zero for the same reason those three are: `wreck_building`
    # dispatches it through `_ROOF_FRAC`, not through `wind_flow`'s path.
    "roof_stripped":    (0, 0.00, 0, (0.00, 0.00), 0, 0, 0.00),
    "roof_collapsed":   wind_flow.BREAK_PLAN["roof_collapsed"],
    "partial_collapse": wind_flow.BREAK_PLAN["partial_collapse"],
    "leveled":          wind_flow.BREAK_PLAN["leveled"],
    # NOT "swept". See `wreck_building`'s guard and the module docstring --
    # a hurricane does not reach it; `disaster.surge` does.
}

# (roof bay drop fraction, _Win_ wall blow-out fraction) for the three states
# below `wind_flow`'s floor. Both climb together because they are the same
# physical cause (rising suction / rising internal pressure), read off the
# skill's onset table:
#   shingles_lost    ~ "cover_partial" <20% shingles, 79 mph / 35 m/s
#   cover_lost       ~ "cover_major" >20% + "openings", 96-97 mph / 43 m/s
#   deck_panels_lost ~ "deck_major" >25% sheathing, 115-140 mph / 51-63 m/s
# The BAY is the finest resolution a whole-mesh `SetActive` can offer (each
# one covers ~50-100 m2, `modular_house._ROOF_COVER`), which is coarser than
# a literal "20% of shingles" -- these fractions are picked so the roughly
# 2-4 bay roofs this kit builds still land closest to the right BUCKET
# (a couple gone / about half gone / most gone), not to the literal percent.
# Window fraction is 0 at `shingles_lost`: openings do not fail below the
# ASCE breach threshold, which onsets with `cover_lost`, not before it.
# GENTLED 2026-08-30 AFTER THE FIRST RENDER, because these numbers did not
# match the definitions sitting right above them in `BREAK_PLAN`.
#
# `deck_panels_lost` is defined as "1-3 sheathing panels off (1-2 bays)" and
# was implemented as 0.80 — eighty percent of the roof. On the kit house that
# is a gutted roof, and it is why the user's first look at a Cat-2 plate was
# "why are all the roofs gone?".
#
# THE GEOMETRY IS THE REAL CONSTRAINT, and it is worth stating plainly: a kit
# house carries only about THREE roof bays (cottage 13 prims total). Three
# bays cannot express four gradations — any fraction that rounds to one bay
# is already a third of the roof. So the light end of this ladder is honest
# about that instead of pretending: `shingles_lost` drops NO bay at all.
# That is not a cop-out. Under 20% shingle loss is genuinely invisible from
# 400 m up, and the level is still carried in the ground truth, where it is
# the thing that actually matters for a detector. Adding real shingle-scale
# damage needs a roof TEXTURE variant, not more bay-dropping.
#
# (roof-bay fraction, window fraction)
#
# `roof_stripped` JOINED THIS TABLE 2026-08-31. It used to be
# `wind_flow.BREAK_PLAN["roof_stripped"]` — the WHOLE roof fractured into 10
# pieces and thrown/settled by PhysX, which is the tornado bake's own
# mechanism and inherits its two defects for a hurricane: (a) the 25-min
# settle the OSMO pod's PhysX GPU never actually engages for (see the bake
# launcher's bug catalogue entry), and (b) fragments that ship FROZEN IN
# MID-AIR when the settle is cut short — exactly what the coordinator's
# review found floating over `shoreline_obl.png`. But `roof_stripped`'s own
# definition ("covering AND sheathing peeled off; STRUCTURE AND WALLS
# WHOLE") is IDENTICAL in kind to `deck_panels_lost` one rung down — just
# more of it — and that rung already proved `strip_roof(frac=1.0)` is both
# cheaper and more honest for this: nothing here has earned a ragged
# fracture edge, and "all bays gone, walls untouched" is exactly what
# `SetActive(False)` on every bay gives for free. Window fraction 0.45 sits
# above `deck_panels_lost`'s 0.30 because more of the envelope is breached
# by the time every bay is gone.
_ROOF_FRAC = {
    "shingles_lost":    (0.00, 0.00),
    "cover_lost":       (0.34, 0.08),   # ~1 bay of 3
    "deck_panels_lost": (0.55, 0.30),   # ~2 bays of 3 — its "1-3 panels"
    "roof_stripped":    (1.00, 0.45),   # every bay
}

# Every level in `_ROOF_FRAC` that can drop AT LEAST ONE bay needs a rafter
# lattice authored under the hole it leaves — `shingles_lost` never drops a
# bay (see the comment above `_ROOF_FRAC`) so it is excluded; the other three
# all can. Read by `wreck_building` to decide whether to call
# `author_rafters` after `strip_roof`.
RAFTER_LEVELS = ("cover_lost", "deck_panels_lost", "roof_stripped")

# The three rungs authored by POSE, not by `wind_flow`'s fracture+settle path
# — see `pose_roof_collapsed` / `pose_partial_collapse` / `pose_leveled` and
# the module docstring's "TOP RUNGS, NO PHYSICS" section. `wreck_building`
# dispatches these BEFORE it ever reaches the legacy fracture branch.
_POSE_LEVELS = ("roof_collapsed", "partial_collapse", "leveled")

# ---------------------------------------------------------------------------
# JOB A tuning (2026-08-31 second pass) -- see `_squash_and_tilt`'s docstring
# for WHY a scale replaced the pure hinge: measured on
# `~/hurricane_previews/ROUND1_L3/collapsed_house_obl.png`, a `roof_collapsed`
# brick house's whole roof read as a rigid lid propped open above the intact
# walls, gable end pointing at the sky -- a pure rotation of one stiff plate
# cannot flatten a pitch or drop a ridge, only a Z-SCALE toward the eave
# plane can, and every numeric range below is named so the acceptance table
# in `test_hurricane_house_pose_math.py` locks them instead of a future edit
# silently drifting the look back toward a lid.
ROOF_COLLAPSE_SCALE_RANGE = (0.30, 0.50)      # roof height kept, about the eave
ROOF_COLLAPSE_TILT_RANGE_DEG = (5.0, 12.0)    # SMALL -- the scale does the work
# The NOMINAL tilt range above interacts multiplicatively with each mesh's
# own footprint (an 11 m-wide `ROOF_GABLE` swings its far edge by
# `11 * sin(angle)`, several metres at the top of that range) -- this budget
# is the MAX extra vertical excursion `_squash_and_tilt` lets the tilt add,
# auto-shrinking the effective angle on a wide bay well below the nominal
# range so it does not eat the drop/cap windows below. See `_squash_and_
# tilt`'s own docstring for the measured failure this replaced.
ROOF_COLLAPSE_TILT_HEIGHT_BUDGET_M = 0.15
ROOF_COLLAPSE_DROP_RANGE_M = (0.0, 0.6)       # below the bay's own eave/wall-top
ROOF_COLLAPSE_HEIGHT_FRAC = 0.75              # cap: max z <= this * pristine ridge

PARTIAL_ROOF_SCALE_RANGE = (0.30, 0.50)
PARTIAL_ROOF_TILT_RANGE_DEG = (6.0, 16.0)
PARTIAL_ROOF_TILT_HEIGHT_BUDGET_M = 0.15
PARTIAL_ROOF_DROP_RANGE_M = (0.0, 0.6)
# How close a wall/door/roof piece's own min-Y has to sit to the house's
# overall min-Y to count as "in the windward row". 0.5 m (the original
# value) misses `villa` (single hip-roof mesh, measured gap 0.625 m -- its
# own base inset from the wall footprint, no separate porch-row bay) and
# `terrace` (measured gap 0.626 m) entirely, silently leaving BOTH styles'
# `partial_collapse` roof untouched at that end while still racking their
# walls -- a wall down with a full-height roof still sitting on top of it.
# 0.7 clears both measured gaps with room, and the next-nearest gap in the
# whole 8-style measurement is 1.334 m (`terrace`'s second roof piece), so
# there is no risk of a bump this small pulling in the wrong row.
PARTIAL_ROW_TOL_M = 0.7
PARTIAL_WALL_RACK_RANGE_DEG = (70.0, 90.0)
PARTIAL_WALL_MAX_Z_M = 1.2   # HEIGHT ABOVE ITS OWN RESTING FLOOR, not an
                             # absolute world Z -- an upper-storey wall
                             # racks onto its OWN (still-standing) floor,
                             # which can itself be several metres up
                             # (`_floor_for`); "1.2 m" caps how TALL the
                             # fallen wall stands above that floor.

# THE UPPER-STOREY FLOOR SLAB ITSELF (JOB A REVIEW FIX, 2026-08-31 third
# pass). Racking the racked-row WALLS down was not enough on a multi-storey
# style: measured on `house_l_family_leveled.usd`, `house_floor` still
# spanned z [0.00, 3.51] after every wall was flattened to <= 0.6 m -- the
# UPPER floor slab (the thing those walls used to hold up) was never posed
# at all, so it hung in mid-air at its pristine 3.51 m with nothing left
# under it. Unlike `PARTIAL_WALL_MAX_Z_M`, this cap IS absolute: a floor
# slab that has lost its support is not "resting on its own storey" the way
# an unaffected wall elsewhere in the house still is -- it is falling, and
# `partial_collapse` only ever touches the slab over the RACKED end (the
# rest of the house, and its floors, stand).
PARTIAL_FLOOR_SCALE_RANGE = (0.5, 0.8)
PARTIAL_FLOOR_TILT_RANGE_DEG = (3.0, 8.0)
PARTIAL_FLOOR_TILT_HEIGHT_BUDGET_M = 0.10
PARTIAL_FLOOR_MAX_Z_M = 1.2   # ABSOLUTE -- see the comment above.
PARTIAL_FLOOR_REST_RANGE_M = (0.1, 0.5)
# `house_floor` is laid out on a COARSER grid than the wall/roof pieces --
# measured 5x5 m blocks vs. the roof's ~2-3 m bays -- so the nearest floor
# piece's own min-Y sits 1.334 m from the house's overall min-Y on every
# multi-storey style measured (`l_family`, `two_storey`, `wide_house`,
# `terrace`), well past `PARTIAL_ROW_TOL_M` (0.7). Re-using that tolerance
# for floors would silently match NO floor piece at all. The next-nearest
# floor block sits at 6.334 m, so anything in (1.334, 6.334) is safe.
PARTIAL_FLOOR_ROW_TOL_M = 2.0

LEVELED_ROOF_SCALE_RANGE = (0.12, 0.30)       # JOB A: "z-scaled <= 0.3"
LEVELED_ROOF_TILT_RANGE_DEG = (5.0, 15.0)
LEVELED_ROOF_TILT_HEIGHT_BUDGET_M = 0.10      # tighter -- the absolute 2.5 m
                                              # cap below has less slack
LEVELED_ROOF_MAX_Z_M = 2.5                    # absolute cap, every style
LEVELED_ROOF_REST_RANGE_M = (0.3, 0.6)        # above GROUND, on the fallen walls
LEVELED_WALL_RACK_RANGE_DEG = (80.0, 95.0)
LEVELED_WALL_MAX_Z_M = 0.6
LEVELED_WALL_REST_M = 0.05

# EVERY FLOOR ABOVE THE LOWEST ONE, ONTO THE PILE -- same review fix as
# `PARTIAL_FLOOR_*` above, for the fully-collapsed rung: at `leveled` there
# is no "unaffected end" left, so every upper-storey `house_floor` slab
# (identified the same way `pose_leveled`'s docstring already identifies an
# upper WALL -- its own floor level is above `ground_z`) comes down, not
# just the one over a racked row.
LEVELED_FLOOR_SCALE_RANGE = (0.5, 0.8)
LEVELED_FLOOR_TILT_RANGE_DEG = (3.0, 8.0)
LEVELED_FLOOR_TILT_HEIGHT_BUDGET_M = 0.10
LEVELED_FLOOR_MAX_Z_M = 1.2
LEVELED_FLOOR_REST_RANGE_M = (0.1, 0.5)

DAMAGED_LEVELS = ("shingles_lost", "cover_lost", "deck_panels_lost",
                  "roof_stripped", "roof_collapsed", "partial_collapse",
                  "leveled")

# Reused, not redefined: a hurricane cannot move anything wind_flow's own
# tornado ladder could not, and there is no hurricane-specific exception to
# that list (0 wind-moved cars below ~130 mph gusts is a PLACEMENT question
# for `disaster.hurricane`, not an immovability one).
IMMOVABLE = wind_flow.IMMOVABLE
is_immovable = wind_flow.is_immovable


# ---------------------------------------------------------------------------
# Recovering a placement's category from a LIVE prim
# ---------------------------------------------------------------------------
# `apply_placements` never stores `category` as an attribute
# (`scene_generator.py:4161`): a piece's leaf prim name literally IS
# `f"{category}_{group}_{i}"`, with `group`/`i` always plain integers. Trim
# the trailing pair and `damage._sub_of` reads the result exactly the way it
# reads a placement dict's `category` field -- no attribute walk needed.
_TAIL_RE = re.compile(r"_\d+_\d+$")


def _category_of(prim):
    return _TAIL_RE.sub("", prim.GetName())


# ---------------------------------------------------------------------------
# Roof bays
# ---------------------------------------------------------------------------
def roof_bay_prims(house_prim):
    """The per-bay roof meshes under one house's root prim.

    `modular_house.build_building` roofs a plan in per-block units -- one
    `Roof_01` per fully-covered 10x10 block, one `Roof_Half_01` per leftover
    5 m column (`:721-727`) -- and each is its own prim, a direct child of
    the house's root scope (the `{parent}/inst/h_{i}` per-house scoping
    `suburb_tornado_launch_script.py:361,370` already uses). That per-piece-
    ness is what makes partial roof loss nearly free elsewhere in this file.

    Deliberately EXCLUDES the porch/bay CAP (`Outer_Wall_Quart_Roof_01`,
    category suffix `bay_roof`, placed over any protruding bay window or
    porch) -- it has no field-coverage role of its own
    (`modular_house._ROOF_COVER` does not count it either), and dropping it
    opens a hole over a porch rather than shortening the roof.

    Includes inactive bays (a second call after a previous strip pass still
    sees the whole set) -- callers that only want what is still up should
    filter on `.IsActive()` themselves, as `strip_roof` does.
    """
    from pxr import Usd

    from . import damage

    out = []
    for prim in Usd.PrimRange(house_prim, Usd.PrimAllPrimsPredicate):
        if prim == house_prim or not prim.IsValid():
            continue
        if damage._sub_of(_category_of(prim)) == "roof":
            out.append(prim)
    return out


def strip_roof(house_prim, frac, rng, seed_dir=None):
    """Deactivate a share of the house's roof bays. NO FRACTURE.

    See the module docstring for why fracture would be both slower and wrong
    for this: the kit's own per-bay meshes already give the clean
    gone-or-there look a lost shingle course or sheathing panel actually has.
    `frac` is one of `_ROOF_FRAC`'s roof numbers (`shingles_lost` a few bays,
    `cover_lost` about half, `deck_panels_lost` most) -- `roof_stripped`
    itself is NOT reached through this function; it stays `wind_flow`'s full
    fracture of every roof module, dispatched from `wreck_building`.

    `seed_dir`, when given, is `wind_bearing_deg` -- the compass bearing the
    wind blows TOWARD (`hurricane_wind_field.md` section 2.3's convention).
    Real roof loss starts at the windward eave and corner and works inward
    (Marshall 2004), so bays are ranked by how far each one's own WORLD
    position sits toward where the wind is COMING FROM, and the drop is
    drawn from the most-exposed slice of the roof rather than uniformly
    across it -- a random scatter reads as hail, not wind, and a street
    where every house drops bays on the same side reads as one storm.
    Falls back to a plain, unbiased draw when `seed_dir` is None, same as
    `wind_flow`'s own documented fallback for walls.

    Returns the number of bays deactivated.
    """
    from pxr import Usd, UsdGeom

    bays = [b for b in roof_bay_prims(house_prim) if b.IsActive()]
    if not bays or frac <= 0.0:
        return 0
    k = max(1, min(len(bays), int(round(frac * len(bays)))))

    if k >= len(bays):
        chosen = bays
    elif seed_dir is None:
        chosen = rng.sample(bays, k)
    else:
        target = math.radians((float(seed_dir) + 180.0) % 360.0)
        ux, uy = math.sin(target), math.cos(target)   # bearing -> (E, N)

        def _score(b):
            m = UsdGeom.Imageable(b).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())
            t = m.ExtractTranslation()
            return t[0] * ux + t[1] * uy

        ranked = sorted(bays, key=_score, reverse=True)
        # THE EXPOSED 60%, not an exact top-k every time -- tuned, not
        # sourced, and for the same reason `_bias_wall_pool` gives walls
        # slack: an exact top-k pick makes every house of one style lose the
        # identical bays, which reads as a template rather than a storm.
        pool_n = max(k, int(math.ceil(len(ranked) * 0.6)))
        chosen = rng.sample(ranked[:min(pool_n, len(ranked))], k)

    for b in chosen:
        b.SetActive(False)
    return len(chosen)


# ---------------------------------------------------------------------------
# Openings
# ---------------------------------------------------------------------------
def blow_out_windows(house_prim, frac, rng):
    """Swap a share of window walls for the plain 5 m panel. NO FRACTURE.

    A window in this kit is not its own prim -- it is a whole WALL PIECE
    with the pane baked into the mesh (`modular_house.WINDOWS_5`). "Blown
    out" is therefore a REFERENCE SWAP: clear the wall's own reference and
    point it at `WALL_5["plain"]` (`Outer_Wall_Quart_01`) -- the exact
    window-less stand-in `damage._fragment_assets` already hands to a
    FRACTURED wall (`damage.py:271-277`) because it still reads as a piece
    of the house. No new prims, no fracture.

    A live prim carries no `category` attribute to filter candidates on, so
    they are found by their CURRENT reference target instead: every window
    variant's asset name contains `_Win_` (`modular_house.WINDOWS_5`,
    including the kit's own lowercase-q `Outer_Wall_quart_Win_01`), so that
    substring on the authored reference is the whole test.

    Returns the number of walls swapped.
    """
    from detail import modular_house as mh

    if frac <= 0.0:
        return 0
    plain_usd = mh._usd(mh.WALL_5["plain"])

    cands = []
    for prim in _live_children(house_prim):
        items = _authored_reference_items(prim)
        if any("_Win_" in (r.assetPath or "") for r in items):
            cands.append(prim)
    if not cands:
        return 0

    k = max(1, min(len(cands), int(round(frac * len(cands)))))
    chosen = cands if k >= len(cands) else rng.sample(cands, k)
    for prim in chosen:
        refs = prim.GetReferences()
        refs.ClearReferences()
        refs.AddReference(plain_usd)
        prim.Load()
    return len(chosen)


def _blow_doors(house_prim):
    """Every door and garage door gone. Deactivated, not fractured -- the
    loose leaf is already accounted for by the plank field once `wind_flow`
    picks up from `roof_stripped`, exactly as `wind_flow.py:306-320` does it
    unconditionally further up its own ladder.

    Unconditional (no `frac`) rather than a fractional draw, because ASCE 7
    Table 26.13-1's internal-pressure jump is a THRESHOLD event: an opening
    either breaches or it does not, and once one does the whole envelope is
    repressurised, not a graded fraction of it.
    """
    from . import damage

    n = 0
    for prim in _live_children(house_prim):
        if damage._sub_of(_category_of(prim)) in ("door", "door_slot"):
            prim.SetActive(False)
            n += 1
    return n


def _live_children(house_prim):
    """Active descendants of *house_prim*, itself excluded."""
    from pxr import Usd

    for prim in Usd.PrimRange(house_prim, Usd.PrimAllPrimsPredicate):
        if prim != house_prim and prim.IsValid() and prim.IsActive():
            yield prim


def _authored_reference_items(prim):
    """A prim's own authored reference list, or `[]`. Never raises.

    `prim.GetMetadata("references")` composes to an `Sdf.ReferenceListOp`;
    `GetAddedOrExplicitItems()` is the accessor `fire_bake.py:997` and
    `nucleus_catalogue.py:130` already use to read it back as a plain list
    of `Sdf.Reference`, each with an `.assetPath`.
    """
    try:
        refs = prim.GetMetadata("references")
    except Exception:
        return []
    return refs.GetAddedOrExplicitItems() if refs else []


# ---------------------------------------------------------------------------
# TOP RUNGS, NO PHYSICS -- pose-authoring `roof_collapsed` / `partial_
# collapse` / `leveled` directly on a baked archetype, offline, bare pxr
# ---------------------------------------------------------------------------
# WHY THIS EXISTS. The three rungs above `roof_stripped` used to be
# `wind_flow.wreck_building`'s fracture path (10-16 pieces per module) run
# through `settle.run` — which is a TORNADO signature (a fan of roof
# triangles and wall shards heaped on the lawn), not a hurricane one. Marshall
# (2004) again: at Cat 2-3 "less than 15 percent of homes sustained
# structural wind damage," and even that minority does not look like a
# tornado debris field — the shell stands, the roof structure sags or drops
# INTO the footprint, and a wall that fails goes over as a RIGID PANEL, not a
# spray of studs. A pose (a rigid transform edit to a piece that already
# exists) says that correctly and costs nothing: no fracture, no rigid body,
# no settle, no PhysX GPU that the OSMO pod's own container never actually
# engages for (see the bake launcher's bug catalogue — the same reason
# `_ROOF_FRAC` exists at all).
#
# THE TECHNIQUE. Every mesh `bake.export_object` writes carries EXACTLY ONE
# `xformOp:transform` (measured: `house_roof_7_12`'s is
# `scale(0.01) -> translate(0,0,3.5)`, world-baked already) — so a pose is
# `op.Set(old_matrix * hinge)`, never a second op stacked on top of the first
# (a NEW xformOp:transform on a prim that already has one raises with an
# EMPTY exception string, the same bug `washaway`'s duplicate-op fix
# documents). `hinge` rotates about a WORLD-SPACE pivot line: translate the
# pivot to the origin, rotate, translate back — `_hinge_matrix`.
#
# The hinge EDGE is picked from the piece's own measured bounding box at its
# base (z near its minimum), not from a hardcoded "ridge runs along Y"
# assumption — `_bbox_of` reads the piece's raw points once per call.
# Measured on `house_cottage_pristine.usd`'s `house_roof_7_12`: the ridge
# (21 points within 5 cm of the peak z=7.877) clusters at x +-0.04 spanning
# the FULL y range (+-5.71) — i.e. the ridge runs along LOCAL Y at x=0 for
# this kit, gable ends at the y extremes — but nothing here assumes that
# holds for every style, and a rigid rotation about ANY of the four base
# edges is a physically legitimate hinge regardless (a roof plane can also
# tip over sideways at a gable-end edge in a real failure).
#
# EVERY POSE IS SETTLED TO A DELIBERATE TARGET HEIGHT AFTERWARD
# (`_settle_to` — see `_floor_top_z`/`_floor_levels`), using `bake.
# world_point_bounds`-style point transforms, NEVER `UsdGeom.BBoxCache`
# (`fix-floating-debris/SKILL.md`'s whole point: a BBoxCache reading is the
# AABB of an AABB and can report a piece grounded while it hangs metres in
# the air). `_settle_to` ALWAYS repositions to its target rather than only
# correcting an overshoot — see its own docstring for the measured bug that
# forced this: an outer-eave hinge's OWN lever-arm arithmetic can swing a
# wide roof bay's far corner down by more than the bay's whole height, and
# reactively lifting the rigid piece to fix that dragged its near (hinge)
# edge into the sky along with it.
#
# `GABLE-END WALL PIECES` -- NOT A SEPARATE PRIM IN THIS KIT. The skill's
# ladder table says "gable-end wall pieces (if identifiable by name)
# deactivated or laid outward" for `roof_collapsed`. Measured: they are not
# identifiable by name because they do not exist as a name. `modular_house.
# apply_palette`'s own comment says it first — "the brick that appeared only
# at the top of a house was never the wall — it is `Brick_01_Dirt` on the
# GABLE SUBSET of `Roof_01` itself." The gable infill is a GeomSubset inside
# the single roof mesh, not a wall prim under this category, so there is
# nothing to deactivate or lay outward independently of the roof piece it is
# part of without splitting that mesh by subset — NOT done here (it would be
# more fracture-shaped work for a feature the hinge rotation already reads
# as "the roof structure failed," and every wall this file's `roof_collapsed`
# leaves alone is left alone deliberately, per the ladder's own "other walls
# standing" contract). Recorded here so the gap is not mistaken for an
# oversight on the next pass.


def _bbox_of(matrix, pts):
    """World-space `(xmin, xmax, ymin, ymax, zmin, zmax)` for `pts` (a mesh's
    raw `points` array, LOCAL space) under `matrix`. Point-based, like
    `bake.world_point_bounds` -- never `UsdGeom.BBoxCache`."""
    from pxr import Gf

    lo = [1e30, 1e30, 1e30]
    hi = [-1e30, -1e30, -1e30]
    for p in pts:
        w = matrix.Transform(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
        for k in range(3):
            v = float(w[k])
            if v < lo[k]:
                lo[k] = v
            if v > hi[k]:
                hi[k] = v
    return lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]


def _single_transform_op(prim):
    """The prim's own lone `xformOp:transform`, or `None`.

    Every piece `bake.export_object` writes carries exactly one of these
    (measured — see the section docstring). `None` means this prim is not
    poseable this way (a Scope, a GeomSubset host with no xform of its own,
    or a piece authored with a translate/rotate/scale stack instead of one
    combined matrix) and callers skip it rather than guess at a stack order.
    """
    from pxr import UsdGeom

    ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
    if len(ops) != 1 or ops[0].GetOpType() != UsdGeom.XformOp.TypeTransform:
        return None
    return ops[0]


def _local_points(prim):
    from pxr import UsdGeom

    mesh = UsdGeom.Mesh(prim)
    return mesh.GetPointsAttr().Get() if mesh else None


def _hinge_matrix(pivot, axis, angle_deg):
    """Rotate `angle_deg` about the WORLD-SPACE line through `pivot` parallel
    to `axis` -- translate to the origin, rotate, translate back. Applied as
    `old_matrix * hinge` (row-vector convention: `old_matrix` already maps
    local mesh points to their CURRENT world position; post-multiplying adds
    a further world-space transform on top of that, exactly the technique
    `tools/bake_hurricane_trees.py` uses for its own scale*rotate*translate
    product)."""
    from pxr import Gf

    rot = Gf.Rotation(Gf.Vec3d(*axis), float(angle_deg))
    to_origin = Gf.Matrix4d().SetTranslate(Gf.Vec3d(*(-c for c in pivot)))
    back = Gf.Matrix4d().SetTranslate(Gf.Vec3d(*pivot))
    return to_origin * Gf.Matrix4d().SetRotate(rot) * back


def _floor_top_z(house_prim):
    """The highest world Z any `house_floor*` mesh under `house_prim`
    reaches -- the resting surface every pose in this section clamps against.
    Falls back to 0.0 (world ground) if the house has no floor mesh at all,
    which should not happen on a `modular_house` build but must not crash a
    pose pass if it somehow does."""
    from pxr import Usd, UsdGeom

    zmax = None
    for prim in Usd.PrimRange(house_prim, Usd.PrimAllPrimsPredicate):
        if prim == house_prim or not prim.IsA(UsdGeom.Mesh):
            continue
        if _category_of(prim) != "house_floor":
            continue
        pts = _local_points(prim)
        op = _single_transform_op(prim)
        if not pts or op is None:
            continue
        _, _, _, _, _, hz = _bbox_of(op.Get(), pts)
        zmax = hz if zmax is None else max(zmax, hz)
    return 0.0 if zmax is None else zmax


def _floor_levels(house_prim):
    """Sorted, de-duplicated list of every STOREY's own floor-top Z under
    `house_prim` -- `[0.28]` for a single-storey cottage, `[0.28, 3.78]` for
    a two-storey villa (`modular_house.STOREY_M = 3.5` apart).

    `_floor_top_z` (the TOPMOST of these) is right for clamping a ROOF pose
    -- a roof only ever rests on the ceiling of the building's OWN top
    storey, however many there are below it. It is WRONG for clamping a
    WALL pose on a multi-storey house: a ground-floor wall's own floor is
    the ground floor, not whatever the tallest storey happens to be, and
    using a single global value pushed every lower-storey wall pose up to
    the height of the top floor -- measured as a `below_slab` false
    positive in `tools/hurricane_house_pose_bake.py`'s own verification
    audit on every multi-storey style (`l_family`, `terrace`, `two_storey`,
    `wide_house`) before this existed. `_floor_for` picks the right one of
    these per wall.
    """
    from pxr import Usd, UsdGeom

    zs = set()
    for prim in Usd.PrimRange(house_prim, Usd.PrimAllPrimsPredicate):
        if prim == house_prim or not prim.IsA(UsdGeom.Mesh):
            continue
        if _category_of(prim) != "house_floor":
            continue
        pts = _local_points(prim)
        op = _single_transform_op(prim)
        if not pts or op is None:
            continue
        _, _, _, _, _, hz = _bbox_of(op.Get(), pts)
        zs.add(round(hz, 3))
    return sorted(zs)


def _floor_for(levels, base_z, tol=0.5):
    """The floor level in `levels` that `base_z` (a piece's OWN, PRE-POSE
    base Z) actually stands on: the highest level at or below `base_z +
    tol`, or the lowest level if `base_z` sits below every floor (should not
    happen), or `0.0` if the house has no floor mesh at all."""
    if not levels:
        return 0.0
    below = [z for z in levels if z <= base_z + tol]
    return max(below) if below else min(levels)


def _squash_matrix(z0, sz):
    """World-space Z-only scale by `sz` about the horizontal plane `z=z0` --
    translate the plane to the origin, scale Z, translate back. Points
    already ON the plane (`z == z0`) are fixed; everything above it moves
    toward the plane by `sz`. Same to-origin/back technique as
    `_hinge_matrix`, just a scale instead of a rotation in the middle."""
    from pxr import Gf

    down = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, 0.0, -z0))
    scale = Gf.Matrix4d().SetScale(Gf.Vec3d(1.0, 1.0, sz))
    back = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, 0.0, z0))
    return down * scale * back


def _squash_and_tilt(prim, rng, scale_range, tilt_range_deg, edge=None,
                     tilt_height_budget_m=0.15):
    """Squash one roof-bay MESH toward its own EAVE PLANE (a Z-only scale
    about its pre-pose zmin, drawn from `scale_range`) and add a SMALL tilt
    (`tilt_range_deg`) about one of its own four base edges -- REPLACES
    `_hinge_bay_down` (removed 2026-08-31, second pass), which rotated the
    whole rigid roof plate as one stiff panel and could not flatten a pitch
    or drop a ridge, only tip the plate -- measured on
    `~/hurricane_previews/ROUND1_L3/collapsed_house_obl.png`: a `roof_
    collapsed` roof reading as a rigid lid propped open above intact walls,
    its gable end pointing at the sky, not a roof structure that had failed.
    A structure that actually fails drops its ridge AND flattens its pitch,
    which only a scale toward the base can do to a single rigid mesh; the
    small tilt on top is cosmetic asymmetry, not the mechanism.

    THE TILT ANGLE IS CAPPED BY A HEIGHT BUDGET, NOT SAMPLED RAW. A ROOF
    BAY'S LEVER ARM DOMINATES A "SMALL" DEGREE FIGURE. `ROOF_GABLE` is
    measured ~11 m across; hinging THAT WIDE a plate by even a modest
    `tilt_range_deg` draw (5-12) swings the opposite base edge down by
    `width * sin(angle)` -- at 11 m and 12 degrees, ~2.3 m, several times
    the roof-collapse acceptance window's own 0.6 m drop slack, and it
    reappears with the wrong sign at `partial_collapse`'s cap check too
    (measured on the FIRST cut of this function, before the budget: every
    style failed at least one "settled min outside [wall_top-0.6,
    wall_top]" or "max_z > pristine ridge" check, by up to several metres --
    the exact same lever-arm mistake `_settle_to`'s own docstring already
    documents for the OLD pure-hinge code, reintroduced here by sampling an
    angle without checking what it does to a wide mesh). Capping the angle
    so `width_along_the_tilt_axis * sin(angle) <= tilt_height_budget_m`
    keeps the tilt genuinely cosmetic on a wide bay (an 11 m roof gets a
    fraction of a degree) while still landing close to the nominal
    `tilt_range_deg` on a narrow one (a ~2 m porch/bay cap easily clears
    5-12 degrees within the same budget) -- the SAME physical asymmetry,
    scaled to what each mesh's own footprint can absorb without eating the
    drop/cap windows the squash is sized against.

    THE TILT SIGN IS CHOSEN FROM THE REAL POINTS, NOT A PROXY. `_hinge_bay_
    down`'s sign came from ONE synthetic test point at
    `(far_corner_xy, zmax)` -- not an actual point on a gable mesh (the far
    corner's REAL z is zmin, the eave, not the ridge) -- and for a symmetric
    gable, picking the sign from a point that does not exist on the mesh
    silently chose the sign that LIFTS the true far eave instead of
    lowering it, which is exactly the propped-lid bug above. This evaluates
    BOTH signs against the mesh's OWN full point set (post-squash) and keeps
    whichever gives the smaller resulting max Z -- correct by construction,
    not by convention, and it costs two `_bbox_of` passes over a roof mesh
    that is at most a few hundred points.

    Returns a dict (`op`, `pts`, `new`, `orig_zmin`, `orig_zmax`, `scale`)
    for `_settle_to` to finish with, or `None` if `prim` is not poseable
    (see `_single_transform_op`) or its own Z extent is degenerate (a flat
    mesh has no eave plane to squash toward).
    """
    op = _single_transform_op(prim)
    if op is None:
        return None
    pts = _local_points(prim)
    if not pts:
        return None
    old = op.Get()
    xmin, xmax, ymin, ymax, zmin, zmax = _bbox_of(old, pts)
    if zmax - zmin < 1e-4:
        return None

    sz = rng.uniform(*scale_range)
    squashed = old * _squash_matrix(zmin, sz)

    e = edge or rng.choice(("xmin", "xmax", "ymin", "ymax"))
    if e in ("xmin", "xmax"):
        axis = (0.0, 1.0, 0.0)
        pivot = (xmin if e == "xmin" else xmax, 0.5 * (ymin + ymax), zmin)
        lever_arm = xmax - xmin
    else:
        axis = (1.0, 0.0, 0.0)
        pivot = (0.5 * (xmin + xmax), ymin if e == "ymin" else ymax, zmin)
        lever_arm = ymax - ymin

    raw_angle = rng.uniform(*tilt_range_deg)
    if tilt_height_budget_m is not None and lever_arm > 1e-6:
        cap_deg = math.degrees(math.asin(
            min(1.0, tilt_height_budget_m / lever_arm)))
        angle = min(raw_angle, cap_deg)
    else:
        angle = raw_angle
    best_sign, best_new, best_max = 1.0, squashed, None
    for sign in (1.0, -1.0):
        cand = squashed * _hinge_matrix(pivot, axis, sign * angle)
        cand_max = _bbox_of(cand, pts)[5]
        if best_max is None or cand_max < best_max:
            best_max, best_new, best_sign = cand_max, cand, sign

    op.Set(best_new)
    return dict(op=op, pts=pts, new=best_new, edge=e, angle=best_sign * angle,
               orig_zmin=zmin, orig_zmax=zmax, scale=sz)


def _settle_to(info, target_min_z):
    """Translate `info` (from `_squash_and_tilt`, `_rack_wall_outward`, or an
    equivalent wall pose)
    so its lowest transformed point lands EXACTLY at `target_min_z` --
    ALWAYS, not merely when it overshoots below.

    THIS REPLACES AN EARLIER "clamp only if below" VERSION, and the reason
    is a real, measured bug in that version, worth recording. A roof bay's
    horizontal span (~11 m for `ROOF_GABLE`) is more than DOUBLE its own
    height (~4.6 m), so hinging the whole rigid bay through even the
    prescribed 15-35 degrees swings its FAR corner down by
    `far_dx * sin(angle)` -- at 35 degrees and an 11 m lever arm, close to
    6.4 m, more than the roof's own height. "Clamp only if below the floor"
    then lifted the WHOLE rigid piece by that overshoot to fix the far
    corner, dragging the NEAR (hinge) corner -- which was never wrong and
    should have stayed near its original wall-top height -- up into the
    sky with it. Measured before this fix: `cottage/leveled`'s roof reached
    10.65 m against a pristine peak of 7.877 m, and `wide_house/leveled`
    reached 15.36 m. Setting the resting height DIRECTLY, independent of
    whatever the rotation's own lever-arm arithmetic produced, is immune to
    this: the rotation controls the TILT, this controls the HEIGHT, and the
    two no longer fight each other through a reactive lift. Point-based
    (`_bbox_of`), never `UsdGeom.BBoxCache`.
    """
    from pxr import Gf

    lo_z = _bbox_of(info["new"], info["pts"])[4]
    dz = target_min_z - lo_z
    if abs(dz) > 1e-6:
        moved = info["new"] * Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, 0.0, dz))
        info["op"].Set(moved)
        info["new"] = moved
    return info


def _compress_to_span(info, max_span_m):
    """AFTER `_settle_to`: if `info`'s current world-Z span still exceeds
    `max_span_m`, squash it toward its OWN (already-settled) base so the
    span becomes exactly `max_span_m`. A no-op if it is already under.

    WHY THIS EXISTS. A racked WALL piece's rotation-angle range alone cannot
    guarantee a height cap the way `_squash_and_tilt`'s scale does for a
    roof: `_rack_wall_outward` rotates about whichever of the piece's own
    X/Y footprint spans is SHORTER, on the assumption that dimension is a
    thin wall thickness -- true for a plain panel, but measured FALSE for
    several styles' front-row "Outer_Wall_Quart" corner units, whose own
    short footprint dimension is 1.37-1.45 m, not the few-centimetre
    thickness the assumption expects. No rotation angle changes that
    dimension's OWN size, so even a perfect 90-degree rack leaves the piece
    standing ~1.4 m tall -- above both `PARTIAL_WALL_MAX_Z_M` (1.2) and
    `LEVELED_WALL_MAX_Z_M` (0.6). Squashing toward the base it was just
    settled onto (the base Z is the FIX point, so the resting height
    `_settle_to` set is preserved exactly) guarantees the cap regardless of
    which piece's own cross-section produced the overage.
    """
    lo, hi = _bbox_of(info["new"], info["pts"])[4:6]
    span = hi - lo
    if max_span_m is not None and span > max_span_m > 0.0:
        sz = max_span_m / span
        squashed = info["new"] * _squash_matrix(lo, sz)
        info["op"].Set(squashed)
        info["new"] = squashed
    return info


def _drop_floor(floor_mesh, rng, ground_z, scale_range, tilt_range_deg,
                tilt_height_budget_m, rest_range_m, max_z_m):
    """Squash+tilt+settle ONE `house_floor` mesh the same way a roof bay is
    posed (`_squash_and_tilt`), then clamp its resting height so its own
    max Z never exceeds the ABSOLUTE `max_z_m` -- a floor slab that has
    lost the wall(s) that held it up is not "resting on its own storey" the
    way an untouched wall elsewhere in the house still is; it is falling,
    all the way down, same as the debris under it.

    WHY THIS EXISTS. Measured on `house_l_family_leveled.usd` before this
    fix: every wall was correctly flattened to `LEVELED_WALL_MAX_Z_M`, but
    `house_floor` still spanned up to 3.51 m -- the upper-storey slab those
    walls used to hold up was never posed at all and hung in mid-air with
    nothing left under it, an obvious "floating slab" the world_point_
    bounds verify's own floating/below_slab checks do not catch (they only
    look at roof/bay_roof categories -- see `audit_file`'s docstring).

    Returns `True` if the mesh was posed, `False` if it was not poseable
    (see `_single_transform_op`) or degenerate (see `_squash_and_tilt`).
    """
    info = _squash_and_tilt(floor_mesh, rng, scale_range, tilt_range_deg,
                            tilt_height_budget_m=tilt_height_budget_m)
    if info is None:
        return False
    _, _, _, _, lo, hi = _bbox_of(info["new"], info["pts"])
    span = hi - lo
    target = ground_z + rng.uniform(*rest_range_m)
    target = max(ground_z, min(target, max_z_m - span))
    _settle_to(info, target)
    return True


def _house_bbox(house_prim, cats=("house_wall", "house_roof", "house_floor")):
    """World-space `(xmin, xmax, ymin, ymax, zmin, zmax)` over every mesh in
    `cats` under `house_prim` -- the whole footprint, used to find which end
    of the house is "the racked end" (`pose_partial_collapse`) or where its
    centroid is (`pose_leveled`)."""
    from pxr import Usd, UsdGeom

    lo = [1e30, 1e30, 1e30]
    hi = [-1e30, -1e30, -1e30]
    for prim in Usd.PrimRange(house_prim, Usd.PrimAllPrimsPredicate):
        if prim == house_prim or not prim.IsA(UsdGeom.Mesh):
            continue
        if _category_of(prim) not in cats:
            continue
        pts = _local_points(prim)
        op = _single_transform_op(prim)
        if not pts or op is None:
            continue
        xn, xx, yn, yx, zn, zx = _bbox_of(op.Get(), pts)
        for k, v in enumerate((xn, yn, zn)):
            if v < lo[k]:
                lo[k] = v
        for k, v in enumerate((xx, yx, zx)):
            if v > hi[k]:
                hi[k] = v
    return lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]


def _meshes_of(house_prim, category):
    from pxr import Usd, UsdGeom

    return [p for p in Usd.PrimRange(house_prim, Usd.PrimAllPrimsPredicate)
            if p != house_prim and p.IsA(UsdGeom.Mesh)
            and _category_of(p) == category]


def pose_roof_collapsed(stage, house_prim, rng, floor_top_z=None):
    """`roof_collapsed`, pose-authored: EVERY roof bay (main AND porch/bay
    caps -- JOB A, 2026-08-31 second pass: no roof mesh is left intact any
    more) is squashed toward its own eave plane
    (`ROOF_COLLAPSE_SCALE_RANGE`) with a small tilt on top
    (`ROOF_COLLAPSE_TILT_RANGE_DEG`, see `_squash_and_tilt`) and dropped so
    its lowest point lands within `ROOF_COLLAPSE_DROP_RANGE_M` below its OWN
    pre-pose eave (= where it used to rest on the wall top), clamped so its
    max Z never exceeds `ROOF_COLLAPSE_HEIGHT_FRAC` of its OWN pristine
    ridge and never drops below the house's own ground floor. No fracture,
    no wall changes -- see the section docstring for why gable-end infill is
    not handled separately (it is a GeomSubset inside the roof mesh, not a
    prim here).

    REPLACES THE PURE-HINGE VERSION (measured broken on
    `~/hurricane_previews/ROUND1_L3/collapsed_house_obl.png` -- a roof
    reading as a rigid lid propped open above intact walls): a hinge alone
    rotates one stiff plate and cannot flatten a pitch or drop a ridge; see
    `_squash_and_tilt`'s docstring for the mechanism and the sign-selection
    fix that goes with it.

    Matches `wind_flow.BREAK_PLAN["roof_collapsed"]`'s own intent ("roof
    structure down") more literally than the fracture path did: EVERY bay
    fails, not a fractured fraction of one, which is also why this needs no
    per-house wind-bearing bias -- an isotropic failure has no windward side
    to prefer, the same reasoning `wreck_building` already applies to
    `leveled`.

    Returns the number of bays posed.
    """
    levels = _floor_levels(house_prim)
    ground_z = min(levels) if levels else 0.0
    n = 0
    for bay in (_meshes_of(house_prim, "house_roof")
               + _meshes_of(house_prim, "house_bay_roof")):
        info = _squash_and_tilt(bay, rng, ROOF_COLLAPSE_SCALE_RANGE,
                                ROOF_COLLAPSE_TILT_RANGE_DEG,
                                tilt_height_budget_m=ROOF_COLLAPSE_TILT_HEIGHT_BUDGET_M)
        if info is None:
            continue
        z0 = info["orig_zmin"]      # this bay's OWN wall-top / eave height
        cap = ROOF_COLLAPSE_HEIGHT_FRAC * info["orig_zmax"]
        span = _bbox_of(info["new"], info["pts"])[5] \
            - _bbox_of(info["new"], info["pts"])[4]
        target = z0 - rng.uniform(*ROOF_COLLAPSE_DROP_RANGE_M)
        # NEVER punch the cap (a shallow drop on a small `scale` draw can
        # still poke a high-elevation upper-storey roof above its own
        # `HEIGHT_FRAC` line -- measured on `l_family`/`two_storey`/`wide_
        # house`'s upper gable, whose absolute eave height is a much bigger
        # share of its own ridge height than a single-storey roof's) AND
        # NEVER below this house's own ground floor.
        target = max(ground_z, min(target, cap - span))
        _settle_to(info, target)
        n += 1
    return n


def pose_partial_collapse(stage, house_prim, rng, floor_top_z=None,
                          tol=PARTIAL_ROW_TOL_M):
    """`partial_collapse`, pose-authored: the windward END of the house --
    every wall piece sitting in the row nearest the archetype's own local
    MIN-Y edge -- racks 70-90 degrees outward about its own base edge, and
    the roof bay(s) reaching that row are squashed toward their own eave
    plane and tilted toward the racked end (`_squash_and_tilt`, hinged at
    the FAR/max-y edge so the affected end drops while the rest of the bay
    stays close to its original pose). Doors in the racked row are
    deactivated (they go with the wall). The rest of the shell stands.

    THE ROOF TREATMENT CHANGED (JOB A, 2026-08-31 second pass) from a pure
    10-25 degree hinge to `_squash_and_tilt` for the same reason `pose_roof_
    collapsed` did: measured, a hinge-only droop could leave the affected
    bay's OWN max Z above the house's pristine ridge (a stiff plate tipped
    on an edge raises its far side before the `_settle_to` translation ever
    gets to lower it) -- backwards for "the roof comes down to wall-top
    level". A squash-then-tilt cannot exceed its own pristine height by
    construction (the scale only ever multiplies the height range by
    `PARTIAL_ROOF_SCALE_RANGE`, both factors < 1), so this bay never reads
    as taller than the standing majority of the roof it is attached to.

    WHY MIN-Y, AND WHY THAT IS ENOUGH TO BE "windward" WITHOUT A VARIANT PER
    HOUSE. Every archetype is baked at yaw 0 (`bake_hurricane_archetypes_
    launch_script.py`'s own `_BAKE_BEARING = 0.0` convention, which
    `modular_house.build_building` makes local -Y = south, its own front
    convention). `partial_collapse` is one of only two levels in `suburb_
    hurricane_launch_script._WIND_YAWED`, so the ASSEMBLY rotates the WHOLE
    placed house to `hurricane.wind_bearing_at(x, y)` rather than to the
    street. Racking a FIXED local side here and letting that placement-time
    rotation carry it is the same trick `wind_flow`'s own header describes
    for the fracture path's throw bias -- baking a direction into the
    archetype would be wasted, since the assembly re-aims the whole house
    anyway. Min-Y is chosen only for consistency with `_BAKE_BEARING`'s own
    convention, not because south is special.

    `tol` (metres) is how close to the house's own min-Y a wall/door/roof
    piece's own min-Y has to sit to count as "in that row" -- see
    `PARTIAL_ROW_TOL_M`'s own comment for the two styles (`villa`, single
    hip-roof mesh; `terrace`, row-house) whose own roof geometry sits
    farther inset from the wall footprint than the original 0.5 m default
    caught, which left THEIR racked end with a wall down and an untouched
    full-height roof still over it.

    Returns the number of wall pieces racked.
    """
    if floor_top_z is None:
        floor_top_z = _floor_top_z(house_prim)
    _, _, y_min, _, _, _ = _house_bbox(house_prim)
    levels = _floor_levels(house_prim)

    racked = 0
    for wall in _meshes_of(house_prim, "house_wall"):
        op = _single_transform_op(wall)
        pts = _local_points(wall)
        if op is None or not pts:
            continue
        wl = _bbox_of(op.Get(), pts)
        if wl[2] > y_min + tol:      # ymin of this wall -- not in the row
            continue
        # AND ENTIRELY IN IT, not just touching a corner of it. A true
        # front-row wall runs ALONG the row (its own Y-depth is a few tens
        # of centimetres); a SIDE wall spans the whole house depth and its
        # bounding box also happens to touch y_min at one end, which the
        # ymin-only check above cannot tell apart from actually being a
        # front-row piece. Excluding anything whose own Y-extent is more
        # than 2 m keeps side/corner walls out of the racked set.
        if (wl[3] - wl[2]) > 2.0:
            continue
        # THIS WALL'S OWN STOREY, not the topmost floor -- a ground-floor
        # wall on a two-storey house racks down onto the GROUND floor, not
        # up onto the floor above it (`_floor_levels`'s docstring).
        wall_floor = _floor_for(levels, wl[4])
        racked += _rack_wall_outward(wall, rng, wall_floor)

    for cat_name in ("house_door", "house_door_slot"):
        for door in _meshes_of(house_prim, cat_name):
            op = _single_transform_op(door)
            pts = _local_points(door)
            if op is None or not pts:
                continue
            if _bbox_of(op.Get(), pts)[2] <= y_min + tol:
                door.SetActive(False)

    for bay in _meshes_of(house_prim, "house_roof"):
        op = _single_transform_op(bay)
        pts = _local_points(bay)
        if op is None or not pts:
            continue
        if _bbox_of(op.Get(), pts)[2] > y_min + tol:   # does not reach the row
            continue
        info = _squash_and_tilt(bay, rng, PARTIAL_ROOF_SCALE_RANGE,
                                PARTIAL_ROOF_TILT_RANGE_DEG, edge="ymax",
                                tilt_height_budget_m=PARTIAL_ROOF_TILT_HEIGHT_BUDGET_M)
        if info is None:
            continue
        z0 = info["orig_zmin"]
        cap = info["orig_zmax"]   # NEVER above this bay's own pristine ridge
        _, _, _, _, lo, hi = _bbox_of(info["new"], info["pts"])
        span = hi - lo
        target = z0 - rng.uniform(*PARTIAL_ROOF_DROP_RANGE_M)
        target = max(floor_top_z, min(target, cap - span))
        _settle_to(info, target)

    # THE UPPER-STOREY FLOOR SLAB OVER THE RACKED END ONLY (JOB A review
    # fix). `house_floor` sits on a COARSER grid than the wall/roof pieces
    # (`PARTIAL_FLOOR_ROW_TOL_M`'s own comment), so it needs its own,
    # larger, row tolerance -- reusing `tol` here would match no floor
    # piece on any multi-storey style. Ground-floor slabs (`_floor_for`
    # maps to `ground_z`) are never touched: "the rest of the house
    # stands" includes the ground floor under the racked wall, which is
    # still there.
    ground_z = min(levels) if levels else 0.0
    for floor in _meshes_of(house_prim, "house_floor"):
        op = _single_transform_op(floor)
        pts = _local_points(floor)
        if op is None or not pts:
            continue
        fb = _bbox_of(op.Get(), pts)
        if fb[2] > y_min + PARTIAL_FLOOR_ROW_TOL_M:   # not over the racked end
            continue
        if _floor_for(levels, fb[4]) <= ground_z + 1e-6:
            continue   # the ground floor itself -- stays put
        _drop_floor(floor, rng, ground_z, PARTIAL_FLOOR_SCALE_RANGE,
                   PARTIAL_FLOOR_TILT_RANGE_DEG,
                   PARTIAL_FLOOR_TILT_HEIGHT_BUDGET_M,
                   PARTIAL_FLOOR_REST_RANGE_M, PARTIAL_FLOOR_MAX_Z_M)
    return racked


def _rack_wall_outward(wall, rng, floor_top_z,
                       angle_range=PARTIAL_WALL_RACK_RANGE_DEG,
                       max_span_m=PARTIAL_WALL_MAX_Z_M):
    """One wall piece, racked about its own base edge (z at its own
    minimum). Length axis picked from whichever of X/Y the piece's own base
    footprint spans more -- `modular_house` walls run along one or the
    other depending on which side of the house they are on.

    THE SIGN CHECKS Z, NOT Y. An earlier version checked whether the test
    point's Y coordinate dropped, which is meaningless for the `yspan >=
    xspan` branch: that branch's rotation AXIS is `(0, 1, 0)`, and rotating
    about an axis never moves a point's coordinate ALONG that axis, so `y`
    was constant across both signs and the "choice" silently always fell
    through to `+1`. Z is affected by rotation about either X or Y, so
    checking it (does the wall's own top corner come DOWN) is correct for
    both branches and matches what "racked" should mean regardless of
    which way the wall runs.

    `max_span_m`, when given, calls `_compress_to_span` after settling --
    see that function's docstring for WHY: the axis this function treats as
    "thickness" (whichever of X/Y is shorter) is measured 1.37-1.45 m on
    several styles' front-row corner ("Outer_Wall_Quart") units, not the
    few-centimetre panel thickness the rotation-only approach assumes, so a
    perfect 90-degree rack alone still leaves the piece standing ~1.4 m
    tall on those units -- above `PARTIAL_WALL_MAX_Z_M`.
    """
    from pxr import Gf

    op = _single_transform_op(wall)
    pts = _local_points(wall)
    old = op.Get()
    xmin, xmax, ymin, ymax, zmin, zmax = _bbox_of(old, pts)
    xspan, yspan = xmax - xmin, ymax - ymin
    if xspan >= yspan:
        axis = (1.0, 0.0, 0.0)
        pivot = (0.5 * (xmin + xmax), ymin, zmin)
        test = Gf.Vec3d(0.5 * (xmin + xmax), ymin, zmax)
    else:
        axis = (0.0, 1.0, 0.0)
        pivot = (xmin, 0.5 * (ymin + ymax), zmin)
        test = Gf.Vec3d(xmin, 0.5 * (ymin + ymax), zmax)

    angle = rng.uniform(*angle_range)
    best_sign, best_z = 1.0, None
    for sign in (1.0, -1.0):
        z = _hinge_matrix(pivot, axis, sign * angle).Transform(test)[2]
        if best_z is None or z < best_z:
            best_z, best_sign = z, sign

    new = old * _hinge_matrix(pivot, axis, best_sign * angle)
    op.Set(new)
    # FLAT ON ITS OWN FLOOR -- see `pose_leveled`'s identical comment. The
    # 0.05 m rest margin comes back OUT of the compress budget so the
    # piece's own TOP lands at `floor_top_z + max_span_m` exactly (an
    # uncorrected budget would leave it at `floor_top_z + 0.05 +
    # max_span_m`, 0.05 m over the acceptance every time).
    rest = 0.05
    info = _settle_to(dict(op=op, pts=pts, new=new), floor_top_z + rest)
    _compress_to_span(info, max(0.05, max_span_m - rest)
                      if max_span_m is not None else None)
    return 1


def pose_leveled(stage, house_prim, rng, floor_top_z=None):
    """`leveled`, pose-authored: EVERY wall racks (`LEVELED_WALL_RACK_RANGE_
    DEG`, 80-95 degrees) about its own base edge, hinged on whichever edge
    sits FARTHER from the house's own centroid (so it falls outward all the
    way round rather than in one direction), and every roof piece (main
    roof AND porch/bay caps) is squashed toward its own eave plane
    (`LEVELED_ROOF_SCALE_RANGE`, <= 0.3) with a small tilt on top
    (`_squash_and_tilt`) and settles onto the fallen walls -- "walls down
    flat around the slab, roof LYING ON TOP," never a shard pile, and never
    a two-storey house whose upper walls end up resting at the height of a
    floor that, at this rung, has itself come down too.

    EVERY WALL GOES ALL THE WAY TO GROUND (JOB A, 2026-08-31 second pass),
    not to its own storey's floor. `pose_partial_collapse`'s "rack onto this
    wall's OWN storey" is right for a PARTIAL failure (the storeys below an
    unaffected end are still standing, so an upper wall racking onto its own
    intact floor is correct), but `leveled` is total: nothing is left
    standing to catch an upper-storey wall at 3.5 m, so it has to fall all
    the way to the house's own lowest floor (`ground_z`, `min(_floor_
    levels(...))`) like every ground-floor wall does. Using the per-storey
    reference here (the ORIGINAL version of this function) left every
    upper-storey wall on a two-storey style resting near its own floor's
    height -- e.g. ~3.5 m up on `l_family`/`terrace`/`two_storey`/`wide_
    house` -- which both fails the "every wall max z <= `LEVELED_WALL_MAX_
    Z_M`" acceptance and does not read as "resting on ground" at all.

    THE ROOF SCALE REPLACES THE OLD SHALLOW-HINGE-ONLY TRICK for the same
    reason `pose_roof_collapsed`/`pose_partial_collapse` changed: a rigid
    plate rotated about one edge, however shallow the angle, still has its
    own WIDTH projecting into height (`sin(angle) * width`), and past a
    modest angle that growth outpaces what the rotation lowers -- measured
    on an early cut of THIS function at 65-85 degrees, `cottage/leveled`'s
    roof topped out at 10.99 m and `l_family`'s at 16.2 m, taller than the
    INTACT roof. A z-scale toward the eave has no such failure mode: the
    scaled height range is bounded by construction (`scale < 1` always
    shrinks it), so a small residual tilt on top cannot blow it back up the
    way a bare hinge could. Doors are deactivated (irrelevant once the wall
    carrying them is down).

    No wind-bearing bias, matching `wind_flow`'s own comment for its fracture
    path: past `partial_collapse` every wall fails regardless of facing, so
    biasing a pool here would dress an isotropic collapse up as directional
    for nothing.
    """
    xmin, xmax, ymin, ymax, _, _ = _house_bbox(house_prim)
    cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
    levels = _floor_levels(house_prim)
    ground_z = min(levels) if levels else 0.0

    from pxr import Gf

    n = 0
    for wall in _meshes_of(house_prim, "house_wall"):
        op = _single_transform_op(wall)
        pts = _local_points(wall)
        if op is None or not pts:
            continue
        old = op.Get()
        wl_xn, wl_xx, wl_yn, wl_yx, wl_zn, wl_zx = _bbox_of(old, pts)
        xspan, yspan = wl_xx - wl_xn, wl_yx - wl_yn
        if xspan >= yspan:
            axis = (1.0, 0.0, 0.0)
            edge_y = wl_yn if abs(wl_yn - cy) > abs(wl_yx - cy) else wl_yx
            pivot = (0.5 * (wl_xn + wl_xx), edge_y, wl_zn)
            # THE TEST POINT'S Z MUST BE THE WALL'S OWN TOP (`wl_zx`), NOT
            # `wl_yx` (a Y-extent value with no business in a Z slot) --
            # that typo shipped once already here and put the "which sign
            # is outward" test at a height derived from the wall's LENGTH
            # instead of its own storey height, which happens to be close
            # enough in scale to often pick the right sign by accident and
            # wrong often enough to be worth calling out explicitly.
            test = Gf.Vec3d(0.5 * (wl_xn + wl_xx), edge_y, wl_zx)
        else:
            axis = (0.0, 1.0, 0.0)
            edge_x = wl_xn if abs(wl_xn - cx) > abs(wl_xx - cx) else wl_xx
            pivot = (edge_x, 0.5 * (wl_yn + wl_yx), wl_zn)
            test = Gf.Vec3d(edge_x, 0.5 * (wl_yn + wl_yx), wl_zx)

        angle = rng.uniform(*LEVELED_WALL_RACK_RANGE_DEG)
        best_sign, best_d = 1.0, None
        for sign in (1.0, -1.0):
            p = _hinge_matrix(pivot, axis, sign * angle).Transform(test)
            d = math.hypot(p[0] - cx, p[1] - cy)
            if best_d is None or d > best_d:
                best_d, best_sign = d, sign

        new = old * _hinge_matrix(pivot, axis, best_sign * angle)
        op.Set(new)
        # FLAT ON THE GROUND -- 80-95 degrees is nearly or fully flat, so
        # settle right at the house's own LOWEST floor plus a hair for the
        # wall's own thickness, regardless of which storey this wall came
        # from (see the docstring's "EVERY WALL GOES ALL THE WAY TO GROUND").
        info = _settle_to(dict(op=op, pts=pts, new=new),
                          ground_z + LEVELED_WALL_REST_M)
        # See `_compress_to_span`'s docstring: some front-row corner units'
        # own "thickness" axis measures 1.37-1.45 m, well above
        # `LEVELED_WALL_MAX_Z_M`, which no rotation angle alone changes.
        # `LEVELED_WALL_MAX_Z_M` IS an ABSOLUTE world-Z cap ("resting on
        # ground" -- unlike `PARTIAL_WALL_MAX_Z_M`, which is relative to a
        # wall's own, possibly-elevated, storey floor), so the SPAN budget
        # handed to `_compress_to_span` has to subtract back out the base
        # this piece is already resting on (`ground_z + LEVELED_WALL_REST_
        # M`) to land the piece's own TOP at the cap, not `max_z_m` more
        # metres above an already-nonzero base.
        _compress_to_span(info, max(0.05, LEVELED_WALL_MAX_Z_M - ground_z
                                    - LEVELED_WALL_REST_M))
        n += 1

    # EVERY FLOOR ABOVE THE LOWEST ONE, ONTO THE PILE -- see
    # `LEVELED_FLOOR_*`'s own comment: a `leveled` house has nothing left
    # standing to hold an upper-storey slab up, so every `house_floor` mesh
    # whose OWN pristine level is above `ground_z` gets the same squash+
    # tilt+settle treatment as the roof, capped at an ABSOLUTE
    # `LEVELED_FLOOR_MAX_Z_M`. The ground-floor slab(s) -- `_floor_for`
    # maps to `ground_z` itself -- are left alone; they are already the
    # base everything else is settling onto.
    for floor in _meshes_of(house_prim, "house_floor"):
        op0 = _single_transform_op(floor)
        pts0 = _local_points(floor)
        if op0 is None or not pts0:
            continue
        own_zmin = _bbox_of(op0.Get(), pts0)[4]
        if _floor_for(levels, own_zmin) <= ground_z + 1e-6:
            continue   # the ground floor itself -- stays put
        _drop_floor(floor, rng, ground_z, LEVELED_FLOOR_SCALE_RANGE,
                   LEVELED_FLOOR_TILT_RANGE_DEG,
                   LEVELED_FLOOR_TILT_HEIGHT_BUDGET_M,
                   LEVELED_FLOOR_REST_RANGE_M, LEVELED_FLOOR_MAX_Z_M)

    for bay in (_meshes_of(house_prim, "house_roof")
               + _meshes_of(house_prim, "house_bay_roof")):
        info = _squash_and_tilt(bay, rng, LEVELED_ROOF_SCALE_RANGE,
                                LEVELED_ROOF_TILT_RANGE_DEG,
                                tilt_height_budget_m=LEVELED_ROOF_TILT_HEIGHT_BUDGET_M)
        if info is None:
            continue
        _, _, _, _, lo, hi = _bbox_of(info["new"], info["pts"])
        span = hi - lo
        # "ROOF LYING ON TOP" of the flattened walls -- rest just above the
        # fallen-wall layer (`LEVELED_ROOF_REST_RANGE_M`), capped so the
        # WHOLE pile never exceeds `LEVELED_ROOF_MAX_Z_M` regardless of how
        # tall this bay's own pristine ridge used to be (the acceptance is
        # an ABSOLUTE height, not a fraction of the original -- a leveled
        # two-storey house reads the same height as a leveled cottage).
        target = ground_z + rng.uniform(*LEVELED_ROOF_REST_RANGE_M)
        target = max(ground_z, min(target, ground_z + LEVELED_ROOF_MAX_Z_M - span))
        _settle_to(info, target)

    for cat_name in ("house_door", "house_door_slot"):
        for door in _meshes_of(house_prim, cat_name):
            door.SetActive(False)
    return n


# ---------------------------------------------------------------------------
# The rafter lattice -- what makes a stripped roof read as a house
# ---------------------------------------------------------------------------
# Every level in `RAFTER_LEVELS` drops at least one roof bay to bare
# `SetActive(False)` and authors NOTHING in its place, which is the "empty
# box" the review found: walls with nothing inside, no rafters, no ceiling
# joists, and in some rungs a bare unbound floor slab under an open sky. A
# rafter lattice is the cheap fix -- merged boxes, one mesh per house,
# `planks`-style (8 points / 6 faces per box, faceVarying normals) -- and it
# is authored from the DROPPED BAY'S OWN measured geometry, not a fixed
# pitch, so it always matches the roof it replaces.
RAFTER_SPACING_M = 0.60
RAFTER_W_M, RAFTER_T_M = 0.05, 0.15
RIDGE_W_M, RIDGE_T_M = 0.06, 0.20

# ---------------------------------------------------------------------------
# RAGGED RAFTERS -- a perfectly regular cage reads as under-construction, not
# storm-stripped. Coordinator review of the shipped library:
# `stripped_roof_house_obl.png` -- "houses wear COMPLETE, perfectly regular
# rafter cages -- every rafter present, no sheathing remnants -- reads as
# under-construction rather than storm-stripped"; `shoreline_obl.png` --
# "several identical rafter cages in one block". These knobs break that
# regularity per BAY, seeded off the same `rng` every other draw in this
# file already consumes from -- same house, same seed, same break pattern
# every time `_rafter_specs_for_bay`/`author_rafters` run.
#
# THE RIDGE BOARD IS NEVER REMOVED OR SNAPPED. It is the thickest,
# most-continuously-fastened member in a real roof frame (nailed to every
# rafter pair, rather than resting on top of a wall plate the way a common
# rafter does) and Marshall's own progression (roof-DOWN, covering- then
# sheathing- then structure) has it failing last if at all, on anything
# short of `roof_collapsed`. It only ever gets the lighter "shreds"
# treatment below -- a scrap of decking still nailed to it, not a
# structural loss.
RAFTER_REMOVE_FRAC_RANGE = (0.15, 0.35)    # gone outright, no trace
RAFTER_SNAP_FRAC_RANGE = (0.10, 0.20)      # broken, still hanging
RAFTER_SNAP_KEEP_RANGE = (0.35, 0.70)      # fraction of length kept, from the
                                           # RIDGE end -- the free end is the
                                           # one that broke and dropped
RAFTER_SNAP_TILT_RANGE_DEG = (10.0, 25.0)  # EXTRA downward pitch on the
                                           # broken piece, on top of the
                                           # roof's own slope
RAFTER_SNAP_MAX_OVERSHOOT_M = 0.3          # how far PAST the original eave
                                           # the free end may droop -- see
                                           # `_snap_rafter`'s own docstring
                                           # for the measured below-slab bug
                                           # this caps (a shallow-pitched
                                           # style, a long `keep` draw and a
                                           # big tilt draw compounding to put
                                           # the free end underground)

# Sheathing debris still clinging to what is left of the frame -- irregular,
# torn-edged polygons, NOT the clean rectangular deck panel the kit itself
# would author. "Per roof" is read as per HOUSE (one `author_rafters` call
# covers every dropped bay in the house at once), not per bay -- a hurricane
# does not leave 1-3 patches on EVERY exposed bay, it leaves a few scraps
# total on whatever is left of the roof.
PATCH_COUNT_RANGE = (1, 3)
PATCH_SIZE_RANGE_M = (0.5, 2.0)        # across, the larger of the two radii
PATCH_ASPECT_RANGE = (0.5, 1.0)        # minor/major radius ratio -- never a
                                       # perfect circle
PATCH_VERTS_RANGE = (6, 9)
PATCH_JAG_FRAC = 0.35                  # per-vertex radius jitter, +-
PATCH_THICKNESS_M = 0.02

# Ridge shreds -- smaller, and anchored near the ridge (`t_range` close to
# 0 in `_patch_placement_on_bay`) rather than scattered down the slope; the
# aspect range is tighter so they read as long, narrow strips running along
# the ridge board, not another round patch.
RIDGE_SHRED_COUNT_RANGE = (2, 4)
RIDGE_SHRED_SIZE_RANGE_M = (0.3, 0.7)
RIDGE_SHRED_ASPECT_RANGE = (0.25, 0.5)
RIDGE_SHRED_VERTS_RANGE = (5, 7)


def _yaw_pitch_for_direction(dx, dy, dz):
    """`(yaw_deg, pitch_deg)` for `planks._box`'s `Rz(yaw) @ Ry(pitch)`
    convention such that the box's local +X axis, after rotation, points
    along the world direction `(dx, dy, dz)`.

    Derived directly from `_box`'s own matrix at `roll=0`: local +X maps to
    world `(cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), -sin(pitch))`. Solving
    for a target unit vector gives `yaw = atan2(dy, dx)`,
    `pitch = atan2(-dz, hypot(dx, dy))` -- verified against the measured
    `house_cottage_pristine.usd` ridge geometry (a rafter built for the
    y-axis-ridge case with these formulas lands within a few mm of the
    measured eave and ridge points).
    """
    horiz = math.hypot(dx, dy)
    yaw = math.degrees(math.atan2(dy, dx))
    pitch = math.degrees(math.atan2(-dz, horiz))
    return yaw, pitch


def _ridge_info(bay_prim):
    """Measure one roof bay's own pitch geometry from its RAW points --
    never assume "ridge runs along Y" holds for every style/bay.

    Returns `None` for a degenerate/flat mesh (z span under 0.3 m -- should
    never happen for a real roof bay, but a rafter lattice over nothing is
    worse than no lattice at all). Otherwise a dict with:

      `ridge_point(run)` / `eave_point(run, eave_coord)` -- 3-tuples in world
        space, parametrised along the ridge's own RUN axis (whichever of
        world X/Y the top-of-mesh points spread along more -- the ridge
        LINE's own direction, not a hardcoded axis);
      `run_lo/run_hi` -- the ridge's own extent along its run axis;
      `eave_lo/eave_hi` -- the two candidate eave positions on the
        PERPENDICULAR axis (one or both may be absent for a half-gable,
        filtered by `_rafter_specs_for_bay`);
      `ridge_coord`, `z_ridge`, `z_eave`.
    """
    op = _single_transform_op(bay_prim)
    pts = _local_points(bay_prim)
    if op is None or not pts:
        return None
    matrix = op.Get()
    world = [matrix.Transform(_gf_vec3d(p)) for p in pts]
    zs = [w[2] for w in world]
    zmin, zmax = min(zs), max(zs)
    if zmax - zmin < 0.3:
        return None
    top = [w for w in world if w[2] > zmax - 0.05]
    base = [w for w in world if w[2] < zmin + 0.05]
    txs, tys = [w[0] for w in top], [w[1] for w in top]
    xspread, yspread = max(txs) - min(txs), max(tys) - min(tys)
    axis_is_y = yspread >= xspread

    if axis_is_y:
        ridge_coord = sum(txs) / len(txs)
        run_lo, run_hi = min(tys), max(tys)
        bxs = [w[0] for w in base]
        eave_lo, eave_hi = min(bxs), max(bxs)

        def ridge_point(run):
            return (ridge_coord, run, zmax)

        def eave_point(run, eave_c):
            return (eave_c, run, zmin)
    else:
        ridge_coord = sum(tys) / len(tys)
        run_lo, run_hi = min(txs), max(txs)
        bys = [w[1] for w in base]
        eave_lo, eave_hi = min(bys), max(bys)

        def ridge_point(run):
            return (run, ridge_coord, zmax)

        def eave_point(run, eave_c):
            return (run, eave_c, zmin)

    return dict(ridge_point=ridge_point, eave_point=eave_point,
                run_lo=run_lo, run_hi=run_hi, eave_lo=eave_lo,
                eave_hi=eave_hi, ridge_coord=ridge_coord, z_ridge=zmax,
                z_eave=zmin)


def _gf_vec3d(p):
    from pxr import Gf

    return Gf.Vec3d(float(p[0]), float(p[1]), float(p[2]))


def _rafter_and_ridge_specs(info, spacing_m=RAFTER_SPACING_M,
                            w_m=RAFTER_W_M, t_m=RAFTER_T_M):
    """Pure geometry: `(common_rafter_specs, ridge_spec_or_None)` for one
    bay's `_ridge_info` -- rafters at `spacing_m` centres along the ridge's
    own run, one or two slopes (a half-gable only has one eave), plus a
    ridge board spanning the whole run. Pure geometry, no pxr.

    Kept SEPARATE from `_rafter_specs_for_bay` (which calls this and then
    optionally raggedizes the common rafters before reassembling) so a
    caller that needs to treat the ridge board differently from the common
    rafters -- `author_rafters`'s per-bay stat tally, `_raggedize_rafters`'s
    own "never touch the ridge" rule -- never has to guess which returned
    spec dict was the ridge by comparing widths."""
    run_lo, run_hi = info["run_lo"], info["run_hi"]
    ridge_coord = info["ridge_coord"]
    span = max(1e-6, run_hi - run_lo)
    n = max(2, int(round(span / spacing_m)))
    slopes = []
    if info["eave_lo"] < ridge_coord - 0.2:
        slopes.append(info["eave_lo"])
    if info["eave_hi"] > ridge_coord + 0.2:
        slopes.append(info["eave_hi"])

    specs = []
    for i in range(n + 1):
        run = run_lo + span * i / n
        rp = info["ridge_point"](run)
        for eave_c in slopes:
            ep = info["eave_point"](run, eave_c)
            dx, dy, dz = ep[0] - rp[0], ep[1] - rp[1], ep[2] - rp[2]
            length = math.sqrt(dx * dx + dy * dy + dz * dz)
            if length < 0.05:
                continue
            yaw, pitch = _yaw_pitch_for_direction(dx, dy, dz)
            specs.append({
                "class": "rafter",
                "x": 0.5 * (rp[0] + ep[0]), "y": 0.5 * (rp[1] + ep[1]),
                "z": 0.5 * (rp[2] + ep[2]), "l": length, "w": w_m, "t": t_m,
                "yaw": yaw, "pitch": pitch, "roll": 0.0})

    ridge_spec = None
    rp0, rp1 = info["ridge_point"](run_lo), info["ridge_point"](run_hi)
    dx, dy, dz = rp1[0] - rp0[0], rp1[1] - rp0[1], rp1[2] - rp0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length > 0.1:
        yaw, pitch = _yaw_pitch_for_direction(dx, dy, dz)
        ridge_spec = {
            "class": "rafter",
            "x": 0.5 * (rp0[0] + rp1[0]), "y": 0.5 * (rp0[1] + rp1[1]),
            "z": 0.5 * (rp0[2] + rp1[2]), "l": length, "w": RIDGE_W_M,
            "t": RIDGE_T_M, "yaw": yaw, "pitch": pitch, "roll": 0.0}
    return specs, ridge_spec


def _rafter_specs_for_bay(info, spacing_m=RAFTER_SPACING_M,
                          w_m=RAFTER_W_M, t_m=RAFTER_T_M, rng=None):
    """`planks.build`-ready spec dicts (`class/x/y/z/l/w/t/yaw/pitch/roll`)
    for one bay's `_ridge_info` -- see `_rafter_and_ridge_specs` for the
    geometry. Pure geometry, no pxr -- see `test_hurricane_house_pose_math.
    py` for the offline coverage this buys.

    `rng`, when given, breaks the regular cage per `_raggedize_rafters` --
    a share of the COMMON rafters (never the ridge board, see the module's
    "RAGGED RAFTERS" section) removed outright and another share snapped to
    a partial length with the free end drooping down. `rng=None` (the
    default) returns the exact, unmodified full cage this function has
    always produced -- every existing caller/test that does not pass `rng`
    sees byte-identical output.
    """
    common, ridge_spec = _rafter_and_ridge_specs(info, spacing_m, w_m, t_m)
    if rng is not None:
        common, _stats = _raggedize_rafters(common, rng, w_m=w_m, t_m=t_m)
    return common + ([ridge_spec] if ridge_spec is not None else [])


def _raggedize_rafters(rafter_specs, rng, w_m=RAFTER_W_M, t_m=RAFTER_T_M):
    """Break a regular rafter cage: remove `RAFTER_REMOVE_FRAC_RANGE` of
    *rafter_specs* outright (dropped from the returned list) and snap
    another `RAFTER_SNAP_FRAC_RANGE` to a partial length with the free end
    drooping down (`_snap_rafter`) -- the rest stay exactly as given.
    *rafter_specs* must not include the ridge board (`_rafter_specs_for_bay`
    keeps it separate; see the module's "RAGGED RAFTERS" section for why).

    SEEDED, not random: the fractions AND which specific rafters are picked
    both come from `rng`, so the SAME `rng` state (the per-house seed every
    other draw in this file already consumes from) always produces the SAME
    break pattern for the SAME input list.

    At least one rafter always survives untouched-or-snapped when the input
    is non-empty (a bay reduced to zero rafters reads as a bug in the
    lattice generator, not storm damage) -- see the `while` loop below.

    `w_m`/`t_m` are accepted for a uniform call signature with
    `_rafter_and_ridge_specs` but are not currently used -- a snapped
    rafter keeps its own original cross-section, which already came from
    whatever `w_m`/`t_m` built it.

    Returns `(new_specs, stats)`; `stats` has `kept`/`removed`/`snapped`
    counts (always summing to `len(rafter_specs)`). *rafter_specs* itself
    is never mutated.
    """
    n = len(rafter_specs)
    if n == 0:
        return [], dict(kept=0, removed=0, snapped=0)
    order = list(range(n))
    rng.shuffle(order)
    remove_frac = rng.uniform(*RAFTER_REMOVE_FRAC_RANGE)
    snap_frac = rng.uniform(*RAFTER_SNAP_FRAC_RANGE)
    n_remove = int(round(remove_frac * n))
    n_snap = int(round(snap_frac * n))
    while n_remove + n_snap >= n and (n_remove > 0 or n_snap > 0):
        if n_remove >= n_snap and n_remove > 0:
            n_remove -= 1
        elif n_snap > 0:
            n_snap -= 1
        else:
            break
    removed_idx = set(order[:n_remove])
    snapped_idx = set(order[n_remove:n_remove + n_snap])

    out = []
    for i, spec in enumerate(rafter_specs):
        if i in removed_idx:
            continue
        if i in snapped_idx:
            out.append(_snap_rafter(spec, rng))
        else:
            out.append(dict(spec))
    n_kept = n - len(removed_idx) - len(snapped_idx)
    return out, dict(kept=n_kept, removed=len(removed_idx),
                     snapped=len(snapped_idx))


def _snap_rafter(spec, rng, max_overshoot_m=RAFTER_SNAP_MAX_OVERSHOOT_M):
    """One rafter *spec*, SNAPPED: kept from its own RIDGE end only
    (`RAFTER_SNAP_KEEP_RANGE` of its original length), the free (eave) end
    tilted further DOWN by `RAFTER_SNAP_TILT_RANGE_DEG` on top of the roof's
    own slope -- a rafter that broke near the eave and is still hanging from
    the ridge, not one that vanished cleanly or floats in place.

    THE PIVOT IS THE SPEC'S OWN RIDGE-END POINT. `x/y/z` is the box's
    CENTRE (`_rafter_and_ridge_specs` builds every spec as the midpoint
    between a ridge point and an eave point), so the ridge end is
    `centre - 0.5*l*dir`, with `dir` the box's own +X world direction
    recovered from `yaw`/`pitch` by the SAME formula
    `test_yaw_pitch_roundtrips_through_box_rotation_convention` locks for
    `_yaw_pitch_for_direction`'s forward direction.

    New pitch is the old pitch PLUS the extra downward tilt: MORE positive
    pitch means MORE downward (`_yaw_pitch_for_direction`'s own convention,
    world z-component `-sin(pitch)`), so the broken end droops toward the
    floor rather than continuing along the original roof slope -- yaw is
    unchanged, so the break still points the right way across the roof.

    THE FREE END IS CLAMPED AGAINST DROPPING PAST THE ORIGINAL EAVE BY MORE
    THAN `max_overshoot_m`. A SHORTER piece (`RAFTER_SNAP_KEEP_RANGE` can be
    as low as 0.35) pitched MORE steeply (`RAFTER_SNAP_TILT_RANGE_DEG` up to
    25 deg extra) does not automatically stay above where the original
    rafter's own eave end was -- on a shallow-pitched style (measured:
    `terrace/cover_lost`'s `n`/`e` cardinal variants) a high `keep` draw
    combined with a large tilt draw swung the free end to -0.35 to -0.55 m,
    BELOW the house's own ground floor (`hurricane_house_pose_bake.audit_
    file`'s `below_slab` check, >30 cm under `ground_z`). Solved in closed
    form: given the fixed ridge-end pivot and `new_l`, the largest pitch
    that keeps the free end at or above `original_eave_z - max_overshoot_m`
    is `asin((ridge_end_z - floor_z) / new_l)` -- capped there, but never
    reduced below the UN-EXTENDED original pitch (a snap must droop AT
    LEAST as much as the intact slope, never less).
    """
    yaw, pitch, l = spec["yaw"], spec["pitch"], spec["l"]
    yr, pr = math.radians(yaw), math.radians(pitch)
    dirx, diry, dirz = (math.cos(yr) * math.cos(pr),
                        math.sin(yr) * math.cos(pr), -math.sin(pr))
    ridge_end = (spec["x"] - 0.5 * l * dirx, spec["y"] - 0.5 * l * diry,
                spec["z"] - 0.5 * l * dirz)
    original_eave_z = ridge_end[2] + l * dirz

    keep = rng.uniform(*RAFTER_SNAP_KEEP_RANGE)
    new_l = max(0.15, l * keep)
    tilt_pitch = pitch + rng.uniform(*RAFTER_SNAP_TILT_RANGE_DEG)

    # THE CLAMP ABOVE IS ON THE CENTRELINE, THE MESH IS A BOX. A box's own
    # cross-section (`spec["w"]` x `spec["t"]`) puts its lowest CORNER up to
    # `0.5*hypot(w, t)` further down than the centreline endpoint the clamp
    # above bounds -- measured: `terrace/cover_lost`'s low PORCH bay (own
    # eave at z ~ 0.00, i.e. already at ground) still put a snapped piece's
    # corner at -0.36 m against a centreline cap of exactly -0.30 m, the
    # residual being almost exactly that corner offset
    # (`0.5*hypot(0.05, 0.15)` ~= 0.079 m). Tighten the centreline budget by
    # that (plus a small flat buffer) so the RENDERED corner, not just the
    # line down its middle, respects `max_overshoot_m`.
    corner_margin = 0.5 * math.hypot(spec.get("w", RAFTER_W_M),
                                    spec.get("t", RAFTER_T_M)) + 0.02
    floor_z = original_eave_z - max(0.0, max_overshoot_m - corner_margin)
    sin_cap = (ridge_end[2] - floor_z) / new_l
    cap_pitch = math.degrees(math.asin(max(-1.0, min(1.0, sin_cap))))
    new_pitch = max(pitch, min(tilt_pitch, cap_pitch))

    npr = math.radians(new_pitch)
    ndirx, ndiry, ndirz = (math.cos(yr) * math.cos(npr),
                          math.sin(yr) * math.cos(npr), -math.sin(npr))

    out = dict(spec)
    out["x"] = ridge_end[0] + 0.5 * new_l * ndirx
    out["y"] = ridge_end[1] + 0.5 * new_l * ndiry
    out["z"] = ridge_end[2] + 0.5 * new_l * ndirz
    out["l"] = new_l
    out["pitch"] = new_pitch
    return out


# ---------------------------------------------------------------------------
# Ragged sheathing patches + ridge shreds -- irregular polygons `planks._box`
# (rectangular solids only) cannot produce
# ---------------------------------------------------------------------------
def _patch_placement_on_bay(info, rng, t_range=(0.15, 0.85)):
    """One random anchor point + local plane basis on a dropped bay's own
    roof slope, for a ragged sheathing patch or ridge shred. `u_axis` runs
    along the RIDGE (the run direction), `v_axis` runs DOWNSLOPE (ridge to
    eave), `n_axis` is the slope's own outward normal (`u_axis` cross
    `v_axis`) -- all pure tuple/float geometry off `_ridge_info`'s own
    closures, no pxr, so it can be unit-tested the same way
    `_rafter_and_ridge_specs` is.

    `t_range` is how far down the slope (0 = on the ridge, 1 = at the eave)
    the anchor lands -- callers pass a range near 0 for a ridge shred and
    the wider default for a field patch.

    Picks whichever eave side(s) this bay actually has (a half-gable has
    only one -- the same `eave_lo`/`eave_hi` test `_rafter_and_ridge_specs`
    already uses) and returns `None` if the bay has neither, which should
    not happen for anything `_ridge_info` accepted (it already required a
    real Z span) but must not crash a caller looping over several bays.
    """
    ridge_coord = info["ridge_coord"]
    sides = []
    if info["eave_lo"] < ridge_coord - 0.2:
        sides.append(info["eave_lo"])
    if info["eave_hi"] > ridge_coord + 0.2:
        sides.append(info["eave_hi"])
    if not sides:
        return None

    eave_c = rng.choice(sides)
    run = rng.uniform(info["run_lo"], info["run_hi"])
    t = rng.uniform(*t_range)
    rp = info["ridge_point"](run)
    ep = info["eave_point"](run, eave_c)
    center = tuple(rp[k] + t * (ep[k] - rp[k]) for k in range(3))

    # A second ridge point one metre along the run (clamped to stay inside
    # the bay, and flipped back to an "increasing run" sense if it had to
    # step backward) gives the RIDGE axis without assuming which of world
    # X/Y it runs along -- `_ridge_info` already made that call per style.
    step = 1.0 if run + 1.0 <= info["run_hi"] else -1.0
    rp2 = info["ridge_point"](run + step)
    du = tuple(rp2[k] - rp[k] for k in range(3))
    dlen = math.sqrt(sum(c * c for c in du)) or 1.0
    u_axis = tuple((c / dlen) * step for c in du)

    dv = tuple(ep[k] - rp[k] for k in range(3))
    vlen = math.sqrt(sum(c * c for c in dv)) or 1.0
    v_axis = tuple(c / vlen for c in dv)

    n_axis = (u_axis[1] * v_axis[2] - u_axis[2] * v_axis[1],
             u_axis[2] * v_axis[0] - u_axis[0] * v_axis[2],
             u_axis[0] * v_axis[1] - u_axis[1] * v_axis[0])
    nlen = math.sqrt(sum(c * c for c in n_axis)) or 1.0
    n_axis = tuple(c / nlen for c in n_axis)
    return dict(center=center, u_axis=u_axis, v_axis=v_axis, n_axis=n_axis)


def _ragged_patch_points(placement, radius_u, radius_v, half_t, n_verts,
                         jag_frac, rng):
    """Points for one irregular, jagged-edged patch: an N-gon in the
    (`u_axis`, `v_axis`) plane through `placement["center"]`, each vertex at
    an evenly-spaced angle with its own radius jittered by up to `jag_frac`
    of the nominal (`radius_u`, `radius_v`) ellipse -- a natural torn-edge
    outline, not a circle or a clean rectangle -- extruded `+-half_t` along
    `n_axis` so it reads as a solid wafer of sheathing rather than a
    one-sided card.

    ANGLES ARE MONOTONIC (a fixed random start plus an evenly increasing
    step), never independently re-randomised per vertex: jittering the
    RADIUS at a monotonic angle cannot make the polygon cross itself, while
    jittering the ANGLE too could wind two neighbours backward past each
    other.

    Pure geometry (tuples of floats), no pxr -- reused by an offline test
    ("roughly the requested size, no degenerate spike") and by
    `_author_patch_mesh` (which feeds it real placement axes) for the
    actual mesh.

    Returns `(top_pts, bottom_pts)`, each a list of `n_verts` `(x, y, z)`
    tuples, wound the same way around the polygon (`top[i]`/`bottom[i]` are
    the same angular position, one edge apart) -- the side-quad
    construction in `_author_patch_mesh` depends on that correspondence.
    """
    center = placement["center"]
    u_axis, v_axis, n_axis = (placement["u_axis"], placement["v_axis"],
                              placement["n_axis"])
    start = rng.uniform(0.0, 2.0 * math.pi)
    top, bottom = [], []
    for i in range(n_verts):
        a = start + 2.0 * math.pi * i / n_verts
        ru = radius_u * (1.0 + rng.uniform(-jag_frac, jag_frac))
        rv = radius_v * (1.0 + rng.uniform(-jag_frac, jag_frac))
        du, dv = ru * math.cos(a), rv * math.sin(a)
        px = center[0] + u_axis[0] * du + v_axis[0] * dv
        py = center[1] + u_axis[1] * du + v_axis[1] * dv
        pz = center[2] + u_axis[2] * du + v_axis[2] * dv
        top.append((px + n_axis[0] * half_t, py + n_axis[1] * half_t,
                   pz + n_axis[2] * half_t))
        bottom.append((px - n_axis[0] * half_t, py - n_axis[1] * half_t,
                      pz - n_axis[2] * half_t))
    return top, bottom


def _face_normal_from_pts(pts, face_idx):
    """Geometric normal of one face from its first three point indices --
    correct for a planar N-gon and for a quad alike, and avoids relying on
    any winding CONVENTION (the caller's cap/side faces come from different
    constructions)."""
    p0, p1, p2 = pts[face_idx[0]], pts[face_idx[1]], pts[face_idx[2]]
    ax = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    bx = (p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2])
    n = (ax[1] * bx[2] - ax[2] * bx[1], ax[2] * bx[0] - ax[0] * bx[2],
        ax[0] * bx[1] - ax[1] * bx[0])
    length = math.sqrt(sum(c * c for c in n)) or 1.0
    return (n[0] / length, n[1] / length, n[2] / length)


def _author_patch_mesh(stage, path, infos, rng, mat):
    """One merged mesh of ragged sheathing patches + ridge shreds under
    `path`, from whichever of `infos` (a list of `_ridge_info` dicts, one
    per dropped bay with usable pitch geometry) can host one -- see the
    module's "RAGGED RAFTERS" section for the counts and sizes.
    `PATCH_COUNT_RANGE` field patches (roundish, scattered down the slope)
    plus `RIDGE_SHRED_COUNT_RANGE` shreds (long and narrow, anchored near
    the ridge) -- both draw their bay from `infos` with `rng.choice`, so a
    house with several dropped bays spreads its debris across them rather
    than piling everything on one.

    Returns `(path_or_None, stats)`; `stats` has `patches`/`ridge_shreds` --
    the counts ACTUALLY placed, which can be fewer than requested if a
    draw's chosen bay had no eave to anchor one on
    (`_patch_placement_on_bay` returning `None`).
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    if not infos:
        return None, dict(patches=0, ridge_shreds=0)

    prisms = []   # [(top_pts, bottom_pts), ...]

    def _one(size_range, aspect_range, verts_range, t_range):
        info = rng.choice(infos)
        placement = _patch_placement_on_bay(info, rng, t_range=t_range)
        if placement is None:
            return False
        major = 0.5 * rng.uniform(*size_range)
        minor = major * rng.uniform(*aspect_range)
        n_verts = rng.randint(*verts_range)
        top, bottom = _ragged_patch_points(
            placement, major, minor, 0.5 * PATCH_THICKNESS_M, n_verts,
            PATCH_JAG_FRAC, rng)
        prisms.append((top, bottom))
        return True

    n_patch = 0
    for _ in range(rng.randint(*PATCH_COUNT_RANGE)):
        if _one(PATCH_SIZE_RANGE_M, PATCH_ASPECT_RANGE, PATCH_VERTS_RANGE,
               (0.15, 0.85)):
            n_patch += 1
    n_shred = 0
    for _ in range(rng.randint(*RIDGE_SHRED_COUNT_RANGE)):
        if _one(RIDGE_SHRED_SIZE_RANGE_M, RIDGE_SHRED_ASPECT_RANGE,
               RIDGE_SHRED_VERTS_RANGE, (0.0, 0.12)):
            n_shred += 1

    stats = dict(patches=n_patch, ridge_shreds=n_shred)
    if not prisms:
        return None, stats

    pts, counts, idx, nrm = [], [], [], []
    for top, bottom in prisms:
        n = len(top)
        base = len(pts)
        pts.extend(top)
        pts.extend(bottom)
        faces = [[base + i for i in range(n)],                    # top cap
                [base + n + i for i in reversed(range(n))]]        # bottom cap
        for i in range(n):
            j = (i + 1) % n
            faces.append([base + i, base + j, base + n + j, base + n + i])
        for f in faces:
            fn = _face_normal_from_pts(pts, f)
            counts.append(len(f))
            idx.extend(f)
            nrm.extend([fn] * len(f))

    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(Vt.Vec3fArray(
        [Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in pts]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    m.CreateNormalsAttr(Vt.Vec3fArray(
        [Gf.Vec3f(float(c[0]), float(c[1]), float(c[2])) for c in nrm]))
    m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    m.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), min(zs)),
                       Gf.Vec3f(max(xs), max(ys), max(zs))])
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    return path, stats


def author_rafters(stage, house_prim, dropped_bays, root_path=None,
                   tile_m=0.6, tint=(0.55, 0.46, 0.34), roughness=0.82,
                   rng=None, patch_tint=(0.66, 0.60, 0.50),
                   patch_roughness=0.90):
    """One merged rafter-lattice mesh under `house_prim` for every prim in
    `dropped_bays` (roof bays already `SetActive(False)`'d by `strip_roof`)
    that has real pitch geometry (`_ridge_info`), PLUS a second merged mesh
    of ragged sheathing patches and ridge shreds -- see the "RAGGED RAFTERS"
    section above for why a perfectly regular cage is wrong (the
    coordinator's own review: every rafter present, no sheathing remnants,
    reads as under-construction rather than storm-stripped).

    ONE MESH FOR THE WHOLE HOUSE, per the deliverable: every bay's rafters
    and ridge board share the SAME `planks` "class" string ("rafter"), and
    `planks.build` groups by `(class, skin)`, so this is one `UsdGeom.Mesh`
    regardless of how many bays contributed to it. The patches/shreds are a
    SEPARATE mesh (own material, own irregular-polygon geometry
    `planks._box`'s rectangular solids cannot produce) but still one prim
    for the whole house.

    `rng`, when given, both raggedizes the rafter cage (threaded into
    `_raggedize_rafters` per bay) and drives the patch/shred placement --
    pass the SAME per-house `rng` every other draw the caller already uses
    (`strip_roof`, `blow_out_windows`, the pose functions), so a house's
    whole damage pattern is reproducible from one seed. `rng=None` falls
    back to a fixed `random.Random(0)` rather than the `random` module's
    global state, so a caller that forgets to pass one still gets A
    deterministic result (if the SAME one on every call) instead of a
    silently non-reproducible one.

    Bound to a `planks.wood_material` with `reflection_roughness_texture_
    influence` and `metallic_texture_influence` forced to 0.0 -- see
    `harden_wood_material`'s docstring for why: the SAME symptom (a bound,
    texture-resolved OmniPBR floor rendering as a saturated fallback colour)
    was found on this house's own subfloor material and is not fully
    root-caused, so a NEW material this file authors takes the defensive
    fix rather than risk repeating it silently.

    Returns `(mesh_path_or_None, stats)`. `mesh_path_or_None` is the RAFTER
    mesh's path (or `None` if no bay had usable geometry) -- the same
    truthiness `hurricane_house_pose_bake.build_one` already checks it for.
    `stats` has `rafters_kept`/`rafters_removed`/`rafters_snapped` (summed
    over every dropped bay) and `patches`/`ridge_shreds` (the counts
    `_author_patch_mesh` actually placed).
    """
    from . import planks

    if rng is None:
        rng = random.Random(0)

    root = root_path or (house_prim.GetPath().pathString
                         if hasattr(house_prim, "GetPath") else str(house_prim))

    infos = []
    specs = []
    kept = removed = snapped = 0
    for bay in dropped_bays:
        info = _ridge_info(bay)
        if info is None:
            continue
        infos.append(info)
        common, ridge_spec = _rafter_and_ridge_specs(info)
        ragged, st = _raggedize_rafters(common, rng)
        kept += st["kept"]
        removed += st["removed"]
        snapped += st["snapped"]
        specs.extend(ragged)
        if ridge_spec is not None:
            specs.append(ridge_spec)

    mesh_path = None
    if specs:
        mat = harden_wood_material(planks.wood_material(
            stage, root + "/Looks/rafter_wood", tile_m=tile_m, tint=tint,
            roughness=roughness))
        made = planks.build(stage, root + "/rafters", specs,
                            {"rafter": mat}, ssf=1.0, verbose=False)
        mesh_path = made[0] if made else None

    patch_stats = dict(patches=0, ridge_shreds=0)
    if infos:
        patch_mat = harden_wood_material(planks.wood_material(
            stage, root + "/Looks/sheathing_patch", tile_m=0.8,
            tint=patch_tint, roughness=patch_roughness))
        _patch_path, patch_stats = _author_patch_mesh(
            stage, root + "/sheathing_patches", infos, rng, patch_mat)

    stats = dict(rafters_kept=kept, rafters_removed=removed,
                rafters_snapped=snapped)
    stats.update(patch_stats)
    return mesh_path, stats


def harden_wood_material(mat):
    """Force `reflection_roughness_texture_influence` and `metallic_texture_
    influence` to 0.0 on a `planks.wood_material`'s shader.

    WHY. `enable_ORM_texture=True` (which `wood_material` always sets) means
    OmniPBR reads roughness and metallic from the ORM map's G/B channels
    UNLESS influence is explicitly zeroed -- the same mechanism the build-
    hurricane-scenes skill's bug catalogue already flags for a DIFFERENT
    material ("the wet pass changes nothing... roughness then comes from the
    map, not the constant... [REPORTED, NOT YET CONFIRMED]"). This house's
    own baked `subfloor` material was found bound, its texture correctly
    resolved (a fresh bare-pxr audit: 0 of 683 floor meshes unbound across
    all 72 archetypes, every `diffuse_texture`/`normalmap_texture`/
    `ORM_texture` path exists on disk on both the OSMO pod and the
    bind-mounted local container) -- yet the rendered plate still shows a
    saturated flat colour on those same floors. The most likely explanation
    that survives that audit is a texture-driven PBR channel doing something
    the authored constants did not intend (measured: `Ash_Planks_ORM.png`'s
    own metallic channel is 0 everywhere, ruling out metallic specifically,
    but NOT a stale Kit layer cache -- see this module's own bake launcher
    notes) rather than a missing binding, which is exactly the class of bug
    this hardens against for every NEW material this file authors. Reaching
    past `wood_material` to set an input on its own shader is the same
    workaround `washaway.build_rafts` already uses for `diffuse_tint`
    (`planks.wood_material`'s own comment records why: the function has no
    parameter for either).
    """
    from pxr import Sdf, Usd, UsdShade

    for prim in Usd.PrimRange(mat.GetPrim()):
        if prim.GetTypeName() == "Shader":
            sh = UsdShade.Shader(prim)
            sh.CreateInput("reflection_roughness_texture_influence",
                          Sdf.ValueTypeNames.Float).Set(0.0)
            sh.CreateInput("metallic_texture_influence",
                          Sdf.ValueTypeNames.Float).Set(0.0)
    return mat


# ---------------------------------------------------------------------------
# Which wall faces the wind
# ---------------------------------------------------------------------------
def _bearing_of(yaw_deg):
    """Compass bearing (0=N, 90=E, clockwise) a wall's OUTWARD face points,
    given the wall's own placement `yaw_deg`.

    `modular_house.build_building` faces a wall -Y ("south") at yaw 0 --
    its own docstring: "The facade faces -Y before rotation, so yaw=0 puts
    the entrance south" -- and reaches every other side through the same
    `_rot` turn baked into `yaw_deg` by `add()` (`modular_house.py:627`), so
    `yaw_deg` on a placement is already the wall's ABSOLUTE world yaw, not a
    local one. Solving `_rot(0, -1, yaw)` against the compass anchors
    (`_YAW = {"S": 0, "E": 90, "N": 180, "W": 270}`) gives
    `bearing = (180 - yaw) mod 360` for any yaw, not just those four:
    yaw=0 -> south (180), yaw=90 -> east (90), yaw=180 -> north (0),
    yaw=270 -> west (270).
    """
    return (180.0 - float(yaw_deg)) % 360.0


def _ang_diff(a, b):
    """Smallest angle between two bearings, in [0, 180]."""
    return abs((a - b + 180.0) % 360.0 - 180.0)


def windward_walls(walls, wind_bearing_deg, house_yaw_deg=0.0):
    """*walls*, sorted MOST-windward-first.

    Completes what `wind_flow.py:216-224` explicitly declines to do: with no
    heading, it falls back to "a random draw over the whole set... which at
    least does not encode the WRONG mechanism." A hurricane scene HAS the
    heading (`wind_bearing_deg` — the compass bearing the wind blows TOWARD,
    `hurricane_wind_field.md` section 2.3), so the wall the storm actually
    hits first can be picked instead of guessed: the one whose outward face
    is most OPPOSED to that bearing, i.e. closest to `wind_bearing_deg + 180`.

    *walls* is the placement-dict list `wind_flow.wreck_building` already
    filters `items` down to (`damage._sub_of(category) == "wall"`) — there is
    no separate house record carrying wall normals; each wall's own
    `yaw_deg` already IS its facing (see `_bearing_of`). `house_yaw_deg`
    corrects for a caller whose walls carry a LOCAL, pre-house-rotation yaw
    instead of the world yaw `modular_house.build_building` normally bakes
    in (`add()`: `lyaw + yaw`); it defaults to 0 and is a no-op for every
    `items` list this repo's own generator produces.

    Returns `list(walls)`, unsorted, when `wind_bearing_deg` is None, so a
    caller can use this unconditionally and land on `wind_flow`'s own
    documented fallback by simply not supplying a bearing.
    """
    if wind_bearing_deg is None:
        return list(walls)
    target = (float(wind_bearing_deg) + 180.0) % 360.0

    def _score(w):
        yaw = float(w.get("yaw_deg", 0.0)) + float(house_yaw_deg)
        return _ang_diff(_bearing_of(yaw), target)

    return sorted(walls, key=_score)


def _bias_wall_pool(items, wind_bearing_deg, n_walls, pad=2.5):
    """Narrow `items`'s WALL entries to a windward-biased candidate pool.

    `wind_flow.wreck_building` is untouched by this module (see the module
    docstring) and still hard-picks its broken walls with
    `rng.sample(walls, n_walls)` — uniform over whatever `walls` it is
    handed. Handing it every wall in the house is `wind_flow`'s own
    documented fallback for having no heading; handing it only the walls
    THIS module already knows are windward turns that fallback into the real
    mechanism without changing a line of `wind_flow.py`.

    Pool size is `pad * n_walls`, not exactly `n_walls`: a pool sized to the
    hard-pick count would make the choice fully deterministic run to run,
    and a street where every house of one style loses the identical wall
    reads as a template, not a storm. `pad` is tuned, not sourced.
    """
    from . import damage

    walls = [q for q in items
             if damage._sub_of(q.get("category")) == "wall"]
    if not walls or not n_walls:
        return items
    ranked = windward_walls(walls, wind_bearing_deg)
    pool_n = min(len(ranked),
                max(int(n_walls), int(math.ceil(n_walls * pad))))
    keep = {id(w) for w in ranked[:pool_n]}
    return [q for q in items
            if damage._sub_of(q.get("category")) != "wall" or id(q) in keep]


# ---------------------------------------------------------------------------
# JOB B (2026-08-31) -- cardinal bay-drop variants for `cover_lost` /
# `deck_panels_lost`
# ---------------------------------------------------------------------------
# THE PROBLEM. `cover_lost`/`deck_panels_lost` are NOT in `suburb_hurricane_
# launch_script._WIND_YAWED`: the assembly places these houses at their
# ORIGINAL city-layout (street) yaw, never rotated toward the wind (unlike
# `partial_collapse`/`leveled`, which the launcher does re-aim). Their
# archetype's dropped bays, though, are baked ONCE at `_BAKE_BEARING = 0.0`
# via `strip_roof(seed_dir=0.0)` (`hurricane_house_pose_bake.build_one`) --
# a FIXED local side, always the same one, for every house of a style
# regardless of where it ends up facing. Roughly 55% of a Level-3 plate's
# houses land on one of these two rungs (`hurricane.py`'s ROUND 2 cut:
# `cover_lost` ~31% + `deck_panels_lost` ~24%), so the majority of the
# damaged population has its dropped bays on a side that has nothing to do
# with which way the wind was actually blowing at that lot.
#
# THE FIX. Four bake-time variants per style, one per LOCAL cardinal side
# (`house_<style>_<level>_{n,e,s,w}.usd`, built by `hurricane_house_pose_
# bake.py` with `seed_dir=_VARIANT_SEED_DIR[side]` instead of a fixed 0.0),
# plus `windward_variant` below to pick the right one at PLACEMENT time from
# a house's own (street yaw, local wind bearing) pair -- no re-aiming of the
# house itself needed, unlike `_WIND_YAWED`'s levels.
#
# THE LOCAL AXES. `modular_house.build_building`'s own convention (`_bearing_
# of`'s docstring): local -Y is south (compass 180) at yaw 0, and `strip_
# roof`'s own `seed_dir` math (`ux, uy = sin(target), cos(target)`) bakes in
# world X=East / Y=North. So at yaw 0 (bake time, local frame == world
# frame): local +Y -> north ("n"), local -Y -> south ("s"), local +X -> east
# ("e"), local -X -> west ("w"). `_SIDE_BASE_BEARING` is each side's own
# OUTWARD-facing compass bearing at yaw 0 -- the exact same "outward face
# bearing" quantity `_bearing_of` computes for one wall, just anchored per
# cardinal side rather than per wall's own yaw.
_SIDE_BASE_BEARING = {"n": 0.0, "e": 90.0, "s": 180.0, "w": 270.0}

# The `seed_dir` to hand `strip_roof` at BAKE TIME (house at yaw 0, so local
# == world) so the bay-ranking's exposed direction — `strip_roof` scores
# bays toward `(seed_dir + 180) % 360`, "where the wind is coming FROM" —
# lands on that side's own `_SIDE_BASE_BEARING`. Solving
# `(seed_dir + 180) % 360 == b0` gives `seed_dir = (b0 - 180) % 360`:
# n -> 180, e -> 270, s -> 0, w -> 90.
_VARIANT_SEED_DIR = {side: (b0 - 180.0) % 360.0
                     for side, b0 in _SIDE_BASE_BEARING.items()}

# The two rungs a variant actually applies to -- see the section docstring;
# `roof_stripped` drops EVERY bay (`_ROOF_FRAC`'s `1.00`), so it has no
# "windward side" left to place a variant on, and `roof_collapsed`/
# `partial_collapse`/`leveled` are pose-authored (isotropic, or already
# wind-yawed at placement) rather than `strip_roof`-biased at all.
LEVELS_WITH_VARIANTS = ("cover_lost", "deck_panels_lost")
VARIANTS = ("n", "e", "s", "w")


def windward_variant(yaw_deg, wind_bearing_deg):
    """Which LOCAL cardinal side (`'n'`/`'e'`/`'s'`/`'w'`) of a house placed
    at its own STREET yaw `yaw_deg` (unrotated by wind -- `cover_lost`/
    `deck_panels_lost` are not in `_WIND_YAWED`) actually faces the wind
    blowing toward `wind_bearing_deg`, so the matching `house_<style>_
    <level>_{side}.usd` variant (its dropped bays baked toward that LOCAL
    side, `_VARIANT_SEED_DIR`) puts them on the TRUE windward face
    regardless of which way the house happens to front the street.

    THE ALGEBRA. A local side with base bearing `b0` (its own outward
    compass facing at yaw 0, `_SIDE_BASE_BEARING`) ends up facing world
    bearing `(b0 - yaw_deg) % 360` once the whole house is placed at
    `yaw_deg` -- the identical `(180 - yaw) % 360` transform `_bearing_of`
    applies to one wall's own yaw, generalised to any base bearing since
    it is the SAME rigid-body rotation of the whole house. The side that
    should carry the damage is the one whose outward face ends up pointed
    straight at where the wind is COMING FROM -- `(wind_bearing_deg + 180)
    % 360`, matching `windward_walls`'s own target -- so this inverts that
    transform for `b0` (`needed_b0 = (target + yaw_deg) % 360`) and returns
    whichever of the four `_SIDE_BASE_BEARING` entries sits closest to it.

    Exact ties (a house yawed exactly 45/135/225/315 degrees against a wind
    bearing that puts the target exactly between two sides) resolve to
    whichever candidate is nearer the front of `_SIDE_BASE_BEARING`'s own
    iteration order (`min`'s stable tie-break) -- an arbitrary but
    deterministic choice for a genuinely ambiguous case, not a meaningful
    preference for one side over the other.
    """
    target = (float(wind_bearing_deg) + 180.0) % 360.0
    needed_b0 = (target + float(yaw_deg)) % 360.0
    return min(_SIDE_BASE_BEARING,
              key=lambda side: _ang_diff(_SIDE_BASE_BEARING[side], needed_b0))


# ---------------------------------------------------------------------------
# The dispatcher
# ---------------------------------------------------------------------------
def wreck_building(stage, house_prim, level, rng, *, wind_bearing_deg=None,
                   **kw):
    """Wreck one house to *level*, hurricane rules.

    Returns fragment paths — empty below `roof_stripped`, where nothing is
    fractured, matching `strip_roof`/`blow_out_windows`/`_blow_doors`'s own
    "no fracture" contract.

    *house_prim* may be a `Usd.Prim` or a path string; both are accepted
    because this module has two expected callers — one holding a live
    per-house scope during assembly, one recovering a path from a record
    written earlier in the pipeline.

    `shingles_lost` / `cover_lost` / `deck_panels_lost` / `roof_stripped`
    (2026-08-31: joined this group, see `_ROOF_FRAC`) dispatch straight to
    `strip_roof` / `blow_out_windows` / `_blow_doors` on *house_prim*, then
    `author_rafters` for any level in `RAFTER_LEVELS` — no `items`, `tag`,
    `nrng` or `planks_mats` needed for any of these four.

    `roof_collapsed` / `partial_collapse` / `leveled` (`_POSE_LEVELS`)
    dispatch to `pose_roof_collapsed` / `pose_partial_collapse` /
    `pose_leveled` — a rigid POSE edit to the existing walls/roof, no
    fracture, no settle. See the "TOP RUNGS, NO PHYSICS" section above for
    why: the tornado fracture path these three used to share with `wind_
    flow` is a debris-fan signature, not a hurricane one, and it cost a
    25-minute settle per house on a pod whose PhysX GPU never actually
    engaged. Like the four above, none of `items`/`tag`/`nrng`/`planks_mats`
    are needed.

    The legacy `wind_flow.wreck_building` fracture path below is UNREACHABLE
    for every name in `HOUSE_LEVELS` as of the 2026-08-31 pose-authoring
    pass (the two dispatch blocks above now cover all eight) and is kept
    only so a caller passing an out-of-ladder level string does not lose the
    fracture machinery outright — `wind_bearing_deg`'s wall-pool bias
    (`_bias_wall_pool`) still applies there if that path is ever reached.

    `level="swept"` raises: see the module docstring's citations.
    `level="pristine"` is a no-op returning `[]`.
    """
    if level == "swept":
        raise ValueError(
            "hurricane_flow does not reach 'swept': every hurricane home "
            "surveyed with complete destruction was tied to storm surge, "
            "never wind alone (Roueche et al.), and wind-only slab-sweeping "
            "(EF FR12 DOD10, ~200 mph) is above Cat 5 and unreachable at a "
            "suburban site. A scattered swept lot here would read as a "
            "tornado. disaster.surge owns any bare-slab state for this "
            "disaster -- call that instead.")
    if level not in BREAK_PLAN:
        raise ValueError("unknown level {0!r}; expected one of {1}".format(
            level, ", ".join(sorted(BREAK_PLAN))))
    if level == "pristine":
        return []

    prim = (stage.GetPrimAtPath(house_prim)
            if isinstance(house_prim, str) else house_prim)
    if prim is None or not prim.IsValid():
        raise ValueError("no valid house prim at {0!r}".format(house_prim))

    if level in _ROOF_FRAC:
        # CAPTURE THE BAYS BEFORE STRIPPING. `strip_roof` returns only a
        # count; the rafter lattice needs the actual prims that just went
        # inactive, and comparing active-state before/after is the only way
        # to know which ones they were (there is no return-the-prims
        # contract to add to `strip_roof` without breaking its existing
        # "returns the number deactivated" callers).
        before = {b.GetPath(): b for b in roof_bay_prims(prim) if b.IsActive()}
        roof_frac, win_frac = _ROOF_FRAC[level]
        strip_roof(prim, roof_frac, rng, seed_dir=wind_bearing_deg)
        if win_frac > 0.0:
            blow_out_windows(prim, win_frac, rng)
        if level != "shingles_lost":
            # Openings fail at ~96-97 mph, essentially the same gust as
            # `cover_lost`'s ~97 mph threshold -- from here up every door
            # goes, same as `wind_flow.py:306-320` already does
            # unconditionally once IT gets past its own floor.
            _blow_doors(prim)
        if level in RAFTER_LEVELS:
            dropped = [b for path, b in before.items() if not b.IsActive()]
            author_rafters(stage, prim, dropped, rng=rng)
        return []

    if level in _POSE_LEVELS:
        if level == "roof_collapsed":
            pose_roof_collapsed(stage, prim, rng)
        elif level == "partial_collapse":
            pose_partial_collapse(stage, prim, rng)
        else:
            pose_leveled(stage, prim, rng)
        return []

    try:
        items = list(kw["items"])
        tag = kw["tag"]
        nrng = kw["nrng"]
        planks_mats = kw["planks_mats"]
    except KeyError as exc:
        raise ValueError(
            "level {0!r} needs wind_flow's fracture path, which needs {1} "
            "-- pass it as a keyword, the same as wind_flow.wreck_building "
            "itself requires".format(level, exc)) from exc
    mat_cache = kw.get("mat_cache")

    # Windward bias without editing wind_flow.py -- see `_bias_wall_pool`.
    # Not applied at `leveled`: past `partial_collapse` every wall breaks
    # regardless of facing (`wind_flow.py:227-233`), so narrowing the pool
    # there would dress an isotropic collapse up as directional for nothing.
    if wind_bearing_deg is not None and level != "leveled":
        n_walls = wind_flow.BREAK_PLAN[level][0]
        items = _bias_wall_pool(items, wind_bearing_deg, n_walls)

    parent = prim.GetPath().pathString
    return wind_flow.wreck_building(stage, parent, items, tag, level, rng,
                                    nrng, planks_mats, mat_cache=mat_cache)
