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
    # Cat 2-3+. `wind_flow`'s own floor: the WHOLE roof fractures into
    # `roof_seeds` pieces per module; walls and floors still untouched.
    "roof_stripped":    wind_flow.BREAK_PLAN["roof_stripped"],
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
_ROOF_FRAC = {
    "shingles_lost":    (0.00, 0.00),
    "cover_lost":       (0.34, 0.08),   # ~1 bay of 3
    "deck_panels_lost": (0.55, 0.30),   # ~2 bays of 3 — its "1-3 panels"
}

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

    `shingles_lost` / `cover_lost` / `deck_panels_lost` dispatch straight to
    `strip_roof` / `blow_out_windows` / `_blow_doors` on *house_prim* — no
    `items`, `tag`, `nrng` or `planks_mats` needed for those three.

    `roof_stripped` and above dispatch to `wind_flow.wreck_building`
    UNMODIFIED, which does need `items` / `tag` / `nrng` / `planks_mats` (and
    accepts `mat_cache`) — pass them as keywords, the same names
    `wind_flow.wreck_building` itself takes. When `wind_bearing_deg` is also
    given, the wall pool `wind_flow` samples from is pre-biased toward the
    windward face first (`_bias_wall_pool`) — see the module docstring for
    why this is done here rather than by editing `wind_flow.py`.

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
