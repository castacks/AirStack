"""tornado_kit — the ROUND 2 R3 adapter: let the EXISTING sliced-building
tornado ladder (`disaster/tornado_urban.py` + `disaster/tornado_urban_usd.py`)
damage a KIT-STYLE building (`detail/urban_building.py`'s `bld_<style>_DG0`
archetypes — brownstone, brownstone_row, dw_terrace, walkup, office,
commercial_mid, ...) instead of a sliced GAC/downtowncity building.

`_plans/urban_tornado_plan.md` §7's rule R3, verbatim: "an adapter that lets
the EXISTING planner run on a kit-style building and the existing apply half
wreck it, so brownstone rows and walkups in the CORE show real partial
collapses — the damage the user is asking to SEE." The user's own complaint
(round 2) was that 13 of 20 corridor records had no damage path at all
because they were kit styles or AEC brownstones the round-1 ladder could not
touch.

WHY AN ADAPTER, NOT A PLANNER FORK
------------------------------------
`tornado_urban.plan_damage` is written against ONE grid abstraction,
`quake_sliced._Grid`, and `_Grid` reads exactly five fields off each
placement dict's own `p["_side"]` / `p["_role"]` / `p["_storey"]` /
`p["_bay"]` / `p["_size"]` (plus `p["prim_path"]`, read by `quake_sliced.
_path`). A REAL GAC/downtowncity slice already carries all five —
`gac_storey_slice.as_placements` stamps them at cut time (see that
function's own docstring: "THIS IS THE BRIDGE between the slicer and the
damage ladder"). A kit building built by `urban_building.build_building`
carries NONE of them — `_place()` only ever writes `usd`, `x_m`/`y_m`/`z_m`,
`yaw_deg`, `roll_deg`, `pitch_deg`, `scale`, `category`, `axis_up`,
`raw_pivot`.

Two ways to close that gap: (a) fork `tornado_urban.py` into a second
planner that reads `urban_building`'s OWN grammar directly (band/sub/side
computed ad hoc), or (b) translate `urban_building`'s grammar into the
five fields `_Grid` already understands and change nothing about the
planner. (b) is what this module does, for the same reason
`quake_flow.describe` is ONE function both a kit style AND a sliced style
already share (`_plans/urban_tornado_research.md`/the skill's own
observation: "the planner runs on `quake_flow.describe`'s table so it
should run on [kit styles] unchanged"): one damage ladder, two source
geometries, ZERO duplicated recipe logic. A fork would mean every future
change to the T0-T4 ladder (a new cut table, a new recipe, a height-class
retune) has to land in two files and stay byte-for-byte in step by hand.
The adapter costs one extra pass over the element table per building and
buys a single ladder forever.

THE ADAPTER'S JOB, PRECISELY
-------------------------------
`quake_flow.describe(style, placements, 0, 0, 0)` already classifies every
kit placement into an element record (`role`, `side`, `storey`, `lx`/`ly`,
`mass`, `name` — the piece's own basename) — `classify()`'s own KIT
CATEGORY grammar (`bld_<style>_<sub>`, optionally `_wingN_<sub>`), same
function `kit_substitute.build_kit` already relies on. `adapt()` walks
`describe()`'s own `elements` list (each element keeps its placement as
`e["p"]`, describe/classify's own convention) and stamps the five `_Grid`
fields onto `e["p"]` IN PLACE, deriving every one of them from that
classification rather than re-deriving it from scratch:

    ADAPTER FIELD  DERIVED FROM
    -------------  ------------------------------------------------------
    `_side`        `e["side"]` (already S/E/N/W, `quake_flow._side_of`)
                   for every piece on a wall/parapet run. CORNER pieces
                   (classify role "corner"/"parapet_corner") get a
                   position-derived corner NAME instead — `e["side"]` is
                   a single nearest-wall-line pick and is meaningless at
                   a true corner, so this module derives SW/SE/NW/NE from
                   the SIGN of the piece's own local (lx, ly), NUDGED by
                   its own measured bbox centre rotated by its yaw
                   (`_corner_of` — the raw pivot alone ties EXACTLY on one
                   real style, `office`'s curved corner bay, whose short
                   side depth equals two corner legs; see that function's
                   own docstring for the measured fix), the same partition
                   `gac_storey_slice.ring` draws by which LEG of the
                   footprint corner a piece's position falls in.
                   ROOF/portico/pediment/ornament/balcony pieces get the
                   sentinel `"-"` (never a grid side) — see `_role` below.
    `_role`        the kit CATEGORY, via `quake_flow.classify`'s own
                   `role` plus one extra split this module adds:
                     window/door modules      -> `"wall"`  (the OPENING
                                                  sub-panel role in
                                                  `quake_sliced`'s own
                                                  vocabulary — `is_opening`
                                                  keys off `_bay`'s SUB-
                                                  INDEX, not off `_role`;
                                                  "wall" is simply the
                                                  band role every GAC
                                                  opening piece ALSO
                                                  carries)
                     plain wall/pier modules  -> `"pier"`
                     corner modules           -> `"corner"`
                     parapet-corner modules   -> `"parapet_corner"`
                     parapet/cornice modules  -> `"parapet"`
                     roof modules             -> `"roof"`
                     portico/pediment/
                       ornament/balcony       -> `"core"` (no literal
                                                  "interior" module exists
                                                  in this kit — see NOT
                                                  SUPPORTED below; this is
                                                  the catch-all for
                                                  decorative attachments
                                                  that must never become
                                                  an addressable side/bay
                                                  cell)
                   Which of "wall"/"pier" a piece gets is decided by
                   `_is_glazed_name` (below) — a piece NAME test, exactly
                   the same kind of evidence `quake_flow._G2_WIN_FACES`
                   already carries for the fire/quake ladders' own punched-
                   window recipes, not a guess.
    `_storey`      `e["storey"]` verbatim (`quake_flow._storey_of`,
                   already correct for a kit mass — it is the SAME
                   function a real sliced building's `describe()` call
                   uses).
    `_bay`         `k * 3 + j` — `quake_sliced.bay_no`/`is_opening`'s own
                   n_sub=3 grammar (`BAY_SPLITS`), picked over the n_sub=1
                   alternative (see "WHY n_sub=3" below). `j` (the sub-
                   index) is 1 ("the wide middle sub-panel — the one with
                   the window in it", `quake_sliced.is_opening`'s own
                   docstring) for a glazed piece, 0 for a plain one. `k`
                   (the bay index) bins the piece's own along-side
                   coordinate (`lx` for S/N, `ly` for E/W, recentred onto
                   the band's own [0, run-length] frame the way
                   `urban_building._side_slot`/`_fill` lay pieces out) by
                   the OWNING BAND's own `module` (bay pitch) and `corner`
                   leg — read straight off `urban_building.STYLES[style]`,
                   never a style-wide constant (a style's ground band and
                   storey band can use different modules, e.g. `office`'s
                   8 m ground bays over 4 m storey bays).
    `_size`        the piece's own MEASURED extent, `urban_building.
                   PIECES[name]` (sx, sy, sz) rotated into the BUILDING's
                   own local axes via `urban_building._width`/`_kit`'s own
                   "along"/"thickness" split for whichever side the piece
                   sits on (S/N: along=X, thickness=Y; E/W: along=Y,
                   thickness=X) — this is what `t_hanging_panels`'/
                   `t_out_of_plane_top`'s own `depth = sy if side in
                   (S, N) else sx` formula assumes, and NEVER a guessed
                   flat constant.
    `prim_path`    left untouched when already present (the real authored
                   path `scene_generator.apply_placements` stamps back
                   onto a placement dict at reference time — `wreck_kit`'s
                   own second pass, AFTER `kit_substitute.build_kit` has
                   put pieces on the stage); synthesised deterministically
                   otherwise, so the PURE planner path (`plan_for_kit`,
                   no stage at all) still has a stable, unique, JSON-safe
                   key for every piece — `quake_sliced._remove`/`_path`
                   silently drop a piece with no path, so a None path here
                   would make removal a no-op across the whole plan.

WHY n_sub=3, NOT n_sub=1
--------------------------
`quake_sliced.is_opening(p, n_sub)` is `n_sub <= 1 or sub_ix(p, n_sub) ==
n_sub // 2` — with `n_sub=1` this is unconditionally TRUE for every piece
regardless of role, which fails the one test this adapter must pass
(`test_is_opening_true_exactly_on_glazed_pieces`): a plain, unglazed wall
panel (most of `SM_MBuilding0*_Facade_B`-shaped pool entries) would count as
an "opening" too. `n_sub=3` — the SAME grammar a real GAC/downtowncity slice
uses (`gac_storey_slice.BAY_SPLITS`, `role = "wall" if j == len(fr)//2 else
"pier"`) — lets a piece's own SUB-INDEX carry exactly the window/door-or-not
distinction this module already computed by name, with `is_opening` matching
it precisely and `quake_sliced.n_sub_of` (which returns 3 the moment ANY
element in the grid carries `_role == "pier"`) picking it up automatically
with no extra flag anywhere. Verified per style, not assumed: every tested
style has at least one plain (unglazed) wall-band piece somewhere on its
elevation (`test_n_sub_is_three_on_every_tested_style`).

WHAT IS NOT SUPPORTED — REFUSED, NEVER MIS-DAMAGED
------------------------------------------------------
* **Multi-mass styles** (`urban_building.STYLES[style]` carries `"wings"`
  or `"tower"` — `tower`, `skyscraper_b`, `highrise_step`,
  `block_residential`, `block_office`, `block_stone`). `_bay`'s binning
  reads ONE per-style band-pitch table (`urban_building.STYLES[style][
  "bands"]`), and a wing/tower is placed from a DIFFERENT spec dict
  (`urban_building._mass_specs`'s own recursion) with its own bands/
  modules/legs this adapter never looks up — binning a wing's bays against
  the main mass's pitch table would silently misplace them onto the wrong
  bay index rather than fail loudly. `kit_placements`/`adapt`/
  `plan_for_kit`/`wreck_kit` all raise `ValueError` up front for one of
  these rather than guess.
* **No true "interior" kit module exists** (`urban_building.build_building`
  authors a shell + roof, never floor slabs/columns the way `quake_flow.
  fit_interior` does for the earthquake ladder's own kit-style path). The
  `"core"` `_role` above is therefore never assigned to a real structural
  piece here — only to portico/pediment/ornament/balcony decoration, which
  is intentionally EXCLUDED from every side/bay grid cell (`_side = "-"`)
  so it is never damaged by this ladder at all. Documented, not a bug: a
  balcony rail or a portico column simply stays untouched this round.
* **The curtain-wall tower family (family "05")** has no per-piece window-
  rectangle table (`quake_flow._G2_WIN_FACES` only covers families 01-04
  and the Downtown_West kit) — `_is_glazed_name` would default every
  `SkyscraperFacade`/`SkyscraperCorner` piece to "pier" (unglazed), which
  is wrong for a wall that is ALL glass. Not independently gated here
  because every family-05 style already carries a `tower`/`wings` entry
  and is refused by the multi-mass rule above; noted so a future single-
  mass family-05 addition does not silently mis-glaze.
* **`civic_hall`'s portico band** (`sub == "trim"`, not one of `quake_flow.
  classify`'s `_PARAPET_SUBS` aliases) falls through `classify` to a plain
  `"wall"` role rather than `"parapet"` — a pre-existing property of
  `quake_flow.classify` this module does not alter (out of file
  ownership). Not independently gated: it degrades to an ordinary
  removable wall/pier band, never a crash and never a `_side`/`_bay`
  collision, just a less physically apt label. Not part of this round's
  tested style set.

THE PIPELINE (mirrors `tornado_urban_usd.wreck_urban`'s own shape)
-----------------------------------------------------------------------
    kit_placements(style, seed)          urban_building.build_building,
                                          PURE, no stage, no pxr — confirmed
                                          by reading it: it returns a plain
                                          placement list and never imports
                                          `pxr`.
    info = quake_flow.describe(...)      the classification pass, unchanged
    elements = adapt(placements, info)   stamp the five `_Grid` fields
    plan_for_kit(...)                    kit_placements -> describe -> adapt
                                          -> tornado_urban.plan_damage
    wreck_kit(stage, cell, ...)          kit_substitute.build_kit (into
                                          `<cell>/parts`, NEVER `cell`
                                          itself — the Scope.Define trap
                                          the fire/quake sessions already
                                          hit, `.agents/skills/build-
                                          urban-fire-scenes/SKILL.md`'s bug
                                          catalogue) -> annotate_glazing
                                          (real geometry, only possible now
                                          that pieces are on the stage) ->
                                          re-describe + adapt against the
                                          AUTHORED prim paths ->
                                          plan_damage -> apply_plan
"""

import contextlib
import math
import os
import random

from . import kit_substitute as ksub
from . import quake_flow as qf
from . import quake_sliced as qs
from . import tornado_urban as tu

# A convenience intensity per level for the probe/tests, NOT part of the
# plan schema and not read by `tornado_urban.plan_damage` itself (which
# takes `intensity` as an explicit argument) — the same bucket
# `tests/test_tornado_urban.py`'s own `_plan()` helper uses, reproduced here
# so a caller that only knows "T3" has a reasonable intensity to pass.
LEVEL_INTENSITY = {"T0": 0.05, "T1": 0.20, "T2": 0.40, "T3": 0.60, "T4": 0.85}

# Piece NAME substrings that mean "this module is a window or a door" for
# kits whose art names them plainly (CivilianArea, Downtown_West, the church
# set) — case-insensitive, checked before the measured `_G2_WIN_FACES`
# table so a kit that names its own glazing does not need a table entry.
_GLAZED_NAME_TOKENS = ("window", "door")

# classify() roles that sit on a normal wall/parapet run (a real `_side` in
# S/E/N/W, a real `_bay`).
_RUN_ROLES = ("wall", "parapet")
# classify() roles that are a CORNER piece (position-derived SW/SE/NW/NE).
_CORNER_ROLES = {"corner": "corner", "parapet_corner": "parapet_corner"}
# classify() roles excluded from the side/bay grid entirely (`_side = "-"`).
_EXCLUDED_ROLES = {
    "roof": "roof",
    "portico": "core", "pediment": "core", "ornament": "core",
    "balcony": "core",
}


def _ub():
    """`detail.urban_building`, imported lazily and only after
    `kit_substitute`'s own `sys.path` fix has run — `urban_building` is a
    pure module (no `pxr` at import time, checked: it imports only `json`,
    `math`, `os` at module scope) but it lives under `scene_gen/detail/`,
    which is only guaranteed on `sys.path` once `_ensure_scene_gen_on_path`
    has run — `kit_substitute.styles`'s own documented reason for needing
    the same fix. Reusing that helper rather than re-deriving the path math
    a second time here."""
    ksub._ensure_scene_gen_on_path()
    from detail import urban_building as ub
    return ub


def _refuse_if_unsupported(style):
    """`urban_building.STYLES[style]`, or raise `ValueError` with the exact
    reason — see the module docstring's "WHAT IS NOT SUPPORTED"."""
    ub = _ub()
    spec = ub.STYLES.get(style)
    if spec is None:
        raise ValueError(
            "tornado_kit: no such style {0!r} in urban_building.STYLES "
            "(have {1})".format(style, ", ".join(sorted(ub.STYLES))))
    if spec.get("wings") or spec.get("tower"):
        raise ValueError(
            "tornado_kit: style {0!r} is multi-mass (carries {1}) -- "
            "adapt() only binds bays against the MAIN mass's own per-band "
            "pitch table (urban_building.STYLES[style]['bands']); a wing "
            "or tower is placed from a DIFFERENT spec dict with its own "
            "bands/modules this adapter never looks up, so binning its "
            "pieces against the main mass's pitch would silently misplace "
            "them onto the wrong bay index. Refused rather than "
            "mis-damaged.".format(
                style, "wings" if spec.get("wings") else "a tower"))
    return spec


def kit_placements(style, seed, z0=0.0):
    """`urban_building.build_building(style, 0, 0, 0, Random(seed),
    z0=z0)`'s own placement list — PURE, no stage. Verified by reading
    `build_building`/`_place_band`/`_roof`/`_portico`/`_place`: none of
    them import `pxr`, take a `stage` argument, or touch anything outside
    the plain-dict placement list they return; the sole state they thread
    is the `rng` this function seeds. Canonical frame (x=y=yaw=0) so a
    piece's world coordinates equal its BUILDING-LOCAL ones — `adapt`'s own
    bay-binning arithmetic relies on this."""
    _refuse_if_unsupported(style)
    ub = _ub()
    return ub.build_building(style, 0.0, 0.0, 0.0, random.Random(seed),
                             z0=float(z0))


# ---------------------------------------------------------------------------
# glazing-by-name
# ---------------------------------------------------------------------------
def _is_glazed_name(name):
    """Is piece `name` a window or a door module?

    Two kinds of evidence, in order: the kit's OWN naming (CivilianArea's
    `SM_*Window*`/`SM_Door*`, Downtown_West's `*_doublewindow`/
    `*_singlewindow`/`*_widewindow`/`*_doubleblackdoor`, the church set's
    `*_Window_*`/`*_Door_*` — a case-insensitive substring test), then
    `quake_flow._G2_WIN_FACES` membership — the MEASURED per-piece punched-
    window rectangle table every `r_window_glass`-family fire/quake recipe
    already trusts (`SM_MBuilding0{1,2,3,4}_Facade_*`/`FirstFloor_*`/
    `TopFloor_*` — measured, not named, since the ModernCityEnvironment kit
    never puts "window" in a piece's own basename). A piece in neither is
    treated as unglazed (`"pier"`) — the safe default: a plain panel that
    turns out to carry no window (the common case, e.g. every `Facade_B`-
    class piece measured with zero window relief, `urban_building.fam04`'s
    own docstring) never becomes a false glass-loss candidate.
    """
    low = str(name or "").lower()
    if any(tok in low for tok in _GLAZED_NAME_TOKENS):
        return True
    return name in qf._G2_WIN_FACES


def _corner_of(ub, name, lx, ly, yaw_deg):
    """SW/SE/NW/NE from the SIGN of a corner piece's own local (lx, ly),
    NUDGED first by its own measured bbox centre (rotated by its final
    authored yaw) rather than trusting the raw PIVOT point alone.

    THE DEGENERATE CASE THIS EXISTS TO FIX (found on `office`,
    `urban_building.fam02` with `corner_bays=True`): fam02's curved 8 m
    corner bay consumes the WHOLE short side on this style (`D == 2 *
    corner_leg`, 16 == 2*8), so `_corner_pivot`'s own arithmetic places the
    SW and NE pieces' PIVOTS exactly ON the building's own local y=0
    centreline (`ly == 0.0`, an EXACT floating-point tie, not a precision
    artefact — reproduced by hand against `_corner_pivot`'s formula). A
    bare sign test on the pivot alone puts one of that pair on the wrong
    side of the tie. `urban_building.PIECES[name]`'s own measured
    `(xmin, ymin)` gives each corner piece's LOCAL bbox centre relative to
    its own pivot — genuinely off-centre for every corner mesh checked
    (never `(0, 0)`) — and rotating that offset by the piece's own final
    yaw (`quake_flow.classify`'s `e["yaw"]`, already the fully-resolved
    authored angle) pushes the test point toward the piece's own BODY,
    which sits strictly inside the correct quadrant even when its pivot
    does not. Verified by hand for `office`'s SW/SE/NE/NW quartet: the
    nudge moves the SW piece's local y from 0.0 to -4.1 (now unambiguously
    "S") and leaves every already-unambiguous corner's classification
    unchanged (the piece-sized nudge is small next to a real corner's
    otherwise-large lx/ly).
    """
    meas = ub.PIECES.get(name)
    if meas:
        sx, sy, _sz, xmin, ymin, _zmin = meas
        ox, oy = float(xmin) + float(sx) / 2.0, float(ymin) + float(sy) / 2.0
        a = math.radians(float(yaw_deg or 0.0))
        ca, sa = math.cos(a), math.sin(a)
        lx = lx + (ox * ca - oy * sa)
        ly = ly + (ox * sa + oy * ca)
    ns = "S" if ly < 0.0 else "N"
    ew = "W" if lx < 0.0 else "E"
    return ns + ew


# ---------------------------------------------------------------------------
# _size — the piece's own measured extent, rotated into the building frame
# ---------------------------------------------------------------------------
def _along_thick(ub, name):
    """(along-wall extent, thickness) in the piece's OWN canonical
    authoring frame — `urban_building._width(name)`'s own along/thickness
    split (canonical: along = sx; Downtown_West's "dw" frame: along = sy),
    read off the kit's own measured `PIECES[name]` rather than guessed."""
    meas = ub.PIECES.get(name)
    if not meas:
        return 1.0, 0.3
    sx, sy = float(meas[0]), float(meas[1])
    if ub._kit(name)[1] == "dw":
        return sy, sx
    return sx, sy


def _extents(ub, name, side):
    """`(extent_x, extent_y, extent_z)` in the BUILDING's own local frame
    (canonical build, x=y=yaw=0, so this equals world too).

    S/N sides: the wall runs along building-X, so `extent_x = along`,
    `extent_y = thickness`. E/W sides: the wall runs along building-Y, so
    the two swap. A corner/roof/other piece (side not in S/E/N/W) gets its
    raw measured `(sx, sy, sz)` unrotated — `_facade_area_of`/debris volume
    only ever sort the two largest dims, so axis order does not matter
    there, and no recipe reads a corner/roof piece's `depth` the way
    `t_hanging_panels`/`t_out_of_plane_top` read a WALL piece's.
    """
    meas = ub.PIECES.get(name)
    sz = float(meas[2]) if meas else 3.0
    if side not in qs.SIDES:
        if not meas:
            return 1.0, 0.3, sz
        return float(meas[0]), float(meas[1]), sz
    along, thick = _along_thick(ub, name)
    if side in ("S", "N"):
        return along, thick, sz
    return thick, along, sz


# ---------------------------------------------------------------------------
# _bay — bin the along-side coordinate into the OWNING BAND's own pitch
# ---------------------------------------------------------------------------
def _band_pitch_table(spec):
    """`{band_sub: (module, corner_leg)}` for every band in `spec["bands"]`
    — read directly off `urban_building.STYLES[style]`, never a style-wide
    constant (a style's ground band and storey band can use different
    modules — `fam02`'s 8 m ground bays over 4 m storey bays, `fam03`'s 4 m
    ground vs. no separate module change but a different `corner` leg on
    some bands)."""
    out = {}
    for band in spec["bands"]:
        corner = band.get("corner")
        leg = float(corner[2]) if corner else 0.0
        out[band["sub"]] = (float(band.get("module") or 0.0), leg)
    return out


def _base_sub(sub):
    """`"storey_extra"` (a balcony's own `sub`, `band['sub'] + "_extra"`)
    -> `"storey"`, its host band's `sub` — every other `sub` unchanged."""
    s = str(sub or "")
    return s[:-len("_extra")] if s.endswith("_extra") else s


def _bay_index(e, m, module, leg):
    """The bay index a piece's own along-side coordinate bins to, in the
    band's own [0, run-length] frame (`urban_building._side_slot`/`_fill`'s
    own coordinate convention: S/E pivots at the LOW end of their span, N/W
    at the HIGH end — but in every case the coordinate ALONG THE SIDE THIS
    MODULE SAMPLES increases monotonically with `_fill`'s own slot order
    for all four sides, verified against `_CORNER_END`'s own lo/hi
    assignment: S-low/N-low both sit at the W-adjacent corner, E-low/W-low
    both sit at the S-adjacent corner — exactly what a raw x-then-y
    ordinate bins to with no direction flip needed).

    `module` is the OWNING BAND's own bay pitch; a `module <= 0` (a
    misconfigured/roofless lookup) bins everything to bay 0 rather than
    dividing by zero.
    """
    side = e["side"]
    along = (e["lx"] + m["W"] / 2.0) if side in ("S", "N") else (e["ly"] + m["D"] / 2.0)
    if module <= 1e-9:
        return 0
    return max(0, int((along - leg) // module))


# ---------------------------------------------------------------------------
# prim_path — real when present, synthesised (pure-planner path) otherwise
# ---------------------------------------------------------------------------
def _synth_path(style, role, side, storey, bay, idx):
    """A stable, unique, JSON-safe placeholder path for the PURE planner
    path (no stage exists yet). Never used once a real `prim_path` has been
    stamped by `scene_generator.apply_placements` (`wreck_kit`'s second
    `adapt()` pass) — `adapt()` only calls this when `p.get("prim_path")`
    is falsy.

    `side == "-"` (the roof/core exclusion sentinel) is not a legal USD
    prim-name character, so it is spelled out as `"none"` here — a plain
    string transform, not a change to the sentinel value itself (`_side`
    on the placement dict stays `"-"`, only this synthesised PATH avoids
    it)."""
    side_tok = "none" if side == "-" else side
    return "/World/_tornado_kit_plan/{0}/{1}_{2}_{3:03d}_{4:02d}_{5:04d}".format(
        style, role, side_tok, int(bay), int(storey), int(idx))


# ---------------------------------------------------------------------------
# THE ADAPTER
# ---------------------------------------------------------------------------
def adapt(placements, info):
    """Stamp `_side`/`_role`/`_storey`/`_bay`/`_size` (and `prim_path` when
    absent) onto every placement in `placements`, IN PLACE, from
    `info["elements"]`'s own `quake_flow.classify`ification — `info` is
    `quake_flow.describe(style, placements, 0, 0, 0)`'s own return value,
    called ONCE by the caller (`plan_for_kit`/`wreck_kit`), not by this
    function — see the module docstring's "THE ADAPTER'S JOB" table for
    exactly where each field comes from. Returns `info["elements"]`
    (the same list `tornado_urban.plan_damage`'s `elements` argument
    wants), mutated but not replaced, so a caller already holding a
    reference to it sees the stamped fields too.

    Idempotent: calling this twice on the same `(placements, info)` pair
    recomputes the identical fields (no accumulated state), the one
    exception being `prim_path`, which is left alone once set — exactly
    the behaviour `wreck_kit` needs for its own two-pass call (adapt once
    for nothing in particular before the real one, if ever; adapt for real
    after the pieces are on the stage and carry real paths).
    """
    style = info["style"]
    spec = _refuse_if_unsupported(style)
    ub = _ub()
    pitch = _band_pitch_table(spec)
    masses = info["masses"]
    els = info["elements"]
    for idx, e in enumerate(els):
        p = e["p"]
        m = masses.get(e.get("mass") or "main") or masses["main"]
        name = e.get("name") or qf._piece_name(p)
        role_c = e.get("role")

        if role_c in _CORNER_ROLES:
            side = _corner_of(ub, name, e["lx"], e["ly"], e.get("yaw"))
            trole = _CORNER_ROLES[role_c]
        elif role_c in _EXCLUDED_ROLES:
            side = "-"
            trole = _EXCLUDED_ROLES[role_c]
        elif role_c == "parapet":
            side = e.get("side")
            trole = "parapet"
        else:  # classify()'s generic "wall" bucket
            side = e.get("side")
            trole = "wall" if _is_glazed_name(name) else "pier"

        storey = int(e.get("storey", 0))
        sx, sy, sz = _extents(ub, name, side)

        if side in qs.SIDES:
            base_sub = _base_sub(e.get("sub"))
            module, leg = pitch.get(base_sub, (0.0, 0.0))
            k = _bay_index(e, m, module, leg)
            sub_ix = 1 if trole == "wall" else 0
            bay_field = k * 3 + sub_ix
        else:
            bay_field = 0

        p["_side"] = side
        p["_role"] = trole
        p["_storey"] = storey
        p["_bay"] = int(bay_field)
        p["_size"] = (float(sx), float(sy), float(sz))
        if not p.get("prim_path"):
            p["prim_path"] = _synth_path(style, trole, side, storey,
                                         bay_field, idx)
    return els


# ===========================================================================
# ROUND 4, DEFECT D1 (stream K) — THE KIT GUARD
# ===========================================================================
# The round-3 bench put three kit cells in front of the user and got this
# back, verbatim: *"These are concrete/brick buildings. They are sturdy,
# they aren't meant to have full collapses in tornados ... Random single
# coloured rectangles in places, floating elements everywhere."*
#
# MEASURED, before any of this landed (`plan_for_kit`, bench seeds, the
# bench's own synthetic wind):
#
#     CELL  STYLE            LEVEL  PIECES  REMOVED  count_frac  area_frac
#     B1    brownstone_row   T4      208      94       0.452      0.344
#     B2    dw_terrace       T3       92      11       0.120      0.147
#     B3    walkup           T4      206      25       0.121      0.107
#
# B1 is the headline. 54 of those 94 pieces were `t_facade_collapse` taking
# the WHOLE south elevation floor-to-roof (the ground storey included — it
# is the one recipe allowed to) and another 40 were `t_top_storey_loss`
# ledgering every roof tile. Both are legitimate §8c states for a REAL
# sliced masonry building and for the industrial shed; on a KIT style they
# are what "full collapse" means to the reviewer, and they are what this
# guard takes away from the kit path.
#
# THREE PARTS, in the order they run:
#
#   1. `kit_recipes` / `KIT_LADDER_T` — the RECIPE-LIST REWRITE (the
#      `tornado_urban._guard` / `quake_sliced._guard` pattern: refuse what
#      the class does not support, and SAY SO in the plan's own notes
#      rather than letting a recipe silently no-op). Two recipes banned
#      outright on kit, `cladding_band` retuned from a 2-4 storey x 40-70 %
#      band to a 1-2 storey x 20-45 % one, `chunk` restricted to corner
#      pieces in the top two storeys of its own region.
#   2. `_support_closure` — THE SUPPORT POST-PASS. No surviving piece
#      stands on air. Implemented on the piece GRID's own vocabulary
#      (side/storey/bay), so it needs no geometry, no `pxr`, and runs on a
#      sliced building's grid exactly as it runs on a kit one.
#   3. `_trim_to_budget` — the numeric LOOK cap. §2.6's height-class table
#      (`tornado_urban.HEIGHT_CAPS`) is the HARD ceiling and already runs
#      inside `plan_damage`; `KIT_MAX_COUNT_FRAC` below is the stricter
#      LOOK target this round asks for (T4 <= 0.20 of the building's own
#      pieces, T3 <= 0.12), enforced on the SAME count metric the review
#      quoted ("92/208 = 0.44").
#
# Everything downstream of the plan — the debris ledger, the per-side
# stats, `tornado_urban._shed_unsupported_roof` — is RE-RUN against the
# guarded removal set rather than patched, so a guarded plan is
# indistinguishable in shape from one `plan_damage` produced directly
# (`test_guarded_plan_is_still_a_valid_plan`).
#
# TWO ENV KNOBS, both defaulting to the guarded behaviour:
#   TK_GUARD=0            the whole guard off — the round-3 look, for an
#                         A/B when something reads wrong.
#   TK_ALLOW_COLLAPSE=1   put `facade_collapse`/`top_storey_loss` back on
#                         the KIT ladder (they are never removed from the
#                         SLICED path, which does not come through this
#                         module at all).
# ===========================================================================

TK_GUARD_ON = os.environ.get("TK_GUARD", "1").strip().lower() not in (
    "0", "false", "no")
TK_ALLOW_COLLAPSE = os.environ.get(
    "TK_ALLOW_COLLAPSE", "0").strip().lower() in ("1", "true", "yes")

#: Recipes a KIT building never gets, and exactly why. Both stay on the
#: sliced path (`tornado_urban_usd.wreck_urban`, another stream's file) and
#: on the industrial shed (`tornado_collapse`) — this table is the KIT
#: ladder's own refusal list, not a change to the §8c collapse classes.
KIT_BANNED_RECIPES = {
    "facade_collapse":
        "a whole windward elevation off, floor to roof, the GROUND storey "
        "included. On the bench's own brownstone_row T4 that single recipe "
        "was 54 of 94 removed pieces and is what the round-4 verdict "
        "names: 'these are concrete/brick buildings. They are sturdy, they "
        "aren't meant to have full collapses in tornados.' Still available "
        "to a real SLICED masonry building and to the industrial shed",
    "top_storey_loss":
        "the top storey's exterior wall off 2-3 sides PLUS every roof "
        "piece ledgered as debris. A kit roof is a TILE GRID (measured: 40 "
        "tiles on brownstone_row), so that recipe alone is 0.19 of the "
        "building's own piece count and reads as a decapitation. The top "
        "storey still loses fabric here -- `out_of_plane_top` peels the "
        "windward top 1-2 storeys, `parapet_fall` takes the coping, and "
        "`_shed_unsupported_roof` sheds exactly the tiles left standing "
        "over an emptied bay column -- the difference is that every piece "
        "that goes is EARNED by a hole under it",
}

#: Per-recipe keyword rewrites for the kit ladder. Merged OVER the ladder's
#: own kwargs (so `rc`'s `keep_pier` on `chunk`/`cladding_band` survives).
KIT_RECIPE_KW = {
    # 2-4 storeys x 40-70 % of a side's bays is most of an elevation on a
    # 4-6 storey kit style. 1-2 x 20-45 % is the "toothed upper-half band"
    # the round-4 brief asks for and what the damage record actually shows
    # on brick/masonry mid-rise.
    "cladding_band": {"storeys": (1, 2), "bay_frac": (0.30, 0.55)},
    # `t_chunk`'s own rc_glass restriction, applied to every kit type: the
    # corner block only leaves in the top two storeys of the chunk's own
    # region, so a corner loss never reads as a column of the building
    # having been cut out.
    "chunk": {"corner_top_storeys": 2},
    # Coping off THREE of the four elevations is most of a parapet ring,
    # and once the retune above shrinks the windward band it is also most
    # of what is left removed on a kit T3/T4 plan -- measured on
    # `commercial_mid` T4 (seed 21, bearing 90): S+SW was 6 of 12 removed
    # pieces, below the windward-dominance bar this file's own
    # `test_windward_dominance_bearing_90_hits_south` sets, purely because
    # `parapet_fall` was spending 5 of them on S/W/E. Two sides keeps the
    # coping loss where the wind is.
    "parapet_fall": {"n_sides": 2},
}

#: The LOOK cap, as a fraction of the building's own TOTAL piece count —
#: the metric the round-4 review quoted ("92/208 = 0.44"). §2.6's
#: height-class table (`tornado_urban.HEIGHT_CAPS`, by façade AREA) is the
#: independent hard ceiling and runs inside `plan_damage` already; this one
#: is stricter and is what decides the picture.
KIT_MAX_COUNT_FRAC = {"T0": 0.02, "T1": 0.06, "T2": 0.11,
                      "T3": 0.12, "T4": 0.20}
#: ... and the same cap re-stated on FAÇADE AREA, so a style whose few
#: pieces are huge cannot pass the count test with half its elevation gone.
KIT_MAX_AREA_FRAC = {"T0": 0.03, "T1": 0.08, "T2": 0.14,
                     "T3": 0.16, "T4": 0.25}

#: Roles that carry a building's own structure. A piece of one of these at
#: storey 0 is NEVER removed by this ladder on a kit building (glass loss
#: on the same piece is untouched — a broken shopfront is not a missing
#: shopfront).
KIT_STRUCT_ROLES = ("wall", "pier", "corner")
#: Roles the budget trim may hand back. Parapet/coping and roof pieces are
#: the FIRST thing a tornado takes and are never the "gutted" defect, so
#: they are kept even when the trim is otherwise hungry.
KIT_TRIMMABLE_ROLES = ("wall", "pier", "corner")
#: How far (metres, plan distance in the mass's own local frame) an
#: ornament / portico / pediment / balcony piece looks for the wall it is
#: attached to. 5 m covers one bay either side on every measured kit style
#: (widest bay module: 8 m, `office`'s ground band).
KIT_ORNAMENT_RADIUS_M = 5.0
#: ROUND 4 v7 (lead review): *"check the roof piece/parapet band no longer
#: overhangs where the top storey is fully open on a side ... if >~50% of a
#: side's top-storey pieces are gone the parapet band above that side
#: should shed too"* — B1's roofline band floating over its emptied
#: top-storey street wall. `tornado_urban._shed_unsupported_roof` already
#: sheds a parapet with no SURVIVING wall/pier/corner piece within 6 m
#: (`_ROOF_SUPPORT_RADIUS_M`), which is a LOCAL test: on a 38 m elevation
#: with the wall gone in the middle and one pier still standing at each
#: end, every parapet piece within 6 m of an end pier stays, and the band
#: reads as continuous coping over a hole. This is the SIDE-WIDE test that
#: catches exactly that case.
KIT_PARAPET_SIDE_SHED_FRAC = 0.50
_SUPPORT_MAX_PASSES = 12
_TRIM_MAX_ROUNDS = 16


def _rewrite_recipes(recs):
    """One ladder cell (a `[(name, kwargs), ...]` list) rewritten for a KIT
    building. Returns `(recipes, notes)`."""
    out, notes = [], []
    for name, kw in recs:
        kw = dict(kw or {})
        if name in KIT_BANNED_RECIPES and not TK_ALLOW_COLLAPSE:
            notes.append("kit guard: {0} refused on a kit building -- "
                         "{1}.".format(name, KIT_BANNED_RECIPES[name]))
            continue
        patch = KIT_RECIPE_KW.get(name)
        if patch:
            before = {k: kw.get(k) for k in patch}
            kw.update(patch)
            notes.append("kit guard: {0} retuned {1} -> {2} (a kit "
                         "masonry/brick elevation takes a toothed band, "
                         "not most of a face)".format(name, before, patch))
        out.append((name, kw))
    return out, notes


def _build_kit_ladder():
    """`tornado_urban.LADDER_T`, rewritten cell by cell. Built ONCE at
    import time; the per-cell notes are kept beside it so `plan_for_kit` /
    `wreck_kit` can stamp exactly the ones that applied to the (btype,
    level) actually planned into `plan["notes"]`."""
    ladder, notes = {}, {}
    for btype, by_level in tu.LADDER_T.items():
        ladder[btype] = {}
        for level, recs in by_level.items():
            ladder[btype][level], notes[(btype, level)] = _rewrite_recipes(recs)
    return ladder, notes


KIT_LADDER_T, KIT_LADDER_NOTES = _build_kit_ladder()


@contextlib.contextmanager
def kit_ladder_installed():
    """`tornado_urban.LADDER_T` swapped for `KIT_LADDER_T` for the duration
    of one `plan_damage` call, then restored in a `finally`.

    WHY A SWAP AND NOT A POST-HOC RESTORE PASS. `tornado_urban.plan_damage`
    reads its recipe list off the module-level `LADDER_T` and takes no
    recipe argument; `tornado_urban.py` is another stream's file this round
    and cannot be given one. The two alternatives were (a) un-removing
    every piece a banned recipe touched AFTER the plan is finished — which
    means reasoning backwards out of `plan["regions"]`, and leaves that
    recipe's macroblocks/roof-shed/notes behind to be unpicked one by one —
    or (b) forking `plan_damage`'s twenty-line body into this module, which
    is the one thing this module exists NOT to do (see the docstring's "WHY
    AN ADAPTER, NOT A PLANNER FORK"). The swap keeps ONE planner, runs
    every downstream pass (`_finalise`'s caps, the tear pass, the roof
    shed, the ledger) against the reduced recipe set exactly as if the
    ladder had always read this way, and is scoped to a single call.
    Single-threaded by construction — every caller in this repo builds one
    building at a time.
    """
    if not TK_GUARD_ON:
        yield False
        return
    saved = tu.LADDER_T
    tu.LADDER_T = KIT_LADDER_T
    try:
        yield True
    finally:
        tu.LADDER_T = saved


# ---------------------------------------------------------------------------
# THE SUPPORT POST-PASS — "no piece stands on air"
# ---------------------------------------------------------------------------
def _cell_below(g, side, storey, bay):
    """The pieces DIRECTLY under `(side, storey, bay)` — the nearest storey
    below that actually carries a piece in that bay column, not blindly
    `storey - 1`.

    A kit style's band table is not uniform: `dw_terrace` spends a whole
    storey INDEX on a 1.2 m trim band between two 6 m window bands (the
    round-2 measurement `t_chunk`'s own docstring records), and a bay that
    is empty at that index is not evidence of a hole — the wall below it is
    still there two indices down. Returns `[]` when nothing at all sits
    under this cell (a cantilever the ladder never built and must not
    invent a verdict about)."""
    for s in range(int(storey) - 1, -1, -1):
        got = g.at((side, s, bay))
        if got:
            return got
    return []


def _corner_below(g, corner, storey):
    """`_cell_below` for a corner column (`_Grid.corners`)."""
    for s in range(int(storey) - 1, -1, -1):
        got = g.corners.get((corner, s))
        if got:
            return got
    return []


def _column_dead(g, side, storey, bay, removed):
    """Is the whole bay column under `(side, storey, bay)` gone?"""
    below = _cell_below(g, side, storey, bay)
    if not below:
        return False
    return all(qs._path(e) in removed for e in below)


def _unsupported(g, e, removed):
    """Does element `e` stand on air, given the `removed` path set?

    THE RULE, per role:

    * `wall` / `pier` (a main-side S/E/N/W run piece) — unsupported when
      its OWN bay column below is entirely gone AND it has no live
      NEIGHBOUR to span to: an adjacent bay at its own storey that both
      survives and has a live column of its own. That second clause is
      what lets `quake_sliced._apply_region`'s toothing keep working — a
      single kept pier inside a lost band is standing on the bay next to
      it, which is the whole point of toothing — while B1's own defect
      (a window assembly with black on every side of it) still fails.
    * `corner` / `parapet_corner` — the column test ONLY. A corner is the
      END of both its runs; there is no bay beyond it to span to, so
      "the corner block two storeys above a corner hole" is exactly the
      floating chunk the review circled and it goes.
    * `core` (portico / pediment / ornament / balcony — `_EXCLUDED_ROLES`,
      never addressable as a grid cell and therefore never removed by any
      recipe) — unsupported when EVERY wall/pier/corner piece within
      `KIT_ORNAMENT_RADIUS_M` of it in plan, at its own storey or one
      either side, is gone. This is B1's stranded ornate assembly at the
      roof line: the ladder cannot remove it, so before this pass nothing
      ever could.
    * `roof` — NOT decided here. `tornado_urban._shed_unsupported_roof`
      already owns roof tiles and slabs (an area test for a deck, a local
      support radius for a tile) and is re-run after this pass.
    """
    p = e.get("p") or {}
    role = p.get("_role")
    if role == "roof":
        return False
    side = p.get("_side")
    storey = int(p.get("_storey", 0))

    if side in qs.SIDES:
        if storey <= 0:
            return False
        bay = qs.bay_no(p, g.n_sub)
        if not _column_dead(g, side, storey, bay, removed):
            return False
        if role in ("corner", "parapet_corner"):
            return True
        for nb in (bay - 1, bay + 1):
            for e2 in g.at((side, storey, nb)):
                if qs._path(e2) in removed:
                    continue
                if not _column_dead(g, side, storey, nb, removed):
                    return False
        return True

    if side in qs._CORNER_SIDES:
        if storey <= 0:
            return False
        below = _corner_below(g, side, storey)
        if not below:
            return False
        return all(qs._path(e2) in removed for e2 in below)

    # `core` and anything else off the grid: the attachment test.
    if role != "core":
        return False
    mass = e.get("mass") or "main"
    lx, ly = float(e.get("lx", 0.0)), float(e.get("ly", 0.0))
    r2 = KIT_ORNAMENT_RADIUS_M * KIT_ORNAMENT_RADIUS_M
    near = []
    for e2 in g.els:
        p2 = e2.get("p") or {}
        if p2.get("_role") not in KIT_STRUCT_ROLES:
            continue
        if (e2.get("mass") or "main") != mass:
            continue
        if abs(int(p2.get("_storey", 0)) - storey) > 1:
            continue
        dx = float(e2.get("lx", 0.0)) - lx
        dy = float(e2.get("ly", 0.0)) - ly
        if dx * dx + dy * dy <= r2:
            near.append(qs._path(e2))
    if not near:
        return False
    return all(q in removed for q in near)


def _shed_open_side_parapets(g, removed, plan, note):
    """Shed the parapet/coping band above any elevation whose TOP WALL BAND
    is more than `KIT_PARAPET_SIDE_SHED_FRAC` gone. Returns the paths shed.

    `tornado_urban._shed_unsupported_roof`'s parapet test is a 6 m local
    radius, so a parapet piece near a surviving end pier passes it even
    when the 30 m of wall it actually spans has gone. The user's own read
    of that on the v6 bench: the roofline band floats over the emptied
    top-storey street wall. A parapet is a cantilever off the wall below
    it, so the honest rule is per ELEVATION, not per 6 m disc.

    A `parapet_corner` goes if EITHER of its two legs qualifies — it is
    supported at both ends, and losing one is enough.
    """
    by_mass_side = {}
    for e in g.els:
        p = e.get("p") or {}
        if p.get("_role") not in KIT_STRUCT_ROLES:
            continue
        sd = p.get("_side")
        if sd not in qs.SIDES:
            continue
        mass = e.get("mass") or "main"
        by_mass_side.setdefault((mass, sd), []).append(e)
    # the TOP wall band per mass — the storey a parapet sits on
    top_band = {}
    for (mass, sd), els in by_mass_side.items():
        st = max(int((e.get("p") or {}).get("_storey", 0)) for e in els)
        top_band[mass] = max(top_band.get(mass, -1), st)
    open_sides = set()
    for (mass, sd), els in by_mass_side.items():
        band = [e for e in els
                if int((e.get("p") or {}).get("_storey", 0)) == top_band.get(mass, -1)]
        if not band:
            continue
        gone = sum(1 for e in band if qs._path(e) in removed)
        if gone / float(len(band)) > KIT_PARAPET_SIDE_SHED_FRAC:
            open_sides.add((mass, sd))
    if not open_sides:
        return []
    moved = set(plan.get("displaced") or {})
    shed = []
    for e in g.els:
        p = e.get("p") or {}
        if p.get("_role") not in ("parapet", "parapet_corner"):
            continue
        path = qs._path(e)
        if not path or path in removed or path in moved:
            continue
        mass = e.get("mass") or "main"
        sd = p.get("_side")
        legs = (sd,) if sd in qs.SIDES else qs._CORNER_SIDES.get(sd, ())
        if any((mass, leg) in open_sides for leg in legs):
            shed.append(path)
    if shed:
        note("kit guard: {0} parapet/coping piece(s) shed -- the top wall "
             "band under them is more than {1:.0f}% gone on {2} "
             "elevation(s); a parapet is a cantilever off that wall, and "
             "the 6 m local support test keeps a whole band standing over "
             "a hole as long as one end pier survives".format(
                 len(shed), 100.0 * KIT_PARAPET_SIDE_SHED_FRAC,
                 len(open_sides)))
    return shed


def _support_closure(g, seeds, extra=()):
    """`seeds` (plus `extra`) grown until nothing standing is unsupported.
    Returns `(removed_set, shed_paths)` — `shed_paths` in discovery order,
    so a caller can report/ledger exactly what the SUPPORT rule took (as
    opposed to what a recipe took).

    `extra` is the set of pieces some OTHER pass already took and that the
    budget trim may not hand back — in practice `tornado_urban.
    _shed_unsupported_roof`'s roof/parapet sheds, which are earned by a
    hole under them and are never the gutted-box defect."""
    removed = set(seeds) | set(extra)
    shed = []
    for _ in range(_SUPPORT_MAX_PASSES):
        newly = []
        for e in g.els:
            path = qs._path(e)
            if not path or path in removed:
                continue
            if _unsupported(g, e, removed):
                newly.append(path)
        if not newly:
            break
        removed.update(newly)
        shed.extend(newly)
    return removed, shed


# ---------------------------------------------------------------------------
# THE LOOK BUDGET
# ---------------------------------------------------------------------------
def _keep_score(e, weights, top_band):
    """How much a removed piece BELONGS in a tornado's damage zone —
    the trim hands back the lowest scores first. Highest is best.

    The round-4 brief's own target shape, in order: parapets/coping, then
    the top one-two storeys, then the windward face, then the windward
    corner."""
    p = e.get("p") or {}
    role = p.get("_role")
    side = p.get("_side")
    storey = int(p.get("_storey", 0))
    score = 0.0
    if role in ("parapet", "parapet_corner"):
        score += 6.0
    elif role == "roof":
        score += 5.0
    if storey >= top_band:
        score += 4.0
    elif storey >= top_band - 1:
        score += 2.0
    score += 3.0 * float(weights.get(side, 0.0))
    if side in qs._CORNER_SIDES:
        score += 1.5
    return score


def _facade_area(e):
    return tu._facade_area_of(e)


def _measure(g, removed):
    """`(count_frac, area_frac)` for a removal set — the SAME two metrics
    `plan["stats"]` reports (`removed_count_frac`, `removed_frac`)."""
    n_all = len(g.els) or 1
    total_area = tu._total_facade_area(g)
    area = 0.0
    for e in g.els:
        p = e.get("p") or {}
        if qs._path(e) in removed and p.get("_side") in qs.SIDES:
            area += _facade_area(e)
    return len(removed) / float(n_all), area / total_area


def _trim_to_budget(g, seeds, level, weights, top_band, extra=()):
    """Hand back the lowest-scoring TRIMMABLE seeds until the support
    closure of what is left is inside both `KIT_MAX_COUNT_FRAC[level]` and
    `KIT_MAX_AREA_FRAC[level]`.

    The closure is recomputed from the SEEDS every round rather than
    incrementally un-shedding: restoring a seed can only ever ADD support,
    so a piece the support pass took in an earlier round may legitimately
    stand again, and recomputing is the only way to see that. Returns
    `(seeds, removed, shed, n_restored)`."""
    cap_n = KIT_MAX_COUNT_FRAC.get(level, KIT_MAX_COUNT_FRAC["T4"])
    cap_a = KIT_MAX_AREA_FRAC.get(level, KIT_MAX_AREA_FRAC["T4"])
    idx = {qs._path(e): e for e in g.els}
    seeds = set(seeds)
    extra = set(extra)
    n_all = len(g.els) or 1
    n_restored = 0
    removed, shed = _support_closure(g, seeds, extra)
    for _ in range(_TRIM_MAX_ROUNDS):
        cfrac, afrac = _measure(g, removed)
        if cfrac <= cap_n and afrac <= cap_a:
            break
        cands = [q for q in seeds
                 if q not in extra
                 and (idx.get(q) or {}).get("p", {}).get("_role")
                 in KIT_TRIMMABLE_ROLES]
        if not cands:
            break
        cands.sort(key=lambda q: (_keep_score(idx[q], weights, top_band), q))
        # How many must go for the COUNT metric, at least one per round so
        # the AREA metric alone still converges.
        want = max(1, int(math.ceil((cfrac - cap_n) * n_all))) if cfrac > cap_n else 1
        drop = cands[:min(want, len(cands))]
        for q in drop:
            seeds.discard(q)
        n_restored += len(drop)
        removed, shed = _support_closure(g, seeds, extra)
    return seeds, removed, shed, n_restored


# ---------------------------------------------------------------------------
# THE GUARD ITSELF
# ---------------------------------------------------------------------------
def kit_guard(plan, info, elements, rng, wind, intensity, verbose=True):
    """Rewrite one finished `tornado_urban.plan_damage` plan into the state
    a STURDY masonry/brick kit building is allowed to be in, and re-derive
    everything downstream of the removal set. Mutates `plan` in place and
    returns a small counts dict.

    ORDER, and why each step is where it is:

      1. GROUND STOREY RESTORED. Every `wall`/`pier`/`corner` piece at
         storey 0 comes back. `tornado_urban._enforce_ground_floor_rule`
         only does this for a TOWER and only for piers; the round-4 rule is
         "ground storey structural pieces are NEVER removed (glass loss
         ok)" for every kit building, because a missing ground storey is
         the single thing that makes a standing building read as collapsed.
         `plan["glass"]` is untouched — a blown-out shopfront still blows
         out.
      2. TRIM TO BUDGET + SUPPORT CLOSURE (`_trim_to_budget`), interleaved:
         the closure is what actually decides the removal set, and the trim
         only ever hands back SEEDS, so the two cannot fight.
      3. ROOF/PARAPET RE-SHED. `tornado_urban._shed_unsupported_roof`, run
         again against the guarded holes. That pass only ever ADDS, so a
         tile it shed against the PRE-guard removal set stays shed even if
         the trim has since restored the wall under it: shedding a roof
         tile is never the floating defect, so the conservative direction
         is the safe one. Noted, not hidden.
      4. BUDGET + SUPPORT AGAIN, with step 3's sheds carried in as FIXED
         (never trim candidates, always in the closure). Without this a
         late roof shed pushes a plan back over the cap with nothing left
         to check it -- measured on `office`/`commercial_mid` T4, which
         landed at 0.204 / 0.207 against a 0.20 cap.
      5. DISPLACED pieces that ended up unsupported are DEMOTED to removed
         (a hanging panel with nothing under or beside it is a floating
         panel), their macroblock records go with them, and ONE more
         closure runs because a demotion is itself a new removal.
      6. TEARS on pieces that are now gone are dropped.
      7. The DEBRIS LEDGER and every removal-derived `stats` field are
         RE-RUN (`tornado_urban._ledger_removed`), not patched: a fragment
         whose source piece is standing again would otherwise sit in the
         street with nothing missing above it.
    """
    counts = {"restored_ground": 0, "restored_budget": 0, "shed_support": 0,
              "shed_roof": 0, "shed_parapet": 0, "demoted_displaced": 0,
              "dropped_tears": 0}
    if not TK_GUARD_ON:
        plan.setdefault("notes", []).append(
            "kit guard: DISABLED (TK_GUARD=0) -- round-3 behaviour")
        plan["kit_guard"] = dict(counts, enabled=False)
        return counts

    g = qs._Grid(info, elements)
    idx = {qs._path(e): e for e in g.els}
    level = str(plan.get("level"))
    weights = plan.get("side_weights") or {}
    notes = plan.setdefault("notes", [])
    before_c = float((plan.get("stats") or {}).get("removed_count_frac") or 0.0)
    before_a = float((plan.get("stats") or {}).get("removed_frac") or 0.0)
    n_before = len(plan.get("removed") or ())

    struct_storeys = [int((e.get("p") or {}).get("_storey", 0)) for e in g.els
                      if (e.get("p") or {}).get("_role") in KIT_STRUCT_ROLES]
    top_band = max(struct_storeys) if struct_storeys else g.top

    # -- 1) ground storey ---------------------------------------------------
    seeds = set(plan.get("removed") or ())
    ground = {q for q in seeds
              if (idx.get(q) or {}).get("p", {}).get("_storey", -1) == 0
              and (idx.get(q) or {}).get("p", {}).get("_role") in KIT_STRUCT_ROLES}
    if ground:
        seeds -= ground
        counts["restored_ground"] = len(ground)
        notes.append(
            "kit guard: {0} ground-storey wall/pier/corner piece(s) "
            "restored -- a kit building never loses its ground storey "
            "(glass loss on the same bays is untouched)".format(len(ground)))

    # -- 2) budget + support ------------------------------------------------
    seeds, removed, shed, n_restored = _trim_to_budget(
        g, seeds, level, weights, top_band)
    counts["restored_budget"] = n_restored
    counts["shed_support"] = len(shed)
    if n_restored:
        notes.append(
            "kit guard: {0} piece(s) restored to bring the removal inside "
            "the kit T-level look cap (count {1:.2f} / area {2:.2f}) -- "
            "lowest-scoring first, so what survives is the windward top "
            "storeys, the windward corner and the coping".format(
                n_restored, KIT_MAX_COUNT_FRAC.get(level, 0.0),
                KIT_MAX_AREA_FRAC.get(level, 0.0)))

    plan["removed"] = sorted(removed)
    plan["_removed_set"] = set(removed)

    # -- 3) roof / parapet re-shed -----------------------------------------
    pctx = tu._pctx(info, elements, plan.get("btype") or info.get("type"),
                    rng, plan, wind, weights, plan.get("height_class"),
                    intensity)
    n_roof, n_parapet = tu._shed_unsupported_roof(pctx, plan)
    counts["shed_roof"], counts["shed_parapet"] = int(n_roof), int(n_parapet)
    # ... plus the SIDE-WIDE parapet rule that pass has no notion of.
    side_shed = _shed_open_side_parapets(
        g, set(plan["removed"]), plan, lambda t: notes.append(t))
    for q in side_shed:
        plan["removed"].append(q)
        plan["_removed_set"].add(q)
    counts["shed_parapet"] += len(side_shed)
    # Whatever that pass took beyond the closure is FIXED from here on: a
    # roof tile or a coping run over an emptied bay column is earned by the
    # hole under it, and rule 4 of the round's own brief ("no piece stands
    # on air") outranks the cosmetic budget. It is excluded from the trim's
    # candidate list and carried into the closure instead.
    fixed = set(plan["removed"]) - removed

    # -- 4) budget + support, ONCE MORE, now that the roof pass has had its
    #       say -- otherwise a late shed pushes a plan back over the cap
    #       with nothing left to check it (measured: office/commercial_mid
    #       T4 landed at 0.204/0.207 against a 0.20 cap before this).
    seeds, removed, shed, n_restored2 = _trim_to_budget(
        g, seeds, level, weights, top_band, extra=fixed)
    counts["restored_budget"] += n_restored2
    counts["shed_support"] = len(shed)
    plan["removed"] = sorted(removed)
    plan["_removed_set"] = set(removed)
    if counts["shed_support"]:
        notes.append(
            "kit guard: support post-pass shed {0} piece(s) that would "
            "have stood on air (bay column below entirely gone and no "
            "live neighbour to span to; ornaments within {1:.0f} m of "
            "nothing standing)".format(counts["shed_support"],
                                       KIT_ORNAMENT_RADIUS_M))

    # -- 5) displaced ------------------------------------------------------
    disp = plan.get("displaced") or {}
    demote = []
    for q in list(disp):
        e = idx.get(q)
        if e is None:
            continue
        if q in removed or _unsupported(g, e, removed):
            demote.append(q)
    for q in demote:
        disp.pop(q, None)
        if q not in removed:
            removed.add(q)
    if demote:
        counts["demoted_displaced"] = len(demote)
        plan["macroblocks"] = [mb for mb in (plan.get("macroblocks") or ())
                               if mb.get("path") not in set(demote)]
        # A demotion ADDS a removal, which can strand whatever was leaning
        # on it -- one last closure, or the invariant this whole pass
        # asserts (`test_no_surviving_piece_stands_on_air`) would hold
        # everywhere except right here.
        removed, shed3 = _support_closure(g, removed)
        counts["shed_support"] += len(shed3)
        plan["removed"] = sorted(removed)
        plan["_removed_set"] = set(removed)
        notes.append(
            "kit guard: {0} displaced piece(s) demoted to removed -- a "
            "leaning/hanging panel with nothing under or beside it is a "
            "floating panel".format(len(demote)))

    # -- 6) tears ----------------------------------------------------------
    tears = list(plan.get("tears") or ())
    kept_tears = [t for t in tears if t.get("path") not in removed]
    if len(kept_tears) != len(tears):
        counts["dropped_tears"] = len(tears) - len(kept_tears)
        plan["tears"] = kept_tears
        notes.append("kit guard: {0} tear job(s) dropped -- their piece is "
                     "no longer standing".format(counts["dropped_tears"]))

    # -- 7) ledger + stats -------------------------------------------------
    _rebuild_ledger(plan, info, elements, pctx, wind, intensity)
    plan.pop("_removed_set", None)
    # The two shed counters `_finalise` wrote describe the roof pass that
    # ran INSIDE `plan_damage`; step 3 above ran it a second time against
    # the guarded holes, and a reader (or `test_caps_hold_by_height_class`,
    # which uses `n_roof_shed` as the "this roof piece was accounted for by
    # a named mechanism" evidence) must see BOTH.
    plan["stats"]["n_roof_shed"] = int(
        plan["stats"].get("n_roof_shed", 0)) + counts["shed_roof"]
    plan["stats"]["n_parapet_shed"] = int(
        plan["stats"].get("n_parapet_shed", 0)) + counts["shed_parapet"]
    plan["stats"]["n_support_shed"] = int(counts["shed_support"])

    st = plan["stats"]
    notes.append(
        "kit guard post-pass: removed {0} -> {1} piece(s); count_frac "
        "{2:.3f} -> {3:.3f} (cap {4:.2f}), area_frac {5:.3f} -> {6:.3f} "
        "(cap {7:.2f}) -- these two BEFORE numbers are already under the "
        "rewritten kit ladder; the recipe-list rewrite above is where the "
        "bulk of the reduction happens".format(
            n_before, st["n_removed"], before_c, st["removed_count_frac"],
            KIT_MAX_COUNT_FRAC.get(level, 0.0), before_a,
            st["removed_frac"], KIT_MAX_AREA_FRAC.get(level, 0.0)))
    # NAMED HONESTLY: `*_after_ladder` is what `plan_damage` produced under
    # the REWRITTEN kit ladder (`KIT_LADDER_T` — that rewrite is where the
    # bulk of the reduction happens: measured on the bench's own B1,
    # 0.452 -> 0.130), and `*_final` is after this post-pass, which mostly
    # ADDS (support sheds) and only subtracts when the budget bites.
    plan["kit_guard"] = dict(counts, enabled=True,
                             count_frac_after_ladder=before_c,
                             count_frac_final=st["removed_count_frac"],
                             area_frac_after_ladder=before_a,
                             area_frac_final=st["removed_frac"])
    if verbose:
        print("[tornado_kit] guard: {0}".format(plan["kit_guard"]))
    return counts


def _rebuild_ledger(plan, info, elements, pctx, wind, intensity):
    """`tornado_urban._ledger_removed` + every removal-derived `stats`
    field, re-run against the guarded removal set. Mirrors
    `tornado_urban._finalise`'s own stats block field for field (the fields
    it computes from `removed`/`debris`); the ones that describe the RECIPE
    pass (`n_tears`, `n_glass_candidates`, `glass_measured`, ...) are
    carried over from the plan `plan_damage` already produced."""
    g = pctx["g"]
    stats = dict(plan.get("stats") or {})
    region = stats.get("region")
    region = tuple(region) if region else None
    ledger = tu._ledger_removed(pctx, plan, wind, intensity, region=region)
    plan["debris"] = ledger["frags"]

    removed = sorted(set(plan["removed"]))
    plan["removed"] = removed
    idx = {qs._path(e): e for e in g.els}
    per_side, max_storey, removed_area = {}, -1, 0.0
    for q in removed:
        e = idx.get(q)
        if not e:
            continue
        pp = e.get("p") or {}
        max_storey = max(max_storey, int(pp.get("_storey", 0)))
        sd = pp.get("_side")
        if sd in qs.SIDES:
            per_side[sd] = per_side.get(sd, 0) + 1
            removed_area += tu._facade_area_of(e)
    total_area = tu._total_facade_area(g)
    debris_volume = sum(f["size"][0] * f["size"][1] * f["size"][2]
                        for f in plan["debris"])
    stats.update({
        "n_pieces": len(g.els), "n_removed": len(removed),
        "removed_frac": float(removed_area / total_area),
        "removed_count_frac": (len(removed) / float(len(g.els))
                               if g.els else 0.0),
        "n_glass": len(plan.get("glass") or ()),
        "n_displaced": len(plan.get("displaced") or {}),
        "n_debris": len(plan["debris"]),
        "debris_volume_m3": float(debris_volume),
        "source_volume_m3": float(ledger["source_volume_m3"]),
        "max_removed_storey": int(max_storey) if max_storey >= 0 else None,
        "removed_by_side": dict(sorted(per_side.items())),
        "n_glass_shards": int(ledger["n_glass_shards"]),
        "glass_shards_thinned": bool(ledger["glass_shards_thinned"]),
        "n_struct_debris": int(ledger["n_struct"]),
        "struct_debris_thinned": bool(ledger["struct_thinned"]),
        "debris_thinned": bool(ledger["struct_thinned"]
                               or ledger["glass_shards_thinned"]),
        "n_berm": int(ledger["n_berm"]), "n_berm_kept": int(ledger["n_berm_kept"]),
        "berm_share_level": float(ledger["berm_share_level"]),
        "n_tears": len(plan.get("tears") or ()),
    })
    plan["stats"] = stats


# ---------------------------------------------------------------------------
# THE PURE PLANNER PATH
# ---------------------------------------------------------------------------
def plan_for_kit(style, level, rng, wind, seed=7, btype=None,
                 height_class=None, intensity=None):
    """`kit_placements` -> `describe` -> `adapt` -> `tornado_urban.
    plan_damage`. PURE — no stage, no `pxr`. Returns `(placements, info,
    plan)`.

    `btype` defaults from `kit_substitute.styles()[style]["type"]` — the
    live table (`urban_building.STYLES` + `quake_flow.FAMILY_TYPE`, NOT a
    frozen archetype bake — see `styles()`'s own docstring for why), so a
    caller that only knows the style name gets the same urm/rc/rc_glass
    classification `LADDER_T` is keyed on without having to import
    `quake_flow.FAMILY_TYPE` itself.
    """
    placements = kit_placements(style, seed)
    info = qf.describe(style, placements, 0.0, 0.0, 0.0)
    elements = adapt(placements, info)
    if btype is None:
        btype = ksub.styles()[style]["type"]
    info["type"] = btype
    plan = _plan_guarded(info, elements, level, btype, rng, wind,
                         height_class, intensity, verbose=False)
    return placements, info, plan


def _plan_guarded(info, elements, level, btype, rng, wind, height_class,
                  intensity, verbose=True):
    """`tornado_urban.plan_damage` under the KIT ladder, then `kit_guard`
    — the one call `plan_for_kit` and `wreck_kit` share, so the pure
    planner path and the stage path can never drift on which guard ran.

    `height_class`/`intensity` are resolved HERE (rather than left to
    `plan_damage`'s own defaults) because `kit_guard` needs both after the
    fact and reading them back off the finished plan would re-derive the
    intensity from the wind a second time."""
    if height_class is None:
        height_class = tu.height_class_for(info.get("H"))
    if intensity is None:
        intensity = float((wind or {}).get(
            "intensity", (wind or {}).get("speed_frac", 0.0)))
    intensity = float(intensity)
    with kit_ladder_installed() as guarded:
        plan = tu.plan_damage(info, elements, level, btype, rng, wind,
                              height_class=height_class, intensity=intensity)
    if guarded:
        for note in KIT_LADDER_NOTES.get((btype, str(level))) or ():
            plan["notes"].append(note)
    kit_guard(plan, info, elements, rng, wind, intensity, verbose=verbose)
    return plan


# ---------------------------------------------------------------------------
# THE STAGE PATH
# ---------------------------------------------------------------------------
def wreck_kit(stage, cell, style, level, rng, nrng, mats, tag, wind, seed=7,
             btype=None, height_class=None, intensity=None, ssf=1.0,
             verbose=True):
    """Apply the urban-tornado ladder to one KIT-style building and return
    `quake_flow.wreck_building`'s ctx shape, unchanged — the same contract
    `tornado_urban_usd.wreck_urban` honours for a sliced building, so a
    later bake launcher can call this wherever it would call that (round 2's
    §6 "next round" item 2: "the bake for [kit buildings] is
    `bake_quake_archetypes`-shaped").

    Sequence (see the module docstring's PIPELINE section for why each step
    is ordered where it is):

      1. `kit_substitute.build_kit(stage, cell + "/parts", style, seed=
         seed, ssf=ssf)` — authors the kit INTO `<cell>/parts`, never
         `cell` itself. `cell` is very often an `Xform` that already
         carries the building's own placement transform (translate/
         rotateZ), and `scene_generator.apply_placements` does
         `UsdGeom.Scope.Define(stage, Sdf.Path(parent_path))` on whatever
         path it is handed — `Scope.Define` on a prim that already has a
         non-Scope type SNAPS IT BACK TO THE ORIGIN (the exact trap
         `.agents/skills/build-urban-fire-scenes/SKILL.md`'s bug catalogue
         and this round's own brief (§1) warn about verbatim: "never
         `UsdGeom.Scope.Define` on a cell that carries a transform").
         `build_kit` returns the placement list with REAL `prim_path`s
         stamped on each dict (`apply_placements`'s own `p["prim_path"] =
         prim_path`, "so callers can post-process").
      2. `tornado_urban_usd.annotate_glazing(stage, placements)` — a
         MEASURED read of each authored piece's own material bindings,
         only possible now that the pieces are actually on the stage;
         stamps `_glass_faces`/`_glass_frac` onto each placement dict so
         `tornado_urban.t_glass_loss` can prefer real evidence over this
         module's name-based `_is_glazed_name` prior (see that recipe's
         own docstring on the two candidate-picking paths).
      3. RE-`describe` + `adapt` against the now-authored placements — the
         `prim_path`s `adapt` sees this time are the REAL ones `build_kit`
         stamped in step 1, not the synthetic placeholders `plan_for_kit`'s
         pure path would have used, so `apply_plan`'s removal/glass-void/
         displacement steps resolve against real stage prims.
      4. `tornado_urban.plan_damage`.
      5. `tornado_urban_usd.apply_plan`.

    `tornado_urban_usd` is imported LAZILY (inside this function, not at
    module scope) — it imports `pxr` at its own module top, and this
    module is otherwise pure (importable, and imported by the pure test
    suite, with no `pxr` on the path at all).
    """
    from . import tornado_urban_usd as tuu

    _refuse_if_unsupported(style)
    placements = ksub.build_kit(stage, cell + "/parts", style, seed=seed,
                                ssf=ssf)
    tuu.annotate_glazing(stage, placements)
    tuu.annotate_surface(stage, placements)  # FX2 HOOK (§8e F3) — one line, mirrors annotate_glazing above; authorised outside FX2's own file list per the round's brief
    # ROUND 2, THE WINDOW-NAME PRIOR (lead, after `tools/_tk_glass_probe.py`
    # measured every kit style): `annotate_glazing` is truthful and finds
    # ZERO glazing on kit builds — Downtown_West window-module subsets bind
    # untextured/shared-atlas materials that fail `is_glazing`, and the MCE
    # families paint windows into one facade atlas. But a module NAMED
    # window/door IS the pane, so pieces annotate measured 0 on get a
    # name-based prior stamp; the planner then lists them as glass and
    # `tornado_urban_usd.apply_plan`'s knock-out branch voids each as a
    # whole module (no material to rebind). MCE styles have no window-named
    # modules and honestly keep zero glass vocabulary.
    n_prior = 0
    for p in placements:
        if int(p.get("_glass_faces") or 0) > 0:
            continue
        cat = str(p.get("category") or "").lower()
        if ("window" in cat) or ("wnd" in cat) or ("door" in cat):
            p["_glass_faces"] = 1
            p["_glass_frac"] = 0.8    # a window module is mostly pane
            n_prior += 1
    if n_prior and verbose:
        print("[tornado_kit] window-name prior: {0} module(s) stamped as "
              "glass (annotate_glazing measured 0 on them — kit materials "
              "carry no glazing signal)".format(n_prior))
    info = qf.describe(style, placements, 0.0, 0.0, 0.0)
    if btype is None:
        btype = ksub.styles()[style]["type"]
    info["type"] = btype
    elements = adapt(placements, info)

    ctx = {"stage": stage, "parent": cell, "info": info, "rng": rng,
           "nrng": nrng, "mats": mats if mats is not None else {},
           "tag": tag, "loose": [], "static_extra": [], "velocity": {},
           "authored": [], "notes": [], "verbose": verbose}
    ctx["kit"] = {"style": style, "btype": btype, "level": level,
                  "intensity": intensity, "height_class": height_class,
                  "n_pieces": len(placements)}

    plan = _plan_guarded(info, elements, level, btype, rng, wind,
                         height_class, intensity, verbose=verbose)
    ctx["plan"] = plan
    ctx["counts"] = tuu.apply_plan(stage, ctx, plan, verbose=verbose)
    if verbose:
        print("[tornado_kit] {0} ({1}, {2}, {3:.0f} m, {4} piece(s)): {5}"
              .format(style, btype, level, info.get("H") or 0.0,
                      len(placements), ctx["counts"]))

    # R7 HOOK (stream RF, `disaster/tornado_roof.py`): same shape as
    # `tornado_urban_usd.wreck_urban`'s own hook -- a SEPARATE rng stream
    # (`tornado_roof.roof_seed(tag)`) so the facade plan above stays
    # byte-identical whether or not this ever runs.
    import random as _random

    from . import tornado_roof
    roof_rng = _random.Random(tornado_roof.roof_seed(tag))
    roof_plan = tornado_roof.plan_roof(info, elements, level, wind, roof_rng,
                                       height_class, intensity,
                                       facade_plan=plan)
    ctx["roof_plan"] = roof_plan
    ctx["roof_counts"] = tornado_roof.apply_roof(stage, ctx, roof_plan,
                                                 verbose=verbose)
    return ctx
