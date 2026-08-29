"""kit_substitute — swap a whole-asset building for its KIT equivalent so the
fire ladder can actually damage it.

THE PROBLEM THIS SOLVES
------------------------
`urban_fire.burn_building` gutts interiors, empties windows, burns through
floors and roofs and collapses a shell — and every one of those recipes works
by taking ELEMENTS away. It therefore needs a building assembled from façade
modules, which `detail/urban_building.build_building` produces and
`quake_flow.describe` turns into an element table.

Every whole-asset pack in the city library is the opposite. MEASURED
(`tools/pack_structure_probe.py`, 2026-08-29) — every one of them is a SINGLE
merged mesh:

    MCE  SM_MERGED_BP_MBuilding01   1 mesh,  4 subsets, "LOD0"
    MCE  SM_MERGED_BP_MBuilding02   1 mesh, 13 subsets, "LOD0"
    MCE  SM_MERGED_BP_MBuilding05   1 mesh, 31 subsets, "LOD0"
    Muyang    BG_Building_A         1 mesh,  5 subsets
    Dmytro    Building_TypeA_A      1 mesh,  9 subsets
    GAC       SM_Building_01        1 mesh, 14 subsets

and those "subsets" are MATERIAL groups spanning the building's whole height,
not pieces. Nothing can be removed from one, nothing can be bound per storey,
so the ladder degenerates to a single flat multiplier over the asset — the
uniform grey box that this exists to stop.

WHY SUBSTITUTION IS HONEST FOR ModernCityEnvironment SPECIFICALLY
------------------------------------------------------------------
`SM_MERGED_BP_MBuilding*` — the name says MERGED — is an export of the SAME
ART as `ModernCityEnvironment01`, the façade kit that
`urban_building.STYLES` families 01-05 are built from (user, 2026-08-29: "both
moderncity asset packs are exports of the same scene ... one is the kit bash
meshes other is the same meshes assembled"). Replacing a merged MCE building
with a kit building is therefore not a substitution of one look for another;
it is the same art, re-assembled from the parts it was welded out of.

It is NOT a 1:1 twin. The merged assets are their own assemblies — MEASURED
(`config/asset_sets/urban.yaml`, the pool this pack is drawn from): MBuilding01
is 28.5 x 18.5 x 29.0 m, MBuilding05 is 27.7 x 20.6 x 62.4 m, MBuilding02 is
91.1 x 96.1 x 68.7 m — so the match is by SIZE, not by name, against the LIVE
style table (see `styles()`), and each of the three has a real candidate
within the gates (`commercial_mid`, `highrise_step`, `block_residential`
respectively — see `check()`). That is also why this module is written
generally rather than as a lookup table: the same mechanism substitutes any
pack, and the only thing that changes per pack is how defensible the swap is.

WHAT THIS DELIBERATELY DOES NOT DO
-----------------------------------
It does not swap a building the fire never reached. An intact whole-asset
building looks better than an intact kit one — that is why the city draws from
these packs at all — so the swap happens ONLY where the spread solve says the
building burns, and the city keeps its own stock everywhere else.

SLICING A same_art (MCE) BUILDING IS ALSO FORBIDDEN — NOT JUST DISCOURAGED
----------------------------------------------------------------------------
`gac_storey_slice` exists and it CAN cut a merged MCE mesh on a measured
storey/bay grid — the mesh itself is perfectly cuttable geometry. What it
cannot do is recover which triangles are the window reveal, which are the
joist behind a removed façade module, which are the balcony rail: slicing
recovers separability by POSITION, not by IDENTITY. Every `burn_building`
recipe is written against named, addressable parts, so a sliced MCE building
degrades those recipes to "blacken a rectangle" — the rectangular char
artefact the user explicitly rejected (user, 2026-08-29: "why are we
splitting up the moderncity buildings if we have versions of them that look
good? Just use those.").

So `route()` (below) treats `slice` as available ONLY for packs that were
never the kit's own art in the first place (GAC, the AEC brownstones,
downtowncity — packs with their own real, if unnamed, parts). For `same_art`
it is `kit` or nothing: when no kit style is within `MAX_H_RATIO` /
`MAX_AREA_RATIO`, the building is REFUSED (`('skip', reason)`), never handed
to the slicer as a fallback. Refusing to damage a building is better than
damaging it wrong, and that refusal always carries a reason — see `route`'s
and `plan`'s docstrings.
"""

import math
import os
import re
import sys

# Packs for which a swap is defensible, most to least.
#   same_art  the whole asset is an export of the kit's own source art
#   sized     no shared art; the swap is justified only by footprint/height
#             and should be opted into deliberately
SAME_ART = ("ModernCityEnvironment/",)

# PACKS THAT MUST NOT APPEAR IN A FIRE SCENE AT ALL (user, 2026-08-29: "since
# muyang is the only unusable don't use it for fire").
#
# Muyang DownTown is the one pack with no route to real damage. MEASURED:
# `tools/pack_structure_probe.py` — one merged mesh, 5 subsets, 756-4,643
# points for a 45-131 m building; `tools/openings_probe.py` — NO glass subset,
# so its windows are painted into the texture. That combination defeats every
# option at once: nothing to take apart (no pieces), nothing to bind per
# storey (subsets span the height), no openings to empty, and too few points
# to slice into a storey/bay grid the way GAC can be. It is a photograph on a
# massing shell, and the only thing it can carry is a flat tint — which is the
# grey-box failure this whole line of work exists to remove.
#
# So it is excluded rather than damaged badly. `unburnable()` is the gate; a
# fire preset should also drop it from the building pools so the layout never
# places one in the first place.
UNBURNABLE = ("Muyang/DownTown/",)


def unburnable(usd):
    """True when this asset cannot carry credible fire damage — see UNBURNABLE."""
    return any(k in str(usd) for k in UNBURNABLE)


# How far a kit style may be from the building it replaces before the swap is
# refused and the building is left alone. A fire that turns a 90 m block into a
# 25 m one is a worse artefact than a fire that does not gut it.
MAX_H_RATIO = 1.6          # taller/shorter by more than this -> refuse
MAX_AREA_RATIO = 2.6       # plan area ratio
_cache = {}

# `styles()` needs `detail.urban_building` and `disaster.quake_flow` as
# TOP-LEVEL packages rooted at `scene_gen` — the convention every lazy import
# elsewhere in this file (and in `quake.py`, `urban_fire.py`, ...) already
# assumes. That holds when Kit or a launch script has put `scene_gen` on
# `sys.path`, but NOT when this file is run directly as a script
# (`python3 disaster/kit_substitute.py`, the way `check()` is verified):
# there, `sys.path[0]` is `.../scene_gen/disaster`, and `from detail import
# urban_building` fails with "No module named 'detail'" — MEASURED directly:
# `cd scene_gen && python3 -c "import sys; print(sys.path[0])"` run as
# `disaster/<name>.py` prints the `disaster/` dir, not `scene_gen/`. So the
# lazy import adds `scene_gen` itself first, unconditionally safe (a no-op
# once it is already there) and cheap (no import happens until a style
# table is actually needed).
_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _ensure_scene_gen_on_path():
    if _SCENE_GEN not in sys.path:
        sys.path.insert(0, _SCENE_GEN)


def styles():
    """`{style: {W, D, H, type, family}}` for every style `build_kit` can
    actually construct — `urban_building.STYLES`, measured with the SAME
    arithmetic `build_building` and `quake_flow._mass_specs` use — NOT the
    frozen DG0 archetype bake an earlier version of this function read.

    WHY THE LIVE TABLE AND NOT THE BAKE (2026-08-29, corrected after review):
    `build_kit()` calls `urban_building.build_building(style, ...)`, which
    assembles a style from its façade pieces AT RUNTIME — it never opens an
    archetype USD. The DG0 bake is a frozen intact archetype used elsewhere
    (the earthquake live-lean rebuild seeds off the bake's own `ARCH_SEED`,
    `bake.py`'s damage-grade ladder); for THIS matcher it was only ever a
    lookup table of dimensions, and MEASURED it is a stale, incomplete one:
    only 16 of `STYLES`'s 30 styles are baked, and the missing 14 are
    precisely the TALL and the LARGE (`skyscraper_a` 25x25x103,
    `block_office` 70x50x91, `block_residential` 88x56x67, down to `church`
    16.5x27.5x8.2 — the baked table's tallest style topped out at 43 m, its
    largest footprint at 42x30). Every block-scale merged MCE asset was
    refused for want of a candidate that was simply never in the table —
    that was the bug this rewrite fixes, not a bake that needs extending.

    HEIGHT MUST INCLUDE WINGS AND TOWERS. Summing `spec["bands"]["h"]`
    directly is wrong for anything with a `tower` or `wings` entry: it gives
    `block_residential` — a podium with three 16-20-storey slabs standing on
    it — a height of 8.0 m (the podium alone), against a real top of 67 m.
    `quake_flow._mass_specs` already gets this right (it has to, to place
    fire/quake recipes at the correct z) and recurses into every wing, so
    `max(m['top'] for m in _mass_specs(style, 0, 0, 0))` is the true height.

    `type` (`urm` / `rc` / `rc_glass`, used for `best_style`'s `prefer_type`)
    is NOT stored on `STYLES` entries directly — probed empirically, every
    entry lacks a `type` key. Each entry DOES carry `family` ("01"-"05",
    "dw_b", "civ", "church" — the kit family it was built from), and
    `quake_flow.FAMILY_TYPE` is the SAME table the bake's own `type` field
    was populated from (`quake_flow.describe`: `FAMILY_TYPE.get(spec.get(
    "family", "01"), "urm")`). So `family` -> `FAMILY_TYPE` recovers the
    classification exactly, rather than defaulting it and silently biasing
    every `prefer_type` match.
    """
    if "live" in _cache:
        return _cache["live"]
    _ensure_scene_gen_on_path()
    from detail import urban_building as ub
    from disaster import quake_flow as qf

    out = {}
    for name, spec in ub.STYLES.items():
        W, D = ub.footprint(spec)
        top = max(m["top"] for m in qf._mass_specs(name, 0.0, 0.0, 0.0))
        out[name] = {"W": float(W), "D": float(D), "H": float(top),
                     "type": qf.FAMILY_TYPE.get(spec.get("family", "01"), "urm"),
                     "family": spec.get("family")}
    _cache["live"] = out
    return out


def pack_of(usd):
    u = str(usd)
    if "/bld_" in u:
        return "kit"
    for key in SAME_ART:
        if key in u:
            return "same_art"
    return "other"


# Kit builds are exported `bld_<style>_<level>.usd` — DG0-DG5 damage grades
# plus the OV (overview) and SETTLE/TILT bake passes (`disaster/bake.py`).
# MEASURED against the live manifest (2026-08-29): every one of its 144
# entries fits this pattern. Style names themselves are lowercase words
# joined by underscores (see `urban_building.STYLES`), so greedily matching
# everything up to the LAST recognised level suffix is unambiguous.
_LEVEL_SUFFIX = re.compile(r"^bld_(.+)_(?:DG\d+|OV|SETTLE|TILT)$")


def _kit_style_of(usd):
    """Best-effort style name from an already-kit usd path, or None.

    Used by `route()` for the `pack_of(usd) == 'kit'` case: that building
    needs no substitution, but the caller may still want to know WHICH style
    it already is (e.g. to reuse the same fire recipe table). A usd outside
    the bake's own naming convention (a future export, a hand-placed asset)
    returns None rather than guessing — `route`'s docstring says the caller
    keeps whatever style it already has in that case.
    """
    base = os.path.basename(str(usd))
    stem = base[:-4] if base.lower().endswith(".usd") else base
    m = _LEVEL_SUFFIX.match(stem)
    return m.group(1) if m else None


def best_style(W, D, H, table=None, exclude=(), prefer_type=None):
    """The kit style that best stands in for a W x D x H building.

    HEIGHT DOMINATES, and it is matched in LOG space. A fire scene reads
    first as a skyline: a building that comes back two storeys shorter is
    noticed immediately, while one whose plan is a few metres off is not.
    Footprint enters as an area ratio and an aspect ratio, both also in log
    space so that "half as big" and "twice as big" cost the same.

    Returns `(style, score)` with score 0 = exact, or `(None, None)` when
    nothing is within `MAX_H_RATIO` / `MAX_AREA_RATIO` — a refusal, not a
    nearest-anything fallback.
    """
    table = table or styles()
    W, D, H = float(W), float(D), float(H)
    area = max(1e-6, W * D)
    aspect = max(W, D) / max(1e-6, min(W, D))
    best, best_s = None, None
    for name, s in table.items():
        if name in exclude:
            continue
        hr = max(s["H"], H) / max(1e-6, min(s["H"], H))
        ar = max(s["W"] * s["D"], area) / max(1e-6, min(s["W"] * s["D"], area))
        if hr > MAX_H_RATIO or ar > MAX_AREA_RATIO:
            continue
        s_aspect = max(s["W"], s["D"]) / max(1e-6, min(s["W"], s["D"]))
        score = (3.0 * abs(math.log(hr))
                 + 1.0 * abs(math.log(ar))
                 + 0.6 * abs(math.log(max(s_aspect, aspect)
                                      / max(1e-6, min(s_aspect, aspect)))))
        if prefer_type and s["type"] != prefer_type:
            score += 0.25
        if best_s is None or score < best_s:
            best, best_s = name, score
    return best, best_s


def route(usd, W=None, D=None, H=None, btype=None, table=None):
    """('kit', style) | ('slice', None) | ('skip', reason) for one asset.

    The single decision point a launcher or the city builder calls PER
    BUILDING to decide what to do with it, so the pack-gating / best-fit /
    refuse-vs-fallback logic lives in exactly one place instead of being
    re-derived at every call site. In order:

      unburnable(usd)             -> ('skip', reason)            — Muyang
                                      DownTown; see UNBURNABLE.
      pack_of(usd) == 'kit'       -> ('kit', style-or-None)       — already a
                                      kit build, no substitution needed; the
                                      style is derived from the bake's own
                                      naming convention when possible
                                      (`_kit_style_of`), else None — the
                                      caller keeps whatever style it already
                                      has.
      pack_of(usd) == 'same_art'  -> ('kit', style) when `best_style` finds
                                      one within MAX_H_RATIO / MAX_AREA_RATIO
                                      of (W, D, H); otherwise ('skip', reason)
                                      — see the module docstring
                                      ("SLICING A same_art BUILDING IS ALSO
                                      FORBIDDEN"): a same_art building that
                                      finds no kit match is REFUSED, never
                                      handed to the slicer.
      anything else               -> ('slice', None)              — GAC, the
                                      AEC brownstones, downtowncity: packs
                                      with their own real (if unnamed) parts,
                                      for `gac_storey_slice` to cut on a
                                      measured grid.

    Every refusal carries a human-readable reason. A silent refusal here is
    indistinguishable from a building the fire never reached — the same
    point `plan`'s docstring makes for the batch case; `plan` is now a thin
    wrapper over this function.
    """
    if unburnable(usd):
        return "skip", "unburnable: {0}".format(usd)
    pk = pack_of(usd)
    if pk == "kit":
        return "kit", _kit_style_of(usd)
    if pk == "same_art":
        if W is None or D is None or H is None:
            return ("skip", "same_art asset with no W/D/H given to match "
                    "against a kit style ({0})".format(usd))
        st, _score = best_style(W, D, H, table, prefer_type=btype)
        if st is None:
            return ("skip",
                    "same_art asset {0:.0f} x {1:.0f} x {2:.0f} m has no kit "
                    "style within {3:.1f}x height / {4:.1f}x area — refusing "
                    "rather than slicing the merged mesh (slicing loses "
                    "IDENTITY, not geometry — see module docstring) for "
                    "{5}".format(W, D, H, MAX_H_RATIO, MAX_AREA_RATIO, usd))
        return "kit", st
    return "slice", None


def plan(buildings, involved, table=None, packs=("same_art",), verbose=True):
    """Decide the swap for every building the fire actually reached.

    `buildings` is a list of dicts carrying at least `usd`, `W`, `D`, `H` and
    an index; `involved` is the set of indices whose level is above F0.
    Returns `{index: style}` for the ones that will be rebuilt from the kit,
    and reports the ones it refused and why — a silent refusal here looks
    exactly like a building the fire missed.

    A thin wrapper over `route()`: for each involved, burnable building whose
    pack is in `packs` (default just `same_art`, the only pack `route` will
    ever return a kit match for), `route` decides kit-or-refuse. A building
    routed to `slice` counts as refused HERE — `plan` only ever produces kit
    swaps, so ask `route()` directly when the right answer for a building is
    to slice it instead.
    """
    table = table or styles()
    out, refused, off_pack = {}, [], 0
    for b in buildings:
        i = b.get("i", b.get("idx"))
        if i not in involved:
            continue
        if unburnable(b["usd"]):
            continue
        pk = pack_of(b["usd"])
        if pk == "kit":
            continue                    # already a kit building
        if pk not in packs:
            off_pack += 1
            continue
        action, val = route(b["usd"], b["W"], b["D"], b["H"], btype=b.get("btype"),
                            table=table)
        if action == "kit" and val is not None:
            out[i] = val
        else:
            refused.append((b.get("style", "?"), b["W"], b["D"], b["H"]))
    if verbose:
        print("[kit_sub] {0} building(s) swapped to kit, {1} refused "
              "(no style within {2:.1f}x height / {3:.1f}x area), {4} outside "
              "the enabled pack(s) {5}".format(
                  len(out), len(refused), MAX_H_RATIO, MAX_AREA_RATIO,
                  off_pack, ",".join(packs)))
        for nm, W, D, H in refused[:6]:
            print("[kit_sub]   refused {0:<26} {1:.0f} x {2:.0f} x {3:.0f} m"
                  .format(nm, W, D, H))
    return out


def build_kit(stage, cell, style, seed=0, ssf=1.0, hide=None):
    """Assemble kit `style` under `cell`; return its placement list.

    The build helper so a launcher or the city builder does not each
    reimplement the three-call sequence (`quake._d_live_lean` already does
    this inline for the earthquake path — this is the same recipe pulled out
    for `route`'s `'kit'` outcome). `pxr`, `detail.urban_building` and
    `scene_generator` are imported LAZILY, INSIDE this function, on purpose:
    this module is imported HOST-SIDE by `check()` (`python3
    disaster/kit_substitute.py`, no Kit running), where none of those three
    are importable — `scene_generator` pulls in `pxr` at module scope, so a
    top-level import here would turn every host-side check into an
    ImportError instead of a pass/fail report.

    `hide` is an optional prim path — the merged original being replaced —
    made invisible once the kit building is in place, so the intact mesh
    never double-renders under the burned one.

    Everything this authors lands under `cell` via
    `scene_generator.apply_placements(stage, placements, cell, ssf)` — that
    Xform is the "link prim" a user drags to move (or later query/delete) the
    whole rebuilt building as one unit, the same way `_d_live_lean` groups a
    rebuilt building under its own scope.
    """
    import random

    from detail import urban_building as ub
    import scene_generator as sg
    from pxr import UsdGeom

    pls = ub.build_building(style, 0.0, 0.0, 0.0, random.Random(seed))
    sg.apply_placements(stage, pls, cell, ssf)
    ub.apply_glass_tint(stage, pls)
    if hide:
        prim = stage.GetPrimAtPath(hide)
        if prim and prim.IsValid():
            UsdGeom.Imageable(prim).MakeInvisible()
    return pls


def check(verbose=True):
    """Host-side: the table loads and the matcher behaves."""
    bad = []
    try:
        t = styles()
    except Exception as exc:
        print("[kit_sub] check FAILED: {0}".format(exc))
        return ["styles(): {0}".format(exc)]
    # every urban_building.STYLES entry must make it into the live table —
    # styles() filters nothing, so a mismatch means _mass_specs or footprint
    # silently misbehaved for some style rather than raising
    try:
        from detail import urban_building as ub
        if len(t) != len(ub.STYLES):
            bad.append("styles() returned {0} of urban_building.STYLES's {1} "
                       "entries".format(len(t), len(ub.STYLES)))
    except Exception as exc:
        bad.append("could not cross-check styles() coverage against "
                   "urban_building.STYLES: {0}".format(exc))
    # an exact self-match must return that same style at score ~0
    for name, s in list(t.items())[:4]:
        got, score = best_style(s["W"], s["D"], s["H"], t)
        if got != name or (score or 1) > 1e-6:
            # another style may coincide exactly; accept an equal-size twin
            if got is None or abs(t[got]["H"] - s["H"]) > 1e-6:
                bad.append("{0} does not match itself (got {1})".format(name, got))
    # a 300 m tower has no kit equivalent EVEN in the corrected table (whose
    # tallest style, skyscraper_a, is 103 m: 302 / 103 = 2.93, still outside
    # MAX_H_RATIO) and must be REFUSED, not approximated. Asserted here
    # directly against best_style, and again below through route() end to
    # end, because this exact assertion is the only thing standing between a
    # 300 m building and it silently coming back 100 m tall.
    got, _ = best_style(60.0, 140.0, 302.0, t)
    if got is not None:
        bad.append("a 302 m tower matched {0} — the height gate is not "
                   "binding".format(got))
    # pack gating
    if pack_of("x/ModernCityEnvironment/Collected_Building01/a.usd") != "same_art":
        bad.append("MCE not recognised as same_art")
    if pack_of("x/bld_office_DG0.usd") != "kit":
        bad.append("a kit bake not recognised as kit")
    if pack_of("x/GreatAmericanCity/SM_Building_01.usd") != "other":
        bad.append("GAC should be 'other' until opted in")
    if not unburnable("a/Muyang/DownTown/Assets/BG_Building_A.usd"):
        bad.append("Muyang DownTown is not gated out of fire")
    if unburnable("a/GreatAmericanCity/SM_Building_01.usd"):
        bad.append("GAC wrongly gated out of fire")
    # only involved buildings are touched
    bl = [{"i": 0, "usd": "a/ModernCityEnvironment/b.usd", "W": 28.5, "D": 18.5, "H": 29.0},
          {"i": 1, "usd": "a/ModernCityEnvironment/c.usd", "W": 28.5, "D": 18.5, "H": 29.0}]
    got = plan(bl, involved={0}, table=t, verbose=False)
    if set(got) != {0}:
        bad.append("plan() touched a building the fire never reached: {0}".format(got))
    # route(): the single per-building decision point
    #   1-3. the three REAL measured MCE merged-asset dimensions
    #        (`config/asset_sets/urban.yaml`; see the module docstring) must
    #        each route to a NAMED kit style within the gates. Asserted by
    #        exact name, not just "matched something", so a future change to
    #        the table or the score weights cannot silently regress any of
    #        them back to a refusal.
    for usd, W, D, H, want in (
        ("a/ModernCityEnvironment/Collected_Building01/"
         "SM_MERGED_BP_MBuilding01.usd", 28.5, 18.5, 29.0, "commercial_mid"),
        ("a/ModernCityEnvironment/Collected_Building05/"
         "SM_MERGED_BP_MBuilding05.usd", 27.7, 20.6, 62.4, "highrise_step"),
        ("a/ModernCityEnvironment/Collected_Building02/"
         "SM_MERGED_BP_MBuilding02.usd", 91.1, 96.1, 68.7, "block_residential"),
    ):
        act, val = route(usd, W, D, H, table=t)
        if act != "kit" or val != want:
            bad.append("{0} did not route to '{1}': got {2}".format(
                usd, want, (act, val)))
    #   4. Muyang DownTown (unburnable) routes to skip, with a reason
    act, val = route("a/Muyang/DownTown/Assets/BG_Building_A.usd",
                     45.0, 45.0, 90.0, table=t)
    if act != "skip" or not val:
        bad.append("Muyang DownTown did not route to a reasoned skip: "
                   "{0}".format((act, val)))
    #   5. GAC (real, unnamed parts) routes to slice
    act, val = route("a/GreatAmericanCity/SM_Building_01.usd",
                     20.0, 15.0, 25.0, table=t)
    if act != "slice" or val is not None:
        bad.append("a GAC building did not route to slice: {0}".format((act, val)))
    #   6. a 302 m MCE tower routes to a reasoned skip, NEVER to slice, EVEN
    #      against the corrected (taller) table — slicing would substitute
    #      geometric separability for the identity slicing cannot recover
    #      (see the module docstring)
    act, val = route("a/ModernCityEnvironment/Collected_Building02/"
                     "SM_MERGED_BP_MBuilding02.usd", 60.0, 140.0, 302.0, table=t)
    if act != "skip" or not val:
        bad.append("a 302 m MCE tower did not route to a reasoned skip "
                   "(same_art must never fall back to slicing): "
                   "{0}".format((act, val)))
    #   7. an asset that is ALREADY a kit build is not re-substituted: it must
    #      route to 'kit' from its own filename, WITHOUT best_style ever
    #      running (proved by giving it dimensions no style could match)
    act, val = route("x/bld_office_DG0.usd", 999.0, 999.0, 999.0, table=t)
    if act != "kit" or val != "office":
        bad.append("an already-kit asset was re-substituted instead of kept: "
                   "{0}".format((act, val)))
    if verbose:
        print("[kit_sub] check {0}".format("ok" if not bad else "FAILED"))
        for b in bad:
            print("  " + b)
    return bad


if __name__ == "__main__":
    raise SystemExit(1 if check() else 0)
