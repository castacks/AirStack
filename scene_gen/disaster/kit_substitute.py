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

It is NOT a 1:1 twin. The merged assets are their own assemblies (MBuilding02
is 91 x 96 x 69 m; no single kit style is), so the match is by SIZE — see
`best_style`. That is also why this module is written generally rather than as
a lookup table: the same mechanism substitutes any pack, and the only thing
that changes per pack is how defensible the swap is.

WHAT THIS DELIBERATELY DOES NOT DO
-----------------------------------
It does not swap a building the fire never reached. An intact whole-asset
building looks better than an intact kit one — that is why the city draws from
these packs at all — so the swap happens ONLY where the spread solve says the
building burns, and the city keeps its own stock everywhere else.
"""

import json
import math
import os

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
# WHERE THE BAKE LIVES, SEARCHED RATHER THAN HARDCODED. The quake bake has
# been renamed once already mid-session (`archetypes_quake` -> `archetype`,
# commit 6734c6ca), which silently broke every path that named it — including
# the pool entries, which had to be rewritten to Nucleus in the same commit.
# A list that is tried in order costs nothing and turns the next rename into a
# no-op instead of a FileNotFoundError thirty seconds into a launch.
_ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets")
_ARCH_DIRS = ("archetype", "archetypes_quake", "archetype_r1",
              "archetypes_quake_r1")


def _manifest():
    for d in _ARCH_DIRS:
        p = os.path.normpath(os.path.join(_ASSETS, d, "archetypes.json"))
        if os.path.exists(p):
            return p
    raise FileNotFoundError(
        "no archetype manifest under {0} (tried {1})".format(
            os.path.normpath(_ASSETS), ", ".join(_ARCH_DIRS)))

# How far a kit style may be from the building it replaces before the swap is
# refused and the building is left alone. A fire that turns a 90 m block into a
# 25 m one is a worse artefact than a fire that does not gut it.
MAX_H_RATIO = 1.6          # taller/shorter by more than this -> refuse
MAX_AREA_RATIO = 2.6       # plan area ratio
_cache = {}


def styles(path=None):
    """`{style: {W, D, H, type, family, usd}}` for every baked DG0 archetype.

    The DG0 bake is the INTACT kit building. `burn_building` does not consume
    it — it rebuilds the style from pieces — so this table is used purely to
    choose WHICH style to stand in for the asset being replaced.
    """
    p = os.path.normpath(path or _manifest())
    if p in _cache:
        return _cache[p]
    out = {}
    with open(p) as fh:
        for e in json.load(fh):
            if e.get("level") != "DG0":
                continue
            out[e["style"]] = {"W": float(e["W"]), "D": float(e["D"]),
                               "H": float(e["H"]), "type": e.get("type", "rc"),
                               "family": e.get("family"), "usd": e.get("usd")}
    _cache[p] = out
    return out


def pack_of(usd):
    u = str(usd)
    if "/bld_" in u:
        return "kit"
    for key in SAME_ART:
        if key in u:
            return "same_art"
    return "other"


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


def plan(buildings, involved, table=None, packs=("same_art",), verbose=True):
    """Decide the swap for every building the fire actually reached.

    `buildings` is a list of dicts carrying at least `usd`, `W`, `D`, `H` and
    an index; `involved` is the set of indices whose level is above F0.
    Returns `{index: style}` for the ones that will be rebuilt from the kit,
    and reports the ones it refused and why — a silent refusal here looks
    exactly like a building the fire missed.
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
        st, score = best_style(b["W"], b["D"], b["H"], table,
                               prefer_type=b.get("btype"))
        if st is None:
            refused.append((b.get("style", "?"), b["W"], b["D"], b["H"]))
            continue
        out[i] = st
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


def check(verbose=True):
    """Host-side: the table loads and the matcher behaves."""
    bad = []
    try:
        t = styles()
    except Exception as exc:
        print("[kit_sub] check FAILED: {0}".format(exc))
        return ["styles(): {0}".format(exc)]
    if len(t) < 8:
        bad.append("only {0} DG0 style(s) in the bake".format(len(t)))
    # an exact self-match must return that same style at score ~0
    for name, s in list(t.items())[:4]:
        got, score = best_style(s["W"], s["D"], s["H"], t)
        if got != name or (score or 1) > 1e-6:
            # another style may coincide exactly; accept an equal-size twin
            if got is None or abs(t[got]["H"] - s["H"]) > 1e-6:
                bad.append("{0} does not match itself (got {1})".format(name, got))
    # a 300 m tower has no kit equivalent and must be REFUSED, not approximated
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
    if verbose:
        print("[kit_sub] check {0}".format("ok" if not bad else "FAILED"))
        for b in bad:
            print("  " + b)
    return bad


if __name__ == "__main__":
    raise SystemExit(1 if check() else 0)
