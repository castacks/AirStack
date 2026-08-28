"""tornado_people.py — the people the tornado HIT, and how much of each one
a camera looking down can actually see.

WHAT THIS MODULE IS FOR, restated 2026-08-27 after the first 100 m render was
reviewed and rejected. It is not a population model of a struck neighbourhood.
It is a TARGET GENERATOR for an aerial search benchmark, and the two are not
the same thing — the first render was arguably the better sociology and it was
useless as a dataset:

    67 figures, of which 55 were STANDING on the wreckage or walking on the
    road: neighbours digging, survivors picking through their own house,
    walking wounded on the carriageway. Twelve were casualties. From 40 m the
    plate read as a building site.

Two things were wrong with that and both were decisions, not bugs.

  1. MOST OF THE FIGURES HAD NOT BEEN HIT BY ANYTHING. A digger, a bystander
     and a walking-wounded survivor are all real at T+45 min and none of them
     is what this dataset is scored on. Out of scope, by request: the scene is
     about the people the tornado hit, so the uninvolved are gone — see WHAT IS
     DELIBERATELY ABSENT.
  2. A STANDING FIGURE ON A DEBRIS PILE IS THE EASY PROBLEM ANYWAY. It is a
     vertical object on a horizontal one, high-contrast from any altitude and
     any azimuth. What is hard, and what nobody has a labelled set of, is a
     HORIZONTAL body in a horizontal field of pale boards, with an unknown
     part of it under a sheet of plywood.

So every figure this module places is now a casualty lying in or on the debris,
and the module's whole job is to control TWO axes explicitly, because those two
axes are the benchmark:

    ATTITUDE   face-up, face-down, on the left side, on the right side, drawn
               up, or pinned sitting. Recorded as `attitude` on every record.
    OCCLUSION  nothing on the body at all (`full`), or one of a dozen named
               patterns that cover a NAMED STRETCH of it and leave the rest
               proud — legs under a sheet with the head and chest out; head and
               chest under, legs out; a band across the midriff; one flank.
               Recorded as `occlusion`, `covered_frac` and `visible_parts`.

NOTHING IS FULLY BURIED. `max_covered_frac` is a hard ceiling and the piece
generator solves each covering board against it. A target a camera cannot see
is not a hard example, it is a WRONG LABEL — `disaster.people` retired its
`exposed_interior` scenario over exactly this and the reasoning is not
relitigated here. Sinking a figure out of sight and calling the record
`partial` would poison the set it exists to build.

WHERE THE BODIES GO
-------------------
Location is still a research question and the answer has not changed:

  * **Almost everybody survives.** Joplin's catastrophic zone — total
    structural destruction — held 4,716 people and killed 122: **97.4%
    survived** (NIST NCSTAR 3 geolocated all 161 fatalities; Paul & Stimers
    2014 supplied the population denominators). Whole path: 13,547 people, 161
    dead, ~1,371 injured. Of the injured **89% are minor** and 86% go home
    (Niederkrotenthaler et al. 2013, 1,398 patients across 39 hospitals).
    So these figures are OVER-REPRESENTED against the world by design, and it
    is a dataset decision recorded as one — a true rate over ~39 damaged
    houses would put one or two casualties in a 500 m corridor and there would
    be nothing to score.
  * **They are where the houses were.** Ten to fifteen minutes of warning
    means nobody evacuates: people shelter where they already are, on the
    ground floor, interior (bathroom 39%, closet 37%, hallway 10% — Hammer &
    Schmidlin 2002, 190 occupants of 65 F4/F5-damaged homes). So the victim
    count is driven PER WRECKED HOUSE, not by a global head count over the
    plate: `per_wreck`, weighted by damage level.
  * **A third of the dead are recovered off their own footprint.** CDC MMWR
    61(28) recorded injury location and recovery location separately for all
    338 April-2011 deaths: **90.5% injured indoors, 3.3% outdoors, 37.0%
    RECOVERED outdoors.** Most of that is "went out with the wall that failed
    and landed in the yard" — the MMWR records no distance — which is why
    `where` puts a large share in the debris skirt and the yard and only a
    little of it far downtrack.
  * **They cluster.** Chiu et al. 2013 (247 Alabama decedents) recorded who
    each victim was found with: only **7.7% were alone**. `cluster_chance`
    puts a second body within a few metres of the first.
  * **Long-range lofting is record-book territory, not a category.** The
    documented survivals are 398 m (Matt Suter, an F2, GPS-measured) and 76 m
    (a mattress at Dawson Springs). There is no published distribution of
    human throw distances and no published azimuthal distribution for victim
    deposition — the 78%-left statistic everyone quotes is Snow et al. 1995 on
    LIGHTWEIGHT debris lofted into the parent storm (cheques and photographs),
    and near-surface flow is CONVERGENT toward the centreline anyway (Karstens
    et al. 2013 digitised 10,300 tree falls at Joplin and 94,500 at
    Tuscaloosa). `trail` is therefore short and its spread is wide.

THE EPOCH IS STILL T+30-60 MIN — no responders, no heavy equipment, no search
markings, no triage tents. Joplin's professional rubble-rescue count was
SEVENTEEN against 1,371 injured, and the civilian extrication share in the
earthquake literature is 60-100% (Bartolucci et al. 2020). A scene with orange
search markings in it is depicting T+24 h and a different problem.

THE SURFACE IS MEASURED NOW, NOT GUESSED
----------------------------------------
The other half of why the first render looked wrong: figures were seated on
`DEBRIS_Z_M[level]`, one constant per damage class, in a field of 755 boards
whose actual top surface ranges from 0 to about 1.4 m over a couple of metres.
Some floated, some were buried to the shins, and several were standing on the
tilted face of a half-sheet of plywood.

`_Deck` fixes it by MEASURING: the launcher hands the planner the plank specs
it just authored (`ctx["plank_specs"]`) and the deck is the top of the highest
board over each cell of a 0.8 m grid. Every body is then

  * seated ON that surface at its own three stations (feet, waist, head), and
  * REFUSED if those three disagree by more than `max_deck_tilt_m` —

which is the single change that makes a lying figure read as lying on the
debris rather than through it. A body needs a metre and a half of something
flat, and in a plank field that is a real constraint: the refusal tally
(`deck_tilt`) is printed with the rest.

A PURE PLANNER
--------------
No stage, no Isaac imports, no USD. `plan_people` takes a context of things
somebody else built and returns placements, plank specs and ground truth, so
the whole plan can be run and asserted on the host before a container starts.
`scene_gen/tests/test_tornado_people_poses.py` does exactly that.
"""

import math
import os
import random

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

DEFAULTS = {
    # HOW MANY BODIES, PER WRECKED HOUSE. Not a global head count over the
    # plate, because a body's location is a function of where a house failed:
    # a 100 m plate with five wrecks and a 500 m corridor with forty should
    # not be given the same number and asked to spread it.
    #
    # Weighted by damage level for the obvious reason — the levelled and swept
    # houses are the ones whose occupants had a structure fail on them, and
    # NIST's Joplin finding is that a best-available interior refuge area "is
    # not expected to offer life-safety protection" once the structure goes.
    # A roof-collapsed house is survivable; a swept slab is where the deaths
    # were. Ranges are inclusive.
    "per_wreck": {
        "leveled": [2, 3],
        "swept": [1, 3],
        "partial_collapse": [1, 2],
        "roof_collapsed": [0, 2],
    },
    # THE CEILING. 40 casualties is already an order of magnitude more than
    # the epidemiology supports (see the docstring) and it is as many as a
    # 500 m corridor can carry without the plate reading as a massacre.
    "max_total": 40,

    # THROWN CLEAR, downtrack of a flattened house. Kept small and kept short:
    # the arithmetic in the docstring puts UNDER ONE person in a 500 m
    # corridor genuinely thrown any distance, and there is no published
    # distribution to draw a longer one from.
    "trail": {"count": [1, 3], "range_m": [8.0, 26.0], "spread_deg": 55.0},

    "seed": 91,

    # A LYING FIGURE IS 1.8 m OF GROUND, NOT A POINT, and two of them 1.1 m
    # apart interpenetrate along their whole length. 2.4 m is a body length
    # plus a margin, tested at three stations down each body.
    "min_separation_m": 2.4,

    # NOBODY OFF THE EDGE OF THE WORLD. `suburb_scene.apply_ground` lays its
    # sheet over exactly `region` and nothing beyond it, and a LYING figure
    # lies entirely on ONE side of its placement point — feet a metre inside
    # the boundary still puts a head half a metre outside it. Every body is
    # tested end to end. Dropped, not clamped, for the same reason
    # `planks.clip_to_region` drops: clamping piles figures along the boundary
    # in a line, which is a worse artefact than the one it fixes.
    "edge_margin_m": 1.6,

    # THEY CLUSTER. Chiu et al. 2013: only 7.7% of victims were found alone.
    # The chance that a body is placed as a PAIR with one already down, within
    # `cluster_r_m`. It is also the strongest aerial cue there is — two bodies
    # together is a scene a searcher stops at.
    "cluster_chance": 0.34,
    "cluster_r_m": [1.8, 4.0],

    # THE MOMENT THIS SCENE DEPICTS. T+30-60 min is the only window in which
    # aerial victim detection has any value at all: before it the roads are
    # impassable and nobody is looking, after it the organised grid search has
    # started and there is nobody left alive to find.
    "epoch_min": 45.0,

    # HOW FLAT THE DEBRIS HAS TO BE UNDER A BODY. The spread between the deck
    # height at the feet, the waist and the head. 0.26 m over ~1.8 m is about
    # 8 degrees — a body will drape over that and read as draped. Twice it is
    # a figure lying on the tilted face of a sheet of plywood, which is what
    # the first render showed. Needs `ctx["plank_specs"]`; without it every
    # spot passes and the deck is flat by assumption.
    "max_deck_tilt_m": 0.26,

    # ...AND A LOOSER ONE ON THE WRECKAGE ITSELF. A body on a levelled house
    # is DRAPED, not level: the pile has nothing flat on it at 1.8 m, so
    # holding `pile` and `skirt` to the lawn's 0.26 m is what empties the
    # wreckage of people — which is exactly what happened the moment
    # `planks._lay` was fixed and the board mat stopped being lumpy. 0.55 m
    # over a body length is about 17 degrees: a figure lying over a broken
    # joist, which is the target this dataset exists to produce.
    "max_deck_tilt_pile_m": 0.55,

    # HOW FAR A BODY SETTLES INTO WHAT IT IS LYING ON, as a fraction of its own
    # depth. Small on purpose: this is the debris ON TOP that does the
    # occluding, and it is measurable, whereas sink is neither controllable nor
    # visible in the record. A body sunk past its own half-depth has its
    # silhouette eaten from below and nothing in `visible_parts` says so.
    "sink_frac": [0.0, 0.32],

    # ---- THE TWO AXES THE BENCHMARK IS ABOUT ------------------------------

    # ATTITUDE. Face-up, face-down, on either side, and drawn up. ALL FOUR ARE
    # HORIZONTAL, and that is now the rule rather than an accident.
    #
    # `trapped_sit` — a figure pinned SITTING with its legs under the pile,
    # head and shoulders proud — was in this bag and was CUT ON REVIEW
    # (2026-08-27, second render). The argument for it was good: a vertical
    # trunk against a horizontal field is a different detection problem and a
    # model that has only seen lying bodies has learnt half the target. The
    # argument against it is what the render showed — a figure sitting bolt
    # upright in a levelled block reads as somebody who sat down, not as
    # somebody the building fell on, and it was the one silhouette in the set
    # that looked posed. It is the same objection that removed the standing
    # figures and it is settled the same way.
    #
    # Roughly a third face-up, a third face-down, a third on a side. There is
    # no literature on the attitude distribution of tornado casualties — there
    # is barely any on their location — so this is a DATASET decision to cover
    # the space evenly rather than an estimate of the world, and it is written
    # down as one.
    "poses": {
        "lying_supine": 0.17,        # face-up, one knee raised
        "lying_supine_open": 0.16,   # face-up, arms out, legs straight
        "lying_prone": 0.17,         # face-down, one arm sprawled
        "lying_prone_reach": 0.15,   # face-down, both arms overhead
        "lying_side_l": 0.13,        # on the left side
        "lying_side_r": 0.12,        # on the right side
        "lying_curled_l": 0.10,      # drawn up — the SHORT silhouette
    },

    # OCCLUSION. `none` is the fully-observable class the user asked for
    # first — a body lying ON the debris with nothing over it — and it is the
    # single largest entry because it is the only class a detector can be
    # trained on before it is asked to do the hard one. Everything else names
    # the stretch of body that goes UNDER and therefore the parts that stay
    # visible; see `_OCCLUSION`.
    "occlusion": {
        "none": 0.30,
        "feet_shins": 0.07,
        "legs": 0.09,
        "legs_hips": 0.07,
        "midriff": 0.07,
        "torso": 0.08,
        "torso_head": 0.07,
        "head_only": 0.05,
        "upper_body": 0.06,
        "all_but_head": 0.05,
        "all_but_feet": 0.04,
        "banded": 0.05,
        "flank": 0.06,
    },

    # THE HARD CEILING ON HOW MUCH OF A BODY MAY BE HIDDEN. Enforced while the
    # pieces are being generated, not checked afterwards: a pattern whose span
    # would exceed it is TRIMMED, so a figure is never authored that the record
    # then has to describe as invisible. See the top of the file for why this
    # is the most important number in the module.
    "max_covered_frac": 0.80,

    # WHERE, as shares. All four are debris-field locations; there is no
    # "somewhere else" any more.
    #
    #   pile     on the board mat over and around the wrecked house, which is
    #            where an occupant of a failed structure ends up. The deepest
    #            debris and the hardest read.
    #   skirt    the outer mat, 1.0-1.9 footprints out: still boards, but
    #            thinner, so the body has contrast under it.
    #   yard     open ground beside the wreck — lawn, mud, scour. The
    #            highest-contrast targets in the scene and the control case.
    #   street   the carriageway, inside the track. Joplin's search doctrine
    #            was literally "house by house, car by car, block by block",
    #            and the road is the only hard standing in a levelled block.
    "where": {"pile": 0.36, "skirt": 0.28, "yard": 0.26, "street": 0.10},

    # A SHARE OF THE CASUALTIES ARE DEAD. 86.6% of tornado deaths are
    # on-scene (Chiu et al. 2013) and 94% in May et al. 2000, so a corridor at
    # T+45 min genuinely holds both — and a benchmark that scores "find the
    # people" wants to know which is which. It changes nothing visually; it is
    # a label.
    "casualty_share": 0.25,
}

# HOW DEEP THE DEBRIS IS, by damage level, in metres. STILL AN ESTIMATE, and
# now only a FALLBACK: with `ctx["plank_specs"]` the deck is measured off the
# boards the launcher actually authored (`_Deck`), which is what this table was
# always a stand-in for. It survives because the bench and the host-side tests
# run without a plank field.
DEBRIS_Z_M = {
    "pristine": 0.0,
    "roof_stripped": 0.15,
    "roof_collapsed": 0.55,
    "partial_collapse": 0.70,
    "leveled": 0.85,
    "swept": 0.30,
}

# Levels whose house is gone enough that a body is IN the debris rather than
# beside an intact building.
_WRECKED = ("roof_collapsed", "partial_collapse", "leveled", "swept")
_FLATTENED = ("leveled", "swept")

WHERE = ("pile", "skirt", "yard", "street")


def resolve_cfg(config):
    """`DEFAULTS` with the scene config's `people:` block merged over."""
    cfg = dict(DEFAULTS)
    user = (config or {}).get("people") or {}
    for k, v in user.items():
        if k in ("poses", "occlusion", "where", "per_wreck") and isinstance(
                v, dict):
            cfg[k] = dict(cfg.get(k, {}), **v)
        elif k == "trail" and isinstance(v, dict):
            cfg[k] = dict(cfg["trail"], **v)
        else:
            cfg[k] = v
    return cfg


# ---------------------------------------------------------------------------
# WHAT THE BODY IS, IN METRES
# ---------------------------------------------------------------------------
# LANDMARKS AS FRACTIONS OF STANDING HEIGHT, from Drillis & Contini (1966) —
# the segment table Winter's *Biomechanics and Motor Control of Human Movement*
# reproduces and every gait lab still uses. Heights from the soles:
#
#     ankle 0.039 H   knee 0.285 H   hip 0.530 H   waist 0.600 H
#     shoulder 0.818 H   chin 0.870 H   crown 1.000 H
#
# and the breadths that decide how wide a covering board has to be to cover
# anything at all: biacromial (shoulder) 0.245 H, hip 0.191 H.
_LANDMARK = {"ankle": 0.039, "knee": 0.285, "hip": 0.530, "waist": 0.600,
             "shoulder": 0.818, "chin": 0.870, "crown": 1.000}

# HOW EACH ATTITUDE OCCUPIES THE GROUND.
#
#   attitude    what the ground truth calls it: `face_up`, `face_down`,
#               or `side`. THIS IS ONE OF THE TWO AXES the benchmark is
#               about, so it is a recorded field and not an implementation
#               detail — "front, back, sideways" is exactly the request.
#   reach       the figure's GROUND LENGTH as a fraction of its stature,
#               measured from the placement point (which is the SOLES: the
#               RenderPeople origin sits between the feet and `roll_deg` turns
#               the rig about it) toward the head. 1.0 for an extended body;
#               a curled one is folded and only covers ~0.62 of its stature.
#   half_w      half the body's width across that axis, as a fraction of
#               stature. Wider for the open poses because the arms are out.
#   parts       the part table `_visible_parts` reads, in units of `reach`.
_PARTS_EXTENDED = (("feet", 0.00, 0.07), ("shins", 0.07, 0.29),
                   ("thighs", 0.29, 0.50), ("hips", 0.48, 0.60),
                   ("torso", 0.58, 0.83), ("arms", 0.55, 0.88),
                   ("shoulders", 0.78, 0.88), ("head", 0.86, 1.00))
# A DRAWN-UP BODY IS NOT A SCALED-DOWN EXTENDED ONE. Knees to chest puts the
# feet back under the thighs, so the foot band is at the same end as the shins
# and the trunk occupies proportionally more of the ground length.
_PARTS_CURLED = (("feet", 0.00, 0.14), ("shins", 0.08, 0.32),
                 ("thighs", 0.24, 0.50), ("hips", 0.44, 0.60),
                 ("torso", 0.54, 0.82), ("arms", 0.48, 0.84),
                 ("shoulders", 0.74, 0.86), ("head", 0.84, 1.00))
# ARMS OVERHEAD PUSH THE HEAD DOWN THE TABLE. `lying_prone_reach` is 1.14 of
# its own stature long because the hands are out past the crown, so the top
# 12% of its ground length is HANDS — read against `_PARTS_EXTENDED` a board
# over the fingertips would be reported as covering the head. The crown sits
# at 1.00/1.14 = 0.88 of the reach and everything below it scales the same way.
_PARTS_REACH = (("feet", 0.00, 0.06), ("shins", 0.06, 0.25),
                ("thighs", 0.25, 0.44), ("hips", 0.42, 0.53),
                ("torso", 0.51, 0.73), ("arms", 0.62, 1.00),
                ("shoulders", 0.68, 0.78), ("head", 0.75, 0.88))

_ATTITUDE = {
    "lying_supine": {"attitude": "face_up", "reach": 1.00, "half_w": 0.13,
                     "parts": _PARTS_EXTENDED},
    "lying_supine_open": {"attitude": "face_up", "reach": 1.00,
                          "half_w": 0.20, "parts": _PARTS_EXTENDED},
    "lying_prone": {"attitude": "face_down", "reach": 1.00, "half_w": 0.17,
                    "parts": _PARTS_EXTENDED},
    "lying_prone_reach": {"attitude": "face_down", "reach": 1.14,
                          "half_w": 0.14, "parts": _PARTS_REACH},
    "lying_side_l": {"attitude": "side", "reach": 0.88, "half_w": 0.12,
                     "parts": _PARTS_EXTENDED},
    "lying_side_r": {"attitude": "side", "reach": 0.88, "half_w": 0.12,
                     "parts": _PARTS_EXTENDED},
    "lying_curled_l": {"attitude": "side", "reach": 0.62, "half_w": 0.15,
                       "parts": _PARTS_CURLED},
}
# `trapped_sit` IS DELIBERATELY NOT HERE. It was the one upright attitude in
# the set — trunk vertical, legs under the pile, its own board arrangement over
# the lap and a leaning piece against the chest — and it was cut on review
# because a figure sitting bolt upright in a levelled block reads as somebody
# who sat down. The pose itself survives in `scene_generator._HUMAN_POSES`
# (`disaster.people` still names it in `GROUND_POSES`); nothing in this module
# can draw it, and `_visible_parts` no longer has a seated branch.

# HOW TALL THE BODY IS AT EACH STATION, above the surface it is lying on, as
# a fraction of its own stature.
#
# THE SECOND THING THE FIRST BENCH RENDER SHOWED, and it is the one the review
# called "humans going through the debris". A covering piece was seated at
# `z_m + lift` — the top of the body's own CENTRE LINE, so 0.35 m for a
# 1.80 m figure — and that is the right height for a flat chest and the wrong
# height for everything else on a body. `lying_supine` raises a knee 0.20 m,
# `lying_prone` puts a heel 0.30 m up, `lying_curled_l` draws both knees to
# the chest: a sheet laid at chest height over any of those passes straight
# through the limb.
#
# A rigid piece resting on a body rests on the HIGHEST thing under its own
# footprint, so `_crest` takes the maximum over the stretch of body the piece
# spans, and `_cover` seats it there.
#
#   base    the flat-body height: the chest for a face-up or face-down figure,
#           the shoulder for a side-lying one.
#   bands   (t_lo, t_hi, frac) over the pose's own ground reach, for the parts
#           that stand higher than that.
#
# ESTIMATED FROM THE POSE ARITHMETIC, not measured off a render — the numbers
# come from the joint solves in `scene_generator._HUMAN_POSES` (the supine
# knee's 0.20 m, the prone heel's 0.364 m shin at 55 deg of flexion) plus the
# limb's own thickness. VERIFY ON SIGHT: a board that floats over a body is
# this table too high, and one that cuts through a knee is it too low.
_BODY_RISE = {
    "lying_supine": {"base": 0.211,
                     "bands": ((0.20, 0.42, 0.240),    # the raised knee
                               (0.55, 0.76, 0.244))},  # forearm on the chest
    "lying_supine_open": {"base": 0.211, "bands": ()},
    "lying_prone": {"base": 0.211,
                    "bands": ((0.04, 0.25, 0.235),)},  # the heel, drawn up
    "lying_prone_reach": {"base": 0.211, "bands": ()},
    "lying_side_l": {"base": 0.256,
                     "bands": ((0.26, 0.56, 0.278),    # the top knee
                               (0.54, 0.88, 0.289))},  # shoulder + top arm
    "lying_side_r": {"base": 0.256,
                     "bands": ((0.26, 0.56, 0.278),
                               (0.54, 0.88, 0.289))},
    "lying_curled_l": {"base": 0.256,
                       "bands": ((0.16, 0.58, 0.344),  # both knees, drawn up
                                 (0.56, 0.94, 0.289))},
}


def _crest(pose, t0, t1, height):
    """Metres from the body's own ground plane to its top over `[t0, t1]`."""
    spec = _BODY_RISE.get(pose) or {"base": 0.21, "bands": ()}
    f = float(spec["base"])
    for (a0, a1, h) in spec["bands"]:
        if t1 > a0 and t0 < a1:
            f = max(f, float(h))
    return f * float(height)


# ---------------------------------------------------------------------------
# OCCLUSION — the second axis, and the reason this module exists
# ---------------------------------------------------------------------------
# Each pattern names the stretch (or stretches) of the body that goes UNDER
# debris, in fractions of the figure's own ground reach, soles at 0 and crown
# at 1. Everything else stays proud. `_visible_parts` turns the covered spans
# back into the list of body parts a camera can see, which is what the record
# carries and what the request asked for in so many words: *for partially
# observable I want various different parts of the body visible.*
#
# THE SPANS ARE CHOSEN TO SPREAD OVER THE WHOLE BODY rather than to be
# realistic one at a time. Any single one of them is something a debris
# photograph shows; the DISTRIBUTION is a dataset decision, because the point
# is that a detector meets every partial silhouette rather than the one that
# happens to be commonest.
#
# `flank` is the odd one out and is handled separately: it covers one SIDE of
# the body along most of its length rather than a band across it, so it leaves
# one arm and one leg out and nothing else. It is what a sheet of sheathing
# landing edge-on beside a body actually does.
_OCCLUSION = {
    "none":         (),
    "feet_shins":   ((0.00, 0.30),),
    "legs":         ((0.00, 0.52),),
    "legs_hips":    ((0.00, 0.62),),
    "midriff":      ((0.42, 0.70),),
    "torso":        ((0.46, 0.84),),
    "torso_head":   ((0.52, 1.04),),
    "head_only":    ((0.80, 1.04),),
    "upper_body":   ((0.40, 1.04),),
    "all_but_head": ((0.00, 0.80),),
    "all_but_feet": ((0.20, 1.04),),
    "banded":       ((0.06, 0.30), (0.56, 0.86)),
    "flank":        "lateral",
}

# THE PIECES THAT DO THE COVERING, and why they are mostly BIG.
#
# `planks.STOCK` draws a field that is 34% studs by count, because that is what
# a stick-built house is made of. A 2x4 laid across a casualty covers 0.12 m of
# them and is a stick at any altitude a drone flies at. What actually buries
# somebody, and what actually reads from 40 m, is SHEET GOODS: a broken half
# sheet of OSB, a slab of roof deck with its shingles on. So the covering mix
# here is deliberately not the field mix — three quarters sheet, a quarter
# lumber, and the lumber is there to break up the edges rather than to hide
# anything.
#
#   (weight, width-along-the-body, length-across-it, thickness)
_COVER_STOCK = (
    ("sheathing", 0.52, (0.45, 1.15), (0.95, 2.30), (0.011, 0.019)),
    ("deck",      0.22, (0.55, 1.05), (0.85, 1.90), (0.050, 0.090)),
    ("board",     0.16, (0.14, 0.26), (1.10, 3.20), (0.016, 0.026)),
    ("joist",     0.10, (0.18, 0.30), (1.80, 3.60), (0.038, 0.058)),
)


def _draw_cover_stock(rng):
    r = rng.random() * sum(s[1] for s in _COVER_STOCK)
    for (k, w, along, across, th) in _COVER_STOCK:
        r -= w
        if r <= 0.0:
            return (k, rng.uniform(*along), rng.uniform(*across),
                    rng.uniform(*th))
    k, _w, along, across, th = _COVER_STOCK[-1]
    return (k, rng.uniform(*along), rng.uniform(*across), rng.uniform(*th))


def _weighted_key(bag, rng, default=None):
    """Draw a key from a `{name: weight}` bag. Weights are normalised here."""
    items = [(k, float(v)) for k, v in sorted((bag or {}).items())
             if float(v) > 0.0]
    tot = sum(v for _k, v in items)
    if not items or tot <= 0.0:
        return default
    r = rng.random() * tot
    for k, v in items:
        r -= v
        if r <= 0.0:
            return k
    return items[-1][0]


def _union(spans):
    """Total length of the union of `[(lo, hi), ...]`, clipped to [0, 1]."""
    out, cur_lo, cur_hi = 0.0, None, None
    for (a0, a1) in sorted((max(a, 0.0), min(b, 1.0)) for (a, b) in spans):
        if a1 <= a0:
            continue
        if cur_hi is None or a0 > cur_hi:
            if cur_hi is not None:
                out += cur_hi - cur_lo
            cur_lo, cur_hi = a0, a1
        else:
            cur_hi = max(cur_hi, a1)
    if cur_hi is not None:
        out += cur_hi - cur_lo
    return out


def _overlap(span, spans):
    """How much of `span` the union of `spans` covers, in the same units."""
    lo, hi = span
    if hi <= lo:
        return 0.0
    got = 0.0
    for (a0, a1) in sorted(spans):
        a0, a1 = max(a0, lo), min(a1, hi)
        if a1 > a0:
            got += a1 - a0
    # `spans` are already merged by the caller, so this is exact rather than
    # an over-count.
    return min(got, hi - lo)


def _visible_parts(pose, spans):
    """Which named body parts a camera can still see, given covered spans.

    A part counts as VISIBLE while less than half of its own extent is under
    something. Half rather than all of it, because a shoulder with a board
    across its outer third is a shoulder anyone can see and a leg under a
    sheet from the knee down is not a leg any more — and because the ground
    truth is read by a scorer, not by a renderer, so the useful answer is
    "would a human labeller draw this part", not "is a single pixel of it lit".
    """
    plan = _ATTITUDE.get(pose) or {}
    table = plan.get("parts") or _PARTS_EXTENDED
    merged = []
    for (a0, a1) in sorted(spans):
        if merged and a0 <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], a1)
        else:
            merged.append([a0, a1])
    merged = [tuple(m) for m in merged]
    out = []
    for (name, lo, hi) in table:
        if _overlap((lo, hi), merged) < 0.5 * (hi - lo):
            out.append(name)
    return out


# ---------------------------------------------------------------------------
# THE DEBRIS SURFACE, MEASURED
# ---------------------------------------------------------------------------

class _Deck(object):
    """Top-of-debris height over a coarse grid, from the authored plank field.

    THE NUMBER THAT WAS BEING GUESSED. Every figure in the first 100 m render
    was seated on `DEBRIS_Z_M[level]` — one constant per damage class — in a
    field of 755 boards whose real top surface swings from 0 to ~1.4 m over a
    couple of metres. So some figures floated, some were buried to the shins,
    and several were standing on the tilted face of a half sheet of plywood.

    `ctx["plank_specs"]` is the list the launcher passed to `planks.build`, so
    this is the same geometry that was authored, not a model of it: each
    board's eight corners come from `planks._box` and its top z is stamped into
    every cell of its own axis-aligned footprint.

    CELL SIZE IS A TRADE and 0.8 m is the far side of it. A body is ~1.8 m
    long, so three sample stations at 0.8 m resolution is enough to catch the
    tilt that matters and coarse enough that a 3 m joist costs a handful of
    cells rather than a hundred. The axis-aligned footprint over-covers a
    diagonal board, which makes the deck slightly HIGHER and the tilt test
    slightly stricter than the truth — the conservative direction.
    """

    def __init__(self, specs, cell_m=0.8, points=()):
        from . import planks
        self.cell = float(cell_m)
        self.g = {}
        self.n = 0
        # THE ARCHETYPE PILE, WHICH THE PLANK SPECS DO NOT CONTAIN.
        #
        # `_Deck` was built from the board field alone, and on a levelled lot
        # the boards are the THIN part of the debris — the deep part is the
        # baked wreck USD, which this module cannot see because it is a
        # referenced instance and this planner never touches a stage. The
        # consequence showed up the moment `planks._lay` was fixed to lay
        # boards flat: the measured deck dropped to a couple of centimetres
        # everywhere, `_DECK_BAND["pile"]`'s floor stopped being cleared, and
        # the casualties were pushed out of the wreckage onto the skirt and
        # the street. Measured across the two builds, `where` went
        # pile=8 skirt=5 to pile=3 skirt=7 — "I no longer see people inside
        # the house debris, they seem to only be surrounding it".
        #
        # `points` is `[(x, y, top_z)]` sampled off the authored archetype by
        # the launcher, which CAN walk it. Stamped into the same grid, so a
        # body on the pile is seated on the pile.
        for q in points or ():
            k = (int(math.floor(float(q[0]) / self.cell)),
                 int(math.floor(float(q[1]) / self.cell)))
            if self.g.get(k, -1e9) < float(q[2]):
                self.g[k] = float(q[2])
                self.n += 1
        for sp in specs or ():
            try:
                pts, _n = planks._box(sp)
            except Exception:                  # a malformed spec is data
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            top = max(p[2] for p in pts)
            i0 = int(math.floor(min(xs) / self.cell))
            i1 = int(math.floor(max(xs) / self.cell))
            j0 = int(math.floor(min(ys) / self.cell))
            j1 = int(math.floor(max(ys) / self.cell))
            if (i1 - i0 + 1) * (j1 - j0 + 1) > 400:
                continue                       # a board the size of a house
            self.n += 1
            for i in range(i0, i1 + 1):
                for j in range(j0, j1 + 1):
                    k = (i, j)
                    if self.g.get(k, -1e9) < top:
                        self.g[k] = top

    def at(self, x, y):
        """Height of the debris surface at (x, y). Bare ground reads 0.0."""
        return self.g.get((int(math.floor(x / self.cell)),
                           int(math.floor(y / self.cell))), 0.0)

    def profile(self, x, y, ux, uy, reach):
        """Deck height under the feet, the waist and the head of a body."""
        return [self.at(x + ux * t * reach, y + uy * t * reach)
                for t in (0.08, 0.5, 0.92)]


class _FlatDeck(object):
    """The no-plank-field fallback: `DEBRIS_Z_M`, flat, tilt test disabled.

    The bench and the host-side tests run with no authored boards at all, and
    a planner that refused every spot in that case would be untestable off a
    GPU. `level_z` is what the caller thinks the pile height is.
    """

    def __init__(self, level_z=0.0):
        self.z = float(level_z)
        self.n = 0

    def at(self, x, y):
        return self.z

    def profile(self, x, y, ux, uy, reach):
        return [self.z, self.z, self.z]


# ---------------------------------------------------------------------------
# Placement helpers
# ---------------------------------------------------------------------------

def _human_placement(ctx, usd, x, y, z_ground, yaw, pose, prone=False):
    """Delegate to `disaster.people`, which owns the measured pose geometry.

    NOT REIMPLEMENTED HERE ON PURPOSE. That function carries per-character
    stature scaling, the ground-contact solve for the seated poses, the
    RenderPeople yaw/roll corrections, the lay-down roll, the long-axis SPIN
    that puts a figure on its side and the lift that goes with each — all of it
    derived by measurement against the rigs, and all of it equally true of a
    tornado casualty. Duplicating it would mean maintaining two copies of
    arithmetic that took a while to get right.
    """
    from . import people
    return people._human_placement(ctx, usd, x, y, z_ground, yaw, pose,
                                   prone=prone)


def _measure(ctx, usd):
    """`(standing_height_m, body_depth_m)` for one character.

    The same resolver call `_human_placement` makes, so the numbers the debris
    is laid against are the numbers the figure was placed with. `sy` is the
    front-to-back depth of the A-posed bbox — the body's THICKNESS once it is
    face-up or face-down, and what decides how far above the deck a board over
    the chest has to sit.
    """
    ap = ctx["asset_pools"]
    fp = ctx["resolver"].get(usd, "human", scale=ap.scale_of(usd),
                             axis_up=ap.axis_of(usd))
    return (float(fp.get("sz", 1.8)) or 1.8, float(fp.get("sy", 0.35)) or 0.35)


def _LYING_POSES():
    """The lying-pose names, from the one table that defines them."""
    from . import people
    return set(people.LYING_POSES)


def _lying_roll(pose):
    """The roll a lying pose is laid down with, or 0 for an upright one.

    ONE SOURCE OF TRUTH. `disaster.people.LYING_POSES` is what
    `_human_placement` actually applies, so reading it here rather than
    repeating "+-90" means a pose added to that table cannot end up with its
    body axis computed backwards in this one.
    """
    from . import people
    return float(people.LYING_POSES.get(str(pose), 0.0))


def _body_axis(pose, yaw_deg, roll_deg):
    """Unit vector from the placement point toward the figure's HEAD.

    A laid-down rig is turned about its own origin, which sits at the SOLES, so
    the whole body is on ONE side of the placement point and which side depends
    on the sign of the roll:

        roll +90  character up -> pre-yaw -Y -> world +(cos yaw, sin yaw)
        roll -90  character up -> pre-yaw +Y -> world -(cos yaw, sin yaw)

    THE SPIN DOES NOT ENTER. A side-lying figure is a laid-down one turned
    about its own long axis (`people.LYING_SPIN`, applied as `pitch_deg`), and
    a rotation about an axis cannot move a vector that lies along it — so the
    head still points at +u for every roll +90 pose, on its face or on its
    side.
    """
    br = math.radians(float(yaw_deg))
    u = (math.cos(br), math.sin(br))
    if float(roll_deg) < 0.0:
        return (-u[0], -u[1])
    return u


def _bearing(ax, ay, bx, by):
    return math.degrees(math.atan2(by - ay, bx - ax))


def _wreck_z(level):
    return float(DEBRIS_Z_M.get(str(level), 0.4))


class _Field(object):
    """Accumulates placements, plank specs and ground truth, and enforces the
    no-interpenetration and on-the-plate rules."""

    def __init__(self, cfg, ctx, rng):
        self.cfg = cfg
        self.ctx = ctx
        self.rng = rng
        self.sep = float(cfg.get("min_separation_m", 2.4))
        reg = ctx.get("region")
        self.region = tuple(float(q) for q in reg) if reg else None
        self.margin = float(cfg.get("edge_margin_m", 1.6))
        self.taken = []
        self.humans = []
        self.records = []
        self.debris = []
        self.refused = {}
        specs = ctx.get("plank_specs")
        pts = ctx.get("deck_points") or ()
        # THE DECK IS MEASURED IF THE CALLER MEASURED IT. `_FlatDeck` is the
        # bench-and-tests fallback and it disables the tilt test by
        # construction, which is honest: with no boards there is nothing to be
        # tilted.
        self.deck = _Deck(specs, points=pts) if (specs or pts) else None
        self.max_tilt = float(cfg.get("max_deck_tilt_m", 0.26))
        self.max_tilt_pile = float(cfg.get("max_deck_tilt_pile_m", 0.55))

    # -- refusals -----------------------------------------------------------
    def _no(self, why):
        self.refused[why] = self.refused.get(why, 0) + 1
        return False

    # -- the plate ----------------------------------------------------------
    def inside(self, x, y, pad=None):
        if self.region is None:
            return True
        p = self.margin if pad is None else float(pad)
        x0, y0, x1, y1 = self.region
        return (x0 + p) <= x <= (x1 - p) and (y0 + p) <= y <= (y1 - p)

    def free(self, x, y):
        s2 = self.sep * self.sep
        for (px, py) in self.taken:
            if (px - x) ** 2 + (py - y) ** 2 < s2:
                return False
        return True

    # -- the surface --------------------------------------------------------
    def deck_for(self, level):
        """The surface sampler to use near a wreck at this damage level."""
        return self.deck or _FlatDeck(_wreck_z(level))

    def body_ok(self, deck, x, y, ux, uy, reach, pad=None, max_tilt=None):
        """Is there room, plate and a flat enough surface for a whole body?

        THE THREE STATIONS ARE THE POINT. A lying figure lies entirely on ONE
        side of its placement point, so testing that point tests its feet and
        nothing else — the plate check, the spacing check and the tilt check
        all have to run at the head and the waist as well. This is the gate
        that turned "a figure on the tilted face of a sheet of plywood" into a
        refusal instead of a render.
        """
        pts = [(x, y), (x + ux * reach * 0.5, y + uy * reach * 0.5),
               (x + ux * reach, y + uy * reach)]
        for (ex, ey) in pts:
            if not self.inside(ex, ey, pad):
                return self._no("off_plate")
        for (ex, ey) in pts:
            if not self.free(ex, ey):
                return self._no("too_close")
        prof = deck.profile(x, y, ux, uy, reach)
        if max(prof) - min(prof) > (self.max_tilt if max_tilt is None
                                    else float(max_tilt)):
            return self._no("deck_tilt")
        return True

    # -- authoring ----------------------------------------------------------
    def pick_usd(self):
        pool = list(self.ctx.get("humans") or ())
        return self.rng.choice(pool) if pool else None

    def add(self, x, y, z_ground, yaw, pose, where, usd=None, sink_m=0.0,
            alive=True, note=None, extra=None):
        """Author one casualty. Returns the placement dict, or None."""
        usd = usd or self.pick_usd()
        if not usd:
            return self._no("no_rigged_character") or None
        prone = str(pose) in _LYING_POSES()
        p = _human_placement(self.ctx, usd, x, y, z_ground, yaw, pose,
                             prone=prone)
        if p is None:
            return None
        if sink_m:
            p["z_m"] = float(p["z_m"]) - float(sink_m)
        plan = _ATTITUDE.get(pose) or {}
        hgt, _d = _measure(self.ctx, usd)
        reach = float(plan.get("reach", 1.0)) * hgt
        ux, uy = _body_axis(pose, yaw, _lying_roll(pose))
        # A BODY OCCUPIES A LINE, NOT A POINT. Registering only the placement
        # point leaves 1.8 m of body invisible to the spacing check and the
        # next draw is free to lay somebody across its chest.
        self.taken.append((x, y))
        self.taken.append((x + ux * reach * 0.5, y + uy * reach * 0.5))
        self.taken.append((x + ux * reach, y + uy * reach))
        self.humans.append(p)
        rec = {
            # 3 DP, NOT 2. `_Deck`'s cells are 0.8 m and its steps are hard
            # edges, so a centimetre of rounding can move a body's recorded
            # position into the NEXT cell and make the ground truth disagree
            # with the surface it was actually seated on. A millimetre cannot.
            "x": round(float(x), 3), "y": round(float(y), 3),
            "z": round(float(p["z_m"]), 3),
            "pose": p.get("pose", pose),
            # THE FIRST OF THE TWO AXES, as a field and not as a pose name a
            # reader has to decode: front, back, sideways, or sitting.
            "attitude": plan.get("attitude", "upright"),
            "where": where,
            "yaw": round(float(yaw) % 360.0, 1),
            # WHICH WAY THE BODY POINTS. A box drawn round a prone target has
            # to know which end is which, and it is not recoverable from `yaw`
            # alone because the roll sign decides it.
            "body_axis_deg": round(math.degrees(math.atan2(uy, ux)) % 360.0, 1),
            "reach_m": round(reach, 2),
            "alive": bool(alive),
            # SET FROM THE GEOMETRY BELOW, not from the fact that something was
            # done to the figure. `_cover` overwrites all four.
            "visibility": "full",
            "occlusion": "none",
            "covered_frac": 0.0,
            "sunk_frac": round(float(sink_m) / max(_d, 1e-6), 3),
            "visible_parts": [n for (n, _a, _b) in _PARTS_EXTENDED],
            "boards": 0,
            "note": note,
        }
        if extra:
            rec.update(extra)
        self.records.append(rec)
        return p


# ---------------------------------------------------------------------------
# The debris that goes ON the body
# ---------------------------------------------------------------------------

def _trim_spans(spans, cap):
    """Clip `spans` (in body-reach units) so their union is at most `cap`.

    ENFORCED HERE RATHER THAN CHECKED AFTERWARDS, and that is the whole
    difference between a hard example and a wrong label. A pattern that would
    bury more of a figure than `max_covered_frac` allows is SHORTENED, so the
    module cannot author a body it then has to describe in the ground truth as
    invisible. The trim comes off the far end of the last span, which is the
    end furthest from the head for every pattern that has one.
    """
    out, used = [], 0.0
    for (lo, hi) in spans:
        lo, hi = max(float(lo), 0.0), min(float(hi), 1.0)
        if hi <= lo:
            continue
        room = cap - used
        if room <= 0.02:
            break
        if (hi - lo) > room:
            hi = lo + room
        out.append((lo, hi))
        used += hi - lo
    return out


def _cover_piece(px, py, ux, uy, along, across, thick, klass, top_z, deck_z,
                 rng, propped, lateral=0.0, run_along=False):
    """One piece of debris resting on a body. Returns a `planks` spec.

    ACROSS, NOT ALONG — for everything except the `flank` pattern. A board
    parallel to a prone figure hides nothing and reads as a coincidence, so a
    covering piece is yawed to the body's normal with a modest spread: enough
    that the set does not look combed, not so much that one ends up running
    down the body. `planks._box` puts the piece's LENGTH along its local +X, so
    the length is the dimension that crosses the body and the WIDTH is the
    stretch of body it hides — which is why `along` and `across` are named for
    the body rather than for the board.

    TWO WAYS A PIECE SITS ON A BODY, and both are in the debris photographs:

      flat     lying across the figure, bridging: its middle is held up by the
               body and its ends are unsupported over the surrounding deck.
               `z` is the body's top plus half the thickness.
      propped  one end down on the debris surface, the piece rising over the
               body and continuing past it into the air. This is the one that
               gives an aerial frame a shadow and a depth cue.

    THE PROPPED SIGN IS EASY TO GET WRONG, so it is written out. `planks._box`
    builds R = Rz(yaw) @ Ry(pitch) @ Rx(roll) with the length along local +X,
    so a point at (+L/2, 0, 0) lands at world z of `-sin(pitch) * L/2`:
    POSITIVE pitch drops the +X end. The yaw is set to the bearing of the
    direction the piece RISES in, so the +X end is the high one and the pitch
    is NEGATIVE.
    """
    if run_along:
        d = (ux, uy)                       # the piece lies down the body
        n = (-uy, ux)
    else:
        d = (-uy, ux) if rng.random() < 0.5 else (uy, -ux)
        n = d
    cx = px + n[0] * lateral
    cy = py + n[1] * lateral
    if not propped:
        j = 0.0 if run_along else rng.uniform(-0.16, 0.16)
        return {
            "x": cx + d[0] * j, "y": cy + d[1] * j,
            "z": top_z + thick * 0.5 + rng.uniform(0.0, 0.03),
            "len": across, "wide": along, "t": thick,
            "yaw": math.degrees(math.atan2(d[1], d[0]))
                   + rng.gauss(0.0, 13.0),
            "pitch": rng.uniform(-4.0, 4.0), "roll": rng.uniform(-7.0, 7.0),
            "class": klass, "propped": False,
        }
    rise = max(0.07, top_z + thick * 0.5 - deck_z)
    run = rng.uniform(0.55, 1.15)
    th = math.atan2(rise, run)
    overhang = rng.uniform(0.15, 0.60)
    ln = run + overhang
    off = (overhang - run) * 0.5
    return {
        "x": cx + d[0] * math.cos(th) * off,
        "y": cy + d[1] * math.cos(th) * off,
        "z": top_z + thick * 0.5 + math.sin(th) * off,
        "len": ln, "wide": along, "t": thick,
        "yaw": math.degrees(math.atan2(d[1], d[0])) + rng.gauss(0.0, 11.0),
        "pitch": -math.degrees(th), "roll": rng.uniform(-8.0, 8.0),
        "class": klass, "propped": True,
    }


def _cover(f, rec, pose, x, y, ux, uy, reach, base_z, deck_z, pattern,
           height=1.8):
    """Lay the pattern's debris on one body and write the ground truth.

    Returns the covered spans, in body-reach units. `base_z` is the body's own
    GROUND PLANE — the surface it is lying on, after any sink — and each piece
    is seated on `_crest` above that over its own footprint, so a sheet laid
    across a raised knee rests ON the knee instead of through it.

    THE PIECES ARE SOLVED AGAINST THE SPAN, NOT SCATTERED NEAR IT. Each piece
    hides `along` metres of the body's own length, so the generator walks the
    span from one end laying pieces until it is covered, drawing the WIDTH
    first and stepping by it — which is what makes `legs` mean the legs and
    `head_only` mean the head, rather than meaning "some boards, thereabouts".
    """
    rng = f.rng
    cap = float(f.cfg.get("max_covered_frac", 0.80))
    spans = _OCCLUSION.get(pattern, ())
    flank = (spans == "lateral")
    if flank:
        # ONE SIDE OF THE BODY, ALONG ITS LENGTH. A sheet that came down
        # edge-on beside a casualty: it hides one arm, one leg and half the
        # trunk, and leaves the other half of every part of the figure in
        # clear air. Different in kind from a band across the body, and the
        # only pattern whose covered fraction is not a fraction of the length.
        lo = rng.uniform(0.05, 0.22)
        hi = min(1.0, lo + rng.uniform(0.55, 0.78))
        spans = ((lo, hi),)
    else:
        spans = _trim_spans(spans, cap)
    if not spans:
        rec["occlusion"] = "none"
        rec["visibility"] = "full"
        rec["covered_frac"] = 0.0
        rec["boards"] = 0
        rec["visible_parts"] = _visible_parts(pose, ())
        return ()

    plan = _ATTITUDE.get(pose) or {}
    half_w = float(plan.get("half_w", 0.15)) * (reach / max(
        float(plan.get("reach", 1.0)), 1e-6))
    laid, made = [], 0
    for (lo, hi) in spans:
        s = lo
        guard = 0
        while s < hi - 0.02 and guard < 12:
            guard += 1
            klass, along, across, thick = _draw_cover_stock(rng)
            if flank:
                along = rng.uniform(1.1, 2.3)          # runs DOWN the body
                across = rng.uniform(0.5, 1.1)
                klass = "sheathing" if across > 0.6 else "deck"
            w = along / max(reach, 1e-6)               # in reach units
            if s + w > hi:
                w = hi - s
                along = w * reach
                if along < 0.10:
                    break
            t = s + w * 0.5
            px = x + ux * t * reach
            py = y + uy * t * reach
            # ON THE HIGHEST THING UNDER IT. A rigid board resting on a body
            # bears on the body's crest across its own footprint, not on the
            # chest height everywhere.
            top_z = base_z + _crest(pose, s, s + w, height)
            propped = (not flank) and rng.random() < 0.26
            if flank:
                lat = (half_w + rng.uniform(0.02, 0.24)) * (
                    1.0 if rng.random() < 0.5 else -1.0)
                sp = _cover_piece(px, py, ux, uy, across, along, thick, klass,
                                  top_z, deck_z, rng, False, lateral=lat,
                                  run_along=True)
            else:
                sp = _cover_piece(px, py, ux, uy, along, max(
                    across, 2.0 * half_w + 0.25), thick, klass, top_z, deck_z,
                    rng, propped)
            sp["for"] = "casualty"
            f.debris.append(sp)
            laid.append((s, s + w))
            made += 1
            # A SMALL GAP SOMETIMES. Debris that abuts perfectly along a body
            # is a fitted lid; real pieces overlap or leave a slot, and a slot
            # is what makes a partially covered figure legible at all.
            s += w * rng.uniform(0.86, 1.12)

    cov = _union(laid)
    if flank:
        # HALF THE WIDTH, SO HALF THE COVER. The union above is the stretch of
        # the body the sheet runs beside; only one side of it is under
        # anything, so the fraction of the FIGURE that is hidden is half of it.
        cov *= 0.5
    cov = min(cov, cap)
    rec["occlusion"] = pattern
    rec["covered_frac"] = round(cov, 3)
    rec["boards"] = made
    # SUNK COUNTS TOO, at half weight: a body a third of its depth into the
    # pile has its silhouette eaten from below, which no board explains.
    hidden = max(cov, float(rec.get("sunk_frac", 0.0)) * 0.5)
    rec["visibility"] = "partial" if hidden >= 0.15 else "full"
    rec["visible_parts"] = (
        [n for (n, _a, _b) in (_ATTITUDE.get(pose, {}).get("parts")
                               or _PARTS_EXTENDED)]
        if flank else _visible_parts(pose, laid))
    return tuple(laid)


# ---------------------------------------------------------------------------
# Placing one casualty
# ---------------------------------------------------------------------------

def _candidate(f, w, where, near=None):
    """A point for one body: `(x, y)` or None.

    `near` clusters this body onto one already down — Chiu et al. put only
    7.7% of victims alone, and two bodies together is also the strongest
    aerial cue there is that a searcher should stop.
    """
    rng = f.rng
    fp = max(6.0, float(w.get("fp", 12.0)))
    if near is not None:
        r_lo, r_hi = f.cfg.get("cluster_r_m", [1.8, 4.0])
        a = rng.uniform(0.0, 2.0 * math.pi)
        r = rng.uniform(float(r_lo), float(r_hi))
        return near[0] + math.cos(a) * r, near[1] + math.sin(a) * r
    if where == "street":
        pts = list(f.ctx.get("road_pts") or ())
        if not pts:
            return None
        # THE ROAD IN FRONT OF THIS HOUSE, not a road anywhere on the plate.
        # Drawn uniformly, `street` put bodies on carriageways two blocks from
        # any wreck — and a body on the road is a body that came OUT of the
        # house beside it. The MMWR's 37%-recovered-outdoors is displacement of
        # tens of metres, not of blocks. Nearest six, then one at random, so
        # the frontage is not always the same point.
        pts.sort(key=lambda q: (q[0] - w["x"]) ** 2 + (q[1] - w["y"]) ** 2)
        near_road = pts[:6]
        if ((near_road[0][0] - w["x"]) ** 2
                + (near_road[0][1] - w["y"]) ** 2) > 45.0 ** 2:
            return None
        rx, ry, tan = near_road[rng.randrange(len(near_road))]
        n = math.radians(float(tan) + 90.0)
        off = rng.uniform(-3.2, 3.2)
        return (float(rx) + math.cos(n) * off + rng.uniform(-1.2, 1.2),
                float(ry) + math.sin(n) * off + rng.uniform(-1.2, 1.2))
    # TIGHTER THAN IT WAS. `yard` ran to 2.40 footprints — 34 m off a 14 m
    # house — and a wreck on the shoulder of a 46 m track has most of that
    # annulus outside the corridor entirely, which is how a casualty ended up
    # on an untouched lawn two lots away under an intact tree. `_Field.in_track`
    # refuses those now, but refusing thirty candidates to place one body is a
    # slow way to say "draw closer to the house", and closer is also the better
    # model: the MMWR's 37%-recovered-outdoors is displacement of metres to
    # tens of metres, not of blocks.
    lo, hi = {"pile": (0.42, 1.00), "skirt": (0.95, 1.60),
              "yard": (1.15, 2.00)}.get(where, (0.6, 1.4))
    a = rng.uniform(0.0, 2.0 * math.pi)
    # sqrt so the draw is uniform over the ANNULUS rather than over the radius,
    # which otherwise crowds every body onto the inner edge.
    r = fp * math.sqrt(lo * lo + (hi * hi - lo * lo) * rng.random())
    return w["x"] + math.cos(a) * r, w["y"] + math.sin(a) * r


# HOW MUCH DEBRIS EACH LOCATION CLASS WANTS UNDER IT. The deck is measured
# (`_Deck`), so this is a real test rather than a label: `pile` means "on the
# boards", `yard` means "on the ground beside them", and a draw that lands on
# the wrong surface is refused and retried rather than quietly mislabelled.
_DECK_BAND = {"pile": (0.03, 1.30), "skirt": (0.0, 0.80),
              "yard": (0.0, 0.22), "street": (0.0, 0.22),
              "trail": (0.0, 0.30)}


def _one_casualty(f, w, where, near=None, tries=22):
    """Place a single body. Returns `(x, y)` on success, else None."""
    rng = f.rng
    cfg = f.cfg
    pose = _weighted_key(cfg.get("poses"), rng, "lying_supine")
    plan = _ATTITUDE.get(pose) or _ATTITUDE["lying_supine"]
    usd = f.pick_usd()
    if not usd:
        f._no("no_rigged_character")
        return None
    height, depth = _measure(f.ctx, usd)
    reach = float(plan["reach"]) * height
    deck = f.deck_for(w.get("level"))
    band = _DECK_BAND.get(where, (0.0, 1.3))
    placed = None
    for _t in range(tries):
        c = _candidate(f, w, where, near=near)
        if c is None:
            return None
        x, y = c
        # A BODY LIES ALONG THE FLAT DIRECTION, and searching for it is the
        # difference between a corridor with casualties on the debris and one
        # with all of them out on the lawn. A plank mat is not flat, but it is
        # not isotropic either: `planks._lay` aligns boards weakly across the
        # flow, so at most points there IS a bearing along which a metre and a
        # half of surface agrees to a few centimetres, and a random yaw finds
        # it about one time in six. Measured on the bench's own field, trying
        # eight bearings and keeping the flattest took the `pile` and `skirt`
        # acceptance rate from ~1 in 12 candidates to better than half.
        #
        # It is also what a body does. Something that comes to rest on rubble
        # settles along the surface rather than across it.
        best = None
        a0 = rng.uniform(0.0, 360.0)
        for _k in range(8):
            yaw = (a0 + _k * 45.0) % 360.0
            ux, uy = _body_axis(pose, yaw, _lying_roll(pose))
            prof = deck.profile(x, y, ux, uy, reach)
            tilt = max(prof) - min(prof)
            if best is None or tilt < best[0]:
                best = (tilt, yaw, ux, uy, max(prof))
        tilt, yaw, ux, uy, z = best
        # THE HIGHEST BOARD, NOT THE AVERAGE ONE. A body laid at the mean of
        # its three stations is INSIDE whatever board stands above that mean —
        # up to half `max_deck_tilt_m` of it — which is the other half of
        # "humans going through the debris". Laid on the maximum it rests on
        # the highest board and merely bridges the lower ones, which is what a
        # body on a plank mat actually does. The tilt test is what keeps that
        # bridge short.
        if f.deck is not None and not (band[0] <= z <= band[1]):
            f._no("wrong_surface")
            continue
        # A BODY ON THE PILE IS DRAPED, NOT LEVEL — `max_deck_tilt_pile_m`.
        if not f.body_ok(deck, x, y, ux, uy, reach,
                         max_tilt=(f.max_tilt_pile
                                   if where in ("pile", "skirt") else None)):
            continue
        placed = (x, y, yaw, ux, uy, z)
        break
    if placed is None:
        return None
    x, y, yaw, ux, uy, z = placed

    s_lo, s_hi = cfg.get("sink_frac", [0.0, 0.32])
    sink = depth * rng.uniform(float(s_lo), float(s_hi))
    alive = rng.random() >= float(cfg.get("casualty_share", 0.25))
    p = f.add(x, y, z, yaw, pose, where, usd=usd, sink_m=sink, alive=alive,
              note="%s, %s" % (where, w.get("level", "debris")))
    if p is None:
        return None
    rec = f.records[-1]

    # ---- the debris that goes on top --------------------------------------
    from . import people
    # THE BODY'S OWN GROUND PLANE, off the PLACED figure rather than off the
    # pile: the figure has been sunk into the deck and the two are no longer
    # the same surface. `z_m` is the body's centre line once the roll has laid
    # it over, so its ground plane is one lift BELOW that and `_crest` measures
    # up from there.
    base_z = float(p["z_m"]) - people._lying_lift(pose, depth, height)
    pattern = _weighted_key(cfg.get("occlusion"), rng, "none")
    _cover(f, rec, pose, x, y, ux, uy, reach, base_z, z, pattern,
           height=height)
    return (x, y)


def _trail(f, budget):
    """Bodies thrown clear, downtrack of a flattened house.

    KEPT SMALL AND KEPT SHORT — the arithmetic is in the module docstring and
    it puts UNDER ONE person in a 500 m corridor genuinely thrown any distance
    at all. The bearing is the throw direction with a WIDE spread
    (`spread_deg`, a Gaussian): near-surface flow in a tornado is convergent
    toward the centreline, the 78%-left deposition figure is for lightweight
    debris lofted into the parent storm, and there is no published azimuthal
    distribution for victim deposition. A tight fan would be an invented claim.
    """
    spec = f.cfg.get("trail") or {}
    wrecks = [w for w in f.ctx.get("wrecks") or ()
              if w.get("level") in _FLATTENED]
    if not wrecks or budget <= 0:
        return 0
    lo, hi = spec.get("count", [1, 3])
    want = min(int(budget), f.rng.randint(int(lo), int(hi)))
    d_lo, d_hi = spec.get("range_m", [8.0, 26.0])
    sp = math.radians(float(spec.get("spread_deg", 55.0)))
    th = math.radians(float(f.ctx.get("throw_deg", 0.0)))
    order = list(wrecks)
    f.rng.shuffle(order)
    made = 0
    for w in order:
        if made >= want:
            break
        # A THROWN BODY LANDS IN THE OPEN, so it is placed by hand rather than
        # through `_candidate`: the point of the class is that it is NOT on the
        # pile, and its high contrast against mud or lawn is what makes it the
        # control case for the whole set.
        stub = dict(w)
        for _t in range(10):
            a = th + f.rng.gauss(0.0, sp)
            d = f.rng.uniform(float(d_lo), float(d_hi))
            stub["x"] = w["x"] + math.cos(a) * d
            stub["y"] = w["y"] + math.sin(a) * d
            stub["fp"] = 0.001            # `_candidate` then lands on it
            if _one_casualty(f, stub, "trail", tries=4):
                f.records[-1]["note"] = "thrown %.0f m from a %s house" % (
                    d, w.get("level"))
                f.records[-1]["where"] = "trail"
                made += 1
                break
    return made


# ---------------------------------------------------------------------------
# The catalogue — every attitude against every occlusion, on a grid
# ---------------------------------------------------------------------------

# The order the bench lays the occlusion patterns out in: nothing on the body
# first, then bands walking UP it from the feet to the head, then the two that
# take most of it, then the compound ones. A reader looking along a row should
# see the cover move.
CATALOGUE_OCCLUSION = ("none", "feet_shins", "legs", "legs_hips", "midriff",
                       "torso", "torso_head", "head_only", "upper_body",
                       "all_but_head", "all_but_feet", "banded", "flank")
CATALOGUE_POSES = ("lying_supine", "lying_supine_open", "lying_prone",
                   "lying_prone_reach", "lying_side_l", "lying_side_r",
                   "lying_curled_l")


def plan_catalogue(cfg, ctx, rng, poses=None, patterns=None, origin=(0.0, 0.0),
                   step=(3.4, 4.6), ground_z=0.0):
    """One casualty per (pose, occlusion) cell of a grid. For the bench.

    THE SAME CODE PATH `plan_people` USES — `_Field.add`, `_human_placement`,
    `_cover` — with the location sampling replaced by a grid and the pose and
    pattern forced instead of drawn. A bench that reimplements the thing it is
    checking proves nothing; a bench that cannot photograph a named cell is no
    use for iterating on one.

    Every body is laid along +X (yaw 0 for the roll +90 poses, 180 for the roll
    -90 ones, since `_body_axis` flips with the roll sign) so a row reads left
    to right and the occlusion patterns can be compared down a column.

    No sink: the catalogue is judging the POSE and the pieces on top of it, and
    a body that is also a third of its depth into the ground is two variables.
    """
    f = _Field(cfg, ctx, rng)
    poses = list(poses or CATALOGUE_POSES)
    patterns = list(patterns or CATALOGUE_OCCLUSION)
    cells = {}
    from . import people
    for j, pose in enumerate(poses):
        plan = _ATTITUDE.get(pose)
        if plan is None:
            continue
        roll = _lying_roll(pose)
        yaw = 0.0 if roll >= 0.0 else 180.0
        ux, uy = _body_axis(pose, yaw, roll)
        for i, pat in enumerate(patterns):
            x = float(origin[0]) + i * float(step[0])
            y = float(origin[1]) + j * float(step[1])
            usd = f.pick_usd()
            if not usd:
                return f.humans, f.debris, f.records, cells
            height, depth = _measure(f.ctx, usd)
            reach = float(plan["reach"]) * height
            p = f.add(x, y, ground_z, yaw, pose, "catalogue", usd=usd,
                      note="%s / %s" % (pose, pat))
            if p is None:
                continue
            rec = f.records[-1]
            base_z = float(p["z_m"]) - people._lying_lift(pose, depth, height)
            _cover(f, rec, pose, x, y, ux, uy, reach, base_z, ground_z,
                   pat, height=height)
            cells["%s__%s" % (pose, pat)] = (x + ux * reach * 0.5,
                                             y + uy * reach * 0.5)
    return f.humans, f.debris, f.records, cells


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def plan_people(cfg, ctx, rng):
    """Plan the whole casualty population. Returns `(humans, debris, records)`.

    `ctx` keys, and who owns each:

        wrecks       [{x, y, fp, intensity, level}]      the assembly launcher
        road_pts     [(x, y, tangent_deg)]               the assembly launcher
        plank_specs  the list handed to `planks.build`   the assembly launcher
        deck_points  [(x, y, top_z)] off the ARCHETYPE, OPTIONAL — see `_Deck`
        intensity_at f(x, y) -> 0..1, OPTIONAL           `disaster.tornado`
        canopies     [(x, y, r)] trees that still have a crown, OPTIONAL
        throw_deg    float                               `disaster.tornado`
        region       (x0, y0, x1, y1) metres, OPTIONAL   the assembly launcher
        humans       [usd]  RIGGED RenderPeople          `suburb_scene`
        resolver, asset_pools                            `scene_generator`

    `plank_specs` IS WHAT MAKES A BODY LIE ON THE DEBRIS instead of through it
    — see `_Deck`. Optional, because the bench and the host-side tests have no
    board field; without it the deck is `DEBRIS_Z_M[level]`, flat, and the tilt
    test cannot fail.

    `region` IS THE PLATE and without it the planner will lay somebody half off
    the edge of the world: a lying figure extends its whole length past its
    placement point, so a body whose feet are a metre inside the boundary can
    still have its head outside it. Every body is tested end to end.

    `debris` is the plank specs that do the occluding; the caller authors them
    with `disaster.planks`. Returned rather than authored because this module
    never touches a stage. Each spec carries its own `t`, `pitch`, `roll` and
    `class`: a PROPPED piece has a solved pitch, and randomising it in the
    launcher is the difference between a board resting on somebody and a board
    that has fallen off them.
    """
    f = _Field(cfg, ctx, rng)
    wrecks = [w for w in (ctx.get("wrecks") or ())
              if w.get("level") in _WRECKED]
    budget = int(cfg.get("max_total", 40))
    per = cfg.get("per_wreck") or {}
    order = list(wrecks)
    rng.shuffle(order)
    made = 0
    for w in order:
        if made >= budget:
            break
        lo, hi = per.get(str(w.get("level")), [1, 2])
        want = rng.randint(int(lo), int(hi))
        for _k in range(want):
            if made >= budget:
                break
            where = _weighted_key(cfg.get("where"), rng, "pile")
            if where == "street" and not ctx.get("road_pts"):
                where = "skirt"
            at = _one_casualty(f, w, where)
            if at is None:
                continue
            made += 1
            # ...AND THEY CLUSTER. 92.3% of Chiu's victims were found with
            # somebody else. The companion goes at the same location class,
            # a couple of metres away, and costs a slot from the same budget.
            if made < budget and rng.random() < float(
                    cfg.get("cluster_chance", 0.34)):
                if _one_casualty(f, w, where, near=at, tries=10):
                    made += 1
    made += _trail(f, budget - made)
    if f.refused:
        # SAY WHY A SCENE CAME BACK SHORT. "12 wanted, 7 placed" is not
        # actionable; `deck_tilt` says the plank field has no flat metre and a
        # half anywhere near that wreck, `wrong_surface` says the location
        # class could not find its own kind of ground, `off_plate` says the
        # plate is too small and `too_close` says it is too crowded.
        print("[tornado_people] {0} casualt(ies) placed; refusals {1}".format(
            len(f.humans), dict(sorted(f.refused.items()))))
    return f.humans, f.debris, f.records


def write_records(path, records, meta=None):
    """Ground truth to JSON, one entry per casualty."""
    import json

    d = os.path.dirname(os.path.abspath(path))
    if d:
        os.makedirs(d, exist_ok=True)
    with open(path, "w") as fh:
        json.dump({"meta": meta or {}, "people": records}, fh, indent=1)
    return path


def summarise(records):
    """Counts by ATTITUDE, by OCCLUSION, by visibility, by location.

    THE FIRST TWO ARE THE BENCHMARK. A summary that cannot show "every figure
    in this scene is face-up" or "nothing is covered below the waist" is not
    much of a summary — the previous one reported scenario counts, which
    looked entirely reasonable in the run that was rejected on sight.
    """
    by_a, by_o, by_v, by_w, by_p = {}, {}, {}, {}, {}
    parts = {}
    for r in records:
        by_a[r.get("attitude")] = by_a.get(r.get("attitude"), 0) + 1
        by_o[r.get("occlusion")] = by_o.get(r.get("occlusion"), 0) + 1
        by_v[r.get("visibility")] = by_v.get(r.get("visibility"), 0) + 1
        by_w[r.get("where")] = by_w.get(r.get("where"), 0) + 1
        by_p[r.get("pose")] = by_p.get(r.get("pose"), 0) + 1
        for n in (r.get("visible_parts") or ()):
            parts[n] = parts.get(n, 0) + 1
    part = [r for r in records if r.get("visibility") == "partial"]
    return {
        "total": len(records),
        "by_attitude": by_a, "by_occlusion": by_o, "by_visibility": by_v,
        "by_where": by_w, "by_pose": by_p, "visible_parts": parts,
        "boards": sum(int(r.get("boards", 0)) for r in records),
        "max_covered_frac": round(max(
            [float(r.get("covered_frac", 0.0)) for r in records] or [0.0]), 3),
        "mean_covered_frac": (
            round(sum(float(r.get("covered_frac", 0.0))
                      for r in part) / len(part), 3) if part else 0.0),
        "alive": sum(1 for r in records if r.get("alive")),
    }


# ---------------------------------------------------------------------------
# WHAT IS DELIBERATELY ABSENT, and why
# ---------------------------------------------------------------------------
#
# EVERY UPRIGHT FIGURE WHO WAS NOT HIT — removed 2026-08-27 on review, and this
#   is the largest change the module has had. It used to place, and the first
#   100 m render showed, 55 of them against 12 casualties:
#
#     `neighbour_dig`   3-6 civilians working one collapsed pile. The best-
#                       grounded scenario in the file — Greensburg had 68% of
#                       households doing search and rescue, the earthquake
#                       literature puts civilian extrication at 60-100%
#                       (Bartolucci et al. 2020) and Joplin fielded SEVENTEEN
#                       professional rubble-rescue personnel against 1,371
#                       injured. All true, and none of it a target.
#     `on_the_rubble`   survivors picking through their own house.
#     `street`          walking wounded on the carriageway.
#     `assisted`        two supporting a third.
#     `in_vehicle`      occupants standing beside a displaced car.
#
#   They are gone because this dataset scores finding the people the tornado
#   HIT, and every one of those figures walked out of it. They are also, and
#   separately, the EASY problem: a standing person on a debris field is a
#   vertical object on a horizontal one and reads from any altitude and any
#   azimuth. Nothing about the research behind them was found to be wrong and
#   the code is in git history if a distractor population is ever wanted.
#
# `parking_refuge` — the largest WILDFIRE scenario at 0.30, and it has no
#   tornado counterpart. Moore, Joplin and Mayfield all had NO public tornado
#   shelters. Ten to fifteen minutes of warning is not enough to receive it,
#   decide, lock up, drive, park and get inside; observed public-shelter use is
#   ~4% where one exists and 0% where none does.
#
# `gridlock` — a neighbourhood traffic jam. Across six metros over 2011-2018
#   only Oklahoma City ever produced a mass traffic reversal (Hatzis &
#   Klockow-McClain 2022), it was broadcast-directed, and it is a metro-
#   arterial phenomenon. The authors also state plainly that they *"have no
#   way to quantify the number of people who actually evacuated"*.
#
# `pools` / `cul_de_sac` — wildfire refuge geography. A pool is shelter from
#   radiant heat and nothing at all in a wind event.
#
# CELLARS, SAFE ROOMS AND HOUSE INTERIORS — real, and where people actually
#   shelter (bathroom 39%, closet 37%, hallway 10%: Hammer & Schmidlin 2002),
#   but a drone benchmark cannot score a target it cannot see. This is
#   `disaster.people`'s `exposed_interior` lesson: *a target that cannot be
#   seen cannot be labelled, and an annotation for an invisible one is worse
#   than no annotation at all.* It is also why `max_covered_frac` exists.
#
# FULLY BURIED FIGURES — same argument, and it is now enforced rather than
#   hoped for. `_trim_spans` shortens any occlusion pattern that would exceed
#   `max_covered_frac`, so the module cannot author a body it would then have
#   to describe as invisible.
#
# `trapped_sit`, THE ONE UPRIGHT CASUALTY — pinned sitting, legs under the
#   pile, head and shoulders proud. Cut on the second review for the same
#   reason the standing figures went: a figure sitting bolt upright in a
#   levelled block reads as somebody who sat down rather than as somebody a
#   building fell on. Every attitude in the mix is horizontal now.
#
# RESPONDERS, HEAVY EQUIPMENT, SEARCH MARKINGS, TRIAGE TENTS — T+12-24 h
#   artefacts. See `epoch_min`.
#
# MOBILE HOMES — 38-47% of US tornado deaths against ~5.4% of housing stock,
#   a ~10x exposure-adjusted per-capita risk (Fricker & Friesenhahn 2022). Our
#   suburb is entirely site-built modular kit, so the scene cannot represent
#   the highest-risk housing type at all. A small manufactured-home park is
#   still the single highest-value asset addition available.
