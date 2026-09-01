---
name: model-tornado-paths
description: Read before adding curvature, touchdown/liftoff ramps, or a near-surface wind-DIRECTION field to disaster/tornado.py (the urban-tornado round, plan §2.2/§2.4) — the literature verdict on whether a wobbled straight line is a realistic damage-path centerline, and the sourced knob-value table (curvature_deg_per_km, touchdown_m/liftoff_m, wobble_m/wobble_period_m, translation_frac, inflow_frac, over_frac) for stream P to reconcile against.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Model Tornado Paths — what the research says, in knob values

Companion: `_plans/tornado_path_research.md` (the full research write-up
this skill's numbers are drawn from — every claim here has a source there).
Also read `build-tornado-scenes/SKILL.md` first if you have not — it is the
prerequisite for the rest of the tornado pipeline (fracture, planks, ground
scour) and covers the "why the fire code cannot be reused" material this
skill does not repeat.

This skill is engineering-facing: numbers, mechanisms, and where each one
comes from. It does not re-explain tornado meteorology in general.

## What the code models today (`disaster/tornado.py`)

* `DEFAULTS["heading_deg"]` + `_axes(cfg)`: a straight track direction,
  origin `origin_m`, no curvature term.
* `_wobble(cfg)`: an `along -> lateral offset` function — two harmonics
  (`0.62*sin(2*pi*a/per) + 0.38*sin(2*pi*a/(per*0.37) + 1.71)`), amplitude
  `wobble_m` (DEFAULTS 22.0), period `wobble_period_m` (DEFAULTS 340.0).
  This is a MEANDER of the centerline itself — a shear, applied identically
  by `frame()` (reading track coordinates) and inverted exactly by
  `from_track()` (placing marks in track coordinates), so ground relief
  and the intensity field never disagree about where the centerline is.
* `intensity_field(cfg, region, rng)`: the EF proxy `0..1`. Cross-track:
  flat for `core_frac` of the half-width then smoothstep to 0. Along-track:
  width AND peak intensity both "breathe" via `along_min`/`width_min`/
  `along_period_m`, out of phase, two harmonics each. Edge: band-limited
  noise perturbs the LEVEL SET (the boundary), never the value, so untouched
  ground never speckles with damage.
* `throw_field(cfg)`: direction = heading rotated `curl_deg` toward the
  LEFT of travel (cyclonic bias), jittered `+-spread_deg` per object,
  distance scaling with intensity via `throw_m`.
* No infinite-vs-finite track: the field is evaluated everywhere: there is
  no `touchdown_m`/`liftoff_m` concept yet — a track implicitly crosses the
  whole plate.
* No wind-DIRECTION field: `intensity_field` gives magnitude only; nothing
  in the module today returns a bearing for a point off the centerline.

## What it does NOT model, that stream P is adding (plan §2.2/§2.4)

1. **Curvature** — a signed, along-track-dependent lateral shear
   (`curvature_deg_per_km`), implemented inside `_wobble` so `from_track`
   stays the exact inverse.
2. **Finite track extent** — `touchdown_m`/`liftoff_m`, ramping intensity
   (and width) up from 0 after touchdown and tapering before liftoff.
3. **`wind_at(cfg, x, y)`** — a near-surface wind DIRECTION field: tangential
   (cyclonic) + translation + inward radial (inflow) components, returning
   a world bearing, a speed fraction, cross-track position, and an "over
   the core" flag.

## Verdict on straightness

**A gently-wobbled straight line is a defensible first-order model for the
CENTERLINE of a 1-1.5 km window cut from the mature/strong phase of a real
track — but it needs a small constant curvature term, and it should not be
mistaken for a claim that real paths are smooth arcs.** Suckling & Ashley
(2006) explicitly justify straight-chord tornado paths at the scale of an
ENTIRE track because side-to-side deviation is small relative to along-track
length. But both of the best-documented urban-adjacent tracks (Joplin 2011,
El Reno 2013) show the deviation concentrated into a small number of large,
discrete heading changes — commonly near genesis and especially near
rope-out/occlusion — rather than continuous gentle bending. Real curvature
is LEFT-biased (cyclonic; mesocyclone-occlusion detachment steers the
tornado left/rearward of the parent storm late in life) and it is NOT
uniform along the track: mild through the mature core, sharper at the ends.
A single constant `curvature_deg_per_km` (this skill's recommendation
below) reproduces the mild-core case correctly; it will undersize what a
window landing on a genesis or rope-out segment would show, which is an
accepted simplification for this round, not an error to silently fix by
over-tuning the constant. Full case evidence and every number's source:
`_plans/tornado_path_research.md` §0.1, §1.

## Recommended knob values

All ranges are what the literature search actually supports; the default is
what this skill recommends stream P start from. **Every one of these is a
recommendation for the lead to reconcile against stream P's implementation
— none of it is meant to silently change `DEFAULTS` or drift the suburb
l1-l3 scenes**, per the plan's "byte-identical unless a preset opts in"
rule.

| knob | recommended range | recommended default | source / reasoning |
|---|---|---|---|
| `curvature_deg_per_km` | 3-15 deg/km (mature-phase segment); up to ~30 deg/km if a preset deliberately wants a "near rope-out" feel | **6 deg/km**, sign **+ = LEFT of travel** (matches `curl_deg`'s existing sign convention) | [I] computed from Joplin's own waypoint distances (research doc §0.1); no source publishes deg/km directly. Sign confirmed by Nixon & Allen (2021): occlusion-driven deviation is left/rearward-biased; El Reno's SE->E->NE swing is a ~90 deg LEFT turn over 26.2 km (~3.4 deg/km AVERAGED over its whole life, but concentrated late — the 6 deg/km default represents a window sitting in the more-curved-than-average part of a life, which is more useful for a 1.5 km urban crop than the whole-track average) |
| `touchdown_m` / `liftoff_m` ramp length (`ramp_m`) | 150-500 m for "visible but not dominant" within a 1-1.5 km plate | **300 m**, `None` (disabled, matching current suburb behavior) unless a preset explicitly wants a visible touchdown/rope-out | [I]/[E]. NIST's Joplin RMW reconstruction shows the core growing from 113 m to 260-290 m over roughly the first 3.2 km (2 miles) of a VIOLENT, long-track tornado — a full-scale ramp of that length would consume an entire 1-1.5 km urban plate on its own. 300 m is a deliberate compression of that real ramp so a scene can show SOME touchdown/rope-out character without losing the mature core; treat it as a scene-design choice anchored to, not measured at, the NIST figure |
| `wobble_m` (1.5 km URBAN plate) | 15-40 m | **keep the existing 22 m** (DEFAULTS, unscaled — it is metres, not a plate fraction) | [E]. No source isolates centerline meander from RMW growth or subvortex orbiting (research doc §2, §0.3). The existing value is NOT proven wrong by anything found this round; do not scale it up to match subvortex-orbit numbers (400-1000+ m radius, Bluestein et al. 2018) — that is a different, violent-tornado-specific phenomenon, not the centerline's own meander |
| `wobble_period_m` (1.5 km URBAN plate) | 250-500 m | **keep the existing 340 m** | same reasoning as `wobble_m` — no direct evidence found either way for the 1.5 km plate; the existing period (already tuned to avoid the "drawn S-curve" failure the build-tornado-scenes skill documents) is not contraindicated |
| `translation_frac` | 0.15-0.45 (0.15-0.22 for violent, slow-relative-to-strength tornadoes; up to 0.45-0.67 for weaker/relatively-fast movers) | **0.22** (down slightly from the plan's stated 0.25) | [C]/[I]. NIST's own Gmax fit for Joplin (4.5-5.0) gives translation_frac = 1/Gmax = 0.20-0.22 for the single best-documented violent tornado in the search. A 5-tornado cross-check (Joplin, Spencer SD, Mulhall, Naplate, Sidney — research doc §5.2) spans 0.15-0.67; 0.22 sits at the well-supported violent-tornado end, which is the regime an urban EF3-5 core scene is most likely representing |
| `inflow_frac` | 0.30 (unchanged) if the field stays a single 2D value applied uniformly across a building's height; **0.5-0.6** as a compromise default if the plan wants one number that better represents ground-level/low-story conditions; up to ~1.5-2.0 only for a purpose-built ground-hugging term (debris/scour, not upper-story glazing) | **0.5**, with the caveat below stated in the plan text at the call site | [C], the headline correction. NIST's Rankine-vortex fit for Joplin: alpha = 15-25 deg (0=radial, 90=tangential), i.e. near-surface (<20 m AGL) radial flow ~2-3.7x the tangential component; the report separately cites Karstens et al. (2013)'s independent tree-fall fit at "on the order of 2:1." This is 6-12x the plan's stated 0.30. BUT it is height-dependent: Kosiba & Wurman's DOW boundary-layer work found the strong inflow confined to roughly the lowest 10-14 m AGL, "little to no inflow" by ~30 m AGL. A single z-independent `wind_at` cannot represent both a ground floor and a tower's 30th story correctly with one number. 0.5 is a deliberate compromise, not the empirical value — it leans the model further toward the strongly-documented near-surface case than 0.30 does, without applying the full 2:1 ratio to every story of every building |
| `over_frac` | tie to `core_frac`, do not set independently | **= `core_frac`** (0.22 on the tuned suburb value, 0.30 on `DEFAULTS`) | [E]. No source gives this number directly — it is a scene-engineering abstraction. `core_frac` is the physical definition of "the RMW core is over this point" already in `intensity_field`; reusing it avoids a second, unvalidated threshold living next to a validated one |

### A note on the translation/inflow interaction the model doesn't yet have

Honerkamp, Yan & Snyder's review (research doc §5.1) notes that in the
classical decomposition, the translation vector adds to the TANGENTIAL
component on the right flank / subtracts on the left (which is exactly
`vt * t_dir + V * heading_dir`, already in the plan), but ALSO adds to the
RADIAL component on the REAR of the tornado and subtracts on the FRONT —
a front/rear asymmetry orthogonal to the left/right one, which
`wind_at`'s `inflow_frac * vt` term (scaled uniformly around the circle)
does not capture. Confirmed via DOW data on the Spencer, SD tornado. Not a
blocker for this round — flagged as a follow-up refinement, since adding it
means `inflow` needs its own directional term rather than a scalar
fraction of `vt`.

## The wind-direction model (plan §2.4) — verdict

The plan's model:

    vt     = prof                               (cross-track intensity profile, 0..1)
    t_dir  = -heading_dir if c > 0 (left) else +heading_dir     (cyclonic tangential)
    V      = translation_frac * peak             (along +heading_dir)
    inflow = inflow_frac * vt                     (toward the centreline)
    wind   = vt * t_dir + V * heading_dir + inflow * inward_dir

**The additive tangential + translation structure is directly validated**
by DOW data on Spencer, SD (1998): VT=15 m/s, Vt_max=81-97 m/s, and the
measured L/R wind-speed difference (~30 m/s) matches `2*VT` almost exactly
— confirming right-flank = tangential+translation, left-flank =
tangential-translation is the correct first-order shape. Keep it.

**The one number that should change is `inflow_frac`** — see the table
above. 0.30 is defensible as an UPPER-STORY value but understates the
near-surface case by roughly an order of magnitude against the single
best-documented tornado in the search (Joplin: ~2:1 radial:tangential at
<20 m AGL). Recommend 0.5 as the single-field compromise, with the
height-dependence caveat carried in a comment at the call site so a future
round that adds a z-term to `wind_at` has the anchor numbers ready
(2:1 near-surface, dropping toward the plan's original 0.30 by ~30 m AGL,
per Kosiba & Wurman).

**`translation_frac` should move slightly down**, from 0.25 to ~0.22, to
match the single best-documented violent-tornado fit (Joplin, NIST's own
Rankine-vortex reconstruction). This is a small change and well within the
uncertainty either way — not worth blocking on if stream P has already
locked 0.25 for another reason (e.g. matching an existing suburb value).

**`over_frac` (0.18 in the plan text) should be tied to `core_frac`**
rather than chosen independently — no source validates 0.18 specifically,
and `core_frac` is already the model's own definition of "inside the flat
core."

Consequences the ladder should still show, unchanged from the plan: on the
RIGHT flank the wind blows forward (windward face = the one facing BACK
along the track); on the LEFT flank the windward face looks DOWN-track; in
the core (`over_frac`) several faces are hit and debris deposits along
`bearing_deg`, which is where the plan's left-bias deposition inherits from
`curl_deg`'s cyclonic convention (see Q6/§6.1 in the research doc for why
that sign is doubly supported — a near-field rotational mechanism AND, for
a different physical reason entirely, a far-field lofted-debris mechanism
that does not otherwise transfer to this scene).

## Sources

See `_plans/tornado_path_research.md` for the full annotated source list
(17 entries, each with the URL actually opened and what it supports). The
headline ones behind the table above:

* NIST NCSTAR 3 (2014, final Joplin report — **not** NCSTAR 2 as the plan
  brief and the build-tornado-scenes skill currently say; corrected here,
  worth fixing at both those call sites next time either is touched):
  Gmax 4.5-5.0, alpha 15-25 deg, RMW 113->260-290 m, DR 2.0-2.3.
* Suckling & Ashley (2006): the straight-chord justification and national
  heading climatology.
* Marshall, Davis & Runnels (AMS, Joplin survey) and Marshall et al. (AMS,
  El Reno survey): both full-track heading narratives, nested EF-band
  widths, heavy-debris throw distances.
* Bluestein, Thiem, Snyder & Houser (2018, MWR): El Reno subvortex
  statistics — the "this is a different phenomenon from `_wobble`" evidence.
* Honerkamp, Yan & Snyder (JWEIA review): the Spencer SD DOW numbers behind
  the additive tangential+translation validation and the translation_frac
  cross-check table.
* Nixon & Allen (2021, WAF): the leftward-deviation mechanism and rate.
