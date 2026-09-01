---
name: fix-floating-debris
description: >-
  How "the debris is floating" was actually diagnosed and fixed — the `UsdGeom.BBoxCache` blind spot that made a library of airborne wood audit as clean, the points-based seating that replaced it (`bake.world_point_bounds`, `bake.reseat_meshes_in_file`, `vegetation.log_points`), the five separate debris populations and which code owns each, and the review discipline that would have found it in one round instead of six. Read before touching any seating, settling or audit code, before writing a probe that measures whether something is on the ground, and before answering a report that a scene "looks wrong" with a measurement that says it is fine.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Fix floating debris (and how not to spend six rounds on it)

## The one thing to take away

**`UsdGeom.BBoxCache` cannot tell you whether something is on the ground.** It
returns the AABB of an AABB, and for the thin diagonal slivers that Voronoi
fracture produces it over-reports the extent several-fold **downward as well as
upward**. Every seating pass, every audit and every probe in this repository
was built on it, so a library full of airborne wood measured as perfectly
seated, and six rounds of "still floating / no it isn't" followed.

If a look defect survives a measurement, **suspect the measurement.**

---

# What was actually wrong

Reported repeatedly against the 1 km suburban wildfire plate, over 2026-08-27
and 28. Five faults, in three different files, each of which fully explained
the symptom and none of which was the whole story.

## 1. The blockage litter used a hard-coded datum

`people._blocker_debris` lifted every piece to `0.10 m` with the comment "the
carriageway is at ~0.10". True on the 1600 m plat and on nothing else:
`apply_ground`'s z ladder SCALES WITH PLATE SPAN, and the 1 km preset pins
`roads.z_scale: 0.15`, putting its asphalt at 0.015 m. Every piece stood 8.5 cm
up. At this scene's 24-degree sun that is 19 cm of detached shadow, which is
exactly what reads as floating.

`_road_z(plan)` derives it from `ctx["z_scale"]`, which `build_ctx` now
publishes.

## 2. "Resting on the trunks" did not mean a trunk was under it

The fix for (1) seated the third of limbs that rest on the trunk sections at
their **centre** on a log crown — so a 3 m stick had BOTH ends 0.8 m in the
air, and because every propped piece was seated identically they lined up in a
band at crown height. Measured on the authored field: 15 of 38 pieces a
blockage, ends at 0.62-1.01 m.

**A spec-level check passed this**, because per piece the limb genuinely was
touching something. A limb across a log is a LEAN-TO: one end on it, the other
on the ground. The contact point is an END, not the middle.

## 3. The lowest VERTEX is not at `z - r`

`vegetation.log_mesh` jitters every ring vertex radially by `rough` (+-17% of
the radius) and only has a vertex at the exact bottom when `sides` puts one
there. So a piece seated on `z - r` floats by up to `rough * r` — measured
spread 0.039 m on a real field, 8 cm on a 0.48 m trunk section, always in the
float direction. **Invisible to any test that asserts on the planner's own
`{p0, p1, r0, r1}`,** which is what rounds one and two had asserted on.

`vegetation.log_points` is the barrel-vertex maths split out of the pxr-only
path so `people._seat_debris` can MEASURE what will be authored, and
`tests/test_blocker_debris.py` can measure the same thing on the host.

**And the piece measured has to be the piece built.** `scene_api._tube`
silently turned the caller's `sides` into `max(7, sides + 1)` AND seeded the
jitter with `abs(hash(prim_path))` — which Python randomises per PROCESS unless
`PYTHONHASHSEED` is pinned. `vegetation.piece_seed` is now shared by planner and
author, and the side count travels in the spec.

## 4. Seat on the GROUND, not on the road

The litter scatters over `half_w` either side of the centreline and +-9 m along
it, so most of it lands on the VERGE — and the ladder puts grass BELOW asphalt.
Seating the field on the road datum leaves every verge piece proud. Seating on
grass buries the minority that are on the carriageway by 1.2 cm, about 2.5% of
a trunk's diameter, which is invisible. **The error must point that way:** a
piece slightly bedded into a surface is a pose the world produces; one hovering
over it is not.

## 5. THE BIG ONE — `BBoxCache` is an AABB of an AABB

With the blockage litter clean, the reports continued. Measured on
`tree_Black_Oak_snag/log_017` in a built scene:

    local points fill   x +-0.471  y +-0.284  z +-0.240   (extent EXACT)
    world z from POINTS       0.4208 .. 0.5649   span 0.144   <- the truth
    world z from BBoxCache    0.0000 .. 0.9857   span 0.986   <- 6.8x, and
                                                                 reaches 0.42 m
                                                                 BELOW the piece

The extent attribute is not stale — it matches the points exactly in LOCAL
space. `ComputeWorldBound` transforms that box's eight corners and
re-axis-aligns; for a thin sliver lying diagonally in its own box, the result
is enormous in every direction. A chunk hanging 42 cm in the air reports a
bbox bottom of exactly `0.000` and reads as resting on the ground.

Everything downstream inherits the blindness: `bake._reseat_roots`,
`bake.audit_archetype`, `bake._seat_plan`, `vegetation.wood_debris`'s own
`piece.bounds[0][2] = ground_z - 0.001`, and all three probes written during
the investigation.

    bbox-based sweep of the archetype library:      4 offenders
    points-based sweep of the SAME files:       3,570 offenders, 62 of 78 files

## 6. `bole_*` was not in the family list

The felled trunk segments of a `*_fallen` tree — cut by `vegetation.topple`,
settled by PhysX, so exactly the population a settle freezes mid-flight. All
twelve in the library were airborne, Black_Oak's `bole_00` by **2.07 m**. They
never appeared in a name census because there are two per archetype and the
census printed the commonest 25 names. **Enumerate families exhaustively or not
at all**; a truncated census reads as a complete one.

And a bole seats on GRADE, not on what is under it: the ordinary support rule
dropped Black_Oak's 7.8 m trunk onto its own 1.15 m stump, leaving the far end
2.3 m up. A trunk balanced across a stump slides off.
`ground_only=("bole_",)` seats those on the earth and processes them FIRST, so
the loose wood settles onto the trunk rather than the trunk onto the wood.

---

# The five debris populations, and who owns each

They are authored by completely different code and fixing one does nothing for
the others. From the air they are indistinguishable, which is why a report has
to be resolved by MEASURING the population rather than by guessing.

| prim pattern | what it is | authored by | live or baked |
|---|---|---|---|
| `inst/blockdeb_<i>_<j>` | road-blockage litter | `people._blocker_debris` -> `scene_api._place_debris` | live |
| `inst/blocker_<i>` | a fallen-tree blockage | referenced archetype | baked |
| `inst/t_<i>/log_*` | tree ground debris | `vegetation.wood_debris` at bake | baked |
| `inst/t_<i>/bole_*` | felled trunk segments | `vegetation.topple` + settle at bake | baked |
| `inst/h_<i>/frag_*` | house fracture fragments | `fracture` + settle at bake | baked |

**A defect in a baked archetype is a defect in every instance of it.** 78 files
back ~6,000 scene objects, so the population to sweep is FILES, not objects —
and a per-file fix repairs the whole plate.

---

# The tools

## `bake.world_point_bounds(prim, xcache)`

Tight world AABB from a mesh's transformed POINTS. Use it anywhere a decision
depends on where geometry actually is. It is slower than `BBoxCache` and it is
the only number that is true.

## `bake.reseat_meshes_in_file(path, air_tol=..., ground_only=...)`

The repair. Pure `pxr`, offline, no Kit, no solver, no re-bake: it opens a
baked archetype, measures every mesh on points, and lowers loose pieces onto
whatever is genuinely beneath (or onto grade). ~11 s over 78 archetypes,
converges in 2-3 passes.

Three details that matter:

- **The shift goes INTO the transform, not onto it.** Appending an
  `xformOp:translate` moved each piece by `dz * its own z scale` — these
  fragments carry an `xformOp:transform` WITH SCALE and an appended translate
  composes inside it. Measured: an authored -1.579 m moved the piece -1.279 m
  (scale 0.8098); -0.904 moved it -0.327 (scale 0.3616). **A repair that
  silently applies a fraction of what it computed is worse than none**, because
  the audit afterwards reports a smaller number and reads as progress.
  `bake._shift_z` adds `dz` to the matrix's translation.
- **The support rule is `_reseat_roots`' v3**, unchanged and correct: the
  supporter's top must land IN our vertical span (a box that merely TOWERS PAST
  us might be a wall we are wedged against, and the tie breaks toward
  dropping), and the plan overlap must be a SEAT — `min_ovl` of the SMALLER of
  the two plan areas, so a small stub can hold up a large sheet. Fed
  bbox-of-bbox boxes it is worthless; fed point boxes it is exact.
- **Run it to a fixed point.** A piece can land on one that later moves.

## `tools/debris_float_probe.py`

Points-based, two modes:

    tools/debris_float_probe.py --archetypes DIR      # the complete answer
    tools/debris_float_probe.py <flat scene.usd>      # a built plate

`--archetypes` is the one to reach for. Probing a scene stops at `-n` floaters
and reports whichever population the traversal reached first — which is how an
early round concluded "it is the tree bake" from ten `tree_Black_Oak_snag`
hits that turned out to be sticks resting on other sticks.

## `tests/test_blocker_debris.py`

Eight groups, 2,432 pieces over four plate sizes and sixteen seeds, no Isaac
and no `pxr`. It exists because the blockage litter's three faults were each
"verified" against the spec and each shipped.

---

# Acceptance

Measure, do not assert. The bar agreed with the user is **2 cm**, and what
gets reported is the distribution, not a pass/fail against a threshold that
might be the wrong one:

    6,005 loose pieces (log_ / frag_ / brk_ / debris / bole_)
    p50 0.0000   p90 0.0000   p99 0.0000   MAX 0.0197 m
    pieces over 2 cm: 0        all 12 boles at 0.0000

---

# The review discipline that would have saved four rounds

- **A sample is not a survey.** Stopping at the first N offenders and naming
  the population they belong to is a finding dressed up. Sweep everything, or
  say plainly that it is a sample.
- **A name filter is a place to hide.** Two populations were missed by one
  (`bole_`, and the whole unfiltered scene sweep). Enumerate exhaustively, and
  print the whole census, not the top of it.
- **A support test is only as good as its boxes.** The same v3 rule was
  worthless on bbox boxes and exact on point boxes.
- **False positives cost credibility too.** An unfiltered scene sweep flagged
  1,732 meshes; ~1,030 were fence infill held by a 0.08 m frame, lamp heads
  held by their own pole, bench seats and car bodies — all rejected by the
  seat-overlap and towers-past rules. Thin and towering supports are the
  systematic false-positive mode.
- **When the user says it looks wrong and the numbers say it is fine, the
  numbers are the thing to doubt.** Ask for one prim path. One did more than
  four sweeps: `/World/stage/generated/inst/t_5425/log_017`.

---

# Known gaps

- **The upstream passes still use `BBoxCache`.** `_reseat_roots`,
  `audit_archetype` and `_seat_plan` run at BAKE time and were not changed —
  only the offline repair was. A fresh `bake_archetypes_launch_script.py` run
  will reproduce the floating and need `reseat_meshes_in_file` after it. Moving
  those three onto `world_point_bounds` is the real fix and is not done.
- **`vegetation.wood_debris` seats on a trimesh bbox** (`piece.bounds[0][2]`),
  which has the same failure for the same reason. Not changed.
- **The repair is a vertical translation only.** It cannot lay a piece down,
  so a long piece frozen at a steep angle is seated at its lowest point and
  keeps its angle. That is correct for chunks and marginal for anything long
  and thin; `ground_only` exists because a bole hit exactly that limit.
- **No physics.** Settling the debris with PhysX would be the authoritative
  answer and is blocked on the archetypes being instanced (USD forbids
  authoring inside an instance, and un-instancing ~5,500 trees is the
  documented OOM). The affordable version is to settle the 78 ARCHETYPES, each
  in its own frame — which is what the bake already does, and what needs the
  points fix above to be trusted.

---

# The same blind spot recurred in `disaster/settle.py` (fire-city bakes, 2026-08-31)

Independent of everything above — a different pipeline (per-building fire
bakes, not the suburb wildfire plate), a different file — but the exact same
mechanism, closing the loop the "Known gaps" section above predicted. Found on
the `city_smoke43` fire-city smoke bake:

`settle.py`'s below-grade audit (`_z_min`) used `UsdGeom.BBoxCache`, which is
the AABB of a rotated shard's own local AABB and inflates DOWNWARD by up to
1.74 m (measured p99 1.03 m / max 1.74 m on 566 gac bodies, p99 0.56 m / max
0.70 m on 550 kit bodies). `_lift_below_grade` then hoisted 131 (gac) and 59
(kit) bodies by that spurious amount — a repair pass MANUFACTURING floaters,
exactly this skill's headline finding, in a completely different subsystem.
Two more faults rode along: the final "still moving" recheck stepped only
0.33 s and flagged anything over 12 mm/s (a body creeping in a settled pile,
not one frozen mid-flight — 80 of 561 bodies deleted from one export for
this alone), and the stall detector gave up the instant the moving-body count
stopped setting a new all-time minimum, cutting a 400-step quiet phase off at
350.

**`SETTLE_REST_V2=1`** (opt-in — see `settle.py`'s own module docstring) is
the fix: grade measured off `_points_z_min` instead of `BBoxCache`; a body
counts as moving only if its NET travel over a rolling `creep_window` (1 cm
`creep_tol`) exceeds the tolerance, however much it jitters inside the
window; a stall freezes the jitterers (`rigidBodyEnabled=False`, so they keep
holding up whatever is on them) and lets the loop keep chasing bodies that
are genuinely still travelling; the convex-decomposition hull budget scales
with piece size (`_scaled_decomp`) instead of one fixed budget for a 0.8 m
chip and a 25 m sliced shell alike. Off by default — the MCE kit look is
frozen and this switches the code path — but the fire drivers
(`scene_gen/tools/fire_bake.sh`, `scene_gen/tools/fire_city_bake.sh`) turn it
on for every baked record whose kind is not `kit`. `verify_export`/
`REST_CHECK` fail a bake with `still_moving > 0`; `FB_REST_STRICT=1` makes
that a hard failure of the driver script rather than a loud print.

Same lesson as the section above, restated because it kept being true: a
seating/rest AUDIT built on `BBoxCache` will pass a scene that is actually
airborne, and the fix — for a shard as much as for a whole rigid body — is to
measure the mesh's own POINTS, never its box.
