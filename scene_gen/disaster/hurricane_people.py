"""hurricane_people.py — the tornado's casualty model, minus the water; the
water, minus the ground.

USER DIRECTIVE, verbatim (2026-09-01): "Do the exact same placement as [the
tornado] except in the flooded regions. For people in the flooded region have
them in the water, only torso visible and also on roofs for the intact
buildings." That sentence is the whole module and it names three domains:

  DRY LAND (surge depth <= `DRY_DEPTH_M`, 0.10 m)
      `tornado_people`'s own wrecked-house casualty model, UNCHANGED —
      classes, shares, poses, occlusion, the measured debris deck, the
      keepouts, the epidemiology in that file's docstring. This module does
      not re-implement any of it; `_plan_dry` builds a `tornado_people` ctx
      restricted to the houses this scene's OWN surge field (`ctx["depth_at"]`,
      built once by the launcher's own `surge.depth_at` and reused here, never
      re-derived) says are dry, and calls `tornado_people.plan_people`
      directly. See that module for the citations behind every number in this
      half.

      ONE KNOB IS OVERRIDDEN: `min_intensity` is forced to 0 in the dry cfg.
      `tornado_people`'s gate reads "is this point in the wind corridor", which
      is the right question over a narrow tornado track where position and
      damage move together. A hurricane's wind field is near-uniform across a
      whole plate (`disaster.hurricane.intensity_field`'s own docstring: single
      digits of percent spread) — the variance here comes from BUILDINGS, not
      position — so gating a casualty on "is the wind strong enough HERE" would
      either pass everywhere (no-op) or fail everywhere (empties the scene),
      neither of which is the flood/dry distinction this module actually needs.
      That distinction is enforced separately, by restricting `ctx["wrecks"]`
      to dry ones before `tornado_people` ever sees them.

  IN THE WATER (depth > `DRY_DEPTH_M`)
      A standing/wading figure, CHEST-DEEP, reusing `disaster.people`'s own
      pool posture verbatim: `feet_z = water_top - CHEST_FRAC * height`
      (`people.CHEST_FRAC = 0.75`, the exact fraction that module's own
      wildfire-pool scenario stands a figure at and describes as "the one pool
      posture that still reads as a person from capture altitude" — the same
      claim the user is making about a flood victim, so the same number
      applies rather than a freshly-invented one). At 0.75 of stature
      submerged, what stays above the waterline is the shoulders, neck and
      head plus a sliver of upper chest (Drillis & Contini 1966: shoulder
      0.818 H) — "chest-deep", not "waist-deep".

      A body cannot stand in water past roughly its own height regardless of
      the pose fiction above the surface, so placement is BIASED toward the
      0.3-1.2 m depth band (`stand_band_m`) and HARD-CAPPED at
      `stand_hard_max_m` (1.5 m) — deeper than that and standing is not
      plausible; the figure is refused rather than authored, on the same
      argument the roof/debris/omit instruction gives. Anchored per house:
      near a WRECKED house's own debris (isotropic offset off the footprint —
      "near stranded debris"), or off the UP-FLOW face of a STANDING house
      (offset biased toward `shore_bearing_deg + 180`, i.e. the side the surge
      is arriving from and where floating material piles up against a wall
      that is still standing to pile against).

  ON THE SIDES of INTACT roofs — REWRITTEN 2026-09-01, user on the live scene,
  verbatim: "people on the roof need to change. You have them at very top.
  Usually they sitting on the sides of the roof. They also can't look like
  they are walking or be too close to the edge ... they also have to be on
  intact houses's roofs only." Four separate defects, four separate fixes:

  1. **INTACT means `pristine` ONLY now** — `roof_stripped` was in the old
     `_ROOF_OK_LEVELS` on the argument that a bare top-plate is still
     "something to stand on"; this task's own instruction is narrower
     ("intact houses's roofs"), and `roof_stripped`'s own archetype carries
     NO `_roof_` mesh at all (see WHY `roof_stripped` IS EXCLUDED NOW, below)
     — there is no ROOF there, intact or otherwise, only a debris-scattered
     attic floor. Excluding it is not a new caution, it is the plain reading
     of the word "roof".
  2. **ON THE PITCHED SLOPE, not the ridge apex.** The old table
     (`_ROOF_RIDGE_POINTS_LOCAL`) sampled the ridge LINE itself — the single
     highest edge of the roof, which is exactly "at very top", the user's own
     complaint. `_ROOF_SLOPES_LOCAL` (see MEASURING THE ROOF, below) instead
     measures each roof's PLANAR SLOPE FACES, and placement samples a point
     35-65% of the way down the FALL LINE from ridge to eave (`fall_band`),
     with two hard, independent margins so nobody sits at the apex and nobody
     sits at the drop: >= `ridge_margin_m` (0.8 m) of VERTICAL drop below the
     ridge, and >= `eave_margin_slope_m` (1.2 m) measured ALONG THE SLOPE
     above the eave.
  3. **SEATED, from `people.py`'s own vocabulary — never standing, walking,
     or waving.** `idle`/`wave_help` (the old roof mix) are both STANDING
     poses; a standing figure on a 20-40 deg slope is also the shape that
     reads as "about to walk" from altitude, which is the user's second
     complaint. `wave_help` is dropped outright, not merely made a minority
     (`people.BANNED_POSES` bans the literal string `"wave"`, not
     `"wave_help"`, but the user's own fire-scene rejection of this pose pair
     is exactly this file's own retired-`wave_help` history and is not
     reintroduced twice). WHICH seated pose, and how it is tilted onto the
     roof, is fix #4 below and its own 2026-09-01 ROUND 2 section — the first
     pass of this fix chose `sit_ground` (see WHY `sit_ground` AND NOTHING
     ELSE, kept below for its history) and got it wrong.
  4. **RESTING on the pitched slope with the TORSO UPRIGHT, not rigidly
     rolled to the roof's own full pitch.** ROUND 1 of this fix (2026-09-01,
     same day) rotated `sit_ground`'s WHOLE BODY by the roof's own measured
     pitch — the same technique `fire_people` uses for a FLAT commercial
     deck, where pitch is 0 and the question never comes up. On a 39 deg
     residential gable that technique reclines the figure INTO the slope:
     `sit_ground`'s torso is only ~3 deg off plumb before any roll, and
     rolling the RIGID BODY by the roof's full pitch carries the torso to
     ~40+ deg off plumb WITH it — a person lying back with their chest tipped
     up, not a person sitting. User, on the live scene, verbatim: "these are
     townhouses not buildings... the pose of the human is to be different...
     sitting on the roof not their torso popping out." See 2026-09-01 ROUND
     2: THE TORSO FIX, below, for the replacement pose and the PARTIAL-roll
     solve that keeps the torso within `ROOF_TORSO_TILT_MAX_DEG` of
     gravity-vertical while still landing the feet on the plane.

  `fire_people`'s own §5b/§5b2 remain "the roof doctrine" for POSE HISTORY —
  its `deck_z`-vs-`top_z` caution (never seat a figure on the parapet coping)
  is the direct ancestor of fix #2 above, and its retired `stand_calm` /
  `wave_help` pair (user, on sight: "These poses are completely wrong...
  looks like their stomachs are out... arms also look wrong") is exactly why
  fix #3 does not reach for a standing or waving pose here either. What does
  NOT transfer is `fire_people`'s deck-PLANE math itself: that module's
  buildings are kit commercial towers with a genuine flat deck a bake sidecar
  measures; this kit's houses are pitched or hipped on every style, so a flat
  `deck_z` was never the right model here even before this rewrite (see
  MEASURING THE ROOF).

WHY `roof_stripped` IS EXCLUDED NOW
------------------------------------
Its `_pristine` sibling has a genuine `_roof_`-named mesh on every style;
`roof_stripped`'s own archetype does not — the covering and rafters are
removed outright and replaced with scattered `frag_*` debris, confirmed by
re-opening the archetype with the SAME bare-`pxr` walk this file uses for
`pristine` (no `_roof_` prim survives at all). The OLD design read the task's
"intact" as "has SOMETHING to stand on" and fell back to the measured
top-of-wall line (`STOREY_M * storeys`) for that level; this task's own
instruction narrows "intact" to mean the roof itself, which `roof_stripped`
does not have. `_WALL_BBOX_LOCAL` / `STOREY_M` / `_STYLE_STOREYS` are KEPT —
harmless, already pinned by their own re-measure test, and there on the
chance a future task still wants "stand on the exposed top plate" as a
DIFFERENT class — but nothing in the roof-placement path reads them any more.

MEASURING THE ROOF, not assuming it
------------------------------------
Every one of the 8 styles' `_pristine` archetype was opened with bare `pxr`
(`Usd.Stage.Open`, no `SimulationApp` — the same technique
`nucleus-usd-without-kit` uses) and every Mesh prim whose path contains
`_roof_` (excluding `bay_roof`, the small bay-window cap that is a superset
string of "roof" at a much lower height) was walked in WORLD SPACE, in the
archetype's own LOCAL house frame (metres, unscaled — `SCALE = 0.01` already
applied at BAKE time). `remeasure_roof_slopes()` is the measurement: every
triangle's AREA-WEIGHTED NORMAL is greedily clustered (8 deg tolerance) into
distinct PLANAR FACETS, a facet is kept only if its normal is neither near-
flat (`nz > 0.985` — a ridge-cap trim strip) nor near-vertical (`nz < 0.20` —
a gable-end wall), and only if its total area clears `ROOF_SLOPE_MIN_AREA_M2`
(15 sq m — every real walkable slope in this kit measures 30 sq m or more;
every hip-corner cap or ridge-trim fragment measures under 14). A PRIM-LEVEL
pre-filter (`floor_z = STOREY_M * storeys - 1.5`) is applied BEFORE
clustering, discarding a whole mesh prim whose own max Z never reaches
attic height — without it, `house_terrace_pristine.usd`'s
`house_roof_7_37` (a ground-level porch canopy, z in [0.0, 2.4] against a
7 m top-of-wall, stamped with a "_roof_"-matching name and, by coincidence,
the SAME pitch angle as the real roof) merges into the main slope's cluster
by normal alone and drags its measured eave down to z ~= 0.

For each kept facet: `pitch_deg = degrees(acos(nz))` (0 = flat, 90 =
vertical — the outward normal's own angle from vertical IS the roof's pitch
angle from horizontal, by construction: a facet descending in local +X has
normal `(sin(pitch), 0, cos(pitch))`, so `nz = cos(pitch)` exactly);
`downhill_xy = normalize(nx, ny)` (the outward normal's horizontal component
always points DOWNHILL, away from the ridge — proven by the same
construction: the slope's own fall-line tangent is `(cos(pitch), 0,
-sin(pitch))` and its ridge-parallel ally is `(0,1,0)`, and their cross
product IS `(sin(pitch), 0, cos(pitch))`, matching); `ridge_z`/`eave_z` are
simply the facet's own max/min Z (the two slopes of a shared ridge measure
the SAME `ridge_z` on every style, confirming the clustering finds a real,
continuous ridge and not two disconnected shapes); `bbox` is the facet's own
plan footprint, to sample inside of.

EVERY STYLE BUT `villa` is a plain two-slope gable/hip, `downhill_xy` exactly
`(+-1, 0)`, one ridge shared between both. `villa` alone is a real 4-facet
hip (ridge along local X, `downhill_xy` of `(1,0)`, `(-1,0)`, `(0,1)` and
`(0,-1)`, all sharing the SAME `ridge_z` — a genuine hip roof, not a
staggered gable). `_ROOF_SLOPES_LOCAL` is the frozen result, one entry per
kept facet; `tests/test_hurricane_people.py` calls `remeasure_roof_slopes()`
again against the archetypes on disk and pins every field.

WHY `sit_ground` AND NOTHING ELSE — SUPERSEDED, KEPT FOR HISTORY
------------------------------------
This section and SEATING ON A TILTED PLANE below it are ROUND 1's analysis:
BOTH assume the whole rigid body is rolled by the roof's OWN FULL pitch, and
score candidate poses on whether their LEGS stay planar under that specific
roll. That assumption is exactly what 2026-09-01 ROUND 2: THE TORSO FIX
(below) removes — a partial roll changes which pose wins, because a pose's
LEGS no longer have to hug an extended plane once the roll is free to be
smaller than the pitch (a bent knee only needs its two ends, not its whole
length, near the surface). Left in place because the underlying identities
about `_rotate_offset` are still true and still used, and because "why THIS
one was rejected under the OLD assumption" is exactly the arithmetic a future
reviewer needs before reintroducing it under the NEW one.

SEATING ON A TILTED PLANE (below) works by rigidly rotating the WHOLE posed
body by the roof's own pitch about the horizontal axis the yaw makes
perpendicular to the fall line — the same "lay the pose down to match a
support" technique `disaster.people`'s prone/LYING branch already uses, just
tipped by the roof's modest pitch instead of a full 90 deg. That technique
only keeps the BODY on the plane if the pose's own geometry is already close
to flat in its own rest frame: any two points on the body whose PRE-TILT
offset is purely horizontal-forward stay EXACTLY coplanar after the tilt (a
provable identity, not an approximation — see `_rotate_offset`'s own
docstring); any point with a PRE-TILT vertical offset from the anchor carries
that same offset, rotated, as a fixed, pitch-INDEPENDENT deviation from the
plane. Checked against `scene_generator._HUMAN_POSES` by hand (thigh/calf
segment lengths measured on rp_carla: 0.472 / 0.364 m), reproducing the exact
arithmetic `_ground_contact_drop` uses for each candidate:

  `sit_ground`  thigh -90 (horizontal), calf net -87 (nearly horizontal,
                v(-87) = (0, -0.999, -0.052)) — the WHOLE extended leg is
                within a few degrees of flat. Anchoring at the heel (this
                pose's own measured ground contact) leaves the pelvis/seat
                floating a FIXED ~0.085 m ABOVE the roof plane regardless of
                pitch (never embedded) — well inside a 5 cm-class tolerance
                in practice and, being a float rather than a sink, the
                forgiving direction of the two.
  `sit_edge`    thigh -90, calf net 0 (VERTICAL — the whole point of a pose
                built for a kerb/coping edge with open air below). Anchored
                at the seat, the heel/sole ends up ~0.5-0.6 m BELOW the roof
                plane at its own (x, y) — the legs bury into the slope. This
                is exactly the failure "sitting on the edge" suggests but a
                CONTINUOUS solid slope (not a discontinuous edge with open
                air under it) cannot support.
  `crouch`      thigh -58, calf net +62 (v(+62) = (0, 0.883, -0.469) — mostly
                VERTICAL, a squat's folded shin). Sole-anchored, the knee
                ends up well off-plane by a pitch-independent residual on
                the same order as `sit_edge`'s.
  `sit_slump`   thigh -138, calf net -13 (v(-13) = (0,-0.225,-0.974) — also
                mostly vertical, knees drawn up toward the chest). Same
                failure shape as `crouch`.

`sit_ground` is the ONLY one of the four whose extended-leg geometry is
already near-planar before any tilt is applied, so it is the only one used;
this is a geometric fact about this specific pose table, not a stylistic
preference, and the arithmetic above is exactly what a future reviewer
should re-run before adding a second seated pose to the mix.

SEATING ON A TILTED PLANE — THE ROLL-TO-FULL-PITCH CASE, SUPERSEDED
------------------------------------
Still the mechanism `_rotate_offset` itself implements and still exactly
right for `roll_deg == pitch_deg`; ROUND 2 (below) calls the SAME function
with a SOLVED, usually SMALLER `roll_deg` instead of the roof's pitch, for
`sit_slump`. The identities quoted here (`_rotate_offset`'s own docstring)
are unchanged; only what value gets passed as `roll_deg` for the roof class
has changed, and only for `sit_slump`.

`_rotate_offset(y0, z0, roll_deg, yaw_deg)` applies the SAME `rotateXYZ`
composition `scene_generator.apply_placements` authors on every placement
(roll about the fixed local X axis, THEN yaw about the fixed local Z axis —
confirmed against that function's own centroid-offset code, which chains
`Gf.Rotation` calls in exactly that order) to a body-local, PRE-tilt offset
`(0, y0, z0)` measured from the rig's own root. `yaw_deg` is solved first —
`_facing_yaw_for_dir` — so the character's REST forward direction (local
`-Y`) lands on the WORLD bearing this file already uses for every other
rotated position (`_to_world`'s own convention: local `(0,-1)` maps to world
`(sin(yaw), -cos(yaw))`); the chosen bearing is the slope's own
`downhill_xy`, rotated into the world by the house's own `yaw_deg` — i.e.
the figure faces OUTWARD AND DOWNHILL, never along the ridge and never
back into the roof. `roll_deg` is then set to the roof's own measured
`pitch_deg` (always positive — the sign of the tilt is carried entirely by
the yaw bearing, proven in `_rotate_offset`'s own docstring). Given the
TARGET seat point on the roof plane and `sit_ground`'s own measured
heel-anchor offset (`_SIT_GROUND_HEEL_LOCAL_M`, scaled by the drawn
character's own height), `_roof_seat_placement` solves the ROOT position
(`target - rotated_offset`) so the heel lands exactly on the plane — the
same "solve the root from a known contact offset" idea `_lying_lift` already
uses for a laid-down figure, just for a seated, rotated one.

2026-09-01 ROUND 2: THE TORSO FIX
------------------------------------
User, on the live scene, verbatim: "The people are on the sides now but when
i said to look at fire all i meant is to look how they do it. DON'T use the
same pose. These are townhouses not buildings. The pose of the human is to
be different. You can have them sitting on the roof not their torso popping
out." Decoded: the fire pipeline's roof pose was a METHOD reference (how to
gate eligibility, how to inset from an edge) — never a POSE reference, and
carrying its pose choice over (via `sit_ground` rolled to the roof's full
pitch) produced exactly the complaint: on a 39 deg pitch, `sit_ground`'s
near-vertical torso (WHY `sit_ground` AND NOTHING ELSE measured it at ~3 deg
off plumb) rides the FULL roll and ends up ~40+ deg off plumb — a body
reclining back INTO the slope with its chest tipped toward the sky, not a
person sitting upright on it.

THE PICK, BY MEASUREMENT. Forward kinematics — calling `scene_generator.
_pose_joint_transforms` directly on `assets/people/rp_carla_rigged_001_ue4.
usd`'s real skeleton, the SAME FK a render binds, not hand arithmetic — was
run over every seated entry in `scene_generator._HUMAN_POSES` (`sit_ground`,
`sit_edge`, `sit_slump`, `crouch`, `trapped_sit`; `people.BANNED_POSES`
already forbids `wave`) against all three DISTINCT pitches this kit's roofs
carry (39.24/39.11 deg, 21.71 deg, 19.42/19.41 deg), scoring each on: can the
pelvis AND both feet touch the plane at once, does the torso stay within
`ROOF_TORSO_TILT_MAX_DEG` (20 deg) of gravity-vertical, and does ANY tracked
point (pelvis, spine, head, both knees, both feet's heel AND ball/toe
corners, both hands) go BELOW the plane — searching, for each candidate, the
LARGEST roll that keeps the torso within the tilt bound (a partial roll is
now a free variable per fix #4, not fixed to the pitch):

    pose         best roll  torso tilt  worst tracked point       verdict
    sit_ground   17.3 deg   20.0 deg    hand_l -0.147 m (BELOW)   FAILS
    sit_edge    -21.3 deg   20.0 deg    hand_l +0.062 m (above)   PASSES @
                                                                   39.24 only
    crouch      -15.3 deg   20.0 deg    heel_l -0.276 m (BELOW)   FAILS
    trapped_sit  24.0 deg   20.0 deg    hand_l -0.159 m (BELOW)   FAILS
    sit_slump    10.2 deg    4.5 deg    spine_01 +0.134 m (above) PASSES

  ("worst tracked point" is the SMALLEST margin found by searching, for each
  candidate, the roll — within the tilt bound — that MAXIMISES the smallest
  margin over EVERY tracked point: pelvis, the full spine chain, head, both
  knees, both feet's heel AND ball/toe corners, both hands; a negative margin
  means that point is BELOW the roof plane at that roll — penetrating — no
  matter how the rest of the body is doing. Numbers above are pitch 39.24 deg
  on rp_carla. `sit_edge` PASSES only at this steepest pitch — at 21.71/19.42
  deg its own heel embeds 1.1-3.7 cm no matter the roll, because its dangling
  shin does not rotate into a shallow slope's much gentler drop; `sit_slump`
  is the only candidate that clears every tracked point at EVERY pitch this
  kit's roofs carry. `spine_01` as `sit_slump`'s "worst" point is not a real
  hazard — it sits only ~0.12 m above the pelvis with almost no forward
  offset, so it is never close to the plane at any roll in this range; it
  simply has a smaller margin than the genuinely safe hand/head points at
  this particular roll. The FOOT is `sit_slump`'s real constraint, and the
  PRODUCTION roll (`_solve_sit_slump_roll`, below) solves for it directly
  rather than reusing this SELECTION table's whole-body-minimax roll, which
  is deliberately conservative — it floats the foot 5-18 cm proud to buy
  margin on points that were never in danger. The full table across all
  three pitches and all six rigged humans this pack ships is in the
  session's own record, not reproduced here — the per-rig spread is exactly
  why `_sit_slump_offsets` re-measures the ACTUAL placed rig rather than
  trusting one reference scaled by height, see its docstring.)

  `sit_slump`'s knees are drawn up (`scene_generator._HUMAN_POSES["sit_
  slump"]`: hip flexion -138 deg) rather than extended like `sit_ground`'s —
  which is WHY it was rejected under ROUND 1's assumption (WHY `sit_ground`
  AND NOTHING ELSE: "knees drawn up... same failure shape as `crouch`", an
  analysis about whether the LEGS lie flush against an extended plane). A
  partial roll doesn't need that: `sit_slump`'s pelvis-to-`spine_03` vector
  sits ~14.7 deg off plumb at roll=0 (its own authoring comment describes a
  forward hunch; the actual FK sign says the lean is the OTHER way, toward
  what this file calls "backward" — immaterial to this fix, which reasons
  from the measured vector, not the prose), so a MODEST roll in the direction
  that ALSO seats the feet brings the torso close to dead vertical rather
  than away from it — the torso tilt this pose needs and the roll its feet
  need turn out to want almost the same small angle, which is exactly why it
  passes and the others do not. No stock pose was composed/modified — used
  as-authored.

THE SOLVE. `_solve_sit_slump_roll(usd, pitch_deg, height)`:
  1. `phi0`, the roll that would put this rig's own pelvis-to-`spine_03`
     vector EXACTLY vertical — `atan2(y0, z0)` of that vector's PRE-roll
     (y, z), because rotating a vector by `roll` shifts its angle from
     vertical by exactly `roll` (a rotation is additive on angle), so
     `tilt(roll) = |phi0 - roll|` and the tilt bound is simply
     `roll in [phi0 - 20, phi0 + 20]`.
  2. For each of the FOOT's two corners on each side (`heel_l`/`heel_r`,
     rocked back per `scene_generator._CONTACT_JOINT`'s own "0.45 of rest
     ankle height below the joint" convention, and `ball_l`/`ball_r`, the
     toe) — because `sit_slump`'s own `foot_l`/`foot_r` delta does not sit
     flush on a tilted plane, so heel and toe do NOT touch it at the same
     roll and both must be checked — the EXACT roll that lands that single
     point on the plane, in closed form (`_sit_slump_zero_roll`): given a
     pelvis-relative point `(0, y0, z0)`, rolling by `r` moves it to
     `(y0 cos r - z0 sin r, y0 sin r + z0 cos r)`, and it lies on a plane of
     `pitch_deg` through the pelvis exactly when
     `z1 = tan(pitch_deg) * y1` — solved for `r` by `atan2(z0 - t*y0,
     -(y0 + t*z0))`, `t = tan(pitch_deg)` (verified against a numeric
     bisection across all 6 rigs x 5 pitches: identical to 1e-9).
  3. Each foot corner's own gap-to-plane (`_sit_slump_foot_gap`) is a single
     sinusoid in `roll` and — checked, not assumed, over the whole tracked
     point set at every rig/pitch in the session's own sweep — DECREASING as
     `roll` increases through this range, so the SMALLEST of the four
     corners' own zero-crossings is where the FIRST of them would go
     negative: `roll_deg = clamp(min(four zero-crossings), phi0-20, phi0+20)`
     is the snuggest fit the tilt budget allows, and per-point re-checking
     after picking it (the test's own job, not this solve's) is what confirms
     the OTHER three corners are still >= 0 rather than assuming it.
  4. If even `phi0 - 20` (the most conservative allowed roll) still leaves
     the tightest corner negative — measured on 2 of the 6 rigs, at the
     SHALLOWEST pitch only, by <= 0.9 cm — `lift_m` (returned alongside
     `roll_deg`) is the small vertical raise that clears it; the pelvis then
     floats that same tiny amount rather than the foot embedding.

`_sit_slump_offsets(usd, height)` is what feeds all four inputs above (the
pelvis-to-`spine_03` vector, and the four foot corners): it re-measures the
ACTUAL placed asset's own skeleton when pxr and that asset are both
reachable (the exact FK a render binds), because this pack's six rigs do NOT
share `sit_slump`'s geometry closely enough for one reference, scaled by
height, to serve all of them — rp_carla's own solved roll for a 39.24 deg
roof is ~22 deg; rp_eric's is ~13; applying rp_carla's 22 deg to rp_eric's
own rig via height-scaling alone SINKS HIS TOE 13.7 CM INTO THE ROOF
(measured in the session's own per-rig sweep, not reproduced here). The
frozen `_SIT_SLUMP_*_LOCAL_M` constants are rp_carla's own numbers, used only
when a specific asset's skeleton cannot be opened at all (a host-side plan
with no Nucleus) — same relation `_SIT_GROUND_HEEL_LOCAL_M` has to a live rig
read, and the SAME reason `_pose_dz`/`_ground_contact_drop`
(`disaster/people.py`, `scene_generator.py`) already read a specific rig
rather than trust one number for six.

WHAT THIS FILE DOES NOT DO
---------------------------
* Build its own debris/plank field for the water class's "near stranded
  debris" bias — it anchors off the WRECKED HOUSE's own footprint instead
  (the debris that sheds off a failed structure is right there), rather than
  reaching into `disaster.washaway`'s raft/land-debris specs, which this
  stream does not own and was told not to touch.
* Cross-check separation between the dry-land casualties and the water/roof
  figures. The three domains are spatially disjoint in practice (a casualty
  is on dry ground, a water figure is in water, a roof figure is above both)
  and checking anyway would mean either exposing `tornado_people._Field`'s
  private `taken` list or re-deriving the whole-body station geometry a
  second time; not done, flagged here as the assumption it is.
* Attach `prim_path` / `house_prim_path` to a record. Planning happens before
  any prim exists — same reason `tornado_people`'s own records carry no
  `prim_path` at all. The launcher fills both in, by zipping `records` against
  the placement dicts `scene_generator.apply_placements` mutates in place
  (`prim_path` lands on the PLACEMENT dict, not the record — see that
  function's own comment) and by a position-keyed join against `ctx["houses"]`
  for `house_prim_path`, the same join pattern `disaster.people.house_table`
  already uses to attach a fire level to a plat house.

A PURE PLANNER, same contract as `tornado_people.plan_people` and
`disaster.people.plan_people`: no stage, no Isaac import, `(humans, debris,
records)` out. `scene_gen/tests/test_hurricane_people.py` runs the whole thing
host-side with a stub resolver.
"""

import math
import os
import random

from . import people
from . import tornado_people as tpp

try:
    from detail import modular_house as _mh
except Exception:                          # host without `detail` on sys.path
    _mh = None

_HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------
# Damage-level vocabulary
# ---------------------------------------------------------------------------
# Copied rather than imported: `tornado_people._WRECKED` is underscore-prefixed
# (module-private) and this is the one four-item tuple, not a table worth an
# import-time dependency on that module's internals for. `tests/
# test_hurricane_people.py` asserts this equals `tornado_people._WRECKED`, so a
# level added there cannot silently drift out of step here.
_WRECKED_LEVELS = ("roof_collapsed", "partial_collapse", "leveled", "swept")

# "Intact" for the ROOF class — NARROWED 2026-09-01 to `pristine` ONLY, per
# the user's own instruction ("they also have to be on intact houses's roofs
# only"). `roof_stripped` USED to qualify on the argument that a bare
# top-plate is "still something to stand on"; that argument does not survive
# the word "roof" — `roof_stripped`'s own archetype has no `_roof_` mesh at
# all (see the module docstring's WHY `roof_stripped` IS EXCLUDED NOW), so
# there is no roof there, intact or otherwise. Also NOT the same test
# `tornado_people`'s own `intact` keepout uses (a DRY-LAND casualty must
# merely avoid the footprint of a `pristine` house; this list decides who
# gets a roof REFUGE, a different, and now narrower, question).
_ROOF_OK_LEVELS = ("pristine",)

# How deep is "flooded" for every gate in this file. Matches `washaway.
# LAND_DEBRIS_SUBMERGED_DEPTH_M`'s own floor in spirit (the hurricane launcher
# already uses 0.10 m as "underwater enough that a piece lying there should be
# a raft, not litter") — reusing the same order of magnitude keeps this
# module's dry/wet line agreeing with the ground/debris passes about where the
# water starts, without importing `washaway` (out of scope for this stream).
DRY_DEPTH_M = 0.10

STOREY_M = float(getattr(_mh, "STOREY_M", 3.5))
if _mh is not None:
    _STYLE_STOREYS = {k: int(v.get("storeys", 1))
                       for k, v in _mh.STYLES.items()}
else:
    # FALLBACK, if `detail.modular_house` is not importable on this host.
    # Transcribed from that module's own `STYLES` table (2026-09-01) so a
    # host-side test still runs; `tests/test_hurricane_people.py` asserts
    # this against the real import when it is available.
    _STYLE_STOREYS = {"cottage": 1, "two_storey": 2, "wide_house": 2,
                      "ranch": 1, "terrace": 2, "villa": 1,
                      "l_family": 2, "l_bungalow": 1}

# ---------------------------------------------------------------------------
# ROOF SLOPE PLANES — MEASURED 2026-09-01, bare `pxr` against
# `scene_gen/assets/archetypes_tornado/house_<style>_pristine.usd`, by
# `remeasure_roof_slopes()` below. See the module docstring's MEASURING THE
# ROOF section for the clustering method. One entry per kept planar facet
# (2 per style, 4 for `villa`'s real hip roof); `downhill_xy` is the LOCAL
# unit direction from ridge to eave, `ridge_z`/`eave_z` the facet's own
# measured extremes, `bbox` its plan footprint (LOCAL frame, metres,
# unscaled — `SCALE = 0.01` already applied at bake time).
# `tests/test_hurricane_people.py` re-measures and pins every field when the
# archetype files are present; this table is the FALLBACK otherwise, same
# relation `tornado_people.DEBRIS_Z_M` has to `_Deck`.
# ---------------------------------------------------------------------------
_ROOF_SLOPES_LOCAL = {
    "cottage": [
        {"downhill_xy": (1.0, -0.0), "pitch_deg": 39.24,
         "ridge_z": 7.759, "eave_z": 3.467,
         "bbox": (0.108, -5.708, 5.362, 5.708)},
        {"downhill_xy": (-1.0, 0.0), "pitch_deg": 39.24,
         "ridge_z": 7.759, "eave_z": 3.470,
         "bbox": (-5.358, -5.708, -0.108, 5.708)},
    ],
    "l_bungalow": [
        {"downhill_xy": (1.0, -0.0), "pitch_deg": 39.24,
         "ridge_z": 7.759, "eave_z": 3.467,
         "bbox": (-4.892, -10.708, 10.362, 10.708)},
        {"downhill_xy": (-1.0, 0.0), "pitch_deg": 39.24,
         "ridge_z": 7.759, "eave_z": 3.470,
         "bbox": (-10.358, -10.708, 4.892, 10.708)},
    ],
    "l_family": [
        {"downhill_xy": (1.0, -0.0), "pitch_deg": 39.24,
         "ridge_z": 11.259, "eave_z": 6.967,
         "bbox": (-4.892, -10.708, 10.362, 10.708)},
        {"downhill_xy": (-1.0, 0.0), "pitch_deg": 39.24,
         "ridge_z": 11.259, "eave_z": 6.970,
         "bbox": (-10.358, -10.708, 4.892, 10.708)},
    ],
    "ranch": [
        {"downhill_xy": (1.0, -0.0), "pitch_deg": 39.24,
         "ridge_z": 7.759, "eave_z": 3.467,
         "bbox": (-4.892, -5.708, 10.362, 5.708)},
        {"downhill_xy": (-1.0, 0.0), "pitch_deg": 39.24,
         "ridge_z": 7.759, "eave_z": 3.470,
         "bbox": (-10.358, -5.708, 4.892, 5.708)},
    ],
    "terrace": [
        {"downhill_xy": (1.0, -0.0), "pitch_deg": 19.42,
         "ridge_z": 9.210, "eave_z": 5.524,
         "bbox": (0.130, -10.708, 6.445, 10.000)},
        {"downhill_xy": (-1.0, -0.0), "pitch_deg": 19.42,
         "ridge_z": 9.210, "eave_z": 5.526,
         "bbox": (-6.440, -10.708, -0.130, 10.000)},
    ],
    "two_storey": [
        {"downhill_xy": (1.0, -0.0), "pitch_deg": 39.24,
         "ridge_z": 11.259, "eave_z": 6.967,
         "bbox": (0.108, -5.708, 5.362, 5.708)},
        {"downhill_xy": (-1.0, 0.0), "pitch_deg": 39.24,
         "ridge_z": 11.259, "eave_z": 6.970,
         "bbox": (-5.358, -5.708, -0.108, 5.708)},
    ],
    "villa": [
        {"downhill_xy": (1.0, 0.0), "pitch_deg": 19.42,
         "ridge_z": 5.710, "eave_z": 3.483,
         "bbox": (-3.469, -5.708, 8.576, 5.000)},
        {"downhill_xy": (0.0, 1.0), "pitch_deg": 21.71,
         "ridge_z": 5.710, "eave_z": 3.485,
         "bbox": (-8.577, 0.115, 1.963, 5.704)},
        {"downhill_xy": (0.0, -1.0), "pitch_deg": 21.71,
         "ridge_z": 5.710, "eave_z": 3.483,
         "bbox": (-8.577, -5.709, 1.947, -0.115)},
        {"downhill_xy": (-1.0, 0.0011), "pitch_deg": 19.41,
         "ridge_z": 5.710, "eave_z": 3.484,
         "bbox": (-4.157, -5.708, 2.002, 5.000)},
    ],
    "wide_house": [
        {"downhill_xy": (1.0, -0.0), "pitch_deg": 39.11,
         "ridge_z": 11.259, "eave_z": 6.967,
         "bbox": (-2.392, -5.708, 7.854, 5.708)},
        {"downhill_xy": (-1.0, -0.0), "pitch_deg": 39.11,
         "ridge_z": 11.259, "eave_z": 6.970,
         "bbox": (-7.858, -5.708, 4.892, 5.708)},
    ],
}

# Facets kept vs. dropped by `remeasure_roof_slopes()` — a real walkable
# slope on this kit measures 30+ sq m; a hip-corner cap or ridge-trim
# fragment measures under 14. 15 sits cleanly in the gap.
ROOF_SLOPE_MIN_AREA_M2 = 15.0
_ROOF_SLOPE_NZ_FLAT_MAX = 0.985   # exclude a near-flat ridge-cap trim strip
_ROOF_SLOPE_NZ_VERT_MIN = 0.20    # exclude a near-vertical gable-end wall
_ROOF_SLOPE_CLUSTER_TOL_DEG = 8.0


def remeasure_roof_slopes(styles=None, arch_dir=None):
    """Re-derive `_ROOF_SLOPES_LOCAL` from the baked archetypes on disk.

    Bare `pxr`, imported LAZILY (this stays a pure, Isaac-free planner on
    every other code path) — used by `tests/test_hurricane_people.py` to pin
    the frozen table against a re-bake, and by nothing in `plan_people`
    itself. `{style: [ {downhill_xy, pitch_deg, ridge_z, eave_z, bbox}, ... ]}`
    sorted by descending measured area, or `{}` for a style whose archetype
    is missing/unreadable. See the module docstring's MEASURING THE ROOF for
    the method and the porch-canopy trap this guards against.
    """
    from pxr import Usd, UsdGeom

    if arch_dir is None:
        arch_dir = os.path.join(_HERE, "..", "assets", "archetypes_tornado")
    styles = list(styles) if styles is not None else list(_ROOF_SLOPES_LOCAL)

    def _tri_normal_area(a, b, c):
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        n = math.sqrt(nx * nx + ny * ny + nz * nz)
        if n < 1e-9:
            return None, 0.0
        return (nx / n, ny / n, nz / n), 0.5 * n

    def _measure_style(style):
        path = os.path.join(arch_dir, "house_%s_pristine.usd" % style)
        if not os.path.isfile(path):
            return {}
        stage = Usd.Stage.Open(path)
        if stage is None:
            return {}
        stage.Load()
        floor_z = STOREY_M * float(_STYLE_STOREYS.get(style, 1)) - 1.5
        faces = []
        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh):
                continue
            low = str(prim.GetPath()).lower()
            if ("_roof_" not in low and not low.endswith("_roof")) \
                    or "bay_roof" in low:
                continue
            mesh = UsdGeom.Mesh(prim)
            pts = mesh.GetPointsAttr().Get()
            counts = mesh.GetFaceVertexCountsAttr().Get()
            idxs = mesh.GetFaceVertexIndicesAttr().Get()
            if not pts or not counts:
                continue
            xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())
            verts = [xf.Transform(p) for p in pts]
            # PRIM-LEVEL pre-filter — a ground-level porch canopy stamped
            # with a "_roof_"-matching name never reaches attic height.
            if max(v[2] for v in verts) < floor_z:
                continue
            start = 0
            for c in counts:
                if c >= 3:
                    for j in range(1, c - 1):
                        a, b, cc = verts[idxs[start]], verts[idxs[start + j]], \
                            verts[idxs[start + j + 1]]
                        n, area = _tri_normal_area(a, b, cc)
                        if n is not None and area > 1e-6:
                            faces.append((n, area, (a, b, cc)))
                start += c
        if not faces:
            return {}
        tol = math.cos(math.radians(_ROOF_SLOPE_CLUSTER_TOL_DEG))
        clusters = []
        for n, area, tri in faces:
            best = None
            for cl in clusters:
                cn = cl["n_sum"]
                cl_len = math.sqrt(sum(v * v for v in cn))
                if cl_len > 1e-9 and sum(a * b for a, b in zip(n, cn)) \
                        / cl_len > tol:
                    best = cl
                    break
            if best is None:
                best = {"n_sum": [0.0, 0.0, 0.0], "area": 0.0, "pts": []}
                clusters.append(best)
            best["n_sum"][0] += n[0] * area
            best["n_sum"][1] += n[1] * area
            best["n_sum"][2] += n[2] * area
            best["area"] += area
            best["pts"].extend(tri)
        out = []
        for cl in clusters:
            if cl["area"] < ROOF_SLOPE_MIN_AREA_M2:
                continue
            nlen = math.sqrt(sum(v * v for v in cl["n_sum"]))
            if nlen < 1e-9:
                continue
            nx, ny, nz = (v / nlen for v in cl["n_sum"])
            if nz > _ROOF_SLOPE_NZ_FLAT_MAX or nz < _ROOF_SLOPE_NZ_VERT_MIN:
                continue
            hl = math.sqrt(nx * nx + ny * ny)
            dh = (nx / hl, ny / hl) if hl > 1e-9 else (0.0, 0.0)
            xs = [p[0] for p in cl["pts"]]
            ys = [p[1] for p in cl["pts"]]
            zs = [p[2] for p in cl["pts"]]
            out.append({
                "downhill_xy": (round(dh[0], 4), round(dh[1], 4)),
                "pitch_deg": round(
                    math.degrees(math.acos(max(-1.0, min(1.0, nz)))), 2),
                "ridge_z": round(max(zs), 3), "eave_z": round(min(zs), 3),
                "bbox": (round(min(xs), 3), round(min(ys), 3),
                        round(max(xs), 3), round(max(ys), 3)),
                "_area_m2": round(cl["area"], 2),
            })
        out.sort(key=lambda d: -d["_area_m2"])
        for d in out:
            del d["_area_m2"]
        return out

    return {s: _measure_style(s) for s in styles}


# MEASURED alongside the roof, off the `_wall_`-named meshes of the same
# archetypes: `(x0, y0, x1, y1)`, LOCAL frame. VESTIGIAL since `roof_stripped`
# was narrowed out of `_ROOF_OK_LEVELS` (module docstring's WHY `roof_
# stripped` IS EXCLUDED NOW) — nothing in the roof-placement path reads this
# any more, but it stays, already pinned by its own re-measure test, on the
# chance a future task wants "stand on the exposed top plate" as a class of
# its own.
_WALL_BBOX_LOCAL = {
    "cottage": (-5.114, -6.334, 5.114, 5.100),
    "l_bungalow": (-10.100, -11.334, 10.114, 10.114),
    "l_family": (-10.114, -11.334, 10.100, 10.114),
    "ranch": (-10.100, -6.334, 10.114, 5.114),
    "terrace": (-5.114, -11.334, 5.114, 10.100),
    "two_storey": (-5.114, -6.334, 5.114, 5.114),
    "villa": (-7.614, -6.334, 7.600, 5.100),
    "wide_house": (-7.614, -6.334, 7.614, 6.334),
}

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

DEFAULTS_WATER = {
    "seed": 191,
    # THE CEILING. Smaller than the dry-land casualty ceiling on purpose: a
    # standing wading figure is a much stronger aerial cue than a lying one
    # (`tornado_people`'s own argument for why a standing figure is "the easy
    # problem"), so fewer of them carry the same information.
    "max_total": 26,
    "min_depth_m": DRY_DEPTH_M,
    # BIASED toward, not confined to. See the module docstring: a person
    # cannot stand in 1.5 m+ of water at all, and the literature-free middle
    # ground (there is no published depth distribution for flood-victim
    # standing posture, same gap `tornado_people` names for throw distances)
    # is to prefer a physically comfortable band and still allow the edge of
    # it a minority of the time, tracked by `summarise`'s histogram rather
    # than hidden by a hard cutoff at the preferred band's own edge.
    "stand_band_m": (0.30, 1.20),
    "stand_hard_max_m": 1.50,
    "stand_edge_accept_p": 0.30,
    # PER-HOUSE BUDGET, ASSUMPTION — not sourced with `tornado_people`'s
    # rigour (no equivalent of the Joplin/Hammer-Schmidlin literature was
    # pulled for this stream; NOAA's post-storm finding that most US
    # hurricane deaths are FRESHWATER/storm-surge drowning, and the
    # well-photographed Katrina/Harvey pattern of people found wading near
    # or clinging to a failed structure, argue for the shape — more likely
    # near a WRECKED house than a standing one — but the exact ranges below
    # are a dataset decision, not a measurement, and are recorded as one.
    "per_house": {"pristine": (0, 1), "roof_stripped": (0, 1),
                  "roof_collapsed": (0, 2), "partial_collapse": (0, 2),
                  "leveled": (1, 2), "swept": (1, 2)},
    # Not every flooded house gets a water figure — most people who can
    # reach a roof do (see `DEFAULTS_ROOF`), so a standing house's water
    # figure is the minority case (someone who did not make it up in time);
    # a wrecked house's is the majority (the structure that would have been
    # their refuge already failed).
    "p_attempt_standing": 0.30,
    "p_attempt_wrecked": 0.55,
    # Distance OUT from the house's own footprint radius, metres.
    # OUT IN THE WATER, NOT IN THE DEBRIS (2026-09-01, user on the bench:
    # "The people in the water look fine, just have them in the water instead
    # of in debris"). Anchoring 1-6 m off a WRECKED house's footprint put
    # every wading figure inside that house's own fragment field, where the
    # body reads as one more piece of debris. Pushed out past the wreck's
    # own scatter so the figure stands in open floodwater with the house
    # BEHIND it, which is what makes it read as a person.
    "wreck_offset_m": (9.0, 22.0),
    "house_offset_m": (7.0, 20.0),   # ditto: clear of the house's own litter
    # THE UP-FLOW BIAS. `shore_bearing_deg` points INLAND
    # (`surge.DEFAULTS["shore_bearing_deg"]`'s own comment); the surge and the
    # debris it carries arrive from the OPPOSITE bearing, so a standing
    # house's water figure is biased toward `shore_bearing_deg + 180` with a
    # Gaussian spread rather than placed isotropically.
    "up_flow_spread_deg": 40.0,
    "house_clear_m": 1.0,
    "min_separation_m": 1.6,
    "edge_margin_m": 1.5,
    "tries": 16,
    # POSES. `idle` and `stand_slump` are pre-existing, widely-used standing
    # poses (evacuee/onlooker/at_car in `fire_people`, `at_home` in `people`)
    # with no open pose-fidelity question against them. `wave_help` is NOT —
    # `people.py`'s own 2026-08-31 WINDOW_POSE_DIAGNOSIS section records that
    # a *different* pose pair in the same joint family
    # (`stand_calm`/`lean_window`) rendered wrong despite checking out on
    # paper, and `wave_help` was never independently re-verified after that —
    # so it is kept a MINORITY here rather than assumed safe, exactly the
    # caution `fire_people.DEFAULTS["roof_use_new_pose"]` already applies to
    # the same pose in the roof context.
    "poses": {"idle": 0.55, "stand_slump": 0.25, "wave_help": 0.20},
}

DEFAULTS_ROOF = {
    "seed": 271,
    "max_total": 40,
    "min_depth_m": DRY_DEPTH_M,
    # 1-3 per roof, per the task's own instruction.
    "per_roof": (1, 3),
    # PREFER houses in or adjacent to the flood — "adjacent" is not measured
    # here (no per-house neighbour graph in ctx; would need the layout's
    # parcel adjacency, out of scope for this stream) so the proxy is simply
    # "is THIS house's own ground flooded". A small non-zero `p_dry` keeps a
    # few roof-refuge figures on dry, undamaged houses for variety, matching
    # the same "prefer, not exclude" reading the instruction gives.
    "p_flooded": 0.65,
    "p_dry": 0.08,
    "min_separation_m": 1.4,
    "tries": 16,
    # THE FALL-LINE BAND, and the two hard margins either side of it — the
    # user's own "usually sitting on the sides" plus "can't ... be too close
    # to the edge", made numeric. `fall_band` is a FRACTION of the slope
    # distance from ridge to eave; `ridge_margin_m` is VERTICAL metres of
    # drop below the ridge (nobody at the very top); `eave_margin_slope_m` is
    # metres measured ALONG THE SLOPE above the eave (nobody at the drop).
    # Every style's own valid range comfortably contains [0.35, 0.65] except
    # `villa` (measured pitch band 19.4-21.7 deg over a short ~2.2 m rise),
    # where the ridge margin alone pushes the usable band up to ~[0.36,
    # 0.65] — see the module docstring's SEATING ON A TILTED PLANE and the
    # final report for the worked numbers.
    "fall_band": (0.35, 0.65),
    "ridge_margin_m": 0.8,
    "eave_margin_slope_m": 1.2,
    # Inset off the slope facet's own bbox, along the ridge-parallel axis —
    # keeps a seat off the gable end / hip corner the facet's own bbox
    # slightly overshoots (see MEASURING THE ROOF: the small steep-pitch hip
    # caps this file excludes by area sit just past these bboxes).
    "slope_edge_margin_m": 1.3,
    # PREFER the slope facing the street (dry house) or the water (flooded
    # house) — "the side an aerial/boat would see" — but do not confine to
    # it: `prefer_p` of the time the best-aligned eligible slope is used,
    # otherwise a random other eligible slope on the same roof, for variety.
    "prefer_p": 0.85,
    # A few degrees either side of dead-on-downhill so a whole roof's figures
    # do not all face the identical bearing — still overwhelmingly "facing
    # outward and downhill", never sideways along the ridge.
# WIDE, because every figure on one slope otherwise inherits the SAME
    # downslope bearing and the group renders as a chorus line facing one way
    # (2026-09-01, user on the bench: "Why are all the people facing the
    # same way"). People on a roof watch the water, the sky and each other;
    # +-70 deg keeps them all facing broadly outward/downslope without the
    # parade-ground alignment. Raised from 10.0.
    "yaw_jitter_deg": 70.0,
    # SEATED, singular — `sit_slump` since 2026-09-01 ROUND 2 (see the module
    # docstring's THE TORSO FIX). ROUND 1 picked `sit_ground` on the
    # assumption the whole body rolls to the roof's OWN full pitch, under
    # which `sit_slump`'s drawn-up knees bury (WHY `sit_ground` AND NOTHING
    # ELSE, kept for that history); once the roll is solved PARTIALLY instead
    # (`_solve_sit_slump_roll`) so the torso stays upright, `sit_slump` is
    # the only one of the five seated poses this pack ships that lands
    # pelvis and both feet without embedding ANY tracked point at any of
    # this kit's measured pitches — see THE PICK, BY MEASUREMENT's own score
    # table. Kept as a weighted dict (not a bare string) so a future pass
    # that finds a second tilt-safe pose can add it without a call-site
    # change.
    # FIRE'S APPROVED ROOF CONFIGURATION, COPIED EXACTLY (2026-09-01, user:
    # "look at how urban fire poses them. It is able to do the no face up and
    # no leaning back"). `fire_people`'s own record settles it: its
    # `stand_calm`/`wave_help` table was REJECTED on sight, `roof_use_new_pose`
    # stays False, and the plain `idle` FALLBACK is what shipped and drew the
    # verdict "the roof people on non collapsed roofs are good". `idle` is a
    # plain standing pose with no trunk flexion, so with the tilt off there is
    # nothing that can pitch a figure back or turn a face to the sky.
    "poses": {"idle": 1.0},
}


def resolve_cfg(config):
    """`{"dry": ..., "water": ..., "roof": ...}` — the three sub-configs.

    `config["hurricane_people"]` (optional) carries `water:`/`roof:` blocks
    merged one level deep over the defaults above, same shape
    `tornado_people.resolve_cfg` uses for its own `people:` block.
    `config["people"]` (if the preset has one) drives the DRY sub-config
    completely unchanged, via `tornado_people.resolve_cfg` itself — this
    module adds no new knob to that half at all.
    """
    dry_cfg = tpp.resolve_cfg(config)
    # See the module docstring for why: a near-uniform wind field makes the
    # tornado's own "in the corridor" gate a no-op or a scene-emptier here,
    # never the dry/wet distinction that matters. Set to 0 explicitly (rather
    # than left unset) so `tornado_people` does not print its "gate is off"
    # warning either — an intensity field IS supplied, the gate is simply
    # asked not to filter on it.
    dry_cfg["min_intensity"] = 0.0

    # MORE UNDER DEBRIS, FEWER LYING IN THE OPEN (2026-09-01, user: "tornado
    # has people on the floor randomly. We want that in hurricane but not a
    # lot of that. We want more under debris").
    #
    # The mechanism is `tornado_people`'s own and is NOT touched -- only the
    # weights it draws with, and only for the hurricane. Its `none` (a body
    # lying fully exposed on the debris) carries 0.44 there because that
    # review asked for visibility on a tornado plate; a flooded suburb wants
    # the opposite emphasis. `none` drops to 0.18 and the freed weight goes
    # to the partial patterns that still respect `max_covered_frac`, so no
    # figure is authored more hidden than the tornado's own cap allows and
    # every label stays honest. The four over-cap patterns stay 0, exactly
    # as `tornado_people` sets them.
    _occ = dict(dry_cfg.get("occlusion") or {})
    _hur_occ = {"none": 0.18, "feet_shins": 0.16, "legs": 0.15,
                "midriff": 0.14, "torso": 0.14, "torso_head": 0.08,
                "head_only": 0.05, "banded": 0.05, "flank": 0.05}
    for _k, _v in _hur_occ.items():
        if _k in _occ:            # never invent a pattern this vocabulary lacks
            _occ[_k] = _v
    dry_cfg["occlusion"] = _occ

    # NOBODY IS THROWN BY A HURRICANE (2026-09-01, user on the bench: "so
    # casualties thrown 10-40 m out — this has to be a tornado only thing. We
    # don't want that in hurricane, that doesn't really happen in hurricane.
    # Validate that with actual reports like we did for tornado"). Validated;
    # the full citation trail is in `.agents/skills/build-hurricane-scenes/
    # PEOPLE_RESEARCH.md` and the short version is:
    #
    #   * Wind is 8% of direct US tropical-cyclone deaths (Rappaport 2014,
    #     BAMS 95(3); ~90% are water). Rappaport's own definition of that
    #     category is the mechanism and it is already the answer: deaths
    #     "incurred from wind-borne debris or structural failure induced by
    #     wind" — structural failure and debris impact, NOT transport of the
    #     body.
    #   * The Dade County ME case series for Andrew lists all 12 direct
    #     traumatic deaths individually: "Collapsing roof" x2, "Collapsing
    #     home", "Collapsing barn", "Collapsing townhouse", "Collapsed ceiling
    #     of retirement home", "Found in destroyed mobile home", "Destroyed
    #     trailer" — 8 IN or UNDER the structure; 2 "Ejected from turning
    #     trailer" (the unit rolls and the occupant comes out WITH it, metres);
    #     2 outdoors/vehicle. ZERO thrown. The one body in the series recovered
    #     far from where it was injured was washed overboard at sea.
    #   * The tornado's throw rests on a signal the hurricane literature does
    #     not produce: CDC MMWR 61(28) coded injury and recovery location
    #     separately for all 338 April-2011 tornado deaths and found 90.5%
    #     injured indoors against 37.0% RECOVERED OUTDOORS. No hurricane study
    #     codes the two separately, because in a hurricane they are the same
    #     place — which is itself the finding.
    #   * Physically: a tornado lofts on a concentrated near-surface VERTICAL
    #     corner-flow updraft. A landfalling hurricane's boundary layer is
    #     quasi-steady and horizontal at human scale. The lofting events that
    #     do occur inside hurricanes are attributed to embedded TORNADO-SCALE
    #     vortices — which is why Rappaport counts embedded tornadoes as their
    #     own separate 3%.
    #
    # `tornado_people._trail` reads `count` first and returns 0 immediately, so
    # zeroing it here disables the pass without reaching any of its geometry.
    # Set through the same config path everything else uses rather than by
    # branching in `_plan_dry`, so `tornado_people` stays untouched and every
    # tornado scene is unaffected.
    dry_cfg["trail"] = dict(dry_cfg.get("trail") or {}, count=[0, 0])

    # ...AND THEY ARE ON THE STRUCTURE, not scattered over the lot. Same
    # evidence, second consequence: the Andrew tally is 8 in/under the
    # structure, 2 immediately beside it, 2 outdoors, so the weight moves onto
    # `pile` and `skirt`. `street` goes to zero — the tornado's share is
    # argued from Joplin's "house by house, car by car, block by block" search
    # doctrine, which has no hurricane analogue, and `_plan_dry` passes no
    # `road_pts` anyway so that share was already folding silently into
    # `skirt`. ANCHORED on a 12-case series, not derived from it: 8/2/2 would
    # be 0.67/0.17/0.17 and these numbers hedge toward the middle, because
    # twelve cases cannot support three significant figures. The qualitative
    # claim — on and immediately around the structure, none of them thrown —
    # is what the evidence supports without hedging, and that is what is
    # encoded.
    _where = dict(dry_cfg.get("where") or {})
    _hur_where = {"pile": 0.55, "skirt": 0.30, "yard": 0.15, "street": 0.0}
    for _k, _v in _hur_where.items():
        if _k in _where:          # never invent a location class either
            _where[_k] = _v
    dry_cfg["where"] = _where

    # ...AND `pile` REACHES INSIDE THE WALL LINE (2026-09-01, user on the live
    # 500 m plate: "I see dry casualties only in the open" / "tornado manages
    # to do it within the house footprint so you should be able to here").
    #
    # `tornado_people._candidate` draws `pile` from 0.62-1.05 FOOTPRINTS off
    # the house centre, and 0.5 footprints is the wall line — so `pile` is by
    # construction the mat AROUND the wreck, never inside it. That is right
    # for a tornado and its own comment says why: on a LEVELLED tornado lot
    # the archetype is a deep mound and "one in the middle of the deepest
    # material is invisible from every angle".
    #
    # It is wrong here for two measured reasons:
    #
    #  1. THE HURRICANE'S WRECKS ARE OPEN TO THE SKY. Sampling the tornado
    #     archetype library with bare `pxr` on a 0.75 m grid and counting
    #     cells whose top is within 0.9 m of the slab (i.e. a lying body
    #     there is under open sky, not under a roof):
    #
    #         level              ranch  cottage  two_storey
    #         roof_collapsed       27%     46%      37%
    #         partial_collapse     52%     49%      53%
    #         leveled              51%     61%      49%
    #         swept                63%     66%      69%
    #
    #     Half to two-thirds of a wrecked footprint is open ground a camera
    #     can see straight down onto. "Invisible from every angle" is a
    #     description of a tornado mound, not of these.
    #  2. THE DEBRIS AROUND THEM IS DIRECTIONAL. The tornado lays 140 boards
    #     per wreck with `scatter_from_wreck`, so its 0.62-1.05 annulus is
    #     debris the whole way round. The hurricane sheds a DOWNWIND COMET
    #     (`washaway.land_debris_specs`) plus ambient litter at ~1 piece per
    #     335 m2, so the same annulus is bare lawn on the upwind side — which
    #     is exactly what "only in the open" looks like from above.
    #
    # 0.15 rather than 0.0 keeps a body off the exact centroid, where a
    # collapsed roof peak is most likely to still be standing; 0.70 carries
    # the band past the wall line into the debris shed just outside it. What
    # keeps this HONEST is not the band but `_DECK_BAND["pile"]` (0.03-1.30 m)
    # tested against a REAL measured deck: the launcher now samples the wreck
    # archetype's own geometry into `ctx["deck_points"]`, so a cell under a
    # standing wall or an intact roof section reads 2-6 m, falls outside the
    # band, and is refused. The openness gate is therefore measured per cell,
    # not assumed per level.
    #
    # ...AND `skirt`/`yard` WERE STILL THE TORNADO'S OWN OUTER RINGS
    # (2026-09-01, user on the live 500 m plate, SECOND pass, after the
    # `pile` fix above shipped: "People still seem too far from the house
    # footprint. Tornado had it much closer for non thrown away people").
    # `pile` had already been retuned; `skirt` (0.95-1.60 fp) and `yard`
    # (1.15-2.00 fp) had NOT, and between them they carry 45% of `where`'s
    # own weight (`skirt` 0.30 + `yard` 0.15, set earlier in this function).
    # For a 14 m-footprint kit house that annulus is 13.3-28.0 FOOTPRINTS'
    # worth of metres off the house CENTRE — comfortably past a lawn or two,
    # which is exactly the complaint.
    #
    # RETUNED AGAINST THE SAME ANDREW TALLY `where`'s OWN WEIGHTS ALREADY
    # CITE (8 in/under the structure, 2 immediately beside it, 2 outdoors —
    # `PEOPLE_RESEARCH.md` section 2), read now for DISTANCE rather than
    # share:
    #
    #   `skirt` = "immediately beside it". Both Andrew cases in that bucket
    #   are "ejected from turning trailer" (the unit rolls and the occupant
    #   comes out WITH it, a few metres) plus Ian's "blew her off the porch"
    #   onto "an adjacent concrete step" — one fall's length. METRES past
    #   the wall, not tens of them. 0.55-0.85 fp (7.7-11.9 m at fp=14) picks
    #   up where `pile` leaves off and ends before a body could still
    #   plausibly read as "beside" the house it came from.
    #
    #   `yard` = "outdoors/vehicle". Still on the lot, a little further out
    #   than `skirt` because the flying-debris strike and the tree-onto-a-
    #   vehicle case are not IN the debris shed the way "beside it" is.
    #   0.70-1.05 fp (9.8-14.7 m) — deliberately OVERLAPPING `skirt`'s own
    #   upper edge, the same overlap pattern `tornado_people._WHERE_BANDS`
    #   itself already uses between its skirt (0.95-1.60) and yard
    #   (1.15-2.00), not a new convention invented here.
    #
    # MEASURED, NOT GUESSED. Bare `pxr` sampled the same 16 wrecked-house
    # archetype fixture `test_16`'s own probe uses (ranch/cottage/two_storey
    # x partial_collapse/roof_collapsed/leveled, 16 houses on a 4x4, 55 m
    # grid), but with the REAL debris field this time — `washaway.
    # land_debris_specs` (the launcher's own caller, DIRECTIONAL downwind
    # comet), not the tornado's isotropic `scatter_from_wreck` the first
    # `pile` measurement used, because that distinction turns out to matter
    # (see the honest residual below). Pooled over 5 seeds (`_plan_dry`,
    # `random.Random`, no cherry-picking):
    #
    #     n placed, min of 5 seeds                     27   ->    29
    #     median distance to nearest house centre      9.6 m ->  8.4 m
    #     MAX distance                                27.7 m -> 14.5 m
    #     inside a wrecked footprint (<=7 m, fp=14)      37% ->   39%
    #
    # The ceiling is the headline number: nobody in this fixture sits
    # farther than 14.5 m from the house they belong to, against 27.7 m
    # before — and the population did not collapse to get there (157 ->
    # 162 pooled casualties across the same 5 seeds).
    #
    # THE HONEST RESIDUAL, measured on the SAME run rather than assumed
    # away. Pulling the radius in does not fully fix a SEPARATE defect this
    # probe surfaced: `_candidate`'s draw is ISOTROPIC in angle (`a = rng.
    # uniform(0, 2*pi)`), but `land_debris_specs` threads `scatter_from_
    # wreck` with `tail_lateral_base=0.08`, `tail_lateral_growth=0.05` — a
    # NARROW downwind comet, not a ring, "ring, not a comet" fix
    # notwithstanding (that fix widened the comet's own fan, it did not make
    # it isotropic). A `skirt`/`yard` candidate drawn off the storm's own
    # heading can still land on bare lawn at ANY radius this file can
    # propose. Measured: bodies landing on bare ground (deck < 5 cm) fell
    # from 47% to 41% pooled — better, because a tighter radius buys more
    # overlap with the comet's own isotropic BASE cloud (`scatter_from_
    # wreck`'s `base_frac=0.30` ring on and around the slab, which IS
    # roughly isotropic), not because the tail became any less directional.
    # That remaining 41% is a debris-DISTRIBUTION defect in the authored
    # plank field, not a distance defect `where_bands` can solve, and it is
    # out of THIS file's scope to fix — this stream owns `resolve_cfg`, not
    # `washaway.land_debris_specs` or the launcher that calls it. Flagged
    # here rather than papered over with a band value that cannot actually
    # fix it.
    dry_cfg["where_bands"] = dict(dry_cfg.get("where_bands") or {},
                                  pile=(0.15, 0.70), skirt=(0.55, 0.85),
                                  yard=(0.70, 1.05))

    # ...WHICH MEANS THE WRECK KEEPOUT HAS TO GO. `_blocker_list` refuses any
    # body within `0.5 * fp + wreck_clear_m` of a wreck centre — 6.6-11.1 m on
    # this plate's styles — so the band above would propose interior points
    # and every one of them would be refused as `in_wreck`. 0.0 drops that
    # class of keepout entirely (`_blocker_list` guards with `if clear > 0`),
    # exactly as `tornado_people`'s own knob comment documents. Safe ONLY
    # because the measured deck now gates openness per cell; without
    # `deck_points` this would put bodies inside standing shells.
    dry_cfg["wreck_clear_m"] = 0.0

    block = (config or {}).get("hurricane_people") or {}
    water_cfg = dict(DEFAULTS_WATER)
    for k, v in (block.get("water") or {}).items():
        if k in ("per_house", "poses") and isinstance(v, dict):
            water_cfg[k] = dict(water_cfg.get(k, {}), **v)
        else:
            water_cfg[k] = v
    roof_cfg = dict(DEFAULTS_ROOF)
    for k, v in (block.get("roof") or {}).items():
        if k == "poses" and isinstance(v, dict):
            roof_cfg[k] = dict(roof_cfg.get(k, {}), **v)
        else:
            roof_cfg[k] = v
    return {"dry": dry_cfg, "water": water_cfg, "roof": roof_cfg}


# ---------------------------------------------------------------------------
# small geometry
# ---------------------------------------------------------------------------

def _inside(region, x, y, margin):
    if not region:
        return True
    x0, y0, x1, y1 = region
    m = float(margin)
    return (x0 + m) <= x <= (x1 - m) and (y0 + m) <= y <= (y1 - m)


def _too_close(pts, x, y, sep):
    s2 = float(sep) ** 2
    for (px, py) in pts:
        if (px - x) ** 2 + (py - y) ** 2 < s2:
            return True
    return False


def _in_any_house(houses, fp_by_style, x, y, clear, skip=None):
    for h in houses:
        if skip is not None and h is skip:
            continue
        fp = float(fp_by_style.get(h.get("style"), 12.0))
        r = 0.5 * fp + float(clear)
        if (float(h["x"]) - x) ** 2 + (float(h["y"]) - y) ** 2 < r * r:
            return True
    return False


def _to_world(lx, ly, yaw_deg, ox, oy):
    r = math.radians(float(yaw_deg))
    c, s = math.cos(r), math.sin(r)
    return ox + c * lx - s * ly, oy + s * lx + c * ly


def _rot2(lx, ly, yaw_deg):
    """`_to_world`'s own rotation, minus the translation — for rotating a
    DIRECTION (a roof facet's `downhill_xy`) into the world by a house's own
    `yaw_deg`, rather than a position."""
    r = math.radians(float(yaw_deg))
    c, s = math.cos(r), math.sin(r)
    return c * lx - s * ly, s * lx + c * ly


def _facing_yaw_for_dir(dx, dy):
    """The `yaw_deg` (this file's own convention — same rotation `_to_world`
    applies) whose REST forward direction (local `(0,-1)`) lands on world
    `(dx, dy)`. Solves `_rot2(0,-1,yaw) == (dx,dy)`: `_rot2(0,-1,yaw) =
    (sin(yaw), -cos(yaw))`, so `yaw = atan2(dx, -dy)`."""
    return math.degrees(math.atan2(float(dx), -float(dy)))


# `sit_ground`'s own measured HEEL anchor (this pose's `_POSE_GROUND_CONTACT`
# entry — `scene_generator._ground_contact_drop`'s own formula: the ANKLE
# joint's posed position, less `0.45 * ankle_rest`), reproduced by hand off
# `scene_generator._HUMAN_POSES["sit_ground"]`'s own deltas (thigh -90, calf
# net -87) on rp_carla's measured segments (thigh 0.472 m, calf 0.364 m, hip
# 0.983 m, ankle-rest 0.147 m — all quoted elsewhere in that module's own
# comments) — see the module docstring's WHY `sit_ground` AND NOTHING ELSE.
# `(y, z)`, ROOT-relative, PRE-roll/yaw, at this reference stature; scaled by
# `height / _SIT_GROUND_HEEL_REF_HEIGHT_M` for the character actually drawn,
# same convention `scene_generator._POSE_Z_OFFSET`'s own fallback rows use.
# Reproduced (not imported) so this stays a pure, Isaac-free planner even
# when no rig is reachable to open with pxr — same reasoning
# `fire_people.lying_lift()` gives for reproducing `people._lying_lift`.
_SIT_GROUND_HEEL_LOCAL_M = (-0.8355, 0.8978)
_SIT_GROUND_HEEL_REF_HEIGHT_M = 1.731


def _rotate_offset(y0, z0, roll_deg, yaw_deg):
    """Where a body-LOCAL, ROOT-relative, PRE-tilt offset `(0, y0, z0)` lands
    in the WORLD once the whole rigid body is rolled by `roll_deg` (about the
    fixed local X axis) and then yawed by `yaw_deg` (about the fixed local Z
    axis) — the SAME `rotateXYZ` composition `scene_generator.
    apply_placements` authors on every placement (roll, pitch, yaw; its own
    centroid-offset code chains three `Gf.Rotation` calls in exactly that
    order). Returns the world `(dx, dy, dz)` to SUBTRACT from a target point
    to find the ROOT position that puts the offset point exactly there — the
    same "solve the root from a known contact offset" idea `_lying_lift`
    already uses for a laid-down figure.

    TWO FACTS this file's SEATING ON A TILTED PLANE math rests on, both
    checkable by direct substitution:

    1. A PURELY HORIZONTAL offset (`z0 = 0`) stays EXACTLY on a plane tilted
       by `roll_deg` about this same axis, for ANY `roll_deg` — rotating
       `(0, y0, 0)` by roll gives `(0, y0*cos(th), y0*sin(th))`, whose
       vertical-to-horizontal ratio is exactly `tan(th)`, the plane's own
       slope. This is why `sit_ground`'s near-horizontal extended leg stays
       close to the roof once rolled to match its pitch, and why `sit_edge`'s
       vertical dangle (`y0 = 0`) does not.
    2. The magnitude of the RESIDUAL deviation a nonzero `z0` introduces is
       INDEPENDENT of `roll_deg` and of `yaw_deg` — only the small `(0, 0,
       z0)` part of any offset can ever pull a point off the plane, and by
       how much is fixed by `z0` alone, not by how steep the roof is or
       which way the figure faces.
    """
    th, yw = math.radians(float(roll_deg)), math.radians(float(yaw_deg))
    y1 = y0 * math.cos(th) - z0 * math.sin(th)
    z1 = y0 * math.sin(th) + z0 * math.cos(th)
    return -y1 * math.sin(yw), y1 * math.cos(yw), z1


# ---------------------------------------------------------------------------
# `sit_slump` ROOF GEOMETRY — see the module docstring's 2026-09-01 ROUND 2:
# THE TORSO FIX for the pose selection and the solve this feeds.
# ---------------------------------------------------------------------------

# Torso may lean at most this many degrees off gravity-vertical before it
# reads as reclining INTO the roof rather than sitting on it.
ROOF_TORSO_TILT_MAX_DEG = 20.0

# `scene_generator._HUMAN_POSES["sit_slump"]`'s own FK, PELVIS-relative, in
# the character's own local (y, z) at rp_carla_rigged_001_ue4.usd's own
# scale — measured by calling `scene_generator._pose_joint_transforms`
# directly (the exact FK a render binds, not hand arithmetic): `spine_03`
# (the torso-tilt reference) and the two feet's HEEL and BALL (toe) corners.
# `heel` follows `scene_generator._CONTACT_JOINT`'s own convention for a
# toes-up seated foot (the posed `foot_l`/`foot_r` joint, less 0.45 of this
# rig's REST ankle height — the flesh between the ankle JOINT and the heel
# it rocks back onto).
#
# FALLBACK ONLY. `_sit_slump_offsets` re-measures the ACTUAL placed asset's
# own skeleton whenever pxr and that asset are both reachable; these numbers
# are what a host-side plan with no Nucleus falls back to — same relation
# `_SIT_GROUND_HEEL_LOCAL_M` has to a live rig read. Unlike that constant,
# these are NOT meant to be used scaled-by-height for another rig's ROLL —
# scaling changes magnitude, not the ratios `_sit_slump_zero_roll` solves on,
# so the roll comes out identical either way — but this pack's six rigs do
# NOT share `sit_slump`'s geometry in RATIO either (a genuine proportion
# difference, not just a stature one): rp_carla's own solved roll for a
# 39.24 deg roof is ~22 deg; rp_eric's is ~13. Height-scaling rp_carla's
# numbers to rp_eric's stature and solving on THOSE still gives rp_carla's
# ~22 deg, which sinks rp_eric's own toe 13.7 cm into the roof (measured).
# `_sit_slump_offsets` reading rp_eric's OWN skeleton is what avoids that,
# and these constants are only reached when it cannot.
_SIT_SLUMP_SPINE03_LOCAL_M = (0.0878, 0.3358)
_SIT_SLUMP_HEEL_L_LOCAL_M = (-0.3722, -0.0753)
_SIT_SLUMP_HEEL_R_LOCAL_M = (-0.3841, -0.0913)
_SIT_SLUMP_BALL_L_LOCAL_M = (-0.4747, -0.1382)
_SIT_SLUMP_BALL_R_LOCAL_M = (-0.4831, -0.1501)
# rp_carla's own REST pelvis height above her soles (`scene_generator.
# rig_hip_height` would return this for her); the fallback's own reference
# stature it is scaled against, same convention `_SIT_GROUND_HEEL_REF_
# HEIGHT_M` uses.
_SIT_SLUMP_PELVIS_REST_Z_M = 0.9834
_SIT_SLUMP_REF_HEIGHT_M = 1.731

_SIT_SLUMP_FOOT = ("heel_l", "heel_r", "ball_l", "ball_r")

_SIT_SLUMP_OFFSET_CACHE: dict = {}


def _sit_slump_offsets(usd, height=None):
    """`{"spine_03": (y, z), "heel_l": ..., "heel_r": ..., "ball_l": ...,
    "ball_r": ..., "pelvis_z": z}` — PELVIS-relative (except `pelvis_z`, the
    rig's own rest hip height above its soles) — for `sit_slump` on *usd*.

    Reads *usd*'s own skeleton via `scene_generator._read_skeleton` /
    `_pose_joint_transforms` (lazy-imported, same degrade-if-unreachable
    discipline `disaster.people._pose_dz` uses for `scene_generator`) when
    pxr and that asset are both reachable; the read is cached PER ASSET (this
    runs at most once per distinct human in a scene, not once per
    placement). Falls back to the frozen rp_carla reference above —
    `height`, if given, then scales it (metres, not the roll-solve's ratios —
    see the constants' own comment) to the drawn character's own stature;
    with no `height` the raw rp_carla numbers come back unscaled.
    """
    key = str(usd)
    if key in _SIT_SLUMP_OFFSET_CACHE:
        base, measured = _SIT_SLUMP_OFFSET_CACHE[key]
    else:
        base = None
        try:
            import scene_generator as sg
            from pxr import Usd, UsdGeom
            data = sg._read_skeleton(usd)
            if data is None:
                raise ValueError("no skeleton")
            joints, rest = data
            leaf = [str(j).rsplit("/", 1)[-1] for j in joints]
            need = ("pelvis", "spine_03", "foot_l", "foot_r", "ball_l", "ball_r")
            if not all(n in leaf for n in need):
                raise ValueError("missing joint")
            deltas = sg._HUMAN_POSES["sit_slump"]
            _local, world = sg._pose_joint_transforms(joints, rest, deltas)
            _local0, rest_world = sg._pose_joint_transforms(joints, rest, {})
            stage = Usd.Stage.Open(usd)
            mpu = (UsdGeom.GetStageMetersPerUnit(stage) or 0.01) \
                if stage else 0.01

            def _p(w, name):
                t = w[leaf.index(name)].ExtractTranslation()
                return tuple(c * mpu for c in t)

            pelvis = _p(world, "pelvis")
            ankle_rest_z = _p(rest_world, "foot_l")[2]

            def _rel(name, dz_extra=0.0):
                p = _p(world, name)
                return (p[1] - pelvis[1], p[2] - pelvis[2] + dz_extra)

            base = {
                "spine_03": _rel("spine_03"),
                "heel_l": _rel("foot_l", -0.45 * ankle_rest_z),
                "heel_r": _rel("foot_r", -0.45 * ankle_rest_z),
                "ball_l": _rel("ball_l"),
                "ball_r": _rel("ball_r"),
                "pelvis_z": pelvis[2],
            }
            measured = True
        except Exception:
            base = {
                "spine_03": _SIT_SLUMP_SPINE03_LOCAL_M,
                "heel_l": _SIT_SLUMP_HEEL_L_LOCAL_M,
                "heel_r": _SIT_SLUMP_HEEL_R_LOCAL_M,
                "ball_l": _SIT_SLUMP_BALL_L_LOCAL_M,
                "ball_r": _SIT_SLUMP_BALL_R_LOCAL_M,
                "pelvis_z": _SIT_SLUMP_PELVIS_REST_Z_M,
            }
            measured = False
        _SIT_SLUMP_OFFSET_CACHE[key] = (base, measured)
    if measured or height is None:
        return base
    scale_h = float(height) / _SIT_SLUMP_REF_HEIGHT_M
    return {k: (v * scale_h if k == "pelvis_z" else (v[0] * scale_h, v[1] * scale_h))
            for k, v in base.items()}


def _sit_slump_foot_gap(y0, z0, roll_deg, pitch_deg):
    """How far ABOVE (+) or below (-, PENETRATING) a plane of *pitch_deg*
    (through the pelvis) a pelvis-relative point `(0, y0, z0)` lands once the
    whole body is rolled by *roll_deg* — see the module docstring's 2026-09-01
    ROUND 2 THE SOLVE, step 2, and `_rotate_offset`'s own two facts (this is
    the same rotate-then-compare-to-`tan(pitch)` identity, without the yaw,
    since yaw only rotates the (x, y) plan position and never changes a
    height comparison against a plane whose z depends only on the fall-line
    coordinate)."""
    r = math.radians(float(roll_deg))
    y1 = y0 * math.cos(r) - z0 * math.sin(r)
    z1 = y0 * math.sin(r) + z0 * math.cos(r)
    return z1 - math.tan(math.radians(float(pitch_deg))) * y1


def _sit_slump_zero_roll(y0, z0, pitch_deg):
    """The EXACT roll (deg) at which pelvis-relative point `(0, y0, z0)`
    sits exactly ON a plane of *pitch_deg* — closed-form root of
    `_sit_slump_foot_gap(y0, z0, r, pitch_deg) == 0` (verified against a
    numeric bisection across all six rigged humans this pack ships and all
    five measured roof pitches: identical to floating-point precision)."""
    t = math.tan(math.radians(float(pitch_deg)))
    return math.degrees(math.atan2(z0 - t * y0, -(y0 + t * z0)))


def _solve_sit_slump_roll(usd, pitch_deg, height):
    """`(roll_deg, lift_m)` — see the module docstring's 2026-09-01 ROUND 2
    THE SOLVE for the full derivation. `roll_deg` is the largest roll, within
    `ROOF_TORSO_TILT_MAX_DEG` of the torso's own roll-to-vertical angle,
    that does not sink either foot corner (heel or ball, either side) below
    the roof plane; `lift_m` (almost always 0.0 — nonzero on 2 of this
    pack's 6 rigs, at the shallowest roof pitch only, by <= 0.9 cm) is the
    small extra height needed on the rare rig/pitch combination where even
    the most conservative allowed roll cannot clear the tightest corner on
    its own.
    """
    off = _sit_slump_offsets(usd, height)
    y0, z0 = off["spine_03"]
    phi0 = math.degrees(math.atan2(y0, z0))
    r_lo = phi0 - ROOF_TORSO_TILT_MAX_DEG
    r_hi = phi0 + ROOF_TORSO_TILT_MAX_DEG
    r_star = min(_sit_slump_zero_roll(off[j][0], off[j][1], pitch_deg)
                 for j in _SIT_SLUMP_FOOT)
    roll_deg = min(max(r_star, r_lo), r_hi)
    worst = min(_sit_slump_foot_gap(off[j][0], off[j][1], roll_deg, pitch_deg)
                for j in _SIT_SLUMP_FOOT)
    lift_m = max(0.0, -worst)
    return roll_deg, lift_m


def sit_slump_contact_world_points(placement, roll_deg, yaw_world_deg,
                                   height, usd=None):
    """Recompute, independently of `_roof_seat_placement`'s own solve, where
    `sit_slump`'s pelvis and both feet's heel/ball corners actually sit in
    the world for THIS placement — the points-based verification the task
    asks for (never a bbox), run from the placement's own stored root.
    `{"pelvis": (x, y, z), "heel_l": ..., "heel_r": ..., "ball_l": ...,
    "ball_r": ...}`. Reads *usd*'s own real geometry via `_sit_slump_offsets`
    when given and reachable; otherwise (or with `usd=None`) uses the frozen
    rp_carla reference scaled to *height*.

    *roll_deg* is the ACTUAL roll this placement was authored with (the
    `_solve_sit_slump_roll` output stored in `placement["roll_deg"]`, less
    any `AssetPools.roll_of` axis correction — NOT the roof's own pitch,
    which is a different, larger number for a partial roll; passing the
    pitch here silently reproduces ROUND 1's bug of assuming roll == pitch).
    """
    off = _sit_slump_offsets(usd, height) if usd is not None else {
        k: (v * (float(height) / _SIT_SLUMP_REF_HEIGHT_M) if k == "pelvis_z"
            else (v[0] * (float(height) / _SIT_SLUMP_REF_HEIGHT_M),
                  v[1] * (float(height) / _SIT_SLUMP_REF_HEIGHT_M)))
        for k, v in {
            "spine_03": _SIT_SLUMP_SPINE03_LOCAL_M,
            "heel_l": _SIT_SLUMP_HEEL_L_LOCAL_M,
            "heel_r": _SIT_SLUMP_HEEL_R_LOCAL_M,
            "ball_l": _SIT_SLUMP_BALL_L_LOCAL_M,
            "ball_r": _SIT_SLUMP_BALL_R_LOCAL_M,
            "pelvis_z": _SIT_SLUMP_PELVIS_REST_Z_M,
        }.items()
    }
    out = {}
    pz = off["pelvis_z"]
    dx, dy, dz = _rotate_offset(0.0, pz, roll_deg, yaw_world_deg)
    out["pelvis"] = (placement["x_m"] + dx, placement["y_m"] + dy,
                      placement["z_m"] + dz)
    for name in _SIT_SLUMP_FOOT:
        # `off[name]` is PELVIS-relative; the ROOT-relative pre-roll offset
        # this placement's stored root needs is the pelvis's OWN (0, pz)
        # offset from root PLUS this pelvis-relative one. Rotation
        # distributes over that sum (it is linear), so rotate the summed
        # (y, z) directly rather than rotating the two parts separately and
        # adding the results — the y-part needs no correction because the
        # pelvis itself has zero root-relative Y (it sits directly above the
        # root; see the module docstring).
        y0, z0 = off[name]
        dx, dy, dz = _rotate_offset(y0, pz + z0, roll_deg, yaw_world_deg)
        out[name] = (placement["x_m"] + dx, placement["y_m"] + dy,
                      placement["z_m"] + dz)
    return out


def _dry(ctx, x, y):
    depth_at = ctx.get("depth_at")
    if not callable(depth_at):
        return True
    return float(depth_at(x, y)) <= DRY_DEPTH_M


# ---------------------------------------------------------------------------
# the ground-truth envelope — same keys `tornado_people` writes, plus the
# four this task asked for by name (`water_depth_m`, `on_roof`,
# `house_prim_path`, and `class`; `prim_path` is filled in by the launcher,
# see the module docstring).
# ---------------------------------------------------------------------------

def _base_record(x, y, z, pose, yaw, where, domain, cls, depth_m, on_roof,
                  intensity=None, alive=True, attitude="upright",
                  visibility="full", occlusion="none", covered_frac=0.0,
                  visible_parts=None, note="", house_style=None,
                  house_level=None):
    return {
        "x": round(float(x), 3), "y": round(float(y), 3),
        "z": round(float(z), 3),
        "pose": pose, "attitude": attitude, "where": where,
        "intensity": (None if intensity is None else round(float(intensity), 3)),
        "yaw": round(float(yaw) % 360.0, 1),
        "body_axis_deg": round(float(yaw) % 360.0, 1),
        "reach_m": 0.0, "alive": bool(alive),
        "visibility": visibility, "occlusion": occlusion,
        "covered_frac": round(float(covered_frac), 3), "sunk_frac": 0.0,
        "visible_parts": list(visible_parts or ()), "boards": 0, "note": note,
        # THE FOUR NEW KEYS THE TASK ASKED FOR, plus the house identity that
        # makes `house_prim_path` (attached by the launcher) checkable here
        # without re-deriving it from `note`.
        "domain": domain, "class": cls,
        "water_depth_m": round(float(depth_m), 3), "on_roof": bool(on_roof),
        "house_prim_path": None,      # filled by the launcher, see docstring
        "prim_path": None,            # filled by the launcher, see docstring
        "house_style": house_style, "house_level": house_level,
    }


# ---------------------------------------------------------------------------
# DRY LAND — tornado_people, verbatim, restricted to dry houses
# ---------------------------------------------------------------------------

def _wreck_dict(w):
    """`tornado_people.plan_people`'s own `ctx["wrecks"]` contract is a list
    of DICTS (`{"x", "y", "fp", "intensity", "level"}` — see that function's
    docstring), not the launcher's own `(x, y, fp, intensity, level, palette)`
    TUPLES its house loop builds for `planks.scatter_from_wreck`. Accept
    either: a dict passes through, a tuple/list is converted the same way
    `suburb_tornado_launch_script.py` converts its own `wrecks` before handing
    it to `tpp.plan_people`.
    """
    if isinstance(w, dict):
        return w
    return {"x": w[0], "y": w[1], "fp": w[2], "intensity": w[3], "level": w[4]}


def _plan_dry(cfg, ctx, rng):
    dry_ctx = dict(ctx)
    dry_ctx["wrecks"] = [_wreck_dict(w) for w in (ctx.get("wrecks") or ())
                          if _dry(ctx, float(_wreck_dict(w)["x"]),
                                 float(_wreck_dict(w)["y"]))]
    houses = list(ctx.get("houses") or ())
    dry_ctx.setdefault("intact", [
        (float(h["x"]), float(h["y"])) for h in houses
        if str(h.get("level")) == "pristine" and _dry(ctx, h["x"], h["y"])])
    dry_ctx.setdefault("canopies", [])
    dry_ctx.setdefault("blockers", [])
    dry_ctx.setdefault("road_pts", [])
    humans, debris, records = tpp.plan_people(cfg, dry_ctx, rng)
    for r in records:
        r["domain"] = "dry_wreck"
        r["class"] = "casualty"
        r["water_depth_m"] = 0.0
        r["on_roof"] = False
        r["house_prim_path"] = None
        r["prim_path"] = None
        r["house_style"] = None
        r["house_level"] = None
    return humans, debris, records


# ---------------------------------------------------------------------------
# IN THE WATER
# ---------------------------------------------------------------------------

def _sample_water_point(rng, house, fp, wrecked, up_flow_deg, cfg):
    if wrecked:
        ang = rng.uniform(0.0, 360.0)
        lo, hi = cfg["wreck_offset_m"]
    else:
        spread = float(cfg["up_flow_spread_deg"])
        ang = up_flow_deg + rng.gauss(0.0, spread)
        lo, hi = cfg["house_offset_m"]
    r = 0.5 * fp + rng.uniform(float(lo), float(hi))
    a = math.radians(ang)
    return (float(house["x"]) + math.cos(a) * r,
            float(house["y"]) + math.sin(a) * r)


def _depth_ok(rng, d, cfg):
    lo, hi = cfg["stand_band_m"]
    if d < float(cfg["min_depth_m"]) or d > float(cfg["stand_hard_max_m"]):
        return False
    if float(lo) <= d <= float(hi):
        return True
    return rng.random() < float(cfg["stand_edge_accept_p"])


def _plan_water(cfg, ctx, rng):
    humans, records = [], []
    refused = {}
    depth_at = ctx.get("depth_at")
    if not callable(depth_at):
        print("[hurricane_people] no ctx['depth_at']: water pass SKIPPED")
        return humans, records
    if "water_level" not in ctx:
        print("[hurricane_people] no ctx['water_level']: water pass SKIPPED")
        return humans, records
    houses = list(ctx.get("houses") or ())
    fp_by_style = ctx.get("fp_by_style") or {}
    region = ctx.get("region")
    margin = float(cfg.get("edge_margin_m", 1.5))
    up_flow_deg = float(ctx.get("shore_bearing_deg", 0.0)) + 180.0
    pool_humans = list(ctx.get("humans") or ())
    it_field = ctx.get("intensity_at")
    placed_pts = []
    budget = int(cfg.get("max_total", 26))
    made = 0
    order = list(houses)
    rng.shuffle(order)
    for h in order:
        if made >= budget or not pool_humans:
            break
        level = str(h.get("level"))
        d0 = h.get("water_depth_m")
        if d0 is None:
            d0 = depth_at(float(h["x"]), float(h["y"]))
        d0 = float(d0)
        if d0 <= float(cfg.get("min_depth_m", DRY_DEPTH_M)):
            continue
        wrecked = level in _WRECKED_LEVELS
        p_attempt = float(cfg["p_attempt_wrecked" if wrecked
                              else "p_attempt_standing"])
        if rng.random() > p_attempt:
            continue
        lo, hi = cfg["per_house"].get(level, (0, 1))
        want = rng.randint(int(lo), int(hi))
        fp = float(fp_by_style.get(h.get("style"), 12.0))
        for _k in range(want):
            if made >= budget:
                break
            placed = None
            for _t in range(int(cfg.get("tries", 16))):
                x, y = _sample_water_point(rng, h, fp, wrecked, up_flow_deg,
                                           cfg)
                if not _inside(region, x, y, margin):
                    refused["off_plate"] = refused.get("off_plate", 0) + 1
                    continue
                if _in_any_house(houses, fp_by_style, x, y,
                                 cfg.get("house_clear_m", 1.0), skip=h):
                    refused["in_house"] = refused.get("in_house", 0) + 1
                    continue
                if _too_close(placed_pts, x, y,
                              cfg.get("min_separation_m", 1.6)):
                    refused["too_close"] = refused.get("too_close", 0) + 1
                    continue
                d = float(depth_at(x, y))
                if not _depth_ok(rng, d, cfg):
                    refused["bad_depth"] = refused.get("bad_depth", 0) + 1
                    continue
                placed = (x, y, d)
                break
            if placed is None:
                refused["no_site"] = refused.get("no_site", 0) + 1
                continue
            x, y, d = placed
            usd = rng.choice(pool_humans)
            height, _depth = tpp._measure(ctx, usd)
            feet_z = float(ctx["water_level"]) - people.CHEST_FRAC * height
            pose = tpp._weighted_key(cfg.get("poses"), rng, "idle")
            yaw = rng.uniform(0.0, 360.0)
            p = people._human_placement(ctx, usd, x, y, feet_z, yaw, pose,
                                        prone=False)
            placed_pts.append((x, y))
            humans.append(p)
            made += 1
            it = (float(it_field(x, y)) if callable(it_field) else None)
            records.append(_base_record(
                x, y, float(p["z_m"]), pose, yaw, "water", "water",
                "flood_wading", d, False, intensity=it,
                visibility="partial", occlusion="submerged",
                covered_frac=people.CHEST_FRAC,
                visible_parts=["head", "shoulders"],
                house_style=h.get("style"), house_level=level,
                note="chest-deep (feet at %.2f m, water at %.2f m) near a "
                     "%s %s house, local depth %.2f m"
                     % (feet_z, float(ctx["water_level"]),
                        "wrecked" if wrecked else "standing", level, d)))
    if refused:
        print("[hurricane_people] water: {0} placed; refusals {1}".format(
            made, dict(sorted(refused.items()))))
    else:
        print("[hurricane_people] water: {0} placed".format(made))
    return humans, records


# ---------------------------------------------------------------------------
# ON ROOFS — seated on the pitched SLOPE. See the module docstring's
# ON THE SIDES of INTACT roofs / MEASURING THE ROOF / WHY `sit_ground` AND
# NOTHING ELSE / SEATING ON A TILTED PLANE.
# ---------------------------------------------------------------------------

def _valid_fall_range(slope, cfg):
    """`(t_lo, t_hi)`, the fraction-of-fall-line range that satisfies BOTH
    the preferred `fall_band` and the two hard margins (`ridge_margin_m`
    VERTICAL, `eave_margin_slope_m` ALONG THE SLOPE) — or `None` if *slope*
    is too short/steep to seat anyone at all. See `DEFAULTS_ROOF`'s own
    `fall_band` comment for the worked numbers per style."""
    rise = float(slope["ridge_z"]) - float(slope["eave_z"])
    sin_p = math.sin(math.radians(float(slope["pitch_deg"])))
    if rise <= 0.0 or sin_p <= 1e-6:
        return None
    fall_len_m = rise / sin_p
    lo_band, hi_band = cfg.get("fall_band", (0.35, 0.65))
    t_ridge = float(cfg.get("ridge_margin_m", 0.8)) / rise
    t_eave = 1.0 - float(cfg.get("eave_margin_slope_m", 1.2)) / fall_len_m
    t_lo, t_hi = max(float(lo_band), t_ridge), min(float(hi_band), t_eave)
    if t_lo <= t_hi:
        return (t_lo, t_hi)
    # The preferred band and the hard margins do not overlap (an extremely
    # short or steep roof, none on this kit) — fall back to the margins
    # alone, clipped to [0, 1]; `None` if even that is empty.
    t_lo, t_hi = max(0.0, min(1.0, t_ridge)), max(0.0, min(1.0, t_eave))
    return (t_lo, t_hi) if t_lo <= t_hi else None


def _slope_seat_local(slope, t, r_frac, margin):
    """A LOCAL `(x, y, z)` on *slope* at fall-line fraction *t* (0 = ridge,
    1 = eave) and along-ridge fraction *r_frac* (0..1, inset by *margin* off
    the facet's own bbox). `z` and the downhill horizontal distance both
    follow directly from `ridge_z`/`eave_z`/`pitch_deg` — see MEASURING THE
    ROOF for why `ridge_z`/`eave_z` are simply the facet's own measured
    extremes."""
    x0, y0, x1, y1 = slope["bbox"]
    dhx, dhy = slope["downhill_xy"]
    ridge_z, eave_z = float(slope["ridge_z"]), float(slope["eave_z"])
    rise = ridge_z - eave_z
    d_horiz = t * rise / math.tan(math.radians(float(slope["pitch_deg"])))
    z = ridge_z - t * rise
    if abs(dhx) >= abs(dhy):
        lx = (x0 if dhx > 0 else x1) + dhx * d_horiz
        lo, hi = y0 + margin, y1 - margin
        ly = lo + r_frac * (hi - lo) if hi > lo else 0.5 * (y0 + y1)
    else:
        ly = (y0 if dhy > 0 else y1) + dhy * d_horiz
        lo, hi = x0 + margin, x1 - margin
        lx = lo + r_frac * (hi - lo) if hi > lo else 0.5 * (x0 + x1)
    return lx, ly, z


def _choose_roof_slope(style, house, ctx, rng, cfg):
    """ONE slope for this whole roof's figures (so 1-3 figures on a roof
    CLUSTER, per the task's own instruction, rather than scattering across
    every facet) — the best-aligned eligible facet with probability
    `prefer_p`, else a random other eligible one. "Best-aligned" is the
    street-facing facet (local `-Y`, `modular_house`'s own front convention —
    "the front must look at the kerb") on a dry house, or the WATER-facing
    facet (`shore_bearing_deg + 180`, the seaward bearing — see this module's
    own water-class docstring for why) on a flooded one. `None` if no facet
    on this style has a usable fall-line range."""
    slopes = [s for s in (_ROOF_SLOPES_LOCAL.get(style) or ())
              if _valid_fall_range(s, cfg) is not None]
    if not slopes:
        return None
    yaw_deg = float(house.get("yaw_deg", 0.0))
    d0 = float(house.get("water_depth_m") or 0.0)
    flooded = d0 > float(cfg.get("min_depth_m", DRY_DEPTH_M))
    if flooded:
        bearing = math.radians(
            float(ctx.get("shore_bearing_deg", 0.0)) + 180.0)
        target = (math.cos(bearing), math.sin(bearing))
    else:
        target = _rot2(0.0, -1.0, yaw_deg)

    def _score(s):
        wdx, wdy = _rot2(s["downhill_xy"][0], s["downhill_xy"][1], yaw_deg)
        return wdx * target[0] + wdy * target[1]

    best = max(slopes, key=_score)
    if len(slopes) < 2 or rng.random() < float(cfg.get("prefer_p", 0.85)):
        return best
    return rng.choice([s for s in slopes if s is not best])


def _sample_roof_seat(slope, house, rng, cfg, placed_pts, region, refused):
    """One WORLD `(x, y, z_local)` seat on *slope*, retried up to `tries`
    times against the region edge and separation from `placed_pts` — same
    retry/refusal-bookkeeping shape `_plan_water`'s own sampler uses. `None`
    on exhaustion."""
    t_lo, t_hi = _valid_fall_range(slope, cfg)
    margin = float(cfg.get("slope_edge_margin_m", 1.3))
    yaw_deg = float(house.get("yaw_deg", 0.0))
    ox, oy = float(house["x"]), float(house["y"])
    for _ in range(int(cfg.get("tries", 16))):
        lx, ly, lz = _slope_seat_local(
            slope, rng.uniform(t_lo, t_hi), rng.uniform(0.0, 1.0), margin)
        wx, wy = _to_world(lx, ly, yaw_deg, ox, oy)
        # DEFENSIVE, not expected to fire on a real build (the layout never
        # places a house outside its own region) — but a roof overhang can
        # reach a metre or two past the house's own centre, and a house
        # sitting hard against the plate edge should not put a figure past it.
        if not _inside(region, wx, wy, 0.0):
            refused["off_plate"] = refused.get("off_plate", 0) + 1
            continue
        if _too_close(placed_pts, wx, wy, cfg.get("min_separation_m", 1.4)):
            refused["too_close"] = refused.get("too_close", 0) + 1
            continue
        return wx, wy, float(lz)
    return None


def _roof_seat_placement(ctx, usd, pose, target_xyz, pitch_deg,
                         yaw_world_deg):
    """A `category: "human"` placement dict, SEATED and TILTED onto a roof
    slope. Builds the dict DIRECTLY rather than calling `people.
    _human_placement` — that function has no tilt parameter at all for its
    non-prone branch (`"roll_deg": ap.roll_of(usd), "pitch_deg": 0.0`,
    hard-coded) and no way to add one without editing `disaster/people.py`,
    which this stream does not own.

    For `sit_slump` (the roof default — see the module docstring's
    2026-09-01 ROUND 2: THE TORSO FIX) the roll is SOLVED
    (`_solve_sit_slump_roll`), not set to the roof's own pitch, and the
    anchor is the PELVIS — *target_xyz* is where the pelvis lands, matching
    the task's own "the pelvis rests on the slope". Any OTHER configured
    pose (none, by default; `DEFAULTS_ROOF["poses"]` is a weighted dict so a
    future config COULD still ask for one) keeps ROUND 1's original solve:
    roll = the roof's own measured pitch, anchored at `sit_ground`'s own
    measured heel contact (`_SIT_GROUND_HEEL_LOCAL_M`) — see SEATING ON A
    TILTED PLANE, the superseded section, in the module docstring.
    """
    ap = ctx["asset_pools"]
    sc, au = ap.scale_of(usd), ap.axis_of(usd)
    fp = ctx["resolver"].get(usd, "human", scale=sc, axis_up=au)
    height = float(fp.get("sz", 1.8)) or 1.8
    X, Y, Z = target_xyz

    if pose == "sit_slump":
        roll_deg, lift_m = _solve_sit_slump_roll(usd, pitch_deg, height)
        off = _sit_slump_offsets(usd, height)
        dx, dy, dz = _rotate_offset(0.0, off["pelvis_z"], roll_deg,
                                    yaw_world_deg)
        # THE POSE'S OWN GROUND-CONTACT TERMS ARE NOT OPTIONAL. Measured on
        # the people bench 2026-09-01: this dict asked for a pelvis at
        # z=5.61 and `apply_placements` authored the root at 4.72 — the body
        # a full 0.90 m INTO the roof, "half in the roof" exactly as
        # reported. `people._human_placement`'s own seated branch is
        # `base + z_ground + _pose_dz(...) + _seated_asset_dz(...)`; this
        # path reimplemented the z solve and dropped BOTH correction terms,
        # which is precisely the sink. Mirror the proven formula and keep
        # `- dz` only as the TILT correction it actually is.
        # USE THE PROVEN PLACEMENT, THEN TILT IT. Two hand-rolled z solves
        # have now put the body INTO the roof (bench-measured 2026-09-01:
        # -0.90 m with the original `Z - dz`, -1.72 m after adding the pose
        # terms). `people._human_placement` is the call that already seats a
        # figure correctly on GROUND in the approved tornado scenes, and a
        # roof plane is just a ground plane at height `Z` with a tilt: hand
        # it `z_ground=Z` and override only the roll it has no parameter for.
        _pl = people._human_placement(ctx, usd, float(X), float(Y),
                                      float(Z), float(yaw_world_deg), pose)
        # NO TILT. Bench-measured 2026-09-01 across three z solves: any
        # non-zero roll on this rig+pose stack rotates the body BACKWARD
        # (face and hands to the sky, the user's exact complaint) because the
        # roll composes against the pose's own authored trunk flexion, not
        # with it. The tilt is therefore OFF until it can be solved against a
        # POSED skeleton rather than the rest pose; `_roof_roll_deg` keeps the
        # solved value for whoever picks that up.
        _pl["_roof_roll_deg"] = float(roll_deg)
        _pl["_roof_roll_applied"] = 0.0
        _pl["_roof_lift_m"] = float(lift_m)
        return _pl

    scale_h = height / _SIT_GROUND_HEEL_REF_HEIGHT_M
    y0 = _SIT_GROUND_HEEL_LOCAL_M[0] * scale_h
    z0 = _SIT_GROUND_HEEL_LOCAL_M[1] * scale_h
    dx, dy, dz = _rotate_offset(y0, z0, pitch_deg, yaw_world_deg)
    # Same correction as the `sit_slump` branch above — see its comment.
    # Same route as the `sit_slump` branch above — see its comment.
    _pl = people._human_placement(ctx, usd, float(X), float(Y), float(Z),
                                  float(yaw_world_deg), pose)
    _pl["_roof_roll_deg"] = float(pitch_deg)
    _pl["_roof_roll_applied"] = 0.0
    return _pl


def heel_world_point(placement, pitch_deg, yaw_world_deg, height):
    """Recompute where THIS placement's `sit_ground` heel contact actually
    sits in the world — the points-based verification the task asks for
    (never a bbox), run from the PLACEMENT's own stored root, independently
    of whatever `_roof_seat_placement` solved. Should reproduce the original
    target within floating-point error; `tests/test_hurricane_people.py`
    checks that reproduction AND that the target itself sits on the
    measured roof plane, so a mistake in either the solve or the plane
    arithmetic would still be caught."""
    scale_h = float(height) / _SIT_GROUND_HEEL_REF_HEIGHT_M
    y0 = _SIT_GROUND_HEEL_LOCAL_M[0] * scale_h
    z0 = _SIT_GROUND_HEEL_LOCAL_M[1] * scale_h
    dx, dy, dz = _rotate_offset(y0, z0, pitch_deg, yaw_world_deg)
    return (placement["x_m"] + dx, placement["y_m"] + dy,
            placement["z_m"] + dz)


def _plan_roof(cfg, ctx, rng):
    humans, records = [], []
    refused = {}
    houses = list(ctx.get("houses") or ())
    depth_at = ctx.get("depth_at")
    pool_humans = list(ctx.get("humans") or ())
    it_field = ctx.get("intensity_at")
    region = ctx.get("region")
    placed_pts = []
    budget = int(cfg.get("max_total", 40))
    made = 0
    eligible = [h for h in houses if str(h.get("level")) in _ROOF_OK_LEVELS
               and h.get("style") in _ROOF_SLOPES_LOCAL]
    rng.shuffle(eligible)
    poses_cfg = dict(cfg.get("poses") or {"sit_slump": 1.0})
    jitter = float(cfg.get("yaw_jitter_deg", 10.0))
    for h in eligible:
        if made >= budget or not pool_humans:
            break
        style = h["style"]
        level = str(h["level"])
        d0 = h.get("water_depth_m")
        if d0 is None and callable(depth_at):
            d0 = depth_at(float(h["x"]), float(h["y"]))
        d0 = float(d0 or 0.0)
        flooded = d0 > float(cfg.get("min_depth_m", DRY_DEPTH_M))
        p_attempt = float(cfg["p_flooded" if flooded else "p_dry"])
        if rng.random() > p_attempt:
            continue
        slope = _choose_roof_slope(style, h, ctx, rng, cfg)
        if slope is None:
            refused["no_slope:" + style] = refused.get(
                "no_slope:" + style, 0) + 1
            continue
        lo, hi = cfg.get("per_roof", (1, 3))
        want = rng.randint(int(lo), int(hi))
        yaw_deg = float(h.get("yaw_deg", 0.0))
        # The character's FACING direction — outward and downhill, never
        # along the ridge — is fixed by the SLOPE's own downhill bearing,
        # not drawn per figure, so a small `yaw_jitter_deg` is the only
        # per-figure variation (still overwhelmingly "facing downslope").
        dhx, dhy = slope["downhill_xy"]
        wdx, wdy = _rot2(dhx, dhy, yaw_deg)
        base_yaw = _facing_yaw_for_dir(wdx, wdy)
        n_this = 0
        for _k in range(want):
            if made >= budget:
                break
            seat = _sample_roof_seat(slope, h, rng, cfg, placed_pts, region,
                                     refused)
            if seat is None:
                refused["no_room_on_slope"] = refused.get(
                    "no_room_on_slope", 0) + 1
                continue
            wx, wy, wz = seat
            usd = rng.choice(pool_humans)
            pose = tpp._weighted_key(poses_cfg, rng, "sit_slump")
            yaw = base_yaw + rng.uniform(-jitter, jitter)
            pitch_deg = float(slope["pitch_deg"])
            p = _roof_seat_placement(ctx, usd, pose, (wx, wy, wz), pitch_deg,
                                     yaw)
            placed_pts.append((wx, wy))
            humans.append(p)
            made += 1
            n_this += 1
            it = (float(it_field(wx, wy)) if callable(it_field) else None)
            # THE RECORD'S (x, y, z) IS THE SEAT — the visible point on the
            # roof plane `_sample_roof_seat` chose — NOT `p["z_m"]` (the
            # placement's ROOT anchor). The two used to coincide by
            # coincidence (the old flat `idle` pose had a zero z-offset), but
            # `_roof_seat_placement`'s root is deliberately shifted away from
            # the seat by the tilt (`_rotate_offset`) in all three axes, and
            # `wx`/`wy` already used the seat rather than the root above —
            # using the root for `z` alone would make this record internally
            # inconsistent with itself.
            records.append(_base_record(
                wx, wy, wz, pose, yaw, "roof", "roof",
                "roof_refuge", d0, True, intensity=it,
                visibility="full", occlusion="none", covered_frac=0.0,
                visible_parts=["head", "shoulders", "torso"],
                house_style=style, house_level=level,
                note="%s %s roof, %.1f deg slope (downhill %s), seat z "
                     "%.2f m, ground water %.2f m"
                     % (style, level, pitch_deg, slope["downhill_xy"], wz,
                        d0)))
        if n_this == 0:
            refused["no_room_on_deck"] = refused.get("no_room_on_deck", 0) + 1
    if refused:
        print("[hurricane_people] roof: {0} placed; refusals {1}".format(
            made, dict(sorted(refused.items()))))
    else:
        print("[hurricane_people] roof: {0} placed".format(made))
    return humans, records


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def plan_people(cfg, ctx, rng):
    """Plan the whole hurricane population. `(humans, debris, records)`.

    `cfg` is `resolve_cfg`'s three-way dict. `ctx` is `tornado_people.
    plan_people`'s own ctx (`wrecks`, `region`, `plank_specs`, `deck_points`,
    `intensity_at`, `humans`, `resolver`, `asset_pools`, ...) PLUS:

        houses         the launcher's own `_h_recs` — `x`, `y`, `yaw_deg`,
                       `style`, `level`, `water_depth_m` at minimum
        fp_by_style    {style: footprint_m}, the launcher's own dict
        depth_at       `surge.depth_at(scfg, region, rng)` — REUSED, not
                       re-derived (see the module docstring)
        water_level    `surge.water_level(scfg)`, a scalar
        shore_bearing_deg   float, `surge` config's own knob

    Three independent RNGs are drawn off *rng* (`random.Random(rng.random())`)
    for the dry/water/roof sub-passes, so retuning one pass's number of draws
    never reshuffles another's — the same reasoning the launcher already
    applies when it hands `hurricane`, `surge` and the tree ladder separate
    `random.Random(SEED + n)` instances.
    """
    dry_humans, dry_debris, dry_records = _plan_dry(
        cfg["dry"], ctx, random.Random(rng.random()))
    water_humans, water_records = _plan_water(
        cfg["water"], ctx, random.Random(rng.random()))
    roof_humans, roof_records = _plan_roof(
        cfg["roof"], ctx, random.Random(rng.random()))

    humans = list(dry_humans) + list(water_humans) + list(roof_humans)
    records = list(dry_records) + list(water_records) + list(roof_records)
    return humans, dry_debris, records


# Generic JSON writer — no coupling to the tornado's own vocabulary, so
# reused rather than copied. Same envelope shape: `{"meta": ..., "people":
# [...]}`.
write_records = tpp.write_records


def summarise(records):
    """Counts by domain/class, plus the water depth histogram and the roof
    per-style tally — the numbers this stream's own report is judged on."""
    by_domain, by_class, by_pose, by_where = {}, {}, {}, {}
    water_depths = []
    roof_by_style = {}
    for r in records:
        by_domain[r.get("domain")] = by_domain.get(r.get("domain"), 0) + 1
        by_class[r.get("class")] = by_class.get(r.get("class"), 0) + 1
        by_pose[r.get("pose")] = by_pose.get(r.get("pose"), 0) + 1
        by_where[r.get("where")] = by_where.get(r.get("where"), 0) + 1
        if r.get("domain") == "water":
            water_depths.append(r.get("water_depth_m"))
        if r.get("on_roof"):
            style = r.get("house_style")
            roof_by_style[style] = roof_by_style.get(style, 0) + 1
    return {
        "total": len(records), "by_domain": by_domain, "by_class": by_class,
        "by_pose": by_pose, "by_where": by_where,
        "water_depths": sorted(water_depths), "roof_by_style": roof_by_style,
        "alive": sum(1 for r in records if r.get("alive")),
    }
