---
name: author-human-poses
description: Authoring and validating POSES for the RenderPeople UE4-skeleton rigs (window leaners, roof victims, buried and passed-out figures) — why joint angles derived on paper rendered as grotesque bodies twice, the empirical POSE ROW method that settled the axis/sign question in one render, the shipped lean_window numbers and how pose couples with placement inset, the per-class pose table, the composed-geometry acceptance rules (a solve-file z is NOT evidence), and the people bench that is the only oracle that counts.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Author human poses (and prove them)

Built 2026-09-01, over eight bench render rounds for the urban-fire people
pass. Everything here was paid for; read it before authoring or "fixing" any
pose.

## The one-sentence version

**Never trust a pose you have not rendered.** Joint-angle authoring against
the UE4/Epic skeleton has a frame-convention gap somewhere between what the
numbers say and what the skinned rig does, and two independent paper
derivations — one from skeleton documentation, one sign-matched against
already-approved shipped poses — both produced bodies the user rejected on
sight ("look like they're about to jump off the roof", "stomachs out, bent
the wrong way, arms look wrong"). The diagnosis pass that cross-checked every
sign against approved poses CONCLUDED THE MATH WAS FINE and the render still
disagreed. The render is the only oracle.

## Where things live

| what | where |
|---|---|
| pose tables + joint transform application | `scene_gen/disaster/people.py` (`_pose_joint_transforms`, the pose dicts, `WINDOW_POSE_DIAGNOSIS` — the written-up negative result) |
| classes, placement geometry, 36-rule gate | `scene_gen/disaster/fire_people.py` |
| the bench (4 buildings, per-figure closeups) | `simulation/isaac-sim/launch_scripts/fire_people_bench_launch_script.py` + `scene_gen/_plans/people_bench/` |
| the retired-but-kept pose experiment | `window_pose_experiment` knob + `_pass_window_pose_experiment` (fire_people.py) |
| rigs (LOCAL mirror — no Nucleus dependency) | `scene_gen/assets/people/` (6 rigged `rp_*_ue4.usd` + 3 posed statics; `fire_people._people_asset` prefers local, falls back to Nucleus) |

## The empirical POSE ROW method (use this, not derivation)

When a new pose is needed, do NOT reason about axes. Do this:

1. Author 6-8 THROWAWAY variants in the pose table, systematically varying
   the hinge axis and sign on the candidate joints — for the window lean this
   was: pelvis ±X, pelvis ±Y (null-axis control), spine-chain ±X (the
   mechanism the old poses used, both signs), pelvis+spine mixed ±X. Name
   them by mechanism (`lw_pelvis_posx`, ...), FK-sanity-check numerically
   (joint limits, no interpenetration at rest) and nothing more.
2. Place ONE figure per variant at consecutive comparable anchors (the bench
   used consecutive window bays of one building), behind a default-off knob
   so a normal solve never sees them.
3. Render the bench ONCE with labelled per-figure closeups
   (`<class>_<nn>_<variant>_closeup.png`).
4. Pick by eye. Ship the winner, retire the scaffolding (keep it in the code
   behind the knob — it is cheap and the next pose question reuses it).

One render settled what three derivation rounds could not: **pelvis +X** is
the forward lean out of the wall plane. Spine-chain hinges at the same
magnitude put the head THROUGH the glazing (reads headless from the street);
±Y tips sideways; mixed reads as standing-looking-down.

## The back-arch / face-up failure — how the mapping was actually pinned

What the bad renders looked like, so you recognise them: the window figures
stood with the SPINE ARCHED BACKWARD ("stomachs out, bent the wrong way" —
user), FACE TILTED UP AT THE SKY, arms swung down-and-out. That is the
signature of a sagittal hinge composed with the OPPOSITE sign to intent —
the pose author meant "fold forward at the hips", the rig did "arch
backward", and the neck/head counter-angles (authored assuming a forward
fold) then pointed the face skyward instead of at the street.

How it was diagnosed — and why the obvious check was NOT enough:

1. The first instinct was "our angles must disagree with the UE/Epic
   skeleton convention — compare against poses that WORK." The diagnosis
   pass replayed the actual bake-time function (`people._pose_joint_
   transforms`) against the real rig and cross-checked every joint's sign
   against three shipped, user-approved poses (`dig_bent`, `stand_slump`,
   `walk`). **Every sign matched precedent.** On paper the new pose was
   composed exactly like the approved ones. This negative result is written
   up as `WINDOW_POSE_DIAGNOSIS` in `people.py` — read it before repeating
   the exercise.
2. The reason sign-matching against approved poses proves nothing: the
   Epic skeleton's joint LOCAL frames are not body-axis-aligned and differ
   per joint (a pelvis "+X" is not the same body direction as a spine_01
   "+X"), and the approved poses exercise different joints in different
   regimes — agreement on the joints they happen to share does not
   constrain the joint you are actually bending. There is no shortcut
   through the conventions; the composed skinned result is the only ground
   truth.
3. So the question was settled empirically: the pose row rendered pelvis
   +X and −X side by side, and the −X figure reproduced the back-arch /
   face-up failure EXACTLY while +X folded forward. That one frame both
   identified the correct sign and explained every earlier bad render.

The face-up half has its own lesson: head direction is authored as a
COUNTER-rotation on top of the hinge (`lean_window` ships pelvis +30° with
neck −16° / head −9°, net head ≈ +5°, face level and looking out). If the
hinge sign is wrong, the counter doubles the error — the face goes from
"slightly up" to "staring at the sky" — which is why the face reads as the
loudest symptom of a flipped hinge. When a rendered figure looks skyward,
suspect the HINGE sign first, not the neck angles.

## The shipped poses, per class

| class | pose | the reasoning |
|---|---|---|
| roof / roof_victim | plain `idle` / `stand_calm` (upright rest) | anything bent (crouch, sit_edge) at a roof edge reads as about-to-jump — user-rejected. Standing, CENTRED on the deck (>=2-3 m inset from every parapet), never on the lip |
| window | `lean_window`: pelvis +30°, arm swing −55°, neck −16° + head −9° (net head +5°, face looks OUT not at the pavement) | the pose-row winner plus a subtle head counter-rotation |
| casualty_apron / roof_debris | `lying_*` family + `buried_reach` | the tornado module's burial poses; sink + covers do the storytelling, not the pose |
| interior_trapped | `stand_calm` (conscious, one storey BELOW the break) and `lying_supine/prone` (passed out on surviving slabs, 1.5-3 m back from broken edges) | a standing figure at the break line reads as a ledge-stander from outside |

**Pose and placement inset are COUPLED.** `lean_window`'s smaller head
displacement (0.152 H vs the old 0.190 H) dropped facade protrusion under
`MIN_PROTRUSION_M` at the old 0.17 m inset — it carries its own
`lean_window_inset_m = 0.09`. Changing a pose's reach means re-deriving its
class's inset, not re-tuning the angles.

## Placement rules that took a round each to learn

- **Windows: measured openings only.** `_pass_window` prefers sides with
  sidecar-MEASURED openings; the derived/synthetic grid is a last resort for
  buildings with zero recorded openings. Equal-probability side choice put
  6/6 leaners on blank party walls (assumed windows) while the real glazed
  elevation stayed empty — and the same bug mis-placed all 21 city window
  figures. Gate rule `window_figure_is_in_an_opening` re-derives the opening
  rect and asserts the figure's chest point is inside it.
- **No window figures in the top 3 storeys, ever** (`window_below_the_top_
  storeys`) — combined with the above-the-fire rule the eligible slice is
  `band_top < storey <= n_top - 3`; a building with an empty slice gets NO
  window figures rather than a bent rule. High leaners read as ledge-standers
  on broken roofs.
- **Roof z is sampled from the actual deck geometry** (points-based, at the
  figure's x/y) — bbox/coping values float figures at the railing lip on
  parapeted buildings.
- **No roof-standing figure of ANY class on any breached roof** — F5/F5c/F6,
  burn-through holes, partial loss: all of it, both `roof` and `roof_victim`
  (they are separate classes; the leak was one of them missing the check).
- **Covers must CONTACT the body.** Tornado `_BODY_RISE` crest fractions are
  calibrated for tornado context and sat 0.41 m above this module's own
  `lying_lift` — covering debris hovered mid-air. Crest is clamped to
  `lift * 2.2` and the gate (`burial_covering_contacts_the_body`) fails any
  gap > 0.05 m per figure (`over_record_id` links each cover to its body).

## Acceptance is COMPOSED GEOMETRY, not solve-file numbers

Two rounds were lost to "verified offline" claims that checked the wrong
thing. The rules that ended it:

- A lying figure's evidence is the COMPOSED mesh AABB bottom vs the support
  surface (±0.03 m), through the same transform+pose path the launcher uses
  — the solve-file z was correct all along while the render floated 0.4 m
  (over-applied lying lift at placement time).
- A cover's evidence is its underside vs the COMPOSED body crest (touch to
  −0.02 m overlap) — "gap 0.20-0.25 m" was once reported as contact.
- A pose's evidence is a render. See the one-sentence version.

## The bench loop

`fire_people_bench_launch_script.py`: 4 real city bakes referenced in a row —
roof-collapsed GAC tower / intact kit F3 / partial-collapse kit F5c / a tall
window-showcase tower — all people classes exercised, per-figure closeups at
2 angles (tight portrait; street-level 12 m @ 1.6 m eye for window; 35°
elevated for burials so a random lie-down yaw can't hide the figure), plus a
composed-vs-solved cross-check that ABORTS if any people-carrying building
failed to compose (people placed at a missing building's cell = figures
floating in empty sky; it happened, it is now unreachable). Iterate
agent → relaunch → closeup audit and only show the user a bench that already
passes; each user round costs more than five internal ones.
