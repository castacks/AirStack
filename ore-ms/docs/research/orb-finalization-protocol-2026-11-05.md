# EXP144: bounded received/final ORB output-stage control

2026-11-05 America/New_York. Parent EXP143/33bc82f81. First 2,000 original source
groups 000000–001999 of exposed night `_2021-08-13-21-18-04`, RGB and thermal.
Fixed engineering prefix, not selected from new error values. References and
earlier full-record outcomes are already development-exposed.

Eight sequential one-attempt native arms: per camera, original ORB control,
original repeat, augmented ORB paired received/final export, and fresh DROID on
the same 2000-group prefix. Seed0, loop closure off, existing settings/inputs,
3600-second native timeout, no outcome-driven retry/tuning. DROID uses unchanged
quarter-memory 2048-slot guard and identical weights/geometry. Its full-record
saved poses are not a substitute: the future-data horizon must match the prefix.

Build a derivative of the exact EXP134 runtime by adding four inline read-only
pointer getters to System.h and compiling a new wrapper only. No native library,
class fields/virtual methods, vocabulary, estimator .cc source or numerical settings
change. Preserve the original native executable. Native library/source/ABI identity
is mandatory. One fixed 20-source RGB original/augmented integration pair may be
run before the scientific freeze, without reference scoring; retain failures.

The pinned Shutdown requests finish but its wait loop is commented out. Explicitly
wait up to60s for local mapper/loop closer finished and no GBA. This is a native
post-worker snapshot, not a requested new global BA or proof of convergence.
The default EuRoC exporter chooses only the largest map. The new exporter visits
every surviving map, retaining original IDs and its first surviving keyframe gauge.
Use the native final relative-frame/keyframe history, culled-reference parent chains
and final keyframe poses. Compose the native float matrix snapshots in float64 and
invert the product. Preserve raw factors/gauges and per-map official EuRoC exports
for independent compatibility checks; do not choose the largest/favorable map.
Canonicalize only a homogeneous last row already within 1e-10 of [0,0,0,1];
reject larger deviations. Native float official-export compatibility uses a fixed
translation allowance 5e-5 + 5e-6*(1+sum factor translation norms) metres and
rotation-component allowance 5e-6*(chain length+3). Primary float64 factor
reconstruction is checked at rtol1e-10/atol1e-8. No source-dependent fit.

Map native history times to the exact double times originally passed from integer
source differences. No nearest-time search or fitted time/scale/frame transform.
Reject missing, lost, duplicated/ambiguous history, changed/removed maps, null/cyclic
parents or invalid pose serialization. Never resurrect a source slot absent from
the paired received tracking-OK output. All expected source slots remain present.
Received/final are paired within one augmented run. Original controls/repeats
characterize asynchronous native variability; do not assume bitwise repeatability.

Freeze source, binary/runtime, input/config, budgets, export/score/audit definitions
before the eight scientific attempts. Lock all predictions before scoring. Use
the common exact valid support of augmented received, finalized and prefix DROID,
preserve received native map/missing boundaries and further split at final/DROID
holes. Original individual coverage stays visible. Unchanged v2 provided-reference
ATE/RPE/rigidity rules; no interpolation, scale/clock/extrinsic fit, new training,
reserve access, adopted selector or physical/online/heldout benchmark claim.

Independent validation reconstructs factors, map gauges, native/official export
direction, timestamp/source identity, masks, segments and raw-reference metrics.
Official float-format controls compare only unambiguous exact source bindings;
all ambiguous/unused rows are reported. Keep failures and all original native,
received, final, DROID, control/repeat and diagnostic namespaces unchanged.
