# Thermal temporal matcher: bounded preflight

12 November 2026. Static inspection only; no new model download, data recovery,
training, inference or accuracy scoring. Follows the [continuation decision](continuation-2026-11-12.md).

**Decision: stop this pilot under its current direct-correspondence requirement.**
The software interface is usable, but the inspected evidence does not establish
the correspondence-error target required before execution. This closes this
proposed pilot, not temporal matching or the broader RGB–thermal study.

## Interface findings

`Module/Frontend/Frontend.py` defines `estimate_pair(t1,t2)` as target-frame
depth plus left-camera flow from t1 to t2. `Matching.py` expects float32
`B×2×H×W` pixel displacements, optional `B×3×H×W` covariance channels
`uu,vv,uv`, and an optional boolean validity mask. Pixels use `(x,y)` order;
the retrieval helper uses integer indexing rather than interpolation.

`Odometry/MACVO.py:run_pair` uses `kp1 = kp0 + flow(kp0)`, rejects inbound
failures, and samples depth/covariance at those target coordinates. Flow also
enters the motion initializer and keypoint selector. Thus keeping network depth
maps and solver code fixed does not keep sampled target depths, selected points
or propagated observation covariance fixed. Any future comparison must declare
whether it measures the total replacement effect or a fixed-point intervention.

A candidate matcher's covisibility is not an observation covariance. Reusing
the original model's covariance for changed correspondences would be an explicit
experimental control, not calibrated uncertainty for the replacement model.

## Target findings

The MS2 loader constructs image/calibration/timestamp frames without assigning
the optional `gt_flow` or `flow_mask` fields. Its separate `depth()` method
returns projected depth with zero-invalid masks; these are not temporal
visibility annotations.

The retrieved final-residual backup manifest inventories 256 archive members
and 236 external frozen inputs. A filename screen finds no `gt_flow`,
`flow_mask`, `occlusion`, `visibility` or `optical_flow` entries. This is only
an inventory screen, not inspection of every tensor's contents or a claim that
no other Box/publisher packet contains such data.

Depth plus supplied pose could produce a projected static-scene displacement
proxy, but does not certify visibility, static point identity or physical timing.
The previous [exact-depth support study](depth-label-retrieval-findings-2026-11-09.md)
also measured only one jointly depth-supported native thermal factor across
19 map-complete rain pairs. That rules out reusing its particular native-factor
support as a substantial direct-error sample; it does not establish global
sparse-label density or rule out an independently specified source-pixel assay.

The existing nominal motion evaluation remains available in principle. The
missing piece for this proposal is its additional direct correspondence
measurement, not the ability to run VO or compute a supplied-reference score.

## Consequence

Do not install a new matcher, restore its runtime or reconstruct temporal labels
to rescue this pilot. Reopening would require either an already-existing named
correspondence/visibility target or an explicit change to a downstream-only
development comparison. The latter would test whether replacement improves
motion, without claiming that correspondence error itself was measured. It
would need a separate bounded protocol before execution.

Source hashes, scope and inventory counts are recorded in
[the preflight receipt](../../experiments/temporal-matcher-preflight-2026-11-12.json).
No runtime behavior or historical scientific tests were rerun.
