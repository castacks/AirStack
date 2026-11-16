# Frozen real-smoke frontend characterization

16 November 2026. Separate from the completed NGCLAB lighting run and its
unresolved camera/reference convention. No smoke predictions precede this plan.
Use the complete extracted SubT-MRS SmokeRoom ZIP selected in the data assessment.
Decode and hash every PNG, read both complete timestamp arrays, and declare
half-second adjacent anchor intervals across the whole RGB span. Each sensor
selects nearest native images, maximum anchor error 25 ms; retain missing support
in the denominator. No smoke-density/outcome window selection or clock fitting.

Reuse the frozen lighting adapter's RGB-to-gray and per-frame thermal min/max
uint8 rule, 4096-feature SIFT mutual-ratio matching, existing learned RGB/thermal
checkpoint loader, deterministic FP32 forward/backward flow, native-size 64×64 grid
and one-pixel cycle criterion. Images remain at their native sizes: RGB 640×480,
thermal 640×512. No retraining, checkpoint/threshold search or data augmentation.
The learned model variants differ by modality; SIFT is the common frontend control.

Primary descriptive outcomes: accepted temporal correspondences per interval,
fraction of learned grid surviving forward/backward cycle, fraction with at least
eight accepted correspondences, and fraction supporting a shared essential/
cheirality solve. These measure frontend availability and consistency, **not true
correspondence accuracy, metric VO, trajectory continuity or recovery**. Consistent
flow can still be wrong. Do not promote this as proof of thermal pose accuracy.

Use published RGB MEI and thermal pinhole calibration. Invert radial/tangential
image distortion iteratively. For MEI use the standard unified-camera sphere lift
(the released camodocal CataCamera liftProjective implementation is the primary
formula reference); normalize the forward-facing ray into a virtual pinhole.
Use focal 400 pixels with each image center as principal point for the common
one-pixel essential solver. Validate forward projection and lift round-trips over
the full query grid before estimates. Preserve source YAMLs; no extrinsics or
supplied integrated odometry is needed for these within-camera diagnostics.

Relate availability to minimum source/target RGB p95–p05 contrast with fixed
bins <40, 40–120, ≥120 on uint8 gray; report the full sequence and fixed thirds
of elapsed time, not chosen favorable smoke windows. Image contrast is a proxy,
not a measurement of smoke density: motion, illumination and scene changes also
matter. Different viewpoints, thermal normalization and pretrained-model domains
prevent a clean causal sensor effect. No smoke-free same-motion control is assumed.

Preflight whole-span inputs and projection, freeze source/protocol/models before
real matching, preserve incomplete attempts, lock predictions before summaries,
and independently recompute counts/denominators and geometry packets. Stop after
this recording plus the lighting cohort. A later pose study requires its own
qualified reference contract; do not silently reuse the failed lighting accuracy
interpretation or substitute a fitted camera-to-mocap transform.
