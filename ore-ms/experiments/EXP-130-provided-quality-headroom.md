# EXP130 — Provided-export quality headroom

Date: 2026-11-04. Branch: ore_ms. Parent: 4da8557cd. Pre-reference-pose source/protocol freeze: 6324a366f.

Question: do the fixed 323 jointly supported EXP129 intervals offer quality headroom beyond a fixed camera? Predictions, source frames and exact acquisition endpoints were frozen before reading reference poses. Recorded SLAM is a qualified nominal-body proxy, not physical ground truth.

Result: all323 score, zero exclusions. Rotation: RGB mean0.25021140deg, thermal0.23224070deg, retrospective oracle0.16556426deg; 28.710% reduction versus pooled best fixed. Against per-window best fixed0.21076723deg, reduction21.447%. Translation: RGB0.24859978m, thermal0.12710241m, oracle0.12693197m; reduction0.134% (0.17044mm). No joint metric, learned selector or adaptive accuracy claim.

Validation:5245 independent checks,8 independent nominal source-frame checks,5 semantic cases. Protocol pins2780inputs/sources; reference ZIPCRC/SHA verified. No native rerun, policy change, reserve payload, reference fit or MSA failure relaxation. Four windows/two development-exposed recordings; training overlap and reference dependencies unresolved.

[Findings](../docs/research/provided-quality-headroom-findings-2026-11-04.md) · [Protocol](../docs/research/provided-quality-headroom-protocol-2026-11-04.md)

Next: freeze a rotation-focused causal uncertainty/health signal audit, explicitly separating completed-output arbitration from pre-input scheduling.
