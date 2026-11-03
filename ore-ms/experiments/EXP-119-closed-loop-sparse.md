# EXP119: native sparse feedback

2026-11-03. Freeze `987c7c96e`; parent EXP118. [Full findings](../docs/research/closed-loop-sparse-findings-2026-11-03.md).

Seven dual-camera captures (fourteen native experts) execute from their own returned observations and controller clock. No skipped frame is decoded or fed to the native estimator; gap endpoints remain explicit and cannot masquerade as adjacent deliveries. The per-window development contract passes. Actual pair calls 434 → 334; synchronized native processing 150.344 → 109.043 seconds. Availability, matched first-write proxy errors, exclusions and simple fixed-camera context are retained in findings.

Twenty-five tests and 32,671 independent checks pass across 410 core rows; 1,413 execution pins unchanged. Strict parity: 12,584 NPZ arrays, 0 mismatches. No training, reserves or defaults changed. Serial offline feedback is not live-queue, physical-accuracy, recovery or generalization validation.

Next boundary: separately frozen availability transitions with explicit fixed-camera controls and correct adjacent/gap semantics. Existing exposed inputs suffice for bounded engineering work. Backup `backups/2026-11-03-closed-loop-sparse`; images/weights/compiler EXP112, base runtime EXP106, parity controls EXP118.

Both arms deliver 204/205 guarded selections with identical matched proxy errors. Each window keeps one active expert throughout scored operation; the selected stream stays dense and the other is probed. Full-read thermal alone is eligible on 205/205 slots, so no adaptive or recovery advantage is established.
