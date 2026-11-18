# ViViD++ varying-light learned proposal support

The [SIFT comparison](vivid-varying-sift-support-2026-11-17.md) returned
33 RGB and zero thermal proposals on 71 frozen half-second anchors. This
experiment asks whether that support pattern transfers to the existing
learned frontends. It uses the identical image headers, 25 ms support rule,
native calibrations and RGB grayscale/author thermal scaling of the SIFT
encoding control. Reference values were excluded from matching and routing.

Restored the full historical compiler runtime and verified Docker image
`sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09`.
The [official RGB checkpoint](https://github.com/MAC-VO/MAC-VO/releases/tag/model)
and the EXP112 thermal archive member match their expected SHA-256 hashes.
The first attempt stopped before inference because the harness required
no migration for both checkpoints. The retry accepted only the existing
audited RGB migration, which removes seven unused keys and initializes the
deterministic sampling grid. Thermal loaded without migration. Both strict
loads, repeated first-pair flows, finite positive covariance and unchanged
model-state hashes passed. The hardware was an RTX PRO 5000 Blackwell with
PyTorch 2.7.1+cu128. All 142 prediction rows passed a separate audit.

Used the repository's native bidirectional temporal flow, fixed 4,096-point
grid and one-pixel cycle gate, focal-400 calibrated ray lift and existing
essential/cheirality solver. No training, input substitutions, threshold
updates, or reference-dependent selection.

| Policy | Supported proposals / 71 | RGB calls | Thermal calls | Projected comparison time | Longest sampled no-output run |
|---|---:|---:|---:|---:|---:|
| Fixed RGB | 51 | 65 | 0 | 16.64 s | 6 anchors |
| Fixed thermal | 62 | 0 | 68 | 18.36 s | 4 anchors |
| RGB-first | 64 | 65 | 18 | 21.52 s | 4 anchors |
| Thermal-first | 64 | 9 | 68 | 20.67 s | 4 anchors |

Both cameras yielded a proposal on 49 anchors, thermal alone on 13, RGB alone
on two and neither on seven. RGB-first added 13 proposals on 18 thermal attempts.
Both priorities yielded 24/24, 22/24 and 18/23 proposals in the fixed
chronological thirds. The time column sums measured per-pair compute for the
replayed calls. It excludes model startup, acquisition and live routing.

Against the same-input SIFT result, learned RGB added 21 previously unsupported
proposals and lost three SIFT-supported proposals, net +18. Learned thermal
produced 62 proposals on inputs that gave SIFT zero. These are frontend support
counts. Geometrically supported proposals can be wrong, and the
[payload preflight](vivid-handheld-payload-intake-2026-11-17.md) leaves 1°
accuracy unqualified. No pose scoring, trajectory or universal sensor priority
follows.

The run evidence is in `/root/ore-ms-artifacts/vivid-plusplus-intake/`.
The local runtime/weights staging directory uses about 14 GiB and is
reconstructible from the verified sources. Docker has the restored image
with about 9.05 GB reported layer size before sharing. Both are cleanup
candidates after testing. Existing sensor bags were reused and no Box
storage was added.
