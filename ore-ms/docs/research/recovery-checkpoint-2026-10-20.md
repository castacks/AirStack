# Research recovery checkpoint

Recorded 2026-10-20 UTC on branch `ore_ms`. This checkpoint commits EXP-010 through EXP-017 code, tests, reports and handoff updates. Generated inputs/results and local notebooks remain excluded from Git.

Latest validation: 67 focused/regression tests pass under both OpenCV 5.0.0 and 4.10.0.84. VO defaults and original source inputs remain unchanged.

Persistent artifacts are in the [shared Box folder](https://cmu.app.box.com/folder/428559630945), under these snapshot directories. Restore prerequisites in order; each manifest contains member hashes and recovery requirements.

- `backups/2026-10-19-exp-010`: archive SHA256 `see manifest`.
- `backups/2026-10-19-exp-011`: archive SHA256 `see manifest`.
- `backups/2026-10-19-exp-012`: archive SHA256 `see manifest`.
- `backups/2026-10-20-exp-013`: archive SHA256 `see manifest`.
- `backups/2026-10-20-exp-014`: archive SHA256 `see manifest`.
- `backups/2026-10-20-exp-015`: archive SHA256 `70abf61a084802384ec9c03d616e0ce72afb14babbf4f34127948f031a481243`.
- `backups/2026-10-20-exp-016`: archive SHA256 `eb2d044417c5eb092f2db621c2a0bd9949c65fba011b86d85a3a363c24a1a845`.
- `backups/2026-10-20-exp-017`: archive SHA256 `326ce0399cbc0b978e34e6b785f70a71d9da73e11bd2e19d55a5598b564380ff`.

EXP-013 contains staged night images and metadata; EXP-014 contains recorded uncertainty pairs; EXP-015 contains geometric population arrays; EXP-016 contains temporal/context diagnostics; EXP-017 contains raw thermal samples, exact preprocessing replay, runtime metadata and preserved failed run. Original repository/model/submodule recovery remains a prerequisite.

Latest result: 18/18 fixed night thermal images reproduce pixel-exactly with the archived producer and OpenCV 4.10.0.84. Physical body/stereo calibration remains inconsistent. See [EXP-017](../../experiments/EXP-017-thermal-provenance.md) and [handoff](../../MS_RESEARCH_HANDOFF.md).

Next bounded check: day/night physical camera ordering and model-predicted stereo disparity signs against observed correspondences. Do not apply a blanket sign flip or claim independent ground-truth calibration.
