# MS² recovery audit — 2026-10-25

Branch `ore_ms`. Checkout initially ended at `ef6ca6f57` (EXP043). Authenticated Box contained newer completed EXP044 evidence. Verified and recovered commits `a0ec22920` and `018e7dbcf` by fast-forward from the incremental bundle, rather than rerunning the completed experiment.

## Recovery and checks

- EXP043 archive SHA256 `5658f69f6b2b186453003728a5a56e78b65cc5b523797c2ec204100f0e657fd9` and all277 member hashes pass.
- EXP044-complete archive SHA256 `1c62a44b23eb554485a07464d5ac4a9e70771b8cc99dbf613f785be4e47e81cd`, bundle/README hashes and all109 member hashes pass. Restored files preserve existing matching content; differing files are rejected.
- Original shared checkpoint archive restored through EXP044's documented shared-link protocol; archive SHA256 and ZIP CRC pass. RGB and thermal checkpoint hashes match EXP044 exactly.
- Both runs' frozen source/input hashes and all78 output-file hashes pass.
- Rebuilt `macvo:blackwell` image `8972864d2d7053e3f257e9fb87762cdf9b971f20bb44a1a75c494567c96eda7c` and `macvo:ms2-smoke` image `8dc9fef8c6da18d5a05d7fc44f53b27c932ed67bfc42d2fccf60fd08cc0a29a8`. Rebuilt image identities differ from historical images; the base Dockerfile has unpinned transitive dependencies.
- Runtime Python3.11.13, Torch2.7.1+cu128, OpenCV4.10.0, NumPy2.2.6, PyPose0.7.3. All14 adapter tests pass with one upstream PyPose docstring deprecation warning.
- Fresh independent audits of RGB and thermal saved runs pass:39 saved pairs each;80 source images and80 producer-model tensors checked per audit; four historical first-input tensors match exactly. Repeat-bitwise flags are historical evidence re-read by the validator, not newly executed model repeats.
- CUDA matrix multiplication passes on RTX PRO5000 Blackwell. Startup driver diagnostics remain in logs; no new model inference was run.

## Reproduce the recovery audit

Restore `backups/2026-10-23-exp-043` and `backups/2026-10-24-exp-044-complete` per their manifests and RECOVERY files. Checkpoints use the shared-link recovery documented in EXP044. Full40GB MS² source archives and historical TD3 archives are not required for the bounded replay and were not restored.

From `ore-ms/MAC-VO-main`, in `macvo:ms2-smoke` with AirStack mounted at `/root/AirStack`:

```bash
python -m pytest Scripts/UnitTest/test_ms2_adapter.py -q
python Scripts/Setup/validate_ms2_frontend_smoke.py \
  --root /root/AirStack/ore-ms \
  --manifest /root/AirStack/ore-ms/results/ms2-timestamp-semantics/source-protocol-input-manifest.json \
  --run /root/AirStack/ore-ms/results/ms2-frontend-smoke/rgb-run-001 \
  --output /path/to/fresh-rgb-audit.json
```

Repeat validation with `thermal-run-001` and a fresh output path. Fresh audit JSON, test/runtime/CUDA/build logs and the session design/results notes are preserved under Box `backups/2026-10-25-recovery-audit`. Credentials, signed URLs, packages and weights are excluded.

## Research state

EXP044 runtime compatibility is complete. The next scientific work remains an explicitly justified timing/reference protocol before physical RGB/thermal error comparisons. Exposure timing, pose-reference direction and checkpoint training provenance remain unresolved; this recovery introduces no new timing correction, calibration, VO, fusion or accuracy claim. No additional image upload is needed to replay this bounded evidence.
