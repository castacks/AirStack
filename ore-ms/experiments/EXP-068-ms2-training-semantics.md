# EXP-068: uncertainty training semantics and checkpoint provenance

Date:2026-10-28 America/New_York. Base b727cb67e, branch ore_ms.

The available covariance training code and inference interface use incompatible raw-output interpretations. Inference returns exp(2*raw); current cov_loss treats raw+epsilon as variance without that exponential. Mask and max_cov arguments are unused in cov_loss. Exact-source CPU probes and an independent audit confirm those facts, and public upstream matches the relevant local methods. **This does not establish how either supplied checkpoint was trained or explain the observed transfer failures.** No fix, training, neural image inference or default change was made.

See the [complete semantics/provenance note](../docs/research/uncertainty-training-semantics-2026-10-28.md) for the source chain, mathematical implication, primary-paper links, exact checkpoint hashes and limits.

## Concrete checks

| Contract probe | Observed result | Implication |
|---|---|---|
| Residual2, raw0.5, epsilon1e-7 | Current covariance loss7.30685141944 | Uses squared error/(raw+epsilon)+log(raw+epsilon) |
| Same inputs with inference variance exp(1) | Consistent unhalved Gaussian objective2.47151776469 | Different objective from current trainer |
| Raw-variance point optimum for squared error4 | raw=3.9999999 | Inference would exponentiate this quantity |
| Log-standard-deviation point optimum for same error | raw=log2=0.69314718056 | Inference variance would equal4 |
| All-false mask and tiny max_cov | Loss unchanged | Arguments unused by cov_loss |
| Negative raw=-0.2 | Training loss NaN; inference variance0.6703200460 | Inference permits raw values that loss cannot accept |
| Detached squared residual | No direct gradient to supplied prediction leaf; covariance gradient=-3.4999985 per element | Covariance objective directly supervises scale only |
| Current cov training freeze block | Only cov_update trainable in structural stub | Separate covariance-stage intent; no historical-run proof |

RGB checkpoint is byte-identical to the official77,369,368-byte release (SHA bec6edd7...). Thermal SHA eed20722... is the unchanged supplied76,827,690-byte file. Containers have434/428 tensor entries and only standard version-key module metadata, without a structured training config/split/commit. The thermal thesis's method cannot bind its recipe to this particular file. No original-training or MS² training-held-out claim is newly established.

## Validation and preservation

PASS:16 source snapshots with SHA256/function AST/line registries; CPU float64 exact-source loss/inference/freeze probes, detached-residual and covariance gradients, negative raw input, mask/cap invariance, multi-step and final-only calculations. The inference probe used a stub forward and identity padding; it was not a learned network forward. Source snapshots and checkpoint bytes are unchanged. An initial harness used a two-channel mask where sequence_loss expects a broadcastable one-channel mask; the corrected probe uses B×1×H×W. Its incomplete source snapshot is retained separately and supplies no result.

Independent validation PASS:88 source/formula/AST/metadata/hash checks. Upstream pinned commit `8683b532d6f6e8981f6b1a646fb5f52f02c1ea61`: loss and trainer byte-identical; forward/inference AST-identical; local strict checkpoint loader is the known difference. RGB official-release streaming byte hash matches. Primary MAC-VO methods and thermal thesis selected training pages were inspected; full copyrighted PDF/text and weights are not backed up. This verifies code contracts and artifact identity, not historical training/calibration.

Tracked scripts: `ms2_training_semantics_audit.py` and `validate_ms2_training_semantics.py` in `MAC-VO-main/Scripts/Setup`. Results: `results/ms2-training-semantics/run-001/` with `local-audit/{source-audit.json,probes.json,checkpoint-containers.json,source-snapshots/}`, external pinned source/metadata and `validation.json`. Run the audit CPU-only inside existing `macvo:ms2-smoke`, with `--root /root/AirStack/ore-ms --output <fresh-folder>`; host analysis Python runs the independent validator with `--root ... --base <run-001>`. No additional package/model download is needed to replay source/checkpoint probes; original checkpoints are prerequisites in the existing private model recovery source.

Private EXP068 archive/bundle includes code,notes,source snapshots,small checkpoint metadata,notebook and validation. Credentials/checkpoint tensors/full PDF/transport archives excluded. No new capture data or selector evaluation. Existing saved results remain valid observations of the same inference interface. Final two captures remain untouched. Next: predeclared empirical scale/coverage diagnostic on existing disparity arrays; training recipe remains a separate provenance limitation.
