# RRM F00 engineering workspace

M00 establishes repository control and assesses the supplied F00-EXT package. All new engineering deliverables belong here on `ore_proj`; only the Program Director pushes. WP01 has not started and requires M00 technical review.

Start with [M00 assessment](verification/reports/M00_ASSESSMENT.md), [G0 recommendation](verification/reports/M00_G0_READINESS.md), and [handoff](handoffs/M00_HANDOFF.md).

The user supplied `RRM-C001-F00-EXT/` in this directory during M00. It remains a read-only local input, excluded from the engineering commit. Its 48 files are inventoried in [the controlled-package manifest](evidence/M00/controlled-package-manifest.json). The package is an EXT preparation draft that quotes frozen F00/SRS sources; it does not include the original frozen qualification matrix, full SRS, amendment approvals or BOUNDARY corrective package. These limitations remain explicit.

F00 remains **CLOSED / FROZEN / QUALIFICATION BLOCKED**. New evidence belongs to a successor campaign. Repository control documentation does not replace controlled requirements or authorize execution.

| Workspace | Purpose |
|---|---|
| governance | Directive, baseline identity and decisions |
| requirements | Requirement-source assessment and reconciliation |
| verification/matrices | Complete D2 row mapping, KPI and independent verification planning |
| verification/procedures | M00 checks and future evidence acceptance procedure |
| verification/reports | Assessment, limitations and G0 disposition |
| simulation | SIL/HIL/correlation plans; no runtime changes |
| integration | Read-only operational assessment |
| scripts | Standard-library M00 validation |
| tests | Validation scope and limitations |
| configurations | Observed versus controlled configuration comparison |
| evidence/M00 | Compact manifests and actual M00 observations |
| handoffs | Review and local commit handoff |

Run `python3 -B RRM/scripts/validate_m00.py` from the repository root. No dependency installation or service startup is needed. Raw recordings, model weights, images and caches do not belong in Git; record external locations and SHA-256 in evidence manifests instead.
