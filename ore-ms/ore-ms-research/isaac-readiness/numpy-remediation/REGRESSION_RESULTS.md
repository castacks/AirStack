# Regression results

| Check | Expected | Observed |
|---|---|---|
| Late Dockerfile assertion | Isaac Python imports NumPy 1.26.4 | The build completed the final pip and assertion layer. |
| Independent positive image check | Disposable candidate container exits zero | Isaac Python 3.11.13, NumPy 1.26.4, exit zero. |
| Independent negative control | Prior pristine image is rejected | Isaac Python 3.11.13, NumPy 2.4.6, exit 1. |
| Compose resolution | Standard and livestream match the override or default | Both matched with the variable unset, set to the candidate tag, and set to a digest. Robot and GCS references matched across all three cases. |

The negative control uses the previously built image configuration ID from the R1 readiness audit, without touching the repaired container. The import-only Docker checks do not request a GPU and the base wrapper emitted driver-initialization text. Both reached the Python version check. See the [positive output](evidence/local-numpy-test.log), [negative output](evidence/prior-numpy-test.log), and [Compose resolutions](evidence/compose-resolution.tsv).
