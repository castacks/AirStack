# Reproduction

From the extracted package root, use CPython 3.12.3 for the recorded byte-level check. All application dependencies are in the standard library.

```sh
sha256sum -c PACKAGE_SHA256SUMS.txt
export PYTHONDONTWRITEBYTECODE=1
out=$(mktemp -d)
python3 simulate.py scene.json "$out/observations.csv"
python3 verify.py "$out/observations.csv"
cmp evidence/observations.csv "$out/observations.csv"
```

In the repository, run the same Python commands from `ore-ms/ore-ms-research` and use a writable output path outside Git. Float formatting and PRNG behavior are verified on the recorded runtime. The evidence folder is included in the frozen package. The two checksums for raw captures and the byte comparison appear in the reproduction log.

The R2 downloadable package records the raw capture as `evidence/observations.csv`. To reproduce the evidence audit, redirect the verification command's output to `"$out/verification.log"` and run `python3 audit_reconciliation.py "$out/observations.csv" "$out/verification.log"`. In the repository the checksummed lightweight logs record the expected capture hash.

The [Research Roadmap Mandate](RESEARCH_ROADMAP_MANDATE.md) governs further execution.
