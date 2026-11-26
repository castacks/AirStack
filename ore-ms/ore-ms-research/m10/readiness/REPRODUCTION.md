# Reproduction

Run from `ore-ms/ore-ms-research/m10/readiness/` or the extracted bundle's `m10/readiness/`. The bundle contains the prior numeric inputs and cited research notes for offline reading. The earlier complete milestone archives remain separately preserved. Only standard Python, Bash, tar and SHA-256 commands are needed.

```sh
sha256sum -c PACKAGE_SHA256SUMS.txt
python3 -B audit.py | cmp evidence/numeric_audit.csv -
bash reproduce.sh /path/to/prior-archives | cmp evidence/archive_integrity.log -
```

The archive path contains `ORE-MS-EXP-PER-001-M01-R2.tar.gz` followed by `M02-R1` through `M09-R1`. The replay extracts to an isolated temporary directory, verifies root and applicable nested manifests, and compares the numeric input files with the frozen archive members. It recalculates 39 controls without running or modifying any model. Earlier full replays are documented in the accepted packages.

To verify scope in Git, compare starting commit `67fe6454963856ba39596679d5905399089c20f3` with the readiness commit. Only this directory and the live roadmap entry are expected. The work breakdown, dependency table and acceptance register compare byte for byte. The repository [scope log](evidence/scope.log) records the verified counts and original start. For the downloadable bundle, verify its root manifest before running these commands.
