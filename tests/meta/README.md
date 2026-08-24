# Contract tests (`tests/meta/`)

Fast, hermetic **contract tests** — plain pytest under the `unit` mark, no
Docker, no GPU — that pin the modular-AirStack (RFC #379/#380) contracts:
the `airstack` CLI's derived configuration, the `module.yaml` manifest and
workspace overlay, stack folder anatomy, fleet files, the bridge → DDS-router
generation, the doctor gates, the generated docs catalog, and the metrics
reporting semantics. A "contract" here is a promise another part of the
system (CI, docs deploys, an agent workflow, a module repo) relies on;
these tests exist so a refactor cannot silently break the promise. They run
in every `airstack test -m unit` invocation and in the `unit-tests.yml` PR
gate.

Each `test_*_contract.py` file names the contract it pins in its module
docstring — start there. Fixtures (e.g. the registry snapshot under
[`fixtures/modules_index/`](fixtures/modules_index/)) are committed copies of
the external inputs the contracts were blessed against.

Two naming notes:

- **`tests/meta/` vs `tests/harness/run_meta.py`** — unrelated despite the
  shared word. This directory holds contract tests ("meta" as in tests about
  the system's contracts rather than its flight behavior);
  `harness/run_meta.py` writes the `run_meta.json` **run metadata** file into
  each results directory. Neither imports the other.
- **The lint outlier** — [`test_launch_single_locus.py`](test_launch_single_locus.py)
  is not a fixture-based contract but a repo-wide **lint**: it walks the real
  launch tree and enforces the single-locus wiring rule (topic wiring lives
  only in stack entry files), with escape hatches listed in
  [`launch_lint_allowlist.txt`](launch_lint_allowlist.txt).

See the [main testing README](../README.md#meta--contract-tests-testsmeta)
for the one-line-per-file index.
