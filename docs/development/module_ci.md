# Module CI: the reusable system-test workflow

AirStack module repos (`asm_*`) do not copy trunk's test suite — they call it.
Trunk owns a reusable GitHub Actions workflow,
[`module-system-tests.yml`](https://github.com/castacks/AirStack/blob/main/.github/workflows/module-system-tests.yml),
that checks out `castacks/AirStack` at a pinned ref, checks out the calling
module repo next to it, registers the module (`airstack module add` +
`airstack module sync`), and runs the **existing system-test suite unchanged**
on a GPU runner ([RFC #379 §5](https://github.com/castacks/AirStack/discussions/379)).
A module's CI is therefore a ~12-line caller.

## Wiring a module repo

Create `.github/workflows/ci.yml` in the module repo:

```yaml
name: module-ci

on:
  pull_request:
  workflow_dispatch:

jobs:
  system-tests:
    uses: castacks/AirStack/.github/workflows/module-system-tests.yml@v0.19.0
    with:
      airstack_ref: v0.19.0    # pin and checkout ref move together
      marks: "build_packages or liveliness"
      sim: isaacsim
    secrets: inherit           # castacks org secrets → registry image cache
```

Pin the workflow ref (`@v0.19.0`) and `airstack_ref` **together** — the
workflow version and the trunk it tests should move as one. During
pre-release development both may point at a branch; re-pin to a release tag
as soon as one exists.

`secrets: inherit` passes the castacks **org secrets**
`DOCKER_REGISTRY_URL` / `DOCKER_REGISTRY_USERNAME` / `DOCKER_REGISTRY_PASSWORD`
through to the workflow. The registry is the internal
`airlab-docker.andrew.cmu.edu/airstack` (**not** ghcr); with the secrets
present, image prep pulls versioned and floating cache tags instead of
cold-building (see [Layer cache: the floating `cache_*` tag](intermediate/testing/ci_cd.md#layer-cache-the-floating-cache_-tag)).
The secrets are optional — without them the run still works, just slower.

## What the workflow does

1. Refuses callers outside the `castacks` org (first-party policy, below).
2. Checks out `castacks/AirStack@airstack_ref`, submodules included.
3. Checks out the calling module repo (at the triggering SHA) into
   `module-under-test/`, submodules included.
4. Validates `module.yaml` with `tools/validate_module.py` — an invalid
   manifest fails the run before any GPU time is spent.
5. Registers the module: `airstack module add ./module-under-test` +
   `airstack module sync`, then runs the module's declared
   `hooks.host_setup` (hooks are idempotent by manifest contract, so the
   explicit run is safe even when `module add` already ran it).
6. Preps images (registry pull → floating-cache retag → build fallback),
   creates the Isaac `omni_pass.env` when `sim` includes `isaacsim`.
7. Runs `pytest tests/ -m "<marks>" --sim <sim> ...` exactly as trunk's
   `system-tests.yml` does — same log-cli flags, same
   `run_meta.json` honesty gate (a collect-only campaign is not a pass).
   `build_packages` is auto-prepended to `marks` when absent, so code is
   always built before launch tests run. If the module repo has a
   `tests/` directory (configurable via `module_tests_dir`), it is appended
   as an additional pytest path — module-specific system tests run in the
   same session.
8. Uploads `tests/results/` as artifact
   `module-test-results-<module>-<run_id>` (90-day retention) and writes a
   job summary (module, refs, marks, sim).

## Inputs

| Input | Default | Meaning |
|---|---|---|
| `airstack_ref` | *(required)* | Tag/branch/SHA of `castacks/AirStack` to test against. Must include the module CLI (`airstack module add`/`sync`). |
| `marks` | `build_packages or liveliness` | pytest marks expression (see [tests/README.md](../../tests/README.md)). `build_packages` auto-prepended when absent. |
| `sim` | `isaacsim` | Sim targets, comma-separated: `isaacsim,msairsim`. |
| `num_robots` | `1` | Robot counts, comma-separated (e.g. `1,3`). |
| `stress_iterations` | `1` | Iterations per (sim, num_robots) config. |
| `stable_duration` | `120` | Seconds for the `test_stable` polling window. |
| `module_tests_dir` | `tests` | Module-relative dir of extra pytest tests; appended to the pytest paths only if it exists. |
| `runs_on` | `["self-hosted","airstack-ephemeral"]` | JSON array string parsed into `runs-on`. |
| `timeout_minutes` | `120` | Job timeout. |

| Secret | Required | Meaning |
|---|---|---|
| `DOCKER_REGISTRY_URL` | no | Internal registry host (castacks org secret). Enables registry-cache image prep. |
| `DOCKER_REGISTRY_USERNAME` | no | Registry username (org secret). |
| `DOCKER_REGISTRY_PASSWORD` | no | Registry password (org secret). |

Runs are **read-only against the registry cache**: modules can consume the
floating cache tags but never republish them (`AIRSTACK_REGISTRY_CACHE_PUSH`
stays unset; only trunk's `docker-build.yml` writes).

## First-party policy

The workflow hard-fails when the calling repository is not in the `castacks`
org. Org GPU runners (OSMO pool, sim licenses) and org registry secrets never
serve third-party code. External modules will be served by a
**dispatch-triggered test bench** (RFC #379 §5, Phase 4): trunk receives a
`repository_dispatch {module_repo, module_ref, airstack_ref}`, runs on its own
runners, and posts a check-run back via a GitHub App — secrets and licenses
never leave trunk, bench time is gated and rate-limited. Until that lands,
external authors can run the suite on their own runners by overriding
`runs_on` in a fork of the workflow, but they get no castacks compute.

## Which marks should a module run?

System tests double as conformance tests — e.g. `tests/waypoint_checker.py`
judges the odometry track regardless of which planner produced it, so passing
`waypoint_flight` *is* the behavioral definition of a working global planner.
Guidance by module category (RFC #379 §5):

| Module category | Marks |
|---|---|
| Global planner | `waypoint_flight`, `autonomy` |
| State estimator | `liveliness`, `sensors`, `takeoff_hover_land` |
| World model / perception | `liveliness`, `sensors` |
| Sim extension | `liveliness` (on the affected sim) |

Declare the chosen marks in the manifest (`tests.marks` in `module.yaml`) so
the claim is inspectable; the CI caller is where they actually run.

## Cost ladder

- **Every push:** `unit` + `build_packages` — the module's own business, run
  however the module likes (a plain `ubuntu-latest` job with the published
  image, or `marks: build_packages` through this workflow — pure
  `build_packages` runs never bake sim images). Minutes, no GPU.
- **PR / nightly:** `liveliness` (plus `sensors` where relevant) via this
  workflow; `sim: msairsim` is the cheap bring-up.
- **Release / compatibility claim:** the module's full conformance mark set on
  GPU — this run is what stamps the badge.
- **Trunk-side nightly canary** (Phase 4) runs registered modules against
  `develop` so breakage surfaces the day it lands.

**Badge semantics:** a compat badge reads "module M @ vM passes marks {…} in a
test stack derived from reference stack S, on AirStack vX" — conformance to a
*stack*, demonstrated by flying it.

## Weekly canary (recommended)

Until the trunk-side canary exists, give the module repo its own cron so drift
against trunk `develop` surfaces weekly instead of at the next release:

```yaml
on:
  schedule:
    - cron: "0 6 * * 1"   # Mondays 06:00 UTC

jobs:
  canary:
    uses: castacks/AirStack/.github/workflows/module-system-tests.yml@develop
    with:
      airstack_ref: develop
      marks: "build_packages or liveliness"
      sim: isaacsim
    secrets: inherit
```

A red canary is a compatibility signal, not necessarily a module bug — check
trunk's changelog before touching module code.

## Smoke-testing the workflow itself

Trunk maintainers can run the workflow against any module repo without a
caller, via `workflow_dispatch` (extra inputs `module_repo` + `module_ref`
select the module):

```bash
gh workflow run module-system-tests.yml --repo castacks/AirStack \
  --ref develop \
  -f module_repo=castacks/asm_dfm2_disturbances -f module_ref=main \
  -f airstack_ref=develop \
  -f marks="build_packages or liveliness" -f sim=isaacsim
```

## GPU runners for module repos

With the default `runs_on`, jobs queue for the label pair
`[self-hosted, airstack-ephemeral]` — the ephemeral OSMO-backed runners. The
orchestrator polls **one repo per instance**, so a module repo must be added
to the poll list before its jobs are picked up: see
[CI/CD Orchestrator → Module repos](../../tests/ci-cd-orchestrator.md#module-repos).
