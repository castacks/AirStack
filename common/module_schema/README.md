# AirStack Module Manifest Schema

The schema for `module.yaml` — the **thin module manifest** from
[RFC #379 §2](https://github.com/castacks/AirStack/discussions/379): dependencies,
identity, and test metadata only. **Wiring deliberately does not live here.** A
module's interface is its launch file's declared args (with canonical-name defaults)
plus the interface conventions spec; its wiring lives in whatever stack includes it.

- Schema: [`module.schema.json`](module.schema.json)
- Validator: [`tools/validate_module.py`](../../tools/validate_module.py)
- Authoring workflow: [`.agents/skills/create-module`](../../.agents/skills/create-module/SKILL.md)

## Validate a module

```bash
# Module directory (schema + cross-file checks):
python3 tools/validate_module.py path/to/module_repo

# Bare manifest (schema only):
python3 tools/validate_module.py path/to/module.yaml
```

Human-readable errors go to stderr; a JSON verdict goes to stdout for scripts:

```json
{"valid": false, "errors": [{"path": "maintainer", "message": "required property is missing"}]}
```

Exit code 0 when valid, 1 otherwise. Contract tests:
[`tests/meta/test_module_manifest_contract.py`](../../tests/meta/test_module_manifest_contract.py).

## Fields

### Required

| Field | Shape | Notes |
|---|---|---|
| `name` | `^[a-z][a-z0-9_]*$` | module identity, snake_case |
| `description` | string, ≥ 8 chars | one-liner for the registry catalog |
| `maintainer` | email | required by governance (RFC #379 §8) |
| `license` | non-empty string | checked at registration |
| `type` | `isaac_extension` \| `ros_package` \| `data` \| `platform` | `platform` is future (RFC #380 §6) |
| `airstack_compat` | semver range | vs trunk `.env` `VERSION`, e.g. `">=0.19.0 <0.21.0"`, `">=0.19.0-alpha.18 <0.20.0"`. Branch names (`main`) and partial versions (`0.19`) are invalid. |
| `targets` | non-empty array of `robot` \| `gcs` \| `isaac-sim` \| `ms-airsim` | host containers touched |

### Optional (with defaults)

| Field | Default | Notes |
|---|---|---|
| `deps` | `{apt: [], pip: []}` | dependency tier 1 (RFC #379 §6); rosdep keys stay in `package.xml` |
| `dockerfile` | `null` | tier 2: repo-relative path ending `Dockerfile.module`, written against `ARG BASE_IMAGE` |
| `overlay_image` | `null` | tier 3: prebuilt overlay image reference |
| `compose` | `null` | repo-relative compose fragment (mounts, env) |
| `assets` | `[]` | `[{url, sha256, dest}]`: `url` must be `https://`, `sha256` is 64 lowercase hex chars, `dest` is a relative path (no `..`) |
| `docs` | `{readme: README.md}` | or `{dir: <path>, nav: <path>}` |
| `foxglove` | `null` | layout/panel fragment so operators can see the module |
| `hooks` | — | `{host_setup: <script path>}` — see hook contract below |
| `tests` | `{packages: [], marks: []}` | colcon unit-test packages + system-test marks from the known set (`unit`, `build_docker`, `build_packages`, `integration`, `liveliness`, `sensors`, `takeoff_hover_land`, `autonomy`, `waypoint_flight`, `optitrack`, `wiring`) |

Unknown top-level keys are rejected (`additionalProperties: false`) — in particular
slot/role/topic annotations, which the RFC deliberately excludes.

### Hook contract (`hooks.host_setup`)

A host-side setup script (precedent:
`robot/ros_ws/src/perception/natnet_ros2/scripts/download-natnet-sdk.sh`). Every hook
script MUST be:

- **idempotent** — safe to run on every sync; exits fast when already done,
- **sudo-free** — never escalates privileges,
- **contained** — writes only inside the module checkout.

## How validation works

`tools/validate_module.py` is python3 stdlib + PyYAML (no `jsonschema` dependency).
It loads `module.schema.json` and interprets it with a generic walker, so **schema
evolution normally requires no validator changes**. The walker understands this
draft-07 subset:

`type` (incl. `["string", "null"]`), `required`, `properties`, `enum`, `pattern`,
`items`, `additionalProperties`, `minLength`, `minItems`.

Two custom behaviors are keyed off `x-airstack-*` annotations in the schema, never
off field names in code:

- `x-airstack-format`: `semver-range` (`airstack_compat`) and `safe-relative-path`
  (no absolute paths, no `..` escapes) — the only custom "format" hooks.
- Cross-file facts, applied only when validating a module **directory**:
  - `x-airstack-check-exists`: the declared path (dockerfile / compose /
    hooks.host_setup / docs entries) must exist in the module → **error**.
  - `x-airstack-warn-missing-dir`: a `tests.packages` entry naming no directory in
    the module → **warning** (stderr only; never affects validity or the verdict).

A minimal valid module lives at
[`tests/fixtures/modules/hello_module`](../../tests/fixtures/modules/hello_module) —
both the contract-test fixture and a copyable example.
