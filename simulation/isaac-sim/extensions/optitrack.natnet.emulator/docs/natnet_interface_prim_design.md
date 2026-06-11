# Design: NatNet Interface Prim (stage-driven emulator)

> Status: **proposed** (design only — no implementation yet).
> Supersedes the hardcoded `defaults.py` `TrackedBodyBinding` approach for the
> Isaac wrapper. See [`../README.md`](../README.md) for the shipped server.
> Delivery is staged across four commits — see
> [Implementation roadmap](#implementation-roadmap-staged-commits).

## Goal

Drive the NatNet emulator entirely from the **USD stage** instead of launch-script
constants. A config prim in the scene declares *that* a NatNet interface should
exist, *what* rigid bodies it streams, and *how* the server is configured. The
extension scans the stage for these prims and manages a server per interface.

Because the configuration lives as authored USD, **the catalog persists between
runs for free** — it serializes with the `.usd`/`.usda` file. This is the answer
to "how do we maintain the catalog between runs."

## Why a prim

- **Persistence:** authored attributes save with the scene; reopening the stage
  restores the full catalog and server config.
- **Discoverability / authoring:** appears in the Stage tree and Property panel;
  editable by hand or by tooling without touching Python.
- **Live control:** toggling an attribute can start/stop the server at runtime.
- **Multi-interface:** N config prims are supported naturally (multi-robot, or a
  unicast + multicast pair).

## Schema (USD applied API schema)

Use **applied API schemas** so the attributes are typed and surface cleanly in the
Property panel. Implement them as **codeless schemas**: author `schema.usda` with
`skipCodeGeneration = true`, run `usdGenSchema` **once** to emit
`generatedSchema.usda` + `plugInfo.json` (no C++/Python classes are generated —
only the registry files), and load that plugin from the extension. USD then reads
the typed fallbacks at runtime. See [Risks & open questions](#risks--open-questions)
— schema registration inside Kit is the main unknown and has a fallback.

Two schemas:

### `NatNetInterfaceAPI` (single-apply)

Applied to a holder prim (e.g. a `Scope` or `Xform`, conventionally
`/World/NatNetInterface`). Carries server-level config:

| Attribute | Type | Maps to `NatNetUnicastServer` |
|-----------|------|-------------------------------|
| `natnet:serverEnabled` | `bool` | start / stop (see Lifecycle) |
| `natnet:serverIp` | `string` | `local_interface` |
| `natnet:mode` | `token` (`unicast`\|`multicast`) | `transmission_type` |
| `natnet:multicastAddr` | `string` | `multicast_address` |
| `natnet:commandPort` | `int` | `command_port` |
| `natnet:dataPort` | `int` | `data_port` |
| `natnet:publishRate` | `float` | `publish_rate` |
| `natnet:natnetVersion` | `string` (`"4.4.0.0"`) | `natnet_version` |

### `NatNetBodyBindingAPI` (multiple-apply)

The **whole catalog lives on the interface prim** — a dictionary of body entries —
so the extension reads one prim to get the full mapping and never traverses the
stage to *find* bindings. A **multiple-apply** API schema expresses that dictionary
natively: one instance per body, keyed by an opaque instance token (`<key>`).

The Motive rigid body name is an **explicit attribute** (`rigidBodyName`), not the
instance token, so it can be typed/edited as a normal text field in the UI without
renaming a schema instance. Each instance carries the per-body metadata plus a
**single-target relationship** to the tracked prim. Relationship targets are
**path-aware** — USD rewrites them automatically when the tracked prim is renamed or
reparented — which a literal path-string dict (e.g. `customData` `VtDictionary`)
would not survive.

| Member (per instance `<key>`) | Type | UI label | Maps to |
|-------------------------------|------|----------|---------|
| `natnet:body:<key>:rigidBodyName` | `string` | "Motive Rigid Body Name" | `sRigidBodyDescription.szName` |
| `natnet:body:<key>:streamingId` | `int` | "Streaming ID" | `sRigidBodyDescription.ID` |
| `natnet:body:<key>:parentId` | `int` | "Parent ID" | `sRigidBodyDescription.parentID` (default `-1`) |
| `natnet:body:<key>:target` | `rel` → prim | "Tracked Prim" | pose source |

> Alternative considered: a single relationship `natnet:bodies` with N targets plus
> parallel `int[]` arrays (`streamingIds`, `parentIds`) indexed alongside the
> targets. Functionally equivalent and slightly more compact, but the multi-apply
> instances are typed, individually editable in the Property panel, and self-naming
> (instance name = body name). Rejected the per-tracked-prim single-apply approach
> from the prior draft because it forced a stage traversal to discover bindings.

### Example (`.usda` sketch)

```usda
def Scope "NatNetInterface" (
    prepend apiSchemas = ["NatNetInterfaceAPI",
                          "NatNetBodyBindingAPI:body0"]
)
{
    bool   natnet:serverEnabled = true
    string natnet:serverIp      = "172.31.0.200"
    token  natnet:mode          = "unicast"
    int    natnet:commandPort   = 1510
    int    natnet:dataPort      = 1511
    float  natnet:publishRate   = 100
    string natnet:natnetVersion = "4.4.0.0"

    # one body entry (key "body0"); add more by applying NatNetBodyBindingAPI:<key>
    string natnet:body:body0:rigidBodyName = "Drone"
    int    natnet:body:body0:streamingId   = 1
    int    natnet:body:body0:parentId      = -1
    rel    natnet:body:body0:target        = </World/base_link>
}
```

## Catalog assembly

For each enabled interface, read its `NatNetBodyBindingAPI` instances directly off
the interface prim (no stage traversal), and build one `sRigidBodyDescription` per
instance (`szName` = `rigidBodyName`, `ID` = streamingId, `parentID` = parentId)
into an `sDataDescriptions`, then `server.set_model_def_payload(catalog.pack())`.
This is the data-driven generalization of the current `make_default_drone_catalog()`
in [`server/natnet_model_types.py`](../optitrack/natnet/emulator/server/natnet_model_types.py).

Resolve each instance's `target` relationship to a `Usd.Prim` handle **once** at
this point and **cache** the `(prim, bodyName, streamingId)` tuples on the manager
entry. The per-frame loop then reuses those handles — it never re-reads the schema
or searches the stage. Rebuild the cache only on enable or when the interface's
binding members change.

Each interface owns its own dictionary, so multi-interface scoping is inherent —
no cross-interface grouping key is needed.

## Python API (authoring + scripting)

The prim is the persisted source of truth, but authoring/reading it should never
require hand-writing `CreateAttribute` / `ApplyAPI` / relationship calls. A thin
facade in the package wraps the schema so scripts can create, edit, and read an
interface — and so the same dataclasses are reused by the runtime manager.

### Dataclasses (in-memory form)

```python
@dataclass
class BodyBinding:
    rigid_body_name: str        # Motive rigid body name (szName)
    target_prim: str            # USD path of the tracked prim
    streaming_id: int = 1
    parent_id: int = -1

@dataclass
class NatNetInterfaceConfig:
    server_enabled: bool = True
    server_ip: str = "172.31.0.200"
    mode: str = "unicast"        # "unicast" | "multicast"
    multicast_addr: str = "239.255.42.99"
    command_port: int = 1510
    data_port: int = 1511
    publish_rate: float = 100.0
    natnet_version: str = "4.4.0.0"
    bodies: list[BodyBinding] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict) -> "NatNetInterfaceConfig": ...
    def to_dict(self) -> dict: ...
```

`from_dict` accepts the dictionary the user passes in a script. `bodies` may be a
list of dicts **or** a `{prim_path: {...}}` mapping (the "dictionary of prims →
rigid body names and stuff"), normalized into `BodyBinding` objects.

### Authoring / reading helpers

```python
def author_interface(stage, prim_path, config) -> Usd.Prim:
    """Create/overwrite the interface prim: apply NatNetInterfaceAPI, set server
    attrs, and apply one NatNetBodyBindingAPI:<name> instance per body with its
    target relationship. `config` may be a NatNetInterfaceConfig or a plain dict."""

def read_interface(prim) -> NatNetInterfaceConfig:
    """Reverse of author_interface — reconstruct the dataclass from authored USD."""

def find_interfaces(stage) -> list[Usd.Prim]:
    """All prims carrying NatNetInterfaceAPI (used by the manager and tooling)."""
```

### Script usage (standalone launch)

Author from a dict and bring the server up in one place — no GUI required:

```python
from optitrack.natnet.emulator.isaac import author_interface, NatNetServerManager

CONFIG = {
    "server_enabled": True,
    "server_ip": "172.31.0.200",
    "mode": "unicast",
    "publish_rate": 100,
    "bodies": {
        "/World/base_link": {"rigid_body_name": "Drone", "streaming_id": 1},
    },
}

author_interface(stage, "/World/NatNetInterface", CONFIG)   # persists with the stage

manager = NatNetServerManager(stage)   # discovers interfaces, honors serverEnabled
manager.start()                        # spins up servers + per-frame pose sampling
# manager.shutdown() on teardown
```

Because `author_interface` writes the same attributes the GUI/manager read, a
script-authored interface is indistinguishable from a hand-authored one and saves
into the `.usd`. A convenience `author_and_start(...)` can wrap the two calls for
the common single-interface script case.

> The dataclasses (`BodyBinding`, `NatNetInterfaceConfig`) are the single in-memory
> representation shared by: `from_dict` (script input) → `author_interface` (USD
> write) → `read_interface` (USD read) → catalog assembly + pose-sampling cache.
> This is also the natural successor to `defaults.py` — its constants become a
> `NatNetInterfaceConfig` default.

## Lifecycle (manager)

A manager owns a map `{Sdf.Path -> NatNetUnicastServer}` keyed by interface prim
path, and reacts to a `Usd.Notice.ObjectsChanged` listener plus stage-open events.

- `natnet:serverEnabled` → **true**: construct a **fresh** `NatNetUnicastServer` from
  the prim's attributes, build + set the MODELDEF payload, `start()`, store in the map.
- `natnet:serverEnabled` → **false**: `server.shutdown()` (closes both sockets, joins
  all child threads), remove from the map.
- Server-config attr changes while enabled (IP, ports, mode): treat as shutdown +
  fresh start so the new socket binding takes effect.
- Binding-member changes while enabled (add/remove a body, edit a `target`,
  streamingId, or parentId): rebuild the cached `(prim, bodyName, streamingId)`
  list and refresh the MODELDEF via `set_model_def_payload()` — **no server restart
  needed**, since the transport/sockets are unchanged.

**Fresh instance per enable (chosen):** the manager always builds a new server
object on enable and discards the old one. The existing server is not designed to
be re-`start()`ed in place — `shutdown()` closes sockets without recreating them and
`start()` appends to `self.threads` without resetting — so reusing an instance across
a disable→enable cycle would re-run dead threads with `shutdown_event` still set.
A fresh instance avoids all of that with no server-side changes. Note this is
**thread** teardown within the Kit process — not an OS process kill: `shutdown()`
ends the server's daemon threads and closes its sockets:

```text
shutdown(): running=False; shutdown_event.set(); close command_socket; close data_socket; join threads
```

## Pose sampling

A per-frame Kit update callback iterates the **cached** `(prim, bodyName,
streamingId)` tuples (resolved once during catalog assembly), reads each prim's
world transform (`UsdGeom.Xformable.ComputeLocalToWorldTransform` via a per-frame
`UsdGeomXformCache`, or the physics view pose), converts it into an
`sFrameOfMocapData` rigid body (params `& 0x01` = tracking valid), and calls
`server.enqueue_mocap_data(frame)`. No schema reads or stage searches happen in the
hot path — only direct pose pulls off the cached handles, exactly as intended.

**Frame convention (`up_axis` option):** Isaac/USD is Z-up; Motive exposes an
"Up Axis" setting. The reference `natnet_ros2` driver (and ours) require it set to
**Z** and do **no** axis conversion — `rb_to_pose` is an identity copy and
`vision_pose_converter_node` only normalizes quaternion sign. So the interface
config carries an `up_axis` (default **`"Z"`**) and the sampler streams the USD
world pose **as-is** by default, matching the rest of the stack. Setting
`up_axis="Y"` emulates a default Y-up Motive by rotating each streamed pose -90° about
X (`(x,y,z) -> (x,z,-y)`, quaternion vector part the same, scalar kept) — a proper
right-handed change of basis. The conversion is the pure, unit-tested
`frames.to_motive_pose`; the manager re-reads `up_axis` on every resync and applies
it in `sample_once`. Authored on the prim as `natnet:upAxis`, editable in the UI
("Up axis" dropdown) and via `build_drone_config(..., up_axis=...)`.

## Packaging notes

- **Codeless applied schemas** keep this dependency-light: `schema.usda` defines
  `NatNetInterfaceAPI` / `NatNetBodyBindingAPI` with `skipCodeGeneration = true`;
  `usdGenSchema` emits `generatedSchema.usda` + `plugInfo.json` (registry only, no
  compiled classes); the extension loads the plugin on startup. Registration
  reliability inside Kit is the main risk — see below.
- **Standalone + Kit parity:** put "scan stage + manage servers + sample poses" in
  a plain module callable from both the Kit `omni.ext.IExt` (GUI/live toggle) and
  the standalone launch scripts (`launch_scripts/example_*_pegasus_launch_script.py`),
  which have no extension lifecycle.
- **Persistence layer:** author interface/binding attributes on the stage's
  root/edit layer (not the session layer) so they save with the scene.

## Implementation roadmap (staged commits)

Each stage is independently reviewable and ends with a concrete manual check.
Stages build on each other **in order** — each depends on the previous:

1. **Commit 1 — Schema + config prim spawn + authoring UI** (no deps)
2. **Commit 2 — Detection + parameter read** (needs the schema + facade from 1)
3. **Commit 3 — Catalog parse + server start** (needs detection/read from 2)
4. **Commit 4 — Enable/disable lifecycle** (needs the start path from 3)

Unit tests are added **within each commit** (co-located in the extension `test/`
dir with a proxy under `tests/sim/`), so every stage lands already covered — see
[Testing & CI integration](#testing--ci-integration) for the mechanism.

> **Design for testability:** keep a pure-Python **config model** (dataclasses,
> `from_dict`/`to_dict`, the `natnet:body:<key>:…` attribute-name builder,
> validation) separate from a thin **USD binding** layer (apply/read against a
> `Usd.Stage`). Most logic is then hermetically testable with no `pxr`/Isaac, and
> the USD shell stays small and `pxr`-guarded.

### Commit 1 — Schema + config prim spawn + authoring UI

Stand up the data model and let a user create/edit an interface by hand. No
scanning, no server.

- **Schema:** `schema.usda` + `plugInfo.json` defining the codeless
  `NatNetInterfaceAPI` (single-apply) and `NatNetBodyBindingAPI` (multiple-apply);
  registered/loaded via `config/extension.toml`.
- **Authoring facade:** `BodyBinding` / `NatNetInterfaceConfig` dataclasses,
  `author_interface()`, and add/remove-body helpers (no `read_*`/manager yet).
- **UI (Kit extension):** a menu entry (e.g. *Create ▸ NatNet Interface*) that
  spawns `/World/NatNetInterface`; a panel to edit the server attributes and to
  **add a tracked body from the current stage selection**, typing *Motive Rigid
  Body Name*, *Streaming ID*, *Parent ID* (the `target` relationship is set from
  the selected prim).
- 🧪 **Tests** (`test/test_config_model.py`, hermetic): `from_dict`/`to_dict`
  round-trips; `bodies` normalization from a list **and** a `{prim_path: {...}}`
  mapping; the `natnet:body:<key>:…` attribute-name builder; validation
  (mode ∈ {unicast, multicast}, distinct command/data ports). Plus
  `test/test_authoring_usd.py` (`pytest.importorskip("pxr")`):
  `author_interface()` → `read_interface()` round-trip on
  `Usd.Stage.CreateInMemory()`, asserting attrs, the multi-apply instances, and the
  `target` relationship.
- ✅ **Manual verify:** create the prim from the menu, select a prim, fill the
  fields, **save and reopen** the stage — the attributes and the `target`
  relationship persist and match what was entered.

### Commit 2 — Detection + parameter read

Make the extension *aware* of interfaces without acting on them.

- `NatNetServerManager` skeleton: `find_interfaces()`, `read_interface() ->
  NatNetInterfaceConfig`, a `Usd.Notice.ObjectsChanged` listener, and stage-open
  hookup. Logs each detected interface and its parsed config (server params + body
  list). No server, no pose sampling.
- 🧪 **Tests** (`test/test_discovery.py`): pure `reconcile(old, new) -> actions`
  diff logic (hermetic) deciding start/stop/rebuild from config deltas;
  `find_interfaces()` on an in-memory stage with/without the API applied
  (`importorskip("pxr")`).
- ✅ **Manual verify:** open a stage containing an interface → logs show the prim
  path and fully parsed config; adding/removing a body or editing a field updates
  the logged readout live.

### Commit 5 — Data enqueue + dynamic catalog + scripting + UI readouts  ✅ IMPLEMENTED

> Rolled the deferred data path together with live updates, scripting, and UI:
> - **`isaac/frames.py`** (pure): `BodySample`, `make_rigid_body_data`, `build_frame`.
>   Sets `params` bit `0x01` (tracking valid — the client *skips* bodies without it)
>   and frame bit `0x02` (model list changed). **Frame convention resolved: emit the
>   USD world pose as-is, no Z-up→Y-up swap** — `natnet_ros2`'s `rb_to_pose` is an
>   identity copy and nothing downstream re-axes, so swapping would desync the pose.
> - **`usd_bindings.read_world_pose(prim)`** — `ComputeLocalToWorldTransform` →
>   `((x,y,z),(qx,qy,qz,qw))` (the pose *stored in the USD stage*).
> - **Manager**: a **dirty flag** (`_needs_resync`) — set on any NatNet prim change
>   (`_on_objects_changed`) or `mark_dirty()`, cleared by the next sample. The
>   physics-step subscription (`subscribe_physics_step_events`) calls `sample_once`,
>   which resyncs the catalog/target cache when dirty (so **bodies added/removed live
>   are picked up**, refreshing the server MODELDEF and flagging `model_list_changed`),
>   then samples every body's world pose and `enqueue_mocap_data`s one frame. Missing
>   prims stream as lost (NaN, valid bit clear).
> - **Scripting**: `NatNetServerManager.start_from_stage()` (author the prim, then
>   call it). `start_server` / `apply_enabled` remain the programmatic enable path.
> - **UI**: each tracked-body row has a live readout — status dot (green=streaming,
>   grey=idle, red=missing) + world position pulled from USD, refreshed ~6 Hz.
> - **Tests**: `test_frames.py` (pure: valid bit, NaN-lost, order, model-change bit,
>   pack decode), `test_pose_sampling.py` (pxr: read_world_pose, sample no/one/missing,
>   moving prim, **body-added-live resync + model_list_changed**), `test_pose_streaming.py`
>   (pxr loopback: streamed `NAT_FRAMEOFDATA` carries the sampled USD position). Proxies
>   added under `tests/sim/optitrack_natnet_emulator/`.

### Commit 6 — Example-script auto-start (load-up integration)  ✅ IMPLEMENTED

> The Pegasus example launch scripts stand the emulator up on scene load — no UI.
> - **`isaac/scene_setup.py`** — `build_drone_config(drones, ...)` (pure: one
>   `BodyBinding` per `(rigid_body_name, streaming_id, target_prim)`, validated) and
>   `start_drone_natnet_server(stage, drones, ...)` (authors the `/World/NatNetInterface`
>   prim, then creates a physics-subscribed `NatNetServerManager` and starts it).
>   Exported from `optitrack.natnet.emulator.isaac`.
> - **Launch scripts** (`launch_scripts/example_one*.py`, `example_multi*.py`): after
>   spawning drones, if `LAUNCH_NATNET=true` they author one rigid body per drone
>   `base_link` and start the server. Single: `("Drone", 1, /World/base_link)`.
>   Multi: `("<NATNET_BODY_NAME><i>", i, /World/drone<i>/base_link)` for each drone.
>   Failures are caught (never kill the sim); the manager is torn down on close.
> - **Import path**: scripts add `../extensions/optitrack.natnet.emulator` to
>   `sys.path` so the package imports without enabling the Kit UI extension (keeps a
>   single, script-owned manager — no duplicate physics/USD subscriptions).
> - **Wiring**: `LAUNCH_NATNET` / `NATNET_BODY_NAME` are passed into the isaac-sim
>   container via `simulation/isaac-sim/docker/docker-compose.yaml`, mirroring the
>   robot-side `natnet_ros2` gate. Server binds the container IP `172.31.0.200`
>   (the config default), which the robot `server_ip` already points at.
> - **Tests**: `test_scene_setup.py` (pure: single/multi mapping, server-param
>   forwarding, duplicate name/id rejection, empty catalog). Proxy under
>   `tests/sim/optitrack_natnet_emulator/`.

### Commit 3 — Catalog parse + server start  ✅ IMPLEMENTED

> **Status (combined with Commit 4):** catalog parse + server start/stop lifecycle
> landed together. What shipped:
> - `isaac/catalog.py`: `build_catalog(config) → sDataDescriptions` (pure ctypes,
>   no USD) + `find_duplicate_targets(config)`.
> - `usd_bindings.resolve_targets(stage, config) → (existing, missing)` diagnostics.
> - `NatNetServerManager` owns **one** server via an injectable
>   `server_factory` (default `default_server_factory`, unicast only). Methods:
>   `start_server` (idempotent — builds catalog, `set_model_def_payload`, `start`),
>   `stop_server`, `toggle_server`, `apply_enabled` (reconcile to `serverEnabled`),
>   `is_running`, `log_target_diagnostics`. `on_shutdown` stops the server.
> - UI: **Create Interface** (prim), plus a live **Start/Stop Server** toggle button
>   + RUNNING/stopped status. The button starts/stops directly from the on-stage
>   prim, **regardless** of `serverEnabled` (per request). Auto-reconcile of
>   `serverEnabled` on scan is intentionally *not* wired (would fight the button);
>   `apply_enabled` is available + tested for future auto-driving.
> - Tests: `test_catalog.py` (no/single/multiple bodies, field fidelity, name
>   truncation, MAX_MODELS guard, duplicate targets), `test_server_lifecycle.py`
>   (mock factory: create+start exactly once, idempotent, toggle, reconcile,
>   on_shutdown, multicast rejected), `test_target_resolution.py` (pxr-guarded:
>   missing/empty/duplicate/mixed), `test_server_from_config.py` (loopback: served
>   MODELDEF == `build_catalog(config)`, clean restart rebinds ports). Proxies added
>   under `tests/sim/optitrack_natnet_emulator/`.

Turn a detected config into a running server.

- Build `sDataDescriptions` from the config, resolve each `target` to a `Usd.Prim`
  and cache `(prim, rigidBodyName, streamingId)`, construct a `NatNetUnicastServer`
  from the server attrs, `set_model_def_payload()`, `start()`, and run the per-frame
  pose-sampling callback → `enqueue_mocap_data()`.
- Initial `serverEnabled` is honored at load (start only if true); **live toggling
  is deferred to Commit 4.**
- 🧪 **Tests** (hermetic): `test/test_catalog_from_config.py` — a
  `NatNetInterfaceConfig` with N bodies packs into an `sDataDescriptions` with the
  right `szName`/`ID`/`parentID` per body (extends the existing catalog tests);
  `test/test_pose_to_frame.py` — pure `pose → sFrameOfMocapData` conversion incl.
  the **Z-up → Y-up** frame convention (feed a known transform, assert position +
  quaternion). Plus a loopback case extending `test_unicast_protocol.py`: a server
  built from a config serves that MODELDEF and streams enqueued frames.
- ✅ **Manual verify:** with `serverEnabled = true` at load, the robot
  `natnet_ros2` connects and `/{ROBOT_NAME}/perception/optitrack/<name>` publishes
  at ~`publishRate`; moving the tracked prim moves the published pose. (A Python
  loopback client can stand in for `natnet_ros2` if the SDK isn't built.)

### Commit 4 — Enable/disable lifecycle  ✅ FOLDED INTO COMMIT 3

> The fresh-instance start/stop state machine (`start_server`/`stop_server`/
> `toggle_server`/`apply_enabled`) and the restart-cleanliness loopback
> (`test_server_from_config.py::test_stop_frees_port_for_restart`) shipped with
> Commit 3 above. Remaining for a follow-up: rebuild-on-config-delta while running,
> and (optionally) auto-reconciling `serverEnabled` edits from the Property panel.

React to `serverEnabled` flipping at runtime.

- On `serverEnabled` **false** → `server.shutdown()` (closes sockets, joins all
  child threads), drop from the map. On **true** → build a **fresh**
  `NatNetUnicastServer` and `start()` again. (Server-config and binding live-edits
  per the [Lifecycle](#lifecycle-manager) section land here or as an immediate
  follow-up.)
- 🧪 **Tests** (`test/test_lifecycle.py`, hermetic): the manager state machine
  (`serverEnabled` true→false→true yields stop then fresh-start actions); and a
  restart-cleanliness loopback test — `start()` then `shutdown()` joins all threads
  and closes both sockets, and a **fresh** instance rebinds the same ports and
  serves again (proves no thread/socket leak across cycles).
- ✅ **Manual verify:** toggle `serverEnabled` in the UI repeatedly — the server
  threads die and respawn cleanly, the pose topic stops and resumes, and there is
  no thread/socket leak across cycles (guaranteed by the fresh-instance approach).

## Testing & CI integration

Follows AirStack's **co-location + proxy** unit-test pattern (see the
[`add-unit-tests`](../../../../../.agents/skills/add-unit-tests/SKILL.md) skill).
The extension already uses it today — test source in
[`test/`](../test/), thin proxies under
[`tests/sim/optitrack_natnet_emulator/`](../../../../../tests/sim/optitrack_natnet_emulator/).

**Mechanism (per new test file):**

1. Write the test co-located in
   `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_<name>.py`,
   decorated `@pytest.mark.unit`.
2. Add a one-line proxy
   `tests/sim/optitrack_natnet_emulator/test_<name>.py` that calls
   `reexport_unit_tests(globals(), repo_path(".../optitrack.natnet.emulator/test"), "test_<name>.py")`.
3. It is then discovered automatically by `pytest tests/ -m unit`,
   `airstack test -m unit`, and CI `system-tests.yml` (PR-open runs `pytest tests/`
   with no `-m` filter) — **no CI YAML change needed.**

**Two specifics for this feature:**

- **`usd-core` dependency.** The `pxr`-guarded tests (`author_interface` /
  `read_interface` / `find_interfaces` against an in-memory stage) only execute if
  `pxr` is importable; otherwise `pytest.importorskip("pxr")` skips them. To make CI
  actually exercise them, add **`usd-core`** to
  [`tests/requirements.txt`](../../../../../tests/requirements.txt) (installed into
  the venv by `system-tests.yml`). Pure config-model / catalog / pose / lifecycle
  tests need no new deps (`numpy` is already present).
- **Not a colcon package.** This extension is a sim-side Python extension, **not** an
  ament/colcon package, so it does **not** go in
  [`tests/colcon_unit_test_packages.yaml`](../../../../../tests/colcon_unit_test_packages.yaml).
  Its CI coverage is the pytest proxy route only (the `build_packages` colcon route
  does not apply).

**Per-commit test artifacts:**

| Commit | Co-located test file(s) | `pxr`? | Proxy to add |
|--------|-------------------------|--------|--------------|
| 1 | `test_config_model.py`, `test_authoring_usd.py` | model: no · authoring: yes | one proxy each |
| 2 | `test_discovery.py` | `find_interfaces`: yes · reconcile: no | one proxy |
| 3 | `test_catalog_from_config.py`, `test_pose_to_frame.py` (+ extend `test_unicast_protocol.py`) | no | one proxy each |
| 4 | `test_lifecycle.py` | no | one proxy |

Update [`tests/sim/README.md`](../../../../../tests/sim/README.md) as proxies are
added. Run locally with `airstack test -m unit -v` or
`pytest simulation/isaac-sim/extensions/optitrack.natnet.emulator/test -m unit -v`.

## Risks & open questions

Ordered roughly by impact. The first two change *how* Commit 1 / Commit 3 are
sequenced; the rest are "note and handle during implementation."

1. **Schema registration inside Kit (highest unknown).** Codeless schemas still
   require `usdGenSchema`-emitted `generatedSchema.usda` + `plugInfo.json` to be
   discovered by USD's plugin system (search paths, `HasAPI`/`ApplyAPI`, typed
   fallbacks) within the Kit runtime. **Mitigation:** make a **schema-registration
   spike the first task of Commit 1**, before any UI. **Fallback:** if codeless
   registration is painful in this Kit/USD version, drop to plain namespaced custom
   attributes (`prim.CreateAttribute("natnet:…")`) — the Python facade hides which
   backing is used, so runtime/UI code is unaffected.

2. **Commit 1 front-loads risk with no streaming payoff.** Schema + facade + custom
   multi-apply UI is heavy and moves no poses. **Option:** prove
   schema + config model + facade + headless manager/server via the **script path +
   unit tests first**, then build the Kit GUI on that tested foundation. Keeps the
   "manually verify the UI" deliverable but rests it on green tests. *(Sequencing
   choice — not yet decided.)*

3. **Multi-apply + relationship UI is the heavy part.** The relationship picker and
   "add body from selection" are the real cost in Commit 1. Consider shipping v1
   with a **single body** (or the relationship + parallel-arrays variant) to close
   the end-to-end loop first, then generalize the catalog UI.

4. **Frame convention is a correctness landmine (not a footnote).** Isaac Z-up /
   meters vs Motive Y-up will silently produce wrong poses, and `natnet_ros2`
   *already* converts — so the risk is **double-converting**. **Make pinning the
   exact wire convention a first-class task in Commit 3**, validated against the
   existing verified `libNatNet` contract and `tests/integration/natnet/`.

5. **Pose-sampling timing.** The world-transform read must happen **after PhysX
   write-back** (the correct post-physics / world callback), or streamed poses lag
   or read stale. Confirm which callback the per-frame loop hooks into.

6. **Rapid enable toggling can race the socket rebind.** Fresh-instance +
   `shutdown()` join (1 s) then re-`start()` on the same port can collide if the old
   socket has not fully closed. `SO_REUSEADDR` is set and the restart-cleanliness
   test (Commit 4) guards this; consider also **debouncing** rapid toggles.

7. **Standalone vs Kit parity for the live toggle.** `serverEnabled` live-toggling
   relies on the `ObjectsChanged` loop, which only runs under Kit. Standalone
   scripts use the **initial** state unless they pump USD notices — document this.

8. **`usd-core` ≠ Isaac's bundled USD.** The CI `pxr` from `usd-core` may differ in
   version from Isaac's USD. Fine for hermetic schema/Sdf unit tests; just be aware
   behavior can differ slightly from the runtime.

9. **Release gating.** Docker image content changes here (extension install), so bump
   `.env` `VERSION` + CHANGELOG before merge (see `bump-version-and-release`).

## Relationship to existing code

- Replaces `defaults.py` `TrackedBodyBinding` (each `NatNetBodyBindingAPI` instance
  is the authored form of that dataclass; `prim_path` becomes the instance's
  `target` relationship).
- Server (`NatNetUnicastServer`) is unchanged — it stays transport-only and
  consumes `set_model_def_payload()` + `enqueue_mocap_data()` exactly as today.

## Debugging discoveries (in-sim end-to-end bring-up)

Hard-won findings from getting the liveliness sentinel green (Isaac emulator →
`natnet_ros2` on the robot). Captured so the next agent does not re-derive them.

1. **GIL starvation kills the background data thread inside Kit.** The server's
   `_data_update_loop` daemon (which `time.sleep`s then sends frames) is reliably
   starved by Kit's render/physics main loop holding the GIL — frames get enqueued
   but never transmitted, so the client handshakes but receives nothing. Fix:
   `NatNetServer.auto_stream` (default `True`) gates that loop; the Isaac wrapper
   sets `auto_stream = False` in `manager.start_server()` and calls
   `server.pump_once()` **synchronously from the physics-step callback**
   (`manager.sample_once()`). Outside Kit (host integration tests, sidecar) the
   default `auto_stream=True` path still works. Don't "optimize" the pump back into
   the daemon thread.

2. **The liveliness/system path runs the *standalone Pegasus script*, not a saved
   scene USD.** `tests/conftest.py` sets `ISAAC_SIM_USE_STANDALONE=true` **and
   `ISAAC_SIM_SCRIPT_NAME=example_multi_px4_pegasus_launch_script.py`** — even for
   `--num-robots 1`. So the NatNet auto-start wiring **must** live in
   `example_multi_…` (it does); `example_one_…` and the default
   `airstack up isaac-sim` (which uses `ISAAC_SIM_USE_STANDALONE=false` →
   `run_isaacsim.launch.py` opening `simple_pegasus.scene.usd`) do **not** exercise
   it. To reproduce the harness manually you must pass all of
   `AUTOLAUNCH=true ISAAC_SIM_USE_STANDALONE=true
   ISAAC_SIM_SCRIPT_NAME=example_multi_px4_pegasus_launch_script.py
   LAUNCH_NATNET=true PLAY_SIM_ON_START=true`.

3. **Single-agent body/prim contract.** For `NUM_ROBOTS=1` the multi script names
   the body `"Drone"` (bare) → target `/World/drone1/base_link` (the multi script's
   drone prim path is always `drone{i}`, even for one drone). This matches the robot
   `natnet_config.yaml` body and the liveliness sentinel topic
   `/robot_1/perception/optitrack/Drone/pose_cov`.

4. **Target prims appear *after* server start.** The Pegasus `base_link` is created
   on the first Play tick — after `start_server()`. `_sample_cache` therefore stores
   target *paths* and re-resolves `stage.GetPrimAtPath()` every `sample_once`, so a
   body goes from "lost" to valid as soon as its prim exists (no `mark_dirty`).

5. **`ss` is unreliable in the Isaac image; use `/proc/net/udp`.** Confirm the
   server is bound by looking for ports `1510`/`1511` as hex (`05E6`/`05E7`) in
   `/proc/net/udp` rather than trusting `ss -ulnp` (returns empty here).

6. **First-message budget.** Cold Isaac boots compile shaders; the NatNet sentinel
   first-message timeout was raised to 120s (`_NATNET_FIRST_MSG_TIMEOUT`) to match
   the generous `/clock` (600s) / sentinel-node (300s) budgets. Once streaming, the
   topic runs at ~98–140 Hz (100 Hz `publishRate`).

7. **The client must retry the connect — the server starts ~100s after the robot.**
   This was the actual liveliness blocker. `natnet_ros2_node` originally called
   `connect_and_setup()` **once** in its constructor; on `NetworkError` it logged and
   gave up. In the standalone path the Isaac emulator only binds 1510/1511 ~100s into
   sim boot, long after the robot's node starts, so the one-shot connect failed
   permanently (topic advertised, zero frames). Fix: `connect_and_setup()` returns
   `bool` and a 2s `connect_timer_` retries the handshake until the first success,
   then cancels itself (also correct for real Motive powered on after the drone).
   Verified by starting Isaac and the node together: the node retries for ~90s, then
   logs "Frame callback registered" the instant the server binds. **To reproduce the
   race manually, start the robot node first, then Isaac** — bringing the robot up
   *after* Isaac is ready hides the bug.
