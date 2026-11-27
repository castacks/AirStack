# RRM-C001 M00-E1C formal owner-evidence request

DESIGN PROPOSAL ONLY — NO DEPLOYMENT OR TIER B EXECUTION AUTHORIZED

Request ID: RFI-M00E1C-001. Addressed for **PD routing to verified environment owners**. State: **PREPARED_FOR_PD_ROUTING_NOT_SENT_TO_EXTERNAL_OWNERS**. No named external contact, delivered request, owner response or authority receipt is inferred. This document is the formal request included in the review packet; no email/chat/GitHub message was sent. No owner records obtained beyond the local role descriptions and preserved evidence.

Please identify the authoritative person/team and delegated scope for OWN01–OWN09, then obtain the14 itemized responses in M00E1C_OWNER_EVIDENCE_ITEMS.csv. Lab-admin/operator roles appear in osmo/README.md and the OSMO tutorial; no named operational owner is established. .github/CODEOWNERS was absent in the bounded local search. Repository authors, committers, package maintainers and source labels must not be promoted to environment/hardware/security owners. PD approval authority is documented as a role, not an identified individual.

| Request | Routing role(s) | Specific evidence |
|---|---|---|
| RQ01 | OWN01;OWN06 | Workflow and execution-host identity: Approved workflow UID/name/version, pool, namespace, node/boot, isolation/tenancy/resource ownership and approval chain |
| RQ02 | OWN01;OWN03;OWN04;OWN08 | Exact source/image/build/mount identities: Registry distribution digests and local config IDs, layers/build provenance, source/overlay/binary/mount manifests, access/rights and release authority; resolve robot empty RepoDigests |
| RQ03 | OWN02 | Simulator runtime capabilities: Exact Isaac build/runtime/GPU-driver/toolkit dependencies; evidence native non-privileged direct runtime is supported; required device/capability/seccomp/mount list |
| RQ04 | OWN02 | Scene/vehicle/cache identity: Exact USD/dependency closure, Iris model/dynamics/sensors/airframe mapping, epoch/seed support, first-Play behavior and audited pre-existing warm artifact |
| RQ05 | OWN03;OWN08 | PX4 baseline and authoritative startup allocation: Source, executable/build target, rcS/airframe10015/full params, frozen originals/approval chains including startup origin/budget and allowed deltas |
| RQ06 | OWN04;OWN05 | MAVLink/MAVROS/DDS effective interfaces: Exact local/bind/peer/sysid/compid, plugin/dependency/QoS/RMW XML/security permissions, generated UDP/DDS port ranges and safe telemetry/service allowlist |
| RQ07 | OWN05;OWN06 | Network and command-boundary enforcement: Effective deny-by-default ingress/egress/route/firewall/discovery policy, no domain1↔0/99 or operational bridges, bounded operator/evidence path |
| RQ08 | OWN02;OWN03;OWN04 | Startup/shutdown/dependency semantics: Actual image entrypoint/autostart/launcher hashes and finite P0/P1 schedule, independent fresh predicate, guard-host compatibility and termination tree |
| RQ09 | OWN01;OWN06;OWN07 | Device/privilege/hardware exclusions: No FC/actuator/USB/serial/CAN/radio connection, minimal GPU allowlist, host-device namespace/hotplug/mount/capability/userns policy and physical ownership |
| RQ10 | OWN05;OWN06 | Security-policy capability: Supported DDS identity/permissions enforcement or equivalent isolated middleware domain, least-privilege runtime and fail-closed policy load behavior |
| RQ11 | OWN09 | Instrumentation and clock/recorder capability: Available I01–I06 hooks/versions/event schemas, native clocks/UTC mapping uncertainty/reset markers, acquisition/drop/overhead/cadence and independent survival/exit contract |
| RQ12 | OWN01;OWN06;OWN07;OWN09 | Operational ownership and emergency response: Named operator/independent stop owner, exact target-only escalation/termination/recovery, numeric time/resource/storage/overhead bounds and scope expiry |
| RQ13 | OWN08 | Frozen F00 and decision authority: FR-01..FR-20 originals, A3 and applicable E9 approval chain, exact qualifying image/parameter disposition and developmental claim limits |
| RQ14 | OWN09 | Retention/durability/immutable attempt handling: Independent executor/supervisor failure domains, bounded durable storage, archive/detached digest procedure and failed/aborted/incomplete/setup attempt retention |

Response contract: for every item supply owner/team identity and authority basis, exact environment/workflow/host/container scope, record revision/UTC acquisition time, intended versus deployed versus observed state, file/manifest byte count and SHA-256, approval/exception references, and explicit missing/unknown fields with accountable closure owner. Do not send credentials, tokens, private keys or device command streams. Prefer authenticated read-only exports and immutable existing records; do not perform an active probe/startup/fault to answer this request without its own authorization. A design statement is not implementation/behavior proof.

PD routing record: owner name/team ____________________; delegated scope/record ____________________; authorized delivery channel ____________________; actual delivery reference/time ____________________; response/evidence receipt ____________________. All blank. If no record exists, state that and retain the external dependency. No deadline or budget is invented. RQ05/RQ13 originals constrain reproduction/qualification and HD06; a development-only diagnostic scope still needs explicit configuration/claim-limit disposition. RQ03/RQ07/RQ09/RQ10 capability/safety gaps block approval of active verification, not offline design completion.
