# AirStack Agent Skills

This directory contains Agent Skills following the [Agent Skills standard](https://agentskills.io).

## Structure

```
.agents/
├── README.md           # This file
└── skills/             # One directory per skill
    └── <skill-name>/
        ├── SKILL.md    # YAML frontmatter + step-by-step instructions
        └── assets/     # (optional) templates and reference files
```

## Available Skills

| Skill | Purpose |
|-------|---------|
| [add-behavior-tree-node](skills/add-behavior-tree-node) | Create behavior tree nodes for high-level mission logic |
| [add-ros2-package](skills/add-ros2-package) | Create a new ROS 2 package (module) from the template |
| [add-task-executor](skills/add-task-executor) | Implement a task executor as a ROS 2 action server (`tasks/*`) |
| [add-unit-tests](skills/add-unit-tests) | Add co-located Python/C++ unit tests and register them for CI |
| [attach-gossip-payload](skills/attach-gossip-payload) | Broadcast custom ROS messages to peers via PeerProfile gossip payloads |
| [bump-version-and-release](skills/bump-version-and-release) | Bump `.env` VERSION + CHANGELOG to clear the version-check gate |
| [capture-discovered-knowledge](skills/capture-discovered-knowledge) | Persist hard-won discoveries to AGENTS.md or a skill |
| [configure-multi-robot](skills/configure-multi-robot) | Multi-robot setup: replicas, ROBOT_NAME, ROS_DOMAIN_ID |
| [create-module](skills/create-module) | Author a thin module repo (module.yaml manifest, CI, test_stack) |
| [create-stack](skills/create-stack) | Create a stack folder: `airstack stack new`, wiring bootstrap, split stacks + bridge.yaml, doctor |
| [debug-module](skills/debug-module) | Systematic autonomous debugging of ROS 2 modules |
| [docker-build-profiles](skills/docker-build-profiles) | Build-time validation for Docker compose profiles and build args |
| [integrate-module-into-layer](skills/integrate-module-into-layer) | Integrate a module into a **stack** (stack entry include, canonical defaults, wiring.md regen, lint) — the layer-bringup workflow is legacy |
| [run-system-tests](skills/run-system-tests) | Run/extend the pytest system-test harness (marks, MetricsRecorder, /pytest) |
| [test-in-simulation](skills/test-in-simulation) | End-to-end module testing in Isaac Sim |
| [update-documentation](skills/update-documentation) | Document modules and update mkdocs navigation |
| [use-airstack-cli](skills/use-airstack-cli) | The `airstack` CLI and the non-interactive `docker exec` pattern |
| [use-feature-notebook](skills/use-feature-notebook) | Local notebook/ entry (design spec + results) for every feature |
| [visualize-in-foxglove](skills/visualize-in-foxglove) | Add topic visualization to Foxglove/GCS |
| [write-isaac-sim-scene](skills/write-isaac-sim-scene) | Create custom Isaac Sim scenes on pegasus_app |
| [write-launch-file](skills/write-launch-file) | Launch-file conventions: canonical-default topic args, ROBOT_NAME namespacing, single-locus rule |
| [write-mkdocs-documentation](skills/write-mkdocs-documentation) | Writing effective MkDocs documentation |

## Usage

Skills are designed for AI coding agents (OpenHands, Claude Code, etc.) to:

1. **Discover** skills through the description field
2. **Activate** relevant skill when task matches
3. **Follow** step-by-step instructions in SKILL.md
4. **Reference** cross-linked skills for related workflows

## For Developers

When adding new skills:

1. Create directory with hyphenated name (e.g., `my-new-skill/`)
2. Add `SKILL.md` with proper YAML frontmatter
3. Follow Agent Skills format specification
4. Reference related skills using relative paths (`../other-skill/`)
5. Update this README **and** the skills table in [AGENTS.md](../AGENTS.md)

Skills ship with the mechanism, not after it (RFC #379 §10): when a workflow
changes (e.g. layer bringups → stacks), update the affected skills in the same
PR as the machinery, or every agent session will faithfully reintroduce the
old pattern.

## References

- **Main Guide:** [AGENTS.md](../AGENTS.md)
- **Agent Skills Spec:** [https://agentskills.io](https://agentskills.io)
- **System Architecture:** [docs/robot/autonomy/system_architecture.md](../docs/robot/autonomy/system_architecture.md)
- **Interface Conventions Spec:** [docs/robot/autonomy/interface_conventions.md](../docs/robot/autonomy/interface_conventions.md)
- **Integration Checklist:** [docs/robot/autonomy/integration_checklist.md](../docs/robot/autonomy/integration_checklist.md)
- **Stacks Guide:** [docs/development/stacks.md](../docs/development/stacks.md)
- **Modules Guide:** [docs/development/modules.md](../docs/development/modules.md)
- **AI Agent Guide:** [docs/development/ai_agent_guide.md](../docs/development/ai_agent_guide.md)
