# AirStack Agent Skills

This directory contains Agent Skills following the [Agent Skills standard](https://agentskills.io).

## Structure

```
.agents/
├── README.md           # This file
└── skills/             # Agent Skills directory
    ├── add-ros2-package/
    │   └── SKILL.md    # Create new ROS 2 packages
    ├── add-task-executor/
    │   └── SKILL.md    # Implement a task executor as a ROS 2 action server
    ├── integrate-module-into-layer/
    │   └── SKILL.md    # Integrate modules into layer bringup
    ├── write-isaac-sim-scene/
    │   └── SKILL.md    # Create Isaac Sim simulation scenes
    ├── debug-module/
    │   └── SKILL.md    # Autonomous debugging strategies
    ├── update-documentation/
    │   └── SKILL.md    # Document modules and update mkdocs
    ├── test-in-simulation/
    │   └── SKILL.md    # End-to-end simulation testing
    └── add-behavior-tree-node/
        └── SKILL.md    # Create behavior tree nodes
```

## Skills Overview

Each skill is a directory containing a `SKILL.md` file with:
- **YAML frontmatter:** Name, description, license, metadata
- **Markdown body:** Step-by-step instructions for the workflow

### Available Skills

| Skill | Purpose |
|-------|---------|
| [add-ros2-package](skills/add-ros2-package) | Create a new ROS 2 package for the autonomy stack |
| [add-task-executor](skills/add-task-executor) | Implement a task executor as a ROS 2 action server |
| [integrate-module-into-layer](skills/integrate-module-into-layer) | Integrate a module into layer bringup |
| [write-isaac-sim-scene](skills/write-isaac-sim-scene) | Create custom simulation environments |
| [debug-module](skills/debug-module) | Systematically debug ROS 2 modules |
| [update-documentation](skills/update-documentation) | Document modules and update mkdocs |
| [test-in-simulation](skills/test-in-simulation) | Test modules in Isaac Sim |
| [add-behavior-tree-node](skills/add-behavior-tree-node) | Create behavior tree nodes |

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
5. Update this README with the new skill

## References

- **Main Guide:** [AGENTS.md](../AGENTS.md)
- **Agent Skills Spec:** [https://agentskills.io](https://agentskills.io)
- **System Architecture:** [docs/robot/autonomy/system_architecture.md](../docs/robot/autonomy/system_architecture.md)
- **Integration Checklist:** [docs/robot/autonomy/integration_checklist.md](../docs/robot/autonomy/integration_checklist.md)
- **AI Agent Guide:** [docs/development/ai_agent_guide.md](../docs/development/ai_agent_guide.md)
