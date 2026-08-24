# What's Next? Tutorials by Topic

Now that you have AirStack running, explore these hands-on tutorials to learn specific workflows. Each section below contains focused tutorials for that area of AirStack.

### 🎮 Simulation Tutorials

Learn to work with Isaac Sim and create custom simulation scenarios.

| Tutorial | Description | Level |
|----------|-------------|-------|
| [Spawning Drones](../simulation/isaac_sim/spawning_drones.md) | Launch scripts for one or many drones in Isaac Sim | Beginner |
| [Custom Scene Setup](../simulation/isaac_sim/scene_setup.md) | Create custom Isaac Sim environments | Intermediate |
| [Pegasus Scene Setup](../simulation/isaac_sim/pegasus_scene_setup.md) | Build scenes with Pegasus extension | Intermediate |

### 🤖 Robot Autonomy Tutorials

Understand the autonomy stack and configure different operation modes.

| Tutorial | Description | Level |
|----------|-------------|-------|
| [Autonomy Modes](../robot/autonomy_modes.md) | Understand onboard vs offboard stack placement | Beginner |
| [System Architecture Overview](../robot/autonomy/system_architecture.md) | Deep dive into the layered architecture | Intermediate |
| [Module Integration](../robot/autonomy/integration_checklist.md) | Add custom modules to the stack | Advanced |

### 🧩 Modular AirStack Tutorials

Compose the autonomy software from modules, stacks, and fleets.

| Tutorial | Description | Level |
|----------|-------------|-------|
| [Modular AirStack Walkthrough](modular_airstack.md) | Fly a reference stack, add a module, make your own stack, scale to a fleet | Beginner |
| [AirStack Modules](../development/modules.md) | Pull in pinned external capability repos with `airstack module add` | Intermediate |
| [AirStack Stacks](../development/stacks.md) | Self-contained topology folders with CI-observed wiring | Intermediate |
| [AirStack Fleets](../development/fleets.md) | Declare whole deployments (robots, vehicles, stacks) in one YAML file | Intermediate |

### 📡 Ground Control Station Tutorials

Learn to monitor and control robots from the GCS.

| Tutorial | Description | Level |
|----------|-------------|-------|
| [User Interface Basics](../gcs/usage/user_interface.md) | Navigate the GCS interface | Beginner |
| [GCS Foxglove Visualization](../gcs/foxglove.md) | Visualize the fleet in Foxglove | Beginner |
| [Adding Waypoints & Geofences](../gcs/waypoints_and_geofences.md) | Interactive route and area editors | Intermediate |

### 💻 Development Tutorials

Customize AirStack and add your own algorithms.

| Tutorial | Description | Level |
|----------|-------------|-------|
| [Fork Your Own Project](../development/beginner/fork_your_own_project.md) | Fork or template AirStack for your project | Beginner |
| [Development Environment Setup](../development/beginner/development_environment.md) | Configure your IDE and tools | Beginner |
| [AI-Assisted Development](../development/advanced/ai_agent_guide.md) | Use AI agents to accelerate development | Intermediate |

### 🚁 Real World Deployment Tutorials

Deploy AirStack to real hardware and fly autonomous missions.

| Tutorial | Description | Level |
|----------|-------------|-------|
| [Deploying to Hardware](../real_world/deploying_to_hardware.md) | Install AirStack on Jetson or VOXL | Intermediate |
| [HITL Testing](../real_world/HITL/index.md) | Hardware-in-the-loop testing workflow | Intermediate |

---

## Learning Paths

Choose a path based on your goals:

**🎯 I want to develop algorithms:**

1. Complete Getting Started (you are here! ✓)
2. [Modular AirStack Walkthrough](modular_airstack.md)
3. [System Architecture](../robot/autonomy/system_architecture.md)
4. [AirStack Modules](../development/modules.md) and [AirStack Stacks](../development/stacks.md)
5. [Development Environment](../development/beginner/development_environment.md)
6. [AI-Assisted Development](../development/advanced/ai_agent_guide.md)

**🚁 I want to deploy to hardware:**

1. Complete Getting Started (you are here! ✓)
2. [Autonomy Modes](../robot/autonomy_modes.md)
3. [HITL Testing](../real_world/HITL/index.md)
4. [Deploying to Hardware](../real_world/deploying_to_hardware.md)

**🎮 I want to create custom simulations:**

1. Complete Getting Started (you are here! ✓)
2. [Spawning Drones](../simulation/isaac_sim/spawning_drones.md)
3. [Custom Scene Setup](../simulation/isaac_sim/scene_setup.md)
4. [Pegasus Scene Setup](../simulation/isaac_sim/pegasus_scene_setup.md)

**👥 I want to coordinate multi-robot teams:**

1. Complete Getting Started (you are here! ✓)
2. [AirStack Fleets](../development/fleets.md)
3. [AirStack Stacks](../development/stacks.md) (split onboard/offboard stacks)
4. [Ground Control Station Basics](../gcs/usage/user_interface.md)
5. [GCS Foxglove Visualization](../gcs/foxglove.md)

---

## Additional Resources

- **[Development Guide](../development/index.md)** - Comprehensive developer documentation
- **[System Architecture](../robot/autonomy/system_architecture.md)** - Understanding the autonomy stack
- **[Contributing Guide](../development/intermediate/contributing.md)** - How to contribute to AirStack
- **[About & FAQ](../about.md)** - Project information and frequently asked questions