# Isaac Sim

The primary simulator we support is [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html). 
We chose Isaac Sim as the best balance between photorealism and physics simulation.


## USD File Naming Conventions
AirStack uses the following file naming conventions:

**Purely 3D graphics**

- `*.prop.usd` ⟵ simply a 3D model with materials, typically encompassing just a single object. Used for individual assets or objects (as mentioned earlier), representing reusable props.

- `*.stage.usd` ⟵ an environment composed of many props, but with no physics, no simulation, no robots. simply scene graphics

**Simulation-ready**

- `*.robot.usd` ⟵ a prop representing a robot plus ROS2 topic and TF publishers, physics, etc.

- `*.scene.usd` ⟵ an environment PLUS physics, simulation, or robots