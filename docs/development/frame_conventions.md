
AirStack uses the East-North-Up (ENU) coordinate system. The robot's `map` frame is expected to be in ENU. However, the `world` frame is free to be in any coordinate system, as long as a suitable transform is provided.

Isaac Sim follows the Forward-Left-Up (FLU) coordinate system. This means that the X-axis points forward, the Y-axis points left, and the Z-axis points up. More info on Isaac Sim frames can be found [here](https://docs.omniverse.nvidia.com/isaacsim/latest/reference_conventions.html).

Read [Scene Setup](../simulation/isaac_sim/scene_setup.md#frame-conventions) for more information on how Isaac Sim's FLU is converted to ENU for AirStack.

