# OptiTrack (asm_optitrack module)

OptiTrack support — the `natnet_ros2` client, the PX4 external-vision fusion bridges, and the NatNet server emulator for Isaac Sim — ships as the standalone [asm_optitrack module](https://github.com/castacks/asm_optitrack) rather than in the AirStack trunk. To use it, add the module to your checkout:

```bash
airstack module add https://github.com/castacks/asm_optitrack --version <tag>
```

See [AirStack Modules](../development/modules.md) for how modules are declared, synced, and overlaid, and the module's own README for setup (including the host-side NatNet SDK download, which runs automatically as the module's `host_setup` hook).
