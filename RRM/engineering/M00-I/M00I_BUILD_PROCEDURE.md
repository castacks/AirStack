# Narrow offline native build procedure

This procedure concerns compilation only. No robot, simulator, ROS node, service, container startup or deployment is authorized. Current host assessment is I2; native commands were not executed because mandatory dependencies are absent.

1. Use an **existing, preloaded** AirStack robot development environment with Jazzy underlay and the repository at the recorded revision. Do not create/start/pull a container under M00-I. If none is accessible, stop and supply this handoff for PD review.
2. Record image immutable digest, OS/architecture, compiler/CMake versions, exact ROS/ament/MAVROS/rcl_interfaces/std_msgs/YAML package versions, generated header/typesupport locations and source checksum. Missing package/header/ABI evidence must remain explicit. Do not install or fetch dependencies. An ordinary Jazzy SDK alone does not establish approved runtime containment.
3. Work from `/root/AirStack` with write access limited to `RRM/`. Do not source robot `.bashrc`, invoke bws/autolaunch or build the whole workspace. The native script accepts only output directories inside RRM and does not execute an environment setup script.
4. Run the read-only conditional builder:

   ```sh
   python3 -B RRM/engineering/M00-I/build_native.py
   ```

   It scans `/opt/ros/jazzy`, `/usr`, `/usr/local` and existing robot workspace merged/isolated install prefixes. It checks original guard identity, C++/CMake availability, real CMake configs, version evidence, generated headers and required shared libraries. Exit2 means no native build verification; consult the JSON outcome to distinguish blockers from incompatibility. Exit0 requires a linked guard binary. Python compilation alone never changes that outcome.
5. Only when the complete genuine dependency inventory passes, the script invokes direct CMake with `RRM/engineering/M00-I/guard_host` as source and the ignored RRM-only `native-output/` directory as build output. It passes the reviewed upstream include directory, explicit prefix path and detected C++ compiler, suppresses inherited CMake/ament/colcon discovery paths, and records configure/compile logs, commands and binary hash. No install target or executable invocation occurs. Linking is verified by successful build plus expected binary existence; this is not an ABI/runtime smoke test. Future execution of this branch remains subject to actual local dependency identity and review.
6. The direct CMake file requires ament_cmake, rclcpp, ament_index_cpp, mavros_msgs, rcl_interfaces, std_msgs and yaml-cpp. Compiler target is C++17. `interface_bringup` is runtime-only for package/config discovery; passing its explicit config to a future running guard requires separately authorized integration. The script does not fake these dependencies.
7. Run only affected offline regression coverage:

   ```sh
   python3 -B RRM/engineering/M00-I/run_regression.py
   ```

   Results are written under I. Do not rerun historical validators that write old evidence. The I validator checks all prior protected bytes and sealed evidence without updating history.

These commands write fresh I engineering evidence and are intended for a new build handoff/checkpoint, not a request to rewrite the committed I record in place. Archive new results under an authorized successor milestone. Compilation/linking does not authorize running the resulting binary or passing the runtime isolation gate.
