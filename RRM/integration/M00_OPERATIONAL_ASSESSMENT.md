# M00 operational assessment

Observed 2026-11-27; timestamps and raw results in `evidence/M00/runtime-probes.json` and `passive-runtime.json`. Probes inspected metadata, imported NumPy with bytecode disabled, and created passive ROS subscriptions for approximately 10 wall seconds. No services, launch/configuration files, image pins or running missions were changed.

| Required reference area | Existing source / historical reference | Current observation and limit |
|---|---|---|
| Isaac/robot startup | Reference README clock-epoch rule and later scene-launch reconciliation | Isaac container start 18:41:25.413772267Z; robot 18:41:26.092998749Z. Container order is consistent; actual Isaac process epoch versus retained TF/controller state was not established |
| NumPy compatibility | `.env` pins Isaac manifest 3e08f31c…; Dockerfile.isaac-ros pins/asserts 1.26.4 | Running Isaac interpreter reports 1.26.4; no reinstall needed. Version match alone does not prove exact image/layer equivalence |
| Camera | Reference README render-writer failure symptoms | 236 messages in 10.029 s; last 480×300 with 432,000 bytes. No image interpretation or acquisition-synchronization qualification |
| LiDAR / Foxglove VDB | Reference README Ouster and VDB checks | 77 raw point-cloud messages, last 12,128 points. Candidate VDB visualization topic returned zero samples; map validity is unknown, not proof of absent map |
| Foxglove WebSocket / tunnels | Reference local endpoint ws://127.0.0.1:8766 | Docker maps host TCP8766 to GCS8765; host listener observed. Browser connection, WebSocket protocol and laptop/OSMO tunnel were not tested |
| WebRTC | Livestream compose TCP49100 and UDP49099 | Both ports published, TCP49100 listener observed. UDP media/browser rendered video not tested |
| OSMO identification/lifecycle | Branch-scoped workflow and optional helper state file | No osmo CLI or /root/.airstack/osmo-state available here; workflow identity unknown. No workflow was submitted, selected, restarted or cancelled |
| Scene loading | Catalog shortname office versus Pegasus Office; manual scene state must be established | printenv for scene/scale/script returned no values (exit 1); effective scene not inferred. No scene load/switch performed |
| Safety / readiness | Reference readiness is limited; November status reports actuation failure | 50 actuation_ready samples, latest false. Current actuation admission is not ready. Candidate /robot_1/mavros/state returned no samples; arming, grounding and safe state remain unknown |

Fresh /clock samples were received (242). Receipt sample counts are neither acquisition frequency qualification nor end-to-end latency. No ULog, independent physical truth, parameter readback, exact-binary attestation or full seven-gate readiness evaluation was acquired. These observations do not admit execution.

Reference conflicts retained: the README mixes old five-task prototype status, October fix/deployment instructions and later console behaviour. November UAV status is newer and explicitly source/episode-scoped. README worker image 6a6f1ea7… differs from branch workflow 8ffbf30a…; neither establishes a running worker or permits changing it. Do not use historical restart/pip/scene recipes without an approved future change and fresh evidence. No historical fix was reapplied.

Dependencies identified: Git and Python standard library for M00; Pydantic >=2.0 for the protected reference core; optional NumPy/OpenCV/pytest/PyYAML for reference UAV/inspection suites; ROS 2 Jazzy, FastDDS, task_msgs, MAVROS, PX4, Docker/NVIDIA runtime, Isaac/Pegasus for integrated SIL; model/GPU/Cosmos dependencies only for separately authorized learned reasoning work. M00 installed none. Controlled E4 exact executable identities govern reproduction rather than current tool versions or tags.
