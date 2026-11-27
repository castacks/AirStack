# Static Isaac geometry live retry R4

27 November 2026 UTC. **FAIL overall. Live camera and both depth controls passed.** The run stopped at the first Isaac `/clock` sample wait. No 10 m issue or target frame, exact-time TF, or geometry receipt was produced. This was the single authorized retry. [Machine-readable gate results](static-isaac-geometry-r4/retry-verification.json)

## Preflight and isolation

The checkout matched `2f6e80f8a549d41df5fa36c3ff5b5a744b83383d` on `ore-ms-orb-development`. All 63 CPU tests passed, including the seven frozen receipt tests. The cached Isaac Sim 5.1.0 image, frozen USD stage and fixture, and staged exporter matched their pins. The stopped diagnostic container passed structured inspection with exactly the approved scratch binds, no network, private IPC, and privileged mode disabled. The host issued one start under the 360-second deadline. Docker recorded 99.69 seconds elapsed. [Camera initialization qualification](STATIC_ISAAC_CAMERA_INITIALIZATION_R4.md)

Live readback confirmed that the render product used `/World/ReceiptCamera` on the pinned stage. The authored and effective USD world matrices were identical. The origin was `[0, 0, 1]` m. The optical axes were +X, −Z, and +Y in stage coordinates, within the frozen roundoff allowance. The independent camera review derived 33 × 25 pinhole intrinsics with `fx = fy = 20` and `cx = 16, cy = 12` in integer-centered pixel coordinates. The exporter preserved its authored and effective camera readbacks.

## Independent control verification

Both observations retained raw native axial depth and instance IDs, the target survey, camera metadata, matching Replicator references and native simulation timestamps. The independent review decoded the pixels from the arrays and intersected the corresponding camera rays with the surveyed quad. Both pixels in each control resolved to `/World/ReceiptTarget`. All ranges below are metres.

| Control | Pixel | Measured axial depth | Analytic ray range | Measured ray range |
| --- | --- | ---: | ---: | ---: |
| 5 m | Center (16, 12) | 5.0 | 5.0 | 5.0 |
| 5 m | Off-axis (18, 12) | 5.000000476837158 | 5.024937810560445 | 5.024938289775858 |
| 15 m | Center (16, 12) | 15.0 | 15.0 | 15.0 |
| 15 m | Off-axis (18, 12) | 15.0 | 15.074813431681335 | 15.074813431681335 |

The 5 m pair carried native reference `[7702, 30]` and resolved time `0.30000000447034836 s`. The 15 m pair carried `[7704, 30]` and resolved time `0.4000000059604645 s`. These are control acquisition timestamps. The frozen two-second gate requires a separate observed ROS simulation-clock sequence.

## First failed gate and evidence

The ROS bridge loaded rclpy and the `ros_clock_setup` phase completed. On the epoch step, the worker did not receive an Isaac `/clock` sample within the configured three-second wait. Lifecycle line 16 records the traceback. No raw clock file was written. The evidence does not distinguish a missing graph execution, a publisher or discovery problem, or a subscriber-delivery problem. The 10 m observations, two-second interval, exact-time TF lookup, real receipt verification and M08 binding remained unexecuted.

Kit logged shutdown and Docker recorded a clean exit `0`. The supervisor rejected the incomplete sequence, returned `1` and wrote `FAILED.json`. No completion marker exists. The diagnostic container was removed after inspection. Operational container IDs, image IDs, start times, restart counts and status matched before and after, and prior R1–R4 evidence and frozen M05–M08 checksums verified. The previous R4 result remains unchanged.

The raw evidence directory is `/root/ore-ms-artifacts/static-isaac-geometry-r4-retry.XUIjeQHu`. It contains the two control arrays and surveys, camera setup readbacks, journals, independent reviews, exit record, launcher inputs and `SHA256SUMS.txt`. All 74 hashes verified. The manifest digest is `463b63e32eef35d191f74e11ff1cebbde0057e0caee936ddde6c73fc42535f7e`.

The next dependency is a witnessed Isaac `/clock` epoch under the isolated render-step sequence. Instrument graph execution, publisher discovery and subscriber delivery separately before seeking any further execution authorization. Keep the observed-clock and exact timing requirements unchanged. No further simulator run, attenuation experiment, M12 or M13 work was started.
