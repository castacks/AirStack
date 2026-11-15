# UAV physics and execution evidence

2026-11-15, `ore_proj`, base `07ff1dcbb` plus working changes. Simulation only, one GPU; Cosmos deferred. These records may inform later model work, but no model was loaded or trained.

## Control and observation chain

Public AirStack task actions drive the trajectory controller and cascaded position/velocity PID. MAVROS sends attitude/thrust through the robot interface to PX4 SITL. Pegasus receives motor references, applies rotor forces, body torques and drag, and Isaac Sim integrates rigid-body motion. Simulated sensors feed the estimator and ROS odometry. The tests use this dynamic chain; they do not teleport the aircraft.

Independent truth comes from live PhysX rigid bodies through dynamic-control. USD transforms can be stale under Fabric. Source hashes, clock observations and recorder-health histories accompany the captures. Pairing by wall receipt within 150 ms is an observation alignment bound, not acquisition synchronization.

## Vehicle parameters

| Quantity | Evidence and interpretation |
| --- | --- |
| Authored body mass | 1.5 kg; does not include every rigid body. |
| Live assembled mass | 2.834705 kg: body 1.5, lidar 1.0, camera 0.216090, four rotors together about 0.118615 kg. |
| Authored body inertia | Diagonal (0.029125, 0.029125, 0.055225) kg m²; this is not the assembled coupled inertia. |
| Live mass-weighted COM | Approximately (0.015206, -0.000086, 0.007192) m relative to body in the measured resting configuration. |
| Rotor model | Source default quadratic thrust `T = k * omega²`, `k = 8.54858e-6`, maximum angular speed 1100 rad/s; instantaneous response in this model. Defaults are not measured rotor performance. |
| Ideal hover estimate | About 902 rad/s for the assembled mass, assuming gravity 9.81 m/s² and the source coefficient. The body-only estimate of 656 rad/s understates the loaded requirement. Neither is a command or measured hover speed. |
| Ideal maximum thrust | About 41.38 N, ideal thrust/weight 1.49 under those assumptions. No real-aircraft performance qualification. |
| Drag | Source-default body-axis coefficients (0.5, 0.3, 0). Drag alone does not establish a wind field. |

No mass, collider geometry, materials, solver settings or controller gains were changed to obtain a pass. The passive tilted diagnostic changed only initial pitch and spawn height, without arming.

## Ground interaction

Resting body-center height is about 0.06175 m in PhysX while estimator ground altitude varies around zero. Small negative estimated altitude does not prove floor penetration. The takeoff action's target is absolute map z, so a command of 2 m is not universally a 2 m AGL command.

Seven live rigid bodies are present: body, four rotors, camera and lidar. Five descendant camera collision meshes remain active despite spawn code calling collider removal on the camera root. This configuration discrepancy remains documented; geometry was not deleted to improve results.

Two floor actor paths produce different records. Environment geometry supplies supporting impulses with nearly zero separation. The ground-plane actor also produces positive-gap, zero-impulse manifolds. Passive tilted settling reproduced camera gaps of about 8–22 mm with zero impulse. A contact report is therefore not automatically a physical impact.

The policy declared before the separation-aware qualified episodes allows body/floor support near ground with at most 1 mm numerical penetration. Camera/floor reports are permitted only in ground-adjacent takeoff/landing phases, below 0.25 m relative body height, with strictly positive separation and zero impulse. Rotor-floor interaction, camera penetration/impulse, unexpected pairs or airborne reports violate this case's policy. Missing separation is UNKNOWN and blocks qualification. Old flights lacking separation remain unqualified.

Impulse magnitudes per report header are not force measurements. Several headers may occur per physics step; no weight or support force was inferred from header counts.

## Measured dynamics

| Episode | Physical result and limit |
| --- | --- |
| Earlier independent vertical | Hover z 1.99568–2.04249 m, peak hover speed 0.03969 m/s; ascent peak 0.244 and descent peak 0.309 m/s despite requested 0.5 m/s. Missing contact separation prevents full qualification. |
| Separation-qualified vertical | Maximum z 2.08534 m, horizontal radius 0.14528 m, speed 0.45958 m/s; takeoff, hover, landing support and final disarm passed the declared bounded case. |
| Separation-qualified operator stop | PAUSE at 0.30999 m/s; maximum extra displacement 0.15178 m; stable stop within 1.83266 wall seconds including one-second dwell. Navigation canceled and normal landing/disarm completed. |
| Automatic source-stale stop | At fault injection 0.31397 m/s; maximum extra displacement 0.14085 m; stable-stop criterion met in 3.04970 wall seconds including one-second dwell. Detector reason receipt at +0.0225 s and new positive ack at +0.2713 s are receipt measurements. Supervisor cancellation/LAND completed recovery. |
| Earlier fixed circle | Requested radius 0.5 m / speed 0.3 m/s; observed physical peak 0.42245 m/s. Raw reference tracking RMSE 0.17174 m with frame/receipt limits. Requested speed is not a demonstrated hard bound. |
| Relative localization diagnostic | Preflight-only truth-assisted frame registration, then 1166 held-out samples: RMSE 0.02677 m, maximum 0.05345 m. Not SLAM or globally calibrated localization. |

Landing can invoke the existing stall-triggered PX4 LAND fallback near the floor. An action succeeding does not establish that the original descent controller alone completed touchdown.

Physics callbacks are approximately 0.01 simulated seconds. One measured world/wall ratio was about 0.762 and varies under load. Process deadlines and braking dwell use monotonic wall time; dynamics and source timestamps retain their own clock domains. The monitor first used a 0.25-s simulation timer; the source/acknowledgement revision now evaluates/publishes on a 0.25-s wall timer. Source and accepted-receipt ages retain a one-ROS-second limit; advancing-stamp receipt has a separate two-wall-second deadline. The probe’s two-wall-second freshness limit remains. In the automatic-stop episode, full-flight simulation/wall ratio was about 0.716, so these clocks must not be conflated.

## Transfer limits and next priorities

The restricted red-marker detector uses exact-stamp RGB/depth/calibration and ideal simulated depth. Its optical surface point is not an object center or a world-bound target. Predefined viewpoints are not perception-directed flight. Forward-camera snapshots do not establish ground-area coverage.

The historical receipt-only watchdog accepted ten-second-old data and made no retry after PAUSE rejection. The new source/acknowledgement revision addresses these contracts with isolated actual-binary tests and one bounded physical automatic-PAUSE case. A dedicated monitor received only the stale-stamp input; the main flight estimator/controller retained genuine state. Automatic LAND, exclusive mode arbitration and physical recovery under service failure remain unqualified.

Keep measured parameters, source defaults, modeling assumptions and unknowns separate in any later Cosmos information bank. Preserve all blocked and failed attempts; the passing cases do not establish repeatability or erase old missing evidence.

In the automatic-stop trace, speed first dipped below 0.1 m/s, then rebounded before settling. Displacement decreased after its maximum excursion. The acceptance rule therefore uses a full one-second low-speed dwell, not the first threshold crossing or final endpoint alone. These measurements do not identify a constant deceleration, braking force or actuator lag.

## Observation-conditioned viewpoint geometry

The camera optical-forward axis at acquisition transformed to map was approximately(0.999963,0.007918,0.003367). Horizontal projection selected0.635471m at fixed mapz2m; independent body displacement between captures was0.554962m. The0.080510m difference of lengths is not a registered vector tracking error. Capture speeds were0.01755/0.02291m/s after measured hover/settling.

With unchanged K (fx/fy≈184px,480×300), nominal pinhole FOV is approximately105.05°horizontal/78.38°vertical. Measured median optical surface depth decreased3.465420→2.904359m; inverse-depth-squared scaling from these measured depths predicts bbox-area ratio1.42368, while actual raw-frame bbox ratio is1.40741. The initial policy prediction was1.5. These are single-scene geometric diagnostics, not identified sensor/control parameters or semantic uncertainty reduction. Optical depth is not Euclidean range or target-center distance. No mass, solver or collision geometry changed.
