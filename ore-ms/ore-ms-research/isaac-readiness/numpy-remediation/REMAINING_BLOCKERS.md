# Remaining readiness blockers

The NumPy remediation was verified from a published digest in an existing OSMO pod. For fresh workflows, pass the qualified `ISAAC_SIM_IMAGE` digest explicitly and prevent an implicit build. The normal release tag remains the default when the override is absent. A fresh OSMO scheduling and full autostart run remains unverified.

An offline Pegasus startup could not resolve its external asset root. Normal networked startup succeeded. The clock and dynamic TF remain visible in robot domain 1 and unobserved in GCS domain 0. The recreation caused a clock reset and a brief heartbeat gap with subsequent recovery. Sustained OSMO tunnel stability, external Foxglove rendering, headless planner readiness and scientific geometry correctness remain open. This evidence does not close the earlier multispectral or fusion conditions.

Next, schedule a bounded fresh OSMO qualification using the published digest and an authenticated workstation. Verify unattended startup and repeat the sensor and bridge checks before a separate decision on the geometry pilot.
