# User Interface

The operator interface is **Foxglove Studio**, extended with the AirStack panels installed by the GCS container:

- **3D fleet view** — per-robot meshes, trajectories, global plans, and maps in a shared global frame ([GCS Foxglove Visualization](../foxglove.md))
- **Robot Tasks panel** — send takeoff / land / navigate / trajectory / search / exploration commands per robot
- **Waypoint and Polygon editors** — click-to-place routes and geofence areas ([Adding Waypoints and Geofences](../waypoints_and_geofences.md))
- **Per-robot tabs** — camera and depth feeds for each robot in the rendered layout

## Debugging tips

Launch just the GCS container:

```bash
airstack up gcs                               # desktop profile
docker compose --profile deploy up gcs-real   # deployed GCS hardware
```

Get a shell in the running container:

```bash
docker exec -it airstack-gcs-1 bash
```
