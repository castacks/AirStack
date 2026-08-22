# User Interface

The GCS control panel shows per-robot status: **Ping**, **Recording**, and **Battery** (voltage and percentage when the robot's MAVROS battery topic is bridged to the GCS).

The main operator interface is **Foxglove Studio** — see [GCS Foxglove Visualization](../foxglove.md) and [Adding Waypoints and Geofences](../waypoints_and_geofences.md).

## Debugging tips

Launch just the GCS container:

```bash
docker compose up gcs          # desktop profile
docker compose --profile deploy up gcs-real   # deployed GCS hardware
```

Run docker in interactive mode:

```bash
docker exec -it gcs /bin/bash
```
