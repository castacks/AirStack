# Foxglove connectivity

The workspace had no listener on TCP 8766 at inspection. A bounded connection to `127.0.0.1:8766` was refused. GCS was created and had never started. The internal transport gate failed. External Mac visualization was untested.

When a GCS endpoint is operational, forward the workspace port from the Mac and open `ws://localhost:8766` in Foxglove. Record connection success and which of RGB, depth, odometry, map and TF display.

```bash
osmo workflow port-forward airstack-dev-314 workspace --port 8766:8766 --connect-timeout 86400
```
