# Failure telemetry

| Failure | Evidence and outcome |
|---|---|
| PX4 setup pip failure | apt NumPy 1.26.4 had no RECORD file. The dedicated build venv fixed the local image build. |
| Runtime render failure | NumPy 2.4.6 and a writer dtype error preceded empty camera streams. The live downgrade restored frames. Image-only import still returns 2.4.6. |
| Partial boot | Entrypoint reported failed AirStack startup and kept the SSH pod alive. Docker initially listed no containers. |
| Missing odometry and map | MAVROS published under the duplicated prefix. Namespace fix restored odometry, map-to-base TF and populated markers. |
| Headless components | DROAN logged failure to initialize GLAD and exited -11. GCS-local Foxglove desktop also exited -11. The bridge stayed up. |
| External tunnel | User-reported 1011 keepalive timeout, with several MB queued on the pod-side TCP connection. Saturation is plausible. External latency and loss were not measured. |
| Browser panel | Web client reported an unknown Robot Tasks panel. The helper installed the extension into the Mac desktop path. |
| Domain visibility | Domain 0 probe saw no clock or dynamic TF. Domain 1 received both. |

The [redacted log excerpt](evidence/runtime-excerpts.txt) contains selected markers. The existing core dump and user files were preserved. No missing-asset or GPU startup fault was qualified during the repaired run.
