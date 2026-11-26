# Foxglove performance

Two pod-local WebSocket connections opened successfully and delivered data for approximately 12 s each. The second connection was newly opened after the first. Payload totals were 289.7 MB and 285.1 MB, or 24.1 MB/s and 23.7 MB/s, excluding WebSocket and TCP overhead. The first run received 292 RGB frames and 282 depth frames. Its largest observed gaps were 135 ms for RGB, 141 ms for depth and 2.25 s for odometry. Receipt gaps do not establish publisher drops. There were no comparable upstream sequence counters, synchronized acquisition/receive pairs, or external network timing measurements.

Nine resource snapshots were taken in each of two windows. Median GCS CPU was 66.89% before adding a probe and 72.07% in the probe window. Isaac medians were 564.07% and 577.69%. A pre-existing connection had queued traffic in both windows, and the added probe ended before all resource snapshots completed. These descriptive numbers do not isolate the cost of Foxglove. An external disconnected/connected comparison, sustained load test, automatic reconnect through the OSMO tunnel, end-to-end latency, and sequence loss remain untested.

See the two probe JSON files, GPU CSVs and Docker JSONL in [evidence](evidence/).
