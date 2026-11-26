# Foxglove R2 browser integration

26 November 2026 UTC. Workspace `aadb4af61cb54256-f3a1f0333d394b98`, checkout `d82cfbbd52ea76d1511d2c066564e45a8f6cbc4e`. The workspace WebSocket and checked-in layout were verified. This session had no access to the workstation browser, its local port forward, or the organization's extension settings. Browser connectivity, forward target, effective panel settings and rendering therefore remain unverified. [R2 evidence](evidence/foxglove-browser-r2.json) · [Foxglove R1](FOXGLOVE_COMMUNICATION_R1.md)

## Connection and fresh messages

A new workspace connection to `ws://127.0.0.1:8766` upgraded with HTTP 101. In ten seconds it received 218 right RGB images, 197 right-depth images and 100 `/gcs/robot_markers` updates. Each stream had as many distinct header stamps as received messages, with the last later than the first. This establishes fresh data on the workspace socket over the sample window. The marker and sensor stamps use different apparent time bases, so compare progression within each stream.

The earlier screenshot shows a red exclamation beside the data source, a duplicate-channel error in **Problems**, and an inactive-tab buffering notice. The duplicate was reproduced on a working workspace socket. Open **Problems** for the exact current error. A connection-close message and a stalled receive count are stronger evidence of interrupted transport. The historical screenshot cannot establish the current browser socket state. [Foxglove interface](https://docs.foxglove.dev/docs/visualization)

### Workstation verification

1. On the workstation, set `WF` to the same workflow ID used by the attached IDE's SSH forward. Use `osmo workflow query "$WF"` to confirm that the workflow and its `workspace` task are running. Compare `hostname` in the attached IDE with the workspace identity above. If the existing port listener's target is uncertain, identify it before replacing the forward. Start an explicit forward and leave it running.

   ```bash
   osmo workflow port-forward "$WF" workspace --port 8766:8766 --connect-timeout 86400
   ```

2. In the browser, open DevTools **Network → WS** and connect Foxglove to `ws://localhost:8766`. Keep the tab in the foreground. Select the current socket and check for HTTP 101 and incoming frames across two observations several seconds apart. Open **Problems** and read the exact messages. If the workspace probe passes while the workstation socket cannot upgrade, inspect the local forward and browser connection details.
3. In a personal copy of the layout, add **Raw Messages** panels for `/robot_1/sensors/front_stereo/right/image_rect` and `/gcs/robot_markers`. Check that `header.stamp` and `markers[0].header.stamp` advance within their respective feeds. Confirm that the RGB Image and 3D panels select those feeds and render their content after the fresh connection. A stationary scene can look identical across new frames. If headers advance while a panel remains blank, inspect its topic selection and rendering errors. [Foxglove Raw Messages](https://docs.foxglove.dev/docs/visualization/panels/raw-messages)

## Right-depth display

The checked-in Image panel selects `/robot_1/sensors/front_stereo/right/depth_ground_truth` and right CameraInfo, with **Gradient**, **Value min 0.05**, **Value max 1.2**, and timestamp synchronization off. One- and two-robot layout renders retained those settings. No new template correction was justified.

Five depth frames received through the bridge were 480 × 300, little-endian `32FC1`. Each had 66,720 finite-positive pixels (46.3%) and 77,280 infinite pixels, with no zero, negative finite or NaN values. Foxglove supports this encoding directly and draws infinite pixels black. The raw measurements and encoding were preserved. [Foxglove Image panel](https://docs.foxglove.dev/docs/visualization/panels/image)

In the browser, select the depth Image panel and verify its exact topic, Gradient, and both numeric bounds, or import the checked-in template as a new layout. To isolate the display, turn off **Sync timestamps** and hide additional overlays. In Raw Messages on the depth topic, verify `encoding`, `width`, `height`, and advancing `header.stamp`. Check that finite structure is visible next to the black invalid region. For an additional visual check, temporarily select **Color mode → Color map → Turbo** with the same bounds, then restore the preferred palette. If fresh messages continue while the isolated panel stays entirely black, capture its settings and browser errors. R2 could not perform the client rendering check.

## Robot Tasks

The checkout contains AirLab CMU's `robot-commands` v2.0.0 manifest and prebuilt JavaScript. The bundle registers `Robot Tasks`, matching the `robot-commands.Robot Tasks` panel entries in the default and generated layouts. This is static compatibility evidence. No `.foxe` was present in the checked-in extension directory.

- **Web.** In the current organization, check **Settings → Extensions** for the enabled extension. If absent, users with a developer seat can drag an approved `.foxe` onto the visualization page. An organization administrator can also provide a private extension for the organization. Obtain the package from the extension owner. The active organization's package availability and seat permissions could not be verified. [Local installation](https://docs.foxglove.dev/docs/extensions/local-development) · [Extension permissions](https://docs.foxglove.dev/docs/extensions) · [Organization settings](https://docs.foxglove.dev/docs/settings)
- **Desktop.** On the workstation, use `AIRSTACK_OSMO_WF="$WF" ./airstack.sh osmo foxglove` instead of the direct forward. It copies the prebuilt bundle and opens the forward. Reload Desktop if required, check **Settings → Extensions**, connect to `ws://localhost:8766`, and import the layout. Verify the panel loads with its task controls idle. [AirStack OSMO tutorial](../../../docs/tutorials/airstack_on_osmo.md)

## VDB channel and deployment decision

The WebSocket still advertised both `Marker` and `MarkerArray` on `/robot_1/vdb_mapping/vdb_map_visualization`. The robot graph showed the named `/robot_1/vdb_mapping` publisher using `Marker`, plus bare DDS publishers for both schemas. GCS discovery showed only the bare DDS publishers on that topic. Typed ROS subscriptions received the same `Marker` stamp in domains 1 and 0. Both typed `MarkerArray` probes timed out after nine seconds. Neither raw VDB WebSocket channel delivered a message in the ten-second sample. The separate GCS aggregate delivered 100 updates, and the checked-in 3D panel displays that aggregate while keeping the raw VDB channel hidden.

DDS Router exposes the extra type, which was idle across the bounded checks. The original discovery trigger and whether the endpoint is stale remain unresolved. Keep the raw `Marker` topic and its consumers, including the robot keepalive, random-walk planner and GCS visualizer. The smallest candidate is the single-entry DDS type qualifier proposed in R1. A controlled deployment must verify one raw VDB schema, continuing typed `Marker` delivery and a live GCS aggregate before accepting that change. [R1 routing proposal](FOXGLOVE_COMMUNICATION_R1.md#duplicate-vdb-channel)

All five readiness gates passed in seven seconds. R2 adds the report, evidence and an index entry. No change to the live router, runtime, sensor data, extension bundle or checked-in layout was needed for the checks completed here.
