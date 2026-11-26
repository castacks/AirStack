# Foxglove Communication & Visualization R1

26 November 2026 UTC. Observed workspace hostname `aadb4af61cb54256-f3a1f0333d394b98`, branch `ore-ms-orb-development`, runtime checkout `c9911cfff`. The workspace-side bridge was reachable. This audit changed the checked-in image panel's display range. The submitted workflow ID, the user's saved layout and browser-side rendering were unavailable for verification. [Runtime Baseline R1](RUNTIME_BASELINE_R1.md) · [Verification evidence](evidence/foxglove-communication-r1.json)

## Duplicate VDB channel

| Observation | Result |
| --- | --- |
| Robot domain 1 | The VDB mapper advertises and publishes `visualization_msgs/msg/Marker`. The graph also shows bare DDS endpoints for both `Marker` and `MarkerArray`. |
| GCS domain 0 | Both publisher endpoints for `/robot_1/vdb_mapping/vdb_map_visualization` are attributed to bare DDS. A typed ROS subscription received the same `Marker` header stamp as the robot-domain subscription. |
| Typed `MarkerArray` subscriptions | Both domains timed out at nine seconds. No named ROS node publishing that type on this topic was observed. |
| GCS WebSocket | It advertised both VDB schemas under the same name. In a ten-second subscription window neither raw VDB channel delivered a frame. The separate `/gcs/robot_markers` channel delivered 98 messages with schema `MarkerArray`. |

The type conflict is already present in the DDS/ROS graph. Foxglove reports the resulting duplicate advertisement. The mapper is the verified source of the `Marker` data. DDS Router exposes the extra `MarkerArray` endpoint. Its original discovery trigger remains unknown. The sampled evidence supports an idle advertisement and does not establish whether it is stale or intermittently used. The separate GCS aggregate legitimately uses `MarkerArray` on its own topic. [GCS visualization contract](../../../docs/gcs/foxglove.md)

The GCS visualizer, the robot's `topic_keepalive`, and random-walk planner source use `Marker`. The exploration launch retains a remap for the same input, whose subscription is currently commented out in that implementation. Preserve the VDB topic name and `Marker` type. The deployed DDS Router 3.4.0 uses the shared robot-to-GCS allowlist, where the VDB entry currently matches the topic name alone. The smallest proposed deployment change is to qualify just that entry with the exact DDS type.

```yaml
  - name: "rt/$(env ROBOT_NAME)/vdb_mapping/vdb_map_visualization"
    type: "visualization_msgs::msg::dds_::Marker_"
```

Apply and qualify this in a separately controlled deployment. Before approving it, verify that the router accepts the selector, GCS receives typed VDB `Marker` data, Foxglove advertises one raw VDB schema, and `/gcs/robot_markers`, RGB, depth, odometry and R1 TF checks still pass. The live router was left untouched. [Shared allowlist](../../../robot/ros_ws/src/autonomy_bringup/config/dds_router.yaml) · [DDS Router type-filter syntax](https://eprosima-dds-router.readthedocs.io/en/v3.3.0/rst/user_manual/configuration.html#topic-filtering)

## Right-depth image

The WebSocket delivered 190 `sensor_msgs/msg/Image` messages on `/robot_1/sensors/front_stereo/right/depth_ground_truth` in ten seconds. Five deserialized frames were 480 × 300, little-endian `32FC1`, frame `camera_right`, stride 1,920 bytes. Each had 66,720 positive finite samples and 77,280 infinite samples, with no zero, negative finite, or NaN samples. Positive values ranged from about 0.052 to 10.75. The median was 0.104 and p95 was 1.089. These are values in the incoming float encoding. The R1 geometry limitations still apply.

Foxglove's Image panel handles `32FC1` directly and renders nonfinite pixels black. The checked-in layout used 0–100 for its gradient, placing the measured p95 about 1.1% along the gradient. The updated template uses 0.05–1.2, placing it about 90% along the gradient. The display clips higher finite values to the high end while the underlying ROS values remain available. The before/after templates were compared structurally. Only the two bounds changed, and the renderer preserved them in one- and two-robot layouts. Client rendering and the user's saved layout remain unverified. [Foxglove Image panel](https://docs.foxglove.dev/docs/visualization/panels/image) · [AirStack layout](../../../gcs/foxglove_extensions/airstack_default.json)

To apply the setting without touching the runtime, edit the right-depth Image panel in a personal copy of the layout. Set **Value min** to **0.05** and **Value max** to **1.2**, or import the updated template as a new layout. A large black region is still expected for the nonfinite values in this scene.

## Robot Tasks

The extension is `robot-commands` v2.0.0 from AirLab CMU. Its manifest points to the checked-in `dist/extension.js` bundle. This checkout carries the packaged JavaScript and manifest. A source-level rebuild would require the extension's source project and its build dependencies. The separate GCS action relay and robot task executors remain outside this display audit.

- **Desktop.** On the workstation, run `AIRSTACK_OSMO_WF="$WF" ./airstack.sh osmo foxglove` from the checkout. The helper copies the prebuilt bundles to `~/.foxglove-studio/extensions` by default and opens the forward. Check **Settings → Extensions** for the enabled Robot Tasks extension. Reload Foxglove Desktop if needed. The installer was verified using a temporary destination and a byte-for-byte comparison of the installed manifest and bundle.
- **Web.** The Desktop directory copy does not load the extension in a browser. Use the organization's approved extension or install a packaged `.foxe` by dropping it onto the visualization page if your seat permits local extensions. Obtain the package from the extension owner. Merely importing the layout does not install its custom panel. No browser install or action invocation was performed. [Foxglove local extension installation](https://docs.foxglove.dev/docs/extensions/local-development) · [Foxglove extension permissions](https://docs.foxglove.dev/docs/extensions)

## Connect from the workstation

Inside the GCS container, the bridge listens on `0.0.0.0:8765` in ROS domain 0 with simulated time enabled. Docker exposes workspace port `8766`. This audit observed HTTP 101 on `ws://127.0.0.1:8766` from the workspace. The workstation requires its own OSMO forward.

1. Set `WF` to the ID of the currently running AirStack workflow, and use `osmo workflow query "$WF"` to confirm it is running and has the `workspace` task. If there are multiple running workspaces, compare the hostname in the attached IDE with the hostname over a forward to the selected workflow. The R1 workspace hostname is recorded above. Use an explicit workflow ID so the saved helper state cannot select a different workspace.
2. On the workstation, keep the following command in the foreground. For Desktop with extension installation, use the helper above instead of starting a second listener on the same port.

   ```bash
   osmo workflow port-forward "$WF" workspace --port 8766:8766 --connect-timeout 86400
   ```

3. Connect Foxglove to `ws://localhost:8766` and keep the tab in the foreground while checking the RGB or depth panel. A client warning about buffered data dropped while inactive describes a client-side buffer event. A keepalive timeout or a failed connection calls for checking the forward's terminal and workflow ID. If a live feed works but a panel is unknown, check the extension independently. If port 8766 is in use, identify the existing forward and confirm its target before replacing it.

No workstation or Desktop connection was tested. No live restart, routing edit, or sensor change was performed. After the source-only layout edit, AirStack readiness passed all five gates, and the R1 state, odometry, sensor, TF and exact-stamp checks passed. MAVROS reported disarmed. [Before/after evidence](evidence/foxglove-communication-r1.json)
