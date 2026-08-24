# `full_macvo` — trunk reference stack

Full autonomy with **MAC-VO** as the disparity source for the local planner.
One of the presets absorbing the `local_*.launch.xml` variant explosion into
named stacks a few include lines apart (RFC #379 §3).

## What it launches

Wrap form (E1): identical to [`full_default`](../full_default/README.md)
except:

1. Perception is included with `launch_macvo:=true`, so the `macvo_ros2` node
   runs and publishes `/$ROBOT_NAME/perception/macvo/disparity` (plus macvo
   odometry/pose/point-cloud topics).
2. The Local layer include passes
   `local_disparity_in_topic:=/$ROBOT_NAME/perception/macvo/disparity` and
   `local_depth_in_topic:=none`, wiring the planner's disparity input to
   MAC-VO's real output topic.

## Supersedes the broken legacy variant

This stack **supersedes**
`robot/ros_ws/src/local/local_bringup/launch/local_macvo_obstacle_avoidance.launch.xml`,
which is broken two ways and is left untouched:

- It passes `odometry_in_topic` / `disparity_in_topic` / `depth_in_topic` —
  arg names missing the `local_` prefix, so `local.launch.xml` never sees them
  and silently uses its stereo defaults.
- Its disparity topic `/$ROBOT_NAME/macvo/disparity` is stale: macvo launches
  inside the `perception` namespace and actually publishes
  `/$ROBOT_NAME/perception/macvo/disparity`.
- It also never enabled `launch_macvo` in perception, so the macvo node it
  pointed at was not even running.

## How to run

```bash
airstack up --stack full_macvo --sim isaac --robots 1
airstack ready
```

## Known limits

- Wrap form: cross-module remaps still live inside the layer bringup launch
  files, not in `stack.launch.xml`.
- `stereo_image_proc` still runs alongside MAC-VO (perception's
  `launch_stereo_image_proc` default is kept `true`): the stereo point cloud
  feeds other consumers, so this stack runs both estimators. A leaner
  macvo-only preset can flip that arg once downstream consumers are audited.
- MAC-VO is GPU-heavy; expect reduced sim real-time factor on a shared GPU.
- No committed equivalence baseline exists for this topology (the legacy
  variant never worked), so the first `wiring.md` snapshot IS the baseline.
- `modules.repos` pins no external modules yet; `docker-compose.yaml` is a
  stub (trunk compose profiles provide all services).

## wiring.md

Generated — the commit arrives with the first snapshot run of
`airstack test -m wiring --stack full_macvo`. Once committed, CI drift-checks
the running graph against it.
