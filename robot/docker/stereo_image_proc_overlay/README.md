# Pinned stereo_image_proc dependency overlay

AirStack's robot image builds upstream image_pipeline 5.0.13 commit
`6c3df3099bc7b1ec92215719215c8eefd0d3aa69` with `valid_window.patch`.
The codeload archive checksum is verified before compilation. The overlay is
installed under `/opt/airstack/stereo_image_proc` and sourced between the apt ROS
underlay and the robot workspace. Its provenance JSON records source, patch,
patched source, executable and library hashes. The apt package remains installed.

The upstream node calculates its disparity valid window before filling the output
image dimensions. At 480x300, minimum disparity 0, range 64 and correlation
window 15, this wraps negative extents to width 4294967218 / height 4294967281.
The fix uses the available input dimensions and emits a canonical empty window
when no positive region is valid. It does not change the stereo matcher or depth.

## Build and regression

Normal robot-image builds include this overlay. To install it in an existing
development container, copy this folder into that container, then run:

```bash
bash /tmp/stereo_image_proc_overlay/build.sh /opt/airstack/stereo_image_proc
source /opt/ros/jazzy/setup.bash
source /opt/airstack/stereo_image_proc/local_setup.bash
ROS_DOMAIN_ID=92 python3 /tmp/stereo_image_proc_overlay/regression.py \
  --prefix /opt/airstack/stereo_image_proc
```

The prefix must not already exist. Builds retain their source/build workspace under
a new `/tmp/airstack-stereo-build.*` directory. The ROS regression sends synthetic
stereo images to isolated test nodes, verifies actual published windows for normal,
negative minimum-disparity and narrow-image cases, and checks each process loads
the candidate library. Use a free domain other than robot/GCS domains 1 and 0.

Installing files does not change an already running node. While grounded and idle,
restart the robot container after successful regression and verify both
`ros2 pkg prefix stereo_image_proc` and the node's loaded library. Run
`airstack ready --json` again. Existing-container installs survive restart but not
container replacement; replacement requires a robot image built with this fix.
No registry image is published by these scripts.

## Measurement limits

A disparity valid window is a rectangle in the left rectified image. Consumers
must reject out-of-bounds and empty windows, invalid/nonfinite disparities and
missing time-matched transforms. Boundary-valued disparities require separate
qualification. Window correctness does not establish stereo accuracy, semantic
target association, target coordinates, map registration or flight readiness.
