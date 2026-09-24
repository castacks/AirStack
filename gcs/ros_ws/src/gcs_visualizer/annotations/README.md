# Scene annotations

Ground-truth boxes in the Isaac world frame, which is the GCS-side global ENU
`map` frame. One file per scene:

    <RESULTS_SCENE>.json            every annotated object
    <RESULTS_SCENE>_obstacles.json  the obstacle subset
    <RESULTS_SCENE>_region.json     the search area / region polygon

Written by `simulation/isaac-sim/utils/frozen_annotations.py` when a frozen
dataset cell is loaded with `GT_ANNOTATIONS=on` and `RESULTS_SCENE=<name>` —
the cell's own `GT_hints.json` and `GT_people.json`, reshaped. This directory
has to exist for that: `scene_annotations.write_annotations` skips a directory
that is absent rather than creating one, so a missing folder is a silent no-op.

Schema, matching the hand-authored files on the dataset branch:

    [{"class": str,
      "bbox_world": {"center_xyz_m": [x, y, z], "size_xyz_m": [dx, dy, dz]}},
     ...]

Nothing on this branch draws them yet — `gcs_visualizer` has no
`annotation_viz_node`. They are written for the scorer.
