# Recorded six-drone rules-based mission

The main planner is [`scripts/three_aoi_mission.py`](../../scripts/three_aoi_mission.py). The filename is historical: [`mission.json`](mission.json) configures two HF survey drones and four LF inspection drones.

These files are the unchanged coordinator, planning helpers and configuration from recorded run `coordinator-scene-refresh-20261023-1` (the coordinator demo v0.5.2 mission). [`planner-source.json`](planner-source.json) records their hashes.

## Logic

1. Fly the two predefined HF survey routes at 25 m.
2. Accumulate RayFronts damage predictions from spatially independent views. Require at least three supporting views before accepting a damage region.
3. Group nearby damage patches into AOIs: up to an 8 m gap and a 60 m combined extent. Wait for stable evidence inside the search bounds.
4. Give each idle LF drone its nearest eligible, unclaimed AOI. This is greedy assignment, not a fleet-wide optimizer.
5. Generate up to six inspection viewpoints at 3.5 m around and within the damage region. Require three reachable viewpoints, check route segments against known obstacle bounds, and dwell for four seconds at each inspection view. Transit detours stay within the 12 m LF ceiling.
6. Execute the tasks, mark their outcome, and assign another AOI. Return when that drone's work or the inspection time budget finishes, allowing a grace period for survey observations to finish processing.

## Main files

- [`three_aoi_mission.py`](../../scripts/three_aoi_mission.py): coordination loop and ROS task publication.
- [`survey_damage_regions.py`](../../scripts/survey_damage_regions.py): persistent damage evidence.
- [`group_aois.py`](../../scripts/group_aois.py): AOI merging.
- [`aoi_queue.py`](../../scripts/aoi_queue.py) and [`aoi_dispatch.py`](../../scripts/aoi_dispatch.py): eligibility, assignment and route-job handling.
- [`damage_aoi.py`](../../scripts/damage_aoi.py) and [`route_search.py`](../../scripts/route_search.py): viewpoints, collision checks and bounded route planning. The A* helper in `route_search.py` is not used by this recorded inspection path.

## How it ran

[`launch.py`](launch.py) starts the coordinator after simulation, perception and the per-drone executors are ready:

```bash
python3 /root/AirStack/scripts/three_aoi_mission.py /results \
  --bounds /results/obstacle_bounds.json \
  --vocabulary /results/vocabulary.json \
  --mission-config /results/mission.json
```

[`robot.sh`](robot.sh) enables the existing task executor and disables the general `rules_planner_node.py`. That executor belongs to the separate [`COA-SEI/task_assigner`](https://github.com/COA-SEI/task_assigner) module, mounted at `modules/task_assigner`; it is not vendored here.

This publishes the recorded planner source, not a complete simulation deployment bundle. Running the launcher also requires its original AirStack/ROS containers, scene assets, perception setup and recording helpers. The browser demo replays the recorded results.

AOI selection uses predicted damage, not known person locations. Scene obstacle bounds are used for route clearance, and human ground truth is used only by the separate coverage preflight to check test opportunity. Person annotations are prepared afterward and do not drive this planner. Completing an inspection route does not prove that everyone in an area was found.
