# `neurosymbolic`: debug stack for the asm_neurosymbolic module

This stack exists only to run and debug the
[asm_neurosymbolic](https://github.com/COA-SEI/asm_neurosymbolic) module against this trunk.
The method lives entirely in the module. This folder only wires it in.

It is `full_default` with one change: the per-robot random-walk global planner is replaced
by the neurosymbolic **team**. The team is a single process, included on `robot_1` only.
From there it:

- reaches every robot in that robot's own ROS domain;
- serves `tasks/semantic_search` (`task_msgs/SemanticSearchTask`) on each robot;
- writes each robot's canonical `global_plan`, which MIGHTY follows.

The module's detectors run as its compose sidecar, `neurosymbolic-perception`.

## Run

Use any Isaac scene. For example, two drones in the Retro Neighborhood stage, with LiDAR
for MIGHTY:

```bash
# modules.repos pins asm_neurosymbolic (private: cloning needs COA-SEI access);
# to debug a local checkout instead: airstack module add /path/to/asm_neurosymbolic
ENABLE_LIDAR=true airstack up --sim isaac --robots 2 --stack neurosymbolic --scene retro-neighborhood
```

Take off, then send each robot a search goal. The polygon is in that robot's own `map`
frame:

```bash
# in a robot container, once per robot (ROS_DOMAIN_ID=<n>, /robot_<n>/...)
ros2 action send_goal /robot_1/tasks/takeoff task_msgs/action/TakeoffTask \
  '{target_altitude_m: 8.0, velocity_m_s: 1.0}'
ros2 action send_goal --feedback /robot_1/tasks/semantic_search task_msgs/action/SemanticSearchTask \
  '{query: person, search_area: {points: [{x: -40, y: -40}, {x: 40, y: -40}, {x: 40, y: 40}, {x: -40, y: 40}]},
    min_altitude_agl: 5.0, max_altitude_agl: 20.0}'
```

The first goal starts the search. A goal on another robot over the same area joins it.
Cancelling a goal takes that robot out of the search.

Where to look:

- The team's log is in `robot_1`'s bringup output.
- `/robot_<n>/neurosymbolic/found` carries one point per person found.
- To keep a `team.jsonl` and sampled frames per search, set the launch arg `log_dir`.

## Lite planner without MIGHTY (SEI-COA demo setup)

The `straight` entry drops MIGHTY and the team process; the lite team, started by hand,
flies straight legs on each trajectory controller with its routes checked against the
scene's `GT_hints.json` boxes. No LiDAR. For the tornado demo scene with the rules
planner demo's 2x3 spawn grid:

```bash
ENABLE_LIDAR=false \
ISAAC_SIM_SCRIPT_NAME=../../../stacks/neurosymbolic/isaac/frozen_scene.py \
ISAAC_SIM_SCENE=omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/final_disaster_dataset_revisions/2026-10-23-coordinator/Tornado/Suburban400/level_3/1/tornado_suburban400_lvl3_1.usd \
ISAAC_SIM_SPAWN_XY="103,48;111,48;103,52;107,52;111,52;107,48" \
airstack up --sim isaac --robots 6 --stack neurosymbolic:straight --headless

# in robot_1's container (M=/root/AirStack/modules/asm_neurosymbolic; GT files from the scene's folder)
ros2 launch neurosymbolic_lite neurosymbolic_lite.launch.xml prior_file:=GT_hints.json people_file:=GT_people.json \
  mission_dir:=$M/neurosymbolic_lite/missions/tornado_suburban400 robots:=robot_1,robot_2,robot_3,robot_4,robot_5,robot_6
```

Then take off and send goals as above, with `max_flight_speed` for the cruise speed.
`frozen_scene.py` references a flattened scene_gen dataset USD's `/World` (those have no
defaultPrim); `ISAAC_SIM_SPAWN_XY` takes one `x,y` per robot, `;`-separated, or a single
`x,y` row centre.
