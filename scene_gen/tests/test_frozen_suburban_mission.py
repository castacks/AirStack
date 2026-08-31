"""`osmo/missions/frozen_suburban_8robot.yaml` — offline, no pod, no sim.

    python3 -m pytest scene_gen/tests/test_frozen_suburban_mission.py

An OSMO iteration costs an hour of GPU and there are 24 of them, so every
coupling this file has to something OUTSIDE it is worth checking before the
pod is booked. The couplings, and what breaks when one slips:

* `SPAWN_CONFIGS` must have exactly NUM_ROBOTS entries — the Isaac launcher
  takes the fleet from the LIST, so a short one leaves robot containers with
  no airframe that never report ready, and the mission dies at the readiness
  gate with nothing to show;
* every spawn must be INSIDE its own robot's sector, on the plate, and clear
  of the geometry — the whole reason the spawns are per-sector rather than a
  kerbside cluster (benchmark skill 4c: MIGHTY flies 1.5 m/s and would spend
  half the budget in transit);
* `scene_yaml` must name an overlay that exists in the package;
* `FROZEN_SCENE` must name a cell of the dataset contract;
* the four arms must be exactly the four that are implemented, each with the
  model servers its method needs and no others.

The sectors are cut with the planner's OWN `sector.py`, so "inside its sector"
here means the same thing it will mean at run time.
"""

import json
import math
import os
import re
import sys

import pytest

yaml = pytest.importorskip("yaml")

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "simulation", "isaac-sim", "utils"))
_SB = os.path.join(_REPO, "robot", "ros_ws", "src", "global", "planners",
                   "search_baselines")
sys.path.insert(0, os.path.join(_SB, "search_baselines"))

import frozen_annotations as fa            # noqa: E402
import sector as sect                      # noqa: E402

MISSION = os.path.join(_REPO, "osmo", "missions", "frozen_suburban_8robot.yaml")
CONFIG_DIR = os.path.join(_SB, "config")
DATASET = os.environ.get("FINAL_DATASET_DIR") or os.path.expanduser(
    "~/SEI-COA/final_disaster_dataset")

ARMS = {"frontier", "lawnmower", "vlfm", "conavgpt2_team"}


@pytest.fixture(scope="module")
def mission():
    with open(MISSION, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _envs(mission):
    return mission["environments"]


def _spawns(entry):
    return json.loads(entry["SPAWN_CONFIGS"])


# ---------------------------------------------------------------------------
# shape
# ---------------------------------------------------------------------------

def test_the_sweep_is_six_cells_by_four_arms(mission):
    envs = _envs(mission)
    assert len(envs) == 24
    cells = {e["FROZEN_SCENE"] for e in envs}
    assert len(cells) == 6
    assert cells == {
        f"{d}/Suburban/level_{n}/1" for d in ("Fire", "Tornado")
        for n in (1, 2, 3)}
    for cell in cells:
        methods = {e["method"] for e in envs if e["FROZEN_SCENE"] == cell}
        assert methods == ARMS, f"{cell}: {methods}"


def test_every_environment_runs_exactly_once(mission):
    """round_robin picks environments[(i-1) % n], so iterations must equal the
    entry count or some cells are flown twice and others not at all."""
    assert mission["environment_order"] == "round_robin"
    assert mission["iterations"] == len(_envs(mission))
    names = [e["name"] for e in _envs(mission)]
    assert len(set(names)) == len(names)


def test_only_people_placement_one(mission):
    """The people axis is five casts over ONE geometry, so it is a different
    sweep. Placement `1` for every cell here."""
    for e in _envs(mission):
        assert e["FROZEN_SCENE"].endswith("/1")


def test_the_fleet_is_eight_and_the_flags_are_all_pinned(mission):
    env = mission["env"]
    assert int(env["NUM_ROBOTS"]) == 8
    # `Stack.apply_env` only MERGES, so an omitted flag inherits whatever the
    # previous mission on the pod set — and a set one swaps in a method that is
    # not being compared.
    for flag in ("CONAVGPT_BASELINE", "FRONTIER_ONLY_BASELINE",
                 "VLFM_BASELINE", "LVLM_BASELINE"):
        assert env[flag] == "false", flag
    assert env["ENABLE_LIDAR"] == "false"
    assert env["GT_ANNOTATIONS"] == "on"
    # The scene is a FILE, and SCENE_CONFIG must be present and EMPTY rather
    # than absent. This assertion used to read `"SCENE_CONFIG" not in env` and
    # it was WRONG in the same way the mission's comment was: `apply_env`
    # MERGES, so an omitted key keeps the repo .env's SCENE_CONFIG=suburb and
    # the pod builds a suburb. (2026-08-29 that is exactly what happened, once
    # the compose gap below let FROZEN_SCENE go missing too.)
    assert "SCENE_CONFIG" in env, (
        "omitting SCENE_CONFIG does not disable the procedural build — it "
        "inherits the repo .env's value. Set it to \"\".")
    assert env["SCENE_CONFIG"] == "", env["SCENE_CONFIG"]
    assert env["AIRSTACK_STACK"] == "full_mighty"
    assert env["LOCAL_PLANNER"] == "mighty"


def test_the_cells_are_addressed_somewhere_a_pod_can_reach(mission):
    """`final_disaster_dataset/` is outside the repo and is not cloned, so a
    pod has no local copy. Without FROZEN_DATASET_ROOT the launcher falls back
    to the bind mount, which does not exist there, and all 24 iterations fail
    at t=0."""
    root = mission["env"].get("FROZEN_DATASET_ROOT", "")
    assert root, "FROZEN_DATASET_ROOT unset — this mission cannot run on a pod"
    assert "://" in root, (
        f"{root} is a local path; a pod has no such mount. Upload the cells "
        "to Nucleus and point this there — the recipe is in "
        ".agents/skills/freeze-dataset-state")
    assert root.rstrip("/").endswith("final_disaster_dataset"), root


def test_the_answer_key_is_never_revealed(mission):
    """A 25 m magenta pole over every survivor group is authored deactivated in
    every cell. PEOPLE_POLES must not be set on a scored run."""
    assert "PEOPLE_POLES" not in mission["env"]
    for e in _envs(mission):
        assert "PEOPLE_POLES" not in e


def test_each_arm_starts_the_model_servers_its_method_needs(mission):
    want = {
        # method            detector, itm,   vlm
        "frontier":        ("true", "false", "false"),
        "lawnmower":       ("true", "false", "false"),
        "vlfm":            ("true", "true",  "false"),
        "conavgpt2_team":  ("true", "false", "true"),
    }
    for e in _envs(mission):
        got = (e["START_DETECTOR_SERVER"], e["START_ITM_SERVER"],
               e["START_VLM_SERVER"])
        assert got == want[e["method"]], (e["name"], got)
        # The readiness step polls {{env.model_port}}; it must name the server
        # this arm actually started, and be empty when there is none.
        expect_port = {"vlfm": "8100", "conavgpt2_team": "8000"}.get(
            e["method"], "")
        assert e["model_port"] == expect_port, e["name"]


def test_every_environment_names_an_overlay_that_exists(mission):
    for e in _envs(mission):
        path = os.path.join(CONFIG_DIR, e["scene_yaml"])
        assert os.path.isfile(path), path


def test_the_four_arms_map_to_launch_files_that_exist(mission):
    """Step 5 launches `{{env.method}}.launch.xml`; step 6 hard-codes the team
    file. A missing launch file surfaces as a failed step 20 minutes into a
    pod."""
    launch = os.path.join(_SB, "launch")
    for method in ARMS:
        assert os.path.isfile(os.path.join(launch, f"{method}.launch.xml")), \
            method


# ---------------------------------------------------------------------------
# the spawns
# ---------------------------------------------------------------------------

def test_every_environment_has_exactly_eight_spawns(mission):
    for e in _envs(mission):
        spawns = _spawns(e)
        assert len(spawns) == int(mission["env"]["NUM_ROBOTS"]), e["name"]
        for s in spawns:
            assert set(s) >= {"x_m", "y_m", "orient"}
            assert len(s["orient"]) == 4
            assert math.isclose(sum(v * v for v in s["orient"]), 1.0,
                                abs_tol=1e-3), s


def test_the_arms_on_one_cell_share_the_same_spawns(mission):
    """Four arms on identical geometry from identical start states: that is the
    premise of attributing a difference to the selection policy."""
    by_cell = {}
    for e in _envs(mission):
        by_cell.setdefault(e["FROZEN_SCENE"], []).append(e["SPAWN_CONFIGS"])
    for cell, configs in by_cell.items():
        assert len(set(configs)) == 1, cell


def test_the_spawns_are_on_the_plate(mission):
    for e in _envs(mission):
        for s in _spawns(e):
            assert -500.0 <= s["x_m"] <= 500.0, (e["name"], s)
            assert -500.0 <= s["y_m"] <= 500.0, (e["name"], s)


def test_each_spawn_is_inside_that_robots_own_sector(mission):
    """Robot N takes slice N-1 of the SAME polygon the overlay carries, cut by
    the planner's own partitioner. A drone that starts outside its sector is
    nudged back in and pays transit it should not have to."""
    seen = set()
    for e in _envs(mission):
        if e["FROZEN_SCENE"] in seen:
            continue
        seen.add(e["FROZEN_SCENE"])
        with open(os.path.join(CONFIG_DIR, e["scene_yaml"]),
                  encoding="utf-8") as fh:
            p = yaml.safe_load(fh)["search_planner"]["ros__parameters"]
        v = p["search_area_xy"]
        poly = [(v[i], v[i + 1]) for i in range(0, len(v), 2)]
        spawns = _spawns(e)
        for i, s in enumerate(spawns):
            sec = sect.sector_for(poly, len(spawns), i, mode="rect",
                                  axis="principal", margin_m=0.0)
            inside = sect.points_in_polygon([[s["x_m"], s["y_m"]]], sec)
            assert bool(inside[0]), (
                f"{e['FROZEN_SCENE']} robot_{i + 1} spawn "
                f"({s['x_m']}, {s['y_m']}) is outside its sector")


def test_the_map_extent_reaches_each_sector_from_its_own_spawn(mission):
    """The three SECTORED arms are `frame_mode: 'local'`: the grid is centred
    on the takeoff point, so the half-extent has to reach the far corner of
    that robot's sector or part of its own area is off the map, silently."""
    seen = set()
    for e in _envs(mission):
        if e["FROZEN_SCENE"] in seen:
            continue
        seen.add(e["FROZEN_SCENE"])
        with open(os.path.join(CONFIG_DIR, e["scene_yaml"]),
                  encoding="utf-8") as fh:
            p = yaml.safe_load(fh)["search_planner"]["ros__parameters"]
        v = p["search_area_xy"]
        poly = [(v[i], v[i + 1]) for i in range(0, len(v), 2)]
        half = p["map_extent_m"] / 2.0
        spawns = _spawns(e)
        for i, s in enumerate(spawns):
            sec = sect.sector_for(poly, len(spawns), i, mode="rect",
                                  axis="principal", margin_m=0.0)
            need = max(math.hypot(x - s["x_m"], y - s["y_m"]) for x, y in sec)
            assert half >= need, (
                f"{e['FROZEN_SCENE']} robot_{i + 1}: half-extent {half:.0f} m "
                f"but its sector reaches {need:.0f} m from the spawn")


@pytest.mark.parametrize("cell", sorted(
    {f"{d}/Suburban/level_{n}/1" for d in ("Fire", "Tornado")
     for n in (1, 2, 3)}))
def test_no_spawn_sits_on_a_building_a_tree_or_a_survivor(mission, cell):
    """Needs the dataset; skips without it. 5 m is the floor the planner found
    on the tightest corner sector — anything at 0 would be a drone starting
    inside a house."""
    usd = os.path.join(DATASET, cell,
                       fa.usd_name_for_cell(os.path.join(DATASET, cell)))
    if not os.path.isfile(usd):
        pytest.skip(f"{cell} is not on this machine")
    people_doc, hints_doc = fa.load_cell(usd)
    boxes = fa.obstacle_boxes(hints_doc)
    entry = next(e for e in _envs(mission) if e["FROZEN_SCENE"] == cell)
    people = [(float(r["x"]), float(r["y"]))
              for r in fa.people_records(people_doc)]
    for i, s in enumerate(_spawns(entry)):
        x, y = s["x_m"], s["y_m"]
        for b in boxes:
            c = b["bbox_world"]["center_xyz_m"]
            hx = abs(b["bbox_world"]["size_xyz_m"][0]) / 2.0
            hy = abs(b["bbox_world"]["size_xyz_m"][1]) / 2.0
            assert not (abs(x - c[0]) <= hx and abs(y - c[1]) <= hy), (
                f"{cell} robot_{i + 1} spawns inside a {b['class']}")
        assert min(math.hypot(x - px, y - py) for px, py in people) > 3.0, (
            f"{cell} robot_{i + 1} spawns on a survivor")


# ---------------------------------------------------------------------------
# the steps
# ---------------------------------------------------------------------------

def test_both_planner_deployments_are_present_and_mutually_exclusive(mission):
    """frontier / lawnmower / vlfm run ONE planner PER ROBOT CONTAINER;
    conavgpt2 runs ONE team planner on offboard-compute. Both branches live in
    the same step list and each must exit 0 on the other's iterations, or every
    sectored run fails a team step and vice versa."""
    cmds = [s["run"]["cmd"] for s in mission["steps"] if "run" in s]
    per_robot = [c for c in cmds if "{{env.method}}.launch.xml" in c]
    team = [c for c in cmds if "conavgpt2_team.launch.xml" in c]
    assert len(per_robot) == 1 and len(team) == 1
    assert 'if [ "{{env.method}}" = "conavgpt2_team" ]' in per_robot[0]
    assert 'if [ "{{env.method}}" != "conavgpt2_team" ]' in team[0]
    assert "exit 0" in per_robot[0] and "exit 0" in team[0]


def test_the_per_robot_planner_targets_the_nth_container_by_name(mission):
    """`container: robot_{n}` would exec all eight planners in robot container
    1 with only ROS_DOMAIN_ID exported — same ROBOT_NAME, so every planner
    takes sector 0 and drives robot_1."""
    step = next(s for s in mission["steps"]
                if "run" in s and "{{env.method}}.launch.xml" in s["run"]["cmd"])
    assert step["run"]["container"] == "airstack-robot-desktop-{n}"
    cmd = step["run"]["cmd"]
    assert "export ROBOT_NAME=robot_{n}" in cmd
    assert "export ROS_DOMAIN_ID={n}" in cmd


def test_the_team_planner_joins_the_shared_domain_with_the_right_camera(mission):
    """offboard-compute's own domain is 99 and its env carries no
    ZED_PITCH_DEG; both must be set explicitly or the planner sees no robots
    and unprojects every point to the wrong bearing."""
    step = next(s for s in mission["steps"]
                if "run" in s and "conavgpt2_team.launch.xml" in s["run"]["cmd"])
    cmd = step["run"]["cmd"]
    assert step["run"]["container"] == "offboard-compute"
    assert "export ROS_DOMAIN_ID=0" in cmd
    assert f"export ZED_PITCH_DEG={mission['env']['ZED_PITCH_DEG']}" in cmd


def test_the_recorded_topics_are_the_translated_gcs_copies(mission):
    """Every raw /robot_N layer is in THAT robot's takeoff-anchored map, and
    this mission's eight takeoff points are hundreds of metres apart — a bag of
    the raw names replays eight maps scattered across the plate."""
    rec = mission["record"]
    assert rec["scope"] == "gcs"
    for layer in ("occupancy", "frontiers", "voxel_map", "value_map",
                  "search/markers"):
        assert f"/gcs/{{robot}}/{layer}" in rec["topics"], layer
        assert f"/{{robot}}/{layer}" not in rec["topics"], layer
    assert "/gcs/{robot}/map_origin" in rec["topics"]
    # The 2048x2048 overhead was ~90% of a 300 GB bag.
    assert "/sim/overhead/image" not in rec["topics"]


def test_the_run_is_gated_on_sim_time_for_both_deployments(mission):
    cmds = [s["run"]["cmd"] for s in mission["steps"] if "run" in s]
    assert any("RUN_COMPLETE {robot}" in c for c in cmds)
    assert any("TEAM_RUN_COMPLETE" in c for c in cmds)


def test_results_are_keyed_by_scene_and_method(mission):
    """24 iterations write into one bind-mounted results tree; a DEST that
    omits either key silently overwrites."""
    for s in mission["steps"]:
        cmd = s.get("run", {}).get("cmd", "")
        if "results/frozen_suburban_8robot" in cmd:
            assert "{{env.RESULTS_SCENE}}" in cmd
            assert "{{env.method}}" in cmd


# ---------------------------------------------------------------------------
# the container boundary
# ---------------------------------------------------------------------------

_COMPOSE = [
    "simulation/isaac-sim/docker/docker-compose.yaml",
    "robot/docker/docker-compose.yaml",
    "robot/docker/robot-base-docker-compose.yaml",
    "gcs/docker/docker-compose.yaml",
    "docker-compose.yaml",
    "stacks/full_mighty/docker-compose.yaml",
]


def _compose_text():
    out = []
    for rel in _COMPOSE:
        path = os.path.join(_REPO, rel)
        if os.path.isfile(path):
            out.append(open(path, encoding="utf-8").read())
    return "\n".join(out)


def test_every_env_var_this_mission_sets_crosses_the_container_boundary():
    """THE 2026-08-29 FAILURE, PINNED.

    A mission's `env:` reaches `airstack up` as PROCESS env. Compose does not
    forward the process environment into a container: a variable is only
    injected if the service's `environment:` lists it. `FROZEN_SCENE` and
    `FROZEN_DATASET_ROOT` were not listed, so on the pod the launch script saw
    NEITHER, fell through to `SCENE_CONFIG` (`suburb`, inherited from the
    repo's .env), rebuilt the plat procedurally and died in `load_archetypes`
    — the archetype bake is not in git and a pod's clone has none.

    The crash named the ARCHETYPES, which is three steps downstream of the
    cause and reads like a missing-asset problem, so it is worth an hour of
    GPU to check the whole boundary offline instead.

    A key passes if a compose file either injects it (`- NAME=...` or the bare
    `- NAME` passthrough) or CONSUMES it as `${NAME}` — ISAAC_SIM_SCRIPT_NAME
    is interpolated into the tmux command line, ISAAC_SIM_CUDA_DEVICES becomes
    CUDA_VISIBLE_DEVICES, and AIRSTACK_STACK selects the stack in airstack.sh.
    """
    with open(MISSION, encoding="utf-8") as fh:
        mission = yaml.safe_load(fh)
    keys = {k for k in mission["env"] if k.isupper()}
    for e in mission["environments"]:
        keys |= {k for k in e if k.isupper()}

    text = _compose_text() + open(os.path.join(_REPO, "airstack.sh"),
                                  encoding="utf-8").read()
    injected = set(re.findall(r"^\s*-\s*([A-Z][A-Z_0-9]*)\s*(?:=|$)",
                              text, re.M))
    consumed = set(re.findall(r"\$\{([A-Z][A-Z_0-9]*)[:\-}]", text))

    missing = sorted(keys - injected - consumed)
    assert not missing, (
        "these env vars never reach any container — the mission sets them and "
        "nothing forwards them: " + ", ".join(missing))


def test_the_scene_selectors_reach_the_isaac_container():
    """The two that actually chose the wrong scene, named explicitly: a
    generic subset check would still pass if they were forwarded to the ROBOT
    container instead, which is where they are useless."""
    isaac = open(os.path.join(_REPO, "simulation", "isaac-sim", "docker",
                              "docker-compose.yaml"), encoding="utf-8").read()
    for var in ("FROZEN_SCENE", "FROZEN_DATASET_ROOT", "SCENE_CONFIG"):
        # Both service blocks: the desktop one and the headless/robot one. A
        # var forwarded to only one of them works on a bench and not on a pod.
        n = len(re.findall(r"^\s*-\s*%s(?:=|$)" % var, isaac, re.M))
        assert n == 2, f"{var} forwarded by {n} of the 2 isaac-sim services"


def test_suburb_colliders_is_passed_by_bare_name():
    """`SUBURB_COLLIDERS=${SUBURB_COLLIDERS:-}` defines it EMPTY, and empty is
    neither 'off' nor 'ground': the launcher's else-branch then collides
    /World/stage — every one of a cell's 6,000+ referenced objects — instead of
    the ground sheet. Unset has to mean unset, which is the bare-name form
    robot-base-docker-compose.yaml uses for DETECTOR_URL."""
    isaac = open(os.path.join(_REPO, "simulation", "isaac-sim", "docker",
                              "docker-compose.yaml"), encoding="utf-8").read()
    assert len(re.findall(r"^\s*-\s*SUBURB_COLLIDERS$", isaac, re.M)) == 2
    assert "SUBURB_COLLIDERS=${" not in isaac


# ---------------------------------------------------------------------------
# the execution layer
# ---------------------------------------------------------------------------

def test_the_bridge_is_checked_before_anything_takes_off(mission):
    """THE 2026-08-29 SWEEP, PINNED. `mighty_bridge` is the only thing that
    turns a plan into motion — it serves tasks/navigate, follows /global_plan,
    publishes mighty/state for the mapper and trajectory_override for the
    controller. It is also the only PYTHON node in the stack that imports
    dynus_interfaces, so a half-written shared install tree kills it and
    nothing else: mighty_node and global_mapper_ros are C++ and come up clean.
    Eight drones took off and hovered for the whole budget while the planners
    published 5,683 plans into nothing, and no step noticed for 95 minutes."""
    steps = mission["steps"]
    idx = [i for i, st in enumerate(steps)
           if "run" in st and "mighty_bridge" in st["run"]["cmd"]
           and "BRIDGE_MISSING" in st["run"]["cmd"]]
    assert idx, "no step verifies mighty_bridge is running"
    check = idx[0]
    takeoff = next(i for i, st in enumerate(steps)
                   if st.get("action", {}).get("task") == "takeoff")
    assert check < takeoff, "the bridge check must run BEFORE takeoff"
    step = steps[check]
    # Per robot, in that robot's own container — a bridge can be dead on one
    # robot and alive on the other seven.
    assert step["run"]["container"] == "airstack-robot-desktop-{n}"
    # It must FAIL the iteration; a warning would have been ignored exactly as
    # the silent zero in the summary was.
    assert "exit 1" in step["run"]["cmd"]
    assert not step.get("optional"), "the bridge check must not be optional"


def test_the_mighty_summary_reads_the_bridges_own_log(mission):
    """`mighty_bridge up`, `follower: engaging` and `dropping trajectory` are
    printed by mighty_bridge, NOT by search_planner. Grepping planner.log for
    them (as this mission did until 2026-08-30) reports 0 on a healthy run and
    0 on a dead one."""
    cmds = [st["run"]["cmd"] for st in mission["steps"] if "run" in st]
    summary = [c for c in cmds if "MIGHTY bridge" in c]
    assert summary, "the per-robot summary no longer reports on the bridge"
    for c in summary:
        for line in c.splitlines():
            if line.strip().startswith("#"):        # the comment explaining it
                continue
            if "mighty_bridge up" in line or "follower: engaging" in line \
                    or "dropping trajectory" in line:
                assert "planner.log" not in line, \
                    f"bridge string grepped out of the planner's log: {line.strip()}"
                assert "mighty_bridge_" in line or "trajectory_controller_" in line \
                    or "ros2 node list" in line, line.strip()


def test_a_few_overhead_stills_are_persisted(mission):
    """/sim/overhead/image is not recorded (2048x2048 every tick was ~90% of a
    300 GB bag) but the plate is what every trajectory plot is drawn on, so a
    few stills are written to /tmp in the GCS container and collected with the
    tee logs."""
    steps = mission["steps"]
    snaps = [st for st in steps
             if "run" in st and "/sim/overhead/image" in st["run"]["cmd"]]
    assert len(snaps) == 1, "expected exactly one overhead-snapshot step"
    step = snaps[0]
    cmd = step["run"]["cmd"]
    assert step["run"]["container"] == "gcs", "the overhead is on domain 0"
    # A missing overhead camera must not cost the iteration.
    assert step.get("optional") is True
    # The file name must reach the runner's collector glob.
    assert "/tmp/overhead_" in cmd
    # And it must not take the FIRST frame: a frozen cell renders blank until
    # its Nucleus textures arrive. Same lesson as the sim_ground gate.
    assert "MIN_STD" in cmd and "WARMUP" in cmd


def test_the_overhead_stills_are_collected_off_the_pod():
    """A file written in a container that nothing copies out does not exist."""
    runner = os.path.join(_REPO, "osmo", "workspace", "mission_runner.py")
    src = open(runner, encoding="utf-8").read()
    i = src.index("TEE_LOG_GLOBS = (")
    block = src[i:src.index(")", i)]
    assert "/tmp/overhead_*.png" in block, \
        "overhead stills are written but never copied into the iteration dir"


def test_the_perception_chain_is_checked_before_anything_takes_off(mission):
    """ROBOT_3, 2026-08-30 09:09:51, PINNED. Seven robots flew 679-1,874 m and
    robot_3 hovered at 19.8 m inside a 24 cm x 19 cm box for the whole
    108-minute iteration — 10 m of path, horizontal speed under 0.2 m/s 100 %
    of the time.

    Every check that existed passed it. Its node list and topic list were
    byte-identical to robot_1's, `mighty_bridge` was up, `tasks/navigate` was
    advertised, the search planner published 4,904 global plans. The failure
    was one layer below all of that: `global_mapper_ros`'s PointCloudCallback
    NEVER FIRED ONCE (robot_1 logged four startup lookupTransform warnings
    from inside it; robot_3 logged none), so `mighty` logged "pclptr_map_ was
    null or empty" 6,775 times and emitted ZERO "Command to execution time"
    lines against robot_1's 731. Corroborated by RSS in `resources.log`:
    mighty grows with the map it holds, and robot_3's stayed at 1,522 MB while
    the others reached 3,484-5,329 MB.

    So the gate has to reach past "is the node alive" to "is the cloud
    arriving, and does it have points in it" — an EMPTY PointCloud2 arrives at
    a perfectly good rate, which is also what a textureless (unlit) scene
    produces, so a rate check alone would pass both failures."""
    steps = mission["steps"]
    idx = [i for i, st in enumerate(steps)
           if "run" in st and "PERCEPTION_DEAD" in st["run"]["cmd"]]
    assert idx, "no step verifies the point cloud is flowing"
    check = idx[0]
    takeoff = next(i for i, st in enumerate(steps)
                   if st.get("action", {}).get("task") == "takeoff")
    assert check < takeoff, "the perception check must run BEFORE takeoff"
    step = steps[check]
    cmd = step["run"]["cmd"]
    # Per robot, in that robot's own container: exactly one of eight failed.
    assert step["run"]["container"] == "airstack-robot-desktop-{n}"
    assert not step.get("optional"), "the perception check must not be optional"
    assert "exit 1" in cmd, "it must fail the iteration, not warn"
    # IT MUST NOT SUBSCRIBE TO A BRIDGED TOPIC. `ros2 topic hz` creates a
    # reader, and a reader on an allowlisted topic activates the robot->GCS
    # dds_router bridge. On 2026-08-30 probing the point cloud that way sent
    # robot_2's ddsrouter from 114 MB to 66,225 MB (linear, ~21 MB/s — the
    # cloud's own bitrate) and erased robot_2 from the bag: 230 odometry
    # messages against ~19,730, and zero global_plan/markers/occupancy.
    # `disparity` is not on the allowlist and is a strictly stronger signal —
    # it exists only if both rectified images arrived and the matcher ran.
    assert "/perception/stereo_image_proc/disparity" in cmd
    bridged = ("/sensors/front_stereo/left/image_rect",)
    for topic in bridged:
        assert "hz \"$IMG\"" not in cmd and topic + "\"" not in cmd, \
            "must not subscribe to {0}: it is on the dds_router allowlist".format(topic)
    # The cloud may only be inspected with `topic info`, which does not subscribe.
    assert "ros2 topic info" in cmd
    assert "topic hz \"$PC\"" not in cmd and "topic echo" not in cmd, \
        "hz/echo on the bridged point cloud is what opened the unbounded route"


def test_the_planner_log_is_collected_off_the_pod(mission):
    """The per-detection confidence score exists in exactly ONE artifact —
    `/tmp/search/planner.log`, where `detector PASS` / `detector SEEN` carry
    the raw score and `detector summary` carries the histogram. No topic,
    marker or jsonl carries a number. The mission's own
    `cp /tmp/search/planner.log "$DEST"/` writes to /root/AirStack/results,
    which is NOT a mounted path, so it only moved the file to another spot
    inside the same disposable container. That is why the 2026-08-30 run could
    be shown to have found 3 of 49 people but could not be asked by how much
    each miss missed."""
    import re as _re
    runner = _re.sub(r"\s+", " ", open(
        os.path.join(_REPO, "osmo", "workspace", "mission_runner.py"),
        encoding="utf-8").read())
    m = _re.search(r"TEE_LOG_GLOBS = \((.*?)\)", runner)
    assert m, "TEE_LOG_GLOBS not found in mission_runner.py"
    globs = m.group(1)
    assert "/tmp/search/*.log" in globs, \
        "planner.log is not collected; the scores die with the pod"
    assert "/tmp/search/*.jsonl" in globs
