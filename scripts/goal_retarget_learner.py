#!/usr/bin/env python3
"""Run drone_soccer's goal-conditioned policy update (revision 7) against a live policy commander.

The policy commander (``goal_retarget_enabled: true``) appends every flown policy tick to
``<run_dir>/ticks.jsonl``, tagged with the learner iterate that flew it. This process, on the
host in drone_soccer's ``gpu`` conda environment, feeds those ticks to
``drone_soccer.goal_retarget.plan_learner.PlanUpdateLearner`` every ``--observe-period-s``:

- A batch is the first ``batch_window_s`` (20 s) of flight of the newest iterate pi_F. Once
  it and one more plan horizon (1 s) are flown, every tick of it gets the ball's
  time-optimal plan to its goal, the goal step from the plan's gap to the real ball
  (``--goal-step-rule``, by default the planned minus the real ball position at the plan's
  end), the shift pi_F already applies (by inverting the frozen expert pi_E), and the label
  pi_E(s, g + dg), where every clip on dg only turns the goal about the ball (the step by at
  most 45 deg, the whole shift to within 90 deg of the true goal). pi_F's LoRA adapters
  are fit to the labels.
- After every update it saves pi_F as ``<run_dir>/policies/policy_<n>.zip``, appends the
  diagnostics to ``updates.jsonl``, and points ``manifest.json`` at the new archive; the
  commander then hot-swaps it in and logs the change. The next batch starts at the first
  tick the commander records as flown by that iterate.

The file protocol is in ``svg_ground_control/goal_retarget_bridge.py``. Start it with
``scripts/run_goal_retarget_learner.sh`` (sets the environment), e.g.::

    scripts/run_goal_retarget_learner.sh --barrier-dataset data_20261119

Ctrl-C stops it and writes ``learner_summary.json``. Every iterate stays on disk.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import logging
import os
import signal
import sys
import time
import warnings
from pathlib import Path

import numpy as np

AIRSTACK_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(AIRSTACK_ROOT / 'robot' / 'ros_ws' / 'src' / 'svg_ground_control'))

from drone_soccer.goal_retarget_config import GOAL_STEP_RULES, PLAN_UPDATE  # noqa: E402
from svg_ground_control.goal_retarget_bridge import (  # noqa: E402
    FORMAT_VERSION,
    LEARNER_LOG_FILE,
    MANIFEST_FILE,
    POLICIES_DIR,
    SESSION_FILE,
    TICKS_FILE,
    UPDATES_FILE,
    TickLogReader,
    format_update_summary,
    json_default,
    policy_file_name,
    rows_to_arrays,
    write_json_atomic,
)

DEFAULT_ROOT = AIRSTACK_ROOT / 'robot' / 'bags' / 'goal_retarget'
DEFAULT_DRONE_SOCCER = AIRSTACK_ROOT.parent / 'drone_soccer'
# pi_E, the frozen expert every label comes from, relative to the drone_soccer checkout:
# the goal-aligned (symmetry-free) frame expert, trained without physics randomization.
DEFAULT_EXPERT = Path('drone_soccer') / 'saved_policies' / 'goal_aligned.zip'
# Where docker-compose mounts drone_soccer and robot/bags inside the robot container.
CONTAINER_DRONE_SOCCER = '/root/drone_soccer'
CONTAINER_BAGS = '/bags'

# `log` writes the detailed record to <run_dir>/learner.log only; `console` prints one
# concise line per event and also lands in learner.log (it propagates to `log`).
log = logging.getLogger('goal_retarget_learner')
console = logging.getLogger('goal_retarget_learner.console')


def parse_args() -> argparse.Namespace:
    """Command-line options; the learner settings default to drone_soccer's `PLAN_UPDATE`."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--run-dir', type=Path,
                        help='commander run directory (default: newest under --root)')
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT,
                        help=f'directory of commander runs (default {DEFAULT_ROOT})')
    parser.add_argument('--drone-soccer', type=Path, default=DEFAULT_DRONE_SOCCER,
                        help='host drone_soccer checkout (the one the container mounts)')
    parser.add_argument('--expert', type=Path,
                        help=f'pi_E archive (default: <drone-soccer>/{DEFAULT_EXPERT}). pi_F starts '
                             "from the commander's model_path, which may be a later iterate (resume)")
    barrier = parser.add_mutually_exclusive_group(required=True)
    barrier.add_argument('--barrier-dataset',
                         help='key of BallResidualConfig.barrier_corners_xy for the current barrier')
    barrier.add_argument('--barrier-corners', type=float, nargs='+', metavar='XY',
                         help='counter-clockwise ball-centre barrier polygon x1 y1 x2 y2 ... (ENU m)')
    parser.add_argument('--goal-step-rule', choices=GOAL_STEP_RULES, default=PLAN_UPDATE.goal_step_rule,
                        help='goal step dg_curr from the planned-minus-real ball position at the plan\'s end '
                             '(final_displacement), its mean over the plan (mean_displacement), or the BPTT '
                             'gradient of the plan-tracking error (bptt)')
    parser.add_argument('--goal-step-gain', type=float, default=PLAN_UPDATE.goal_step_gain,
                        help='dg_curr = gain * displacement, or -gain * dL/dg (bptt)')
    parser.add_argument('--retarget-angle-gain', type=float, default=PLAN_UPDATE.forward.retarget_angle_gain,
                        help="multiple of dg_curr's turn of the goal applied per step, before the 45 deg limit")
    parser.add_argument('--batch-window-s', type=float, default=PLAN_UPDATE.forward.batch_window_s,
                        help='flight of one iterate per batch (s)')
    parser.add_argument('--plan-workers', type=int, default=PLAN_UPDATE.plan_workers,
                        help='processes that solve the ball plans')
    parser.add_argument('--observe-period-s', type=float, default=5.0,
                        help='wall time between learner passes')
    parser.add_argument('--wait-s', type=float, default=600.0,
                        help='how long to wait for a run directory to appear')
    parser.add_argument('--status-period-s', type=float, default=30.0,
                        help='wall time between one-line progress reports on the console')
    parser.add_argument('--max-run-age-min', type=float, default=60.0,
                        help='without --run-dir, ignore runs whose commander started longer ago')
    return parser.parse_args()


def setup_logging(run_dir: Path) -> None:
    """Detailed log to <run_dir>/learner.log, one-line events to stdout."""
    file_handler = logging.FileHandler(run_dir / LEARNER_LOG_FILE)
    file_handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
    log.addHandler(file_handler)
    log.setLevel(logging.INFO)
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(logging.Formatter('%(asctime)s %(message)s', '%H:%M:%S'))
    console.addHandler(stream_handler)
    console.setLevel(logging.INFO)


def quiet_libraries() -> None:
    """Keep a known torch warning off the console."""
    warnings.filterwarnings('ignore', message='Converting a tensor with requires_grad=True to a scalar')


def concise_update(iterate: int, record: dict) -> str:
    """One console line per update: is it healthy, and how far did it move the policy and the goal."""
    return (f"UPDATE {iterate} published | fit {record['rows_fit']}/{record['samples']} ticks | "
            f"loss {record['loss_before']:.3f} -> {record['loss_after']:.3f} | "
            f"action change {record['rms_action_change_m']:.2f} m | "
            f"goal step {record['median_current_offset_m']:.2f} m, "
            f"shift {record['median_accumulated_offset_m']:.2f} -> {record['median_applied_offset_m']:.2f} m")


def has_learner(run_dir: Path) -> bool:
    """Whether a learner already attached to ``run_dir``."""
    return (run_dir / MANIFEST_FILE).exists() or (run_dir / LEARNER_LOG_FILE).exists()


def find_run_dir(args: argparse.Namespace) -> Path:
    """The requested run directory, or the newest recent one under ``--root`` (waiting for one).

    Without ``--run-dir``, only a run whose commander started within ``--max-run-age-min``
    and that has no learner yet is taken, so a stale run of an earlier session is never
    picked up by mistake.
    """
    if args.run_dir is not None:
        if not (args.run_dir / SESSION_FILE).exists():
            raise SystemExit(f'{args.run_dir} has no {SESSION_FILE}; is it a commander run directory?')
        return args.run_dir
    deadline = time.monotonic() + args.wait_s
    announced = False
    while True:
        newest_age_s = None
        runs = sorted(path.parent for path in args.root.glob(f'*/{SESSION_FILE}'))
        if runs:
            newest = runs[-1]
            newest_age_s = time.time() - (newest / SESSION_FILE).stat().st_mtime
            if newest_age_s <= 60.0 * args.max_run_age_min and not has_learner(newest):
                return newest
        if not announced:
            stale = '' if newest_age_s is None else (
                f' (newest, {runs[-1].name}, started {newest_age_s / 60.0:.0f} min ago'
                f'{" and already has a learner" if has_learner(runs[-1]) else ""};'
                ' pass --run-dir to use it)')
            print(f'Waiting for the policy commander to create a run under {args.root}{stale} ...', flush=True)
            announced = True
        if time.monotonic() > deadline:
            raise SystemExit(f'No new run directory appeared under {args.root} within {args.wait_s:.0f} s')
        time.sleep(1.0)


def host_model_path(container_path: str, drone_soccer: Path) -> Path:
    """Map a model path inside the robot container (drone_soccer or /bags mount) to the host."""
    if container_path.startswith(CONTAINER_DRONE_SOCCER + '/'):
        return drone_soccer / container_path[len(CONTAINER_DRONE_SOCCER) + 1:]
    if container_path.startswith(CONTAINER_BAGS + '/'):
        return AIRSTACK_ROOT / 'robot' / 'bags' / container_path[len(CONTAINER_BAGS) + 1:]
    return Path(container_path)


def file_sha256(path: Path) -> str:
    """Hex SHA-256 of a file's bytes."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_session(session: dict, config, start_path: Path, expert_frame: str) -> None:
    """Refuse to learn from a commander whose settings break the learner's assumptions.

    Args:
        session: the commander's ``session.json``.
        config: the learner's `ForwardUpdateConfig` (policy_dt, goal cap).
        start_path: host path of the policy the commander started flying.
        expert_frame: policy frame of pi_E; pi_F, and so the commander's policy, must share it.
    """
    from drone_soccer.train_config import TRAIN

    problems = []
    if session.get('format_version', 1) < FORMAT_VERSION:
        problems.append(f"session format {session.get('format_version', 1)} predates {FORMAT_VERSION} "
                        '(no policy_frame); rebuild svg_ground_control in the robot container')
    elif session['policy_frame'] != expert_frame:
        problems.append(f"the commander flies a {session['policy_frame']}-frame policy; the expert "
                        f'acts in the {expert_frame} frame')
    if abs(float(session['policy_dt']) - config.policy_dt) > 1e-9:
        problems.append(f"policy_dt {session['policy_dt']} != {config.policy_dt}")
    if abs(float(session['max_goal_distance_m']) - config.max_goal_distance_m) > 1e-9:
        problems.append(f"max_goal_distance_m {session['max_goal_distance_m']} != {config.max_goal_distance_m}")
    if session.get('policy_type') != 'standard':
        problems.append(f"policy_type {session.get('policy_type')!r} is not 'standard'")
    for key, expected in (('action_low', TRAIN.action_low), ('action_high', TRAIN.action_high)):
        if not np.allclose(session[key], expected):
            problems.append(f'{key} {session[key]} != training {list(expected)}')
    if not start_path.is_file():
        problems.append(f"commander's starting policy {session['model_path']} not found on the host "
                        f'at {start_path}')
    elif file_sha256(start_path) != session['model_sha256']:
        problems.append(f"{start_path} differs from the commander's starting policy {session['model_path']}")
    if problems:
        raise SystemExit('Commander session is incompatible with the learner:\n  ' + '\n  '.join(problems))


def trajectory_from_rows(rows: list[dict]):
    """The learner's per-tick `AnimationTrajectories` of consecutive tick rows."""
    from drone_soccer.deploy.observation import quaternion_to_rotation_matrix
    from drone_soccer.scripts.visualize_bag_trajectories import AnimationTrajectories

    arrays = rows_to_arrays(rows)
    count = len(rows)
    rotations = np.stack([quaternion_to_rotation_matrix(*quat) for quat in arrays['drone_quat_wxyz']])
    return AnimationTrajectories(
        times=arrays['t'] - arrays['t'][0],
        goal_xy=arrays['goal_xy'],
        goal_update_count=0,
        drone_position=arrays['drone_position'],
        drone_velocity=arrays['drone_velocity'],
        drone_rotation=rotations,
        drone_angular_velocity_body=arrays['drone_angular_velocity_body'],
        ball_position=arrays['ball_position'],
        ball_velocity=arrays['ball_velocity'],
        # Not measured on hardware, and not read by the revision 7 learner.
        ball_orientation_wxyz=np.tile([1.0, 0.0, 0.0, 0.0], (count, 1)),
        ball_angular_velocity_body=np.zeros((count, 3)),
    )


class IterateFlight:
    """The flight of the iterate the learner is updating, as the learner's one continuous trajectory.

    The learner takes a batch from the flight of one pi_F, starting at tick 0 of the trajectory
    it is given. On hardware the commander swaps a published iterate in a tick or so later, and
    a stop or stale inputs split the flight into segments. So the learner is shown the current
    segment from the first tick flown by ``iterate`` on, up to the first tick another policy flew
    (a policy-memory swap). A new segment, a new iterate or a closed batch moves that start;
    flight before it is not learned from.
    """

    def __init__(self, iterate: int) -> None:
        """Wait for ticks of ``iterate`` (0: the commander's starting policy)."""
        self.iterate = iterate
        self.segment: int | None = None
        self.rows: list[dict] = []
        # Index in `rows` of the learner's tick 0, or None until `iterate` flies in this segment.
        self.view_start: int | None = None
        self.total_ticks = 0

    def add(self, rows: list[dict]) -> None:
        """Append newly recorded tick rows; a new segment replaces the old one."""
        for row in rows:
            if row['segment'] != self.segment:
                self._start_segment(row['segment'])
            if row['tick'] != len(self.rows):
                raise RuntimeError(f"segment {self.segment}: expected tick {len(self.rows)}, got {row['tick']}")
            self.rows.append(row)
            self.total_ticks += 1
            if self.view_start is None and row['policy'] == self.iterate:
                self.view_start = len(self.rows) - 1

    def _start_segment(self, segment: int) -> None:
        """Drop the old segment's rows; its unfinished batch is lost."""
        if self.segment is not None:
            log.info(f'segment {self.segment} closed after {len(self.rows)} ticks; '
                     f'{len(self.view_rows())} ticks of iterate {self.iterate} toward its batch dropped')
        self.segment = segment
        self.rows = []
        self.view_start = None

    def view_rows(self) -> list[dict]:
        """Rows of the learner's trajectory: from ``view_start`` while ``iterate`` flies."""
        if self.view_start is None:
            return []
        view = []
        for row in self.rows[self.view_start:]:
            if row['policy'] != self.iterate:
                break
            view.append(row)
        return view

    def await_iterate(self, iterate: int) -> None:
        """Start the next batch at the first tick ``iterate`` flies (it was just published)."""
        self.iterate = iterate
        self.view_start = next(
            (index for index, row in enumerate(self.rows) if row['policy'] == iterate), None)

    def start_after_recorded(self) -> None:
        """Start the next batch, of the same iterate, at the next tick to be recorded."""
        self.view_start = len(self.rows)


def publish_iterate(run_dir: Path, learner, iterate: int, record: dict) -> None:
    """Save pi_F, log its diagnostics, then point the manifest at it (in that order)."""
    name = policy_file_name(iterate)
    final = run_dir / POLICIES_DIR / name
    temporary = run_dir / POLICIES_DIR / f'.{name}.tmp.zip'
    learner.flown_model().save(temporary)
    os.replace(temporary, final)
    entry = {'iterate': iterate, 'policy': f'{POLICIES_DIR}/{name}', 'written_unix': time.time(), 'update': record}
    with open(run_dir / UPDATES_FILE, 'a') as file:
        file.write(json.dumps(entry, default=json_default) + '\n')
    write_json_atomic(run_dir / MANIFEST_FILE, entry)


def build_learner(args: argparse.Namespace, session: dict):
    """Load pi_E and the commander's starting policy, check the session, and build the learner.

    Returns:
        The `PlanUpdateLearner`, its config, the expert's path, the starting policy's path,
        whether pi_F resumes from a policy other than pi_E, and the barrier polygon.
    """
    from stable_baselines3 import PPO

    from drone_soccer.ball_residual_config import BALL_RESIDUAL
    from drone_soccer.goal_retarget.framed_policy import policy_frame_of
    from drone_soccer.goal_retarget.lora import actor_from_sb3
    from drone_soccer.goal_retarget.plan_learner import PlanUpdateLearner

    config = dataclasses.replace(
        PLAN_UPDATE, goal_step_rule=args.goal_step_rule, goal_step_gain=args.goal_step_gain,
        plan_workers=args.plan_workers,
        forward=dataclasses.replace(
            PLAN_UPDATE.forward, batch_window_s=args.batch_window_s, retarget_angle_gain=args.retarget_angle_gain),
    )
    expert_path = args.expert or args.drone_soccer / DEFAULT_EXPERT
    expert = PPO.load(expert_path, device='cpu')
    start_path = host_model_path(session['model_path'], args.drone_soccer)
    check_session(session, config.forward, start_path, policy_frame_of(actor_from_sb3(expert)))
    resuming = file_sha256(start_path) != file_sha256(expert_path)
    if args.barrier_dataset is not None:
        barrier = np.asarray(BALL_RESIDUAL.barrier_corners_xy[args.barrier_dataset], dtype=float)
    else:
        if len(args.barrier_corners) < 6 or len(args.barrier_corners) % 2:
            raise SystemExit('--barrier-corners needs at least three x y pairs')
        barrier = np.asarray(args.barrier_corners, dtype=float).reshape(-1, 2)

    # The plan workers start here. They inherit an ignored SIGINT, so a Ctrl-C in the
    # terminal stops only this process, which then shuts them down.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    learner = PlanUpdateLearner(
        expert, config, barrier,
        initial_flown_model=PPO.load(start_path, device='cpu') if resuming else None)
    return learner, config, expert_path, start_path, resuming, barrier


def status_line(flight: IterateFlight, learner, policy_dt: float, updates: int) -> str:
    """Progress toward the next update, for the console."""
    flown_s = flight.total_ticks * policy_dt
    judged = sum(learner.verdict_counts.values())
    accepted = (f'{learner.verdict_counts["accepted"] / judged:.0%} of judged ticks accepted'
                if judged else 'no ticks judged yet')
    if flight.view_start is None:
        progress = f'waiting for the commander to fly iterate {flight.iterate}'
    else:
        batch_s = len(flight.view_rows()) * policy_dt
        needed_s = (learner.window_ticks + learner.horizon_steps) * policy_dt
        progress = f'iterate {flight.iterate}: {batch_s:.0f}/{needed_s:.0f} s of its batch flown'
    return f'  {flown_s:.0f} s flown | {progress} | {updates} updates | {accepted}'


def main() -> None:
    """Attach to a commander run, then judge, update and publish until Ctrl-C."""
    args = parse_args()
    os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE', 'false')
    run_dir = find_run_dir(args).resolve()
    if has_learner(run_dir):
        raise SystemExit(f'{run_dir} already had a learner ({LEARNER_LOG_FILE} or {MANIFEST_FILE} exists); '
                         'relaunch the policy commander for a new run')
    setup_logging(run_dir)
    quiet_libraries()
    session = json.loads((run_dir / SESSION_FILE).read_text())
    log.info(f"attached to {run_dir} (commander started {session['created']}, drone {session['drone']}); "
             f'session {session}')

    learner, config, expert_path, start_path, resuming, barrier = build_learner(args, session)
    stop = {'requested': False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(requested=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(requested=True))
    policy_dt = config.forward.policy_dt

    policies_dir = run_dir / POLICIES_DIR
    policies_dir.mkdir(exist_ok=True)
    learner.flown_model().save(policies_dir / policy_file_name(0))
    log.info(f'pi_E = {expert_path} ({learner.policy_frame} frame); pi_F starts from {start_path}; '
             f'config {dataclasses.asdict(config)}; barrier {barrier.tolist()}')
    barrier_name = args.barrier_dataset or 'custom barrier'
    start_note = f' | pi_F resumes from {start_path.parent.name}/{start_path.name}' if resuming else ''
    console.info(f'Learner attached to {run_dir.name} | expert {expert_path.name} ({learner.policy_frame} frame)'
                 f'{start_note} | goal step {config.goal_step_rule} x{config.goal_step_gain:g}, turn x{config.forward.retarget_angle_gain:g} | {barrier_name} | '
                 f'{config.forward.batch_window_s:.0f} s of flight per update | '
                 f'details in {run_dir / LEARNER_LOG_FILE}')

    reader = TickLogReader(run_dir / TICKS_FILE)
    flight = IterateFlight(iterate=0)
    windows: list[dict] = []
    updates = 0
    started = time.monotonic()
    next_status = started
    last_status = ''
    try:
        while not stop['requested']:
            pass_started = time.monotonic()
            rows = reader.poll()
            flight.add(rows)
            view = flight.view_rows()
            if rows and view:
                learner.observe(trajectory_from_rows(view))

            # A closed batch: update pi_F, publish it, and wait for the commander to fly it.
            if learner.ready():
                record = learner.update(0, flight.total_ticks * policy_dt)
                record['iterate_flown'] = flight.iterate
                windows.append(record)
                log.info(f'batch of iterate {flight.iterate} closed ({time.monotonic() - pass_started:.1f} s '
                         f'this pass): {json.dumps(record, default=json_default)}')
                if record['updated']:
                    updates += 1
                    publish_iterate(run_dir, learner, updates, record)
                    flight.await_iterate(updates)
                    log.info(f'UPDATE {updates} published -> {POLICIES_DIR}/{policy_file_name(updates)} | '
                             f'{format_update_summary(record)}')
                    console.info(concise_update(updates, record))
                else:
                    console.info(f"batch of iterate {flight.iterate} gave no update (verdicts "
                                 f"{record['window_verdicts']}); its next batch starts now")
                    flight.start_after_recorded()

            if time.monotonic() >= next_status:
                next_status = time.monotonic() + args.status_period_s
                status = status_line(flight, learner, policy_dt, updates)
                if status != last_status:  # quiet while nothing flies
                    console.info(status)
                    last_status = status
            deadline = pass_started + args.observe_period_s
            while not stop['requested'] and time.monotonic() < deadline:
                time.sleep(0.1)
    finally:
        learner.close()

    summary = {
        'run_dir': str(run_dir), 'expert': str(expert_path), 'flown_start': str(start_path),
        'policy_frame': learner.policy_frame, 'config': dataclasses.asdict(config),
        'barrier_corners_xy': barrier.tolist(), 'wall_time_s': time.monotonic() - started,
        'ticks_read': flight.total_ticks, 'final_verdict_counts': dict(learner.verdict_counts),
        'updates': updates, 'windows': windows,
    }
    (run_dir / 'learner_summary.json').write_text(json.dumps(summary, indent=2, default=json_default) + '\n')
    console.info(f'Stopped after {updates} updates; summary in {run_dir / "learner_summary.json"}')


if __name__ == '__main__':
    main()
