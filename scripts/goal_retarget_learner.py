#!/usr/bin/env python3
"""Run drone_soccer's online goal-retargeting learner against a live policy commander.

The policy commander (``goal_retarget_enabled: true``) appends every flown policy tick to
``<run_dir>/ticks.jsonl``. This process, on the host in drone_soccer's ``gpu`` conda
environment, feeds those ticks to ``drone_soccer.goal_retarget.learner.GoalRetargetLearner``
every ``--observe-period-s``. After every update it saves pi_F as
``<run_dir>/policies/policy_<n>.zip``, appends the diagnostics to ``updates.jsonl``, and
points ``manifest.json`` at the new archive; the commander then hot-swaps it in and logs
the change. The file protocol is in ``svg_ground_control/goal_retarget_bridge.py``.

Start it with ``scripts/run_goal_retarget_learner.sh`` (sets the environment), e.g.::

    scripts/run_goal_retarget_learner.sh --barrier-dataset data_20261021

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

from svg_ground_control.goal_retarget_bridge import (  # noqa: E402
    LEARNER_LOG_FILE,
    MANIFEST_FILE,
    POLICIES_DIR,
    SESSION_FILE,
    TICKS_FILE,
    UPDATES_FILE,
    TickLogReader,
    format_update_summary,
    policy_file_name,
    rows_to_arrays,
    write_json_atomic,
)

DEFAULT_ROOT = AIRSTACK_ROOT / 'robot' / 'bags' / 'goal_retarget'
DEFAULT_DRONE_SOCCER = AIRSTACK_ROOT.parent / 'drone_soccer'
# pi_E (forward simulation, and pi_T unless refreshed), relative to the drone_soccer checkout.
DEFAULT_EXPERT = Path('drone_soccer') / 'saved_policies' / 'motor_dynamics_full.zip'
# Where docker-compose mounts drone_soccer inside the robot container.
CONTAINER_DRONE_SOCCER = '/root/drone_soccer'

# `log` writes the detailed record to <run_dir>/learner.log only; `console` prints one
# concise line per event and also lands in learner.log (it propagates to `log`).
log = logging.getLogger('goal_retarget_learner')
console = logging.getLogger('goal_retarget_learner.console')


def parse_args() -> argparse.Namespace:
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
    parser.add_argument('--observe-period-s', type=float, default=5.0,
                        help='wall time between learner passes')
    parser.add_argument('--refresh-target', action='store_true',
                        help='refresh pi_T <- pi_F on stagnation or every 120 s of flight '
                             '(default: pi_T stays locked to the expert pi_E)')
    parser.add_argument('--replay-all-accepted', action='store_true',
                        help='fit every update on all ticks accepted so far, not only the newest batch '
                             '(the best variant in the sim test)')
    parser.add_argument('--batch-size', type=int,
                        help='override GOAL_RETARGET.batch_size (accepted ticks per update)')
    parser.add_argument('--wait-s', type=float, default=600.0,
                        help='how long to wait for a run directory to appear')
    parser.add_argument('--status-period-s', type=float, default=30.0,
                        help='wall time between one-line progress reports on the console')
    parser.add_argument('--max-run-age-min', type=float, default=60.0,
                        help='without --run-dir, ignore runs whose commander started longer ago')
    return parser.parse_args()


def setup_logging(run_dir: Path) -> None:
    file_handler = logging.FileHandler(run_dir / LEARNER_LOG_FILE)
    file_handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
    log.addHandler(file_handler)
    log.setLevel(logging.INFO)
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(logging.Formatter('%(asctime)s %(message)s', '%H:%M:%S'))
    console.addHandler(stream_handler)
    console.setLevel(logging.INFO)


def quiet_libraries() -> None:
    """Keep Warp's kernel-load messages and a known torch warning off the console."""
    import warp

    if hasattr(warp, 'LOG_WARNING'):  # Warp >= 1.18: log levels
        warp.config.log_level = warp.LOG_WARNING
    warp.config.quiet = True  # older Warp
    warnings.filterwarnings('ignore', message='Converting a tensor with requires_grad=True to a scalar')


def concise_update(iterate: int, record: dict) -> str:
    """One console line per update: is it healthy, and how far did it move the policy."""
    refreshed = ' | pi_T refreshed' if record['target_refreshed'] else ''
    return (f"UPDATE {iterate} published | loss {record['loss_before']:.3f} -> {record['loss_after']:.3f} | "
            f"action change {record['rms_action_change_m']:.2f} m | median miss {record['median_miss_m']:.2f} m"
            f'{refreshed}')


def has_learner(run_dir: Path) -> bool:
    """Whether a learner already attached to ``run_dir``."""
    return (run_dir / MANIFEST_FILE).exists() or (run_dir / 'learner.log').exists()


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
    """Map a model path inside the robot container to the host checkout."""
    if container_path.startswith(CONTAINER_DRONE_SOCCER + '/'):
        return drone_soccer / container_path[len(CONTAINER_DRONE_SOCCER) + 1:]
    return Path(container_path)


def check_session(session: dict, config, start_path: Path) -> None:
    """Refuse to learn from a commander whose settings break the learner's assumptions."""
    from drone_soccer.train_config import TRAIN

    problems = []
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


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def trajectory_from_rows(rows: list[dict]):
    """The learner's per-tick `AnimationTrajectories` of one segment of tick rows."""
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
        # Not measured on hardware; only seeds the forward simulation.
        ball_orientation_wxyz=np.tile([1.0, 0.0, 0.0, 0.0], (count, 1)),
        ball_angular_velocity_body=np.zeros((count, 3)),
    )


class SegmentedLearner:
    """Feeds tick segments to one `GoalRetargetLearner`, which judges one continuous flight.

    A new segment restarts the learner's per-flight bookkeeping; ticks of the old segment
    whose t_g was never flown are dropped (verdict ``segment_ended``). pi_F, pi_T, the
    accepted ticks and the refresh clock carry over.
    """

    def __init__(self, learner) -> None:
        self.learner = learner
        self.segment: int | None = None
        self.rows: list[dict] = []
        self.total_ticks = 0

    def add(self, rows: list[dict]) -> None:
        for row in rows:
            if row['segment'] != self.segment:
                self._close_segment()
                self.segment = row['segment']
            if row['tick'] != len(self.rows):
                raise RuntimeError(f"segment {self.segment}: expected tick {len(self.rows)}, got {row['tick']}")
            self.rows.append(row)
            self.total_ticks += 1

    def _close_segment(self) -> None:
        if self.segment is None:
            return
        if self.rows:
            self.learner.observe(trajectory_from_rows(self.rows))
        dropped = len(self.learner.pending_steps)
        if dropped:
            self.learner.verdict_counts['segment_ended'] += dropped
        log.info(f'segment {self.segment} closed after {len(self.rows)} ticks; '
                 f'{dropped} ticks still waiting for t_g dropped')
        self.learner.pending_steps.clear()
        self.learner.next_tick = 0
        self.rows = []

    def observe(self) -> None:
        if self.rows:
            self.learner.observe(trajectory_from_rows(self.rows))


def main() -> None:
    args = parse_args()
    os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE', 'false')
    run_dir = find_run_dir(args).resolve()
    if has_learner(run_dir):
        raise SystemExit(f'{run_dir} already had a learner (learner.log or {MANIFEST_FILE} exists); '
                         'relaunch the policy commander for a new run')
    setup_logging(run_dir)
    quiet_libraries()
    session = json.loads((run_dir / SESSION_FILE).read_text())
    log.info(f"attached to {run_dir} (commander started {session['created']}, drone {session['drone']}); "
             f'session {session}')

    from stable_baselines3 import PPO

    from drone_soccer.ball_residual_config import BALL_RESIDUAL
    from drone_soccer.goal_retarget.learner import GoalRetargetLearner
    from drone_soccer.goal_retarget_config import GOAL_RETARGET

    config = dataclasses.replace(GOAL_RETARGET, replay_all_accepted=args.replay_all_accepted)
    if not args.refresh_target:
        # Lock pi_T to pi_E: never refresh on time or on stagnation.
        config = dataclasses.replace(config, target_refresh_period_s=float('inf'), stagnation_action_change_m=0.0)
    if args.batch_size is not None:
        config = dataclasses.replace(config, batch_size=args.batch_size)
    expert_path = args.expert or args.drone_soccer / DEFAULT_EXPERT
    start_path = host_model_path(session['model_path'], args.drone_soccer)
    check_session(session, config, start_path)
    resuming = file_sha256(start_path) != file_sha256(expert_path)
    if args.barrier_dataset is not None:
        barrier = np.asarray(BALL_RESIDUAL.barrier_corners_xy[args.barrier_dataset], dtype=float)
    else:
        if len(args.barrier_corners) < 6 or len(args.barrier_corners) % 2:
            raise SystemExit('--barrier-corners needs at least three x y pairs')
        barrier = np.asarray(args.barrier_corners, dtype=float).reshape(-1, 2)

    learner = GoalRetargetLearner(
        PPO.load(expert_path, device='cpu'), config, barrier,
        initial_flown_model=PPO.load(start_path, device='cpu') if resuming else None)
    policies_dir = run_dir / POLICIES_DIR
    policies_dir.mkdir(exist_ok=True)
    learner.flown_model().save(policies_dir / policy_file_name(0))
    log.info(f'pi_E = pi_T = {expert_path}; pi_F starts from {start_path}; '
             f'config {dataclasses.asdict(config)}; barrier {barrier.tolist()}')
    target_mode = 'pi_T refreshes from pi_F' if args.refresh_target else 'pi_T locked to expert'
    if config.replay_all_accepted:
        target_mode += ' | replay all accepted'
    barrier_name = args.barrier_dataset or 'custom barrier'
    start_note = f' | pi_F resumes from {start_path.parent.name}/{start_path.name}' if resuming else ''
    console.info(f'Learner attached to {run_dir.name} | expert {expert_path.name}{start_note} | {target_mode} | '
                 f'{barrier_name} | {config.batch_size} accepted ticks per update | '
                 f'details in {run_dir / LEARNER_LOG_FILE}')

    stop = {'requested': False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(requested=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(requested=True))

    reader = TickLogReader(run_dir / TICKS_FILE)
    flight = SegmentedLearner(learner)
    updates: list[dict] = []
    started = time.monotonic()
    next_status = started + args.status_period_s
    last_status = ''
    while not stop['requested']:
        pass_started = time.monotonic()
        rows = reader.poll()
        if rows:
            flight.add(rows)
            flight.observe()
            flight_time_s = flight.total_ticks * config.policy_dt
            log.info(f'pass: +{len(rows)} ticks (segment {flight.segment}, {flight_time_s:.0f} s flown) | '
                     f'accepted waiting {len(learner.accepted)}/{config.batch_size} | '
                     f'pending {len(learner.pending_steps)} | verdicts {dict(learner.verdict_counts)} | '
                     f'{time.monotonic() - pass_started:.1f} s')
            while learner.ready():
                record = learner.update(flight_time_s)
                iterate = len(updates) + 1
                updates.append(record)
                publish_iterate(run_dir, learner, iterate, record)
                log.info(f'UPDATE {iterate} published -> {POLICIES_DIR}/{policy_file_name(iterate)} | '
                         f'{format_update_summary(record)} | {record}')
                console.info(concise_update(iterate, record))
        if time.monotonic() >= next_status:
            next_status = time.monotonic() + args.status_period_s
            judged = sum(learner.verdict_counts.values())
            share = learner.verdict_counts['accepted'] / judged if judged else 0.0
            status = (f'  {flight.total_ticks * config.policy_dt:.0f} s flown | '
                      f'{len(learner.accepted)}/{config.batch_size} toward update {len(updates) + 1} | '
                      f'{share:.0%} of judged ticks accepted')
            if status != last_status:  # quiet while nothing flies
                console.info(status)
                last_status = status
        elapsed = time.monotonic() - pass_started
        deadline = time.monotonic() + max(0.0, args.observe_period_s - elapsed)
        while not stop['requested'] and time.monotonic() < deadline:
            time.sleep(0.1)

    summary = {
        'run_dir': str(run_dir), 'expert': str(expert_path), 'flown_start': str(start_path),
        'config': dataclasses.asdict(config),
        'barrier_corners_xy': barrier.tolist(), 'wall_time_s': time.monotonic() - started,
        'ticks_read': flight.total_ticks, 'final_verdict_counts': dict(learner.verdict_counts),
        'updates': updates,
    }
    (run_dir / 'learner_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    console.info(f'Stopped after {len(updates)} updates; summary in {run_dir / "learner_summary.json"}')


def publish_iterate(run_dir: Path, learner, iterate: int, record: dict) -> None:
    """Save pi_F, log its diagnostics, then point the manifest at it (in that order)."""
    name = policy_file_name(iterate)
    final = run_dir / POLICIES_DIR / name
    temporary = run_dir / POLICIES_DIR / f'.{name}.tmp.zip'
    learner.flown_model().save(temporary)
    os.replace(temporary, final)
    entry = {'iterate': iterate, 'policy': f'{POLICIES_DIR}/{name}', 'written_unix': time.time(), 'update': record}
    with open(run_dir / UPDATES_FILE, 'a') as file:
        file.write(json.dumps(entry) + '\n')
    write_json_atomic(run_dir / MANIFEST_FILE, entry)


if __name__ == '__main__':
    main()
