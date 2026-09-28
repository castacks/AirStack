#!/usr/bin/env python3
# =============================================================================
#  mtl_sortie_client.py — takeoff + SearchMission for ONE robot, from ONE
#  long-lived ROS node. Called by mtl_sortie.sh inside the robot container.
#
#  Why not `ros2 action send_goal`: every CLI call is a brand-new DDS
#  participant. On a loaded sim graph (UDP-only Fast DDS, camera streams,
#  3 robots starting at once) wait_for_server() can return before the server
#  has matched the new request writer, and the goal request is silently
#  dropped: the server never sees it (no "TakeoffTask: arming robot", no
#  status message) and the CLI waits forever. That was the intermittent
#  "a drone never takes off / never starts searching" failure.
#
#  What this client does instead:
#    * one participant for the whole sortie (discovery happens once);
#    * after the server is visible, waits until the node is --settle s old before the first goal;
#    * every attempt gets its own goal UUID; acceptance is confirmed EITHER by
#      the goal response OR by the server's own `_action/status` topic
#      (latched, published the moment a goal is accepted), so a lost goal
#      RESPONSE is not mistaken for a lost goal REQUEST;
#    * no acceptance within --accept-timeout -> resend with a new UUID
#      (up to --retries). A "rejected" reply on a retry means an earlier
#      attempt is running: that one is adopted from the status topic;
#    * if a second one of our goals is ever accepted too, it is cancelled;
#    * Ctrl-C / SIGTERM cancels the active goal (drone holds position).
#
#  usage (inside the container, workspace sourced):
#    python3 mtl_sortie_client.py --robot robot_1 --run-id RUN --alt 30 --vel 2
#        [--no-takeoff] [--dry-run] [--accept-timeout 10] [--retries 4] [--tag mtl_sortie]
#
#  Planner-agnostic: it only needs /<robot>/tasks/takeoff (task_msgs/TakeoffTask)
#  and /<robot>/search_mission (mtl_msgs/SearchMission), so the tigris_search
#  stack's tigris_sortie.sh runs this same client (--tag tigris_sortie).
#
#  exit codes: 0 ok | 2 bad args | 3 action server not available
#              4 takeoff failed | 5 goal never accepted | 6 sortie failed
#              130 interrupted
# =============================================================================
from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
import uuid as uuidlib
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Optional, Tuple

EXIT_OK, EXIT_ARGS, EXIT_NO_SERVER, EXIT_TAKEOFF, EXIT_NOT_ACCEPTED, EXIT_SORTIE, EXIT_INT = (
    0, 2, 3, 4, 5, 6, 130)

# action_msgs/msg/GoalStatus
STATUS_UNKNOWN, STATUS_ACCEPTED, STATUS_EXECUTING, STATUS_CANCELING = 0, 1, 2, 3
STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED = 4, 5, 6
ACTIVE = (STATUS_ACCEPTED, STATUS_EXECUTING, STATUS_CANCELING)
TERMINAL = (STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED)
STATUS_NAMES = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
                4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}


LOG_TAG = "mtl_sortie"   # log prefix + node-name stem; --tag sets it


def log(robot: str, msg: str) -> None:
    print(f"[{LOG_TAG} {robot} {time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ------------------------------------------------------------------ ROS-free core
@dataclass
class GoalTracker:
    """Bookkeeping for the goals ONE sortie step sent to ONE action server.

    Fed from ROS callbacks (goal responses, status arrays); queried by the
    step loop. ROS-free so it can be unit-tested. Goal ids are 16-byte `bytes`.
    """

    sent: List[bytes] = field(default_factory=list)       # in send order
    responses: Dict[bytes, bool] = field(default_factory=dict)   # uuid -> accepted?
    status: Dict[bytes, int] = field(default_factory=dict)       # our goals only
    foreign_active: set = field(default_factory=set)      # other clients' active goals
    adopted: Optional[bytes] = None
    adopted_via: str = ""

    def on_sent(self, gid: bytes) -> None:
        self.sent.append(gid)

    def on_response(self, gid: bytes, accepted: bool) -> None:
        self.responses[gid] = accepted
        if accepted:
            self._adopt(gid, "goal response")

    def on_status(self, entries: Iterable[Tuple[bytes, int]]) -> None:
        foreign = set()
        for gid, st in entries:
            if gid in self.sent:
                self.status[gid] = st
                # Seen in the server's status list = the server accepted it,
                # even if the goal response never reached us.
                self._adopt(gid, "status topic")
            elif st in ACTIVE:
                foreign.add(gid)
        self.foreign_active = foreign

    def _adopt(self, gid: bytes, via: str) -> None:
        if self.adopted is None:
            self.adopted, self.adopted_via = gid, via

    # -- queries
    def accepted(self) -> bool:
        return self.adopted is not None

    def rejected(self, gid: bytes) -> bool:
        return self.responses.get(gid) is False and gid not in self.status

    def terminal_status(self) -> Optional[int]:
        if self.adopted is None:
            return None
        st = self.status.get(self.adopted)
        return st if st in TERMINAL else None

    def duplicates_to_cancel(self) -> List[bytes]:
        """Our OTHER goals that the server also accepted and still runs."""
        if self.adopted is None:
            return []
        return [g for g, st in self.status.items() if g != self.adopted and st in ACTIVE]


@dataclass
class StepConfig:
    name: str                    # "takeoff" / "search_mission"
    accept_timeout_s: float = 10.0
    retries: int = 4
    reject_grace_s: float = 5.0  # after a reject on a retry: wait for status to show an earlier attempt
    retry_pause_s: float = 1.0


def send_until_accepted(tracker: GoalTracker, send: Callable[[], bytes], cfg: StepConfig,
                        say: Callable[[str], None], clock: Callable[[], float] = time.monotonic,
                        sleep: Callable[[float], None] = time.sleep,
                        interrupted: Callable[[], bool] = lambda: False) -> str:
    """Send goals until one is accepted. Returns 'accepted', 'rejected',
    'busy' (server runs someone else's goal), 'timeout' or 'interrupted'."""
    for attempt in range(1, cfg.retries + 1):
        if tracker.accepted():          # an earlier attempt surfaced late
            return "accepted"
        gid = send()
        say(f"{cfg.name}: goal sent (attempt {attempt}/{cfg.retries}, id {gid.hex()[:8]})")
        deadline = clock() + cfg.accept_timeout_s
        while clock() < deadline and not tracker.accepted() and not tracker.rejected(gid):
            if interrupted():
                return "interrupted"
            sleep(0.05)
        if tracker.accepted():
            return "accepted"
        if tracker.rejected(gid):
            if attempt == 1 and not tracker.foreign_active:
                return "rejected"      # the server said no to a fresh goal: a real refusal
            # A retry was refused: most likely an earlier attempt of ours IS
            # running (its response was lost). Give the latched status a moment.
            say(f"{cfg.name}: attempt {attempt} rejected - checking whether an earlier attempt is running")
            grace = clock() + cfg.reject_grace_s
            while clock() < grace and not tracker.accepted():
                if interrupted():
                    return "interrupted"
                sleep(0.05)
            if tracker.accepted():
                return "accepted"
            return "busy" if tracker.foreign_active else "rejected"
        say(f"{cfg.name}: attempt {attempt} not accepted within {cfg.accept_timeout_s:.0f} s "
            "(goal request lost in discovery?) - resending")
        sleep(cfg.retry_pause_s)
    return "accepted" if tracker.accepted() else "timeout"


def format_msg(msg, fields: Optional[Iterable[str]] = None) -> str:
    """Compact one-line rendering of a ROS message (floats to 2 decimals)."""
    names = list(fields) if fields is not None else list(msg.get_fields_and_field_types().keys())
    out = []
    for n in names:
        v = getattr(msg, n, None)
        if hasattr(v, "get_fields_and_field_types"):
            sub = [getattr(v, k) for k in v.get_fields_and_field_types()]
            v = "(" + ",".join(f"{x:.1f}" if isinstance(x, float) else str(x) for x in sub) + ")"
        elif isinstance(v, float):
            v = f"{v:.2f}"
        out.append(f"{n}={v}")
    return " ".join(out)


# ------------------------------------------------------------------ ROS layer
class RosStep:
    """One action server + its status subscription on the shared node."""

    def __init__(self, node, action_type, action_name: str, robot: str):
        from rclpy.action import ActionClient
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from action_msgs.msg import GoalStatusArray

        self.node, self.action_type, self.name, self.robot = node, action_type, action_name, robot
        self.lock = threading.Lock()
        self.tracker = GoalTracker()
        self.client = ActionClient(node, action_type, action_name)
        # Same QoS as rcl_action's status publisher (reliable, transient local):
        # we get the server's current status list even if we subscribe late.
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_sub = node.create_subscription(
            GoalStatusArray, f"{action_name}/_action/status", self._on_status, qos)
        self.handles: Dict[bytes, object] = {}
        self.last_feedback: Optional[object] = None
        self.feedback_count = 0

    # -- callbacks (executor thread)
    def _on_status(self, msg) -> None:
        entries = [(bytes(s.goal_info.goal_id.uuid), int(s.status)) for s in msg.status_list]
        with self.lock:
            self.tracker.on_status(entries)

    def _on_feedback(self, fb_msg) -> None:
        with self.lock:
            if self.tracker.adopted is None or bytes(fb_msg.goal_id.uuid) == self.tracker.adopted:
                self.last_feedback = fb_msg.feedback
                self.feedback_count += 1

    def _on_response(self, gid: bytes, fut) -> None:
        try:
            handle = fut.result()
        except Exception as exc:  # noqa: BLE001
            log(self.robot, f"{self.name}: goal response error: {exc}")
            return
        with self.lock:
            if handle is not None:
                self.handles[gid] = handle
                self.tracker.on_response(gid, bool(handle.accepted))

    # -- actions (main thread)
    def wait_for_server(self, timeout_s: float) -> bool:
        return self.client.wait_for_server(timeout_sec=timeout_s)

    def send(self, goal) -> bytes:
        from unique_identifier_msgs.msg import UUID
        raw = uuidlib.uuid4().bytes
        with self.lock:
            self.tracker.on_sent(raw)
        fut = self.client.send_goal_async(goal, feedback_callback=self._on_feedback,
                                          goal_uuid=UUID(uuid=list(raw)))
        fut.add_done_callback(lambda f, g=raw: self._on_response(g, f))
        return raw

    def handle_for(self, gid: bytes):
        """The rclpy goal handle; synthesized when only the status topic saw the goal."""
        with self.lock:
            h = self.handles.get(gid)
        if h is not None:
            return h
        try:
            from rclpy.action.client import ClientGoalHandle
            from unique_identifier_msgs.msg import UUID
            resp = self.action_type.Impl.SendGoalService.Response()
            resp.accepted = True
            h = ClientGoalHandle(self.client, UUID(uuid=list(gid)), resp)
            with self.lock:
                self.handles.setdefault(gid, h)
            return h
        except Exception as exc:  # noqa: BLE001
            log(self.robot, f"{self.name}: cannot build a goal handle ({exc})")
            return None

    def cancel(self, gid: bytes, wait_s: float = 3.0) -> None:
        h = self.handle_for(gid)
        if h is None:
            return
        try:
            fut = h.cancel_goal_async()
            end = time.monotonic() + wait_s
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.05)
        except Exception as exc:  # noqa: BLE001
            log(self.robot, f"{self.name}: cancel failed: {exc}")

    def fetch_result(self, gid: bytes, wait_s: float = 5.0):
        h = self.handle_for(gid)
        if h is None:
            return None
        try:
            fut = h.get_result_async()
            end = time.monotonic() + wait_s
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.05)
            return fut.result().result if fut.done() else None
        except Exception as exc:  # noqa: BLE001
            log(self.robot, f"{self.name}: result request failed: {exc}")
            return None


class Sortie:
    def __init__(self, args):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        except (ImportError, AttributeError, TypeError):   # rclpy without the option
            rclpy.init()
        self.rclpy = rclpy
        self.args = args
        self.robot = args.robot
        self.R = f"/{args.robot}"
        self.node = rclpy.create_node(f"{LOG_TAG}_client_{args.robot}")
        self.t_created = time.monotonic()
        self.stop = threading.Event()
        self.timed_out_flag: Optional[bool] = None
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)

    def say(self, msg: str) -> None:
        log(self.robot, msg)

    def shutdown(self) -> None:
        try:
            self.executor.shutdown(timeout_sec=1.0)
        except Exception:  # noqa: BLE001
            pass
        try:
            self.node.destroy_node()
            self.rclpy.try_shutdown()
        except Exception:  # noqa: BLE001
            pass

    # -------------------------------------------------------------- helpers
    def _subscribe_state_estimate(self) -> None:
        from std_msgs.msg import Bool
        from rclpy.qos import qos_profile_sensor_data

        def cb(msg):
            self.timed_out_flag = bool(msg.data)
        self.node.create_subscription(
            Bool, f"{self.R}/behavior/drone_safety_monitor/state_estimate_timed_out", cb,
            qos_profile_sensor_data)   # best effort matches reliable and best-effort publishers

    def _wait_state_estimate(self, timeout_s: float) -> None:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end and self.timed_out_flag is not False and not self.stop.is_set():
            time.sleep(0.1)
        if self.timed_out_flag is False:
            self.say("state estimate healthy")
        else:
            self.say(f"WARNING: no healthy state estimate after {timeout_s:.0f} s "
                     f"(last: {self.timed_out_flag}); trying anyway")

    def _run_step(self, step: RosStep, goal, cfg: StepConfig, run_timeout_s: float,
                  fb_fields: Optional[List[str]], fb_period_s: float) -> Tuple[str, Optional[int], object]:
        """Returns (outcome, terminal_status, result_msg)."""
        if not step.wait_for_server(self.args.server_timeout):
            self.say(f"ERROR: action server {step.name} not available after {self.args.server_timeout:.0f} s")
            return "no_server", None, None
        # The server has to discover THIS participant's endpoints too (that is what
        # a fresh CLI does not wait for); give discovery --settle s from node creation.
        time.sleep(max(0.0, self.args.settle - (time.monotonic() - self.t_created)))

        outcome = send_until_accepted(step.tracker, lambda: step.send(goal), cfg, self.say,
                                      interrupted=self.stop.is_set)
        if outcome != "accepted":
            return outcome, None, None
        with step.lock:
            gid, via = step.tracker.adopted, step.tracker.adopted_via
        self.say(f"{cfg.name}: goal accepted (id {gid.hex()[:8]}, confirmed by {via})")

        start = time.monotonic()
        last_print, last_phase, server_gone_since = 0.0, None, None
        while True:
            if self.stop.is_set():
                self.say(f"{cfg.name}: interrupted - cancelling the goal (the drone holds position)")
                if not self.args.no_cancel_on_interrupt:
                    step.cancel(gid)
                return "interrupted", None, None
            with step.lock:
                term = step.tracker.terminal_status()
                dups = step.tracker.duplicates_to_cancel()
                fb, _ = step.last_feedback, step.feedback_count
            for d in dups:
                self.say(f"{cfg.name}: a duplicate attempt (id {d.hex()[:8]}) was accepted too - cancelling it")
                step.cancel(d)
            if fb is not None:
                phase = getattr(fb, "phase", None)
                now = time.monotonic()
                if phase != last_phase or now - last_print >= fb_period_s:
                    self.say(f"{cfg.name}: {format_msg(fb, fb_fields)}")
                    last_print, last_phase = now, phase
            if term is not None:
                result = step.fetch_result(gid)
                return "done", term, result
            if not step.client.server_is_ready():
                server_gone_since = server_gone_since or time.monotonic()
                if time.monotonic() - server_gone_since > 30.0:
                    self.say(f"{cfg.name}: ERROR: action server disappeared for 30 s")
                    return "server_lost", None, None
            else:
                server_gone_since = None
            if time.monotonic() - start > run_timeout_s:
                self.say(f"{cfg.name}: ERROR: no result after {run_timeout_s:.0f} s - cancelling")
                step.cancel(gid)
                return "run_timeout", None, None
            time.sleep(0.1)

    def _report(self, name: str, outcome: str, term: Optional[int], result) -> bool:
        if outcome == "done":
            msg = format_msg(result) if result is not None else "(result unavailable)"
            self.say(f"{name}: finished {STATUS_NAMES.get(term, term)}: {msg}")
            ok = term == STATUS_SUCCEEDED and (result is None or bool(getattr(result, "success", True)))
            return ok
        explain = {
            "rejected": "the server rejected the goal (see its log in the launch tmux)",
            "busy": "the server is busy with a goal this sortie did not send",
            "timeout": f"no attempt was accepted ({self.args.retries} attempts)",
        }.get(outcome, outcome)
        self.say(f"{name}: FAILED - {explain}")
        return False

    # -------------------------------------------------------------- main flow
    def run(self) -> int:
        from mtl_msgs.action import SearchMission
        a = self.args
        cfg_kw = dict(accept_timeout_s=a.accept_timeout, retries=a.retries)

        # Create EVERY entity before the executor starts spinning, so discovery
        # of all of them starts now and overlaps the state-estimate wait.
        takeoff = None
        if not a.no_takeoff:
            from task_msgs.action import TakeoffTask
            takeoff = RosStep(self.node, TakeoffTask, f"{self.R}/tasks/takeoff", self.robot)
            self._subscribe_state_estimate()
        mission = RosStep(self.node, SearchMission, f"{self.R}/search_mission", self.robot)
        self.spin_thread.start()

        if takeoff is not None:
            self._wait_state_estimate(a.state_estimate_timeout)
            goal = TakeoffTask.Goal()
            goal.target_altitude_m = float(a.alt)
            goal.velocity_m_s = float(a.vel)
            self.say(f"takeoff to {a.alt:.1f} m at {a.vel:.1f} m/s")
            outcome, term, result = self._run_step(
                takeoff, goal, StepConfig("takeoff", **cfg_kw), a.takeoff_timeout,
                ["current_altitude_m", "target_altitude_m"], 5.0)
            if outcome == "interrupted":
                return EXIT_INT
            if outcome == "no_server":
                return EXIT_NO_SERVER
            if not self._report("takeoff", outcome, term, result):
                return EXIT_NOT_ACCEPTED if outcome in ("timeout", "busy") else EXIT_TAKEOFF

        goal = SearchMission.Goal()
        goal.start_mission = not a.dry_run
        goal.run_id = a.run_id
        goal.scenario_file = a.scenario_file
        self.say(f"search_mission: start_mission={goal.start_mission} run_id={a.run_id}")
        outcome, term, result = self._run_step(
            mission, goal, StepConfig("search_mission", **cfg_kw), a.mission_timeout,
            ["phase", "progress", "remaining_m", "cross_track_error_m", "current_position"], 5.0)
        if outcome == "interrupted":
            return EXIT_INT
        if outcome == "no_server":
            return EXIT_NO_SERVER
        if self._report("search_mission", outcome, term, result):
            self.say("sortie succeeded")
            return EXIT_OK
        if outcome in ("timeout", "busy", "rejected"):
            self.say("the drone is left hovering at its takeoff point")
            return EXIT_NOT_ACCEPTED
        return EXIT_SORTIE


def parse_args(argv: Optional[List[str]] = None):
    p = argparse.ArgumentParser(description=__doc__ or "MTL sortie client")
    p.add_argument("--robot", required=True)
    p.add_argument("--run-id", default=time.strftime("%Y%m%d-%H%M%S", time.gmtime()))
    p.add_argument("--alt", type=float, default=30.0, help="takeoff altitude [m] (layer offset already added)")
    p.add_argument("--vel", type=float, default=2.0)
    p.add_argument("--no-takeoff", action="store_true")
    p.add_argument("--dry-run", action="store_true", help="plan + publish only; implies --no-takeoff")
    p.add_argument("--scenario-file", default="")
    p.add_argument("--accept-timeout", type=float, default=10.0, help="per attempt [s]")
    p.add_argument("--retries", type=int, default=4)
    p.add_argument("--settle", type=float, default=3.0,
                   help="pause after the server becomes visible, before the first goal [s]")
    p.add_argument("--server-timeout", type=float, default=60.0)
    p.add_argument("--state-estimate-timeout", type=float, default=20.0)
    p.add_argument("--takeoff-timeout", type=float, default=180.0)
    p.add_argument("--mission-timeout", type=float, default=1800.0)
    p.add_argument("--no-cancel-on-interrupt", action="store_true")
    p.add_argument("--tag", default="mtl_sortie",
                   help="log prefix and node-name stem (e.g. tigris_sortie)")
    a = p.parse_args(argv)
    if not a.tag.replace("_", "").isalnum():
        p.error("--tag must be letters, digits and '_' (it is part of the node name)")
    if a.dry_run:
        a.no_takeoff = True
    if a.retries < 1 or a.accept_timeout <= 0:
        p.error("--retries must be >= 1 and --accept-timeout > 0")
    return a


def main(argv: Optional[List[str]] = None) -> int:
    global LOG_TAG
    args = parse_args(argv)
    LOG_TAG = args.tag
    sortie = Sortie(args)

    def on_signal(signum, _frame):
        log(args.robot, f"signal {signum} received")
        sortie.stop.set()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        return sortie.run()
    finally:
        sortie.shutdown()


if __name__ == "__main__":
    sys.exit(main())
