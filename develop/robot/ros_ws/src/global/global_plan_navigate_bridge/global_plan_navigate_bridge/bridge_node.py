# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""global_plan (nav_msgs/Path) -> NavigateTask goal adapter.

The trunk local planner (droan_gl) is a task executor: it only plans while a
NavigateTask goal is active and it REJECTS a new goal while one is running.
The random-walk planner drives it as an action client itself, but planners
that just publish a path on the ``global_plan`` topic (the exploration_planner
and raven modules, any external planner following the interface conventions)
need this adapter:

    global planner --global_plan (Path)--> bridge --NavigateTask--> droan_gl

Behaviour:
  * a new plan whose pose count or final pose changed (``replan_min_change_m``)
    cancels the active goal and, once the cancel is acknowledged, sends the new
    plan as a goal (with retries: droan_gl briefly still reports "active");
  * identical re-publications are ignored;
  * an empty Path cancels the active goal (``cancel_on_empty_plan``);
  * ``~/set_enabled`` (std_srvs/SetBool) pauses/resumes; disabling cancels.
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Path
from std_srvs.srv import SetBool
from task_msgs.action import NavigateTask

from global_plan_navigate_bridge.plan_key import plan_changed, plan_key


class GlobalPlanNavigateBridge(Node):
    def __init__(self):
        super().__init__('global_plan_navigate_bridge')
        self.declare_parameter('goal_tolerance_m', 1.0)
        self.declare_parameter('replan_min_change_m', 0.5)
        self.declare_parameter('cancel_on_empty_plan', True)
        self.declare_parameter('send_retry_period_s', 0.5)
        self.declare_parameter('max_send_retries', 10)
        self.declare_parameter('enabled', True)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._client = ActionClient(self, NavigateTask, 'navigate_task')
        self._sub = self.create_subscription(Path, 'global_plan', self._on_plan, 10)
        self._srv = self.create_service(SetBool, '~/set_enabled', self._on_set_enabled)

        self._active_handle = None       # ClientGoalHandle of the running goal
        self._active_key = None          # plan_key of the running/sending goal
        self._pending = None             # (Path, key) waiting to be sent
        self._sending = False
        self._retries = 0
        self._retry_timer = None
        self.get_logger().info(
            'global_plan -> NavigateTask bridge ready '
            f'(enabled={self._enabled}, goal_tolerance_m='
            f'{self.get_parameter("goal_tolerance_m").value})')

    # ------------------------------------------------------------- inputs
    def _on_set_enabled(self, req, resp):
        self._enabled = bool(req.data)
        if not self._enabled:
            self._pending = None
            self._cancel_active()
        resp.success = True
        resp.message = 'enabled' if self._enabled else 'disabled (active goal cancelled)'
        self.get_logger().info(f'set_enabled -> {resp.message}')
        return resp

    def _on_plan(self, msg: Path):
        if not self._enabled:
            return
        pts = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        key = plan_key(pts)
        min_change = float(self.get_parameter('replan_min_change_m').value)
        current = self._pending[1] if self._pending else self._active_key
        if not plan_changed(current, key, min_change):
            return
        if key is None:
            if bool(self.get_parameter('cancel_on_empty_plan').value):
                self.get_logger().info('empty global_plan -> cancelling active NavigateTask')
                self._pending = None
                self._cancel_active()
            return
        self.get_logger().info(
            f'new global_plan ({key[0]} poses, goal {tuple(round(v, 2) for v in key[1])}) '
            '-> replacing NavigateTask goal')
        self._pending = (msg, key)
        self._dispatch()

    # ------------------------------------------------------------ dispatch
    def _dispatch(self):
        if self._pending is None or self._sending:
            return
        if self._active_handle is not None:
            self._cancel_active()          # _send runs from the cancel callback
            return
        self._send()

    def _cancel_active(self):
        handle = self._active_handle
        if handle is None:
            return
        self._active_handle = None
        self._active_key = None
        fut = handle.cancel_goal_async()
        fut.add_done_callback(lambda _f: self._dispatch())

    def _send(self):
        if self._pending is None:
            return
        if not self._client.server_is_ready():
            self.get_logger().warn('NavigateTask server not available yet; retrying', throttle_duration_sec=5.0)
            self._schedule_retry()
            return
        path, key = self._pending
        goal = NavigateTask.Goal()
        goal.global_plan = path
        goal.goal_tolerance_m = float(self.get_parameter('goal_tolerance_m').value)
        self._sending = True
        self._active_key = key
        fut = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_goal_response)

    def _schedule_retry(self):
        max_retries = int(self.get_parameter('max_send_retries').value)
        if self._retries >= max_retries:
            self.get_logger().error(f'NavigateTask goal rejected {self._retries} times; dropping plan')
            self._pending = None
            self._retries = 0
            self._active_key = None
            return
        self._retries += 1
        period = float(self.get_parameter('send_retry_period_s').value)
        if self._retry_timer is not None:
            self._retry_timer.cancel()
        self._retry_timer = self.create_timer(period, self._on_retry)

    def _on_retry(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None
        self._send()

    def _on_goal_response(self, fut):
        self._sending = False
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('NavigateTask goal rejected (local planner still busy?) - retrying')
            self._schedule_retry()
            return
        self._retries = 0
        self._active_handle = handle
        self._pending = None
        handle.get_result_async().add_done_callback(
            lambda f, h=handle: self._on_result(f, h))
        self.get_logger().info('NavigateTask goal accepted')

    def _on_feedback(self, fb):
        self.get_logger().debug(
            f'navigate: {fb.feedback.status} dist={fb.feedback.distance_to_goal:.2f} m')

    def _on_result(self, fut, handle):
        try:
            result = fut.result().result
            msg = f'success={result.success} "{result.message}"'
        except Exception as exc:  # noqa: BLE001 - report whatever the server returned
            msg = f'error: {exc}'
        self.get_logger().info(f'NavigateTask finished: {msg}')
        if self._active_handle is handle:
            self._active_handle = None
            self._active_key = None
        self._dispatch()


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanNavigateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
