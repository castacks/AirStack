import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from gamma_zero_msgs.msg import SelectedCommand

from gamma_zero_controller.control import Controller, Reference
from gamma_zero_controller.policy_worker import PolicyClient
from gamma_zero_controller.recording import Recorder


class GammaController(Node):
    def __init__(self):
        super().__init__('gamma_controller')
        config = json.loads(Path(self.declare_parameter('config', '').value).read_text())
        pattern = self.declare_parameter('trajectory_pattern', '').value
        if pattern:
            config.setdefault('trajectory', {})['pattern'] = pattern
        self.simulation = self.declare_parameter('simulation', True).value
        if not np.isfinite(config.get('fault_hold_s', 30.)) or config.get('fault_hold_s', 30.) <= 0:
            raise ValueError('Invalid fault hold timeout')
        self.robot = self.declare_parameter('robot_name', 'drone_3').value
        if not self.simulation and not all(config.get(k, False) for k in ['transforms_verified', 'thrust_verified', 'firmware_verified']):
            raise ValueError('Live mode requires verified transforms, thrust and firmware')
        assets = self.declare_parameter('assets', '').value
        socket = self.declare_parameter('policy_socket', '').value
        self.policy = PolicyClient(assets, socket, timeout_s=.02)
        self.policy.act({'linear_velocity_flu': np.zeros(3), 'angular_velocity_flu': np.zeros(3),
                         'projected_gravity_flu': np.array([0., 0., -1.])}, np.zeros(4), np.zeros(4))
        self.policy.reset_history()
        self.control = Controller(self.policy, config)
        self.dt = float(self.policy.assets['control_dt'])
        if abs(self.dt-.02) > 1e-9:
            raise ValueError('This backend requires the evaluated 50 Hz policy')
        self.svg = {}
        self.svg_received = -np.inf
        self.reference = None
        self.reference_received = -np.inf
        self.reference_stamp = -np.inf
        self.svg_command = None
        self.ack = {}
        self.ack_received = -np.inf
        self.pending_since = None
        self.fault_hold_started = None
        self.selected = self.create_publisher(SelectedCommand, 'selected_command', 1)
        self.status = self.create_publisher(String, 'status', 10)
        self.trace = self.create_publisher(String, 'trace', qos_profile_sensor_data)
        self.create_subscription(VehicleOdometry, 'vehicle_odometry', self.state_callback, qos_profile_sensor_data)
        self.create_subscription(MultiDOFJointTrajectory, 'svg_reference', self.reference_callback, 1)
        self.create_subscription(String, 'svg_status', self.svg_callback, 1)
        self.create_subscription(String, 'selected_command_status', self.ack_callback, 1)
        self.create_service(Trigger, '~/engage', self.engage)
        self.create_service(Trigger, '~/handback', self.handback)
        self.create_service(Trigger, '~/start_trajectory', self.start_trajectory)
        self.create_service(Trigger, '~/use_svg', self.use_svg)
        self.create_service(Trigger, '~/hold', self.hold)
        self.log = Recorder(self.declare_parameter('log', '').value)
        self.record({'event': 'configuration', 'config': config, 'bundle_sha256': self.policy.assets['bundle_sha256']})
        self.timer = self.create_timer(self.dt, self.tick)
        self.status_timer = self.create_timer(.1, self.publish_status)


    def record(self, record):
        record = dict(record, ros_s=self.get_clock().now().nanoseconds*1e-9)
        self.log.append(record)
        self.trace.publish(String(data=json.dumps(record, allow_nan=False)))


    def state_callback(self, msg):
        before = self.control.session.reset_counter
        try:
            self.control.session.update_state(msg.q, msg.position, msg.velocity, msg.angular_velocity,
                msg.timestamp_sample*1e-6, time.monotonic(), msg.reset_counter, msg.pose_frame, msg.velocity_frame)
            if before is not None and before != msg.reset_counter:
                raise ValueError('Estimator reset; manual restart required')
        except ValueError as exc:
            self.trip(str(exc), allow_hold=False)


    def reference_callback(self, msg):
        try:
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            now = self.get_clock().now().nanoseconds*1e-9
            if not -.02 <= now-stamp <= .1 or stamp <= self.reference_stamp or msg.header.frame_id != 'map':
                raise ValueError('Invalid reference time/frame')
            if len(msg.points) != 1 or len(msg.points[0].transforms) != 1 or len(msg.points[0].velocities) != 1:
                raise ValueError('Expected one position/velocity reference')
            point = msg.points[0]
            t, v = point.transforms[0], point.velocities[0]
            q = np.array([t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z])
            rate_mode = np.all(q == 0)
            if not rate_mode and abs(np.linalg.norm(q)-1) > .01:
                raise ValueError('Invalid heading quaternion')
            yaw = np.arctan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
            ref = Reference(np.array([t.translation.x, t.translation.y, t.translation.z]),
                            np.array([v.linear.x, v.linear.y, v.linear.z]), float(yaw), v.angular.z, rate_mode)
            if not np.isfinite(np.r_[ref.position, ref.velocity, yaw, ref.yaw_rate]).all():
                raise ValueError('Nonfinite SVG reference')
            self.reference, self.svg_command = ref, msg
            self.reference_stamp, self.reference_received = stamp, time.monotonic()
        except ValueError as exc:
            self.trip(str(exc))


    def svg_callback(self, msg):
        try:
            self.svg = json.loads(msg.data)
            self.svg_received = time.monotonic()
        except ValueError:
            self.trip('Invalid SVG status')


    def ack_callback(self, msg):
        try:
            self.ack = json.loads(msg.data)
            self.ack_received = time.monotonic()
        except ValueError:
            self.trip('Invalid interface acknowledgement', allow_hold=False)


    def engage(self, request, response):
        try:
            now = time.monotonic()
            matching = [d for d in self.svg.get('drones', []) if d.get('name') == self.robot]
            airborne = len(matching) == 1 and matching[0]['state'] == 'ACTIVE' and not self.svg.get('mission_active', True)
            airborne = airborne and now-self.svg_received < .5 and now-self.reference_received < .1
            if self.control.fault:
                raise ValueError('Clear fault using use_svg in conventional mode before engaging')
            if self.pending_since is not None or now-self.ack_received > .1 or self.ack.get('mode') != 0:
                raise ValueError('Confirmed conventional command ownership required')
            self.control.engage(self.get_clock().now().nanoseconds*1e-9, now, airborne)
            self.pending_since = now
            self.record({'event': 'engage_requested'})
            response.success, response.message = True, 'Engagement queued; confirm actual owner in status'
        except (ValueError, RuntimeError, OSError) as exc:
            response.success, response.message = False, str(exc)
        return response


    def handback(self, request, response):
        try:
            self.control.handback(self.get_clock().now().nanoseconds*1e-9, time.monotonic())
            self.pending_since = time.monotonic()
            response.success, response.message = True, 'Handback queued; wait for matching interface acknowledgement'
        except ValueError as exc:
            self.trip(str(exc), allow_hold=False)
            response.success, response.message = False, str(exc)
        return response


    def start_trajectory(self, request, response):
        try:
            self.control.start_trajectory(self.get_clock().now().nanoseconds*1e-9, time.monotonic())
            self.record({'event': 'trajectory_start', 'pattern': self.control.session.trajectory.config.pattern,
                         'origin': self.control.session.trajectory.origin.tolist(), 'yaw': self.control.session.trajectory.yaw})
            response.success, response.message = True, 'Started tested loco_mjx reference'
        except (ValueError, RuntimeError, OSError) as exc:
            response.success, response.message = False, str(exc)
        return response


    def use_svg(self, request, response):
        now = time.monotonic()
        if self.control.mode == 'fault' or now-self.reference_received > .1 or (self.control.mode == 'ctbr' and self.control.fault):
            response.success, response.message = False, 'Fresh conventional reference required; relaunch after state fault'
        else:
            self.control.source, self.control.fault = 'svg', ''
            self.control.hold = None
            self.fault_hold_started = None
            response.success, response.message = True, 'Conventional SVG reference selected; policy remains disengaged'
        return response


    def hold(self, request, response):
        try:
            mode = self.control.mode
            self.control.handback(self.get_clock().now().nanoseconds*1e-9, time.monotonic(), '')
            self.control.mode = mode
            self.control.session.active = mode == 'ctbr'
            self.control.session.last_tick_s = time.monotonic()
            response.success, response.message = True, 'Captured hold on selected backend'
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        return response


    def trip(self, reason, allow_hold=True):
        try:
            if self.control.mode == 'ctbr' and allow_hold:
                self.control.handback(self.get_clock().now().nanoseconds*1e-9, time.monotonic(), reason)
                self.pending_since = time.monotonic()
                self.fault_hold_started = time.monotonic()
            else:
                self.control.mode, self.control.fault = 'fault', reason
        except ValueError:
            self.control.mode, self.control.fault = 'fault', reason
        self.get_logger().error(reason)
        try:
            self.record({'event': 'fault', 'reason': reason, 'mode': self.control.mode})
        except OSError:
            pass


    def tick(self):
        started = time.monotonic()
        ros_s = self.get_clock().now().nanoseconds*1e-9
        c = self.control
        try:
            if c.mode == 'fault':
                return
            if self.fault_hold_started is not None and started-self.fault_hold_started > c.config.get('fault_hold_s', 30.):
                self.trip('Fault hold expired', allow_hold=False)
                return
            owners = self.count_publishers(self.selected.topic_name)
            if owners == 0:
                return
            if owners > 1:
                raise ValueError('Conflicting selected-command publisher')
            if c.session.state is None or not -.02 <= ros_s-c.session.source_s <= .1 or started-c.session.received_s > .1:
                if c.mode == 'ctbr':
                    self.trip('State stale', allow_hold=False)
                return
            if c.mode == 'conventional':
                try:
                    c.update_stability(ros_s, started)
                except ValueError:
                    c.stable_since = None
            idle = any(d.get('name') == self.robot and d.get('state') == 'IDLE' for d in self.svg.get('drones', []))
            if c.mode == 'conventional' and c.source == 'svg' and idle and started-self.svg_received < .5:
                self.ack = {}
                self.ack_received = -np.inf
                self.pending_since = None
                return
            if self.pending_since is not None:
                matched = self.ack.get('generation') == c.generation and self.ack.get('mode') == (1 if c.mode == 'ctbr' else 0)
                if matched and started-self.ack_received < .1:
                    self.pending_since = None
                elif started-self.pending_since > .2:
                    self.trip('Interface handoff acknowledgement expired', allow_hold=False)
                    return
            elif self.ack and started-self.ack_received > .2:
                self.trip('Interface acknowledgement stale', allow_hold=False)
                return
            selected = SelectedCommand()
            selected.header.stamp = self.get_clock().now().to_msg()
            selected.header.frame_id = 'map'
            selected.generation = c.generation
            reference = c.hold if c.source == 'hold' else self.reference
            if c.source == 'svg' and (started-self.reference_received > .1 or started-self.svg_received > .5):
                if c.mode == 'ctbr':
                    raise ValueError('SVG reference/status stale')
                return
            if c.mode == 'ctbr':
                if started-self.svg_received > .5:
                    raise ValueError('SVG lifecycle status stale')
                mapped = c.command(reference, ros_s, started)
                selected.mode = 1
                selected.rates.header = selected.header
                selected.rates.header.frame_id = self.robot+'/base_link'
                selected.rates.thrust.z = mapped['normalized_collective']
                selected.rates.angular_rates.x, selected.rates.angular_rates.y, selected.rates.angular_rates.z = mapped['rates_flu']
                c.inference_s = time.monotonic()-started
                if c.inference_s > self.dt:
                    raise RuntimeError('Inference deadline exceeded')
                c.session.check_state(self.get_clock().now().nanoseconds*1e-9, time.monotonic())
            else:
                selected.mode = 0
                if c.source == 'svg':
                    if self.svg_command is None:
                        return
                    selected.trajectory = self.svg_command
                else:
                    if reference is None:
                        return
                    trajectory = MultiDOFJointTrajectory()
                    trajectory.header = selected.header
                    point = MultiDOFJointTrajectoryPoint()
                    pose, velocity = Transform(), Twist()
                    pose.translation.x, pose.translation.y, pose.translation.z = reference.position.tolist()
                    pose.rotation.z, pose.rotation.w = float(np.sin(reference.yaw/2)), float(np.cos(reference.yaw/2))
                    point.transforms, point.velocities = [pose], [velocity]
                    trajectory.points = [point]
                    selected.trajectory = trajectory
            self.record(dict(c.last_record if selected.mode else {}, event='selected', mode=selected.mode, generation=c.generation, inference_s=c.inference_s, source_s=c.session.source_s))
            if time.monotonic()-started > self.dt:
                raise RuntimeError('Command preparation deadline exceeded')
            if not self.ack and self.pending_since is None:
                self.pending_since = started
            self.selected.publish(selected)
        except (ValueError, RuntimeError, OSError) as exc:
            self.trip(str(exc))


    def publish_status(self):
        c = self.control
        status = {'mode': c.mode, 'source': c.source, 'trajectory_pattern': c.session.trajectory.config.pattern, 'fault': c.fault, 'generation': c.generation,
                  'ack': self.ack, 'ack_fresh': time.monotonic()-self.ack_received < .1,
                  'handoff_pending': self.pending_since is not None, 'inference_s': c.inference_s,
                  'ready_for_engage': c.mode == 'conventional' and not c.fault and c.stable_since is not None and time.monotonic()-c.stable_since >= 1. and time.monotonic()-c.session.received_s <= .1 and time.monotonic()-self.reference_received <= .1 and time.monotonic()-self.svg_received <= .5 and self.pending_since is None and self.ack.get('mode') == 0 and time.monotonic()-self.ack_received <= .1,
                  'state_age_s': time.monotonic()-c.session.received_s if np.isfinite(c.session.received_s) else None,
                  'reference_age_s': time.monotonic()-self.reference_received if np.isfinite(self.reference_received) else None,
                  'saturated': c.last_record.get('saturated', False),
                  'bundle_sha256': self.policy.assets['bundle_sha256'], 'simulation': self.simulation}
        self.status.publish(String(data=json.dumps(status)))


def main():
    rclpy.init()
    node = GammaController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.log.close()
        node.destroy_node()
        rclpy.try_shutdown()
