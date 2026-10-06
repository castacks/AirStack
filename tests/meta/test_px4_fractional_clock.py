"""Execute the actual Pegasus backend methods without Isaac or a MAVLink socket."""
import ast
from fractions import Fraction
from pathlib import Path
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

SOURCE = Path(__file__).parents[2] / 'simulation/isaac-sim/extensions/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/logic/backends/px4_mavlink_backend.py'


def load_backend():
    # Keep method bodies intact, replacing only external imports/dependencies.
    tree = ast.parse(SOURCE.read_text())
    klass = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'PX4MavlinkBackend')
    class Base:
        def __init__(self, config): pass
    config = NS(vehicle_id=0, connection_type='tcpin', connection_ip='localhost',
        connection_baseport=4560, px4_autolaunch=False, px4_vehicle_model='test',
        px4_dir='/unused', update_rate=100, num_rotors=4, input_offset=[0]*4,
        input_scaling=[1]*4, zero_position_armed=[0]*4, enable_lockstep=False)
    namespace = dict(Backend=Base, PX4MavlinkBackendConfig=lambda:config,
        SensorMsg=lambda:NS(received_first_imu=False, new_imu_data=False),
        ThrusterControl=lambda *args:NS(zero_input_reference=Mock()),
        np=NS(zeros=lambda shape:[0]*shape[0], ndarray=list), time=time,
        PX4LaunchTool=object, State=object, carb=NS(log_info=Mock(),log_warn=Mock()),
        mavutil=NS(mavlink=NS(MAV_TYPE_GENERIC=0),
            mavlink_connection=lambda port:NS(close=Mock(),wait_heartbeat=lambda **kw:None)))
    exec(compile(ast.Module(body=[klass], type_ignores=[]), str(SOURCE), 'exec'), namespace)
    return namespace['PX4MavlinkBackend']


@unittest.skipUnless(SOURCE.exists(), 'Pegasus submodule not installed')
class FractionalClockTests(unittest.TestCase):
    def backend(self):
        b=load_backend()()
        b._is_running=True;b._connection=NS(close=Mock())
        b._received_first_hearbeat=True;b._last_heartbeat_sent_time=time.time()
        b.poll_mavlink_messages=Mock();b.send_heartbeat=Mock()
        b.send_sensor_msgs=Mock();b.send_gps_msgs=Mock()
        return b

    def assert_duration(self,b,seconds):
        exact_us=seconds*1_000_000
        self.assertLess(abs(b._current_utime-exact_us),1.0)
        self.assertGreaterEqual(b._utime_remainder_us,0)
        self.assertLess(b._utime_remainder_us,1)
        self.assertAlmostEqual(b._current_utime+b._utime_remainder_us,float(exact_us),places=5)

    def test_float32_physics_duration_does_not_lose_per_step_fraction(self):
        b=self.backend();dt=.009999999776482582
        for _ in range(10000):b.update(dt)
        self.assert_duration(b,Fraction.from_float(dt)*10000)
        self.assertEqual(b._current_utime,99999997)
        self.assertEqual(b.send_sensor_msgs.call_args.args,(b._current_utime,))
        self.assertEqual(b.send_gps_msgs.call_args.args,(b._current_utime,))

    def test_variable_and_submicrosecond_duration(self):
        b=self.backend();steps=[.009999999776482582,1/120,.0000004,.0000007,.003333333]*300
        for dt in steps:b.update(dt)
        self.assert_duration(b,sum(map(Fraction.from_float,steps)))
        c=self.backend()
        for _ in range(10):c.update(.0000004)
        self.assert_duration(c,Fraction.from_float(.0000004)*10)

    def test_integer_microsecond_steps_unchanged_and_instances_isolated(self):
        a,b=self.backend(),self.backend()
        for _ in range(100):a.update(.01)
        self.assertEqual(a._current_utime,1000000);self.assertEqual(a._utime_remainder_us,0)
        a.update(.0000004)
        self.assertEqual(b._current_utime,0);self.assertEqual(b._utime_remainder_us,0)

    def test_all_existing_early_gates_leave_counter_and_remainder_unchanged(self):
        for gate in ('stopped','connection','heartbeat','imu'):
            with self.subTest(gate=gate):
                b=self.backend();b.update(.0000004);before=(b._current_utime,b._utime_remainder_us)
                b.wait_for_first_hearbeat=Mock()
                if gate=='stopped':b._is_running=False
                elif gate=='connection':b._connection=None
                elif gate=='heartbeat':b._received_first_hearbeat=False
                else:b._sensor_data=NS(received_first_imu=True,new_imu_data=False)
                b.update(.01)
                self.assertEqual((b._current_utime,b._utime_remainder_us),before)
                self.assertEqual(b.send_sensor_msgs.call_count,1)
                self.assertEqual(b.send_gps_msgs.call_count,1)

    def test_stop_start_reinitialize_and_noop_reset_preserve_counter_epoch(self):
        b=self.backend();b.update(.0100004);before=(b._current_utime,b._utime_remainder_us)
        b.reset();self.assertEqual((b._current_utime,b._utime_remainder_us),before)
        b.stop();b.update(.1);self.assertEqual((b._current_utime,b._utime_remainder_us),before)
        b.start();self.assertEqual((b._current_utime,b._utime_remainder_us),before)
        self.assertFalse(b._received_first_hearbeat)
        b.update(.1);self.assertEqual((b._current_utime,b._utime_remainder_us),before)
        b._received_first_hearbeat=True;b.update(.0000007)
        self.assert_duration(b,Fraction.from_float(.0100004)+Fraction.from_float(.0000007))

    def test_new_instance_resets_both_parts_of_epoch(self):
        a=self.backend();a.update(.01);a.update(.0000004)
        b=self.backend();self.assertEqual((b._current_utime,b._utime_remainder_us),(0,0.0))


if __name__=='__main__':unittest.main()
