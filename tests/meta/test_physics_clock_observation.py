import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
spec=importlib.util.spec_from_file_location('physical_clock',Path(__file__).parents[2]/'simulation/isaac-sim/launch_scripts/physical_truth.py')
p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)

class World:
    def __init__(self):self.callbacks={};self.removed=[]
    def add_physics_callback(self,name,callback):self.callbacks[name]=callback
    def remove_physics_callback(self,name):self.removed.append(name);del self.callbacks[name]

class Backend:
    _current_utime=123
    _is_running=True
    _received_first_hearbeat=True
    _sensor_data=NS(received_first_imu=True,new_imu_data=False)

class ClockTests(unittest.TestCase):
    def test_actual_dt_totals_and_truncation(self):
        w=World();o=p.ClockObservation();o.bind(w)
        for _ in range(1000):w.callbacks[o.callback_name](.009999999776482582)
        s=o.snapshot();self.assertEqual(s['callback_count'],1000)
        self.assertEqual(s['sum_truncated_callback_dt_us'],9999000)
        self.assertAlmostEqual(s['sum_callback_dt_s'],9.999999776482582)
        self.assertEqual(s['min_callback_dt_s'],s['max_callback_dt_s'])
    def test_world_replacement_resets_and_old_callback_cannot_contaminate(self):
        a,b=World(),World();o=p.ClockObservation();o.bind(a);old=a.callbacks[o.callback_name]
        old(.01);identity=o.observer_id;o.bind(a);self.assertEqual(o.count,1)
        o.bind(b);self.assertNotEqual(o.observer_id,identity);self.assertEqual(o.count,0)
        self.assertEqual(a.removed,[o.callback_name]);old(.01);self.assertEqual(o.count,0)
        b.callbacks[o.callback_name](.02);self.assertEqual(o.count,1)
    def test_failed_removal_old_callback_stays_inert_after_replacement(self):
        class Stale(World):
            def remove_physics_callback(self,name):raise RuntimeError('old world invalid')
        a,b=Stale(),World();o=p.ClockObservation(warn=lambda m:None);o.bind(a)
        old=a.callbacks[o.callback_name];old(.01);o.bind(b)
        old(.03);self.assertEqual(o.count,0)
        b.callbacks[o.callback_name](.02);self.assertEqual(o.count,1)
        self.assertEqual(o.sum_dt_s,.02)
    def test_invalid_dt_is_contained_and_evidence_invalid(self):
        errors=[];o=p.ClockObservation(warn=errors.append)
        for v in [float('nan'),0,-1,True]:o.on_step(v)
        self.assertEqual(o.count,0);self.assertTrue(o.snapshot()['observation_error'])
        self.assertEqual(len(errors),1)
    def test_backend_snapshot_is_read_only_and_hashed(self):
        b=Backend();r=p.backend_clocks(NS(_backends=[object(),b]),{})
        self.assertEqual(r[0]['backend_instance_id'],hex(id(b)))
        self.assertEqual(r[0]['current_utime_us'],123);self.assertEqual(b._current_utime,123)
        self.assertEqual(len(r[0]['source_sha256']),64);self.assertFalse(r[0]['new_imu_data'])
        self.assertEqual(p.backend_clocks(NS(),{}),[])
    def test_attach_and_remove_failure_do_not_escape(self):
        class Bad(World):
            def add_physics_callback(self,*a):raise RuntimeError('not initialized')
        o=p.ClockObservation(warn=lambda m:None);o.bind(Bad())
        self.assertFalse(o.attached);self.assertEqual(o.error,'not initialized')
        w=World();o.bind(w);w.callbacks.clear();o.close();self.assertFalse(o.attached)
        self.assertIsNotNone(o.error)
if __name__=='__main__':unittest.main()
