"""Regression for the size-dependent local handoff failure in mission 480."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.serialization import serialize_message
from coordination_bringup.gossip_node import GossipNode
from coordination_bringup.map_snapshot import write_snapshot
from test_semantic_delta import message


def test_actual_separate_planner_fuses_map_growth(monkeypatch, tmp_path):
    monkeypatch.setenv('RAVEN_SEMANTIC_MAP_SHARING', 'true')
    monkeypatch.setenv('GOSSIP_MEASURE_BYTES', 'true')
    monkeypatch.setenv('RAVEN_MAP_STREAM_PORT', '0')
    monkeypatch.setenv('RAVEN_MAP_SNAPSHOT_DIR', str(tmp_path))
    repo = next(p for p in Path(__file__).resolve().parents if (p/'airstack.sh').exists())
    config = repo/'common/ros_packages/coordination/coordination_bringup/config/gossip_payloads.yaml'
    progress = tmp_path/'planner.json'
    child = '''
import hashlib,json,os,sys
from pathlib import Path
import numpy as np
import rclpy
from raven_nav.raven_nav_node import RavenNavNode
os.environ['ROBOT_NAME']='robot_82'
labels=['label'+str(i) for i in range(40)]
rclpy.init(args=['--ros-args','-p','semantic_map_sharing:=true','-p','query_labels:='+json.dumps(labels)])
n=RavenNavNode();n._boot_enu=np.zeros(3)
def save():
 if n._vox_xyz is not None:
  arr=np.ascontiguousarray(np.c_[n._vox_xyz,n._vox_scores],dtype='<f4')
  p=Path(sys.argv[1]);t=p.with_suffix('.tmp')
  t.write_text(json.dumps(dict(cells=len(arr),hash=hashlib.sha256(arr.tobytes()).hexdigest(),updates=n._semantic_map_updates)))
  t.replace(p)
n.create_timer(.1,save)
rclpy.spin(n)
'''
    nodes, process = [], None
    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    log = (tmp_path/'planner.log').open('w')
    try:
        for name in ['robot_81','robot_82']:
            monkeypatch.setenv('ROBOT_NAME',name)
            node=GossipNode();node._boot_pos=(0.,0.,0.)
            node._setup_payload_subscriptions(str(config))
            nodes.append(node);executor.add_node(node)
        process=subprocess.Popen([sys.executable,'-c',child,str(progress)],stdout=log,stderr=log)
        for stage, size in enumerate([1000,100000],start=1):
            matrix=np.zeros((size,43),dtype='<f4');matrix[:,0]=np.arange(size)*.5;matrix[:,2]=5;matrix[:,3:]=.025
            msg=message(matrix,stage);msg.source_robot='robot_81'
            expected=hashlib.sha256(matrix.tobytes()).hexdigest()
            write_snapshot('robot_81',serialize_message(msg))
            deadline=time.monotonic()+25
            while time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.05)
                assert process.poll() is None, (tmp_path/'planner.log').read_text()[-3000:]
                if progress.exists():
                    data=json.loads(progress.read_text())
                    if data['cells']==size and data['hash']==expected:
                        assert data['updates']['robot_81']>=stage
                        break
            else:
                raise AssertionError(f'planner failed to fuse growing map: {size} cells; '+(tmp_path/'planner.log').read_text()[-1500:])
        # Allow the final fusion ACK to reach the stream worker before teardown.
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
    finally:
        for node in nodes:
            executor.remove_node(node);node.destroy_node()
        executor.shutdown();rclpy.shutdown()
        if process:
            process.terminate()
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired: process.kill();process.wait()
        log.close()
