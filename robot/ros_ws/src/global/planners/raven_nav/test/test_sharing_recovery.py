"""Regression checks for zero-stamped rays and wide-map batch materialization."""
import json,time
from types import SimpleNamespace as S
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2,PointField
from coordination_msgs.msg import SemanticVoxelMap
import raven_nav.raven_nav_node as module
from raven_nav.raven_nav_node import RavenNavNode
from raven_nav.semantic_map import SemanticMapFusion


def make_node(features, queries):
    labels=['rgb_r','rgb_g','rgb_b','observation_count']+[f'f{i}' for i in range(features.shape[1])]
    n=len(features);xyz=np.c_[np.arange(n)*.5,np.zeros(n),np.full(n,10)].astype(np.float32)
    values=np.c_[np.zeros((n,3)),np.ones(n),features].astype(np.float32)
    sent=[];node=S(_boot_enu=np.zeros(3),_robot_name='robot_1',_query_labels=[f'q{i}' for i in range(len(queries))],
        _semantic_feature_labels=labels,_reusable_feature_map=True,_feature_queries=queries,
        _feature_probability=True,_semantic_fusion=SemanticMapFusion(),_own_semantic_snapshot=(xyz,values,1),
        _semantic_map_received_files={},_semantic_map_received_at={},_semantic_map_updates={},_semantic_map_rejections=0,
        get_logger=lambda:S(error=lambda *args,**kw:None))
    node.refreshes=0
    def refresh():
        node.refreshes+=1
        RavenNavNode._refresh_semantic_map(node)
    node._refresh_semantic_map=refresh
    node._on_stream_semantic_map=lambda msg,**kw:RavenNavNode._on_stream_semantic_map(node,msg,**kw)
    node._semantic_map_ack_pub=S(publish=lambda msg:sent.append((json.loads(msg.data),node.refreshes)))
    packets=[]
    for i in [2,3]:
        m=SemanticVoxelMap();m.source_robot=f'robot_{i}';m.labels=labels;m.voxel_size=.5
        matrix=np.c_[xyz+[0,(i-1)*.5,0],values].astype('<f4')
        names=['x','y','z']+[f'sim_{j}' for j in range(len(labels))]
        m.cloud=PointCloud2(header=Header(stamp=Time(sec=i),frame_id='world_xy_agl'),height=1,width=n,
            fields=[PointField(name=name,offset=4*j,datatype=7,count=1) for j,name in enumerate(names)],
            point_step=4*len(names),row_step=4*len(names)*n,data=matrix.tobytes(),is_dense=True)
        packets.append((m.source_robot,serialize_message(m),i))
    return node,sent,packets


def test_batch_fusion_exact_and_ack_after_materialization(monkeypatch):
    rng=np.random.default_rng(7);features=rng.normal(size=(1100,1152)).astype(np.float32)
    queries=rng.normal(size=(7,1152)).astype(np.float32)
    results=[]
    for batch in [False,True]:
        node,acks,packets=make_node(features,queries)
        monkeypatch.setenv('RAVEN_MAP_BATCH_FUSION',str(batch).lower())
        monkeypatch.setattr(module,'received_snapshots',lambda *a:iter(packets))
        RavenNavNode._poll_stream_semantic_maps(node)
        assert node._semantic_map_rejections==0
        assert set(node._semantic_fusion.sources)=={'robot_1','robot_2','robot_3'}
        assert node._semantic_map_updates=={'robot_2':1,'robot_3':1}
        assert len(acks)==2 and all(r>=1 for _,r in acks)
        assert node.refreshes==(1 if batch else 2)
        results.append((node._semantic_fusion.arrays(),node._vox_scores))
    for a,b in zip(results[0][0],results[1][0]):np.testing.assert_array_equal(a.view(np.uint32),b.view(np.uint32))
    np.testing.assert_array_equal(results[0][1],results[1][1])


def test_batch_failure_never_acknowledges_unfused_maps(monkeypatch):
    node,acks,packets=make_node(np.ones((4,2),np.float32),np.ones((2,2),np.float32))
    monkeypatch.setenv('RAVEN_MAP_BATCH_FUSION','true')
    monkeypatch.setattr(module,'received_snapshots',lambda *a:iter(packets))
    node._refresh_semantic_map=lambda:(_ for _ in ()).throw(ValueError('bad fusion'))
    RavenNavNode._poll_stream_semantic_maps(node)
    assert not acks and node._semantic_map_rejections==1


@pytest.mark.parametrize('raw,source_sec,expected_sec',[(True,0,87),(True,42,42),(False,0,0)])
def test_zero_header_raw_rays_stamp_actual_input_once(raw,source_sec,expected_sec):
    sent=[];node=S(_share_raw_rays=raw,_vlfm_baseline=False,_target_objects=['car'],_query_labels=['car','wall'],
        _score_threshold=.65,_shared_rays_pub=S(publish=sent.append),
        get_clock=lambda:S(now=lambda:S(to_msg=lambda:Time(sec=87))))
    origins=np.array([[0,0,10]],np.float32);scores=np.array([[.9,.1]],np.float32)
    stamp=Time(sec=source_sec)
    RavenNavNode._publish_shared_rays(node,origins,np.array([[1,0,0]],np.float32),scores,['sim_0','sim_1'],stamp)
    assert sent[0].header.stamp.sec==expected_sec
    assert stamp.sec==source_sec
    assert sent[0].width==1
