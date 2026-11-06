import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
import pytest
from raven_nav.feature_map import query_features, export_feature_snapshot, feature_snapshot_path
from raven_nav.semantic_map import SemanticMapFusion


def test_arbitrary_query_not_present_when_map_was_created():
    # All feature channels survive fusion; a new query requires no new map.
    f = SemanticMapFusion()
    labels = ['feature:model:0', 'feature:model:1', 'feature:model:2']
    f.update('r1', [[0,0,5]], [[0,0,1]], labels, 1)
    f.update('r2', [[5,0,5]], [[0,1,0]], labels, 1)
    xyz, values = f.arrays()
    np.testing.assert_allclose(query_features(values, [[0,0,1]], False).ravel(), [1,0])
    np.testing.assert_allclose(query_features(values, [[0,1,0]], False).ravel(), [0,1])


def test_full_features_scoring_matches_torch_for_unseen_queries():
    torch = pytest.importorskip('torch')
    from rayfronts.utils import compute_cos_sim
    rng = np.random.default_rng(4)
    features = rng.normal(size=(57,1152)).astype('f4')
    queries = rng.normal(size=(7,1152)).astype('f4')
    for probability in [False,True]:
        expected = compute_cos_sim(torch.from_numpy(queries),torch.from_numpy(features),probability).numpy()
        np.testing.assert_allclose(query_features(features,queries,probability,13),expected,atol=5e-6)


def test_export_decompresses_independent_pca_bases_before_fusing(tmp_path,monkeypatch):
    torch = pytest.importorskip('torch')
    from omegaconf import OmegaConf
    monkeypatch.setenv('RAVEN_MAP_SNAPSHOT_DIR',str(tmp_path))
    monkeypatch.setenv('ROBOT_NAME','robot_1')
    # Two different coefficient vectors describe the same canonical feature.
    native = torch.tensor([[1.,2.]])
    basis = torch.tensor([[0.,1.,0.],[1.,0.,0.]])
    mapper = S(global_vox_xyz=torch.tensor([[0.,0.,8.]]),global_vox_feat=native,
               global_vox_rgb=torch.tensor([[1.,.5,0.]]),global_vox_cnt=torch.ones(1,1),
               feat_compressor=S(decompress=lambda f:f@basis),
               encoder=S(align_spatial_features_with_language=lambda f:f))
    q = torch.tensor([[1.,0.,0.],[0.,1.,0.]])
    cfg = OmegaConf.create({'encoder':{'model':'test'},'querying':{'compressed':False,'compute_prob':True}})
    server = S(mapper=mapper,cfg=cfg,_queries_labels={'text':['new object','background']},_queries_feats={'text':q})
    expected = query_features((native@basis).numpy(),q.numpy())
    export_feature_snapshot(server,{'scores':expected})
    with np.load(feature_snapshot_path('robot_1')) as data:
        np.testing.assert_array_equal(data['features'],[[2.,1.,0.]])
        assert data['features'].shape[1] == 3  # not query scores or native 2D coefficients
        assert list(data['query_labels']) == ['new object','background']
    with pytest.raises(ValueError,match='differs'):
        export_feature_snapshot(server,{'scores':np.zeros_like(expected)})


def test_feature_stream_delta_and_fusion_produce_bb_for_new_query(tmp_path,monkeypatch):
    from raven_nav.raven_nav_node import RavenNavNode
    from coordination_msgs.msg import SemanticVoxelMap
    from coordination_bringup.semantic_delta import encode_delta
    from rclpy.serialization import deserialize_message,serialize_message
    from sensor_msgs.msg import PointCloud2,PointField
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time
    from raven_nav.behaviors.voxel_behavior import VoxelBehavior
    labels=['rgb_r','rgb_g','rgb_b','observation_count']+[f'feature:test:{i}' for i in range(3)]
    xyz=np.array([[10+x*.5,10+y*.5,8+z*.5] for x in range(5) for y in range(5) for z in range(3)],dtype='f4')
    values=np.tile([1,0,0,2,0,0,1],(len(xyz),1)).astype('f4')
    mat=np.c_[xyz,values].astype('<f4'); names=['x','y','z']+[f'sim_{i}' for i in range(7)]
    msg=SemanticVoxelMap(source_robot='robot_2',labels=labels,voxel_size=.5,
        cloud=PointCloud2(header=Header(stamp=Time(sec=100),frame_id='world_xy_agl'),height=1,width=len(mat),
            fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(names)],
            point_step=40,row_step=40*len(mat),data=mat.tobytes(),is_dense=True))
    full,base=encode_delta(serialize_message(msg),None)
    msg.cloud.header.stamp.sec=101
    delta,_=encode_delta(serialize_message(msg),base)
    assert deserialize_message(delta,SemanticVoxelMap).cloud.width == 0
    node=S(_boot_enu=np.zeros(3),_robot_name='robot_1',_query_labels=['new object','background'],
        _semantic_feature_labels=labels,_semantic_fusion=SemanticMapFusion(),_semantic_map_received_at={},
        _semantic_map_updates={},_semantic_map_rejections=0,_own_semantic_snapshot=None,
        _reusable_feature_map=True,_feature_queries=np.array([[0,0,1],[1,0,0]],dtype='f4'),
        _feature_probability=True,_semantic_map_ack_pub=S(publish=lambda m:None))
    node.get_logger=lambda:S(error=lambda *a,**kw:pytest.fail(str(a)))
    node._refresh_semantic_map=lambda:RavenNavNode._refresh_semantic_map(node)
    for data in [full,delta]:
        RavenNavNode._on_stream_semantic_map(node,deserialize_message(data,SemanticVoxelMap))
    vb=VoxelBehavior(get_clock=lambda:None,score_threshold=.5,min_cluster_size=10,confirm_hits=1)
    vb.condition_check(node._vox_xyz,node._vox_scores,node._query_labels,['new object'],committed_origin=None,committed_dir=None)
    assert vb.target_voxel_clusters
    assert set(vb.cluster_query_map.values()) == {'new object'}
    assert node._semantic_map_updates['robot_2']==2


def test_large_reusable_map_over_real_tcp_and_acknowledged_delta():
    import threading,time
    from coordination_bringup.map_stream import MapStream
    from coordination_bringup.semantic_delta import encode_delta
    from coordination_msgs.msg import SemanticVoxelMap
    from rclpy.serialization import serialize_message,deserialize_message
    from sensor_msgs.msg import PointCloud2,PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time
    n,d=20000,1152
    labels=[f'feature:test:{i}' for i in range(d)]
    names=['x','y','z']+[f'sim_{i}' for i in range(d)]
    matrix=np.zeros((n,d+3),dtype='<f4');matrix[:,0]=np.arange(n)*.5;matrix[:,2]=8;matrix[:,3:]=.25
    msg=SemanticVoxelMap(source_robot='robot_1',labels=labels,voxel_size=.5,
        cloud=PointCloud2(header=Header(stamp=Time(sec=100),frame_id='world_xy_agl'),height=1,width=n,
            fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(names)],
            point_step=4*(d+3),row_step=4*(d+3)*n,data=matrix.tobytes(),is_dense=True))
    got=[];traffic=[];ready=threading.Event();fusion=SemanticMapFusion()
    def receive(data):
        incoming=deserialize_message(data,SemanticVoxelMap)
        values=point_cloud2.read_points_numpy(incoming.cloud,field_names=names)
        fusion.update(incoming.source_robot,values[:,:3],values[:,3:],incoming.labels,incoming.cloud.header.stamp.sec)
        got.append(incoming.cloud.width);ready.set();return incoming.source_robot
    receiver=MapStream(receive,lambda *args:None,lambda name:True,host='127.0.0.1',port=0,interval=.02)
    sender=MapStream(lambda data:'robot_2',lambda *args:traffic.append(args),lambda name:True,
                     host='127.0.0.1',port=0,interval=.02,encode=encode_delta)
    try:
        sender.peer('robot_2',receiver.endpoint());sender.snapshot(serialize_message(msg))
        assert ready.wait(30),'large map did not arrive'
        deadline=time.monotonic()+30
        while not traffic or not traffic[-1][-1]:
            assert time.monotonic()<deadline,'large map not acknowledged'
            time.sleep(.02)
        ready.clear();matrix[123,3+1100]=.75;msg.cloud.data=matrix.tobytes();msg.cloud.header.stamp.sec=101
        sender.snapshot(serialize_message(msg));assert ready.wait(30)
        assert got == [n,1]
        _,values=fusion.arrays()
        assert values[123,1100] == .75
        assert values.shape == (n,d)
        assert traffic[0][2] > 90_000_000
    finally:
        sender.close();receiver.close()


def test_late_feature_encoder_does_not_falsely_reject_peer():
    from raven_nav.raven_nav_node import RavenNavNode
    node=S(_boot_enu=np.zeros(3),_robot_name='robot_1',_reusable_feature_map=True,
           _semantic_feature_labels=None,_semantic_map_rejections=0)
    RavenNavNode._on_stream_semantic_map(node,S(source_robot='robot_2'))
    assert node._semantic_map_rejections == 0
