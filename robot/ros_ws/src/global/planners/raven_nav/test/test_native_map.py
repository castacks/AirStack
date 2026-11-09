import json
import threading
import time
from types import SimpleNamespace as S
import numpy as np
import pytest
from raven_nav.native_map import NativeFeatureDecoder, basis_fingerprint, project_native
from raven_nav.feature_map import query_features
from coordination_msgs.msg import SemanticVoxelMap
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs_py import point_cloud2
from coordination_bringup.semantic_delta import encode_delta


def message(coefficients, basis, robot='robot_1', revision=1):
    c=np.asarray(coefficients,dtype='f4');b=np.asarray(basis,dtype='f4')
    labels=['rgb_r','rgb_g','rgb_b','observation_count']+[f'pca:test:{i}' for i in range(c.shape[1])]
    matrix=np.c_[np.arange(len(c))*.5,np.zeros(len(c)),np.full(len(c),8),np.tile([1,0,0,2],(len(c),1)),c].astype('<f4')
    names=['x','y','z']+[f'sim_{i}' for i in range(len(labels))]
    return SemanticVoxelMap(source_robot=robot,representation='native_pca_v1',feature_space='test',
        basis_id=basis_fingerprint(b),basis_rows=b.shape[0],basis_cols=b.shape[1],decoded_dimension=b.shape[0],
        pca_basis=b.ravel().tolist(),labels=labels,voxel_size=.5,
        cloud=PointCloud2(header=Header(stamp=Time(sec=revision),frame_id='world_xy_agl'),height=1,width=len(c),
            fields=[PointField(name=f,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,f in enumerate(names)],
            point_step=4*len(names),row_step=4*len(names)*len(c),data=matrix.tobytes(),is_dense=True))


def read_features(msg):
    names=['x','y','z']+[f'sim_{i}' for i in range(len(msg.labels))]
    return point_cloud2.read_points_numpy(msg.cloud,field_names=names)[:,7:]


def test_two_senders_with_different_pca_bases_reconstruct_same_feature_space():
    a=np.eye(3,dtype='f4')[:,:2];b=a[:,::-1].copy()
    decoder=NativeFeatureDecoder('test',3,lambda c,b:c@b.T)
    r1=decoder.decode(message([[1,2]],a))
    r2=decoder.decode(message([[2,1]],b,'robot_2'))
    np.testing.assert_array_equal(read_features(r1),[[1,2,0]])
    np.testing.assert_array_equal(read_features(r2),[[1,2,0]])
    assert list(r1.labels)==list(r2.labels)
    # A new query was never needed by the sender when building the map.
    np.testing.assert_allclose(query_features(read_features(r1),[[1,0,0]],False),[[1/np.sqrt(5)]])


def test_delta_sends_basis_once_and_resync_or_basis_change_sends_it_again():
    b=np.eye(3,dtype='f4')[:,:2]
    wire,base=encode_delta(serialize_message(message([[1,2],[3,4]],b)),None)
    first=deserialize_message(wire,SemanticVoxelMap);assert len(first.pca_basis)==6
    decoder=NativeFeatureDecoder('test',3,lambda c,b:c@b.T);decoder.decode(first)
    msg=message([[1,2],[3,5]],b,revision=2)
    wire,state=encode_delta(serialize_message(msg),base);delta=deserialize_message(wire,SemanticVoxelMap)
    assert delta.is_delta and len(delta.pca_basis)==0 and delta.cloud.width==1
    np.testing.assert_array_equal(read_features(decoder.decode(delta)),[[3,5,0]])
    with pytest.raises(ValueError,match='missing source PCA'):
        NativeFeatureDecoder('test',3,lambda c,b:c@b.T).decode(delta)
    wire,_=encode_delta(serialize_message(msg),None)
    assert len(deserialize_message(wire,SemanticVoxelMap).pca_basis)==6
    changed=message([[1,2]],b[:,::-1],revision=3)
    wire,_=encode_delta(serialize_message(changed),state)
    assert not deserialize_message(wire,SemanticVoxelMap).is_delta
    bad=message([[1,2]],b);bad.pca_basis[0]=42
    with pytest.raises(ValueError,match='fingerprint'):
        decoder.decode(bad)


def test_native_projection_uses_sender_basis_and_existing_encoder():
    import torch
    server=S(mapper=S(device='cpu',encoder=S(align_spatial_features_with_language=lambda f:2*f)),
             _queries_feats={'text':torch.zeros(1,3)})
    b=np.eye(3,dtype='f4')[:,:2]
    np.testing.assert_array_equal(project_native(server,[[1,2]],b),[[2,4,0]])
    assert project_native(server,np.empty((0,2),dtype='f4'),b).shape==(0,3)


def test_native_sender_wire_does_not_replace_own_expanded_map(tmp_path,monkeypatch):
    from raven_nav.raven_nav_node import RavenNavNode
    from raven_nav.semantic_map import SemanticMapFusion
    from coordination_bringup.map_snapshot import read_snapshot
    from raven_nav.native_map import canonical_channels
    monkeypatch.setenv('RAVEN_MAP_SNAPSHOT_DIR',str(tmp_path))
    values=np.array([[1,0,0,2,1,2,0]],dtype='f4');b=np.eye(3,dtype='f4')[:,:2];published=[]
    node=S(_boot_enu=np.zeros(3),_robot_name='robot_1',_query_labels=['car','wall'],
        _semantic_feature_labels=canonical_channels('test',3),_semantic_fusion=SemanticMapFusion(),
        _own_semantic_snapshot=(np.array([[0,0,8]],dtype='f4'),values,1),
        _native_compressed_map=True,_native_wire_snapshot=dict(values=values[:,:6],basis=b,
            basis_id=basis_fingerprint(b),space='test',decoded_dimension=3),
        _reusable_feature_map=True,_feature_queries=np.array([[1,0,0],[0,1,0]],dtype='f4'),
        _feature_probability=True,_semantic_map_pub=S(publish=published.append))
    node.get_clock=lambda:S(now=lambda:S(to_msg=lambda:Time(sec=1)))
    node.get_logger=lambda:S(error=lambda *a,**kw:pytest.fail(str(a)))
    RavenNavNode._refresh_semantic_map(node,publish=True)
    data,_=read_snapshot('robot_1',None);msg=deserialize_message(data,SemanticVoxelMap)
    assert msg.representation=='native_pca_v1' and len(msg.labels)==6
    assert node._semantic_fusion.arrays()[1].shape==(1,7)
    assert node._vox_scores.shape==(1,2)


def test_native_stream_decodes_before_raven_fusion_ack(tmp_path,monkeypatch):
    from coordination_bringup.map_stream import MapStream
    from coordination_bringup.map_snapshot import write_native_received_snapshot,native_received_snapshots,write_received_snapshot,received_snapshots
    from raven_nav.raven_nav_node import RavenNavNode
    from raven_nav.semantic_map import SemanticMapFusion
    from raven_nav.native_map import canonical_channels
    monkeypatch.setenv('RAVEN_MAP_SNAPSHOT_DIR',str(tmp_path))
    fused=threading.Event();received=[];traffic=[];stop=threading.Event();b=np.eye(3,dtype='f4')[:,:2]
    node=S(_boot_enu=np.zeros(3),_robot_name='robot_2',_query_labels=['new query','wall'],
        _semantic_feature_labels=canonical_channels('test',3),_semantic_fusion=SemanticMapFusion(),
        _own_semantic_snapshot=None,_reusable_feature_map=True,
        _feature_queries=np.array([[1,0,0],[0,1,0]],dtype='f4'),_feature_probability=True,
        _semantic_map_received_at={},_semantic_map_updates={},_semantic_map_rejections=0,
        _semantic_map_ack_pub=S(publish=lambda msg:fused.set()))
    node.get_logger=lambda:S(error=lambda *a,**kw:pytest.fail(str(a)))
    node._refresh_semantic_map=lambda:RavenNavNode._refresh_semantic_map(node)
    decoder=NativeFeatureDecoder('test',3,lambda c,b:c@b.T)
    def worker():
        native_seen={};decoded_seen={}
        while not stop.is_set():
            for source,data,version in native_received_snapshots('robot_2',native_seen):
                msg=decoder.decode(deserialize_message(data,SemanticVoxelMap))
                write_received_snapshot('robot_2',source,serialize_message(msg));native_seen[source]=version
            for source,data,version in received_snapshots('robot_2',decoded_seen):
                RavenNavNode._on_stream_semantic_map(node,deserialize_message(data,SemanticVoxelMap));decoded_seen[source]=version
            time.sleep(.01)
    thread=threading.Thread(target=worker,daemon=True);thread.start()
    def receive(data):
        fused.clear();msg=deserialize_message(data,SemanticVoxelMap)
        received.append(msg);write_native_received_snapshot('robot_2',msg.source_robot,data)
        assert fused.wait(10),'map was not decoded and fused'
        return msg.source_robot
    receiver=MapStream(receive,lambda *a:None,lambda peer:True,host='127.0.0.1',port=0,interval=.02)
    sender=MapStream(lambda data:'robot_2',lambda *a:traffic.append(a),lambda peer:True,
        host='127.0.0.1',port=0,interval=.02,encode=encode_delta)
    try:
        sender.peer('robot_2',receiver.endpoint());sender.snapshot(serialize_message(message([[1,2]],b)))
        deadline=time.monotonic()+10
        while not traffic or not traffic[-1][-1]:
            assert time.monotonic()<deadline;time.sleep(.02)
        assert node._semantic_map_updates['robot_1']==1
        sender.snapshot(serialize_message(message([[1,3]],b,revision=2)))
        while len(traffic)<2 or not traffic[-1][-1]:
            assert time.monotonic()<deadline;time.sleep(.02)
        assert len(received[0].pca_basis)==6 and not received[1].pca_basis
        np.testing.assert_array_equal(node._semantic_fusion.arrays()[1][0,4:],[1,3,0])
    finally:
        sender.close();receiver.close();stop.set();thread.join(2)


def test_native_export_ingress_and_actual_raven_callbacks(tmp_path,monkeypatch):
    import torch
    from omegaconf import OmegaConf
    from raven_nav.feature_map import export_feature_snapshot,feature_snapshot_path
    from raven_nav.raven_nav_node import RavenNavNode
    from raven_nav.semantic_map import SemanticMapFusion
    from coordination_bringup.map_snapshot import read_snapshot,write_native_received_snapshot,received_snapshots
    from rayfronts.utils import compute_cos_sim
    monkeypatch.setenv('RAVEN_MAP_SNAPSHOT_DIR',str(tmp_path))
    monkeypatch.setenv('RAVEN_NATIVE_COMPRESSED_MAP','true')
    monkeypatch.setenv('ROBOT_NAME','robot_2')
    torch.manual_seed(8);n,k,d,out=128,100,1024,1152
    coefficients=torch.randn(n,k);basis=torch.randn(d,k)*.03;head=torch.randn(d,out)*.03
    def align(f):
        return (f.squeeze(-1).squeeze(-1)@head).unsqueeze(-1).unsqueeze(-1)
    xyz=torch.stack([torch.zeros(n),torch.full((n,),-8.),torch.arange(n)*.5],dim=1)
    mapper=S(device='cpu',global_vox_xyz=xyz,global_vox_feat=coefficients,
        global_vox_rgb=torch.ones(n,3),global_vox_cnt=torch.ones(n,1),
        feat_compressor=S(basis=basis,decompress=lambda c:c@basis.T),
        encoder=S(align_spatial_features_with_language=align))
    queries=torch.randn(3,out);expanded=coefficients@basis.T@head
    expected=compute_cos_sim(queries,expanded,True).numpy()
    labels=['new target','wall','tree']
    server=S(mapper=mapper,cfg=OmegaConf.create({'encoder':{'model':'test'},
        'querying':{'compressed':False,'compute_prob':True}}),
        _queries_labels={'text':labels},_queries_feats={'text':queries},status=S(name='MAPPING'))
    try:
        export_feature_snapshot(server,{'scores':expected[:64]})
        with np.load(feature_snapshot_path('robot_2')) as data:
            assert data['native_coefficients'].shape==(n,100)
            assert data['features'].shape==(n,1152)
            assert float(data['native_reference_error'])<.002
        node=S(_boot_enu=np.zeros(3),_robot_name='robot_2',_query_labels=labels,
            _feature_file_version=None,_semantic_feature_labels=None,_native_compressed_map=True,
            _semantic_map_rejections=0,_semantic_fusion=SemanticMapFusion(),_semantic_source_stamp=0,
            _reusable_feature_map=True,_semantic_map_pub=S(get_subscription_count=lambda:0),
            _semantic_map_received_at={},_semantic_map_updates={},_semantic_map_received_files={},
            _semantic_map_ack_pub=S(publish=lambda msg:None))
        node.get_logger=lambda:S(error=lambda *a,**kw:pytest.fail(str(a)))
        node.get_clock=lambda:S(now=lambda:S(to_msg=lambda:Time(sec=2)))
        node._refresh_semantic_map=lambda **kw:RavenNavNode._refresh_semantic_map(node,**kw)
        node._on_stream_semantic_map=lambda msg:RavenNavNode._on_stream_semantic_map(node,msg)
        RavenNavNode._poll_own_feature_map(node)
        raw,_=read_snapshot('robot_2',None);wire=deserialize_message(raw,SemanticVoxelMap)
        assert wire.representation=='native_pca_v1' and wire.basis_cols==100
        wire.source_robot='robot_1'
        raw,_=encode_delta(serialize_message(wire),None)
        write_native_received_snapshot('robot_2','robot_1',raw)
        deadline=time.monotonic()+10
        while not list(received_snapshots('robot_2',{})):
            assert time.monotonic()<deadline,'existing RayFronts ingress did not decode native map'
            time.sleep(.02)
        RavenNavNode._poll_stream_semantic_maps(node)
        assert node._semantic_map_updates['robot_1']==1
        np.testing.assert_allclose(node._vox_scores,expected,atol=2e-5)
        assert node._semantic_map_rejections==0
    finally:
        server.status.name='CLOSED'
        if getattr(server,'_native_ingress_thread',None):server._native_ingress_thread.join(2)
