from types import SimpleNamespace as S
import time
import numpy as np
import pytest
import torch
from raven_nav.wire_pca import WirePCA,project_wire,wire_fingerprint
from raven_nav.native_map import NativeFeatureDecoder
from raven_nav.feature_map import export_feature_snapshot,feature_snapshot_path
from coordination_msgs.msg import SemanticVoxelMap
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from sensor_msgs_py import point_cloud2
from rclpy.serialization import serialize_message,deserialize_message
from coordination_bringup.semantic_delta import encode_delta


def message(c,b,m,revision=1):
    c=np.asarray(c,dtype='f4');labels=['rgb_r','rgb_g','rgb_b','observation_count']+[f'pca:test:{i}' for i in range(c.shape[1])]
    names=['x','y','z']+[f'sim_{i}' for i in range(len(labels))]
    matrix=np.c_[np.arange(len(c))*.5,np.zeros(len(c)),np.full(len(c),8),np.tile([1,0,0,2],(len(c),1)),c].astype('<f4')
    return SemanticVoxelMap(source_robot='robot_1',representation='wire_pca_v1',feature_space='test',
        basis_id=wire_fingerprint(b,m),basis_rows=b.shape[0],basis_cols=b.shape[1],decoded_dimension=b.shape[0],
        pca_basis=b.ravel().tolist(),pca_mean=m.tolist(),labels=labels,voxel_size=.5,
        cloud=PointCloud2(header=Header(stamp=Time(sec=revision),frame_id='world_xy_agl'),height=1,width=len(c),
            fields=[PointField(name=f,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,f in enumerate(names)],
            point_step=4*len(names),row_step=4*len(names)*len(c),data=matrix.tobytes(),is_dense=True))


def features(msg):
    return point_cloud2.read_points_numpy(msg.cloud,field_names=['x','y','z']+[f'sim_{i}' for i in range(len(msg.labels))])[:,7:]


def test_centered_pca_does_not_mutate_features_rng_or_refit_on_later_data():
    rng=np.random.default_rng(9);x=rng.normal(size=(200,128)).astype('f4')+4;original=x.copy()
    codec=WirePCA(100);torch.manual_seed(44);state=torch.random.get_rng_state().clone()
    c=codec.encode(x,'cpu')
    assert torch.equal(state,torch.random.get_rng_state())
    np.testing.assert_array_equal(x,original)
    assert c.shape==(200,100);b=codec.basis.copy();m=codec.mean.copy()
    np.testing.assert_allclose(c.mean(axis=0),0,atol=2e-5)
    server=S(mapper=S(device='cpu'))
    y=project_wire(server,c,b,m);assert y.shape==x.shape
    assert np.mean((y-x)**2)<np.mean((m-x)**2)
    codec.encode(x+10,'cpu');np.testing.assert_array_equal(codec.basis,b);np.testing.assert_array_equal(codec.mean,m)
    assert WirePCA(100).encode(x[:20],'cpu') is None


def test_wire_basis_mean_transmitted_once_and_reconstructed_without_encoder_head():
    b=np.eye(4,dtype='f4')[:,:2];m=np.array([10,20,30,40],dtype='f4')
    decoder=NativeFeatureDecoder('test',4,lambda *a:pytest.fail('wire PCA must not apply native encoder projection'))
    full,base=encode_delta(serialize_message(message([[1,2],[3,4]],b,m)),None)
    expanded=decoder.decode(deserialize_message(full,SemanticVoxelMap))
    np.testing.assert_array_equal(features(expanded),[[11,22,30,40],[13,24,30,40]])
    raw,_=encode_delta(serialize_message(message([[1,2],[3,5]],b,m,2)),base);delta=deserialize_message(raw,SemanticVoxelMap)
    assert delta.is_delta and not delta.pca_basis and not delta.pca_mean
    np.testing.assert_array_equal(features(decoder.decode(delta)),[[13,25,30,40]])
    changed_mean=m+1
    raw,_=encode_delta(serialize_message(message([[1,2]],b,changed_mean,3)),base)
    assert not deserialize_message(raw,SemanticVoxelMap).is_delta
    bad=message([[1,2]],b,m);bad.pca_mean[0]+=1
    with pytest.raises(ValueError,match='fingerprint'):decoder.decode(bad)


def test_wire_export_leaves_real_mapper_features_uncompressed_and_records_loss(tmp_path,monkeypatch):
    from omegaconf import OmegaConf
    from rayfronts.utils import compute_cos_sim
    from coordination_bringup.map_snapshot import write_native_received_snapshot,received_snapshots,read_snapshot
    from raven_nav.raven_nav_node import RavenNavNode
    from raven_nav.semantic_map import SemanticMapFusion
    monkeypatch.setenv('RAVEN_MAP_SNAPSHOT_DIR',str(tmp_path));monkeypatch.setenv('RAVEN_WIRE_PCA_MAP','true')
    monkeypatch.setenv('RAVEN_NATIVE_COMPRESSED_MAP','false');monkeypatch.setenv('ROBOT_NAME','robot_2')
    torch.manual_seed(7);n,d=140,128;f=torch.randn(n,d)+2;original=f.clone();q=torch.randn(3,d)
    mapper=S(device='cpu',global_vox_xyz=torch.stack([torch.zeros(n),torch.full((n,),-8.),torch.arange(n)*.5],dim=1),
        global_vox_feat=f,global_vox_rgb=torch.ones(n,3),global_vox_cnt=torch.ones(n,1),feat_compressor=None,
        encoder=S(align_spatial_features_with_language=lambda x:x))
    expected=compute_cos_sim(q,f,True).numpy();labels=['car','wall','tree']
    server=S(mapper=mapper,_queries_labels={'text':labels},_queries_feats={'text':q},status=S(name='MAPPING'),
        cfg=OmegaConf.create({'encoder':{'model':'test'},'querying':{'compressed':False,'compute_prob':True}}))
    try:
        export_feature_snapshot(server,{'scores':expected[:64]})
        assert mapper.feat_compressor is None and torch.equal(mapper.global_vox_feat,original)
        with np.load(feature_snapshot_path('robot_2')) as data:
            assert data['native_coefficients'].shape==(n,100) and data['features'].shape==(n,d)
            assert not data['local_mapper_pca'] and np.isfinite(data['native_reference_error'])
        node=S(_boot_enu=np.zeros(3),_robot_name='robot_2',_query_labels=labels,_feature_file_version=None,
            _semantic_feature_labels=None,_native_compressed_map=True,_wire_pca_map=True,
            _semantic_map_rejections=0,_semantic_fusion=SemanticMapFusion(),_semantic_source_stamp=0,
            _reusable_feature_map=True,_semantic_map_pub=S(get_subscription_count=lambda:0),
            _semantic_map_received_at={},_semantic_map_updates={},_semantic_map_received_files={},_semantic_map_ack_pub=S(publish=lambda m:None))
        node.get_clock=lambda:S(now=lambda:S(to_msg=lambda:Time(sec=2)))
        node.get_logger=lambda:S(error=lambda *a,**kw:pytest.fail(str(a)))
        node._refresh_semantic_map=lambda **kw:RavenNavNode._refresh_semantic_map(node,**kw)
        node._on_stream_semantic_map=lambda msg:RavenNavNode._on_stream_semantic_map(node,msg)
        RavenNavNode._poll_own_feature_map(node)
        np.testing.assert_allclose(node._vox_scores,expected,atol=2e-5)  # own evidence remains full precision
        raw,_=read_snapshot('robot_2',None);wire=deserialize_message(raw,SemanticVoxelMap)
        assert wire.representation=='wire_pca_v1' and wire.basis_cols==100 and len(wire.pca_mean)==d
        wire.source_robot='robot_1';raw,_=encode_delta(serialize_message(wire),None)
        write_native_received_snapshot('robot_2','robot_1',raw);deadline=time.monotonic()+10
        while not list(received_snapshots('robot_2',{})):
            assert time.monotonic()<deadline;time.sleep(.02)
        RavenNavNode._poll_stream_semantic_maps(node)
        assert node._semantic_map_updates['robot_1']==1 and node._semantic_map_rejections==0
        assert node._wire_pca_rmse>=0
    finally:
        server.status.name='CLOSED'
        if getattr(server,'_native_ingress_thread',None):server._native_ingress_thread.join(2)
