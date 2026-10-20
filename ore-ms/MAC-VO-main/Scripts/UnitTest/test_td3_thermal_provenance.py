from pathlib import Path
import cv2
import numpy as np
import pytest
from Scripts.Setup.td3_thermal_provenance import producer_functions,preprocess,compare_image,triangulate
from Scripts.Setup.audit_td3_geometry import rectification_residual

PRODUCER=Path(__file__).resolve().parents[3]/'data/calibration/upstream/get_thermal_data.py'


def test_ast_loader_and_source_equivalence_without_mutation():
    funcs=producer_functions(PRODUCER)
    raw=np.arange(4096,dtype=np.uint16).reshape(64,64)+1000;original=raw.copy()
    result=preprocess(raw,funcs)
    expected=funcs['enhance_image'](funcs['process_image'](raw.copy(),'firestereo_v2'))
    np.testing.assert_equal(result,expected);np.testing.assert_equal(raw,original)
    assert result.dtype==np.uint8


def test_only_named_functions_are_executed(tmp_path):
    source=tmp_path/'producer.py'
    source.write_text("raise RuntimeError('main side effect')\n"+'\n'.join(f'def {name}(*args): return 7' for name in ['read_kalibr_stereo','stereo_rectify','process_image','enhance_image']))
    assert producer_functions(source)['enhance_image'](0)==7


def test_invalid_constant_and_degenerate_percentiles():
    funcs=producer_functions(PRODUCER)
    for raw in [np.ones((20,20),np.uint16),np.ones((20,20),np.uint8),np.empty((0,0),np.uint16)]:
        with pytest.raises(ValueError):preprocess(raw,funcs)
    raw=np.ones((100,100),np.uint16);raw[0,0]=100
    with pytest.raises(ValueError):preprocess(raw,funcs)


def test_channel_replication_and_exact_difference():
    gray=np.arange(256,dtype=np.uint8).reshape(16,16);rgb=np.repeat(gray[:,:,None],3,axis=2)
    r=compare_image(rgb,gray);assert r['exact_full_array_equal'] and r['channels_replicated']
    changed=gray.copy();changed[0,0]=10
    r=compare_image(rgb,changed);assert r['gray_mae']==10/256 and r['gray_max_abs']==10


def test_cheirality_changes_without_changing_reprojection():
    k=np.array([[100.,0,50],[0,100.,50],[0,0,1.]])
    p1=k@np.column_stack([np.eye(3),np.zeros(3)])
    p2=k@np.column_stack([np.eye(3),np.array([-.3,0,0])])
    uv0=np.array([[50.,50.],[60.,60.],[40.,40.]])
    uv1=uv0-np.array([15.,0])
    positive,xyz=triangulate(p1,p2,uv0,uv1)
    flipped=p2.copy();flipped[:,3]*=-1
    negative,negxyz=triangulate(p1,flipped,uv0,uv1)
    assert positive['positive_both_count']==3 and negative['positive_both_count']==0
    np.testing.assert_allclose(xyz,-negxyz,atol=1e-12)
    assert positive['reprojection_px']['p10_p50_p90'][1]<1e-10
    assert negative['reprojection_px']['p10_p50_p90'][1]<1e-10


def test_consistent_rectification_and_empty_triangulation():
    k=np.array([[100.,0,50],[0,100.,50],[0,0,1.]])
    p1=k@np.column_stack([np.eye(3),np.zeros(3)]);p2=p1.copy();p2[0,3]=-30
    relative=np.eye(4);relative[0,3]=.3
    result=rectification_residual(relative,dict(p=p1,r=np.eye(3)),dict(p=p2,r=np.eye(3)))
    assert all(abs(v)<1e-12 for v in result.values())
    r,xyz=triangulate(p1,p2,np.empty((0,2)),np.empty((0,2)))
    assert r['count']==0 and xyz.shape==(0,3)


def test_producer_rectify_strided_transform_compatibility():
    funcs=producer_functions(PRODUCER)
    transform=np.eye(4);transform[0,3]=-.3
    config=dict(K_left=np.array([[100.,0,32],[0,100.,32],[0,0,1.]]),
                K_right=np.array([[100.,0,32],[0,100.,32],[0,0,1.]]),
                distort_left=np.zeros(5),distort_right=np.zeros(5),T_right2left=transform)
    raw=np.arange(4096,dtype=np.uint16).reshape(64,64)
    result=funcs['stereo_rectify'](raw.copy(),raw.copy(),config)
    expected=cv2.stereoRectify(config['K_left'],config['distort_left'],config['K_right'],config['distort_right'],
                              (64,64),np.eye(3),np.array([[.3],[0.],[0.]]),flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
    for actual,reference in zip(result[4:8],expected[:4]):np.testing.assert_equal(actual,reference)
