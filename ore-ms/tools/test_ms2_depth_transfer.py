import numpy as np
from ms2_depth_transfer import scaled_metrics,decision


def test_identical_shape_up_to_scale_is_perfect_for_both():
    gt=np.array([[1.,2.],[3.,4.]])
    row=scaled_metrics(gt,gt*3,gt*.2)
    assert row['common_pixels']==4
    assert row['metrics']['mact']['abs_rel']==0
    assert row['metrics']['mono']['rmse']<1e-14


def test_missing_candidate_pixel_has_a_coverage_cost():
    gt=np.ones((2,2))*3;mono=gt.copy();mono[0,0]=np.nan
    row=scaled_metrics(gt,gt,mono)
    assert not row['candidate_preserves_baseline_support']
    assert row['common_pixels']==3 and row['baseline_pixels']==4
    assert not decision([row]*20)['pass']


def test_no_reference_or_positive_predictions_is_not_a_success():
    row=scaled_metrics(np.zeros((2,2)),np.ones((2,2)),np.ones((2,2)))
    assert row['metrics'] is None and not decision([row]*20)['pass']


def test_no_pixel_pooling_or_single_frame_success_rescues_record():
    gt=np.array([[1.,2.],[3.,4.]])
    row=scaled_metrics(gt,gt*3,gt*.2)
    assert not decision([row])['pass']
    # Equal errors fail the strictly improving gate even on20frames.
    assert not decision([row]*20)['pass']


def test_mean_and_rmse_both_required():
    row={'candidate_preserves_baseline_support':True,'metrics':{
        'mact':{'abs_rel':.2,'rmse':1.},'mono':{'abs_rel':.1,'rmse':2.},
        'unscaled_mact':{'abs_rel':.3,'rmse':3.}}}
    result=decision([row]*20)
    assert result['checks']['abs_rel_improves'] and not result['pass']


def test_same_reference_range_and_positive_support_for_both():
    gt=np.array([[0.,5.],[90.,np.nan]])
    row=scaled_metrics(gt,np.array([[1.,5.],[2.,3.]]),np.array([[1.,10.],[2.,3.]]))
    assert row['reference_pixels']==row['common_pixels']==1
    assert row['metrics']['mono']['scale']==.5
