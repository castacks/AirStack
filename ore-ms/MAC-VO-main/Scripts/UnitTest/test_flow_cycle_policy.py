from Scripts.Setup.predict_flow_cycle import choose

def test_fixed_camera_not_masked_by_undefined_signal():
 c={name:{'motion_available':True,'closure_median':None,'angular_covariance_median':None} for name in ('rgb','thermal')}
 assert choose(c,'rgb')=='rgb' and choose(c,'thermal')=='thermal';assert choose(c,'closure') is None
 c['thermal']['closure_median']=2.;assert choose(c,'closure')=='thermal'

def test_ties_and_unsupported_low_score():
 c={name:{'motion_available':True,'closure_median':1.,'angular_covariance_median':1.} for name in ('rgb','thermal')}
 assert choose(c,'closure')==choose(c,'variance')=='rgb'
 c['rgb'].update(motion_available=False,closure_median=0.,angular_covariance_median=0.)
 assert choose(c,'closure')==choose(c,'variance')=='thermal' and choose(c,'rgb') is None
 c['thermal']['motion_available']=False;assert choose(c,'closure') is None
