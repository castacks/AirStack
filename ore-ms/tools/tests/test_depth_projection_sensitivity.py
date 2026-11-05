import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from depth_projection_sensitivity import neighborhood

def feature(z=9,u=5,v=5):return [u,v,0,4,z,False,False]
def test_singleton_is_insufficient_even_if_exact():
 d=np.zeros((11,11),np.uint16);d[5,5]=2560;r=neighborhood(feature(10),d,10,.1,1)
 assert r['group']=='insufficient' and len(r['candidates'])==1 and r['interval_class']=='contains_unit'

def test_flat_reference_short_and_long_intervals():
 d=np.full((11,11),2560,np.uint16)
 assert neighborhood(feature(9),d,10,.1,1)['group']=='locally_consistent'
 assert neighborhood(feature(9),d,10,.1,1)['interval_class']=='short_for_all'
 assert neighborhood(feature(11),d,10,.1,1)['interval_class']=='long_for_all'

def test_mixed_depth_range_keeps_all_labels():
 d=np.full((11,11),2560,np.uint16);d[5,4]=2048;d[5,6]=3072;r=neighborhood(feature(9),d,10,.1,1)
 assert r['group']=='mixed_depth' and r['ratio_lower']==.75 and r['ratio_upper']==1.125 and r['interval_class']=='contains_unit' and len(r['candidates'])==9

def test_relative_span_threshold_inclusive():
 d=np.zeros((11,11),np.uint16);d[5,4:7]=np.array([95,100,105],np.uint16)*256
 assert neighborhood(feature(99),d,10,.1,1)['group']=='locally_consistent'
 d[5,6]=106*256
 assert neighborhood(feature(99),d,10,.1,1)['group']=='mixed_depth'

def test_radius_and_bounds_and_zero_do_not_fill():
 d=np.zeros((6,6),np.uint16);d[0,0]=2560;d[2,2]=2560
 a=neighborhood(feature(u=0,v=0),d,1,.1,1);b=neighborhood(feature(u=0,v=0),d,1,.1,2)
 # Right projection at original u is outside even when reference pixel exists.
 assert not a['candidates'] and not b['candidates'] and a['interval_class']=='unavailable'
 a=neighborhood(feature(u=1,v=1),d,1,.1,1);assert len(a['candidates'])==2 and a['rejected']['zero']==7

def test_unit_ratio_endpoint_is_not_strict_short():
 d=np.full((11,11),2560,np.uint16);d[5,4]=2304
 assert neighborhood(feature(9),d,10,.1,1)['interval_class']=='contains_unit'
