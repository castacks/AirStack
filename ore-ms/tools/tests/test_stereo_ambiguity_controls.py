from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from stereo_ambiguity_controls import bin_key,calculate

def points(x,y,z=10,offset=0):return [dict(feature_index=i+offset,uv=[15,15],reference_m=z,gap=a,error=b,faithful=True,reference_group='locally_consistent') for i,(a,b) in enumerate(zip(x,y))]
def test_exact_bin_edges():
 for z,d in [(4.999,0),(5,1),(10,2),(20,3),(40,4),(80,5)]:assert bin_key(dict(reference_m=z,uv=[30,60]),[90,90],'joint')==[d,2,1]
def test_ties_and_exact_retained_support():
 p=points([0,0,1,1,2,2,3,3,4,4],[9,9,8,8,7,7,6,6,5,5]);v=calculate(p,[90,90],'depth',10);assert abs(v['within_bin']+1)<1e-12 and v['retained_feature_indices']==list(range(10)) and v['retained_fraction']==1
 assert calculate(p,[90,90],'depth',30)['within_bin'] is None

def test_empty_and_constant_stay_explicit():
 assert calculate([],[90,90],'joint',10)['retained_fraction'] is None
 v=calculate(points([1]*30,list(range(30))),[90,90],'depth',30);assert v['within_bin'] is None and v['retained_points']==0 and v['bins'][2]['reason']=='constant'

def test_between_bin_confounding_and_within_reversal():
 a=points(list(range(10)),list(range(9,-1,-1)),z=2);b=points(list(range(100,110)),list(range(109,99,-1)),z=20,offset=10);v=calculate(a+b,[90,90],'depth',10)
 assert v['full_raw']['rho']>0 and v['retained_raw']['rho']>0 and abs(v['within_bin']+1)<1e-12 and v['retained_points']==20
