"""Boundary tests for retrospective temporal matching and statistics."""
import math
from covariance_lead_time import match, association, positive_log

def test_match():
    rows=[{'start':0,'end':100_000_000},{'start':100_000_000,'end':200_000_000},
          {'start':200_000_000,'end':300_000_000},{'start':1_500_000_000,'end':1_600_000_000}]
    assert match(rows,0)==[(0,1),(1,2)]
    assert match(rows,.1)==[(0,2)]
    assert match(rows,.5)==[]
    assert match(rows[:1],0)==[]
    assert match([{'start':0,'end':100_000_000},{'start':99_000_000,'end':200_000_000}],0)==[]
    assert match([{'start':0,'end':100_000_000},{'start':150_000_000,'end':200_000_000}],0)==[(0,1)]
    assert match([{'start':0,'end':100_000_000},{'start':150_000_001,'end':200_000_000}],0)==[]

def test_statistics():
    assert association([1]*8,list(range(8)))['status']=='constant'
    assert association(range(7),range(7))['rho'] is None
    assert math.isclose(association(range(8),range(7,-1,-1))['rho'],-1)
    assert association([None]*8,range(8))['n']==0
    assert positive_log(0) is None and positive_log(-1) is None and positive_log(None) is None
    assert positive_log(1)==0
