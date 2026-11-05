from copy import deepcopy
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from orb_full_record_matched import matched_view

def fixture():
    d=dict(time_ns=[0,1,2],source_ids=['000000','000001','000002'],poses=[[[1]],[[2]],[[3]]],segments=[0,0,0])
    o=deepcopy(d);o['poses'][1]=None;o['segments']=[0,None,1];return d,o

def test_exact_masks_and_fragment_labels_without_native_pose_edits():
    d,o=fixture();old=deepcopy(d);v=matched_view(d,o)
    assert v['poses']==[[[1]],None,[[3]]] and v['segments']==[0,None,1] and d==old

@pytest.mark.parametrize('kind',['clock','source','null_segment','droid_missing'])
def test_mismatched_support_refused(kind):
    d,o=fixture()
    if kind=='clock':o['time_ns'][0]=.1
    elif kind=='source':o['source_ids'][0]='000009'
    elif kind=='null_segment':o['segments'][1]=0
    else:d['poses'][0]=None
    with pytest.raises(ValueError):matched_view(d,o)
