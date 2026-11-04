from copy import deepcopy
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import droid_ms2_development as native
from droid_longer_worker import allocation_settings,fixed_allocation


def test_101_frames_and_filler_fit_in_power_of_two_buffer():
    result=allocation_settings(native.SETTINGS,101)
    assert result['buffer']==128 and result['buffer']>=101+16+1
    assert {k:v for k,v in result.items() if k!='buffer'}=={k:v for k,v in native.SETTINGS.items() if k!='buffer'}


@pytest.mark.parametrize('frames',[True,0,-1,101.])
def test_invalid_frame_budget_rejected(frames):
    with pytest.raises(ValueError):allocation_settings(native.SETTINGS,frames)


def test_override_restores_original_even_on_failure():
    original=native.SETTINGS
    plan={'source_frames':101,'settings':allocation_settings(original,101)}
    with pytest.raises(RuntimeError):
        with fixed_allocation(plan):
            assert native.SETTINGS['buffer']==128
            raise RuntimeError('injected native exception')
    assert native.SETTINGS is original


@pytest.mark.parametrize('field,value',[('warmup',7),('filter_thresh',2.),('stereo',False),('buffer',256)])
def test_any_unfrozen_native_setting_change_is_rejected(field,value):
    settings=allocation_settings(native.SETTINGS,101);settings[field]=value
    with pytest.raises(ValueError):
        with fixed_allocation({'source_frames':101,'settings':settings}):pass
