import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parent))
from conditions import validate,sensor_parameters

@pytest.mark.parametrize('bad',[
    {'rgb_noise':-1},{'delay':float('nan')},{'patch_strength':1.1},
    {'patch_enabled':'false'},{'layout':'missing'},{'layout_seed':8},
    {'layout':'stock','patch_enabled':True},{'seed':True},{'extra':1},
])
def test_invalid_conditions(bad):
    with pytest.raises(ValueError):validate(bad)

def test_clean_attack_geometry_identity():
    c=validate({'layout':'furnished_b','layout_seed':5,'seed':18,'patch_enabled':True,'patch_strength':.6,'rgb_noise':12,'delay':.2})
    clean=validate(dict(c,patch_enabled=False,patch_strength=0,rgb_noise=0,depth_noise=0,delay=0))
    for key in ['layout','layout_seed','seed','light','patch_size','patch_height']:assert clean[key]==c[key]
    assert sensor_parameters(c)=={'disturbance_seed':18,'rgb_noise_stddev':12.,'depth_noise_stddev_m':0.,'fixed_sensor_delay_s':.2}
