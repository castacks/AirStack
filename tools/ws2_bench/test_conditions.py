import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parent))
from conditions import validate,sensor_parameters

@pytest.mark.parametrize('bad',[
    {'rgb_noise':-1},{'delay':float('nan')},{'patch_size':.05},{'patch_size':1.0},
    {'patch_enabled':'false'},{'layout':'missing'},{'layout_seed':8},
    {'layout':'stock','patch_enabled':True},{'seed':True},{'extra':1},
])
def test_invalid_conditions(bad):
    with pytest.raises(ValueError):validate(bad)

def test_clean_attack_geometry_identity():
    c=validate({'layout':'furnished_b','layout_seed':5,'seed':18,'patch_enabled':True,'patch_size':.6,'rgb_noise':12,'delay':.2})
    clean=validate(dict(c,patch_enabled=False,rgb_noise=0,depth_noise=0,delay=0))
    for key in ['layout','layout_seed','seed','light','patch_size']:assert clean[key]==c[key]
    assert sensor_parameters(c)=={'disturbance_seed':18,'rgb_noise_stddev':12.,'depth_noise_stddev_m':0.,'fixed_sensor_delay_s':.2}

@pytest.mark.parametrize('legacy',[
    {'patch_strength':0}, {'patch_strength':.55}, {'patch_strength':1}, {'patch_height':1.2},
])
def test_legacy_patch_controls_cannot_silently_change_recorded_conditions(legacy):
    with pytest.raises(ValueError,match='Legacy patch controls'):
        validate(dict(patch_enabled=True,**legacy))

def test_current_yaml_uses_only_enabled_and_size_for_patch():
    import yaml
    from episode import resolved
    here=Path(__file__).resolve().parent
    for path in (here/'scenarios').glob('*.yaml'):
        c=resolved(yaml.safe_load(path.read_text()))['condition']
        assert {k for k in c if k.startswith('patch_')}=={'patch_enabled','patch_size'}
    for raw in yaml.safe_load((here/'demo.yaml').read_text())['conditions']:
        validate(raw)
