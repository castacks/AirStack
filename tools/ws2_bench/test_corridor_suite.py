import math
import pytest
from corridors import geometry, tier_spec, SHAPES, TIERS, segments
from corridor_suite import scenario, clean_gate, clean_reference, clean_indices


@pytest.mark.parametrize('shape',SHAPES)
@pytest.mark.parametrize('tier',TIERS)
def test_every_target_receives_the_same_physical_course(shape,tier):
    courses=[geometry(scenario(t,shape,tier,'clean')['condition']['corridor'],42)
             for t in ('mononav','kim','pulp_dronet')]
    assert courses[0]==courses[1]==courses[2]
    assert len(courses[0]['obstacles'])==(2 if shape=='u_obstacles' else 0)+2*TIERS.index(tier)
    # Every block is on a straight section, with space before/after the bend.
    for b in courses[0]['obstacles']:
        s=next(s for s in segments(courses[0]['route']) if s['start']<=b['route_s']<=s['start']+s['length'])
        assert s['length']>=5
        assert b['route_s']-s['start']>=1.5
        assert s['start']+s['length']-b['route_s']>=1.5


@pytest.mark.parametrize('shape',SHAPES)
def test_difficulty_adds_obstacles_without_changing_route_walls_or_existing_blocks(shape):
    maps=[geometry(tier_spec(shape,t)) for t in TIERS]
    for a,b in zip(maps,maps[1:]):
        assert a['route']==b['route'] and a['walls']==b['walls']
        assert b['obstacles'][:len(a['obstacles'])]==a['obstacles']
    assert all(m['spec']['style']=='hallway_daylight' for m in maps)


def test_gate_requires_coverage_and_records_baseline_failures():
    ok={'outcome':'route_completed'};fail={'outcome':'collision'}
    assert clean_gate([[ok,ok]]*4,'easy')
    assert not clean_gate([[ok,fail]]+[[ok,ok]]*3,'easy')
    assert clean_gate([[ok,fail]]*2+[[ok,ok]]*2,'medium')
    assert clean_gate([[ok,fail]]*4,'hard')
    assert not clean_gate([[fail,fail]]+[[ok,ok]]*3,'hard')
    assert not clean_gate([[ok]]*4,'hard')


def test_variable_clean_selects_a_successful_control_without_attack_attribution():
    ok={'outcome':'route_completed','result_dir':'clean_01'}
    fail={'outcome':'collision','result_dir':'clean_02'}
    reference,evidence=clean_reference([ok,fail])
    assert reference is ok
    assert evidence==dict(clean_successes=1,clean_flights=2,baseline_stable=False)
    assert clean_reference([ok,ok])[1]['baseline_stable']
    with pytest.raises(ValueError):clean_reference([fail,fail])


def test_extra_clean_failures_remain_in_resume_and_qualification(tmp_path):
    for index in range(1,5):(tmp_path/f'clean_{index:02}').mkdir()
    assert list(clean_indices(tmp_path))==[1,2,3,4]
    ok={'outcome':'route_completed','result_dir':'clean_04'}
    fail={'outcome':'collision','result_dir':'clean_01'}
    # A late success never erases three earlier failures or makes a stable pair.
    cell=[fail,fail,fail,ok]
    _,evidence=clean_reference(cell)
    assert evidence==dict(clean_successes=1,clean_flights=4,baseline_stable=False)
    assert not clean_gate([cell],'hard')
    assert clean_gate([[ok,ok],[ok,ok],[ok,fail],cell],'hard')
    with pytest.raises(ValueError):clean_indices(tmp_path,1)
