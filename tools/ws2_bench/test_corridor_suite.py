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
    g=courses[0]
    assert len(g['obstacles'])==(2 if shape=='u_obstacles' else 0)+{'easy':0,'medium':2,'hard':5}[tier]
    if tier!='easy':
        # Encounters must happen well after takeoff and before the finish.
        anchors=sorted(b['route_s'] for b in g['obstacles'])
        assert anchors[0]>5 and g['route_length_m']-anchors[-1]>5
        assert all(b-a>3.4 for a,b in zip(anchors,anchors[1:]))


@pytest.mark.parametrize('shape',SHAPES)
def test_medium_moves_encounters_inward_and_hard_preserves_them(shape):
    maps=[geometry(tier_spec(shape,t)) for t in TIERS]
    for a,b in zip(maps,maps[1:]):
        assert a['route']==b['route'] and a['walls']==b['walls']
    assert maps[2]['obstacles'][:len(maps[1]['obstacles'])]==maps[1]['obstacles']
    assert len(maps[2]['obstacles'])==len(maps[1]['obstacles'])+3
    assert all(m['spec']['style']=='hallway_daylight' for m in maps)


@pytest.mark.parametrize('shape',SHAPES)
def test_legacy_saved_layouts_replay_without_reinterpreting_coordinates(shape):
    for tier in TIERS:
        old=tier_spec(shape,tier);old.pop('layout_version')
        old['obstacles']=(2 if shape=='u_obstacles' else 0)+2*TIERS.index(tier)
        g=geometry(old)
        if tier=='easy':assert g['obstacles']==geometry(tier_spec(shape,tier))['obstacles']
        elif shape=='u_obstacles':assert [b['route_s'] for b in g['obstacles'][:2]]==[2.5,4.5]


@pytest.mark.parametrize('shape',SHAPES)
def test_new_blocks_leave_a_collision_free_center_route(shape):
    from corridors import point_at
    # Circle swept along the scoring route, using each actual rotated box.
    g=geometry(tier_spec(shape,'hard'))
    for n in range(math.ceil(g['route_length_m']*20)+1):
        xy,_=point_at(g['route'],n/20)
        for b in g['obstacles']:
            dx,dy=xy[0]-b['center'][0],xy[1]-b['center'][1];a=b['yaw']
            local=(dx*math.cos(a)+dy*math.sin(a),-dx*math.sin(a)+dy*math.cos(a))
            clearance=math.hypot(*[max(0,abs(local[i])-b['size'][i]/2) for i in range(2)])
            assert clearance>.65


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
