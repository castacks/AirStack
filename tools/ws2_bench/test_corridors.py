"""Check corridor traversal and target interfaces without Isaac."""
import pytest
import math
from corridors import geometry,Traversal,point_at,SHAPES
from episode import resolved,worker_command
from conditions import validate


def test_geometry_hash_tolerates_libm_roundoff_but_detects_real_changes():
    from corridors import geometry_hash
    import copy
    a=geometry({'shape':'curve','obstacles':0});b=copy.deepcopy(a)
    b['route_length_m']+=2e-15
    assert geometry_hash(a)==geometry_hash(b)
    b['walls'][0]['center'][0]+=.001
    assert geometry_hash(a)!=geometry_hash(b)


@pytest.mark.parametrize('shape',SHAPES)
def test_complete_traversal_requires_ordered_turns(shape):
    g=geometry(dict(shape=shape,width=3.5,obstacles=2),42)
    t=Traversal(g)
    assert not t.update([*g['route'][-1],1.2]) # teleport to endpoint cannot pass skipped gates
    t=Traversal(g)
    for i in range(int(g['route_length_m']*10)+1):t.update([*point_at(g['route'],i/10)[0],1.2])
    assert t.metrics()['route_completed']
    assert g==geometry(dict(shape=shape,width=3.5,obstacles=2),42)
    assert len(g['obstacles'])==2


def test_task_goals_are_only_supplied_to_goal_based_worker():
    condition={'layout':'corridor','corridor':{'shape':'elbow'}}
    for model in ['mononav','kim','pulp_dronet']:
        c=resolved({'planner':model,'condition':condition})
        assert ('--mission-goals' in worker_command(c))==(model=='mononav')
    with pytest.raises(ValueError):validate({'layout':'stock','corridor':{'shape':'elbow'}})
    with pytest.raises(ValueError):validate({'layout':'corridor','corridor':{'width':1}})
    with pytest.raises(ValueError):resolved({'planner':'pulp_dronet','condition':{**condition,'patch_enabled':True}})


def test_attack_twins_keep_geometry_and_contact_surfaces():
    from generated_layouts import realized
    clean=validate({'layout':'corridor','layout_seed':42,'corridor':{'shape':'u_obstacles','obstacles':4}})
    noisy=validate(dict(clean,rgb_noise=32,delay=.5))
    assert realized(clean)==realized(noisy)


def test_bare_bent_course_and_low_partition_patch_support():
    bare=validate({'layout':'corridor','corridor':{'shape':'elbow','obstacles':0}})
    assert geometry(bare['corridor'])['obstacles']==[]
    with pytest.raises(ValueError,match='obstacle surface'):
        validate(dict(bare,patch_enabled=True))
    low=geometry({'shape':'u_obstacles','style':'paper_lab_low','obstacles':4})
    assert all(w['size'][2]==1.2 for w in low['walls'])
    assert all(b['size'][2]==1.0 for b in low['obstacles'])


def test_bevel_changes_real_wall_geometry_without_changing_scoring_route():
    plain=geometry({'shape':'elbow','width':3.5})
    bevel=geometry({'shape':'elbow','width':3.5,'corner_bevel':1.})
    assert len(bevel['walls'])==len(plain['walls'])+1
    assert bevel['route']==plain['route'] and bevel['gates']==plain['gates']
    assert bevel['placement_id']!=plain['placement_id']
    with pytest.raises(ValueError):geometry({'shape':'curve','corner_bevel':1.})


def test_finish_is_a_cross_section_without_a_hidden_centerline_goal():
    g=geometry(dict(shape='elbow'),42);t=Traversal(g)
    for i in range(140):t.update([*point_at(g['route'],i/10)[0],1.2])
    assert not t.update([4.,7.4,1.2])
    assert t.update([4.,7.6,1.2])
    assert t.metrics()['route_scoring_version']==2


def test_stall_uses_simulation_time_and_forward_route_progress():
    t=Traversal(geometry({'shape':'elbow'},42))
    t.update([-4,0,1.2],10)
    assert not t.stalled(10,60) # paused simulation
    t.update([-3.5,0,1.2],30)
    assert not t.stalled(89,60)
    t.update([-4,0,1.2],90) # backward movement does not reset progress window
    assert t.stalled(90,60)


@pytest.mark.parametrize('shape',['elbow','zigzag','u_obstacles'])
def test_rounded_bends_are_real_geometry_with_ordered_scoring(shape):
    g=geometry({'shape':shape,'width':4.,'bend_radius':2.5,'obstacles':4})
    assert len(g['route'])>10
    t=Traversal(g)
    for i in range(math.ceil(g['route_length_m']*10)+1):
        t.update([*point_at(g['route'],i/10)[0],1.2])
    assert t.metrics()['route_completed']
    assert all(w['size'][0]>.04 for w in g['walls'])
    with pytest.raises(ValueError):geometry({'shape':shape,'width':5.,'bend_radius':2.})


def test_broad_bends_need_corresponding_physical_straight_length():
    with pytest.raises(ValueError,match='no straight section'):
        geometry({'shape':'u_obstacles','width':4.,'bend_radius':4.})
    g=geometry({'shape':'u_obstacles','width':4.,'bend_radius':4.,'route_scale':1.5,'obstacles':4})
    assert g['route'][0]==[-4.,0.]
    assert g['route'][-1]==[-4.,9.]
    assert 29<g['route_length_m']<30 and len(g['obstacles'])==4
