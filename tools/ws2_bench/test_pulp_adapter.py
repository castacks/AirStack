"""Third-target integration boundaries; no simulator/GPU needed."""
import math
import numpy as np
import pytest
from pathlib import Path
import importlib.util
from episode import resolved, worker_command
from dashboard import feedback_command, adaptive_command
from model_adapters import get_adapter, validate_attacks
from campaign import candidates
from expanded_space import bounds, reference_action, action_to_episode
from mission import defaults, agent_mission

policy_path=get_adapter('pulp_dronet').repository/'airstack_integration/policy.py'
policy=None
if policy_path.is_file():
    spec=importlib.util.spec_from_file_location('pulp_policy',policy_path)
    policy=importlib.util.module_from_spec(spec);spec.loader.exec_module(policy)


def test_third_target_preserves_reactive_contract_and_worker():
    c=resolved({'planner':'pulp_dronet'})
    assert c['mission_mode']=='avoidance'
    assert 'goal_distance' not in agent_mission('pulp_dronet',c)
    args=worker_command(c)
    assert 'pulp_dronet_airstack.py' in args and 'mononav_airstack.py' not in args
    assert args[args.index('--maximum-speed')+1]==str(c['maximum_speed'])
    assert '--depth-source' not in args
    assert get_adapter('pulp_dronet').frame_rate==10
    assert get_adapter('pulp_dronet').min_tracking_speed==.01
    assert get_adapter('mononav').min_tracking_speed==get_adapter('kim').min_tracking_speed==.01
    with pytest.raises(ValueError,match='depth estimator'):
        resolved({'planner':'pulp_dronet','diagnostic_depth_source':'ground-truth'})
    tiny=worker_command(resolved({'planner':'pulp_dronet','pulp_variant':'tiny'}))
    assert tiny[tiny.index('--variant')+1]=='tiny'
    with pytest.raises(ValueError,match='pulp_variant'):
        resolved({'planner':'pulp_dronet','pulp_variant':'other'})


def test_patch_is_blocked_through_all_entry_paths():
    assert bounds('pulp_dronet')['patch_enabled']==[False]
    with pytest.raises(ValueError,match='FCRN/Kim'):
        validate_attacks('pulp_dronet',{'patch_enabled':True})
    with pytest.raises(ValueError):candidates('grid',2,42,['pulp_dronet'],profile='patch')
    with pytest.raises(ValueError):feedback_command({'planner':'pulp_dronet','profile':'patch'},Path('/tmp/out'))
    a=reference_action();a['patch_enabled']=True
    with pytest.raises(ValueError):action_to_episode(a,'pulp_dronet',42,'bad',defaults('pulp_dronet'))


def test_dashboard_and_sampler_select_only_requested_target():
    args=feedback_command({'planner':'pulp_dronet'},Path('/tmp/out'))
    assert args[args.index('--planner')+1]=='pulp_dronet'
    args=adaptive_command({'planner':'pulp_dronet','action_space':'expanded','policy':'random'},Path('/tmp/out'))
    assert args[args.index('--planner')+1]=='pulp_dronet'
    assert all(c['planner']=='pulp_dronet' for c in candidates('random',4,42,['pulp_dronet']))
    matrix=candidates('random',6,42,['pulp_dronet','mononav','kim'],matrix=True)
    assert [c['planner'] for c in matrix]==['pulp_dronet','mononav','kim']
    assert 'input_fov_deg' not in matrix[1] and 'yaw_reference_mode' not in matrix[1]


@pytest.mark.skipif(policy is None,reason='Install the sibling pulp-dronet worker to check its numerical policy')
def test_left_turn_frame_convention():
    p=policy.Policy(.35,120,0)
    v,w=p.step(.5,.2)
    assert v==pytest.approx(.28) and w==pytest.approx(math.pi/3)
    pose=np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,1.2],[0,0,0,1]])
    points=policy.trajectory(pose,v,w)
    assert points[-1,1]>0 and points[-1,3]>0 # +camera Z is world +X, left is +Y
    assert np.all(points[:,2]==1.2)
    with pytest.raises(ValueError):p.step(float('nan'),.2)


@pytest.mark.skipif(policy is None,reason='Install the sibling pulp-dronet worker to check its numerical policy')
def test_yaw_rate_accumulates_in_sim_time_and_bounds_windup():
    ref=policy.HeadingReference()
    assert ref.update(0,.4,100)==0
    for i in range(1,11):yaw=ref.update(0,.4,100+i*.1)
    assert yaw==pytest.approx(.4)
    assert ref.update(0,.4,101)==pytest.approx(.4) # paused simulation: no extra rotation
    for i in range(11,100):yaw=ref.update(0,.4,100+i*.1)
    assert yaw==pytest.approx(math.pi/4)
