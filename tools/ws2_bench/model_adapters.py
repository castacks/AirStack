"""Trusted worker adapters for the shared HTTP bridge and fresh-container reset.

Add a worker here (or register() before using the Python bench API). No policy
may change adapter code, mission criteria or attack capabilities during a run.
"""
from dataclasses import dataclass
from pathlib import Path

WORKSPACE=Path(__file__).resolve().parents[3]

@dataclass(frozen=True)
class ModelAdapter:
    name: str
    repository: Path
    image: str
    entrypoint: str
    mission_mode: str
    mission_defaults: dict
    mission_fields: tuple
    worker_arguments: tuple
    weight_patterns: tuple = ()
    torch_cache: bool = False
    attacks: tuple = ('rgb_noise','delay')
    input_contract: str = 'RGB + camera calibration + odometry over HTTP /frame'
    output_contract: str = 'HTTP /trajectory; bridge publishes trajectory to AirStack'
    reset_contract: str = 'Fresh worker container/model state and fresh simulator/PX4 for each episode'

    def arguments(self, config):
        return [str(value).format(**config) for value in self.worker_arguments]

    def describe(self):
        return {'name':self.name,'mission_mode':self.mission_mode,'supported_attacks':list(self.attacks),
                'input':self.input_contract,'output':self.output_contract,'reset':self.reset_contract,
                'worker_image':self.image,'entrypoint':self.entrypoint}

REGISTRY={}

def register(adapter):
    if not isinstance(adapter,ModelAdapter) or adapter.name in REGISTRY:
        raise ValueError('Register a unique ModelAdapter')
    if adapter.mission_mode not in ('goal','avoidance'):
        raise ValueError('Supported evaluation contracts are goal and avoidance')
    if adapter.mission_defaults.get('mission_mode')!=adapter.mission_mode:
        raise ValueError('Mission mode differs from adapter defaults')
    if set(adapter.attacks)-{'rgb_noise','delay','fcrn_patch'}:
        raise ValueError('Unknown attack capability')
    REGISTRY[adapter.name]=adapter

def get_adapter(name):
    try:return REGISTRY[name]
    except KeyError:raise ValueError('Unknown model adapter: '+str(name)) from None

def validate_attacks(planner, condition, allow_patch=True):
    capabilities=get_adapter(planner).attacks
    for key,cap in [('rgb_noise','rgb_noise'),('delay','delay'),('patch_enabled','fcrn_patch')]:
        if condition.get(key) and (cap not in capabilities or (cap=='fcrn_patch' and not allow_patch)):
            raise ValueError(f'{planner}: {cap} is outside the selected attack capabilities; no ZoeDepth patch is available')
    return condition

COMMON=dict(goal_distance=8.,goal_radius=.5,minimum_travel=3.,minimum_displacement=1.,
            maximum_stationary_fraction=.5,trajectory_horizon=2.)
register(ModelAdapter('mononav',WORKSPACE/'MonoNav','mononav-demo:1.0','mononav_airstack.py','goal',
    dict(COMMON,mission_mode='goal',timeout=180.,initial_speed=.4,maximum_speed=.5,velocity=.3),
    ('goal_distance','goal_radius','velocity'),
    ('--depth-source','zoe','--zoe-depth-scale','1.68','--rate','1','--warmup-frames','6',
     '--velocity','{velocity}','--goal-distance','{goal_distance}','--goal-radius','{goal_radius}',
     '--min-tsdf-points','1000','--tsdf-local-radius','8'),torch_cache=True))
register(ModelAdapter('kim',WORKSPACE/'Collision-avoidance','collision-avoidance-airstack:1.0',
    'collision_avoidance_airstack.py','avoidance',
    dict(COMMON,mission_mode='avoidance',timeout=120.,initial_speed=.2,maximum_speed=.35,velocity=.4),
    ('minimum_travel','minimum_displacement','maximum_stationary_fraction','initial_speed','maximum_speed','trajectory_horizon'),
    ('--depth-source','fcrn','--rate','3','--initial-speed','{initial_speed}','--maximum-speed','{maximum_speed}',
     '--trajectory-horizon','{trajectory_horizon}'),
    ('airstack_models/NYU_FCRN-checkpoint/NYU_FCRN.ckpt.*','save_model/D3QN_V_3_single.h5'),
    attacks=('rgb_noise','delay','fcrn_patch')))
