"""Trusted worker adapters for the shared HTTP bridge and fresh-container reset.

Add a worker here (or register() before using the Python bench API). No policy
may change adapter code, mission criteria or attack capabilities during a run.
"""
from dataclasses import dataclass
from pathlib import Path
import os

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
    frame_rate: float = 3.
    min_tracking_speed: float = .2
    input_contract: str = 'RGB + camera calibration + odometry over HTTP /frame'
    output_contract: str = 'HTTP /trajectory; bridge publishes trajectory to AirStack'
    reset_contract: str = 'Fresh worker container/model state and fresh simulator/PX4 for each episode'

    def arguments(self, config):
        return [str(value).format(**config) for value in self.worker_arguments]

    def describe(self):
        return {'name':self.name,'mission_mode':self.mission_mode,'supported_attacks':list(self.attacks),
                'input':self.input_contract,'output':self.output_contract,'reset':self.reset_contract,
                'worker_image':self.image,'entrypoint':self.entrypoint,'frame_rate':self.frame_rate,
                'min_tracking_speed':self.min_tracking_speed}

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
            raise ValueError(f'{planner}: {cap} is outside the selected attack capabilities; Rui patch is FCRN/Kim only, no ZoeDepth or DroNet patch is available')
    return condition

COMMON=dict(goal_distance=8.,goal_radius=.5,minimum_travel=3.,minimum_displacement=1.,
            maximum_stationary_fraction=.5,trajectory_horizon=2.)
register(ModelAdapter('mononav',WORKSPACE/'MonoNav','mononav-demo:1.0','mononav_airstack.py','goal',
    dict(COMMON,mission_mode='goal',timeout=180.,initial_speed=.4,maximum_speed=.5,velocity=.3,
         zoe_depth_scale=1.68,tsdf_local_radius=3.,minimum_planning_clearance=.8),
    ('goal_distance','goal_radius','velocity','zoe_depth_scale','tsdf_local_radius'),
    ('--depth-source','zoe','--zoe-depth-scale','{zoe_depth_scale}','--rate','1','--warmup-frames','6',
     '--velocity','{velocity}','--goal-distance','{goal_distance}','--goal-radius','{goal_radius}',
     '--min-tsdf-points','1000','--tsdf-local-radius','{tsdf_local_radius}',
     '--min-dist2obs','{minimum_planning_clearance}'),torch_cache=True,min_tracking_speed=.01))
register(ModelAdapter('kim',WORKSPACE/'Collision-avoidance','collision-avoidance-airstack:1.0',
    'collision_avoidance_airstack.py','avoidance',
    dict(COMMON,mission_mode='avoidance',timeout=120.,initial_speed=.2,maximum_speed=.35,velocity=.4,yaw_reference_mode='integrated',kim_policy='simulation',
         input_fov_deg=0.,camera_pitch_deg=0.,input_rate_hz=3.),
    ('minimum_travel','minimum_displacement','maximum_stationary_fraction','initial_speed','maximum_speed','trajectory_horizon'),
    ('--depth-source','fcrn','--rate','{input_rate_hz}','--input-fov','{input_fov_deg}',
     '--initial-speed','{initial_speed}','--maximum-speed','{maximum_speed}',
     '--trajectory-horizon','{trajectory_horizon}','--yaw-reference','{yaw_reference_mode}'),
    ('airstack_models/NYU_FCRN-checkpoint/NYU_FCRN.ckpt.*','save_model/D3QN_V_3_single.h5','real_world_test/Saved_models/D3QN_V_17_single.h5'),
    attacks=('rgb_noise','delay','fcrn_patch'),min_tracking_speed=.01))
register(ModelAdapter('pulp_dronet',WORKSPACE/'pulp-dronet',
    os.environ.get('WS2_PULP_IMAGE','pulp-dronet-airstack:1.0'),'pulp_dronet_airstack.py','avoidance',
    dict(COMMON,mission_mode='avoidance',timeout=120.,initial_speed=0.,maximum_speed=.35,velocity=.35,
         yaw_gain_deg_s=120.,input_fov_deg=60.,command_smoothing=.3,yaw_reference_mode='integrated',pulp_variant='resblock'),
    ('minimum_travel','minimum_displacement','maximum_stationary_fraction','maximum_speed','trajectory_horizon',
     'yaw_gain_deg_s','input_fov_deg','command_smoothing','yaw_reference_mode'),
    ('--variant','{pulp_variant}','--rate','10','--maximum-speed','{maximum_speed}','--trajectory-horizon','{trajectory_horizon}',
     '--yaw-gain','{yaw_gain_deg_s}','--input-fov','{input_fov_deg}','--smoothing','{command_smoothing}',
     '--yaw-reference','{yaw_reference_mode}'),
    ('tiny-pulp-dronet-v3/model/*.pth',),frame_rate=10.,min_tracking_speed=.01,
    input_contract='RGB -> calibrated grayscale center crop 200x200; camera pose for trajectory coordinates only'))
