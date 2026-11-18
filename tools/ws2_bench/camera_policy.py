"""Shared camera-program proposal space and result-guided baseline mutations."""
import copy
import random

from mission import SUCCESSES
import experiment_protocol as experiments

THREAT_MODEL = (
    'Camera-input threat model: you may compose up to eight live RGB noise, added-delay, '
    'frame-drop and frame-freeze windows, with numeric parameters and time/observed-turn triggers. '
    'You may design feasible corridor geometry, obstacle positions/counts/sizes/orientations and lighting. '
    'For Kim/FCRN only, you may also place/size/time the original opaque Rui patch. '
    'No historical-frame replay, IMU/GPS/odometry mutation, arbitrary code, model/controller edits, '
    'ground-truth corruption, scoring changes or direct flight commands. Odometry yaw rate is '
    'read-only trigger information, not a perturbation target. A freeze stops updates; timestamps '
    'remain original and the fixed target adapter may stop on stale input. Report that behavior. '
    'Objective: discover distinct, reproducible vulnerability conditions efficiently within the '
    'shared flight budget. Do not assume or manufacture superiority over Random/Search. '
    'Before each test state the hypothesis, why this attack/trigger is informative, what would '
    'support/refute it, and how much budget remains for confirmation and factor isolation. '
    'The next validated test executes automatically; inspect evidence and revise the plan afterward. '
    'Clean and attack both run in record mode, even if the clean fails; that pair cannot establish '
    'an attack effect. Missing trigger/exposure/consumer evidence is unknown, not a successful attack. '
    'Keep defense implementation for a later study. A future comparison must use identical action '
    'bounds, prior evidence, flight limits and scoring, including all confirmation/diagnosis flights. '
)


def sample(rng, target):
    from corridor_space import CAMERA_VERSION, validate_action
    from baseline_proposals_v2 import Proposer
    # Reuse the full-support feasible geometry/patch prior, not the old narrow
    # lateral-position sampler. Rejection does not imply uniform feasible maps.
    a=Proposer('random',rng.randrange(2**31),target).propose([],0)['action']
    a.update(action_space_version=CAMERA_VERSION,frame_drop_probability=0.,frame_freeze_enabled=False,
             rgb_noise_stddev=0.,delay_s=0.,camera_program=[])
    for k in ('noise_schedule','delay_schedule'):a[k]=dict(start_s=0.,duration_s=0.,period_s=0.)
    factors=[] if rng.random()<.05 else rng.sample(['rgb_noise','delay','frame_drop','frame_freeze'],rng.randint(1,4))
    for factor in factors:
        if factor=='rgb_noise':a['rgb_noise_stddev']=rng.uniform(.000001,32.)
        elif factor=='delay':a['delay_s']=rng.uniform(.000001,.5)
        elif factor=='frame_drop':a['frame_drop_probability']=rng.uniform(.000001,.8)
        else:a['frame_freeze_enabled']=True
    events=factors+[rng.choice(factors) for _ in range(rng.randint(0,8-len(factors)))] if factors else []
    for factor in events:
        trigger=rng.choice(['mission_time','first_turn'])
        maximum=15. if factor in ('frame_drop','frame_freeze') else 500.
        duration=rng.uniform(.1,maximum if rng.random()<.15 else min(30.,maximum))
        period=0. if trigger=='first_turn' or rng.random()<.5 or duration>=500 else rng.uniform(duration+.000001,500.)
        a['camera_program'].append(dict(factor=factor,trigger=trigger,
            start_s=rng.uniform(0.,500. if rng.random()<.15 else 200.),duration_s=duration,period_s=period,
            turn_rate_rad_s=rng.uniform(.02,1.5)))
    return validate_action(a,target)


def baseline_action(name, seed, history, target):
    from corridor_space import action_key, experiment, validate_action
    rng=random.Random(seed+1009*len(history))
    # Confirmation uses the same paid clean+attack pair as LLM confirmation.
    for r in reversed(history):
        a=r['decision']['action'];pair=r['pairs'][0]
        if experiments.attack_evidence_eligible(a,pair) and pair['perturbed']['outcome'] not in SUCCESSES:
            count=sum(action_key(h['decision']['action'])==action_key(a) for h in history)
            if count<2:return copy.deepcopy(a),experiment('repeat',r['decision']['round'])
    a=sample(rng,target)
    if name=='search' and history and rng.random()<.7:
        parent=rng.choice(history[-4:]);candidate=copy.deepcopy(parent['decision']['action'])
        if parent['pairs'][0]['clean']['outcome'] in SUCCESSES and candidate['camera_program']:
            event=rng.choice(candidate['camera_program']); key=rng.choice(['start_s','duration_s','turn_rate_rad_s'])
            if key=='start_s':event[key]=max(0.,min(500.,event[key]+rng.uniform(-15.,15.)))
            elif key=='turn_rate_rad_s':event[key]=rng.uniform(.02,.5)
            else:
                ceiling=15. if event['factor'] in ('frame_drop','frame_freeze') else 500.
                if event['period_s']:ceiling=min(ceiling,event['period_s']-.01)
                event[key]=max(.1,min(ceiling,event[key]*rng.uniform(.5,1.5)))
            a=validate_action(candidate,target)
    if action_key(a) in {action_key(h['decision']['action']) for h in history}:a=sample(rng,target)
    return a,experiment()
