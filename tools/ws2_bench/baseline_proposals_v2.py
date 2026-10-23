"""Inactive full-support Random and elite-mutation Search discovery proposals.

No runner imports this module yet. It changes neither execution nor evaluation.
The common prior is a documented proposal distribution, not uniform sampling of
feasible scenes. Partial-geometry rejection preserves support while avoiding
exponentially unlikely all-at-once valid seven-obstacle draws. Every final action
uses the existing validator. Exhaustion is explicit, never an easier fallback.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import random
from dataclasses import asdict, dataclass

from conditions import SCHEDULE_LIMITS
from corridor_space import (COUNTS, LIMITS, VERSION as ACTION_VERSION, action_key,
                            reference_action, validate_action)
from corridors import SHAPES, OBSTACLE_LIMITS, geometry, tier_spec
from factor_exposure import assess as assess_exposure
from mission import SUCCESSES

VERSION = 'full_support_discovery_v1'
RANKING_VERSION = 'clearance_progress_risk_v1'
GROUPS = ('geometry', 'attack', 'timing')
RISK_COMPONENTS = {'progress_loss': ('route_progress_percent', 100.),
                   'clearance_loss': ('minimum_obstacle_clearance_m', .25)}


@dataclass(frozen=True)
class Prior:
    """Freeze and provide this same prior to every compared policy, including LLM."""
    useful_time_horizon_s: float = 200.
    full_time_probability: float = .15
    all_off_probability: float = .05
    boundary_probability: float = .01  # Each endpoint; 98% continuous density.
    restart_probability: float = .25
    elite_size: int = 3
    max_scene_attempts: int = 32
    max_obstacle_attempts: int = 256
    max_patch_attempts: int = 128
    max_mutation_attempts: int = 96

    def __post_init__(self):
        if (type(self.useful_time_horizon_s) not in (int,float)
                or not math.isfinite(self.useful_time_horizon_s)
                or not 0 < self.useful_time_horizon_s <= 500):
            raise ValueError('Useful-time horizon must be a shared finite bound in (0,500]')
        for key in ('full_time_probability','all_off_probability','boundary_probability','restart_probability'):
            value=getattr(self,key)
            if type(value) not in (int,float) or not math.isfinite(value) or not 0 < value < 1:
                raise ValueError(key+' must be in (0,1)')
        if self.boundary_probability >= .5:raise ValueError('Continuous density must remain positive')
        for key in ('elite_size','max_scene_attempts','max_obstacle_attempts','max_patch_attempts','max_mutation_attempts'):
            if type(getattr(self,key)) is not int or getattr(self,key) < 1:raise ValueError(key+' must be positive')


def distribution(prior=None):
    p=prior or Prior()
    return dict(version=VERSION,action_space_version=ACTION_VERSION,parameters=asdict(p),
        discrete='Uniform shapes, tiers and allowed integer count within tier; full 31-bit layout seed. Kim includes every sensor/patch factor subset.',
        factors='Default5% all-off, otherwise uniform nonempty allowed factor subsets; configured probability is in parameters. Scalars span complete validator bounds.',
        scalar='Default1% lower and1% upper endpoint atoms; configured probability is in parameters, remainder continuous uniform. Physical invalid proposals are rejected.',
        schedules='35% whole mission;15% delayed remainder;30% finite one-shot;20% periodic. By default15% of non-whole schedules use full0..500s support (configured probability in parameters); other finite windows last2..30s and pulses last2..20s with gaps2..30s. Starts use shared useful-time horizon, default200s.',
        patch='Original Kim-only opaque patch. Every physical obstacle and front/inner face; size0.3..0.9m, mount height0.5..2m, start0..120s and duration0..120s. Start0 mass50%, duration0 mass35%; complete remaining ranges retain support.',
        geometry='Independent full-range coordinates with sequential existing-validator rejection. Bounded restarts and rejections change accepted density; this is NOT uniform over feasible layouts.',
        search='Default25% global restart, otherwise uniform top3 eligible distinct incumbent mutation; configured restart probability and elite count are in parameters. One group: geometry/light/seed, attacks, timing. Rank observed failure first, then the documented measured risk proxy; seeded tie break. Caller may require a local attempt for the first post-initialization discovery; no eligible incumbent still restarts. No shared follow-up queue.',
        search_ranking=dict(version=RANKING_VERSION,
            formula='Sum of observed component means: max(0, clean progress - attack progress)/100 plus max(0, clean minimum clearance - attack minimum clearance)/0.25m.',
            aggregation='Within each distinct setting, average each component over its eligible measured pairs. Sum the available means; absent components are null, not zero observations. Partial coverage and valid/missing sample counts are reported. No measured components ranks last within its failure class.',
            interpretation='A normalized search proxy, not a failure threshold, delivered attack dose, causal attribution or statistical significance. Positive clearance loss can guide search when both flights complete; measured improvements contribute zero risk.'),
        timing_limitation='Long windows may miss the flown interval; record exposure failures rather than resampling based on unseen flight outcomes.',
        duplicate_rule='corridor_space.action_key ignores inactive patch size/mount and schedules of zero noise/delay; raw actions are retained. layout_seed, obstacle order and tier remain because their equivalence has not been established.',
        fairness='Freeze equal supplied history/evidence, prior, validator, action support and reserved flight budget. This generator consumes no flight slots and schedules no repeats; confirmation/diagnosis allocation is a separate common protocol.')


class ProposalExhausted(RuntimeError):
    def __init__(self, message, audit):
        super().__init__(message); self.audit=copy.deepcopy(audit)


def proposal_key(action):
    """Known active-setting equivalence only; raw saved actions stay untouched."""
    return action_key(action)


def _raw_key(action):
    return json.dumps(action,sort_keys=True,separators=(',',':'),allow_nan=False)


def _rng(seed,index,label):
    if type(seed) is not int or type(index) is not int or index < 0:
        raise ValueError('Seed must be integer and proposal index nonnegative integer')
    digest=hashlib.sha256(json.dumps([VERSION,seed,index,label],separators=(',',':')).encode()).digest()
    return random.Random(int.from_bytes(digest,'big'))


def _draw(rng,lo,hi,p):
    if lo==hi:return float(lo)
    u=rng.random()
    if u<p.boundary_probability:return float(lo)
    if u<2*p.boundary_probability:return float(hi)
    return rng.uniform(lo,hi)


def _positive(rng,hi,p):
    value=_draw(rng,0.,hi,p)
    # The lower endpoint atom denotes "off" elsewhere. Redistribute it over
    # the positive interval here instead of creating an ineffective tiny atom.
    return value if value>0 else max(math.nextafter(0.,1.),rng.uniform(0.,hi))


def _schedule(rng,p,horizon=None):
    mode=rng.random();maximum=SCHEDULE_LIMITS['start_s'][1]
    if mode < .35:return dict(start_s=0.,duration_s=0.,period_s=0.)
    full=rng.random()<p.full_time_probability
    horizon=maximum if full else (p.useful_time_horizon_s if horizon is None else horizon)
    start=_draw(rng,0.,horizon,p)
    if mode < .50:return dict(start_s=start,duration_s=0.,period_s=0.)
    if mode < .80:
        duration=_positive(rng,maximum,p) if full else _draw(rng,2.,30.,p)
        return dict(start_s=start,duration_s=duration,period_s=0.)
    if full:
        period=_positive(rng,maximum,p)
        # Keep positive duration strictly below period, including floating edges.
        if period<=math.nextafter(0.,1.):period=math.nextafter(period,math.inf)
        duration=max(math.nextafter(0.,1.),min(math.nextafter(period,0.),rng.random()*period))
    else:
        duration=_draw(rng,2.,20.,p);period=duration+_draw(rng,2.,30.,p)
    return dict(start_s=start,duration_s=duration,period_s=period)


def _reject(audit,stage,reason):
    key=stage+': '+str(reason)
    audit['rejections'][key]=audit['rejections'].get(key,0)+1


def _valid(a,target,audit,stage):
    audit['validator_calls']+=1
    try:return validate_action(a,target)
    except ValueError as exc:_reject(audit,stage,exc);return None


def _duplicate(a,excluded,exact_excluded,audit,stage):
    if proposal_key(a) not in excluded:return False
    kind='exact' if _raw_key(a) in exact_excluded else 'inactive_setting_equivalent'
    audit['duplicate_rejections'][kind]+=1
    _reject(audit,stage,'duplicate action')
    return True


def _partial_tier(count):
    return next(tier for tier,(lo,hi) in COUNTS.items() if lo<=count<=hi)


def _obstacle(rng,shape,p):
    length=geometry(tier_spec(shape,'easy'))['route_length_m']
    return dict(route_s=_draw(rng,4.,length-4.,p),
                **{key:_draw(rng,*bounds,p) for key,bounds in OBSTACLE_LIMITS.items()})


def _fill_obstacles(a,count,rng,target,p,audit):
    a['obstacles']=a['obstacles'][:count]
    for _ in range(len(a['obstacles']),count):
        for _attempt in range(p.max_obstacle_attempts):
            b=copy.deepcopy(a);b['obstacles'].append(_obstacle(rng,a['shape'],p))
            b['difficulty']=_partial_tier(len(b['obstacles']))
            # A partial prefix cannot yet supply a future patch mount.
            b.update(patch_enabled=False,patch_mount=None,patch_start_s=0.,patch_duration_s=0.)
            if _valid(b,target,audit,'obstacle') is not None:
                a['obstacles']=b['obstacles'];break
        else:return False
    return True


def _patch(rng,a,p):
    a['patch_size_m']=_draw(rng,*LIMITS['patch_size_m'],p)
    a['patch_start_s']=0. if rng.random()<.5 else _draw(rng,*LIMITS['patch_start_s'],p)
    a['patch_duration_s']=0. if rng.random()<.35 else _draw(rng,*LIMITS['patch_duration_s'],p)
    a['patch_mount']=dict(obstacle_index=rng.randrange(len(a['obstacles'])),
                          face=rng.choice(('front','inner')),height_m=_draw(rng,.5,2.,p))


def _global(rng,target,p,excluded,exact_excluded,audit):
    for _ in range(p.max_scene_attempts):
        audit['scene_attempts']+=1
        a=reference_action(ACTION_VERSION)
        a.update(shape=rng.choice(SHAPES),difficulty=rng.choice(tuple(COUNTS)),layout_seed=rng.randrange(2**31),
                 light_intensity=_draw(rng,*LIMITS['light_intensity'],p))
        count=rng.randint(*COUNTS[a['difficulty']])
        if not _fill_obstacles(a,count,rng,target,p,audit):
            _reject(audit,'scene','bounded obstacle construction exhausted');continue
        factor_count=3 if target=='kim' else 2
        mask=0 if rng.random()<p.all_off_probability else rng.randrange(1,2**factor_count)
        a['rgb_noise_stddev']=_positive(rng,LIMITS['rgb_noise_stddev'][1],p) if mask&1 else 0.
        a['delay_s']=_positive(rng,LIMITS['delay_s'][1],p) if mask&2 else 0.
        for key in ('noise_schedule','delay_schedule'):a[key]=_schedule(rng,p)
        a['patch_enabled']=bool(mask&4)
        if a['patch_enabled']:
            if not count:_reject(audit,'patch','no physical obstacle');continue
            for _attempt in range(p.max_patch_attempts):
                _patch(rng,a,p)
                valid=_valid(a,target,audit,'patch')
                if valid is not None:break
            else:_reject(audit,'scene','bounded physical patch sampling exhausted');continue
        else:valid=_valid(a,target,audit,'final')
        if valid is None:continue
        if _duplicate(valid,excluded,exact_excluded,audit,'final'):continue
        return valid
    raise ProposalExhausted('Full-support proposal budget exhausted',audit)


def _loss(pair,metric):
    values=[]
    for role in ('clean','perturbed'):
        metrics=pair[role].get('metrics')
        value=metrics.get(metric) if isinstance(metrics,dict) else None
        if type(value) not in (int,float) or not math.isfinite(value):return None
        if metric=='route_progress_percent' and not 0<=value<=100:return None
        values.append(value)
    difference=values[0]-values[1]
    return difference if math.isfinite(difference) else None


def _elites(history,target,seed,index,p):
    groups={}
    for row in history:
        a=row.get('decision',{}).get('action')
        try:
            a=validate_action(a,target)
            if a.get('action_space_version')!=ACTION_VERSION:continue
        except ValueError:continue
        for pair in row.get('pairs',[]):
            if not assess_exposure(a,pair)['eligible']:continue
            key=proposal_key(a)
            group=groups.setdefault(key,dict(action=a,source_rounds=[],failures=0,eligible_pairs=0,
                                            losses={name:[] for name in RISK_COMPONENTS}))
            group['source_rounds'].append(row.get('decision',{}).get('round'))
            group['failures']+=int(pair['perturbed']['outcome'] not in SUCCESSES)
            group['eligible_pairs']+=1
            for name,(metric,_) in RISK_COMPONENTS.items():
                loss=_loss(pair,metric)
                if loss is not None:group['losses'][name].append(loss)
    for key,group in groups.items():
        components={}
        for name,(metric,scale) in RISK_COMPONENTS.items():
            losses=group['losses'][name]
            components[name]=dict(metric=metric,normalization_scale=scale,
                paired_observations=len(losses),missing_or_invalid_pairs=group['eligible_pairs']-len(losses),
                mean_signed_loss=sum(losses)/len(losses) if losses else None,
                mean_positive_normalized_loss=sum(max(0.,v)/scale for v in losses)/len(losses) if losses else None)
        observed=[v['mean_positive_normalized_loss'] for v in components.values()
                  if v['mean_positive_normalized_loss'] is not None]
        group.update(ranking_version=RANKING_VERSION,risk_components=components,
                     normalized_risk_proxy=sum(observed) if observed else None,
                     risk_metric_coverage='complete' if all(v['missing_or_invalid_pairs']==0 for v in components.values())
                                          else 'partial' if observed else 'unavailable',
                     mean_progress_loss=components['progress_loss']['mean_signed_loss'],
                     mean_clearance_loss_m=components['clearance_loss']['mean_signed_loss'])
        group['tie_break']=_rng(seed,index,key).random()
    ranked=sorted(groups.values(),key=lambda g:(g['failures']>0,
        -math.inf if g['normalized_risk_proxy'] is None else g['normalized_risk_proxy'],g['tie_break']),reverse=True)
    return ranked[:p.elite_size]


def _mutate_scalar(rng,value,lo,hi,p):
    # Reflected local moves preserve bounds without piling up clipped endpoints.
    if rng.random()<.2:return _draw(rng,lo,hi,p)
    scale=.10 if rng.random()<.75 else .25
    position=(value-lo+rng.gauss(0.,scale*(hi-lo)))%(2*(hi-lo))
    return lo+(position if position<=hi-lo else 2*(hi-lo)-position)


def _mutate(a,group,rng,target,p,audit):
    b=copy.deepcopy(a);detail={}
    if group=='geometry':
        choice=rng.choice(('shape','count','obstacle','light','seed'))
        detail['coordinate']=choice
        if choice=='shape':b['shape']=rng.choice(SHAPES)
        elif choice=='count':
            b['difficulty']=rng.choice(tuple(COUNTS));count=rng.randint(*COUNTS[b['difficulty']])
            if not _fill_obstacles(b,count,rng,target,p,audit):return None,detail
        elif choice=='obstacle':
            if not b['obstacles']:return None,detail
            index=rng.randrange(len(b['obstacles']));key=rng.choice(('route_s',*OBSTACLE_LIMITS))
            bounds=(4.,geometry(tier_spec(b['shape'],'easy'))['route_length_m']-4.) if key=='route_s' else OBSTACLE_LIMITS[key]
            b['obstacles'][index][key]=_mutate_scalar(rng,b['obstacles'][index][key],*bounds,p)
            detail.update(obstacle_index=index,field=key)
        elif choice=='light':b['light_intensity']=_mutate_scalar(rng,b['light_intensity'],*LIMITS['light_intensity'],p)
        else:b['layout_seed']=rng.randrange(2**31)
    elif group=='attack':
        key=rng.choice(('rgb_noise_stddev','delay_s',*(('patch_enabled','patch_size_m','patch_mount') if target=='kim' else ())))
        detail['coordinate']=key
        if key in ('rgb_noise_stddev','delay_s'):
            b[key]=0. if rng.random()<.15 else _mutate_scalar(rng,b[key],*LIMITS[key],p)
        elif key=='patch_enabled':
            b['patch_enabled']=not b['patch_enabled']
            if b['patch_enabled']:
                if not b['obstacles']:return None,detail
                _patch(rng,b,p)
            else:b.update(patch_mount=None,patch_start_s=0.,patch_duration_s=0.)
        elif key=='patch_size_m':
            if not b['patch_enabled']:return None,detail
            b[key]=_mutate_scalar(rng,b[key],*LIMITS[key],p)
        else:
            if not b['patch_enabled'] or not b['obstacles']:return None,detail
            b['patch_mount']=dict(obstacle_index=rng.randrange(len(b['obstacles'])),face=rng.choice(('front','inner')),
                                  height_m=_draw(rng,.5,2.,p))
    else:
        active=[key for key,scalar in (('noise_schedule','rgb_noise_stddev'),('delay_schedule','delay_s')) if b[scalar]>0]
        if b['patch_enabled']:active.append('patch_timing')
        if not active:return None,detail
        key=rng.choice(active);detail['coordinate']=key
        if key=='patch_timing':
            field=rng.choice(('patch_start_s','patch_duration_s'));detail['field']=field
            b[field]=_mutate_scalar(rng,b[field],*LIMITS[field],p)
        elif rng.random()<.5:b[key]=_schedule(rng,p)
        else:
            field=rng.choice(tuple(SCHEDULE_LIMITS));detail['field']=field
            b[key][field]=_mutate_scalar(rng,b[key][field],*SCHEDULE_LIMITS[field],p)
    return b,detail


class Proposer:
    """Stateless discovery API: same seed/index/history => identical proposal.

    The caller owns the common initial proposals and flight-budget protocol.
    Random ignores outcomes; its exclusion set depends only on saved actions.
    Search never invokes baseline_followup, an LLM, or a simulator.
    """
    def __init__(self,policy,seed,target,prior=None):
        if policy not in ('random','search'):raise ValueError('Expected random or search')
        if target not in ('mononav','kim','pulp_dronet'):raise ValueError('Unsupported target')
        if type(seed) is not int:raise ValueError('Seed must be an integer')
        if prior is not None and not isinstance(prior,Prior):raise ValueError('prior must be a frozen Prior')
        self.policy,self.seed,self.target,self.prior=policy,seed,target,prior or Prior()

    def propose(self,history,proposal_index,*,require_local_attempt=False):
        if type(require_local_attempt) is not bool:
            raise ValueError('require_local_attempt must be Boolean')
        if require_local_attempt and self.policy!='search':
            raise ValueError('Only Search can require a local mutation attempt')
        rng=_rng(self.seed,proposal_index,'proposal');p=self.prior
        audit=dict(version=VERSION,policy=self.policy,seed=self.seed,proposal_index=proposal_index,
                   prior=asdict(p),source=None,validator_calls=0,scene_attempts=0,mutation_attempts=0,rejections={},
                   duplicate_rejections=dict(exact=0,inactive_setting_equivalent=0),
                   require_local_attempt=require_local_attempt,
                   validation_count_scope='Candidate/partial checks only; identity canonicalization validates additionally.')
        excluded=set();exact_excluded=set()
        for row in history:
            try:
                excluded.add(proposal_key(row['decision']['action']))
                exact_excluded.add(_raw_key(row['decision']['action']))
            except (KeyError,ValueError,TypeError):continue
        elites=_elites(history,self.target,self.seed,proposal_index,p) if self.policy=='search' else []
        audit['eligible_elite_count']=len(elites)
        branch=rng.random() if self.policy=='search' else None
        if elites and (require_local_attempt or branch>=p.restart_probability):
            parent=rng.choice(elites);group=rng.choice(GROUPS)
            audit.update(source='elite_mutation',mutation_group=group,parent_rounds=parent['source_rounds'],
                         parent_score=dict(observed_failures=parent['failures'],eligible_pairs=parent['eligible_pairs'],
                                           mean_progress_loss=parent['mean_progress_loss'],
                                           mean_clearance_loss_m=parent['mean_clearance_loss_m'],
                                           ranking_version=RANKING_VERSION,
                                           normalized_risk_proxy=parent['normalized_risk_proxy'],
                                           metric_coverage=parent['risk_metric_coverage'],
                                           components=copy.deepcopy(parent['risk_components'])))
            for _ in range(p.max_mutation_attempts):
                audit['mutation_attempts']+=1
                candidate,detail=_mutate(parent['action'],group,rng,self.target,p,audit)
                if candidate is None:_reject(audit,'mutation','unavailable coordinate');continue
                candidate=_valid(candidate,self.target,audit,'mutation')
                if candidate is None:continue
                if _duplicate(candidate,excluded,exact_excluded,audit,'mutation'):continue
                audit['mutation']=detail
                return dict(action=candidate,audit=audit)
            audit['fallback']='bounded_mutation_exhausted'
        elif self.policy=='search':audit['fallback']='no_eligible_incumbent' if not elites else 'fixed_probability_restart'
        audit['source']='global'
        # Global proposals share a stream across policies, independently of
        # the branch-selection/mutation RNG draws. Common starts are replayable.
        return dict(action=_global(_rng(self.seed,proposal_index,'global'),self.target,p,excluded,exact_excluded,audit),audit=audit)
