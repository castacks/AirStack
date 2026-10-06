"""Seeded new Office coordinates, checked against exported source USD bounds.

No simulator or USD dependency. Saved offsets are authoritative on replay.
The protected straight corridor is a qualification aid, not a planner guarantee.
"""
import hashlib, json, math, random
from functools import lru_cache
from pathlib import Path

COUNTS={'easy':1,'medium':3,'hard':5}
SCHEMA='office_generated_v1'
MANIFEST=Path(__file__).with_name('office_geometry.json')

@lru_cache(maxsize=1)
def geometry():
    return json.loads(MANIFEST.read_text())

def identity(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def number(value,lo,hi,name):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not lo<=value<=hi:
        raise ValueError(f'{name} must be finite in [{lo}, {hi}]')
    return float(value)

def parameters(seed,density,corridor_half_width_m,side_bias):
    if type(seed) is not int or not 0<=seed<2**31:raise ValueError('placement seed must be a 31-bit integer')
    if density not in COUNTS:raise ValueError('density must be easy, medium or hard')
    return {'seed':seed,'density':density,
            'corridor_half_width_m':number(corridor_half_width_m,.7,1.2,'corridor half width'),
            'side_bias':number(side_bias,-1,1,'side bias')}

def intersects(a,b,gap=.1):
    return all(a[0][i]<b[1][i]+gap and b[0][i]<a[1][i]+gap for i in range(3))

def translated(box,delta):
    return [[v+d for v,d in zip(corner,delta)] for corner in box]

def valid_box(box,occupied,corridor):
    g=geometry();roi=g['placement_roi_source_m']
    if any(box[0][i]<roi[0][i] or box[1][i]>roi[1][i] for i in (0,1)):return False
    if abs(box[0][2])>.03:return False
    if not all(any(f[0][0]<=x<=f[1][0] and f[0][1]<=y<=f[1][1] for f in g['floors'])
               for x in (box[0][0],box[1][0]) for y in (box[0][1],box[1][1])):return False
    # In source axes, the world x=-4..+4 route runs along source y.
    protected=[[-corridor,-4.8,-.1],[corridor,4.8,2.2]]
    return not any(intersects(box,other) for other in [protected,*occupied])

def generate(seed,density='easy',corridor_half_width_m=.9,side_bias=0.):
    spec=parameters(seed,density,corridor_half_width_m,side_bias);g=geometry()
    keys=['move']+[key if i==1 else f'{key}_{i}' for i in range(1,COUNTS[density]+1) for key in ('plant','column')]
    for attempt in range(16):
        rng=random.Random(seed+104729*attempt)
        occupied=[b for path,b in g['occupied'].items() if path!=g['sources']['move']['path']]
        offsets={};boxes={}
        for key in keys:
            kind=key.split('_')[0];box=g['sources'][kind]['bounds']
            for _ in range(1024):
                # Continuous positions throughout the permitted Office lobby.
                # side_bias requests one side more often; it does not pick a set.
                side=1 if rng.random()<(1+spec['side_bias']*.8)/2 else -1
                source_x=side*rng.uniform(spec['corridor_half_width_m']+.55,4.4)
                source_y=rng.uniform(-3.6,6.8)
                delta=[round(source_x-(box[0][0]+box[1][0])/2,6),
                       round(source_y-(box[0][1]+box[1][1])/2,6),0.]
                after=translated(box,delta)
                if valid_box(after,occupied,spec['corridor_half_width_m']):break
            else:break
            offsets[key]=delta;boxes[key]=after;occupied.append(after)
        else:
            result={'schema':SCHEMA,'source_sha256':g['source_sha256'],'parameters':spec,
                    'generation_attempt':attempt,'offsets':offsets,'bounds_source_m':boxes}
            result['placement_id']=identity(result)
            return result
    raise ValueError('No supported, nonoverlapping placement for requested constraints; select a different seed or density')

def validate_placement(raw):
    if not isinstance(raw,dict) or set(raw)!={'schema','source_sha256','parameters','generation_attempt','offsets','bounds_source_m','placement_id'}:
        raise ValueError('Malformed generated placement')
    if raw['schema']!=SCHEMA or raw['source_sha256']!=geometry()['source_sha256']:
        raise ValueError('Generated placement geometry source changed')
    p=raw['parameters']
    if not isinstance(p,dict) or set(p)!={'seed','density','corridor_half_width_m','side_bias'}:raise ValueError('Invalid placement parameters')
    parameters(**p)
    if type(raw['generation_attempt']) is not int or not 0<=raw['generation_attempt']<16:raise ValueError('Invalid generation attempt')
    keys={'move'}|{key if i==1 else f'{key}_{i}' for i in range(1,COUNTS[p['density']]+1) for key in ('plant','column')}
    if set(raw['offsets'])!=keys or set(raw['bounds_source_m'])!=keys:raise ValueError('Generated obstacle counts differ')
    occupied=[b for path,b in geometry()['occupied'].items() if path!=geometry()['sources']['move']['path']]
    for key,delta in raw['offsets'].items():
        if not isinstance(delta,list) or len(delta)!=3:raise ValueError('Offset must have three components')
        for v in delta:number(v,-50,50,'offset')
        if delta[2]!=0:raise ValueError('Only floor-plane placement is supported')
        box=translated(geometry()['sources'][key.split('_')[0]]['bounds'],delta)
        if box!=raw['bounds_source_m'][key] or not valid_box(box,occupied,p['corridor_half_width_m']):
            raise ValueError('Unsafe generated placement: overlap, support, bounds or protected corridor')
        occupied.append(box)
    if identity({k:v for k,v in raw.items() if k!='placement_id'})!=raw['placement_id']:
        raise ValueError('Generated placement identity mismatch')
    return raw

def realized(condition):
    if condition['layout']=='generated':
        p=validate_placement(condition['placement'])
        return {**p['offsets'],'placement_id':p['placement_id'],'source_sha256':p['source_sha256'],
                'difficulty':p['parameters']['density'],'generation_seed':p['parameters']['seed']}
    catalog=json.loads(Path(__file__).with_name('layouts.json').read_text())
    return catalog.get(condition['layout'],[{}]*8)[condition['layout_seed']]
