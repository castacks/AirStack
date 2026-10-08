"""Office avoidance challenges with a checked free-space route, not a cleared line.

Route checks are conservative 2-D AABB checks at 1.2 m altitude, using a 0.35 m
vehicle radius. They establish geometric feasibility, not planner success.
"""
import copy
from collections import deque
from generated_layouts import generate, geometry, identity, intersects, translated, valid_box

SCHEMA = 'office_challenge_v1'
CHALLENGES = ('offset_obstacle', 'slalom', 'offset_gap')


def free_route(boxes):
    # Source (x,y) = (-world_y, world_x). Keep takeoff and goal unchanged.
    step = .2
    start, goal = (0, -20), (0, 20)
    def free(node):
        x, y = (v * step for v in node)
        if not (-2.8 <= x <= 2.8 and -4.2 <= y <= 4.2): return False
        if not all(any(f[0][0]<=u<=f[1][0] and f[0][1]<=v<=f[1][1] for f in geometry()['floors'])
                   for u in (x-.35,x+.35) for v in (y-.35,y+.35)):return False
        return not any(b[0][2] < 1.55 and b[1][2] > .85 and
                       b[0][0]-.35 <= x <= b[1][0]+.35 and
                       b[0][1]-.35 <= y <= b[1][1]+.35 for b in boxes)
    if not free(start) or not free(goal): raise ValueError('Blocked challenge endpoint')
    queue = deque([start]); parent = {start: None}
    while queue:
        n = queue.popleft()
        if n == goal: break
        for dx, dy in ((0,1),(1,0),(-1,0),(0,-1)):
            nxt = (n[0]+dx,n[1]+dy)
            if nxt not in parent and free(nxt): parent[nxt]=n; queue.append(nxt)
    if goal not in parent: raise ValueError('No collision-free route through challenge')
    route=[]; n=goal
    while n is not None:
        route.append([round(n[1]*step,4), round(-n[0]*step,4), 1.2]); n=parent[n]
    return list(reversed(route))


def validate_challenge(raw):
    expected={'schema','source_sha256','parameters','generation_attempt','offsets','bounds_source_m','placement_id','feasible_route_world_m'}
    if not isinstance(raw,dict) or set(raw)!=expected or raw.get('schema')!=SCHEMA:
        raise ValueError('Malformed challenge placement')
    g=geometry(); p=raw['parameters']
    if not isinstance(p,dict) or set(p)!={'seed','density','corridor_half_width_m','side_bias','challenge'}:
        raise ValueError('Invalid challenge parameters')
    if type(raw['generation_attempt']) is not int or not 0<=raw['generation_attempt']<16:
        raise ValueError('Invalid challenge generation attempt')
    if raw['source_sha256']!=g['source_sha256'] or p.get('challenge') not in CHALLENGES:
        raise ValueError('Unknown challenge or changed source geometry')
    from generated_layouts import parameters, COUNTS
    parameters(**{k:v for k,v in p.items() if k!='challenge'})
    keys={'move'}|{k if i==1 else f'{k}_{i}' for i in range(1,COUNTS[p['density']]+1) for k in ('plant','column')}
    if set(raw['offsets'])!=keys or set(raw['bounds_source_m'])!=keys: raise ValueError('Wrong challenge prop count')
    occupied=[b for path,b in g['occupied'].items() if path!=g['sources']['move']['path']]
    ends=[[[-.75,-4.7,-.1],[.75,-3.25,2.2]],[[-.65,3.35,-.1],[.65,4.7,2.2]]]
    for key,delta in raw['offsets'].items():
        from generated_layouts import number
        if not isinstance(delta,list) or len(delta)!=3: raise ValueError('Invalid challenge offset')
        for v in delta: number(v,-50,50,'offset')
        if delta[2]!=0: raise ValueError('Only horizontal challenge moves are allowed')
        b=translated(g['sources'][key.split('_')[0]]['bounds'],delta)
        if b!=raw['bounds_source_m'][key] or not valid_box(b,occupied,None) or any(intersects(b,e) for e in ends):
            raise ValueError('Unsupported/overlapping challenge prop or blocked endpoint')
        occupied.append(b)
    route=free_route(occupied)
    if route!=raw['feasible_route_world_m']: raise ValueError('Challenge route check mismatch')
    if identity({k:v for k,v in raw.items() if k!='placement_id'})!=raw['placement_id']:
        raise ValueError('Challenge identity mismatch')
    return raw


def generate_challenge(challenge='offset_obstacle', seed=42):
    if challenge not in CHALLENGES: raise ValueError('Unknown challenge')
    density='easy' if challenge=='offset_obstacle' else 'medium'
    p=copy.deepcopy(generate(seed,density,1.2,0.))
    centers={'offset_obstacle':[(1.3,.35)],
             'slalom':[(-1.6,-.35),(1.3,.35)],
             'offset_gap':[(1.3,-.9),(1.3,1.0)]}[challenge]
    for i,(wx,wy) in enumerate(centers,1):
        key='column' if i==1 else f'column_{i}'
        box=geometry()['sources']['column']['bounds']
        delta=[round(-wy-(box[0][0]+box[1][0])/2,6),round(wx-(box[0][1]+box[1][1])/2,6),0.]
        p['offsets'][key]=delta; p['bounds_source_m'][key]=translated(box,delta)
    p['schema']=SCHEMA; p['parameters']['challenge']=challenge
    boxes=[b for path,b in geometry()['occupied'].items() if path!=geometry()['sources']['move']['path']]
    p['feasible_route_world_m']=free_route(boxes+list(p['bounds_source_m'].values()))
    p['placement_id']=identity({k:v for k,v in p.items() if k!='placement_id'})
    return validate_challenge(p)
