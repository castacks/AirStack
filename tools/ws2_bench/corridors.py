"""Procedural corridor geometry and ordered traversal scoring (no simulator).

Route geometry is evaluation truth, not an input to reactive Kim/PULP workers.
MonoNav can receive explicit mission goals at corners through its goal interface.
"""
import hashlib
import json
import math

SHAPES=('elbow','curve','zigzag','u_obstacles')
TIERS=('easy','medium','hard')


def tier_spec(shape, difficulty='easy'):
    """Common v2 geometry; medium moves blocks inward, hard adds three."""
    if shape not in SHAPES or difficulty not in TIERS:raise ValueError('Unknown corridor/tier')
    count=(2 if shape=='u_obstacles' else 0)+{'easy':0,'medium':2,'hard':5}[difficulty]
    spec=dict(shape=shape,width=4.,obstacles=count,style='hallway_daylight',
              route_scale=2.,difficulty=difficulty,layout_version=2)
    if shape!='curve':spec['bend_radius']=4.
    return spec


def geometry_hash(data):
    """Stable across Python/libm summation differences below a nanometre."""
    def rounded(value):
        if isinstance(value,float):return round(value,9)
        if isinstance(value,list):return [rounded(v) for v in value]
        if isinstance(value,dict):return {k:rounded(v) for k,v in value.items() if k!='placement_id'}
        return value
    return hashlib.sha256(json.dumps(rounded(data),sort_keys=True).encode()).hexdigest()


def validate_spec(raw):
    if not isinstance(raw,dict) or set(raw)-{'shape','width','obstacles','style','corner_bevel','bend_radius','route_scale','difficulty','layout_version'}:
        raise ValueError('corridor requires shape, width and obstacle count')
    spec=dict(shape='elbow',width=3.5,obstacles=2,style='hallway');spec.update(raw)
    if spec['shape'] not in SHAPES:raise ValueError('unknown corridor shape')
    if spec['style'] not in ('hallway','hallway_black','hallway_daylight','paper_lab','paper_lab_low'):raise ValueError('unknown corridor style')
    version=spec.get('layout_version',1)
    if type(version) is not int or version not in (1,2):raise ValueError('Unknown corridor layout version')
    if 'difficulty' in spec:
        if spec['difficulty'] not in TIERS:raise ValueError('Unknown corridor difficulty')
        added=2*TIERS.index(spec['difficulty']) if version==1 else {'easy':0,'medium':2,'hard':5}[spec['difficulty']]
        expected=(2 if spec['shape']=='u_obstacles' else 0)+added
        if spec['obstacles']!=expected:raise ValueError('Obstacle count must match corridor difficulty')
    w=spec['width']
    if isinstance(w,bool) or not isinstance(w,(float,int)) or not math.isfinite(w) or not 2.8<=w<=5:
        raise ValueError('corridor width must be 2.8..5m')
    if type(spec['obstacles']) is not int or not 0<=spec['obstacles']<=7:
        raise ValueError('corridor obstacle count must be 0..7')
    spec['width']=float(w)
    if 'route_scale' in spec:
        scale=spec['route_scale']
        if isinstance(scale,bool) or not isinstance(scale,(float,int)) or not math.isfinite(scale) or not 1<=scale<=2:
            raise ValueError('route_scale must be 1..2')
        spec['route_scale']=float(scale)
    if 'bend_radius' in spec:
        r=spec['bend_radius']
        if isinstance(r,bool) or not isinstance(r,(float,int)) or not math.isfinite(r) or not w/2+.2<=r<=4.2:
            raise ValueError('bend_radius must exceed half-width by 0.2m and be at most 4.2m')
        if spec['shape']=='curve' or spec.get('corner_bevel',0):
            raise ValueError('bend_radius replaces square corners; do not combine with curve/bevel')
        route=path(spec['shape'])
        if 2*r>=min(math.dist(a,b) for a,b in zip(route,route[1:]))*spec.get('route_scale',1.):
            raise ValueError('bend radius leaves no straight section between corners')
        spec['bend_radius']=float(r)
    if 'corner_bevel' in spec:
        b=spec['corner_bevel']
        if isinstance(b,bool) or not isinstance(b,(float,int)) or not math.isfinite(b) or not 0<=b<=1.5:
            raise ValueError('corner_bevel must be 0..1.5m')
        if spec['shape']=='curve' and b:raise ValueError('curve already has continuous bends')
        spec['corner_bevel']=float(b)
    return spec


def path(shape):
    if shape=='elbow':return [[-4.,0.],[3.,0.],[3.,8.]]
    if shape=='zigzag':return [[-4.,0.],[3.,0.],[3.,6.],[-3.,6.],[-3.,12.],[4.,12.]]
    if shape=='u_obstacles':return [[-4.,0.],[4.,0.],[4.,6.],[-4.,6.]]
    if shape=='curve':
        return [[-4.,0.]]+[[round(4*math.cos(a),6),round(4+4*math.sin(a),6)]
                           for a in [-math.pi/2+i*math.pi/24 for i in range(13)]]+[[4.,9.]]
    raise ValueError(shape)


def segments(points):
    result=[];distance=0.
    for a,b in zip(points,points[1:]):
        length=math.dist(a,b);u=[(b[i]-a[i])/length for i in range(2)]
        result.append(dict(a=a,b=b,length=length,start=distance,u=u));distance+=length
    return result


def rounded_path(points,radius):
    """Physical circular bends, approximated by 12 wall panels per quadrant."""
    result=[points[0]]
    for a,p,b in zip(points,points[1:],points[2:]):
        before=[(p[j]-a[j])/math.dist(a,p) for j in range(2)]
        after=[(b[j]-p[j])/math.dist(p,b) for j in range(2)]
        if 2*radius>=min(math.dist(a,p),math.dist(p,b)):
            raise ValueError('bend radius leaves no straight section between corners')
        cross=before[0]*after[1]-before[1]*after[0]
        sign=1 if cross>0 else -1
        start=[p[j]-radius*before[j] for j in range(2)]
        center=[start[0]-sign*before[1]*radius,start[1]+sign*before[0]*radius]
        angle=math.atan2(start[1]-center[1],start[0]-center[0])
        result.extend([[round(center[0]+radius*math.cos(angle+sign*i*math.pi/24),9),
                        round(center[1]+radius*math.sin(angle+sign*i*math.pi/24),9)] for i in range(13)])
    return result+[points[-1]]


def point_at(points,distance):
    ss=segments(points)
    s=next((s for s in ss if distance<=s['start']+s['length']),ss[-1])
    t=max(0,min(s['length'],distance-s['start']))
    return [s['a'][i]+s['u'][i]*t for i in range(2)],s['u']


def offset_line(points,offset):
    ss=segments(points);result=[]
    for i,p in enumerate(points):
        before=ss[max(0,i-1)]['u'];after=ss[min(i,len(ss)-1)]['u']
        n1=[-before[1],before[0]];n2=[-after[1],after[0]]
        den=1+n1[0]*n2[0]+n1[1]*n2[1]
        if den<.1:raise ValueError('corridor contains a reversing corner')
        result.append([p[j]+offset*(n1[j]+n2[j])/den for j in range(2)])
    return result


def geometry(raw,seed=42):
    spec=validate_spec(raw);points=path(spec['shape'])
    if spec.get('route_scale'):
        origin=points[0]
        points=[[origin[j]+spec['route_scale']*(p[j]-origin[j]) for j in range(2)] for p in points]
    if spec.get('bend_radius'):points=rounded_path(points,spec['bend_radius'])
    ss=segments(points);length=sum(s['length'] for s in ss)
    extended=[list(p) for p in points]
    for j in range(2):extended[0][j]-=3*ss[0]['u'][j];extended[-1][j]+=3*ss[-1]['u'][j]
    sides=[offset_line(extended,sign*spec['width']/2) for sign in (1,-1)]
    # Retreat only the protruding inside corner. This changes the physical
    # course for the larger Iris; it never alters contact geometry invisibly.
    bevel=spec.get('corner_bevel',0.)
    if bevel:
        for side_index,side in enumerate(sides):
            sign=1 if side_index==0 else -1;lines=segments(side);expanded=[side[0]]
            for i,p in enumerate(side[1:-1],1):
                before,after=lines[i-1],lines[i]
                cross=before['u'][0]*after['u'][1]-before['u'][1]*after['u'][0]
                if cross*sign>1e-6:
                    b=min(bevel,before['length']/3,after['length']/3)
                    expanded.extend([[p[j]-before['u'][j]*b for j in range(2)],
                                     [p[j]+after['u'][j]*b for j in range(2)]])
                else:expanded.append(p)
            sides[side_index]=expanded+[side[-1]]
    wall_height=1.2 if spec['style']=='paper_lab_low' else 1.8 if spec['style']=='paper_lab' else 3.2
    walls=[]
    for side in sides:
        for s in segments(side):walls.append(dict(center=[(s['a'][0]+s['b'][0])/2,(s['a'][1]+s['b'][1])/2,wall_height/2],
            size=[s['length']+.04,.16,wall_height],yaw=math.atan2(s['u'][1],s['u'][0])))
    for i in (0,-1):
        a,b=sides[0][i],sides[1][i]
        walls.append(dict(center=[(a[0]+b[0])/2,(a[1]+b[1])/2,wall_height/2],size=[math.dist(a,b),.16,wall_height],yaw=math.atan2(b[1]-a[1],b[0]-a[0])))
    # Place wall-attached blocks on straight sections, away from corner apexes.
    candidates=[]
    for s in ss:
        if s['length']>=5:
            candidates.append(s['start']+s['length']*.5)
    if spec['shape']=='u_obstacles':
        candidates=[2.5,4.5,length-4.5,length-2.5] if spec.get('bend_radius') else [3.,6.,16.,19.]
    selected=candidates[:spec['obstacles']]
    if len(selected)<spec['obstacles']:
        selected=[length*(i+1)/(spec['obstacles']+1) for i in range(spec['obstacles'])]
    if 'difficulty' in spec:
        # Preserve every easier-tier obstacle, including side and index.
        # Distances lie on straight sections, outside the takeoff footprint.
        selected=[2.5,4.5] if spec['shape']=='u_obstacles' else []
        if spec['difficulty'] in ('medium','hard'):
            selected += [length-6.,length-3.] if selected else [2.5,length-3.]
        if spec['difficulty']=='hard':
            selected += [10.,length-10.] if spec['shape']=='u_obstacles' else [5.5,length-6.]
        if spec.get('layout_version',1)==2 and spec['difficulty']!='easy':
            # Spread encounters through the route, rather than concentrating
            # them at takeoff/finish. Curved sections use the local tangent.
            # Medium intentionally relocates Easy's U blocks. Hard retains
            # every Medium block (index/side/pose) and adds three encounters.
            fractions=[.18,.38,.62,.82] if spec['shape']=='u_obstacles' else [.25,.75]
            if spec['difficulty']=='hard':
                fractions += [.28,.50,.72] if spec['shape']=='u_obstacles' else [.375,.50,.625]
            selected=[length*f for f in fractions]
        if len(selected)!=spec['obstacles']:raise ValueError('Tier obstacle count mismatch')
    obstacles=[]
    for i,d in enumerate(selected):
        p,u=point_at(points,d);side=1 if (i+seed)%2 else -1
        depth=spec['width']*.45 if spec['style'].startswith('paper_lab') else 1.0
        height=1.0 if spec['style']=='paper_lab_low' else 1.6 if spec['style']=='paper_lab' else 2.5
        offset=side*(spec['width']/2-depth/2)
        obstacles.append(dict(center=[p[0]-u[1]*offset,p[1]+u[0]*offset,height/2],
                              size=[.65,depth,height],yaw=math.atan2(u[1],u[0]),route_s=d))
    # Closely spaced scoring gates prevent shortcutting corners in evaluation.
    gates=[point_at(points,min(length,d))[0] for d in range(2,math.ceil(length),2)]+[points[-1]]
    data=dict(schema='corridor_v1',spec=spec,seed=seed,route=points,route_length_m=length,
              walls=walls,obstacles=obstacles,gates=gates,spawn=[*points[0],.07])
    data['placement_id']=geometry_hash(data)
    return data


class Traversal:
    def __init__(self,geom):
        self.geometry=geom;self.gate=0;self.maximum_progress=0.
        self.progress_stamp=None;self.progress_anchor=0.

    def update(self,position,stamp=None):
        g=self.geometry;xy=position[:2]
        # Complete a corridor cross-section, not a goal point that reactive
        # models never receive. All earlier gates must have been traversed.
        while self.gate<len(g['gates']):
            if self.gate==len(g['gates'])-1:
                last=segments(g['route'])[-1]
                delta=[xy[i]-last['b'][i] for i in range(2)]
                along=sum(delta[i]*last['u'][i] for i in range(2))
                cross=abs(-delta[0]*last['u'][1]+delta[1]*last['u'][0])
                if along<-.5 or cross>g['spec']['width']/2-.25:break
            elif math.dist(xy,g['gates'][self.gate])>g['spec']['width']/2+.15:break
            self.gate+=1
        projection=[]
        for s in segments(g['route']):
            t=max(0.,min(s['length'],sum((xy[i]-s['a'][i])*s['u'][i] for i in range(2))))
            q=[s['a'][i]+s['u'][i]*t for i in range(2)]
            projection.append((math.dist(xy,q),s['start']+t))
        cross,progress=min(projection)
        if cross<=g['spec']['width']/2:self.maximum_progress=max(self.maximum_progress,progress)
        if stamp is not None and (self.progress_stamp is None or self.maximum_progress-self.progress_anchor>=.25):
            self.progress_stamp=stamp;self.progress_anchor=self.maximum_progress
        return self.gate==len(g['gates'])

    def stalled(self,stamp,timeout):
        return self.progress_stamp is not None and stamp-self.progress_stamp>=timeout

    def metrics(self):
        g=self.geometry
        return dict(route_scoring_version=2,route_length_m=g['route_length_m'],route_progress_m=self.maximum_progress,
                    route_progress_percent=100*self.maximum_progress/g['route_length_m'],
                    route_gates_passed=self.gate,route_gate_count=len(g['gates']),
                    route_completed=self.gate==len(g['gates']))
