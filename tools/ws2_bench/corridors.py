"""Procedural corridor geometry and ordered traversal scoring (no simulator).

Route geometry is evaluation truth, not an input to reactive Kim/PULP workers.
MonoNav can receive explicit mission goals at corners through its goal interface.
"""
import hashlib
import json
import math

SHAPES=('elbow','curve','zigzag','u_obstacles')
TIERS=('easy','medium','hard')
OBSTACLE_LIMITS={'lateral_m':(-1.65,1.65),'width_m':(.4,1.4),'depth_m':(.4,1.5),
                 'height_m':(.6,2.8),'yaw_deg':(-60.,60.)}


def tier_spec(shape, difficulty='easy'):
    """Common v3 geometry; medium moves blocks inward, hard adds three."""
    if shape not in SHAPES or difficulty not in TIERS:raise ValueError('Unknown corridor/tier')
    count=(2 if shape=='u_obstacles' else 0)+{'easy':0,'medium':2,'hard':5}[difficulty]
    spec=dict(shape=shape,width=4.,obstacles=count,style='hallway_daylight',
              route_scale=2.,difficulty=difficulty,layout_version=3)
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
    if not isinstance(raw,dict) or set(raw)-{'shape','width','obstacles','style','corner_bevel','bend_radius','route_scale','difficulty','layout_version','obstacle_placements'}:
        raise ValueError('corridor requires shape, width and obstacle count')
    spec=dict(shape='elbow',width=3.5,obstacles=2,style='hallway');spec.update(raw)
    if spec['shape'] not in SHAPES:raise ValueError('unknown corridor shape')
    if spec['style'] not in ('hallway','hallway_black','hallway_daylight','paper_lab','paper_lab_low'):raise ValueError('unknown corridor style')
    version=spec.get('layout_version',1)
    if type(version) is not int or version not in (1,2,3,4,5):raise ValueError('Unknown corridor layout version')
    if version>=4:
        limits={'easy':(0,2),'medium':(2,4),'hard':(4,7)}
        if spec.get('difficulty') not in limits:raise ValueError('v4 requires difficulty')
        lo,hi=limits[spec['difficulty']]
        placements=spec.get('obstacle_placements')
        if not isinstance(placements,list) or not lo<=len(placements)<=hi or len(placements)!=spec['obstacles']:
            raise ValueError('v4 obstacle count must match placements and difficulty bounds')
        for p in placements:
            fields={'route_s','side'} if version==4 else {'route_s',*OBSTACLE_LIMITS}
            if not isinstance(p,dict) or set(p)!=fields:raise ValueError('Obstacle fields must be '+', '.join(sorted(fields)))
            if version==4 and (type(p['side']) is not int or p['side'] not in (-1,1)):
                raise ValueError('Each obstacle requires side -1 or 1')
            if type(p['route_s']) not in (int,float) or not math.isfinite(p['route_s']):raise ValueError('Invalid obstacle route_s')
            if version==5:
                for key,(low,high) in OBSTACLE_LIMITS.items():
                    if type(p[key]) not in (int,float) or not math.isfinite(p[key]) or not low<=p[key]<=high:
                        raise ValueError(f'{key} must be finite and in [{low}, {high}]')
        if version==5 and (spec['width']!=4. or spec.get('route_scale')!=2.):
            raise ValueError('v5 keeps corridor width 4m and route scale 2; only obstacle geometry expands')
    elif 'obstacle_placements' in spec:raise ValueError('Explicit obstacles require layout_version 4 or 5')
    if 'difficulty' in spec and version<4:
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


def _box_corners(box):
    x,y=box['center'][:2];a=box['yaw'];c,s=math.cos(a),math.sin(a)
    w,d=box['size'][:2]
    return [(x+c*u-s*v,y+s*u+c*v) for u,v in [(-w/2,-d/2),(w/2,-d/2),(w/2,d/2),(-w/2,d/2)]]


def _boxes_overlap(a,b,margin=0.):
    """Separating-axis test on the actual rotated collision footprints."""
    for angle in (a['yaw'],a['yaw']+math.pi/2,b['yaw'],b['yaw']+math.pi/2):
        axis=(math.cos(angle),math.sin(angle))
        aa=[x*axis[0]+y*axis[1] for x,y in _box_corners(a)]
        bb=[x*axis[0]+y*axis[1] for x,y in _box_corners(b)]
        if max(aa)+margin<=min(bb) or max(bb)+margin<=min(aa):return False
    return True


def _point_segment_distance(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];den=dx*dx+dy*dy
    f=0. if den==0. else max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den))
    return math.dist(p,(a[0]+f*dx,a[1]+f*dy))


def _segment_box_distance(a,b,box):
    """Exact 2-D route-segment distance to a rotated rectangle."""
    angle=box['yaw'];c,s=math.cos(angle),math.sin(angle)
    def local(p):
        dx,dy=p[0]-box['center'][0],p[1]-box['center'][1]
        return (dx*c+dy*s,-dx*s+dy*c)
    a,b=local(a),local(b);w,d=box['size'][0]/2,box['size'][1]/2
    # Slab clipping detects an intersection, including tangency.
    low,high=0.,1.
    for j,half in enumerate((w,d)):
        delta=b[j]-a[j]
        if abs(delta)<1e-12:
            if abs(a[j])>half:low,high=1.,0.;break
        else:
            t0,t1=sorted(((-half-a[j])/delta,(half-a[j])/delta))
            low=max(low,t0);high=min(high,t1)
    if low<=high:return 0.
    corners=[(-w,-d),(w,-d),(w,d),(-w,d)]
    return min([math.hypot(max(abs(p[0])-w,0.),max(abs(p[1])-d,0.)) for p in (a,b)]+
               [_point_segment_distance(p,a,b) for p in corners])


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
    if 'difficulty' in spec and spec.get('layout_version',1)<4:
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
        if spec.get('layout_version',1)==3 and spec['difficulty']!='easy':
            # Move the old endpoint clusters inward, without adding an inner
            # corner obstruction to every Medium traversal. Hard adds a
            # central encounter as well as two intermediate blocks.
            selected=[5.25,8.25,length-8.25,length-5.25] if spec['shape']=='u_obstacles' else [5.25,length-5.25]
            if spec['difficulty']=='hard':
                selected += [11.,length-11.,length/2] if spec['shape']=='u_obstacles' else [8.25,length-8.25,length/2]
            if any(not 5.<d<length-5. for d in selected):
                raise ValueError('v3 tier requires enough route length for interior encounters')
        if len(selected)!=spec['obstacles']:raise ValueError('Tier obstacle count mismatch')
    if spec.get('layout_version') in (4,5):
        selected=[p['route_s'] for p in spec['obstacle_placements']]
        if any(not 4.<=d<=length-4. for d in selected):raise ValueError('Obstacles must be at least 4m from route endpoints')
        if any(abs(a-b)<1.5 for i,a in enumerate(selected) for b in selected[i+1:]):
            raise ValueError('Obstacle encounters must be separated by at least 1.5m along the route')
    obstacles=[]
    for i,d in enumerate(selected):
        p,u=point_at(points,d);side=spec['obstacle_placements'][i]['side'] if spec.get('layout_version')==4 else (1 if (i+seed)%2 else -1)
        depth=spec['width']*.45 if spec['style'].startswith('paper_lab') else 1.0
        height=1.0 if spec['style']=='paper_lab_low' else 1.6 if spec['style']=='paper_lab' else 2.5
        offset=side*(spec['width']/2-depth/2)
        width=.65;yaw=math.atan2(u[1],u[0])
        if spec.get('layout_version')==5:
            placement=spec['obstacle_placements'][i]
            offset=placement['lateral_m'];width=placement['width_m'];depth=placement['depth_m'];height=placement['height_m']
            yaw+=math.radians(placement['yaw_deg'])
        obstacles.append(dict(center=[p[0]-u[1]*offset,p[1]+u[0]*offset,height/2],
                              size=[width,depth,height],yaw=yaw,route_s=d))
    if spec.get('layout_version')==4:
        # Preserve a continuous nominal-envelope path; this is geometric
        # feasibility, never a guarantee that a target can fly the course.
        samples=[point_at(points,min(length,i*.2))[0] for i in range(math.ceil(length/.2)+1)]
        for b in obstacles:
            def distance(p):
                dx,dy=p[0]-b['center'][0],p[1]-b['center'][1];a=b['yaw']
                x=abs(dx*math.cos(a)+dy*math.sin(a));y=abs(-dx*math.sin(a)+dy*math.cos(a))
                return math.hypot(max(0,x-b['size'][0]/2),max(0,y-b['size'][1]/2))
            if min(map(distance,samples))<.7:raise ValueError('Obstacle blocks the protected 0.7m centreline envelope')
        for i,b in enumerate(obstacles):
            for other in obstacles[i+1:]:
                # Bounding circles conservatively prevent block intersections.
                if math.dist(b['center'][:2],other['center'][:2])<math.hypot(.65,1.)+.1:
                    raise ValueError('Obstacle boxes overlap or are too close')
    elif spec.get('layout_version')==5:
        # Continuous segment checks preserve the full nominal route envelope;
        # exact rotated collision footprints prevent wall/block intersections.
        for i,b in enumerate(obstacles):
            if min(_segment_box_distance(a,z,b) for a,z in zip(points,points[1:]))<.7:
                raise ValueError('Obstacle blocks the protected 0.7m centreline envelope')
            if any(_boxes_overlap(b,w,.03) for w in walls):
                raise ValueError('Obstacle intersects a corridor wall or its 3cm margin')
            if any(_boxes_overlap(b,o,.1) for o in obstacles[i+1:]):
                raise ValueError('Obstacle boxes overlap or are less than 10cm apart')
    # Closely spaced scoring gates prevent shortcutting corners in evaluation.
    gates=[point_at(points,min(length,d))[0] for d in range(2,math.ceil(length),2)]+[points[-1]]
    data=dict(schema='corridor_v1',spec=spec,seed=seed,route=points,route_length_m=length,
              walls=walls,obstacles=obstacles,gates=gates,spawn=[*points[0],.07])
    data['placement_id']=geometry_hash(data)
    return data


def patch_corners(data,size,mount=None):
    """Original texture on one real block face, with explicit replayable mount."""
    mount=mount or {'obstacle_index':0,'face':'front','height_m':.5 if data['spec']['style']=='paper_lab_low' else 1.2}
    if not isinstance(mount,dict) or set(mount)!={'obstacle_index','face','height_m'}:
        raise ValueError('Patch mount requires obstacle_index, face and height_m')
    index=mount['obstacle_index'];height=mount['height_m'];face=mount['face']
    if type(index) is not int or not 0<=index<len(data['obstacles']):raise ValueError('Patch obstacle index is unavailable')
    if type(height) not in (int,float) or not math.isfinite(height):raise ValueError('Invalid patch height')
    b=data['obstacles'][index];x,y,z=b['center'];a=b['yaw']
    forward=(math.cos(a),math.sin(a));left=(-math.sin(a),math.cos(a))
    if face=='front':normal=(-forward[0],-forward[1]);horizontal=left;depth=b['size'][0];width=b['size'][1]
    elif face=='inner':
        p,_=point_at(data['route'],b['route_s'])
        side=1 if (x-p[0])*left[0]+(y-p[1])*left[1]>0 else -1
        normal=(-side*left[0],-side*left[1]);horizontal=forward;depth=b['size'][1];width=b['size'][0]
    else:raise ValueError('Patch face must be front or inner')
    if size>width-.02 or height-size/2<.05 or height+size/2>b['size'][2]-.05:
        raise ValueError('Patch must fit completely on its selected physical surface')
    center=(x+normal[0]*(depth/2+.008),y+normal[1]*(depth/2+.008),height)
    return [(center[0]+horizontal[0]*u,center[1]+horizontal[1]*u,center[2]+v)
            for u,v in [(-size/2,-size/2),(size/2,-size/2),(size/2,size/2),(-size/2,size/2)]]


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
