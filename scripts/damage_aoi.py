"""Evidence-shaped AOIs and collision-checked multi-view inspection, no human GT."""
import numpy as np
from scipy.spatial import cKDTree, ConvexHull
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

def regions(xyz, radius=.9, minimum=6):
    """Density connected 3D evidence; border points cannot bridge dense regions.

    radius is a physical gap allowance, not a ground partition or label score.
    Input repeats are deduplicated at the existing map resolution.
    """
    xyz=np.unique(np.round(np.asarray(xyz)/.5).astype(int),axis=0)*.5
    if len(xyz)<minimum:return []
    tree=cKDTree(xyz);neighbors=tree.query_ball_point(xyz,radius)
    core=np.array([len(n)>=4 for n in neighbors]);ci=np.flatnonzero(core)
    if not len(ci):return []
    pairs=cKDTree(xyz[ci]).query_pairs(radius,output_type='ndarray')
    graph=coo_matrix((np.ones(len(pairs)),(pairs[:,0],pairs[:,1])),shape=(len(ci),len(ci))).tocsr()
    n,groups=connected_components(graph,directed=False)
    distance,nearest=cKDTree(xyz[ci]).query(xyz);assignment=groups[nearest];result=[]
    for g in range(n):
        points=xyz[(assignment==g)&(distance<=radius)]
        if len(points)<minimum:continue
        low=points.min(0)-.25;high=points.max(0)+.25
        xy=np.unique(points[:,:2],axis=0)
        try:hull=xy[ConvexHull(xy).vertices]
        except Exception:hull=np.array([[low[0],low[1]],[high[0],low[1]],[high[0],high[1]],[low[0],high[1]]])
        result.append(dict(center=points[:,:2].mean(0).tolist(),lo=low.tolist(),hi=high.tolist(),polygon=hull.tolist(),damage_voxels=len(points)))
    return sorted(result,key=lambda r:-r['damage_voxels'])

def connection(start, goal, clearance, max_altitude=12.):
    """Direct waypoint first; otherwise checked progressing detour waypoints.

    No staging column or mandatory cruise height. Candidate bends change XY
    and Z together. Return None if no checked route within the ceiling exists.
    """
    start=np.asarray(start,float);goal=np.asarray(goal,float)
    if max(start[2],goal[2])>max_altitude+1e-6:return None
    if clearance(start,goal):return [goal.tolist()]
    delta=goal[:2]-start[:2];distance=np.linalg.norm(delta)
    if distance<.1:return None
    perpendicular=np.array([-delta[1],delta[0]])/distance
    floor=max(3.5,min(start[2],goal[2]))
    heights=sorted(set([floor,max_altitude]+[h for h in (6.,9.,12.) if floor<=h<=max_altitude]))
    candidates=[]
    for fraction in (.25,.5,.75):
        for lateral in (0.,-4.,4.,-8.,8.,-12.,12.):
            for height in heights:
                point=np.r_[start[:2]+fraction*delta+lateral*perpendicular,height]
                cost=np.linalg.norm(point-start)+np.linalg.norm(goal-point)+2*max(0,height-floor)
                candidates.append((cost,point))
    for cost,point in sorted(candidates,key=lambda v:v[0]):
        if clearance(start,point) and clearance(point,goal):return [point.tolist(),goal.tolist()]
    # A longer obstruction may require two bends. All edges still move forward.
    layers=[]
    for fraction in (.2,.8):
        layer=[]
        for lateral in (0.,-6.,6.,-12.,12.):
            for height in heights:
                point=np.r_[start[:2]+fraction*delta+lateral*perpendicular,height]
                if clearance(start,point) if fraction<.5 else clearance(point,goal):layer.append(point)
        layers.append(layer)
    pairs=sorted(((np.linalg.norm(x-start)+np.linalg.norm(y-x)+np.linalg.norm(goal-y)+2*max(0,x[2]-floor,y[2]-floor),x,y) for x in layers[0] for y in layers[1]),key=lambda v:v[0])
    for cost,x,y in pairs:
        if clearance(x,y):return [x.tolist(),y.tolist(),goal.tolist()]
    # More complex obstacles: a small layered visibility graph. XY progresses
    # on every leg, allowing a lateral route around trees/buildings below cap.
    frontier=[(0.,start,[])]
    for fraction in (.15,.4,.65,.85):
        next_frontier=[]
        for lateral in (0.,-6.,6.,-12.,12.,-18.,18.):
            for height in heights:
                point=np.r_[start[:2]+fraction*delta+lateral*perpendicular,height]
                if not clearance(point,point):continue
                options=sorted(((cost+np.linalg.norm(point-old)+.2*max(0,height-floor),old,path) for cost,old,path in frontier),key=lambda v:v[0])
                for cost,old,path in options:
                    if clearance(old,point):
                        next_frontier.append((cost,point,path+[point.tolist()]));break
        frontier=next_frontier
        if not frontier:return None
    for cost,point,path in sorted(frontier,key=lambda v:v[0]+np.linalg.norm(goal-v[1])):
        if clearance(point,goal):
            # Remove every unnecessary bend after obtaining a feasible path.
            remaining=path+[goal.tolist()];simplified=[];current=start
            while remaining:
                last=max(i for i,point in enumerate(remaining) if clearance(current,point))
                current=np.asarray(remaining[last]);simplified.append(current.tolist());remaining=remaining[last+1:]
            return simplified
    return None


def inspection_footprint(region, viewpoints):
    """Planned search envelope, deliberately separate from detected damage."""
    xy=np.vstack([np.asarray(region['polygon']),np.array([v['point'][:2] for v in viewpoints])])
    hull=xy[ConvexHull(xy).vertices]
    return dict(polygon=hull.tolist(),lo=np.r_[hull.min(0),0.].tolist(),hi=np.r_[hull.max(0),1.].tolist(),kind='planned inspection envelope; visibility not established')


def inspection(region, previous, clearance, max_altitude=12., max_views=6):
    """Distinct low views, connected directly where clear, no high exit leg."""
    poly=np.array(region['polygon']);center=np.array(region['center']);candidates=[]
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        for t in np.linspace(0,1,max(2,int(np.linalg.norm(b-a)/4)+1)):
            target=a*(1-t)+b*t;direction=target-center
            if np.linalg.norm(direction)<.1:continue
            direction/=np.linalg.norm(direction)
            for offset in (4.,6.,8.,10.,12.):
                point=np.r_[target+offset*direction,3.5]
                if clearance(point,point):
                    if all(np.linalg.norm(point[:2]-v['point'][:2])>=3. for v in candidates):
                        candidates.append(dict(point=point,yaw=float(np.arctan2(-direction[1],-direction[0])),target=target.tolist()))
                    break
    # Also search accessible interior damage patches, not just the perimeter.
    for target in region.get('inspection_targets',[]):
        target=np.asarray(target)
        for angle in np.arange(0,2*np.pi,np.pi/2):
            direction=np.array([np.cos(angle),np.sin(angle)])
            point=np.r_[target+3.5*direction,3.5]
            if clearance(point,point) and all(np.linalg.norm(point[:2]-v['point'][:2])>=2. for v in candidates):
                candidates.append(dict(point=point,yaw=float(np.arctan2(-direction[1],-direction[0])),target=target.tolist()))
    if len(candidates)<3:return [],[]
    chosen=[min(candidates,key=lambda v:np.linalg.norm(v['point']-previous))]
    while len(chosen)<min(max_views,len(candidates)):
        remaining=[v for v in candidates if all(v is not q for q in chosen)]
        chosen.append(max(remaining,key=lambda v:min(np.linalg.norm(np.array(v['target'])-q['target']) for q in chosen)))
    ordered=[];current=np.asarray(previous);route=[]
    while chosen:
        v=None
        for candidate in sorted(chosen,key=lambda q:np.linalg.norm(current-q['point'])):
            path=connection(current,candidate['point'],clearance,max_altitude)
            if path is not None:v=candidate;break
        if v is None:break
        chosen=[q for q in chosen if q is not v]
        for point in path[:-1]:route.append(dict(point=point,yaw=v['yaw'],view=False))
        route.append(dict(point=v['point'].tolist(),yaw=v['yaw'],view=True,target=v['target'],view_index=len(ordered)+1));ordered.append(v);current=v['point']
    if len(ordered)<3:return [],[]
    return route,[dict(point=v['point'].tolist(),target=v['target'],yaw=v['yaw']) for v in ordered]


def retrace_connection(start, goal, visited_waypoints, clearance, max_altitude=12.):
    """Shortcut a checked reversed route when the local detour search fails."""
    current=np.asarray(start,float);remaining=list(reversed(visited_waypoints))+[list(goal)];path=[]
    if current[2]>max_altitude+1e-6:return None
    while remaining:
        possible=[i for i,point in enumerate(remaining) if point[2]<=max_altitude and clearance(current,point)]
        if not possible:return None
        index=max(possible);current=np.asarray(remaining[index],float);path.append(current.tolist());remaining=remaining[index+1:]
    return path
