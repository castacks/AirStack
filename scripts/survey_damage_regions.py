"""Persistent, multi-view damage grouping for survey-driven inspection tasks.

All inputs are predicted labels and observed geometry. No GT inputs. Numerical
limits express observation independence and inspection task size, not cosine
confidence. Connected evidence is never enlarged into a pretend damage box.
"""
from collections import defaultdict,Counter
import numpy as np
from scipy.spatial import cKDTree,ConvexHull

class SurveyDamageRegions:
    def __init__(self, labels, baseline=3., gap=1.8, max_span=24.):
        self.labels=labels;self.baseline=baseline;self.gap=gap;self.max_span=max_span
        self.damage={labels.index(v) for v in ('collapsed house','collapsed roof','debris')}
        self.structure={labels.index(v) for v in ('house','roof','collapsed house','collapsed roof')}
        self.poses=[];self.votes={};self.sources=defaultdict(set);self.frame_files=[]
        self.tracks={};self.seen_updates=Counter();self.next_id=1;self.groups=[];self.rejected={}

    def add(self, xyz, chosen, state, position, frame_file=''):
        position=np.asarray(position,float)
        if self.poses and min(np.linalg.norm(position[:2]-p[:2]) for p in self.poses)<self.baseline:return False
        frame_id=len(self.poses);self.poses.append(position);self.frame_files.append(frame_file)
        cells=np.round(np.asarray(xyz)/.5).astype(int)
        # One vote per frame/map voxel, including competing non-damage labels.
        seen=set()
        for cell,label,valid in zip(cells,chosen,state):
            key=tuple(cell)
            if key in seen or valid!=1 or label<0:continue
            seen.add(key)
            self.votes.setdefault(key,Counter())[int(label)]+=1
            if int(label) in self.damage:self.sources[key].add(frame_id)
        self._rebuild();return True

    def _rebuild(self):
        selected=[];labels=[]
        for key,votes in self.votes.items():
            ranked=votes.most_common()
            if len(ranked)>1 and ranked[0][1]==ranked[1][1]:continue
            label=ranked[0][0]
            if label in self.damage|self.structure and -.5<=key[2]*.5<12:
                selected.append(key);labels.append(label)
        if not selected:self.groups=[];return
        cells=np.asarray(selected);xyz=cells*.5;labels=np.asarray(labels);n=len(xyz)
        tree=cKDTree(xyz);pairs=tree.query_pairs(self.gap,output_type='ndarray')
        degree=np.bincount(pairs.ravel(),minlength=n) if len(pairs) else np.zeros(n,int)
        # Sparse observations can be attached, but cannot bridge dense groups.
        core=degree>=3;parent=np.arange(n);low=xyz.copy();high=xyz.copy()
        def root(i):
            while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
            return i
        if len(pairs):
            distance=np.linalg.norm(xyz[pairs[:,0]]-xyz[pairs[:,1]],axis=1)
            order=np.lexsort((pairs[:,1],pairs[:,0],distance))
            for i,j in pairs[order]:
                if not (core[i] and core[j]):continue
                a,b=root(i),root(j)
                if a==b:continue
                lo=np.minimum(low[a],low[b]);hi=np.maximum(high[a],high[b])
                if max((hi-lo)[:2])>self.max_span:continue
                parent[b]=a;low[a]=lo;high[a]=hi
        roots=np.array([root(i) for i in range(n)])
        if core.any():
            ci=np.flatnonzero(core);dist,near=cKDTree(xyz[ci]).query(xyz[~core]);roots[~core]=np.where(dist<=self.gap,roots[ci[near]],roots[~core])
        candidates=[];reject=Counter()
        for g in np.unique(roots):
            rows=np.flatnonzero(roots==g);damage_rows=rows[np.isin(labels[rows],list(self.damage))]
            if len(damage_rows)<40:reject['small_observation']+=1;continue
            points=xyz[rows];xy=np.unique(points[:,:2],axis=0)
            try:hull=ConvexHull(xy);polygon=xy[hull.vertices];area=float(hull.volume)
            except Exception:reject['degenerate']+=1;continue
            covariance=np.linalg.eigvalsh(np.cov(xy.T))
            if area<24 or covariance[0]<1.:reject['narrow_or_small_patch']+=1;continue
            contributions=Counter(f for i in damage_rows for f in self.sources[tuple(cells[i])])
            views=[f for f,count in contributions.items() if count>=8]
            if len(views)<3:reject['insufficient_distinct_views']+=1;continue
            members={tuple(cells[i]) for i in damage_rows}
            ground=xyz[damage_rows][xyz[damage_rows,2]<3.]
            targets=[]
            if len(ground):
                pool=np.unique(ground[:,:2],axis=0)
                targets=[pool[np.argmin(np.linalg.norm(pool-pool.mean(0),axis=1))]]
                for _ in range(7):
                    distance=np.min(np.linalg.norm(pool[:,None,:]-np.array(targets)[None,:,:],axis=2),axis=1)
                    if distance.max()<4.:break
                    targets.append(pool[np.argmax(distance)])
            candidates.append((members,dict(center=points[:,:2].mean(0).tolist(),lo=(points.min(0)-.25).tolist(),hi=(points.max(0)+.25).tolist(),polygon=polygon.tolist(),damage_voxels=len(damage_rows),support_voxels=len(rows),area_m2=area,view_count=len(views),survey_frames=[self.frame_files[f] for f in views],view_positions=[self.poses[f].tolist() for f in views],inspection_targets=np.array(targets).tolist(),structure_voxels=int(np.isin(labels[rows],list(self.structure)).sum()),source='Multi-view predicted damage and associated building geometry')))
        # Match by actual shared damage cells, not overlap of broad rectangles.
        used=set();groups=[];updated={}
        for members,group in sorted(candidates,key=lambda v:-len(v[0])):
            matches=[(len(members&old),ident) for ident,old in self.tracks.items() if ident not in used]
            overlap,ident=max(matches,default=(0,0))
            if overlap<max(6,.15*len(members)):
                ident=self.next_id;self.next_id+=1
            used.add(ident);updated[ident]=members;group['region_id']=ident;self.seen_updates[ident]+=1;group['stable_updates']=self.seen_updates[ident];groups.append(group)
        self.tracks.update(updated);self.groups=sorted(groups,key=lambda g:-(g['damage_voxels']*min(g['view_count'],6)))
        self.rejected=dict(reject)
