"""Group predicted damage patches into bounded inspection areas; no person GT."""
import copy
import numpy as np
from scipy.spatial import ConvexHull


def group_regions(regions, gap=8., max_span=60.):
    groups = [[r] for r in sorted(regions, key=lambda r: r['region_id'])]
    def bounds(group):
        return np.min([r['lo'] for r in group], axis=0), np.max([r['hi'] for r in group], axis=0)
    while True:
        pairs = []
        for i, left in enumerate(groups):
            lo, hi = bounds(left)
            for j in range(i + 1, len(groups)):
                lo2, hi2 = bounds(groups[j])
                distance = np.linalg.norm(np.maximum(0, np.maximum(lo[:2]-hi2[:2], lo2[:2]-hi[:2])))
                span = np.max((np.maximum(hi, hi2)-np.minimum(lo, lo2))[:2])
                if distance <= gap and span <= max_span:
                    pairs.append((distance, i, j))
        if not pairs:
            break
        _, i, j = min(pairs)
        groups[i] += groups.pop(j)
    result = []
    for members in groups:
        r = copy.deepcopy(members[0])
        lo, hi = bounds(members)
        xy = np.unique(np.concatenate([m['polygon'] for m in members]), axis=0)
        hull = ConvexHull(xy)
        r.update(lo=lo.tolist(), hi=hi.tolist(), polygon=xy[hull.vertices].tolist(),
                 center=((lo[:2]+hi[:2])/2).tolist(), area_m2=float(hull.volume),
                 member_region_ids=[m['region_id'] for m in members],
                 damage_patches=[m['polygon'] for m in members],
                 stable_updates=max(m['stable_updates'] for m in members),
                 damage_voxels=sum(m['damage_voxels'] for m in members),
                 support_voxels=sum(m.get('support_voxels', 0) for m in members),
                 inspection_targets=[p for m in members for p in m.get('inspection_targets', [])],
                 source='Grouped multi-view predicted damage; 8 m gap, 60 m maximum span')
        result.append(r)
    return result


def same_area(a, b):
    return bool(set(a.get('member_region_ids', [a['region_id']])) &
                set(b.get('member_region_ids', [b['region_id']])))
