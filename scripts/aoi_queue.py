"""Inspection queue decisions independent of ROS and human ground truth."""
from group_aois import same_area
def disposition(region, assignments, bounds):
    if region['stable_updates'] < 2:
        return 'accumulating'
    x,y=region['center']
    if not(bounds[0]<=x<=bounds[2] and bounds[1]<=y<=bounds[3]):
        return 'outside_search_area'
    matches=[a for a in assignments if same_area(a,region)]
    if matches:
        return matches[-1].get('inspection_status','active')
    return 'pending'

def finish_assignment(aoi, ids, statuses):
    if not ids or any(statuses.get(i) not in ('completed','failed','cancelled') for i in ids):
        return False
    aoi['inspection_status']='completed' if all(statuses.get(i)=='completed' for i in ids) else 'failed'
    return True

def can_return(active, survey_drained, attempted_pending, budget_expired):
    return not active and (budget_expired or (survey_drained and attempted_pending))
