#!/usr/bin/env python3
"""Independent bounded vertical-flight assessment using PhysX pose and contacts.
Ground actors are explicitly scoped to the qualified default fixture environment.
Wall receipt pairing (150ms) is not sensor-acquisition synchronization.
"""
import argparse,bisect,json,math,statistics,hashlib
from pathlib import Path


def records(path):return [json.loads(s) for s in path.read_text().splitlines()]


def classify_contact(c, phase, relative_height):
    """Separate near-contact manifolds from support and forbidden interaction.

    Positive separation with zero impulse is proximity, not evidence of impact.
    Only body/floor support and grounded camera/floor proximity are declared here.
    """
    n=c['contact_count'];separations=c.get('separations_m');impulses=c.get('impulse_magnitudes')
    if (not isinstance(separations,list) or not isinstance(impulses,list) or len(separations)!=n or len(impulses)!=n or
        not all(type(x) in (int,float) and math.isfinite(x) for x in separations+impulses) or any(x<0 for x in impulses)):
        return dict(status='UNKNOWN',reason='missing_invalid_contact_geometry',physical_interaction=None)
    actors=(c['actor0'],c['actor1']);floor={'/World/stage/Environment/Geometry','/World/stage/GroundPlane/CollisionPlane'}
    other=[a for a in actors if not a.startswith('/World/base_link')]
    grounded_phase=phase in ('preflight','takeoff','land','recovery_land') and relative_height<=.25
    physical=min(separations)<=0 or max(impulses)>1e-9
    if len(other)==1 and other[0] in floor and grounded_phase:
        if '/World/base_link/body' in actors and min(separations)>=-.001:
            return dict(status='SUPPORT' if physical else 'PROXIMITY',reason='declared_body_floor',physical_interaction=physical)
        if '/World/base_link/ZEDCamera' in actors and min(separations)>0 and max(impulses)<=1e-9:
            return dict(status='PROXIMITY',reason='separated_camera_floor_zero_impulse',physical_interaction=False)
    return dict(status='VIOLATION',reason='undeclared_contact_or_clearance',physical_interaction=physical)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for key in ('events','truth','contacts','status-history','output'):ap.add_argument('--'+key,required=True,type=Path)
    args=ap.parse_args()
    if args.output.exists():ap.error('output must be new')
    events=records(args.events);truth=records(args.truth);contacts=records(args.contacts);history=records(args.status_history)
    summaries=[e['data'] for e in events if e['kind']=='summary']
    if not summaries:ap.error('probe incomplete')
    summary=summaries[-1];et=[e['wall_unix'] for e in events];start,end=et[0],et[-1]
    tt=[t['wall_time_unix_s'] for t in truth]
    if not truth or any(b<=a for a,b in zip(tt,tt[1:])):ap.error('missing/nonmonotonic physical truth')
    def phase(wall):return events[max(0,bisect.bisect_right(et,wall)-1)]['phase']
    selected=[t for t in truth if start<=t['wall_time_unix_s']<=end]
    ground=[t['rigid_body_position_m'] for t in selected if phase(t['wall_time_unix_s'])=='preflight']
    if not ground:ap.error('no preflight physical ground reference')
    origin=[statistics.median(p[i] for p in ground) for i in range(3)]
    hover=[t for t in selected if phase(t['wall_time_unix_s'])=='hover']
    expected_ground={'/World/stage/Environment/Geometry','/World/stage/GroundPlane/CollisionPlane'}
    unexpected=[];unknown=[];proximity=0;unpaired=0;contact_counts={};positive=0;floor_landing=0
    for c in contacts:
        wall=c['wall_unix']
        if not start<=wall<=end or c['contact_count']<=0:continue
        positive+=1;i=bisect.bisect_left(tt,wall);candidates=truth[max(0,i-1):min(len(truth),i+1)]
        t=min(candidates,key=lambda t:abs(t['wall_time_unix_s']-wall))
        if abs(t['wall_time_unix_s']-wall)>.15:unpaired+=1;continue
        ph=phase(wall);contact_counts[ph]=contact_counts.get(ph,0)+1
        classification=classify_contact(c,ph,t['rigid_body_position_m'][2]-origin[2])
        detail=dict(wall=wall,phase=ph,actors=[c['actor0'],c['actor1']],z=t['rigid_body_position_m'][2],**classification)
        if classification['status']=='VIOLATION':unexpected.append(detail)
        elif classification['status']=='UNKNOWN':unknown.append(detail)
        elif classification['status']=='PROXIMITY':proximity+=1
        elif ph=='land' and classification['status']=='SUPPORT':floor_landing+=1

    hs=[h for h in history if start-2<=h['wall_unix']<=end+2]
    expected_bodies={'/World/base_link/'+x for x in ('body','rotor0','rotor1','rotor2','rotor3','ZEDCamera','Lidar')}
    history_ok=(bool(hs) and len({h['source_sha256'] for h in hs})==1 and hs[0]['wall_unix']<=start+2 and hs[-1]['wall_unix']>=end-2 and
        all(h['subscribed'] and h['error'] is None and not h['limit_reached'] and expected_bodies.issubset(h['bodies']) for h in hs) and
        all(0<b['wall_unix']-a['wall_unix']<=2 and b['post_steps']>a['post_steps'] for a,b in zip(hs,hs[1:])))
    p=[t['rigid_body_position_m'] for t in selected];speeds=[math.sqrt(sum(v*v for v in t['rigid_body_linear_velocity_m_s'])) for t in selected]
    max_radius=max(math.hypot(x[0]-origin[0],x[1]-origin[1]) for x in p)
    truth_gaps=[b['wall_time_unix_s']-a['wall_time_unix_s'] for a,b in zip(selected,selected[1:])]
    checks=dict(procedure_completed=summary.get('error') is None and summary.get('takeoff_action') is True and summary.get('land_action') is True and summary.get('hover_in_tolerance') is True,
        final_fresh_disarmed_grounded=all(summary.get(k) is True for k in ('fresh_final_state','final_disarmed','final_on_ground')),
        physical_takeoff=max(x[2] for x in p)>=1.7,
        physical_hover=bool(hover) and hover[-1]['wall_time_unix_s']-hover[0]['wall_time_unix_s']>=7.5 and all(abs(t['rigid_body_position_m'][2]-2)<=.3 for t in hover),
        physical_envelope=max_radius<=2 and max(x[2]-origin[2] for x in p)<=3 and min(x[2]-origin[2] for x in p)>=-.3 and max(speeds)<=2,
        physical_landing=abs(p[-1][2]-origin[2])<=.1 and speeds[-1]<=.05,
        expected_landing_contacts=floor_landing>0,
        contact_stream_complete=history_ok and unpaired==0 and positive>0 and not unknown,
        no_unexpected_contacts=not unexpected,
        truth_complete=bool(truth_gaps) and max(truth_gaps)<=.5 and selected[0]['wall_time_unix_s']-start<=.15 and end-selected[-1]['wall_time_unix_s']<=.15)
    physical_fail=any(not checks[k] for k in ('physical_takeoff','physical_hover','physical_envelope','physical_landing','no_unexpected_contacts'))
    verdict='PASS' if all(checks.values()) else ('FAIL' if physical_fail or not checks['procedure_completed'] else 'NOT DEMONSTRATED')
    result=dict(verdict=verdict,scope='one bounded vertical procedure, default marker scene, receipt-paired PhysX evidence; not broad autonomy qualification',checks=checks,
        ground_reference_m=origin,max_horizontal_radius_m=max_radius,max_physical_speed_m_s=max(speeds),
        min_physical_z_m=min(x[2] for x in p),max_physical_z_m=max(x[2] for x in p),
        observed_hover_span_wall_s=(hover[-1]['wall_time_unix_s']-hover[0]['wall_time_unix_s']) if hover else 0,positive_contact_headers=positive,contact_headers_by_phase=contact_counts,unpaired_contact_headers=unpaired,
        contact_policy='uav-floor-contacts-v2: body penetration tolerance1mm, camera strictly separated/zero impulse near floor only',
        proximity_headers=proximity,unknown_contact_headers=len(unknown),unexpected_contacts=unexpected,source_hashes=sorted({h['source_sha256'] for h in hs}),status_samples=len(hs),
        max_truth_receipt_gap_s=max(truth_gaps,default=None),collision=any(x['physical_interaction'] for x in unexpected) if checks['contact_stream_complete'] else None)
    result['assessor_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    result['input_sha256']={name:hashlib.sha256(getattr(args,name).read_bytes()).hexdigest() for name in ('events','truth','contacts','status_history')}
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':main()
