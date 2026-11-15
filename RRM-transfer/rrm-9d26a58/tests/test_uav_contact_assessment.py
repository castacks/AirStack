import json
from pathlib import Path
import subprocess
import sys

SCRIPT=Path(__file__).parents[1]/'scripts/assess_uav_contacts.py'


def assess(tmp_path,mode='good'):
    summary=dict(error=None,takeoff_action=True,land_action=True,hover_in_tolerance=True,fresh_final_state=True,final_disarmed=True,final_on_ground=True)
    events=[dict(wall_unix=float(t),phase=ph,kind='summary' if t==11 else 'phase',data=summary) for t,ph in [(0,'preflight'),(1,'takeoff'),(2,'hover'),(10,'land'),(11,'land')]]
    truth=[]
    for i in range(111):
        t=i/10
        z=.06 if t<1 else (.06+1.94*(t-1) if t<2 else (2. if t<10 else max(.06,2.-3*(t-10))))
        truth.append(dict(wall_time_unix_s=t,rigid_body_position_m=[0,0,z],rigid_body_linear_velocity_m_s=[0,0,0]))
    def contact(t):return dict(wall_unix=t,contact_count=1,actor0='/World/base_link/body',actor1='/World/stage/GroundPlane/CollisionPlane',impulse_magnitudes=[.1],separations_m=[0.])
    contacts=[contact(.2),contact(10.9)]
    bodies=['/World/base_link/'+n for n in ('body','rotor0','rotor1','rotor2','rotor3','ZEDCamera','Lidar')]
    status=[dict(wall_unix=float(i),source_sha256='test',subscribed=True,error=None,limit_reached=False,bodies=bodies,post_steps=i*100) for i in range(12)]
    if mode=='short_hover':events[3]['wall_unix']=3.
    if mode=='collision':contacts.append(contact(2.5))
    if mode=='rotor_contact':
        row=contact(10.9);row['actor0']='/World/base_link/rotor0';contacts.append(row)
    if mode=='late_status':status=status[3:]
    if mode=='source_change':status[2]['source_sha256']='other'
    if mode=='limit':status[2]['limit_reached']=True
    if mode in ('camera_proximity','camera_penetration','camera_force'):
        row=contact(10.9);row['actor0']='/World/base_link/ZEDCamera';row['separations_m']=[-.002 if mode=='camera_penetration' else .01];row['impulse_magnitudes']=[.01 if mode=='camera_force' else 0.];contacts.append(row)
    if mode=='missing_separation':contacts[1].pop('separations_m')
    if mode=='body_penetration':contacts[1]['separations_m']=[-.005]
    if mode=='bad_hover':truth[25]['rigid_body_position_m'][2]=.5
    files={}
    for name,values in [('events',events),('truth',truth),('contacts',contacts),('status-history',status)]:
        p=tmp_path/(name+'.jsonl');p.write_text(''.join(json.dumps(x)+'\n' for x in values));files[name]=p
    output=tmp_path/'assessment.json'
    command=[sys.executable,str(SCRIPT),'--output',str(output)]
    for name,p in files.items():command.extend(['--'+name,str(p)])
    subprocess.run(command,check=True,capture_output=True,text=True)
    return json.loads(output.read_text())


def test_complete_evidence_passes(tmp_path):assert assess(tmp_path)['verdict']=='PASS'
def test_midair_contact_fails_even_if_action_succeeds(tmp_path):assert assess(tmp_path,'collision')['verdict']=='FAIL'
def test_late_status_cannot_certify_complete_flight(tmp_path):assert assess(tmp_path,'late_status')['verdict']=='NOT DEMONSTRATED'
def test_changed_reporter_source_blocks_pass(tmp_path):assert assess(tmp_path,'source_change')['verdict']=='NOT DEMONSTRATED'
def test_contact_limit_blocks_pass(tmp_path):assert assess(tmp_path,'limit')['verdict']=='NOT DEMONSTRATED'
def test_physical_hover_violation_fails(tmp_path):assert assess(tmp_path,'bad_hover')['verdict']=='FAIL'

def test_short_hover_cannot_pass_successful_action_flags(tmp_path):assert assess(tmp_path,"short_hover")["verdict"]=="FAIL"

def test_rotor_floor_strike_fails_even_on_landing(tmp_path):assert assess(tmp_path,"rotor_contact")["verdict"]=="FAIL"

def test_separated_camera_proximity_is_not_impact(tmp_path):
    r=assess(tmp_path,'camera_proximity');assert r['verdict']=='PASS' and r['collision'] is False and r['proximity_headers']==1

def test_camera_penetration_fails(tmp_path):assert assess(tmp_path,'camera_penetration')['verdict']=='FAIL'
def test_camera_force_fails_even_if_separated(tmp_path):assert assess(tmp_path,'camera_force')['verdict']=='FAIL'
def test_missing_separation_cannot_be_certified(tmp_path):assert assess(tmp_path,'missing_separation')['verdict']=='NOT DEMONSTRATED'
def test_excess_body_penetration_fails(tmp_path):assert assess(tmp_path,'body_penetration')['verdict']=='FAIL'
