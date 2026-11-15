import json
from pathlib import Path
import subprocess
import sys

SCRIPT=Path(__file__).parents[1]/'scripts/assess_uav_stop.py'


def assess(tmp_path,mode='good'):
    events=[dict(kind='supervisory_pause',wall_unix=10.),dict(kind='controller_log',wall_unix=10.05,data=dict(name='robot_1.drone_safety_monitor',msg='Safety command: PAUSE')),
        dict(kind='action_result',phase='navigate_viewpoint',status=5,wall_unix=11.5),dict(kind='summary',wall_unix=20,data=dict(final_disarmed=True,final_on_ground=True))]
    truth=[]
    for i in range(-1,61):
        d=i/10;v=.2 if d<0 else max(0.,.2-.4*d);x=0. if d<0 else (.2*d-.2*d*d if d<.5 else .05)
        if mode=='no_stop':v=.2;x=.2*d
        if mode=='distance' and d>.1:x=1.
        if mode=='stationary':v=0.
        truth.append(dict(wall_time_unix_s=10.+d,rigid_body_position_m=[x,0,2.],rigid_body_linear_velocity_m_s=[v,0,0]))
    contact=dict(verdict='PASS')
    automatic=mode.startswith('automatic')
    if automatic:
        events[0]['kind']='state_fault_injected'
        events[1]['data']=dict(name='robot_1.watchdog_qualification.drone_safety_monitor',msg='State health: stale_source_stamp')
        events[-1]['data'].update(automatic_state_fault_stop=True,qualification_monitor_stopped=True,pending_goal_unreconciled=False)
        events.extend([dict(kind='fault_relay',wall_unix=10.02,injected=True,original_source_s=20.,forwarded_source_s=10.),
                       dict(kind='fault_ack',wall_unix=10.1,data=dict(data=False)),
                       dict(kind='fault_ack',wall_unix=10.3,data=dict(data=True)),
                       dict(kind='fault_watchdog',wall_unix=10.1,data=dict(data=True))])
        if mode=='automatic_no_detection':events[1]['data']['msg']='unrelated log'
        if mode=='automatic_no_ack':events=[e for e in events if not(e['kind']=='fault_ack' and e['data']['data'])]
        if mode=='automatic_no_relay':events=[e for e in events if e['kind']!='fault_relay']
        if mode=='automatic_manual_pause':events.append(dict(kind='supervisory_pause',wall_unix=10.05))
        if mode=='automatic_unreconciled':events[3]['data']['pending_goal_unreconciled']=True
    if mode=='missing_truth':truth=[]
    if mode=='missing_monitor':events.pop(1)
    if mode=='not_canceled':events[2]['status']=4
    if mode=='incomplete_contacts':contact['verdict']='NOT DEMONSTRATED'
    files={}
    for name,values in [('events',events),('truth',truth)]:
        p=tmp_path/(name+'.jsonl');p.write_text(''.join(json.dumps(x)+'\n' for x in values));files[name]=p
    p=tmp_path/'contact.json';p.write_text(json.dumps(contact));files['contact-assessment']=p
    out=tmp_path/'assessment.json';cmd=[sys.executable,str(SCRIPT),'--output',str(out)]
    if automatic:cmd.append('--automatic-state-fault')
    for name,p in files.items():cmd.extend(['--'+name,str(p)])
    subprocess.run(cmd,check=True,capture_output=True,text=True)
    return json.loads(out.read_text())


def test_physical_stop_and_cancel_pass(tmp_path):assert assess(tmp_path)['verdict']=='PASS'
def test_stationary_vehicle_does_not_demonstrate_braking(tmp_path):assert assess(tmp_path,'stationary')['verdict']=='NOT DEMONSTRATED'
def test_missing_physical_evidence_unknown(tmp_path):assert assess(tmp_path,'missing_truth')['verdict']=='NOT DEMONSTRATED'
def test_timeout_with_complete_physics_fails(tmp_path):assert assess(tmp_path,'no_stop')['verdict']=='FAIL'
def test_stopping_distance_violation_fails(tmp_path):assert assess(tmp_path,'distance')['verdict']=='FAIL'
def test_command_send_without_monitor_receipt_unknown(tmp_path):assert assess(tmp_path,'missing_monitor')['verdict']=='NOT DEMONSTRATED'
def test_goal_success_is_not_stop_cancellation(tmp_path):assert assess(tmp_path,'not_canceled')['verdict']=='NOT DEMONSTRATED'
def test_contact_gap_blocks_pass(tmp_path):assert assess(tmp_path,'incomplete_contacts')['verdict']=='NOT DEMONSTRATED'
def test_automatic_fault_stop_pass(tmp_path):assert assess(tmp_path,'automatic_good')['verdict']=='PASS'
def test_automatic_needs_detection(tmp_path):assert assess(tmp_path,'automatic_no_detection')['verdict']=='NOT DEMONSTRATED'
def test_automatic_needs_new_ack(tmp_path):assert assess(tmp_path,'automatic_no_ack')['verdict']=='NOT DEMONSTRATED'
def test_automatic_needs_input_proof(tmp_path):assert assess(tmp_path,'automatic_no_relay')['verdict']=='NOT DEMONSTRATED'
def test_manual_pause_invalidates_automatic_claim(tmp_path):assert assess(tmp_path,'automatic_manual_pause')['verdict']=='NOT DEMONSTRATED'
def test_automatic_requires_terminal_reconciliation(tmp_path):assert assess(tmp_path,'automatic_unreconciled')['verdict']=='NOT DEMONSTRATED'
def test_automatic_metrics_name_injection_time_origin(tmp_path):
    result=assess(tmp_path,'automatic_good')
    assert result['checks']['one_fault_injection'] is True
    assert 'one_pause_command' not in result['checks']
    assert result['metrics']['physical_speed_at_fault_injection_m_s']>.1
    assert 'physical_speed_at_pause_m_s' not in result['metrics']
