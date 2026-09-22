"""Independent artifact checks for completed campaign trials and clean pairing."""
import argparse,json,math
from pathlib import Path
import yaml

def audit_episode(folder):
    folder=Path(folder);r=json.loads((folder/'result.json').read_text());c=json.loads((folder/'scenario.json').read_text())
    errors=[]
    if r['outcome']=='infrastructure_error':return {'directory':str(folder),'passed':False,'errors':['infrastructure error']}
    if r['cleanup_errors']:errors.append('cleanup errors')
    bag={'grounded_lead_in_sim_s':None}
    if r.get('bag_recorded',True):
        bag=json.loads((folder/'bag_verification.json').read_text())
        if not bag['passed']:errors.append('bag preflight/timing check failed')
    samples=json.loads((folder/'samples.json').read_text())
    if any(b['sim_time_s']<=a['sim_time_s'] for a,b in zip(samples,samples[1:])):errors.append('nonincreasing sample times')
    if r['outcome']=='goal_reached' and r['metrics']['final_distance_to_goal_m']>c['goal_radius']:errors.append('goal outside region')
    if r['outcome'] in ['timeout','completed_horizon','insufficient_progress'] and r['metrics']['mission_duration_sim_s']<c['timeout']:errors.append('early horizon termination')
    if c.get('mission_mode')=='avoidance' and r['outcome']=='goal_reached':errors.append('reactive model evaluated against a goal')
    if not r.get('validation_only') and not r.get('planner_command_count'):errors.append('no planner trajectory published')
    if r.get('bag_recorded',True):
        topics={x['topic_metadata']['name']:x['message_count'] for x in yaml.safe_load((folder/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']['topics_with_message_count']}
        for name in ['/ws2/ground_truth/pose','/ws2/ground_truth/oracle','/robot_1/sensors/front_stereo/left/image_rect','/vision_planner/planner_input/compressed']:
            if not topics.get(name):errors.append('missing recorded '+name)
    else:
        if len(samples)<2:errors.append('missing GT sample trace')
        if not (folder/'inference.jpg').exists():errors.append('missing actual inference image')
        else:
            preview=json.loads((folder/'inference.json').read_text())
            if preview['run_id']!=str(folder.resolve()):errors.append('inference belongs to another trial')
    return {'directory':str(folder),'outcome':r['outcome'],'passed':not errors,'errors':errors,'grounded_lead_in_sim_s':bag['grounded_lead_in_sim_s']}

def audit_campaign(root):
    root=Path(root);manifest=json.loads((root/'manifest.json').read_text());trials=[];pairs=[]
    for entry in manifest['trials']:
        for pair in entry['pairs']:
            clean,changed=[Path(pair[role]['result_dir']) for role in ['clean','perturbed']]
            trials.extend([audit_episode(clean),audit_episode(changed)])
            configs=[json.loads((f/'scenario.json').read_text()) for f in [clean,changed]]
            errors=[]
            for key in ['layout','layout_seed','seed','light','patch_size','patch_height']:
                if configs[0]['condition'][key]!=configs[1]['condition'][key]:errors.append('different scene '+key)
            for key in ['planner','height','goal_distance','goal_radius','timeout']:
                if configs[0][key]!=configs[1][key]:errors.append('different mission '+key)
            scene=[json.loads((f/'scene_status.json').read_text()) for f in [clean,changed]]
            if scene[0]['realized_layout']!=scene[1]['realized_layout']:errors.append('different realized geometry')
            first=[json.loads((f/'bag_verification.json').read_text())['first']['position'] for f in [clean,changed]]
            if math.dist(*first)>.01:errors.append('initial GT positions differ >1cm')
            pairs.append({'pair_id':pair['pair_id'],'passed':not errors,'errors':errors,'initial_position_difference_m':math.dist(*first)})
    return {'passed':all(x['passed'] for x in trials+pairs),'trials':trials,'pairs':pairs}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);a=p.parse_args();result=audit_campaign(a.campaign)
    (a.campaign/'artifact_audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)
