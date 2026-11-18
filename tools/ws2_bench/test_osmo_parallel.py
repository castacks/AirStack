import json
from pathlib import Path
import sys
import threading
import time

import pytest
sys.path.insert(0,str(Path(__file__).resolve().parent))
import osmo_parallel as parallel
from osmo_transport import atomic_json,read_json
from episode import fingerprint,resolved
from campaign import clean_twin


def resources(gpus=2,cpu='12',memory='96Gi',storage='160Gi'):
    node={'hostname':'rtx-node','platform_available_fields':{'team':{'rtx':{
        'gpu':gpus,'cpu':cpu,'memory':memory,'storage':storage}}}}
    return {'resources':[node]}


def pools(quota=12):
    return {'node_sets':[{'pools':[{'name':'team','resource_usage':{'quota_free':str(quota)}}]}]}


@pytest.fixture
def settings():
    return {'pool':'team','platform':'rtx','cpu':6,'memory_gib':48,'storage_gib':80,
            'queue_minutes':1,'port':8894}


def test_capacity_uses_per_node_cpu_memory_disk_and_quota(settings):
    assert parallel.capacity(resources(),pools(),settings)['available_jobs']==2
    assert parallel.capacity(resources(),pools(1),settings)['available_jobs']==1
    assert parallel.capacity(resources(cpu='11900m'),pools(),settings)['available_jobs']==1
    assert parallel.capacity(resources(memory='47Gi'),pools(),settings)['available_jobs']==0
    assert parallel.capacity(resources(storage='79Gi'),pools(),settings)['available_jobs']==0
    r=resources();r['resources']*=2  # A node listed under an alias is not two nodes.
    assert parallel.capacity(r,pools(),settings)['available_jobs']==2
    settings['platform']='a100'
    assert parallel.capacity(resources(),pools(),settings)['available_jobs']==0


class Client:
    def __init__(self,free=2):self.free=free
    def call(self,*args,**kwargs):return resources(self.free) if args[0]=='resource' else pools()


def setup_pair(tmp_path):
    import yaml
    scenario=resolved(yaml.safe_load((Path(__file__).parent/'scenarios/corridor_pulp.yaml').read_text()))
    control=tmp_path/'campaign/operator_control.json';atomic_json(control,{'action':'run'})
    return [('clean',clean_twin(scenario)),('perturbed',scenario)],control


def test_actual_concurrent_pair_has_separate_ports_state_and_results(tmp_path,settings):
    trials,control=setup_pair(tmp_path);barrier=threading.Barrier(2);seen=[]
    def runner(trial,out,**kwargs):
        state=kwargs['state_dir'];port=kwargs['settings_override']['port']
        seen.append((state,port));out.mkdir(parents=True)
        atomic_json(state/'remote_live.json',{'active':True,'token':'secret','run_id':out.parent.name})
        atomic_json(state/'remote_snapshot.json',{'run_id':out.parent.name})
        barrier.wait(timeout=4)
        result={'outcome':'route_completed','metrics':{},'configuration_hash':fingerprint(trial),'result_dir':str(out)}
        atomic_json(state/'remote_live.json',{'active':False})
        atomic_json(out/'result.json',result);return result
    result=parallel.run_pair(trials,control.parent/'round_01','pulp_dronet',control,
        runtime=tmp_path,settings=settings,client=Client(),runner=runner,poll_seconds=.01)
    assert set(result)=={'clean','perturbed'}
    assert len({s for s,_ in seen})==len({p for _,p in seen})==2
    public=parallel.public_jobs(tmp_path)
    assert not public['active'] and 'secret' not in json.dumps(public)
    assert not (tmp_path/'remote_live.json').exists()
    # Resume consumes the exact saved pair without scheduling another job.
    parallel.run_pair(trials,control.parent/'round_01','pulp_dronet',control,
        runtime=tmp_path,settings=settings,client=Client(),runner=lambda *a,**k:pytest.fail('duplicate'))


def test_no_free_node_admits_only_one_queued_job(tmp_path,settings):
    trials,control=setup_pair(tmp_path);active=0;peak=0;lock=threading.Lock()
    def runner(trial,out,**kwargs):
        nonlocal active,peak
        with lock:active+=1;peak=max(peak,active)
        time.sleep(.04);out.mkdir(parents=True)
        result={'outcome':'route_completed','metrics':{},'configuration_hash':fingerprint(trial),'result_dir':str(out)}
        atomic_json(out/'result.json',result)
        with lock:active-=1
        return result
    parallel.run_pair(trials,control.parent/'round_01','pulp_dronet',control,
        runtime=tmp_path,settings=settings,client=Client(0),runner=runner,poll_seconds=.01)
    assert peak==1


def test_stop_never_submits_pending_flights(tmp_path,settings):
    trials,control=setup_pair(tmp_path);atomic_json(control,{'action':'stop'})
    result=parallel.run_pair(trials,control.parent/'round_01','pulp_dronet',control,
        runtime=tmp_path,settings=settings,client=Client(),runner=lambda *a,**kw:pytest.fail('submitted'))
    assert all(not r['executed'] and r['outcome']=='user_stopped' for r in result.values())
    assert not list(control.parent.glob('round_*/*/*/attempt_*'))


def test_parallel_viewer_uses_selected_job_and_same_run_only(tmp_path,monkeypatch):
    import dashboard
    monkeypatch.setattr(dashboard,'RUNTIME',tmp_path);monkeypatch.setenv('WS2_EXECUTION_BACKEND','osmo')
    jobs=[{'job_id':char*32,'role':role,'state':'running'} for char,role in [('a','clean'),('b','perturbed')]]
    atomic_json(tmp_path/'remote_parallel.json',{'active':True,'jobs':jobs})
    for job in jobs:
        state=tmp_path/'remote_jobs'/job['job_id']
        atomic_json(state/'remote_live.json',{'active':True,'run_id':job['role'],'token':'hidden'})
        atomic_json(state/'remote_snapshot.json',{'run_id':job['role'],'scene':{'condition':job['role']}})
    assert dashboard.flight_view()['scene']['condition']=='clean'
    atomic_json(tmp_path/'remote_selected.json',{'job_id':'b'*32})
    assert dashboard.flight_view()['scene']['condition']=='perturbed'
    atomic_json(tmp_path/'remote_jobs'/('b'*32)/'remote_snapshot.json',{'run_id':'clean','scene':{'wrong':True}})
    assert dashboard.flight_view().get('scene') is None
    atomic_json(tmp_path/'remote_selected.json',{'job_id':'../../escape'})
    assert dashboard.flight_view()['scene']['condition']=='clean'


def test_auto_preview_prefers_live_frames_but_respects_manual_selection(tmp_path):
    jobs=[dict(job_id='a'*32,role='clean',state='running',phase='startup'),
          dict(job_id='b'*32,role='perturbed',state='running',phase='planner')]
    atomic_json(tmp_path/'remote_parallel.json',dict(active=True,jobs=jobs))
    for job in jobs:
        root=tmp_path/'remote_jobs'/job['job_id']
        atomic_json(root/'remote_live.json',dict(active=True,run_id=job['role']))
        atomic_json(root/'remote_snapshot.json',dict(run_id=job['role'],
            view={'sim_time':10} if job['role']=='perturbed' else None))
    assert parallel.selected_state(tmp_path).name=='b'*32
    assert parallel.public_jobs(tmp_path)['selection_mode']=='auto'
    atomic_json(tmp_path/'remote_selected.json',dict(job_id='a'*32))
    assert parallel.selected_state(tmp_path).name=='a'*32
    assert parallel.public_jobs(tmp_path)['selection_mode']=='manual'
    atomic_json(tmp_path/'remote_selected.json',dict(job_id='auto'))
    assert parallel.selected_state(tmp_path).name=='b'*32


def test_query_failure_does_not_cancel_a_healthy_workflow(tmp_path,monkeypatch):
    import osmo_backend as backend
    import osmo_transport as transport
    import socket
    import yaml
    trials,_=setup_pair(tmp_path)
    config={'pool':'team','platform':'rtx','registry_credential':'registry','data_credential':'data',
        'output_uri':'s3://test/ws2','images':{'pulp_dronet':'registry/model@sha256:'+'a'*64}}
    path=tmp_path/'settings.yaml';path.write_text(yaml.safe_dump(config));s=backend.load_settings(path)
    s['poll_seconds']=0
    with socket.socket() as sock:sock.bind(('127.0.0.1',0));s['port']=sock.getsockname()[1]
    monkeypatch.setattr(backend,'RUNTIME',tmp_path)
    monkeypatch.setattr(backend,'request',lambda *a,**k:(_ for _ in ()).throw(OSError('starting')))
    class Fake:
        def submit(self,path):
            self.payload=json.loads(yaml.safe_load(path.read_text())['workflow']['tasks'][0]['files'][0]['contents'])
            self.queries=0;return 'test-job'
        def status(self,name):
            self.queries+=1
            if self.queries==1:raise RuntimeError('temporary CLI exit 10')
            return 'COMPLETED',{'status':'COMPLETED'}
        def tunnel(self,*a):return None
        def cancel(self,*a):pytest.fail('must not cancel on one failed read')
        def download(self,uri,path):
            source=tmp_path/'remote';source.mkdir();remote='/remote/episode'
            transport.atomic_json(source/'result.json',{'outcome':'route_completed','configuration_hash':self.payload['configuration_hash'],'result_dir':remote})
            receipt=transport.pack_artifacts(source,path,self.payload['run_id'],self.payload['configuration_hash'])
            receipt['remote_episode_dir']=remote;transport.atomic_json(path/'receipt.json',receipt)
    out=tmp_path/'result'
    result=backend.run_remote_episode(trials[0][1],out,client=Fake(),settings_override=s,state_dir=tmp_path/'job')
    assert result['outcome']=='route_completed'
    assert read_json(out/'osmo/query_error.json')['consecutive_failures']==1
    assert not (tmp_path/'remote_live.json').exists()


def test_web_parallel_mode_requires_compatible_controls(tmp_path,monkeypatch):
    import dashboard
    monkeypatch.setenv('WS2_EXECUTION_BACKEND','osmo')
    payload=dict(planner='pulp_dronet',action_space='corridor',policy='random',budget=8,
                 max_parallel=2,clean_validation_runs=0,infrastructure_retries=0,clean_failure_policy='record')
    command=dashboard.adaptive_command(payload,tmp_path/'campaign')
    assert command[command.index('--max-parallel')+1]=='2'
    payload['infrastructure_retries']=1
    with pytest.raises(ValueError,match='Parallel'):dashboard.adaptive_command(payload,tmp_path/'campaign')


def test_campaign_accounts_both_results_after_clean_failure(tmp_path,monkeypatch):
    import agent_campaign,episode
    monkeypatch.setenv('WS2_EXECUTION_BACKEND','osmo')
    monkeypatch.setattr(agent_campaign,'RUNTIME',tmp_path)
    monkeypatch.setattr(episode,'pin_execution',lambda *a:None)
    calls=[]
    def pair(trials,root,planner,control,**kwargs):
        calls.append(trials);results={}
        for role,config in trials:
            out=root/planner/role/'attempt_0';out.mkdir(parents=True)
            result=dict(outcome='collision',metrics={},result_dir=str(out),configuration_hash=fingerprint(config))
            atomic_json(out/'result.json',result);atomic_json(out/'scenario.json',config)
            results[role]=result
        return results
    monkeypatch.setattr(parallel,'run_pair',pair)
    monkeypatch.setattr(agent_campaign,'run_episode',lambda *a,**k:pytest.fail('sequential duplicate'))
    root=tmp_path/'campaign'
    history=agent_campaign.run_adaptive_campaign(root,'random',budget=2,planner='pulp_dronet',
        retries=0,clean_validation_runs=0,clean_failure_policy='record',action_space='corridor',
        camera_programs=True,max_parallel=2,llm_report=False,pause_seconds=0)
    assert len(calls)==1
    measured=history[0]['pairs'][0]
    assert measured['clean']['executed'] and measured['perturbed']['executed']
    assert not measured['factor_exposure']['eligible']  # Failed control cannot establish attack effect.
    assert read_json(root/'report.json')['actual_attempts']==2
    assert read_json(root/'presentation.json')['completed']==2
