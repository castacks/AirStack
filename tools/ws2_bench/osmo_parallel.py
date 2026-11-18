"""Resource-aware execution of independent paired OSMO flights.

The agent still waits for the entire pair before choosing its next experiment.
Each flight has a unique state directory, tunnel, result archive and run identity.
"""
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import fcntl
import hashlib
import math
from pathlib import Path
import re
import socket
import time

from osmo_transport import atomic_json, read_json
from operator_control import request as operator_request


def quantity(value):
    """Parse the Kubernetes quantities returned by OSMO, without rounding up."""
    match=re.fullmatch(r'([0-9]+(?:\.[0-9]+)?)([KMGT]i|[kMGT]|m)?',str(value))
    if not match:raise ValueError('Unrecognized resource quantity')
    factors={'':1,'m':.001,'k':1000,'M':10**6,'G':10**9,'T':10**12,
             'Ki':1024,'Mi':1024**2,'Gi':1024**3,'Ti':1024**4}
    return float(match[1])*factors[match[2] or '']


def capacity(resources,pools,settings):
    """Quota and physical availability are different; both must permit a job."""
    pool=next((p for group in pools.get('node_sets',[]) for p in group.get('pools',[])
               if p.get('name')==settings['pool']),None)
    if pool is None:raise ValueError('Pool absent from current quota response')
    quota=max(0,math.floor(quantity(pool['resource_usage']['quota_free'])))
    required={'gpu':1,'cpu':settings['cpu'],'memory':settings['memory_gib']*1024**3,
              'storage':settings['storage_gib']*1024**3}
    nodes=[];seen=set()
    for node in resources.get('resources',[]):
        name=node.get('hostname')
        if not name or name in seen:continue
        seen.add(name)
        free=node.get('platform_available_fields',{}).get(settings['pool'],{}).get(settings['platform'])
        if free is None:continue  # Never substitute a non-rendering platform.
        slots=max(0,min(math.floor(quantity(free[k])/v) for k,v in required.items()))
        nodes.append({'node':name,'available_jobs':slots})
    return {'available_jobs':min(quota,sum(n['available_jobs'] for n in nodes)),
            'quota_free':quota,'nodes':nodes,'checked_at':time.time()}


def current_capacity(client,settings):
    resources=client.call('resource','list','--pool',settings['pool'],'--platform',settings['platform'],
                          '--mode','free','--format-type','json')
    pools=client.call('pool','list','--pool',settings['pool'],'--format-type','json')
    return capacity(resources,pools,settings)


@contextmanager
def tunnel_port(runtime):
    """Reserve a distinct loopback port among our jobs, without a fixed port list."""
    locks=Path(runtime)/'remote_ports';locks.mkdir(exist_ok=True)
    for _ in range(50):
        with socket.socket() as probe:
            probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
            lock=(locks/f'{port}.lock').open('a')
            try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:lock.close();continue
        try:yield port
        finally:lock.close()
        return
    raise RuntimeError('Could not reserve a separate preview port')


def selected_state(runtime):
    """Resolve only a registered job, never a browser-supplied filesystem path."""
    runtime=Path(runtime);registry=read_json(runtime/'remote_parallel.json') or {}
    if not registry.get('active'):return None
    jobs=registry.get('jobs',[])
    selected=(read_json(runtime/'remote_selected.json') or {}).get('job_id')
    item=next((j for j in jobs if j['job_id']==selected),None)
    if item is None:
        def preview_rank(job):
            job_id=job.get('job_id','')
            if not re.fullmatch(r'[a-f0-9]{32}',job_id):return -1
            root=runtime/'remote_jobs'/job_id
            live=read_json(root/'remote_live.json') or {}
            snapshot=read_json(root/'remote_snapshot.json') or {}
            matching=live.get('active') and snapshot.get('run_id')==live.get('run_id')
            # Prefer a flight already producing frames over a queued/startup
            # job. Explicit user selection always takes precedence.
            frames=matching and bool(snapshot.get('view'))
            inference=matching and any((snapshot.get('inference') or {}).values())
            return (4*bool(frames)+4*bool(inference)+
                    {'planner':4,'paused':3,'takeoff':2,'model_loading':2,'startup':1}.get(job.get('phase'),0))
        running=[j for j in jobs if j.get('state')=='running']
        item=max(running,key=preview_rank) if running else None
    if item is None:item=next((j for j in jobs if j.get('state')!='finished'),None)
    if item is None:item=jobs[-1] if jobs else None
    if item is None:return None
    job_id=item['job_id']
    if not re.fullmatch(r'[a-f0-9]{32}',job_id):raise ValueError('Invalid registered job identity')
    return runtime/'remote_jobs'/job_id


def public_jobs(runtime):
    registry=read_json(Path(runtime)/'remote_parallel.json') or {}
    # This public registry never contains bearer tokens or credential paths.
    selected=(read_json(Path(runtime)/'remote_selected.json') or {}).get('job_id')
    mode='manual' if any(j.get('job_id')==selected for j in registry.get('jobs',[])) else 'auto'
    return {**{k:registry.get(k) for k in ('active','max_parallel','capacity','jobs','campaign')},
            'selection_mode':mode}


def unresolved_jobs(runtime):
    return [p for p in (Path(runtime)/'remote_jobs').glob('*') if
            (read_json(p/'remote_live.json') or {}).get('active') or (p/'osmo_pending_cleanup.json').exists()]


def run_pair(trials,round_root,planner,control_path,*,max_parallel=2,record_bag=False,
             publish=None,client=None,runtime=None,settings=None,runner=None,poll_seconds=5):
    from osmo_backend import Client, RUNTIME, load_settings, run_remote_episode
    from episode import fingerprint
    runtime=Path(runtime or RUNTIME);settings=settings or load_settings();client=client or Client()
    runner=runner or run_remote_episode
    if type(max_parallel) is not int or not 1<=max_parallel<=2:
        raise ValueError('A paired round has at most two independent flights')
    root=Path(round_root).resolve();root.relative_to(runtime.resolve())
    control_path=Path(control_path).resolve();control_path.relative_to(runtime.resolve())
    if (runtime/'osmo_pending_cleanup.json').exists():raise RuntimeError('Resolve pending OSMO cleanup first')
    results={};pending=[];jobs=[];running={};status={'available_jobs':0,'quota_free':0}
    for role,trial in trials:
        out=root/planner/role/'attempt_0'
        job_id=hashlib.sha256(str(out).encode()).hexdigest()[:32]
        item={'role':role,'job_id':job_id,'output':str(out),'state':'waiting'};jobs.append(item)
        saved=read_json(out/'result.json')
        if saved:
            if saved.get('configuration_hash')!=fingerprint(trial):raise ValueError('Saved trial config changed')
            results[role]=saved;item['state']='finished'
        elif out.exists():
            raise RuntimeError('Interrupted OSMO attempt; inspect its recorded job before resuming')
        else:pending.append((item,trial,out))
    def save(active=True):
        value={'active':active,'campaign':str(control_path.parent),'max_parallel':max_parallel,
               'capacity':status,'jobs':jobs,'wall_time':time.time()}
        atomic_json(runtime/'remote_parallel.json',value)
        atomic_json(root/'parallel.json',value)
    def not_executed(item,trial,out,reason,outcome):
        results[item['role']]={'outcome':outcome,'metrics':{},'executed':False,'result_dir':str(out),
            'configuration_hash':fingerprint(trial),'termination':{'source':'osmo_scheduler','reason':reason}}
        item.update(state='finished',outcome=outcome)
    def execute(item,trial,out):
        state=runtime/'remote_jobs'/item['job_id'];state.mkdir(parents=True,exist_ok=True)
        with tunnel_port(runtime) as port:
            return runner(trial,out,record_bag=record_bag,control_path=control_path,
                          state_dir=state,settings_override=dict(settings,port=port))
    runtime.mkdir(parents=True,exist_ok=True)
    with (runtime/'sequence.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if (read_json(runtime/'remote_live.json') or {}).get('active'):
            raise RuntimeError('A legacy remote flight is active')
        if unresolved_jobs(runtime):
            raise RuntimeError('Recover/cancel recorded isolated OSMO jobs before submitting')
        save();last_capacity=-math.inf;waiting_since=time.monotonic()
        with ThreadPoolExecutor(max_workers=max_parallel) as pool:
            try:
                while pending or running:
                    now=time.monotonic();choice=operator_request(control_path)
                    for future,item in list(running.items()):
                        state=runtime/'remote_jobs'/item['job_id']
                        live=read_json(state/'remote_live.json') or {}
                        if live.get('workflow_id'):item['workflow_id']=live['workflow_id']
                        episode=read_json(state/'live_episode.json') or {}
                        item['phase']=episode.get('phase')
                        if not future.done():continue
                        result=future.result();results[item['role']]=result
                        item.update(state='finished',outcome=result['outcome']);del running[future]
                        waiting_since=now;last_capacity=-math.inf
                        if (state/'osmo_pending_cleanup.json').exists():
                            atomic_json(runtime/'osmo_pending_cleanup.json',{'job_id':item['job_id'],
                                'workflow_id':item.get('workflow_id'),'reason':'Isolated job requires cleanup'})
                            raise RuntimeError('Remote job cleanup could not be confirmed')
                    if choice=='stop':
                        for item,trial,out in pending:not_executed(item,trial,out,'operator stopped before submission','user_stopped')
                        pending.clear()
                    if pending and choice=='run' and now-last_capacity>=poll_seconds:
                        try:status=current_capacity(client,settings)
                        except (OSError,RuntimeError,ValueError,KeyError) as exc:
                            status={'available_jobs':0,'quota_free':0,'error':str(exc)[:300],'checked_at':time.time()}
                        last_capacity=now
                        # Reserve not-yet-reflected submissions, avoiding double
                        # counting a free slot while a job is still queued.
                        queued=sum(1 for item in running.values() if not (read_json(
                            runtime/'remote_jobs'/item['job_id']/'remote_snapshot.json') or {}).get('run_id'))
                        count=min(max_parallel-len(running),max(0,status['available_jobs']-queued))
                        # One pending workflow allows OSMO to autoscale a new
                        # node even if its resource listing currently has none.
                        if queued==0 and len(running)<max_parallel and status['quota_free']>0:count=max(1,count)
                        for _ in range(min(count,len(pending))):
                            item,trial,out=pending.pop(0);item['state']='running'
                            running[pool.submit(execute,item,trial,out)]=item
                    if pending and not running and choice=='run' and now-waiting_since>settings['queue_minutes']*60:
                        for item,trial,out in pending:not_executed(item,trial,out,'Resource admission deadline exceeded','infrastructure_error')
                        pending.clear()
                    if choice=='pause':waiting_since=now
                    save()
                    if publish:publish('running' if running else 'waiting_for_resources',jobs)
                    if pending or running:time.sleep(min(1,poll_seconds))
            except BaseException:
                atomic_json(control_path,{'action':'stop'})
                raise
            finally:
                # Executor waits for all graceful stop/artifact cleanup before
                # the global sequence lock or active registry is released.
                for future in running:
                    try:future.result()
                    except Exception:pass
                save(False)
    return results
