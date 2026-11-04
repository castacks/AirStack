import io
import json
from pathlib import Path
import sys
import tarfile
import threading
from http.server import ThreadingHTTPServer

import pytest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parent))
import osmo_backend as backend
import osmo_transport as transport
import runtime_process
from episode import resolved, fingerprint, worker_command


@pytest.fixture
def settings(tmp_path):
    path=tmp_path/'config.yaml'
    value={'pool':'adversialai','platform':'rtxpro5000','registry_credential':'airlab-harbor-cred',
           'data_credential':'dsta-adv','output_uri':'s3://example/ws2/episodes',
           'images':{p:'registry.example/project/'+p+'@sha256:'+'a'*64 for p in ('mononav','kim','pulp_dronet')}}
    path.write_text(yaml.safe_dump(value))
    return backend.load_settings(path)


def scenario(planner='pulp_dronet'):
    names={'pulp_dronet':'pulp','mononav':'mononav','kim':'kim'}
    return resolved(yaml.safe_load((backend.HERE/'scenarios'/('corridor_'+names[planner]+'.yaml')).read_text()))


def test_one_task_one_gpu_no_credentials_or_docker_socket(settings):
    payload,spec,destination=backend.prepare(scenario(),settings,'a'*32,record_video=True)
    workflow=spec['workflow'];task=workflow['tasks'][0]
    assert len(workflow['tasks'])==1 and workflow['resources']['default']['gpu']==1
    assert task['command']==['/usr/bin/python3'] and 'imagePullCredentials' not in task
    assert task['exitActions']['FAIL']=='1-255,1001,3000-4000'
    assert task['outputs']==[{'url':destination}]
    assert payload['configuration_hash']==fingerprint(payload['scenario'])
    serialized=json.dumps(spec)
    assert 'docker.sock' not in serialized and 'hostPath' not in serialized
    assert 'credentials' not in task and 'privileged' not in task
    assert 'API_KEY' not in serialized and '.codex' not in serialized


@pytest.mark.parametrize('planner',['mononav','pulp_dronet'])
def test_remote_patch_scope_unchanged(settings,planner):
    c=scenario(planner);c['condition']['patch_enabled']=True
    with pytest.raises(ValueError,match='FCRN'):backend.prepare(c,settings,'a'*32)


def test_account_check_uses_current_users_pool_and_credential_labels(settings):
    class Account(backend.Client):
        def call(self, *args, **kwargs):
            if args[0] == 'credential':
                return {'credentials': [
                    {'cred_name': settings['data_credential'], 'cred_type': 'DATA'},
                    {'cred_name': settings['registry_credential'], 'cred_type': 'REGISTRY'}]}
            return {'pools': ['different-pool'], 'profile': {}, 'token': 'must-not-be-logged'}
    settings['data_credential'] = 'colleague-storage'
    settings['registry_credential'] = 'colleague-registry'
    with pytest.raises(ValueError, match='cannot access pool') as error:
        Account().check_access(settings)
    assert 'must-not-be-logged' not in str(error.value)
    settings['pool'] = 'different-pool'
    Account().check_access(settings)


def test_campaign_cannot_mix_execution_backend_or_image(tmp_path, monkeypatch, settings):
    from episode import pin_execution
    config = tmp_path/'osmo.yaml'
    config.write_text(yaml.safe_dump(settings))
    monkeypatch.setenv('WS2_OSMO_CONFIG', str(config))
    monkeypatch.setenv('WS2_EXECUTION_BACKEND', 'osmo')
    root = tmp_path/'campaign'; root.mkdir()
    pin_execution(root, ['kim'])
    pin_execution(root, ['kim'])
    settings['images']['kim'] = 'registry.example/kim@sha256:' + 'f'*64
    config.write_text(yaml.safe_dump(settings))
    with pytest.raises(ValueError, match='Execution backend/image/resources changed'):
        pin_execution(root, ['kim'])
    monkeypatch.setenv('WS2_EXECUTION_BACKEND', 'local')
    with pytest.raises(ValueError, match='Execution backend/image/resources changed'):
        pin_execution(root, ['kim'])
    legacy = tmp_path/'legacy'; legacy.mkdir(); (legacy/'config.json').write_text('{}')
    pin_execution(legacy, ['kim'])
    assert not (legacy/'execution.json').exists()
    monkeypatch.setenv('WS2_EXECUTION_BACKEND', 'osmo')
    with pytest.raises(ValueError, match='Legacy campaign'):
        pin_execution(legacy, ['kim'])


def test_pinned_images_required_and_office_explicitly_unsupported(settings):
    settings['images']['pulp_dronet']='registry.example/project/model:latest'
    with pytest.raises(ValueError,match='digest'):backend.prepare(scenario(),settings,'a'*32)
    with pytest.raises(ValueError,match='Office'):backend.prepare({'planner':'kim'},settings,'a'*32)


@pytest.mark.parametrize('field,value',[('cpu',11),('memory_gib',51),('exec_minutes',181),('port',True)])
def test_lab_limits(settings,tmp_path,field,value):
    settings[field]=value;path=tmp_path/'invalid.yaml';path.write_text(yaml.safe_dump(settings))
    with pytest.raises(ValueError):backend.load_settings(path)


def test_process_worker_keeps_target_arguments(monkeypatch):
    for model in ('mononav','kim','pulp_dronet'):
        c=scenario(model);monkeypatch.setenv('WS2_EXECUTION_BACKEND','local');local=worker_command(c)
        monkeypatch.setenv('WS2_EXECUTION_BACKEND','process');remote=worker_command(c)
        assert remote[0]=='/opt/ws2/model/bin/python'
        assert local[local.index('--server')+2:]==remote[remote.index('--server')+2:]
        assert '--gpus' not in remote and remote[remote.index('--server')+1]=='http://127.0.0.1:8765'


def test_process_backend_cannot_start_on_host(monkeypatch):
    monkeypatch.delenv('WS2_REMOTE_TASK',raising=False)
    with pytest.raises(RuntimeError,match='disposable'):runtime_process.verify()


def test_sequential_tunnel_port_reuse_and_live_listener_rejection():
    import socket
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        listener.bind(('127.0.0.1',0));listener.listen()
        port=listener.getsockname()[1]
        with pytest.raises(OSError):backend.verify_port_available(port)
        with socket.create_connection(('127.0.0.1',port)) as peer:
            connection,_=listener.accept()
            connection.shutdown(socket.SHUT_RDWR);connection.close()
            assert peer.recv(1)==b''
    # Server-side TIME_WAIT remains after an otherwise successful flight.
    backend.verify_port_available(port)


def test_artifact_roundtrip_and_wrong_run(tmp_path):
    source=tmp_path/'source';source.mkdir();(source/'result.json').write_text('{"outcome":"collision"}')
    (source/'sub').mkdir();(source/'sub/frame.jpg').write_bytes(b'frame')
    receipt=transport.pack_artifacts(source,tmp_path/'packed','a'*32,'b'*64)
    archive=tmp_path/'packed/artifacts.tar.gz'
    with pytest.raises(ValueError,match='different'):transport.unpack_artifacts(archive,receipt,tmp_path/'out','c'*32,'b'*64)
    transport.unpack_artifacts(archive,receipt,tmp_path/'out','a'*32,'b'*64)
    assert (tmp_path/'out/sub/frame.jpg').read_bytes()==b'frame'
    with pytest.raises(ValueError,match='overwrite'):transport.unpack_artifacts(archive,receipt,tmp_path/'out','a'*32,'b'*64)


@pytest.mark.parametrize('name,kind',[('../escape','file'),('/tmp/escape','file'),('link','symlink')])
def test_unsafe_archive_rejected_before_writes(tmp_path,name,kind):
    archive=tmp_path/'bad.tar.gz'
    with tarfile.open(archive,'w:gz') as tar:
        member=tarfile.TarInfo(name)
        if kind=='symlink':member.type=tarfile.SYMTYPE;member.linkname='/etc/passwd'
        else:member.size=1
        tar.addfile(member,io.BytesIO(b'x'))
    receipt={'run_id':'a','configuration_hash':'b','archive_sha256':transport.file_hash(archive),'files':{name:'fake'}}
    with pytest.raises(ValueError):transport.unpack_artifacts(archive,receipt,tmp_path/'out','a','b')
    assert not (tmp_path/'out').exists()


def test_remote_frame_identity_and_operator_control(tmp_path):
    from osmo_task import handler_type
    (tmp_path/'inference').mkdir();(tmp_path/'inference/pulp_dronet.jpg').write_bytes(b'image')
    payload={'run_id':'a'*32,'token':'test-token','scenario':{'planner':'pulp_dronet'}}
    connected=threading.Event()
    server=ThreadingHTTPServer(('127.0.0.1',0),handler_type(tmp_path,payload,connected))
    thread=threading.Thread(target=server.serve_forever);thread.start()
    session={'url':f'http://127.0.0.1:{server.server_port}','token':'test-token','run_id':'a'*32}
    try:
        assert transport.request(session,'/inference/pulp_dronet.jpg')==b'image'
        transport.request(session,'/control',{'action':'pause'})
        assert connected.is_set() and transport.read_json(tmp_path/'remote_control.json')=={'action':'pause'}
        with pytest.raises(ValueError,match='different run'):transport.request(dict(session,run_id='other'),'/snapshot')
        with pytest.raises(OSError):transport.request(dict(session,token='wrong'),'/snapshot')
        with pytest.raises(OSError):transport.request(session,'/control',{'action':'shell'})
        with pytest.raises(OSError):transport.request(session,'/../../etc/passwd')
    finally:server.shutdown();server.server_close();thread.join()


def test_local_preflight_does_not_need_osmo(monkeypatch):
    import check_install
    monkeypatch.setenv('WS2_EXECUTION_BACKEND','osmo')
    monkeypatch.delenv('WS2_OSMO_CONFIG',raising=False)
    assert not check_install.check_install('kim')['ok']


def test_remote_dispatch_keeps_campaign_api(monkeypatch):
    import episode
    monkeypatch.setenv('WS2_EXECUTION_BACKEND','osmo')
    monkeypatch.setattr(backend,'run_remote_episode',lambda raw,out,**kw:{'outcome':'test','options':kw})
    got=episode.run_episode(scenario(),'/unused',record_video=True,record_bag=False)
    assert got['options']['record_video'] and not got['options']['record_bag']
    with pytest.raises(ValueError,match='local-only'):episode.run_episode(scenario(),'/unused',True)


def test_viewer_never_shows_previous_run(monkeypatch,tmp_path):
    import dashboard
    monkeypatch.setattr(dashboard,'RUNTIME',tmp_path)
    transport.atomic_json(tmp_path/'remote_live.json',{'active':True,'run_id':'new','workflow_id':'job'})
    transport.atomic_json(tmp_path/'remote_snapshot.json',{'run_id':'old','scene':{'old':True}})
    assert dashboard.flight_view().get('scene') is None
    monkeypatch.setenv('WS2_EXECUTION_BACKEND','osmo')
    transport.atomic_json(tmp_path/'remote_live.json',{'active':False})
    (tmp_path/'live_view.jpg').write_bytes(b'old local frame')
    with pytest.raises(OSError,match='No active'):dashboard.flight_frame('/scene.jpg')
    assert dashboard.flight_view().get('scene') is None


def test_remote_result_download_keeps_originals_and_maps_paths(settings,tmp_path,monkeypatch):
    monkeypatch.setattr(backend,'RUNTIME',tmp_path)
    monkeypatch.setattr(backend,'load_settings',lambda:settings)
    class FakeClient:
        def submit(self,path):
            self.payload=json.loads(yaml.safe_load(path.read_text())['workflow']['tasks'][0]['files'][0]['contents'])
            return 'ws2-test-1'
        def status(self,name):return 'COMPLETED',{'status':'COMPLETED'}
        def download(self,uri,path):
            source=tmp_path/'remote';source.mkdir()
            remote='/opt/ws2/AirStack/robot/ros_ws/ws2_runtime/episodes/'+self.payload['run_id']
            transport.atomic_json(source/'result.json',{'outcome':'route_completed','configuration_hash':self.payload['configuration_hash'],'result_dir':remote})
            receipt=transport.pack_artifacts(source,path,self.payload['run_id'],self.payload['configuration_hash'])
            receipt['remote_episode_dir']=remote;transport.atomic_json(path/'receipt.json',receipt)
        def cancel(self,name):pytest.fail('Completed workflow must not be cancelled')
    # No port listener is started; use a free ephemeral port for availability precheck.
    import socket
    with socket.socket() as s:s.bind(('127.0.0.1',0));settings['port']=s.getsockname()[1]
    out=tmp_path/'campaign/flight'
    result=backend.run_remote_episode(scenario(),out,client=FakeClient())
    assert result['outcome']=='route_completed' and result['result_dir']==str(out)
    assert (out/'osmo/download/artifacts.tar.gz').exists()
    assert transport.read_json(tmp_path/'remote_live.json')['active'] is False


def test_uncertain_submission_blocks_duplicate(settings,tmp_path,monkeypatch):
    monkeypatch.setattr(backend,'RUNTIME',tmp_path);monkeypatch.setattr(backend,'load_settings',lambda:settings)
    class Failure:
        def submit(self,path):raise TimeoutError('lost response')
    import socket
    with socket.socket() as s:s.bind(('127.0.0.1',0));settings['port']=s.getsockname()[1]
    result=backend.run_remote_episode(scenario(),tmp_path/'flight',client=Failure())
    assert result['outcome']=='infrastructure_error'
    assert (tmp_path/'osmo_pending_cleanup.json').exists()
    with pytest.raises(RuntimeError,match='cleanup'):backend.run_remote_episode(scenario(),tmp_path/'another',client=Failure())


@pytest.mark.parametrize('cancel_fails',[False,True])
def test_stop_cancels_disconnected_job_and_blocks_if_unconfirmed(settings,tmp_path,monkeypatch,cancel_fails):
    monkeypatch.setattr(backend,'RUNTIME',tmp_path)
    monkeypatch.setattr(backend,'load_settings',lambda:settings)
    control=tmp_path/'control.json'
    class RunningClient:
        cancelled=False
        def submit(self,path):
            transport.atomic_json(control,{'action':'stop'})
            return 'ws2-running-1'
        def status(self,name):
            status='FAILED_CANCELED' if self.cancelled else 'RUNNING'
            return status,{'status':status}
        def cancel(self,name):
            assert name=='ws2-running-1'
            if cancel_fails:raise OSError('connection unavailable')
            self.cancelled=True
        def tunnel(self,*args):pytest.fail('Disconnected stop should cancel before opening a tunnel')
    def disconnected(*args,**kwargs):raise OSError('offline')
    monkeypatch.setattr(backend,'request',disconnected)
    import socket
    with socket.socket() as s:s.bind(('127.0.0.1',0));settings['port']=s.getsockname()[1]
    client=RunningClient()
    result=backend.run_remote_episode(scenario(),tmp_path/'flight',control_path=control,client=client)
    assert result['outcome']=='user_stopped'
    assert client.cancelled is not cancel_fails
    assert (tmp_path/'osmo_pending_cleanup.json').exists() is cancel_fails


def test_scenario_is_data_not_workflow_template(settings):
    c=scenario();c['name']='{{ secrets }}'
    with pytest.raises(ValueError,match='template'):backend.prepare(c,settings,'a'*32)


def test_acknowledged_stop_waits_for_upload_instead_of_cancelling(settings,tmp_path,monkeypatch):
    monkeypatch.setattr(backend,'RUNTIME',tmp_path)
    monkeypatch.setattr(backend,'load_settings',lambda:settings)
    settings['poll_seconds']=1
    control=tmp_path/'control.json';calls=[]
    class Tunnel:
        def poll(self):return None
        def terminate(self):pass
        def wait(self,**kwargs):return 0
    closed=[]
    monkeypatch.setattr(runtime_process,'stop_process',lambda process:closed.append(process))
    class FinishingClient:
        polls=0
        def submit(self,path):
            self.payload=json.loads(yaml.safe_load(path.read_text())['workflow']['tasks'][0]['files'][0]['contents'])
            transport.atomic_json(control,{'action':'stop'})
            return 'ws2-stopping-1'
        def status(self,name):
            self.polls+=1
            status='COMPLETED' if self.polls>=3 else 'RUNNING'
            return status,{'status':status}
        def tunnel(self,*args):return Tunnel()
        def cancel(self,*args):pytest.fail('Acknowledged stop must allow artifact upload')
        def download(self,uri,path):
            source=tmp_path/'remote';source.mkdir()
            transport.atomic_json(source/'result.json',{'outcome':'user_stopped',
                'configuration_hash':self.payload['configuration_hash'],'result_dir':str(source)})
            transport.pack_artifacts(source,path,self.payload['run_id'],self.payload['configuration_hash'])
    def stopping_server(session,path,payload=None):
        if path=='/control':
            calls.append(payload)
            if len(calls)>1:raise OSError('Server has exited during upload')
            return b'{}'
        raise OSError('Server has exited during upload')
    monkeypatch.setattr(backend,'request',stopping_server)
    import socket
    with socket.socket() as s:s.bind(('127.0.0.1',0));settings['port']=s.getsockname()[1]
    out=tmp_path/'flight'
    result=backend.run_remote_episode(scenario(),out,control_path=control,client=FinishingClient())
    assert result['outcome']=='user_stopped' and calls==[{'action':'stop'}]
    assert len(closed)==1 and isinstance(closed[0],Tunnel)
    assert (out/'osmo/download/artifacts.tar.gz').is_file()
