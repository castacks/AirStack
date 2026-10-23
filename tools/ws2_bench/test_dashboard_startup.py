"""Exercise HTTP startup failures without Docker, LLM calls or flights."""
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

import pytest
import check_install
import dashboard


@pytest.fixture
def server(tmp_path, monkeypatch):
    monkeypatch.setattr(dashboard, 'RUNTIME', tmp_path)
    monkeypatch.setattr(dashboard, 'job', None)
    monkeypatch.setattr(dashboard, 'job_output', None)
    monkeypatch.setattr(dashboard, 'MONITOR', False)
    monkeypatch.setattr(check_install, 'check_install', lambda p: {'ok': True, 'errors': []})
    http = dashboard.ThreadingHTTPServer(('127.0.0.1', 0), dashboard.Handler)
    thread = threading.Thread(target=http.serve_forever, daemon=True);thread.start()
    yield f'http://127.0.0.1:{http.server_port}', tmp_path
    if dashboard.job is not None:
        dashboard.job.wait(timeout=5)
    http.shutdown();thread.join();http.server_close()


def request(base, path, payload=None):
    data=None if payload is None else json.dumps(payload).encode()
    req=urllib.request.Request(base+path, data=data, headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=5) as response:
            return response.status, response.read().decode()
    except urllib.error.HTTPError as exc:return exc.code, exc.read().decode()


def test_early_import_failure_is_visible_not_previous_campaign(server, monkeypatch):
    base, root=server
    dashboard.atomic(root/'live_campaign.json', {'output':str(root/'campaigns/old'),'phase':'complete'})
    monkeypatch.setattr(dashboard, 'adaptive_command', lambda p,o: [sys.executable,'-c',
        "import sys; print('ModuleNotFoundError: missing installation dependency',file=sys.stderr);sys.exit(1)"])
    code,body=request(base,'/start/adaptive',{'planner':'kim'})
    assert code==200
    output=json.loads(body)['output']
    for _ in range(100):
        _,body=request(base,'/state');state=json.loads(body)
        if state['launch']['status']=='failed':break
        time.sleep(.02)
    assert state['launch']['exit_code']==1
    assert state['campaign']['output']==output
    assert state['campaign']['phase']=='error'
    assert not state['running']
    assert 'ModuleNotFoundError' in request(base,'/launch-log')[1]
    assert dashboard.campaign_root()==Path(output)


def test_installation_failure_returns_actionable_error_without_launch(server, monkeypatch):
    base,root=server
    monkeypatch.setattr(check_install,'check_install',lambda p:{'ok':False,'errors':['Missing worker image: install selected model']})
    code,body=request(base,'/start/adaptive',{'planner':'kim','policy':'random','action_space':'corridor'})
    assert code==400 and 'Missing worker image' in json.loads(body)['error']
    assert dashboard.job is None
    assert not (root/'dashboard_campaign.log').exists()


def test_spawn_permission_error_is_json_not_dropped_connection(server, monkeypatch):
    monkeypatch.setattr(dashboard,'adaptive_command',lambda p,o:['/definitely/missing/ws2-python'])
    code,body=request(server[0],'/start/adaptive',{'planner':'kim'})
    assert code==500 and 'Cannot start campaign' in json.loads(body)['error']


def test_exit_status_survives_viewer_restart_and_new_cli_wins(tmp_path, monkeypatch):
    monkeypatch.setattr(dashboard,'RUNTIME',tmp_path)
    monkeypatch.setattr(dashboard,'job',None)
    record={'output':str(tmp_path/'campaigns/old'),'started_at':1,'status':'failed','exit_code':1}
    dashboard.atomic(tmp_path/'dashboard_launch.json',record)
    assert dashboard.launch_status()['exit_code']==1
    dashboard.atomic(tmp_path/'live_campaign.json',{'output':str(tmp_path/'campaigns/new')})
    assert dashboard.launch_status() is None
    assert dashboard.campaign_root()==tmp_path/'campaigns/new'


def test_log_is_bounded_and_masks_environment_credentials(tmp_path, monkeypatch):
    monkeypatch.setattr(dashboard,'RUNTIME',tmp_path)
    monkeypatch.setenv('WS2_AGENT_API_KEY','synthetic-private-value')
    (tmp_path/'dashboard_campaign.log').write_text('x'*20000+'\nsynthetic-private-value\nerror')
    result=dashboard.launch_log_tail()
    assert len(result)<=12000 and 'synthetic-private-value' not in result
    assert result.endswith('error')


def test_static_check_reports_missing_install_and_docker(tmp_path,monkeypatch):
    monkeypatch.setattr(check_install,'ROOT',tmp_path)
    monkeypatch.setattr(check_install,'RUNTIME',tmp_path/'runtime')
    from dataclasses import replace
    adapter=replace(check_install.get_adapter('kim'),repository=tmp_path/'missing-model')
    monkeypatch.setattr(check_install,'get_adapter',lambda p:adapter)
    def unavailable(*a):raise FileNotFoundError('docker not found')
    monkeypatch.setattr(check_install,'docker',unavailable)
    result=check_install.check_install('kim')
    assert not result['ok']
    assert any('Missing worker' in e for e in result['errors'])
    assert any('desktop_bringup' in e for e in result['errors'])
    assert any('Docker/container/image' in e for e in result['errors'])


def test_container_absolute_build_symlinks_are_accepted(tmp_path,monkeypatch):
    monkeypatch.setattr(check_install,'ROOT',tmp_path)
    monkeypatch.setattr(check_install,'RUNTIME',tmp_path)
    for package in ('desktop_bringup','local_bringup','mononav_bridge','trajectory_controller','pid_controller'):
        marker=tmp_path/'robot/ros_ws/install'/package/'share/ament_index/resource_index/packages'/package
        marker.parent.mkdir(parents=True)
        marker.symlink_to('/root/only-valid-inside-robot/'+package)
    monkeypatch.setattr(check_install,'docker',lambda *args:[])
    result=check_install.check_install('kim')
    assert not any('not built' in e for e in result['errors'])
