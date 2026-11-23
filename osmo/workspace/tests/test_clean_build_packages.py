"""Targeted generated-artifact recovery must never clean sources or live work."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as S
import pytest
spec=importlib.util.spec_from_file_location('clean_runner',Path(__file__).parents[1]/'mission_runner.py')
runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)


def test_only_requested_generated_package_is_cleaned_before_up(tmp_path,monkeypatch):
    for d in ['build','install','src']:
        for package in ['waypoint_interface','raven_nav']:
            p=tmp_path/'robot/ros_ws'/d/package;p.mkdir(parents=True);(p/'keep').write_text('data')
    stack=S(root=tmp_path,mission={'stack':{'clean_build_packages':['waypoint_interface']},'up_timeout_s':1},services=[],num_robots=1)
    def launch(*a,**kw):
        assert not (tmp_path/'robot/ros_ws/build/waypoint_interface').exists()
        assert not (tmp_path/'robot/ros_ws/install/waypoint_interface').exists()
        assert (tmp_path/'robot/ros_ws/src/waypoint_interface/keep').exists()
        assert (tmp_path/'robot/ros_ws/build/raven_nav/keep').exists()
        return S(stdout='',stderr='',returncode=0)
    stack._airstack=launch
    monkeypatch.setattr(runner,'stack_containers',lambda:[])
    monkeypatch.setattr(runner,'robot_containers',lambda:['robot'])
    assert runner.Stack.up(stack)==['robot']


@pytest.mark.parametrize('entry,active,error',[('../src',[],ValueError),('waypoint_interface',['robot'],RuntimeError)])
def test_clean_rejects_unsafe_or_active_workspace(tmp_path,monkeypatch,entry,active,error):
    stack=S(root=tmp_path,mission={'stack':{'clean_build_packages':[entry]},'up_timeout_s':1},services=[],num_robots=1)
    monkeypatch.setattr(runner,'stack_containers',lambda:active)
    with pytest.raises(error):runner.Stack.up(stack)
