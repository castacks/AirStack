"""Exact native input/clip parity, no re-pairing and process-local restoration."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS
import numpy as np
import pytest
import torch
from DataLoader.Dataset.MS2 import MS2ManifestSequence, THERMAL_RECIPE
from Scripts.Setup.ms2_execution_adapter import prepare, MS2ExecutionSequence, adapted_runner
from Scripts.UnitTest.test_ms2_adapter import pilot


@pytest.fixture
def paired(pilot):
 root,source,records=pilot
 payload=json.loads(source.read_text());payload['sequence']='synthetic/exposed';source.write_text(json.dumps(payload))
 path=root/'paired.json';prepare(root,source,path)
 return root,source,path,records


def compare(a,b):
 assert a.idx==b.idx and a.time_ns==b.time_ns
 for k in ('imageL','imageR','K','baseline','T_BS'):
  x,y=getattr(a.stereo,k),getattr(b.stereo,k)
  assert x.dtype==y.dtype and torch.equal(x,y)
 for k in ('right_time_ns','source_stem','input_transform','recorded_time_semantics','height','width'):
  assert getattr(a.stereo,k)==getattr(b.stereo,k)
 assert a.gt_pose is None and b.gt_pose is None and a.stereo.gt_depth is None and b.stereo.gt_depth is None


@pytest.mark.parametrize('camera,native',[('rgb','rgb'),('thermal','thr')])
def test_all_native_inputs_calibration_metadata_and_clipping_exact(paired,camera,native):
 root,source,path,_=paired
 adapter=MS2ExecutionSequence(root,path,camera,acknowledge_provisional_geometry=True)
 original=MS2ManifestSequence(root,source,native,thermal_transform=THERMAL_RECIPE if native=='thr' else None)
 before=source.read_bytes()
 for i in range(3):compare(adapter[i],original[i])
 assert adapter.signed_baseline==adapter.baseline==original.baseline
 adapter.clip(start_idx=1,end_idx=3);original.clip(start_idx=1,end_idx=3)
 assert len(adapter)==len(original)==2
 compare(adapter[0],original[0]);compare(adapter[1],original[1])
 assert source.read_bytes()==before


@pytest.mark.parametrize('fault',['time','path','stem','recipe','source_hash','sequence','dataset','outside'])
def test_changed_source_contract_is_rejected(paired,fault):
 root,source,path,_=paired;data=json.loads(path.read_text())
 if fault=='time':data['pairs'][0]['thermal_left']['time_ns']+=1
 elif fault=='path':data['pairs'][0]['rgb_right']['path']='different.png'
 elif fault=='stem':data['pairs'][0]['source_stem']='different'
 elif fault=='recipe':data['thermal_recipe']='implicit-normalization'
 elif fault=='source_hash':source.write_text(source.read_text()+' ')
 elif fault=='sequence':data['source_sequence']='other'
 elif fault=='dataset':data['dataset']='td3'
 else:data['source_manifest_relative']='../escape.json'
 path.write_text(json.dumps(data))
 with pytest.raises(ValueError):MS2ExecutionSequence(root,path,'thermal',acknowledge_provisional_geometry=True)


def test_prepare_exclusive_and_acknowledgement_required(paired):
 root,source,path,_=paired
 with pytest.raises(FileExistsError):prepare(root,source,path)
 with pytest.raises(ValueError):MS2ExecutionSequence(root,path,'thermal')
 with pytest.raises(ValueError):MS2ExecutionSequence(root,path,'thr',acknowledge_provisional_geometry=True)


def test_scheduler_rows_preserve_all_four_exact_integer_times(paired):
 root,source,path,records=paired;rows=json.loads(path.read_text())['pairs']
 for i,row in enumerate(rows):
  for c,native in [('rgb','rgb'),('thermal','thr')]:
   for side in ('left','right'):
    assert type(row[c+'_'+side]['time_ns']) is int
    assert row[c+'_'+side]['time_ns']==records[i]['cameras'][native][side]['recorded_time_ns']>2**53
    assert row[c+'_'+side]['path']==records[i]['cameras'][native][side]['path']


def test_scoped_runner_restores_on_original_failure():
 from Scripts.Setup import run_td3_execution_v3 as original
 before=original.TD3ManifestSequence
 with pytest.raises(RuntimeError,match='failure'):
  with adapted_runner() as runner:
   assert runner.TD3ManifestSequence is MS2ExecutionSequence
   raise RuntimeError('failure')
 assert original.TD3ManifestSequence is before


@pytest.mark.parametrize('fail',[False,True])
def test_new_single_entrypoint_preserves_call_and_restores(paired,monkeypatch,fail):
 from Scripts.Setup import run_td3_execution_v3 as base
 from Scripts.Setup.run_ms2_execution import run
 root,_,path,_=paired;args=NS(dataset=root,manifest=path,output=root/'out');before=base.TD3ManifestSequence;calls=[]
 def fake(a,c):
  calls.append((a,c));assert base.TD3ManifestSequence is MS2ExecutionSequence
  if fail:raise RuntimeError('original')
  return 'sentinel'
 monkeypatch.setattr(base,'run',fake)
 if fail:
  with pytest.raises(RuntimeError,match='original'):run(args,'rgb')
 else:assert run(args,'rgb')=='sentinel'
 assert calls==[(args,'rgb')] and base.TD3ManifestSequence is before


@pytest.mark.parametrize('fail',[False,True])
def test_new_shared_entrypoint_restores_worker(paired,monkeypatch,fail):
 from Scripts.Setup import ms2_shared_arbiter_replay as ms2
 root,_,path,_=paired;args=NS(dataset=root,manifest=path,output=root/'shared');before=ms2.controller.worker
 def fake(a):
  assert a is args and ms2.controller.worker is ms2.worker
  if fail:raise RuntimeError('original')
  return 'sentinel'
 monkeypatch.setattr(ms2.controller,'run',fake)
 if fail:
  with pytest.raises(RuntimeError,match='original'):ms2.run(args)
 else:assert ms2.run(args)=='sentinel'
 assert ms2.controller.worker is before


def test_existing_output_is_not_modified(paired):
 from Scripts.Setup.run_ms2_execution import run
 from Scripts.Setup.ms2_shared_arbiter_replay import run as shared
 root,_,path,_=paired;(root/'out/rgb').mkdir(parents=True);(root/'shared').mkdir()
 with pytest.raises(FileExistsError):run(NS(dataset=root,manifest=path,output=root/'out'),'rgb')
 with pytest.raises(FileExistsError):shared(NS(dataset=root,manifest=path,output=root/'shared'))
 assert list((root/'out/rgb').iterdir())==[] and list((root/'shared').iterdir())==[]


def test_process_thread_settings_are_explicit_and_restored():
 import cv2
 previous=(cv2.getNumThreads(),torch.get_num_threads())
 with adapted_runner():assert (cv2.getNumThreads(),torch.get_num_threads())==(1,1)
 assert (cv2.getNumThreads(),torch.get_num_threads())==previous
