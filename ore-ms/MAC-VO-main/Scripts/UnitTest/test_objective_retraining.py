import itertools
import json
from pathlib import Path

import pytest
import torch

from Scripts.UnitTest.test_covariance_retraining_runner import corpus, fixture_factory
from Train.MatchingNet import retraining_runner as base
from Train.MatchingNet import objective_retraining as policy
from Train.MatchingNet import final_layer_retraining as final_policy
from Train.MatchingNet.trace_risk_contract import final_trace_risk_score
from Train.MatchingNet.covariance_contract import sequence_covariance_nll


MODES = tuple(policy.OBJECTIVES)


def equal(a, b):
    if isinstance(a, torch.Tensor):assert torch.equal(a,b)
    elif isinstance(a, dict):
        assert a.keys()==b.keys()
        for k in a:equal(a[k],b[k])
    elif isinstance(a,(tuple,list)):
        assert type(a)==type(b) and len(a)==len(b)
        for x,y in zip(a,b):equal(x,y)
    else:assert a==b


def run(corpus, name, mode, **kwargs):
    root,path,_=corpus
    if mode=='recurrent':return base.train(root,path,root/name,model_factory=fixture_factory,**kwargs)
    return policy.train(root,path,root/name,objective_mode=mode,model_factory=fixture_factory,**kwargs)


def replay(corpus, mode, device):
    root,_,manifest=corpus
    reports=[]
    for name,parent,stop in [('full',None,None),('part',None,2),('resumed',root/'part/step-00000002.pt',None)]:
        reports.append(run(corpus,name,mode,device=device,resume=parent,stop_after=stop))
        contract=json.loads((root/name/'objective-contract.json').read_text())
        assert contract['objective_id']==policy.OBJECTIVES[mode]
        assert contract['legacy_validation_nll_field_semantics'].startswith('selected objective')
    a=torch.load(root/'full/step-00000004.pt',map_location='cpu',weights_only=True)
    b=torch.load(root/'resumed/step-00000004.pt',map_location='cpu',weights_only=True);equal(a,b)
    initial=torch.load(root/manifest['initialization']['file']['path'],weights_only=True)
    assert all(torch.equal(initial[k],a['model'][k]) for k in initial if not k.startswith(base.PREFIX))
    assert any(not torch.equal(initial[k],a['model'][k]) for k in initial if k.startswith(base.PREFIX))
    assert (root/'full/steps.jsonl').read_bytes()==(root/'part/steps.jsonl').read_bytes()+(root/'resumed/steps.jsonl').read_bytes()
    assert reports[0]['validation_nll']==reports[2]['validation_nll']
    assert reports[2]['start_step']==2 and reports[0]['frozen_state_unchanged']
    assert 'objective-policy/'+policy.OBJECTIVES[mode] in a['identity']['source_sha256']
    assert bool(a['rng']['cuda'])==(device=='cuda')
    (root/'replay-evidence.json').write_text(json.dumps({'device':device,'objective_mode':mode,'checkpoint_exact':True,'journal_exact':True,'frozen_exact':True,'validation_exact':True}))


@pytest.mark.parametrize('mode',MODES)
def test_cpu_exact_resume_all_checkpoint_fields_and_journal(corpus,mode):replay(corpus,mode,'cpu')


@pytest.mark.skipif(not torch.cuda.is_available(),reason='CUDA objective resume gate')
@pytest.mark.parametrize('mode',MODES)
def test_cuda_exact_resume_all_checkpoint_fields_and_journal(corpus,mode):replay(corpus,mode,'cuda')


@pytest.mark.parametrize('source,destination',list(itertools.permutations(('recurrent',*MODES),2)))
def test_all_cross_objective_resumes_rejected_before_output(corpus,source,destination):
    root,_,_=corpus;run(corpus,'part',source,device='cpu',stop_after=2)
    before=base.objective,base.source_identity,base.configure,base.PREFIX
    with pytest.raises(ValueError,match='resume contract'):
        run(corpus,'rejected',destination,device='cpu',resume=root/'part/step-00000002.pt')
    assert not (root/'rejected').exists()
    assert (base.objective,base.source_identity,base.configure,base.PREFIX)==before


@pytest.mark.parametrize('mode',MODES)
def test_dispatch_final_stage_and_exact_objective_values(mode):
    class Model:
        def __call__(self,*args):return [mean,mean],[earlier,raw]
    mean=torch.ones(1,2,2,2,requires_grad=True)
    raw=torch.zeros_like(mean,requires_grad=True);earlier=torch.full_like(raw,.5,requires_grad=True)
    pair={'image1':None,'image2':None,'flow':torch.zeros_like(mean,requires_grad=True),'valid_mask':torch.ones(1,1,2,2,dtype=torch.bool)}
    options={'gamma':.85,'max_flow':400.}
    with policy.objective_policy(mode):result=base.objective(Model(),pair,options)
    if mode=='final_diagonal_nll':reference=sequence_covariance_nll([mean],[raw],pair['flow'],pair['valid_mask'],**options)
    else:reference=final_trace_risk_score([mean],[raw],pair['flow'],pair['valid_mask'],max_flow=400.)
    assert result.loss.item()==reference.loss.item() and result.valid_components==8
    result.loss.backward();assert earlier.grad is None and mean.grad is None and pair['flow'].grad is None
    assert raw.grad is not None and torch.isfinite(raw.grad).all()


def test_bindings_restore_on_exception_and_reject_nested_conflicts():
    original=base.objective,base.source_identity,base.configure,base.PREFIX
    with pytest.raises(RuntimeError,match='deliberate'):
        with policy.objective_policy(MODES[0]):
            with pytest.raises(RuntimeError,match='conflicting'):
                with policy.objective_policy(MODES[1]):pass
            raise RuntimeError('deliberate')
    assert (base.objective,base.source_identity,base.configure,base.PREFIX)==original
    with final_policy.final_layer_policy():
        with pytest.raises(RuntimeError,match='conflicting'):
            with policy.objective_policy(MODES[0]):pass
    assert (base.objective,base.source_identity,base.configure,base.PREFIX)==original


@pytest.mark.parametrize('mode',[None,False,[],{},'recurrent','typo'])
def test_explicit_modes_only_and_no_output(corpus,mode):
    root,path,_=corpus
    with pytest.raises(ValueError):policy.train(root,path,root/'rejected',objective_mode=mode,device='cpu')
    assert not (root/'rejected').exists()


@pytest.mark.parametrize('mode',MODES)
def test_existing_output_is_untouched(corpus,mode):
    root,_,_=corpus;(root/'existing').mkdir();(root/'existing/marker').write_text('keep')
    with pytest.raises(FileExistsError):run(corpus,'existing',mode,device='cpu')
    assert list(f.name for f in (root/'existing').iterdir())==['marker']


def test_failure_after_output_writes_sidecar_and_restores_bindings(corpus,monkeypatch):
    root,path,_=corpus;original=base.objective,base.source_identity,base.configure,base.PREFIX
    def broken(*args,**kwargs):
        (root/'failed').mkdir();raise ValueError('fixture engine failure')
    monkeypatch.setattr(base,'train',broken)
    with pytest.raises(ValueError,match='fixture engine'):
        policy.train(root,path,root/'failed',objective_mode=MODES[1],device='cpu')
    assert json.loads((root/'failed/objective-contract.json').read_text())['objective_mode']==MODES[1]
    assert (base.objective,base.source_identity,base.configure,base.PREFIX)==original
