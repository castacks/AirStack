import json
from pathlib import Path

import pytest
import torch
from torch import nn

from Scripts.UnitTest.test_covariance_retraining_runner import corpus
from Train.MatchingNet import retraining_runner as base
from Train.MatchingNet import final_layer_retraining as policy
from Train.MatchingNet.retraining_manifest import sha256


class FixtureModel(nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder=nn.Sequential(nn.Conv2d(3,2,1),nn.BatchNorm2d(2),nn.Dropout(.5))
        self.memory_decoder=nn.Module();update=nn.Module();self.memory_decoder.cov_update=update
        update.gru=nn.Sequential(nn.Conv2d(3,3,1),nn.BatchNorm2d(3),nn.Dropout(.5))
        update.cov_head=nn.Module();update.cov_head.feature=nn.Conv2d(3,3,1)
        update.cov_head.conv4=nn.Conv2d(3,2,1);update.mask=nn.Conv2d(3,2,1)
    def forward(self,a,b):
        update=self.memory_decoder.cov_update;mean=self.encoder(a)
        raw=update.cov_head.conv4(update.cov_head.feature(update.gru(b)))+0*update.mask(b)
        return [mean],[raw]


def factory(root,manifest,device):
    model=FixtureModel();model.load_state_dict(torch.load(Path(root)/manifest['initialization']['file']['path'],weights_only=True))
    return model.to(device),{'migration':'synthetic fixture only'}


@pytest.fixture
def final_corpus(corpus):
    root,path,p=corpus;initial=root/p['initialization']['file']['path'];torch.save(FixtureModel().state_dict(),initial)
    p['initialization']['file']['sha256']=sha256(initial);path.write_text(json.dumps(p));return root,path,p


def test_trainable_scope_and_frozen_module_modes():
    model=FixtureModel();names=policy.configure(model)
    assert names==[policy.PREFIX+'weight',policy.PREFIX+'bias']
    assert not model.training and model.memory_decoder.training
    assert not model.memory_decoder.cov_update.training
    assert not model.memory_decoder.cov_update.gru.training
    assert not model.memory_decoder.cov_update.cov_head.feature.training
    assert model.memory_decoder.cov_update.cov_head.conv4.training
    assert all(p.requires_grad==(name in names) for name,p in model.named_parameters())


def test_bindings_restore_on_failure_and_source_identity_binds_policy():
    old=base.PREFIX,base.configure,base.source_identity
    with pytest.raises(RuntimeError,match='deliberate'):
        with policy.final_layer_policy():
            assert base.PREFIX==policy.PREFIX and base.configure is policy.configure
            assert 'Train/MatchingNet/final_layer_retraining.py' in base.source_identity(factory)
            raise RuntimeError('deliberate')
    assert (base.PREFIX,base.configure,base.source_identity)==old


def test_nested_policy_rejected_without_losing_outer_binding():
    old=base.PREFIX
    with policy.final_layer_policy():
        with pytest.raises(RuntimeError,match='nested'):
            with policy.final_layer_policy():pass
        assert base.PREFIX==policy.PREFIX
    assert base.PREFIX==old


def test_exact_resume_and_all_other_state_frozen(final_corpus):
    root,path,p=final_corpus
    for output,resume,stop in [('full',None,None),('part',None,2),('resumed',root/'part/step-00000002.pt',None)]:
        report=policy.train(root,path,root/output,device='cpu',resume=resume,stop_after=stop,model_factory=factory)
        assert report['trainable_names']==[policy.PREFIX+'weight',policy.PREFIX+'bias']
        assert json.loads((root/output/'policy-contract.json').read_text())['policy']==policy.POLICY
    a=torch.load(root/'full/step-00000004.pt',weights_only=True);b=torch.load(root/'resumed/step-00000004.pt',weights_only=True)
    initial=torch.load(root/p['initialization']['file']['path'],weights_only=True)
    for key,value in a['model'].items():
        assert torch.equal(value,b['model'][key])
        if not key.startswith(policy.PREFIX):assert torch.equal(value,initial[key])
    assert any(not torch.equal(a['model'][key],initial[key]) for key in a['trainable_names'])
    for key,item in a['optimizer']['state'].items():
        for field,value in item.items():assert torch.equal(value,b['optimizer']['state'][key][field])
    assert torch.equal(a['rng']['torch'],b['rng']['torch'])
    assert (root/'full/steps.jsonl').read_text()==(root/'part/steps.jsonl').read_text()+(root/'resumed/steps.jsonl').read_text()


def test_full_block_checkpoint_rejected_and_no_output_created(final_corpus):
    root,path,_=final_corpus;base.train(root,path,root/'full-block',device='cpu',stop_after=2,model_factory=factory)
    old=base.PREFIX,base.configure,base.source_identity
    with pytest.raises(ValueError,match='resume contract'):
        policy.train(root,path,root/'wrong-resume',device='cpu',resume=root/'full-block/step-00000002.pt',model_factory=factory)
    assert not (root/'wrong-resume').exists() and (base.PREFIX,base.configure,base.source_identity)==old


def test_existing_output_never_gets_sidecar(final_corpus):
    root,path,_=final_corpus;output=root/'existing';output.mkdir();(output/'marker').write_text('keep')
    with pytest.raises(FileExistsError):policy.train(root,path,output,device='cpu',model_factory=factory)
    assert sorted(p.name for p in output.iterdir())==['marker']


@pytest.mark.parametrize('bad',['channels','bias','layer'])
def test_wrong_final_layer_structure_rejected(bad):
    model=FixtureModel()
    model.memory_decoder.cov_update.cov_head.conv4=(nn.Identity() if bad=='layer' else nn.Conv2d(3,3 if bad=='channels' else 2,1,bias=bad!='bias'))
    with pytest.raises(ValueError):policy.configure(model)
