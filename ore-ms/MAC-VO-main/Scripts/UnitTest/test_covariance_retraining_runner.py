import copy
import json
from pathlib import Path

import numpy as np
import pytest
import torch
from torch import nn

from Train.MatchingNet.retraining_manifest import inspect_manifest, sha256
from Train.MatchingNet.retraining_runner import train, configure, atomic_save
from Train.MatchingNet.covariance_contract import CONTRACT_ID


class FixtureModel(nn.Module):
    """Synthetic software fixture; no dataset/model-quality interpretation."""
    def __init__(self):
        super().__init__()
        self.encoder = nn.Sequential(nn.Conv2d(3, 2, 1), nn.BatchNorm2d(2), nn.Dropout(.5))
        self.memory_decoder = nn.Module()
        self.memory_decoder.cov_update = nn.Sequential(nn.Dropout(.25), nn.Conv2d(3, 2, 1))
    def forward(self, first, second):
        mean = self.encoder(first)
        raw = self.memory_decoder.cov_update(second)
        return ([mean], [raw]) if self.memory_decoder.training else ((mean, mean), (raw, raw))


def fixture_factory(root, manifest, device):
    model = FixtureModel()
    model.load_state_dict(torch.load(Path(root)/manifest['initialization']['file']['path'], weights_only=True))
    return model.to(device), {'migration': 'synthetic_fixture_only'}


@pytest.fixture
def corpus(tmp_path):
    def pin(path): return {'path':str(path.relative_to(tmp_path)), 'sha256':sha256(path)}
    def document(name, payload):
        path = tmp_path/name;path.write_text(json.dumps(payload));return pin(path)
    initial = tmp_path/'initial.pt';torch.save(FixtureModel().state_dict(), initial)
    ledger = document('ledger.json', {'records':[
        {'id':name, 'aliases':[name+'-alias'], 'unit_kind':'temporal_recording',
         'exposure_status':'reported_reserved' if name=='reserve' else 'development_exposed'}
        for name in ('record-a','record-b','reserve')]})
    evidence = document('label-evidence.json', {'kind':'synthetic software test;not real labels'})
    rng = np.random.default_rng(10)
    samples = []
    for i, record in enumerate(('record-a','record-a','record-b')):
        path = tmp_path/f'pair{i}.npz'
        np.savez(path, image1=rng.random((3,64,64),dtype=np.float32),
                 image2=rng.random((3,64,64),dtype=np.float32),
                 flow=np.ones((2,64,64),dtype=np.float32),valid_mask=np.ones((1,64,64),dtype=bool))
        samples.append({'recording_id':record,'partition':'train' if i<2 else 'validation','file':pin(path)})
    payload = {'schema_version':1,'task':'dense_temporal_flow_covariance','contract_id':CONTRACT_ID,
        'ledger':ledger,'architecture':document('architecture.json',{'pretrain':False}),
        'input_recipe':document('recipe.json',{'kind':'synthetic fixture;no transformations'}),
        'initialization':{'kind':'checkpoint_finetune','file':pin(initial),'training_membership':'unknown'},
        'modality':'rgb','recordings':[{'id':name,'label_kind':'provided_dense_temporal_flow','label_evidence':evidence}
                                     for name in ('record-a','record-b')],
        'samples':samples,'optimization':{'seed':0,'steps':4,'checkpoint_interval':2,'lr':.001,
            'betas':[.9,.999],'eps':1e-8,'weight_decay':0.,'clip_norm':1.,'gamma':.85,'max_flow':100.}}
    manifest = tmp_path/'manifest.json';manifest.write_text(json.dumps(payload))
    return tmp_path,manifest,payload


def execute(corpus, output, **kwargs):
    root, path, _ = corpus
    return train(root,path,root/output,device='cpu',model_factory=fixture_factory,**kwargs)


def test_exact_resume_including_optimizer_rng_and_validation(corpus):
    root,_,payload=corpus
    complete=execute(corpus,'complete')
    partial=execute(corpus,'partial',stop_after=2)
    resumed=execute(corpus,'resumed',resume=root/'partial/step-00000002.pt')
    assert complete['status']==resumed['status']=='COMPLETED'
    assert partial['status']=='STOPPED_AT_DECLARED_STEP' and resumed['start_step']==2
    a=torch.load(root/'complete/step-00000004.pt',weights_only=True)
    b=torch.load(root/'resumed/step-00000004.pt',weights_only=True)
    for key in a['model']: assert torch.equal(a['model'][key],b['model'][key]),key
    for key,state in a['optimizer']['state'].items():
        for field,value in state.items():assert torch.equal(value,b['optimizer']['state'][key][field])
    assert torch.equal(a['rng']['torch'],b['rng']['torch'])
    assert a['rng']['python']==b['rng']['python'] and a['rng']['numpy']==b['rng']['numpy']
    assert complete['validation_nll']==resumed['validation_nll']
    initial=torch.load(root/payload['initialization']['file']['path'],weights_only=True)
    frozen=[key for key in initial if not key.startswith('memory_decoder.cov_update.')]
    assert all(torch.equal(initial[key],a['model'][key]) for key in frozen)
    assert any(not torch.equal(initial[key],a['model'][key]) for key in initial if key not in frozen)
    original=(root/'complete/steps.jsonl').read_text().splitlines()
    stitched=(root/'partial/steps.jsonl').read_text().splitlines()+(root/'resumed/steps.jsonl').read_text().splitlines()
    assert original==stitched


@pytest.mark.skipif(not torch.cuda.is_available(), reason='CUDA resume gate')
def test_cuda_resume_preserves_model_optimizer_and_dropout_rng(corpus):
    root, path, _ = corpus
    for name, resume, stop in [('gpu-full',None,None), ('gpu-part',None,2),
                               ('gpu-resume',root/'gpu-part/step-00000002.pt',None)]:
        train(root,path,root/name,device='cuda',resume=resume,stop_after=stop,model_factory=fixture_factory)
    a=torch.load(root/'gpu-full/step-00000004.pt',weights_only=True,map_location='cpu')
    b=torch.load(root/'gpu-resume/step-00000004.pt',weights_only=True,map_location='cpu')
    for key in a['model']:assert torch.equal(a['model'][key],b['model'][key]),key
    for key,state in a['optimizer']['state'].items():
        for field,value in state.items():assert torch.equal(value,b['optimizer']['state'][key][field])
    assert all(torch.equal(x,y) for x,y in zip(a['rng']['cuda'],b['rng']['cuda']))
    assert (root/'gpu-full/steps.jsonl').read_text()==(root/'gpu-part/steps.jsonl').read_text()+(root/'gpu-resume/steps.jsonl').read_text()


def test_decoder_modes_and_trainables_are_explicit():
    model=FixtureModel();names=configure(model)
    assert names and not model.training and model.memory_decoder.training
    assert not model.encoder.training and not model.encoder[1].training and not model.encoder[2].training
    assert model.memory_decoder.cov_update.training
    assert all(p.requires_grad==name.startswith('memory_decoder.cov_update.') for name,p in model.named_parameters())


@pytest.mark.parametrize('mutation',['overlap','alias','reserve','duplicate','hash','escape','pretrain',
    'membership','label','missing_validation','gamma','betas','empty_targets','image_dtype','mask_dtype','shared_image'])
def test_manifest_rejections(corpus,mutation):
    root,path,p=corpus;p=copy.deepcopy(p)
    if mutation=='overlap':p['samples'][2]['recording_id']='record-a'
    elif mutation=='alias':p['samples'][0]['recording_id']='record-a-alias'
    elif mutation=='reserve':p['recordings'][0]['id']='reserve'
    elif mutation=='duplicate':p['samples'][1]['file']=p['samples'][0]['file']
    elif mutation=='hash':p['samples'][0]['file']['sha256']='0'*64
    elif mutation=='escape':p['samples'][0]['file']['path']='../outside.npz'
    elif mutation=='membership':p['initialization']['training_membership']='verified'
    elif mutation=='label':p['recordings'][0]['label_kind']='pose_proxy'
    elif mutation=='missing_validation':p['samples'][2]['partition']='train'
    elif mutation=='gamma':p['optimization']['gamma']=float('nan')
    elif mutation=='betas':p['optimization']['betas']=[True,.999]
    elif mutation=='pretrain':
        file=root/p['architecture']['path'];file.write_text('{"pretrain":true}');p['architecture']['sha256']=sha256(file)
    else:
        sample=p['samples'][2] if mutation=='shared_image' else p['samples'][0]
        file=root/sample['file']['path']
        with np.load(file) as data:arrays={k:data[k].copy() for k in data.files}
        if mutation=='empty_targets':arrays['valid_mask'][:]=False
        elif mutation=='image_dtype':arrays['image1']=arrays['image1'].astype(np.float64)
        elif mutation=='mask_dtype':arrays['valid_mask']=arrays['valid_mask'].astype(np.float32)
        else:
            with np.load(root/p['samples'][0]['file']['path']) as other:arrays['image1']=other['image1']
        np.savez(file,**arrays);sample['file']['sha256']=sha256(file)
    path.write_text(json.dumps(p))
    with pytest.raises((ValueError,FileNotFoundError)):execute((root,path,p),'rejected')
    assert not (root/'rejected').exists()


@pytest.mark.parametrize('mutation',['identity','trainables','step','frozen','optimizer_lr','optimizer_nan'])
def test_resume_tampering_rejected_before_output(corpus,mutation):
    root,_,_=corpus;execute(corpus,'partial',stop_after=2)
    state=torch.load(root/'partial/step-00000002.pt',weights_only=True)
    if mutation=='identity':state['identity']['manifest_sha256']='0'*64
    elif mutation=='trainables':state['trainable_names']=[]
    elif mutation=='step':state['step']=True
    elif mutation=='frozen':state['model']['encoder.0.weight']+=1
    elif mutation=='optimizer_lr':state['optimizer']['param_groups'][0]['lr']=3
    else:next(iter(state['optimizer']['state'].values()))['exp_avg'].fill_(float('nan'))
    altered=root/'altered.pt';torch.save(state,altered)
    with pytest.raises(ValueError):execute(corpus,'bad-resume',resume=altered)
    assert not (root/'bad-resume').exists()


def test_existing_output_and_checkpoint_never_overwritten(corpus):
    root,_,_=corpus;(root/'existing').mkdir();(root/'existing/marker').write_text('keep')
    with pytest.raises(FileExistsError):execute(corpus,'existing')
    assert (root/'existing/marker').read_text()=='keep'
    destination=root/'exclusive.pt';atomic_save({'value':1},destination)
    with pytest.raises(FileExistsError):atomic_save({'value':2},destination)
    assert torch.load(destination,weights_only=True)['value']==1
    pending=root/'untouched.pending';pending.write_text('keep unrelated pending file')
    with pytest.raises(FileExistsError):atomic_save({'value':3},root/'untouched.pt')
    assert pending.read_text()=='keep unrelated pending file'


def test_nonfinite_gradient_prevents_optimizer_update(corpus,monkeypatch):
    import Train.MatchingNet.retraining_runner as runner
    root,_,_=corpus;original=runner.objective
    def poisoned(model,pair,options):
        value=original(model,pair,options)
        next(p for p in model.parameters() if p.requires_grad).register_hook(lambda g:g*float('nan'))
        return value
    monkeypatch.setattr(runner,'objective',poisoned)
    with pytest.raises(ValueError,match='gradient'):execute(corpus,'bad-gradient')
    report=json.loads((root/'bad-gradient/report.json').read_text())
    assert report['status']=='FAILED' and report['step']==0
    assert not list((root/'bad-gradient').glob('step-*.pt'))
