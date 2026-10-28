"""Source-contract probes and checkpoint container audit; no model/image inference."""
import argparse,ast,copy,hashlib,json,math,platform,shutil,subprocess
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from zoneinfo import ZoneInfo
import torch
SOURCES=['Train/MatchingNet/loss.py','Train/MatchingNet/train_flowformer.py','Train/MatchingNet/evaluate.py','Train/MatchingNet/utils.py','Module/Network/FlowFormerCov/covhead.py','Module/Network/FlowFormerCov/flownet.py','Module/Network/FlowFormerCov/checkpoint.py','Module/Network/FlowFormerCov/__init__.py','Module/Network/FlowFormer/core/decoder.py','DataLoader/Dataset/Train.py','DataLoader/Dataset/TartanAir.py','DataLoader/Dataset/TartanAir2.py','DataLoader/Transform.py','Config/Train/Demo.yaml','Config/Train/FlowFormerCovKITTIthings.yaml','Scripts/Setup/ms2_manifest_stereo.py']
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def run(root,output):
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True);repo=root/'MAC-VO-main';snap=output/'source-snapshots';snap.mkdir();registry={};asts={}
    for rel in SOURCES:
        path=repo/rel;assert path.exists(),path
        registry[rel]={'sha256':sha(path),'lines':len(path.read_text().splitlines())}
        target=snap/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,target)
        if path.suffix=='.py':
            tree=ast.parse(path.read_text());asts[rel]=tree
            registry[rel]['functions']={f'{getattr(n,"name","")}@{n.lineno}':{'first_line':n.lineno,'last_line':n.end_lineno,'ast_sha256':hashlib.sha256(ast.dump(n,include_attributes=False).encode()).hexdigest()} for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)}
    loss_tree=asts['Train/MatchingNet/loss.py'];functions={n.name:n for n in loss_tree.body if isinstance(n,ast.FunctionDef)}
    cov=functions['cov_loss'];uses={name:sum(isinstance(n,ast.Name) and isinstance(n.ctx,ast.Load) and n.id==name for n in ast.walk(cov)) for name in ['flow_mask','max_cov','cov_preds']}
    assert uses['flow_mask']==uses['max_cov']==0 and uses['cov_preds']>0
    assert any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='detach' for n in ast.walk(cov))
    assert not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='exp' for n in ast.walk(cov))
    scope={'torch':torch};selected=[]
    for name in ['flow_loss','cov_loss','final_cov_loss','sequence_loss','sequence_metric']:
        node=copy.deepcopy(functions[name]);node.decorator_list=[];selected.append(node)
    exec(compile(ast.Module(body=selected,type_ignores=[]),'extracted-original-loss','exec'),scope)
    torch.set_num_threads(1);torch.manual_seed(0);torch.set_default_dtype(torch.float64)
    gt=torch.zeros(1,2,1,2);pred=torch.full_like(gt,2.,requires_grad=True);raw=torch.full_like(gt,.5,requires_grad=True);eps=1e-7
    loss,error=scope['cov_loss'](.85,[pred],gt,[raw],torch.ones_like(gt,dtype=torch.bool));expected=4/(.5+eps)+math.log(.5+eps)
    assert math.isclose(loss.item(),expected,rel_tol=1e-12)
    masked,_=scope['cov_loss'](.85,[pred],gt,[raw],torch.zeros_like(gt,dtype=torch.bool),max_cov=.001)
    assert torch.equal(masked,loss)
    loss.backward();assert pred.grad is None
    expected_gradient=(1/(.5+eps)-4/(.5+eps)**2)/gt.numel();assert torch.allclose(raw.grad,torch.full_like(gt,expected_gradient))
    negative,_=scope['cov_loss'](.85,[pred],gt,[torch.full_like(gt,-.2)]);assert torch.isnan(negative)
    raw_only={'loss':loss.item(),'independent_formula':expected,'mask_all_false_same_loss':torch.equal(masked,loss),'max_cov_ignored':True,'direct_flow_gradient':None,'covariance_gradient_per_element':raw.grad.flatten().tolist(),'negative_raw_loss_nonfinite':True}
    seq=[]
    for use_mask in [True,False]:
        cfg=SimpleNamespace(training_mode='cov',gamma=.85,max_flow=.1,cov_mask=use_mask)
        value,_=scope['sequence_loss'](cfg,[pred],gt,torch.zeros_like(gt[:,:1],dtype=torch.bool),[raw]);assert torch.equal(value,loss);seq.append({'cov_mask':use_mask,'loss':value.item()})
    p1=torch.ones_like(gt);p2=torch.full_like(gt,2);v1=torch.full_like(gt,1.5);v2=torch.full_like(gt,2.5)
    multi,_=scope['cov_loss'](.85,[p1,p2],gt,[v1,v2]);formula=.85*(1/(1.5+eps)+math.log(1.5+eps))+(4/(2.5+eps)+math.log(2.5+eps));assert math.isclose(multi.item(),formula,rel_tol=1e-12)
    final,_=scope['final_cov_loss']([p1,p2],gt,[torch.full_like(gt,-2),v2]);assert math.isclose(final.item(),4/(2.5+eps)+math.log(2.5+eps),rel_tol=1e-12)
    # Execute the exact inference method with identity padding and a stub forward.
    tree=asts['Module/Network/FlowFormerCov/flownet.py'];klass=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='FlowFormerCov');inference=copy.deepcopy(next(n for n in klass.body if isinstance(n,ast.FunctionDef) and n.name=='inference'));inference.decorator_list=[]
    class IdentityPadder:
        def __init__(self,shape):pass
        def pad(self,*args):return args
        def unpad(self,x):return x
    raw_probe=torch.tensor([-.2,0,.5,1.]).view(1,2,1,2);flow_probe=torch.zeros_like(raw_probe)
    fake=SimpleNamespace(forward=lambda a,b:((flow_probe,None),(raw_probe,None)));inf_scope={'torch':torch,'InputPadder':IdentityPadder}
    exec(compile(ast.Module(body=[inference],type_ignores=[]),'extracted-original-inference','exec'),inf_scope)
    dummy=torch.zeros(1,3,1,2);_,variance=inf_scope['inference'](fake,dummy,dummy)
    assert torch.equal(variance,torch.exp(2*raw_probe)) and torch.all(variance>0)
    consistent=4/math.exp(1.)+1.;assert not math.isclose(consistent,loss.item())
    # Run exact trainer parameter-freezing match statement on a tiny structural stub.
    train=next(n for n in asts['Train/MatchingNet/train_flowformer.py'].body if isinstance(n,ast.FunctionDef) and n.name=='train')
    freeze=next(n for n in train.body if isinstance(n,ast.Match) and isinstance(n.subject,ast.Name) and n.subject.id=='train_mode');freezing={}
    for mode in ['flow','cov','finalcov']:
        model=torch.nn.Module();model.memory_decoder=torch.nn.Module();model.memory_decoder.cov_update=torch.nn.Linear(1,1);model.other=torch.nn.Linear(1,1)
        exec(compile(ast.Module(body=[copy.deepcopy(freeze)],type_ignores=[]),'extracted-trainer-freeze','exec'),{'train_mode':mode,'model_ptr':model})
        freezing[mode]={k:p.requires_grad for k,p in model.named_parameters()}
    assert all(v for k,v in freezing['cov'].items() if 'cov_update' in k) and not any(v for k,v in freezing['cov'].items() if 'cov_update' not in k)
    assert not any(v for k,v in freezing['flow'].items() if 'cov_update' in k)
    assert all(freezing['finalcov'].values())
    probes={'status':'PASS','raw_variance_loss':raw_only,'sequence_cov_mask_probes':seq,'multi_step_loss':multi.item(),'final_only_loss':final.item(),'inference_stub':{'raw':raw_probe.flatten().tolist(),'variance':variance.flatten().tolist(),'mapping':'exp(2*raw)','model_image_forward':False},'logstd_consistent_objective_at_raw_half':consistent,'pointwise_raw_loss_optimum_for_squared_error_four':4-eps,'logstd_objective_optimum_for_squared_error_four':math.log(2),'trainer_parameter_freezing_stub':freezing,'compiler_decorator':'removed only for isolated exact source probes; no compiled training run'}
    dump(output/'probes.json',probes)
    checkpoints={}
    for name,file,expected in [('rgb','MACVO_FrontendCov.pth','bec6edd7e195bab863132f1e9659cdd26e6eaeae7cfd24a626828de294cf5b3a'),('thermal','MACT_Offroad.pth','eed20722b73c80f016c4646741def4e4e5e2999feaf2c2715c5a6293dc35287a')]:
        path=root/'models/macslam'/file;digest=sha(path);assert digest==expected
        state=torch.load(path,map_location='cpu',weights_only=True);assert isinstance(state,dict)
        non_tensor={k:type(v).__name__ for k,v in state.items() if not isinstance(v,torch.Tensor)}
        metadata=getattr(state,'_metadata',None)
        metadata_schema={} if metadata is None else {k:sorted(v) if isinstance(v,dict) else type(v).__name__ for k,v in metadata.items()}
        checkpoints[name]={'file':file,'sha256':digest,'bytes':path.stat().st_size,'container_type':type(state).__name__,'entries':len(state),'tensor_entries':sum(isinstance(v,torch.Tensor) for v in state.values()),'non_tensor_entries':non_tensor,'metadata_present':metadata is not None,'metadata_entry_schema':metadata_schema,'parameter_schema':{k:{'shape':list(v.shape),'dtype':str(v.dtype)} for k,v in state.items() if isinstance(v,torch.Tensor)},'training_recipe_in_top_level':False if not non_tensor else 'inspect','loaded_weights_only':True,'model_instantiated_or_forwarded':False}
        assert not non_tensor;assert sha(path)==digest
    dump(output/'checkpoint-containers.json',checkpoints)
    assert all(sha(repo/path)==info['sha256'] for path,info in registry.items())
    evidence={'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'starting_commit':'b727cb67e','branch':subprocess.check_output(['git','rev-parse','--abbrev-ref','HEAD'],cwd=repo,text=True).strip(),'script_sha256':sha(__file__),'source_files':registry,'ast_cov_loss_loaded_names':uses,'submodule_flowformer_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo/'Module/Network/FlowFormer',text=True).strip(),'runtime':{'python':platform.python_version(),'torch':torch.__version__,'cpu_only_probes':True,'cuda_available':torch.cuda.is_available()},'scope':'current source-contract/checkpoint containers; no historical training, calibrated covariance or causal transfer claim'}
    dump(output/'source-audit.json',evidence);print(json.dumps({'status':'PASS','source_files':len(registry),'probes':probes,'checkpoints':{k:{key:value for key,value in v.items() if key not in ['parameter_schema','metadata_entry_schema']} for k,v in checkpoints.items()}},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.root,a.output)
