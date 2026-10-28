"""Independent preservation/formula/AST audit of the source-contract experiment."""
import argparse,ast,hashlib,json,math
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def run(root,base):
    audit=base/'local-audit';e=json.loads((audit/'source-audit.json').read_text());p=json.loads((audit/'probes.json').read_text());c=json.loads((audit/'checkpoint-containers.json').read_text());external=json.loads((base/'external-source/manifest.json').read_text());count=0
    def close(a,b):
        nonlocal count
        assert math.isclose(a,b,rel_tol=1e-12,abs_tol=1e-12),(a,b);count+=1
    for relative,record in e['source_files'].items():
        assert sha(root/'MAC-VO-main'/relative)==sha(audit/'source-snapshots'/relative)==record['sha256'];count+=2
    loss_path=root/'MAC-VO-main/Train/MatchingNet/loss.py';tree=ast.parse(loss_path.read_text());cov=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='cov_loss')
    uses={n.id for n in ast.walk(cov) if isinstance(n,ast.Name) and isinstance(n.ctx,ast.Load)}
    assert 'flow_mask' not in uses and 'max_cov' not in uses
    # Inference AST return uses exp(raw * 2); training covariance has no exp call.
    infer=next(n for n in ast.walk(ast.parse((root/'MAC-VO-main/Module/Network/FlowFormerCov/flownet.py').read_text())) if isinstance(n,ast.FunctionDef) and n.name=='inference')
    returned=next(n for n in infer.body if isinstance(n,ast.Return)).value
    exp=returned.elts[1];assert isinstance(exp,ast.Call) and isinstance(exp.func,ast.Attribute) and exp.func.attr=='exp'
    assert isinstance(exp.args[0],ast.BinOp) and isinstance(exp.args[0].op,ast.Mult) and isinstance(exp.args[0].right,ast.Constant) and exp.args[0].right.value==2
    assert not any(isinstance(n,ast.Attribute) and n.attr=='exp' for n in ast.walk(cov));count+=4
    eps=1e-7;s=.5+eps;expected=4/s+math.log(s);close(expected,p['raw_variance_loss']['loss']);close((1/s-4/s**2)/4,p['raw_variance_loss']['covariance_gradient_per_element'][0])
    assert p['raw_variance_loss']['direct_flow_gradient'] is None and p['raw_variance_loss']['negative_raw_loss_nonfinite'];count+=2
    for v in p['sequence_cov_mask_probes']:close(expected,v['loss'])
    close(.85*(1/(1.5+eps)+math.log(1.5+eps))+4/(2.5+eps)+math.log(2.5+eps),p['multi_step_loss']);close(4/(2.5+eps)+math.log(2.5+eps),p['final_only_loss'])
    for raw,var in zip(p['inference_stub']['raw'],p['inference_stub']['variance']):close(math.exp(2*raw),var)
    close(4/math.exp(1)+1,p['logstd_consistent_objective_at_raw_half']);assert not math.isclose(expected,p['logstd_consistent_objective_at_raw_half']);count+=1
    close(4-eps,p['pointwise_raw_loss_optimum_for_squared_error_four']);close(.5*math.log(4),p['logstd_objective_optimum_for_squared_error_four'])
    for mode,params in p['trainer_parameter_freezing_stub'].items():
        for name,flag in params.items():
            expected_flag=('cov_update' in name) if mode=='cov' else ('cov_update' not in name) if mode=='flow' else True
            assert flag==expected_flag;count+=1
    for name,checkpoint in c.items():
        assert sha(root/'models/macslam'/checkpoint['file'])==checkpoint['sha256'];assert checkpoint['entries']==checkpoint['tensor_entries']==len(checkpoint['parameter_schema']);assert checkpoint['non_tensor_entries']=={}
        assert checkpoint['metadata_present'] and all(keys==['version'] for keys in checkpoint['metadata_entry_schema'].values());count+=4
    release=json.loads((base/'external-source/release-rgb-byte-check.json').read_text());assert release['status']=='PASS' and release['sha256']==c['rgb']['sha256'] and release['bytes']==c['rgb']['bytes'];count+=1
    for relative,record in external['files'].items():
        path=base/'external-source'/relative.replace('/','__');assert sha(path)==record['sha256'];local=root/'MAC-VO-main'/relative
        def functions(path):return {n.name:ast.dump(n,include_attributes=False) for n in ast.walk(ast.parse(path.read_text())) if isinstance(n,ast.FunctionDef)}
        a,b=functions(local),functions(path)
        for name in record['ast_identical_functions']:assert a[name]==b[name];count+=1
        for name in record['ast_different_functions']:assert a[name]!=b[name];count+=1
    assert sha(loss_path)==external['files']['Train/MatchingNet/loss.py']['sha256'];assert external['upstream_commit']=='8683b532d6f6e8981f6b1a646fb5f52f02c1ea61';count+=2
    result={'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'starting_commit':'b727cb67e','script_sha256':sha(__file__),'checks':count,'source_files':len(e['source_files']),'checkpoint_tensor_entries':[c[k]['tensor_entries'] for k in ['rgb','thermal']],'upstream_commit':external['upstream_commit'],'evidence_sha256':{str(path.relative_to(base)):sha(path) for path in [audit/'source-audit.json',audit/'probes.json',audit/'checkpoint-containers.json',base/'external-source/manifest.json',base/'external-source/release-rgb-byte-check.json']},'scope':'source contracts, independently checked formulas, metadata/schema and byte preservation; original training history remains unverified'}
    (base/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--base',type=Path,required=True);a=p.parse_args();run(a.root,a.base)
