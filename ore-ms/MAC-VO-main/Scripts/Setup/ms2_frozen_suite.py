"""Declared three-capture EXP064 adapter; preserves EXP063 numerical methods."""
import argparse,json
from pathlib import Path
import numpy as np
import ms2_frozen_recording as predecessor
from ms2_geometry_risk_selection import sha

SUITE_SHA='9781971e743186d0d65f977b3a60d51882e531d1ef33840d1784e0be80435888'
PARENT_SHA='51b2b9a15d1951c2902e6ef8a0a5e45dcae3d1713d428e36d9545b530e0ae377'


def declared(root,suite,condition):
    assert sha(suite/'suite-protocol.json')==SUITE_SHA
    protocol=json.loads((suite/'suite-protocol.json').read_text());record=protocol['records'][condition];base=suite/condition
    assert sha(base/'frozen-models.json')==record['model_artifact_sha256']
    plan=json.loads((base/'frozen-models.json').read_text());parent_path=root/'results/ms2-unexamined-recording/frozen-models.json';assert sha(parent_path)==PARENT_SHA;parent=json.loads(parent_path.read_text())
    assert plan['sequence']==record['sequence'] and plan['primary_family']=='uncertainty' and plan['parent_model_sha256']==PARENT_SHA
    for key in ['models','scalar_fit','frozen_development_inputs','source_sha256']:assert plan[key]==parent[key]
    for name,h in protocol['source_sha256'].items():assert sha(root/'MAC-VO-main/Scripts/Setup'/name)==h
    return protocol,record,base


def binding(function,root,base,digest,*args):
    original=predecessor.FROZEN_SHA256
    try:
        predecessor.FROZEN_SHA256=digest
        return function(root,base,*args)
    finally:predecessor.FROZEN_SHA256=original


def prepare(root,suite,condition):
    protocol,record,base=declared(root,suite,condition)
    binding(predecessor.prepare,root,base,record['model_artifact_sha256'])
    source=json.loads((base/'dataset/source-manifest.json').read_text());old=set()
    prior=root/'results/ms2-unexamined-recording/dataset/source-manifest.json'
    for row in json.loads(prior.read_text())['records']:
        for camera in row['cameras'].values():
            for side in ['left','right']:old.add(camera[side]['sha256'])
    current=[camera[side]['sha256'] for row in source['records'] for camera in row['cameras'].values() for side in ['left','right']]
    assert not set(current)&old,'Overlap with previous test capture'
    (base/'previous-test-overlap.json').write_text(json.dumps({'status':'PASS','previous_test_image_hashes':len(old),'new_camera_images':len(current),'overlap':0},indent=2)+'\n')


def evaluate(root,suite,condition,output):
    protocol,record,base=declared(root,suite,condition)
    binding(predecessor.evaluate,root,base,record['model_artifact_sha256'],output)
    p=output/'report.json';r=json.loads(p.read_text());r['primary']='uncertainty';r['suite']='EXP064';p.write_text(json.dumps(r,indent=2)+'\n')
    p=output/'protocol.json';r=json.loads(p.read_text());r.update(suite_sha256=SUITE_SHA,primary_family='uncertainty',adapter_sha256=sha(__file__),numerical_parent_sha256=protocol['source_sha256']['ms2_frozen_recording.py']);p.write_text(json.dumps(r,indent=2)+'\n')


def parity(root,suite,output):
    assert sha(suite/'suite-protocol.json')==SUITE_SHA
    base=root/'results/ms2-unexamined-recording';assert sha(base/'frozen-models.json')==PARENT_SHA
    binding(predecessor.evaluate,root,base,PARENT_SHA,output)
    original=base/'frozen-evaluation';assert json.loads((original/'report.json').read_text())==json.loads((output/'report.json').read_text())
    checked=0
    for i in range(20):
        with np.load(original/f'selection-{i:06d}.npz') as a,np.load(output/f'selection-{i:06d}.npz') as b:
            assert a.files==b.files
            for key in a.files:assert np.array_equal(a[key],b[key]);checked+=a[key].size
    (suite/'adapter-parity-validation.json').write_text(json.dumps({'status':'PASS','scalar_report_exact':True,'array_values_bitwise':checked,'parent_model_sha256':PARENT_SHA,'suite_sha256':SUITE_SHA,'scope':'implementation replay on already examined EXP063, not new test evidence'},indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['prepare','evaluate','parity']);p.add_argument('--root',type=Path,required=True);p.add_argument('--suite',type=Path,required=True);p.add_argument('--condition',choices=['day','night','rainy']);p.add_argument('--output',type=Path)
    a=p.parse_args();root=a.root.resolve();suite=a.suite.resolve()
    if a.stage=='prepare':prepare(root,suite,a.condition)
    elif a.stage=='evaluate':evaluate(root,suite,a.condition,a.output.resolve())
    else:parity(root,suite,a.output.resolve())
