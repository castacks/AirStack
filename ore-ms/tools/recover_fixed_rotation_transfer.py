"""Recover the predeclared two primary100 recordings; verify archive/member pins."""
from pathlib import Path
import hashlib
import json
import tarfile

ROOT=Path(__file__).resolve().parents[1]
TMP=Path('/tmp/thermal-intake')
OUT=ROOT/'results/fixed-rotation-transfer/recovery'


def sha(path):
    with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def main():
    OUT.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((TMP/'unweighted-longer-manifest.json').read_text())
    archive=TMP/'unweighted-longer.tar.gz';assert sha(archive)==manifest['archive_sha256']
    protocol=json.loads((ROOT/'results/thermal-incremental-comparison/recovery/binding-lock.json').read_text())['source_protocol']
    selected={}
    for c in ['day','night']:
        case=protocol['cases'][c]
        selected['ore-ms/'+case['source']]=OUT/'inputs'/c/'source-manifest.json'
        selected['ore-ms/'+case['calibration']]=OUT/'inputs'/c/'calibration.json'
        for camera in ['rgb','thermal']:
            for leaf in ['tensor-map.npz','arbiter-events/events.jsonl','effective-config.json']:
                selected['ore-ms/'+case['captures'][camera]+'/'+leaf]=OUT/'captures'/c/camera/leaf
        for i,path in enumerate(case['references']):
            assert i<100
            selected['ore-ms/'+path]=OUT/'references'/c/f'{i:06}.txt'
    restored=[]
    with tarfile.open(archive) as tf:
        for member in tf:
            if not member.isfile() or member.name not in selected:continue
            b=tf.extractfile(member).read();digest=hashlib.sha256(b).hexdigest()
            assert digest==manifest['member_sha256'][member.name]
            old=member.name.removeprefix('ore-ms/')
            if old in protocol['input_sha256']:assert digest==protocol['input_sha256'][old]
            dest=selected.pop(member.name);dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(b)
            restored.append({'member':member.name,'path':str(dest.relative_to(ROOT)),'sha256':digest})
    assert not selected, list(selected)
    configs=[]
    for c in ['day','night']:
        path=OUT/'captures'/c/'rgb/effective-config.json';d=json.loads(path.read_text())['Odometry']
        assert d['args']['mapping'] is False and d['motion']['type']=='StaticMotionModel'
        assert d['keyframe']['type']=='AllKeyframe' and d['outlier']['type']=='CovarianceSanityFilter'
        assert d['optimizer']['args']['graph_type']=='disp'
        configs.append({'condition':c,'config_sha256':sha(path),'same_state_and_graph_contract':True})
    (OUT/'recovery-proof.json').write_text(json.dumps({'archive_sha256':manifest['archive_sha256'],
        'archive_manifest_sha256':sha(TMP/'unweighted-longer-manifest.json'),'files':restored,
        'configurations':configs,'references_scored':False},indent=2)+'\n')
    print('Recovered verified members',len(restored),'and checked matching native RGB configurations.')


if __name__=='__main__':main()
