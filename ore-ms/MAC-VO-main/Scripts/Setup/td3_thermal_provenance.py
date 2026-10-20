"""Bounded thermal producer replay and signed-stereo diagnostics; never rewrite inputs."""
import argparse
import ast
from datetime import datetime, timezone
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from .audit_td3_geometry import compare_thermal_rectification, compare_calibration, static_query, pose_matrix, transform_difference, rectification_residual
from .td3_night_geometry import verify
from .td3_tracking_uncertainty import sha, percentiles

FUNCTIONS=('read_kalibr_stereo','stereo_rectify','process_image','enhance_image')


class OpenCvArrayCompatibility:
    """Canonicalize only stereoRectify R/T array layout for OpenCV5."""
    def __getattr__(self, name):
        return getattr(cv2, name)

    @staticmethod
    def stereoRectify(k0,d0,k1,d1,size,r,t,**kwargs):
        return cv2.stereoRectify(k0,d0,k1,d1,size,np.ascontiguousarray(r),
                               np.ascontiguousarray(t).reshape(3,1),**kwargs)



def producer_functions(path):
    tree=ast.parse(Path(path).read_text())
    nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in FUNCTIONS]
    if {n.name for n in nodes}!=set(FUNCTIONS):raise ValueError('Producer function signature set changed')
    module=ast.Module(body=nodes,type_ignores=[])
    scope={'np':np,'cv2':OpenCvArrayCompatibility(),'cam_dtype':np.float64}
    exec(compile(module,str(path),'exec'),scope)
    return {k:scope[k] for k in FUNCTIONS}


def preprocess(raw, functions, process_type='firestereo_v2', enhance=True):
    raw=np.asarray(raw)
    if raw.ndim!=2 or raw.dtype!=np.uint16 or not raw.size:raise ValueError('Expected nonempty native raw uint16 image')
    if not np.isfinite(raw).all() or np.ptp(raw)==0:raise ValueError('Constant or nonfinite raw image')
    ordered=np.sort(raw.reshape(-1))
    if ordered[round(len(ordered)*.99)-1]<=ordered[round(len(ordered)*.01)]:raise ValueError('Degenerate percentile range')
    result=functions['process_image'](raw.copy(),process_type)
    return functions['enhance_image'](result) if enhance else result


def compare_image(provided, candidate):
    provided=np.asarray(provided);candidate=np.asarray(candidate)
    if provided.ndim==3:
        if provided.shape[2]!=3:raise ValueError('Unexpected provided channel count')
        replicated=bool(np.array_equal(provided[:,:,0],provided[:,:,1]) and np.array_equal(provided[:,:,0],provided[:,:,2]))
        gray=cv2.cvtColor(provided,cv2.COLOR_BGR2GRAY)
    elif provided.ndim==2:replicated=None;gray=provided
    else:raise ValueError('Unexpected provided image shape')
    if candidate.shape!=gray.shape or candidate.dtype!=np.uint8 or provided.dtype!=np.uint8:raise ValueError('Processed image shape/type mismatch')
    delta=np.abs(gray.astype(np.int16)-candidate.astype(np.int16))
    candidate_channels=np.repeat(candidate[:,:,None],3,axis=2) if provided.ndim==3 else candidate
    return dict(provided_shape=list(provided.shape),channels_replicated=replicated,
                exact_full_array_equal=bool(np.array_equal(provided,candidate_channels)),
                gray_exact_equal=bool(np.array_equal(gray,candidate)),
                gray_equal_fraction=float((delta==0).mean()),gray_mae=float(delta.mean()),gray_max_abs=int(delta.max()))


def triangulate(p1,p2,uv0,uv1):
    uv0,uv1=np.asarray(uv0,float),np.asarray(uv1,float)
    if len(uv0)==0:return {'count':0,'finite_count':0,'positive_both_count':0,'positive_both_fraction':None,'depth0_m':percentiles([]),'reprojection_px':percentiles([])},np.empty((0,3))
    hom=cv2.triangulatePoints(np.asarray(p1,float),np.asarray(p2,float),uv0.T,uv1.T).T
    xyz=np.full((len(hom),3),np.nan)
    valid=np.isfinite(hom).all(axis=1)&(np.abs(hom[:,3])>1e-12)
    xyz[valid]=hom[valid,:3]/hom[valid,3:4]
    world=np.column_stack([xyz,np.ones(len(xyz))]);h1=world@p1.T;h2=world@p2.T
    finite=valid&np.isfinite(xyz).all(axis=1)&(np.abs(h1[:,2])>1e-12)&(np.abs(h2[:,2])>1e-12)
    projected0=np.full_like(uv0,np.nan);projected1=np.full_like(uv1,np.nan)
    projected0[finite]=h1[finite,:2]/h1[finite,2:3];projected1[finite]=h2[finite,:2]/h2[finite,2:3]
    positive=finite&(h1[:,2]>0)&(h2[:,2]>0)
    error=np.sqrt((np.sum((projected0-uv0)**2,axis=1)+np.sum((projected1-uv1)**2,axis=1))/2)
    result=dict(count=len(xyz),finite_count=int(finite.sum()),positive_both_count=int(positive.sum()),
                positive_both_fraction=float(positive.sum()/finite.sum()) if finite.any() else None,
                depth0_m=percentiles(h1[:,2]),reprojection_px=percentiles(error))
    return result,xyz


def match_stereo(left,right,p1,p2):
    def gray(image):return cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) if image.ndim==3 else image
    cv2.setRNGSeed(0)
    sift=cv2.SIFT_create(nfeatures=2000)
    kl,dl=sift.detectAndCompute(gray(left),None);kr,dr=sift.detectAndCompute(gray(right),None)
    result=dict(left_keypoints=len(kl),right_keypoints=len(kr),mutual_matches=0,fundamental_inliers=0,status='no_descriptors')
    empty={'uv0':np.empty((0,2)),'uv1':np.empty((0,2)),'inlier':np.empty(0,bool)}
    if dl is None or dr is None:return result,empty
    matcher=cv2.BFMatcher()
    reverse=matcher.knnMatch(dr,dl,k=2)
    back={a.queryIdx:a.trainIdx for pair in reverse if len(pair)==2 for a,b in [pair] if a.distance<.7*b.distance}
    matches=[a for pair in matcher.knnMatch(dl,dr,k=2) if len(pair)==2 for a,b in [pair] if a.distance<.7*b.distance and back.get(a.trainIdx)==a.queryIdx]
    uv0=np.array([kl[m.queryIdx].pt for m in matches],float).reshape(-1,2)
    uv1=np.array([kr[m.trainIdx].pt for m in matches],float).reshape(-1,2)
    result.update(mutual_matches=len(matches),status='insufficient_matches')
    arrays=dict(uv0=uv0,uv1=uv1,inlier=np.zeros(len(matches),bool))
    if len(matches)<8:return result,arrays
    f,mask=cv2.findFundamentalMat(uv0,uv1,cv2.FM_RANSAC,1.5,.999)
    if mask is None:result['status']='no_fundamental_fit';return result,arrays
    keep=mask.ravel().astype(bool);delta=uv0[keep]-uv1[keep]
    result.update(status='ok',fundamental_inliers=int(keep.sum()),vertical_px=percentiles(np.abs(delta[:,1])),
                  signed_disparity_px=percentiles(delta[:,0]),positive_disparity_fraction=float((delta[:,0]>0).mean()) if len(delta) else None)
    flipped=np.asarray(p2).copy();flipped[:,3]*=-1
    for label,projection in [('saved_signed_projection',p2),('translation_sign_diagnostic',flipped)]:
        metrics,xyz=triangulate(np.asarray(p1),np.asarray(projection),uv0[keep],uv1[keep])
        result[label]=metrics;arrays[label+'_xyz']=xyz
    arrays.update(inlier=keep,fundamental=f)
    return result,arrays


def prepare(inputs,source,output):
    inherited=json.loads((source/'plan.json').read_text());files=dict(inherited['files_sha256'])
    extraction=json.loads((inputs/'extraction.json').read_text())
    # Extraction metadata is host-stamped; path mapping is explicit and limited to ore-ms.
    for sample in extraction['samples']:
        for stream in sample['streams'].values():
            for key in ['raw_path','processed_path']:
                stream[key]=stream[key].replace('/root/AirStack/ore-ms/','/workspace/',1)
    paths=[source/'plan.json',Path(__file__),Path(__file__).parents[1]/'UnitTest/test_td3_thermal_provenance.py',
           Path('/workspace/data/calibration/yamaha.yaml'),Path('/workspace/data/calibration/upstream/get_thermal_data.py'),
           Path('/workspace/data/calibration/upstream/get_thermal_data.yaml'),*sorted(inputs.rglob('*'))]
    dataset=Path('/workspace/data/tartandrive3/20251027/night_thermal_so_salon2')
    paths.extend(dataset/f'sensors/thermal_{side}_processed_camera_info/data.yaml' for side in ['left','right'])
    for sample in extraction['samples']:
        paths.extend(Path(s['processed_path']) for s in sample['streams'].values())
    files.update({str(p.resolve()):sha(p) for p in paths if p.is_file()});verify(files)
    output.mkdir(parents=True,exist_ok=False)
    plan=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='3ac07f047',source=str(source),inputs=str(inputs),dataset=str(dataset),
              producer='/workspace/data/calibration/upstream/get_thermal_data.py',config='/workspace/data/calibration/upstream/get_thermal_data.yaml',calibration='/workspace/data/calibration/yamaha.yaml',
              samples=extraction['samples'],files_sha256=files,
              protocol=dict(positions=[0,9,19],source_functions=list(FUNCTIONS),preprocess='unchanged producer rectify->firestereo_v2->enhance',
                  alternatives=['without_enhance','direct_config_matrix'],SIFT_features=2000,mutual_ratio=.7,RANSAC_px=1.5,RANSAC_confidence=.999,seed=0,
                  triangulation='same processed inliers with saved P and flipped right translation, no corrected input adopted',
                  compatibility='OpenCV5 contiguous R and column T adapter; original AST bodies unchanged',scope='bounded computational provenance and sign diagnostics, not historical provenance/metric calibration verification'))
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')


def run(output):
    cv2.setNumThreads(1)
    plan=json.loads((output/'plan.json').read_text());verify(plan['files_sha256'])
    funcs=producer_functions(plan['producer']);cfg=yaml.safe_load(Path(plan['config']).read_text());cal=funcs['read_kalibr_stereo'](cfg)
    dataset=Path(plan['dataset']);infos=[yaml.safe_load((dataset/f'sensors/thermal_{s}_processed_camera_info/data.yaml').read_text()) for s in ['left','right']]
    p1,p2=[np.array(info['p']) for info in infos]
    physical=yaml.safe_load(Path(plan['calibration']).read_text())
    edges=[(e['from_frame'],e['to_frame'],pose_matrix(e['translation']+e['quaternion'])) for e in physical['transform_params']]
    relative=static_query(edges,'thermal_left/optical_frame','thermal_right/optical_frame')
    stored=np.array(cfg['calib']['cam1']['T_cn_cnm1'])
    geometry=dict(matrix_reproduction=compare_thermal_rectification(dataset,Path(plan['config'])),
        body_extrinsics_comparison=compare_calibration(dataset,Path(plan['calibration'])),
        direction_candidates={label:dict(T_right_from_left=t.tolist(),difference_from_body_right_from_left=transform_difference(t,np.linalg.inv(relative)),
            saved_rectification_residual=rectification_residual(np.linalg.inv(t),*infos)) for label,t in [('kalibr_as_stored',stored),('producer_inverse',np.linalg.inv(stored))]})
    (output/'arrays').mkdir();records=[]
    for sample in plan['samples']:
        tag=f"{sample['window']}-{sample['position']:02d}"
        raw=[cv2.imread(sample['streams'][s]['raw_path'],cv2.IMREAD_UNCHANGED) for s in ['left','right']]
        provided=[cv2.imread(sample['streams'][s]['processed_path'],cv2.IMREAD_UNCHANGED) for s in ['left','right']]
        if any(im is None for im in [*raw,*provided]):raise ValueError('Missing image')
        if any(im.shape!=(512,640) or im.dtype!=np.uint16 for im in raw):raise ValueError('Unexpected native raw format')
        rect=funcs['stereo_rectify'](*[r.copy() for r in raw],cal)
        direct=dict(cal);direct['T_right2left']=np.linalg.inv(cal['T_right2left'])
        direct_rect=funcs['stereo_rectify'](*[r.copy() for r in raw],direct)
        record=dict(window=sample['window'],position=sample['position'],images={},geometry={})
        saved={}
        for i,side in enumerate(['left','right']):
            candidate=preprocess(rect[i],funcs,cfg['process'],cfg['enhance'])
            no_enhance=preprocess(rect[i],funcs,cfg['process'],False)
            alternate=preprocess(direct_rect[i],funcs,cfg['process'],cfg['enhance'])
            record['images'][side]=dict(raw_dtype=str(raw[i].dtype),raw_shape=list(raw[i].shape),raw_range=[int(raw[i].min()),int(raw[i].max())],
                timestamp_delta_ns=sample['streams'][side]['delta_ns'],producer=compare_image(provided[i],candidate),
                no_enhance=compare_image(provided[i],no_enhance),direct_matrix=compare_image(provided[i],alternate))
            saved.update({side+'_raw':raw[i],side+'_provided':provided[i],side+'_regenerated':candidate,
                          side+'_without_enhance':no_enhance,side+'_direct_matrix':alternate})
        record['geometry'],matches=match_stereo(*provided,p1,p2)
        saved.update({'matches_'+k:v for k,v in matches.items()})
        np.savez_compressed(output/'arrays'/(tag+'.npz'),**saved)
        records.append(record)
    analysis=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit=plan['base_commit'],opencv=cv2.__version__,
                  numpy=np.__version__,samples=records,geometry=geometry,verified_hashes=verify(plan['files_sha256']))
    (output/'analysis.json').write_text(json.dumps(analysis,indent=2,allow_nan=False)+'\n');plot(output)


def plot(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,4,figsize=(12,7))
    for row,window in enumerate(['early','middle','late']):
        with np.load(output/'arrays'/f'{window}-00.npz') as data:
            for col,(key,title) in enumerate([('left_provided','Provided'),('left_regenerated','Producer replay'),('left_direct_matrix','Direct matrix diagnostic')]):
                axes[row,col].imshow(data[key],cmap='gray',vmin=0,vmax=255);axes[row,col].set_title(f'{window}: {title}');axes[row,col].axis('off')
            prov=data['left_provided'];gray=cv2.cvtColor(prov,cv2.COLOR_BGR2GRAY) if prov.ndim==3 else prov
            delta=np.abs(gray.astype(float)-data['left_regenerated'])
            axes[row,3].imshow(delta,cmap='magma',vmin=0,vmax=255);axes[row,3].set_title('Replay abs difference [0,255]');axes[row,3].axis('off')
    fig.tight_layout();fig.savefig(output/'producer-replay.png',dpi=180);fig.savefig(output/'producer-replay.pdf');plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['prepare','run'])
    parser.add_argument('--inputs',type=Path);parser.add_argument('--source',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.action=='prepare':prepare(args.inputs,args.source,args.output)
    else:run(args.output)
