"""Bounded source-image readiness, independent LK support without learned inference."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json
import cv2
import numpy as np
from .td3_low_motion import lk_track,track_masks


def run(root):
    cv2.setNumThreads(1);cv2.setRNGSeed(0)
    base=root/'results/waypoint-skip2-readiness';dest=base/'points';dest.mkdir(exist_ok=False)
    cases=[];pair_rows=[];image_rows=[];files={}
    def freeze(p):files[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    for group,folder in [('new','waypoint-skip2-readiness'),('prior-dark','new-night-validation')]:
        parent=root/'results'/folder
        extraction_path=parent/'image-extraction.json';freeze(extraction_path)
        extraction=json.loads(extraction_path.read_text())
        dataset=root/'data/tartandrive3/20251027'/('night_so_waypoint_skip2' if group=='new' else 'night_mac_salon_warehousefig8')
        for rel,rec in extraction['files'].items():
            p=dataset/rel;freeze(p);assert files[str(p)]==rec['sha256']
        previews=[]
        for case in extraction['cases']:
            case_name=group+'-'+case['case'];images_by_modality={}
            for modality in ['rgb','thermal']:
                images=[cv2.imread(str(dataset/r[modality+'_left']['path']),cv2.IMREAD_GRAYSCALE) for r in case['rows']]
                assert all(im is not None and im.dtype==np.uint8 and im.shape==images[0].shape for im in images)
                images_by_modality[modality]=images
                h,w=images[0].shape
                yy,xx=np.meshgrid(np.arange(32,h-32,16),np.arange(32,w-32,16),indexing='ij')
                uv=np.column_stack([xx.ravel(),yy.ravel()]).astype(np.float32)
                supports=[];tight_supports=[];texture_supports=[];means=[];stds=[]
                for i,im in enumerate(images):
                    means.append(float(im.mean()));stds.append(float(im.std()))
                    image_rows.append(dict(case=case_name,modality=modality,frame=i,height=h,width=w,mean_gray=float(im.mean()),std_gray=float(im.std()),p01=float(np.percentile(im,1)),p99=float(np.percentile(im,99))))
                for i,(first,second) in enumerate(zip(images,images[1:])):
                    end,back,status,reverse,err,rerr=lk_track(first,second,uv)
                    valid,tight,fb,raw=track_masks(uv,end,back,status,reverse,w,h)
                    x=first.astype(float);mean=cv2.boxFilter(x,-1,(21,21));std=np.sqrt(np.maximum(cv2.boxFilter(x*x,-1,(21,21))-mean*mean,0))
                    texture=std[uv[:,1].astype(int),uv[:,0].astype(int)]
                    tex=valid&(texture>=5)
                    supports.append(float(valid.mean()));tight_supports.append(float(tight.mean()));texture_supports.append(float(tex.mean()))
                    row=dict(case=case_name,modality=modality,pair=i,grid_count=len(uv),valid_count=int(valid.sum()),tight_count=int(tight.sum()),texture_valid_count=int(tex.sum()),median_motion_px=None if not valid.any() else float(np.median(np.linalg.norm(end[valid]-uv[valid],axis=1))))
                    pair_rows.append(row)
                    np.savez_compressed(dest/f'{case_name}-{modality}-{i:03d}.npz',uv=uv,endpoint=end,backward=back,status=status,reverse_status=reverse,fb=fb,valid=valid,tight=tight,texture=texture,texture_valid=tex)
                cases.append(dict(case=case_name,modality=modality,frames=len(images),pairs=len(supports),grid_count=len(uv),mean_gray_median=float(np.median(means)),std_gray_median=float(np.median(stds)),lk_support_min=min(supports),lk_support_median=float(np.median(supports)),lk_support_max=max(supports),tight_support_median=float(np.median(tight_supports)),texture_support_median=float(np.median(texture_supports))))
                print(case_name,modality,'LK median',np.median(supports),flush=True)
            if group=='new':
                for position in [0,10,19]:
                    cells=[]
                    for modality in ['rgb','thermal']:
                        im=cv2.resize(images_by_modality[modality][position],(512,256));im=cv2.cvtColor(im,cv2.COLOR_GRAY2BGR)
                        cv2.putText(im,f'{case["case"]} {modality} frame {position}',(10,24),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),1)
                        cells.append(im)
                    previews.append(np.hstack(cells))
        if group=='new':cv2.imwrite(str(base/'contact-sheet.png'),np.vstack(previews))
    for p in [Path(__file__),Path(__file__).with_name('td3_low_motion.py'),base/'preflight.json',base/'staging/preflight.py',base/'staging/stage_images.py',base/'staging/inspect_archive.py']:freeze(p)
    result=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='7b59219db',opencv=cv2.__version__,cases=cases,pairs=pair_rows,images=image_rows,files_sha256=files,scope='Source processed thermal, not MSA replay. Algorithmic support, not physical accuracy or model inference. Native resolutions/views differ; no modality ranking.')
    (base/'analysis.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':run(Path('/workspace'))
