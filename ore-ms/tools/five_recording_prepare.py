"""Exact original thermal producer controls followed by unchanged MSA preparation."""
import json,shutil,sys
from pathlib import Path
import cv2,numpy as np,yaml
from five_recording_inputs import ROOT,OUT,CAL,STAGE,stage_verify,read,write,sha
sys.path.insert(0,str(ROOT/'MAC-VO-main'))
from Scripts.Setup.td3_msa_replay import msa_rectification
from Scripts.Setup.td3_thermal_provenance import producer_functions,preprocess,compare_image


def prepare():
 stage=stage_verify();lock=read(OUT/'image-lock.json')
 assert lock['stage_plan_sha256']==sha(STAGE)
 for n,h in lock['sha256'].items():assert sha(ROOT/n)==h,n
 cv2.setNumThreads(1);cameras,t,rect,maps=msa_rectification(CAL/'msa_cal')
 cfg=yaml.safe_load((CAL/'upstream/get_thermal_data.yaml').read_text());funcs=producer_functions(CAL/'upstream/get_thermal_data.py')
 original_cal=funcs['read_kalibr_stereo'](cfg);controls=[];schemas=[]
 for w in stage['windows']:
  dest=ROOT/w['dataset'];dest.mkdir(parents=True,exist_ok=False);rawroot=OUT/'raw'/w['clip']
  metadata=ROOT/w['metadata_source'];metadata=metadata.parent/'source'
  for name in ['multisense_left_rect_color_camera_info','multisense_right_camera_info','thermal_left_processed_camera_info','thermal_right_processed_camera_info']:
   src=metadata/'sensors'/name/'data.yaml';target=dest/'sensors'/name/'data.yaml';target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,target)
  for i,side in enumerate(['left','right']):
   target=dest/'sensors'/('thermal_'+side+'_processed_camera_info')/'data.yaml';info=yaml.safe_load(target.read_text())
   info.update(k=cameras[i][1].tolist(),d=np.r_[cameras[i][2],0.].tolist(),distortion_model='equidistant',r=rect[i].tolist(),p=rect[i+2].tolist())
   target.write_text(yaml.safe_dump(info,sort_keys=False))
  rgbinfo=[yaml.safe_load((dest/'sensors'/s/'data.yaml').read_text()) for s in ['multisense_left_rect_color_camera_info','multisense_right_camera_info']]
  for position,(row,assoc) in enumerate(zip(w['rows'],w['raw_associations'])):
   raw=[cv2.imread(str(rawroot/assoc[s]['path']),cv2.IMREAD_UNCHANGED) for s in ['left','right']]
   if any(im is None or im.dtype!=np.uint16 or im.shape!=(512,640) for im in raw):raise ValueError('rawthermal schema mismatch')
   old=funcs['stereo_rectify'](*[im.copy() for im in raw],original_cal)
   for i,side in enumerate(['left','right']):
    provided=cv2.imread(str(rawroot/row['thermal_'+side]['path']),cv2.IMREAD_UNCHANGED)
    control=compare_image(provided,preprocess(old[i],funcs,cfg['process'],cfg['enhance']))
    controls.append({'case':w['case'],'position':position,'side':side,'control':control})
    if not control['exact_full_array_equal']:raise ValueError('original producer replay mismatch; no alternate recipe')
    processed=preprocess(cv2.remap(raw[i],*maps[i],cv2.INTER_LINEAR),funcs,cfg['process'],cfg['enhance'])
    p=dest/row['thermal_'+side]['path'];p.parent.mkdir(parents=True,exist_ok=True)
    assert cv2.imwrite(str(p),np.repeat(processed[:,:,None],3,axis=2))
    rgb=rawroot/row['rgb_'+side]['path'];image=cv2.imread(str(rgb),cv2.IMREAD_UNCHANGED);info=rgbinfo[i]
    if image is None or image.dtype!=np.uint8 or image.shape!=(info['height'],info['width'],3):raise ValueError('RGBcamera-info schema mismatch')
    p=dest/row['rgb_'+side]['path'];p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(rgb,p)
   schemas.append({'case':w['case'],'position':position,'rawthermal_dtype':'uint16','rawthermal_shape':[512,640],
      'preparedthermal_dtype':'uint8','preparedthermal_shape':[512,640,3],'RGBdtype':'uint8'})
  write(dest/'manifest.json',{'pairs':w['rows'],'source':'EXP128 frozen MSA survey; no source rows/times changed'})
  print('PREPARED',w['case'],flush=True)
 write(OUT/'preparation-controls.json',{'original_producer_controls':controls,'schema_checks':schemas,
      'rectification':[a.tolist() for a in rect[:4]],'T_right_from_left':t.tolist(),'physical_body_transform_verified':False,
      'source_recipe_changed':False,'source_indices_or_timestamps_changed':False})
 write(OUT/'prepared-lock.json',{'stage_plan_sha256':sha(STAGE),'image_lock_sha256':sha(OUT/'image-lock.json'),
    'source_sha256':sha(__file__),'original_producer_control_count':len(controls),
    'sha256':{str(p.relative_to(ROOT)):sha(p) for p in (OUT/'prepared').rglob('*') if p.is_file()},
    'preparation_controls_sha256':sha(OUT/'preparation-controls.json'),'reference_labels_read':False})
 stage_verify();print('PASS',len(controls),'exact originalproducer controls;',len(schemas),'fourstream schema checks')
if __name__=='__main__':prepare()
