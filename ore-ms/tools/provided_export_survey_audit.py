"""Byte-exact export input audit and explicit binding of frozen native checker."""
import argparse,sys,zlib
from pathlib import Path
import cv2,numpy as np,yaml
from provided_export_survey import ROOT,OUT,PLAN,PARENT,read,write,sha,verify

def inputs():
 plan=verify();prepared=read(OUT/'prepared-lock.json');assert prepared['plan_sha256']==sha(PLAN);checks=0
 def check(ok,label):
  nonlocal checks
  if not ok:raise AssertionError(label)
  checks+=1
 for path,digest in prepared['sha256'].items():check(sha(ROOT/path)==digest,path)
 crc={path:item for p in (PARENT/'raw').glob('*/image-intake.json') for path,item in read(p)['files'].items()}
 images=infos=0
 for w in plan['windows']:
  target=ROOT/w['dataset'];metadata=(ROOT/w['metadata_source']).parent/'source'
  check(read(target/'manifest.json')['pairs']==w['rows'],'source row/time equality')
  for camera,names in [('rgb',['multisense_left_rect_color_camera_info','multisense_right_camera_info']),
     ('thermal',['thermal_left_processed_camera_info','thermal_right_processed_camera_info'])]:
   cal=[yaml.safe_load((target/'sensors'/name/'data.yaml').read_text()) for name in names]
   p=[np.asarray(c['p'],float) for c in cal]
   check(all(v.shape==(3,4) and np.isfinite(v).all() and v[0,0]>0 and v[1,1]>0 for v in p),'native projection schema')
   check(abs(-p[1][0,3]/p[1][0,0]+p[0][0,3]/p[0][0,0])>0,'native nonzero stereo baseline')
   for i,name in enumerate(names):
    check(sha(metadata/'sensors'/name/'data.yaml')==sha(target/'sensors'/name/'data.yaml'),'camera-info source bytes');infos+=1
    for row in w['rows']:
     key=camera+('_left' if i==0 else '_right');source=PARENT/'raw'/w['clip']/row[key]['path'];dest=target/row[key]['path'];data=source.read_bytes()
     check(sha(source)==sha(dest)==crc[str(source.relative_to(ROOT))]['sha256'],'source image bytes')
     check(zlib.crc32(data)&0xffffffff==crc[str(source.relative_to(ROOT))]['zip_crc32'],'source member CRC')
     image=cv2.imread(str(dest),cv2.IMREAD_UNCHANGED);check(image is not None and image.dtype==np.uint8 and image.shape[:2]==(cal[i]['height'],cal[i]['width']) and
       (image.ndim==2 or image.ndim==3 and image.shape[2]==3),'provided native uint8 schema');images+=1
 check(images==1600 and infos==16,'complete provided population')
 write(OUT/'input-audit.json',{'pass':True,'checks':checks,'source_images_exact':images,'source_camera_info_exact':infos,
  'input_recipe':'archive exports unchanged','physical_calibration_verified':False,'EXP128_control_failure_preserved':True,
  'prepared_lock_sha256':sha(OUT/'prepared-lock.json')})
 print('PASS PROVIDED INPUTS',checks,'checks;',images,'source images;',infos,'camera-info files')

def native():
 # No numerical or eligibility logic is changed in the previously frozen checker.
 # Bind only its artifact root, plan identity and source-integrity provider.
 import five_recording_native_audit as checker
 checker.OUT=OUT;checker.STAGE=PLAN;checker.stage_verify=verify
 checker.native()

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['inputs','native']);a=p.parse_args();globals()[a.action]()
