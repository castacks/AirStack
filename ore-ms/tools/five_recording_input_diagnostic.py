"""Retrospective exact source-recipe diagnostic after retained EXP128 prep failure."""
import sys,hashlib
import cv2,numpy as np,yaml
from five_recording_inputs import ROOT,OUT,CAL,STAGE,stage_verify,read,write,sha
sys.path.insert(0,str(ROOT/'MAC-VO-main'))
from Scripts.Setup.td3_msa_replay import msa_rectification
from Scripts.Setup.td3_thermal_provenance import producer_functions,preprocess,compare_image

def main():
 stage=stage_verify();lock=read(OUT/'image-lock.json')
 for p,h in lock['sha256'].items():assert sha(ROOT/p)==h,p
 cv2.setNumThreads(1);cams,t,rect,maps=msa_rectification(CAL/'msa_cal')
 cfg=yaml.safe_load((CAL/'upstream/get_thermal_data.yaml').read_text());f=producer_functions(CAL/'upstream/get_thermal_data.py');old=f['read_kalibr_stereo'](cfg)
 output=[];groups={}
 for w in stage['windows']:
  source=OUT/'raw'/w['clip'];rr=[]
  metadata=(ROOT/w['metadata_source']).parent/'source'
  sourceinfo=[yaml.safe_load((metadata/'sensors'/('thermal_'+side+'_processed_camera_info')/'data.yaml').read_text()) for side in ['left','right']]
  for pos,(row,assoc) in enumerate(zip(w['rows'],w['raw_associations'])):
   raw=[cv2.imread(str(source/assoc[s]['path']),cv2.IMREAD_UNCHANGED) for s in ['left','right']]
   if any(im is None or im.dtype!=np.uint16 or im.shape!=(512,640) for im in raw):raise ValueError('raw schema')
   previous=f['stereo_rectify'](*[im.copy() for im in raw],old)
   for i,side in enumerate(['left','right']):
    given=cv2.imread(str(source/row['thermal_'+side]['path']),cv2.IMREAD_UNCHANGED)
    old_image=preprocess(previous[i],f,cfg['process'],cfg['enhance'])
    msa=preprocess(cv2.remap(raw[i],*maps[i],cv2.INTER_LINEAR),f,cfg['process'],cfg['enhance'])
    result={'case':w['case'],'position':pos,'side':side,'original_producer':compare_image(given,old_image),
      'frozen_msa_target':compare_image(given,msa),'source_P_equals_frozen_MSA':np.array_equal(np.asarray(sourceinfo[i]['p']),rect[i+2]),
      'source_P_max_abs_difference_MSA':float(np.max(np.abs(np.asarray(sourceinfo[i]['p'])-rect[i+2]))),
      'raw_source':assoc[side],'provided_source':row['thermal_'+side]}
    rr.append(result);output.append(result)
  groups[w['case']]={'comparisons':len(rr),'original_producer_exact':sum(r['original_producer']['exact_full_array_equal'] for r in rr),
      'frozen_msa_target_exact':sum(r['frozen_msa_target']['exact_full_array_equal'] for r in rr),
      'source_P_equals_MSA':sum(r['source_P_equals_frozen_MSA'] for r in rr)}
  print(w['case'],groups[w['case']],flush=True)
 write(OUT/'input-diagnostic.json',{'scope':'fixed recipe comparisons after originalcontrol failure; not recipe search or native success',
   'groups':groups,'rows':output,'native_executed':False,'source_stage_sha256':sha(STAGE),'source_sha256':sha(__file__)})
if __name__=='__main__':main()
