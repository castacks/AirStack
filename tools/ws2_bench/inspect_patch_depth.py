"""Run Kim's actual FCRN preprocessing on saved camera images (inside Kim image).

Reports depth-input sensitivity, not patch attack success. Full-frame differences
can include small pose/render differences between captures.
"""
import argparse,json,sys
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,'/workspace/planner')
from airstack_integration.runtime import CollisionAvoidanceRuntime,center_crop_to_aspect

p=argparse.ArgumentParser(description=__doc__);p.add_argument('captures',type=Path)
a=p.parse_args()
model=CollisionAvoidanceRuntime('save_model/D3QN_V_3_single.h5',
    fcrn_checkpoint='airstack_models/NYU_FCRN-checkpoint/NYU_FCRN.ckpt')
depths={}
try:
    for image in sorted(a.captures.glob('*.jpg')):
        rgb=cv2.cvtColor(center_crop_to_aspect(cv2.imread(str(image))),cv2.COLOR_BGR2RGB)
        inputs=cv2.resize(rgb,(304,228),interpolation=cv2.INTER_CUBIC).astype(np.float32)
        raw=model.session.run(model.fcrn_output,{model.fcrn_input:inputs[None]}).reshape(128,160)
        assert np.isfinite(raw).all()
        np.save(a.captures/(image.stem+'_input.npy'),inputs)
        np.save(a.captures/(image.stem+'_fcrn.npy'),raw)
        depths[image.stem]=raw
    clean=depths['clean'];summary={}
    for name,depth in depths.items():
        d=depth-clean
        summary[name]={'mean_prediction':float(depth.mean()),'mean_delta_vs_clean':float(d.mean()),
                       'mean_absolute_delta_vs_clean':float(np.abs(d).mean()),
                       'normalized_policy_input_mean':float(np.clip(depth/3.5,0,1).mean())}
    (a.captures/'fcrn_sensitivity.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))
finally:model.close()
