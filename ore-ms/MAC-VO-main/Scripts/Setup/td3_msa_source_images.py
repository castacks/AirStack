"""Check supplied MSA source JPEGs without fitting stereo calibration."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from .audit_td3_geometry import pose_matrix
from .td3_calibration_reconcile import camera, triangulation, sampson_pixels, rectified_pixels, score
from .td3_msa_calibration_audit import msa_camera
from .td3_tracking_uncertainty import sha, percentiles


def matches(images):
    cv2.setRNGSeed(0)
    sift = cv2.SIFT_create(nfeatures=2000)
    key, desc = zip(*(sift.detectAndCompute(image, None) for image in images))
    if any(d is None for d in desc):
        raise ValueError('Source image has no descriptors')
    matcher = cv2.BFMatcher()
    reverse = matcher.knnMatch(desc[1], desc[0], k=2)
    back = {a.queryIdx: a.trainIdx for pair in reverse if len(pair)==2
            for a,b in [pair] if a.distance < .7*b.distance}
    kept = [a for pair in matcher.knnMatch(desc[0], desc[1], k=2) if len(pair)==2
            for a,b in [pair] if a.distance < .7*b.distance and back.get(a.trainIdx)==a.queryIdx]
    uv = [np.array([key[i][m.queryIdx if i==0 else m.trainIdx].pt for m in kept], float).reshape(-1,2) for i in range(2)]
    if len(kept)<8:
        raise ValueError('Insufficient source-image matches')
    f, mask = cv2.findFundamentalMat(*uv, cv2.FM_RANSAC, 1.5, .999)
    if mask is None or f is None:
        raise ValueError('Fundamental fit failed')
    return uv, mask.ravel().astype(bool), f, [len(k) for k in key]


def run(root, output):
    output.mkdir(parents=True, exist_ok=False)
    source=root/'data/calibration/msa_cal'; inputs=root/'data/calibration/msa_camera_images'
    paths=[inputs/f'thermal_{side}.jpg' for side in ['left','right']]
    original=[cv2.imread(str(p),cv2.IMREAD_UNCHANGED) for p in paths]
    if any(im is None or im.shape[:2]!=(512,640) or im.dtype!=np.uint8 for im in original):
        raise ValueError('Expected 640x512 uint8 source JPEGs')
    images=[cv2.cvtColor(im,cv2.COLOR_BGR2GRAY) if im.ndim==3 else im for im in original]
    cv2.setNumThreads(1)
    uv, inlier, f, counts=matches(images)
    cams=[msa_camera(source/f'thermal_{s}.intrinsics.yaml') for s in ['left','right']]
    k=[c[1] for c in cams];d=[c[2] for c in cams]
    ext=yaml.safe_load((source/'extrinsics.yaml').read_text())
    t=np.linalg.inv(pose_matrix(ext['thermal_right']['value']))@pose_matrix(ext['thermal_left']['value'])
    rect=cv2.fisheye.stereoRectify(k[0],d[0],k[1],d[1],(640,512),t[:3,:3].copy(),t[:3,3].copy(),flags=cv2.CALIB_ZERO_DISPARITY,balance=0,fov_scale=1)
    rays=[]
    for u,ki,di in zip(uv,k,d):
        xy=cv2.fisheye.undistortPoints(u[:,None],ki,di).reshape(-1,2)
        rays.append(np.column_stack([xy,np.ones(len(xy))]))
    config_path=root/'data/calibration/upstream/get_thermal_data.yaml'
    config=yaml.safe_load(config_path.read_text());old=np.array(config['calib']['cam1']['T_cn_cnm1'])
    old_rays=[]
    for i,u in enumerate(uv):
        ki,di=camera(config,f'cam{i}')
        xy=cv2.undistortPointsIter(u[:,None],ki,di,None,None,(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,100,1e-13)).reshape(-1,2)
        old_rays.append(np.column_stack([xy,np.ones(len(xy))]))
    records={}
    for pop,mask in [('all_mutual',np.ones(len(inlier),bool)),('fundamental_inliers',inlier)]:
        a,b=[r[mask] for r in rays];tri=triangulation(a,b,t,*k)
        residual=sampson_pixels(a,b,t,*k)
        delta=rectified_pixels(a,rect[0],rect[2])-rectified_pixels(b,rect[1],rect[3])
        eligible=tri['eligible'];m=dict(count=len(a),eligible_count=int(eligible.sum()),
            positive_eligible_fraction=float(tri['positive'][eligible].mean()) if eligible.any() else None,
            sampson_px=percentiles(residual),vertical_px=percentiles(np.abs(delta[:,1])),
            signed_disparity_px=percentiles(delta[:,0]))
        controls={label:score(*[r[mask] for r in old_rays],pose,config)[0] for label,pose in [('producer_inverse',np.linalg.inv(old)),('kalibr_as_stored',old)]}
        denominator=min(v['sampson_px']['p10_p50_p90'][1] for v in controls.values())
        ratio=m['sampson_px']['p10_p50_p90'][1]/denominator
        checks=dict(eligible_count_pass=m['eligible_count']>=20,
            cheirality_pass=m['positive_eligible_fraction'] is not None and m['positive_eligible_fraction']>=.95,
            residual_pass=ratio<=.75,residual_ratio=ratio)
        records[pop]=dict(msa=m,controls=controls,checks=checks)
        np.savez_compressed(output/f'{pop}.npz',uv0=uv[0][mask],uv1=uv[1][mask],
            rays0=a,rays1=b,sampson_px=residual,rectified_delta=delta,**tri)
    np.savez_compressed(output/'matches.npz',uv0=uv[0],uv1=uv[1],inlier=inlier,fundamental=f)
    previews=[]
    for i in range(2):
        mx,my=cv2.fisheye.initUndistortRectifyMap(k[i],d[i],rect[i],rect[i+2],(640,512),cv2.CV_32FC1)
        image=cv2.remap(original[i],mx,my,cv2.INTER_LINEAR)
        if image.ndim==2:image=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        previews.append(image)
        cv2.imwrite(str(output/f'rectified-{i}.png'),image)
    preview=np.hstack(previews)
    for y in range(16,512,32):cv2.line(preview,(0,y),(1279,y),(0,200,255),1)
    cv2.imwrite(str(output/'stereo-preview.png'),preview)
    files=[Path(__file__),root/'camera_images.zip',*paths,inputs/'manifest.json',config_path,
        *[p for p in source.iterdir() if p.is_file()]]
    analysis=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='03a102194',
        status='DIAGNOSTIC_SOURCE_PAIR_NO_DEFAULT_CHANGE',opencv=cv2.__version__,
        image_shapes=[list(im.shape) for im in original],keypoint_counts=counts,
        matching_protocol='SIFT2000, mutual ratio0.7, F-RANSAC1.5px/.999, seed0, original JPEGs',
        T_right_from_left=t.tolist(),rectification=[r.tolist() for r in rect[:4]],populations=records,
        files_sha256={str(p.resolve()):sha(p) for p in files})
    (output/'analysis.json').write_text(json.dumps(analysis,indent=2,allow_nan=False)+'\n')
    print(json.dumps({key:analysis[key] for key in ['image_shapes','keypoint_counts','populations']},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('/workspace'))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();run(args.root,args.output)
