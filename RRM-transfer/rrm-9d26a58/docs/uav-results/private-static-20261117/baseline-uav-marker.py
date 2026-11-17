"""Restricted red-fixture observation with calibrated simulated-depth surface point.
Not general object recognition, stereo-depth validation or world-frame grounding.
"""
import math


def detect_red_surface(rgb, depth, intrinsic, *, max_depth_spread_m=.1):
    import cv2
    import numpy as np
    if type(max_depth_spread_m) not in (int,float) or not math.isfinite(max_depth_spread_m) or max_depth_spread_m<=0:
        raise ValueError('finite positive fixture depth-spread limit required')
    if (rgb.ndim!=3 or rgb.shape[2]!=3 or rgb.dtype!=np.uint8 or depth.ndim!=2 or depth.shape!=rgb.shape[:2] or not np.issubdtype(depth.dtype,np.floating)):
        raise ValueError('aligned RGB uint8 and depth dimensions required')
    if len(intrinsic)!=9 or not all(type(x) in (int,float) and math.isfinite(x) for x in intrinsic):
        raise ValueError('finite 3x3 calibration required')
    fx,fy,cx,cy=intrinsic[0],intrinsic[4],intrinsic[2],intrinsic[5]
    if fx<=0 or fy<=0:raise ValueError('positive focal lengths required')
    if any(abs(intrinsic[i])>1e-9 for i in (1,3,6,7)) or abs(intrinsic[8]-1)>1e-9:
        raise ValueError('unsupported non-pinhole intrinsic matrix')
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    mask=cv2.bitwise_or(cv2.inRange(hsv,(0,100,60),(10,255,255)),cv2.inRange(hsv,(170,100,60),(179,255,255)))
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    count,labels,stats,centroids=cv2.connectedComponentsWithStats(mask)
    ids=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=12]
    if not ids:return dict(status='ABSENT',components=0,surface_point_optical_m=None)
    if len(ids)!=1:return dict(status='AMBIGUOUS',components=len(ids),surface_point_optical_m=None)
    i=ids[0];region=(labels==i).astype(np.uint8);interior=cv2.erode(region,np.ones((3,3),np.uint8)).astype(bool)
    valid=interior & np.isfinite(depth) & (depth>0) & (depth<20)
    box=[int(x) for x in stats[i,:4]];uv=[float(x) for x in centroids[i]]
    result=dict(status='DEPTH_UNKNOWN',components=1,bbox_xywh=box,centroid_uv=uv,
                valid_depth_samples=int(valid.sum()),surface_point_optical_m=None,
                depth_source='simulated_depth_ground_truth',point_semantics='observed surface, not object center')
    if valid.sum()<9 or valid.sum()<.5*interior.sum():return result
    low,high=float(depth[valid].min()),float(depth[valid].max())
    result.update(depth_min_m=low,depth_max_m=high,depth_spread_m=high-low,
                  max_depth_spread_m=float(max_depth_spread_m))
    if high-low>max_depth_spread_m:
        result['status']='DEPTH_INCONSISTENT'
        return result
    vs,us=np.nonzero(valid)
    nearest=int(np.argmin((us-np.median(us))**2+(vs-np.median(vs))**2))
    u,v=int(us[nearest]),int(vs[nearest]);z=float(depth[v,u])
    result['surface_pixel_uv']=[u,v]
    result.update(status='OBSERVED',surface_depth_z_m=z,surface_point_optical_m=[(u-cx)*z/fx,(v-cy)*z/fy,z])
    return result
