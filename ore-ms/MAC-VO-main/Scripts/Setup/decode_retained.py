"""Decode retained encoded stereo input exactly as the existing TD3 uint8 adapter."""
import hashlib
import cv2
import numpy as np
import pypose as pp
import torch
from DataLoader.Interface import StereoFrame,StereoData

def image_bytes(image):
    data=image['data']
    if len(data)!=image['bytes'] or hashlib.sha256(data).hexdigest()!=image['sha256']:raise ValueError('retained bytes/hash mismatch')
    raw=cv2.imdecode(np.frombuffer(data,dtype=np.uint8),cv2.IMREAD_UNCHANGED)
    if raw is None:raise ValueError('retained image decode failed')
    if raw.dtype!=np.uint8:raise ValueError('retained execution requires uint8')
    if raw.ndim==2:raw=cv2.cvtColor(raw,cv2.COLOR_GRAY2RGB)
    elif raw.ndim==3 and raw.shape[-1]==3:raw=cv2.cvtColor(raw,cv2.COLOR_BGR2RGB)
    else:raise ValueError('unsupported retained image shape')
    return torch.from_numpy(raw.copy()).permute(2,0,1).unsqueeze(0).float()/255.

def retained_frame(sequence,source,payload,compact_index):
    if payload['camera']!=sequence.modality or payload['source']!=source:raise ValueError('retained camera/source mismatch')
    row=sequence.rows[source];images=[]
    for side in ['left','right']:
        p=payload['images'][side];record=row[sequence.modality+'_'+side]
        if p['path']!=record['path'] or p['time_ns']!=record['time_ns']:raise ValueError('retained source metadata mismatch')
        image=image_bytes(p)
        if tuple(image.shape[-2:])!=(sequence.height,sequence.width):raise ValueError('retained image dimensions mismatch')
        images.append(image)
    t=[row[sequence.modality+'_left']['time_ns']]
    return StereoFrame(idx=[compact_index],time_ns=t,stereo=StereoData(
        T_BS=pp.identity_SE3(1,dtype=torch.float64),K=sequence.K.clone(),
        baseline=torch.tensor([sequence.baseline],dtype=torch.float64),time_ns=t,
        height=sequence.height,width=sequence.width,imageL=images[0],imageR=images[1]))
