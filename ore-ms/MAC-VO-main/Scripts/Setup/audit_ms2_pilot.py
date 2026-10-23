"""Inspect frozen MS² night pilot inputs without model inference or calibration fitting.

Calibration unpickling permits only NumPy numeric-array constructors. Retain
uint16 PNG values, inspect source pose rigidity, and verify preserved member hashes.
"""
from pathlib import Path
import pickle,io,json,numpy as np

class ArrayOnlyUnpickler(pickle.Unpickler):
 def find_class(self,module,name):
  allow={('numpy','ndarray'):np.ndarray,('numpy','dtype'):np.dtype,('numpy.core.multiarray','_reconstruct'):np._core.multiarray._reconstruct,('numpy._core.multiarray','_reconstruct'):np._core.multiarray._reconstruct,('numpy.core.multiarray','scalar'):np._core.multiarray.scalar,('numpy._core.multiarray','scalar'):np._core.multiarray.scalar}
  if (module,name) not in allow:raise pickle.UnpicklingError('Disallowed calibration global '+module+'.'+name)
  return allow[module,name]

def read_calibration(path):
 b=Path(path).read_bytes()
 if len(b)>2*1024*1024:raise ValueError('Calibration exceeds bounded schema inspection limit')
 f=io.BytesIO(b);version=np.lib.format.read_magic(f)
 if version==(1,0):shape,fortran,dtype=np.lib.format.read_array_header_1_0(f)
 elif version==(2,0):shape,fortran,dtype=np.lib.format.read_array_header_2_0(f)
 else:raise ValueError('Unsupported calibration NPY version')
 if shape!=() or not dtype.hasobject:raise ValueError('Expected official scalar dictionary NPY')
 arr=ArrayOnlyUnpickler(f).load()
 if not isinstance(arr,np.ndarray) or arr.shape!=():raise ValueError('Unexpected calibration container')
 result=arr.item()
 if not isinstance(result,dict):raise ValueError('Unexpected calibration payload')
 clean={}
 for key,value in result.items():
  if not isinstance(key,str) or not isinstance(value,np.ndarray) or value.dtype.hasobject or value.size>10000 or not np.isfinite(value).all():raise ValueError('Unsupported calibration value')
  clean[key]={'shape':list(value.shape),'dtype':str(value.dtype),'values':value.tolist()}
 return clean


import math,struct,zlib,hashlib
import cv2

def audit(directory):
    base=Path(directory);pilot=base/'pilot';seq='_2021-08-13-21-18-04';checks=0;images=[];poses=[]
    # Independent PNG chunk CRC validation, including decoder precision/geometry.
    for p in sorted(pilot.rglob('*.png')):
     b=p.read_bytes();assert b[:8]==b'\x89PNG\r\n\x1a\n';offset=8;chunks=0
     while offset<len(b):
      length=struct.unpack('>I',b[offset:offset+4])[0];tag=b[offset+4:offset+8];payload=b[offset+8:offset+8+length];expected=struct.unpack('>I',b[offset+8+length:offset+12+length])[0]
      assert zlib.crc32(tag+payload)&0xffffffff==expected;checks+=1;offset+=length+12;chunks+=1
      if tag==b'IEND':break
     assert offset==len(b);checks+=1
     x=cv2.imread(str(p),cv2.IMREAD_UNCHANGED);assert x is not None;checks+=1
     height,width,bit_depth,color=struct.unpack('>IIBB',b[16:26]);assert (x.shape[1],x.shape[0])==(height,width);assert (x.dtype==np.uint16)==(bit_depth==16);checks+=2
     rec=dict(path=str(p.relative_to(base)),shape=list(x.shape),dtype=str(x.dtype),png_bit_depth=bit_depth,png_color_type=color,min=int(x.min()),max=int(x.max()),p1_p50_p99=np.percentile(x,[1,50,99]).tolist(),nonzero_fraction=float(np.count_nonzero(x)/x.size),sha256=hashlib.sha256(b).hexdigest());images.append(rec)
    for p in sorted((pilot/'odom'/seq).rglob('*.txt')):
     fields=p.read_text().split();assert len(fields)==16 and all(math.isfinite(float(v)) for v in fields);checks+=1
     a=[list(map(float,fields[i:i+4])) for i in range(0,16,4)];assert a[3]==[0,0,0,1];checks+=1
     # Scalar R^T R and determinant, independent of array linalg helper.
     R=[r[:3] for r in a[:3]];orth=math.sqrt(sum((sum(R[k][i]*R[k][j] for k in range(3))-(1 if i==j else 0))**2 for i in range(3) for j in range(3)));det=R[0][0]*(R[1][1]*R[2][2]-R[1][2]*R[2][1])-R[0][1]*(R[1][0]*R[2][2]-R[1][2]*R[2][0])+R[0][2]*(R[1][0]*R[2][1]-R[1][1]*R[2][0])
     poses.append(dict(path=str(p.relative_to(base)),orthogonality=orth,determinant=det,passes_1e_5_screen=orth<1e-5 and abs(det-1)<1e-5))
    for kind in ['sync_data','proj_depth']:
     for row in json.loads((base/(kind+'-pilot-manifest.json')).read_text()):
      p=base/row['path'];assert p.stat().st_size==row['bytes'] and hashlib.file_digest(p.open('rb'),'sha256').hexdigest()==row['sha256'];checks+=1
    calibpaths=list(pilot.rglob('calib.npy'));assert len(calibpaths)==1;checks+=1
    calib=read_calibration(calibpaths[0]);(base/'calibration.json').write_text(json.dumps(calib,indent=2)+'\n')
    folders={}
    for label,kind,sensor,folder in [('rgb_left','sync_data','rgb','img_left'),('rgb_right','sync_data','rgb','img_right'),('thr_left','sync_data','thr','img_left'),('thr_right','sync_data','thr','img_right'),('rgb_depth','proj_depth','rgb','depth'),('thr_depth','proj_depth','thr','depth'),('rgb_depth_filtered','proj_depth','rgb','depth_filtered'),('thr_depth_filtered','proj_depth','thr','depth_filtered')]:
     p=pilot/kind/seq/sensor/folder;folders[label]=sorted(x.stem for x in p.glob('*.png'))
    common=sorted(set.intersection(*(set(x) for x in folders.values())));expected=[f'{i:06d}' for i in range(20)];assert common==expected;checks+=1
    out=dict(status='PASS for source/schema inspection; physical reference/timing conventions require separate audit',independent_checks=checks,sequence=seq,common_stems=common,folders=folders,images=images,poses=poses,calibration_keys=list(calib),timestamp_files=[str(p.relative_to(base)) for p in pilot.rglob('*timestamp*') if p.is_file()],publisher_archive_digest_available=False)
    return out

if __name__ == '__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=audit(args.directory)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('Schema inspection controls:',result['independent_checks'])
