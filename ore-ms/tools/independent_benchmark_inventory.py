"""Read-only configured-asset and scientific-runtime inventory; no test payloads."""
import json,subprocess
from independent_benchmark_contract import ROOT,OUT,read,write,sha,verify
p=verify();probe="import importlib.util,json,torch; print(json.dumps({'torch':torch.__version__,'cuda_runtime':torch.version.cuda,'modules':{n:importlib.util.find_spec(n) is not None for n in ['droid_slam','droid_backends','dpvo','cv2','pypose','numpy','scipy']}}))"
cmd=['docker','run','--rm','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1',p['runtime_for_analytic_tests'],'python','-c',probe]
result=subprocess.run(cmd,capture_output=True,text=True,check=True);runtime=json.loads(result.stdout.splitlines()[-1])
assets={}
for name,rel in {'orb_checkout':'third_party/ORB_SLAM3','orb_library':'third_party/ORB_SLAM3/lib/libORB_SLAM3.so','orb_vocabulary':'third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt','droid_checkout':'third_party/DROID-SLAM','droid_weights':'models/droid/droid.pth','tartanvo_pose':'models/macslam/MACVO_posenet.pkl','tartanvo_depth':'models/macslam/TartanVO_depth_cov.pth','rgb_checkpoint':'models/macslam/MACVO_FrontendCov.pth','thermal_checkpoint':'models/macslam/MACT_Offroad.pth'}.items():
 path=ROOT/rel;assets[name]={'configured_path':rel,'exists':path.exists(),'bytes':path.stat().st_size if path.is_file() else None,'sha256':sha(path) if path.is_file() else None}
write(OUT/'local-inventory.json',{'scope':'configured source/asset paths and installed imports in the analytic runtime, not a global filesystem census','assets':assets,'runtime':runtime,'runtime_digest':p['runtime_for_analytic_tests'],'command':cmd,'exit_code':result.returncode,'stderr':result.stderr,'source_sha256':sha(__file__),'upstream_source_snapshots_sha256':sha(ROOT/'results/independent-benchmark-preparation/sources-001/upstream.json'),'payload_access':False})
print('INVENTORIED',len(assets),'configured assets; no dataset payload accessed')
