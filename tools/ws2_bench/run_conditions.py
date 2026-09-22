"""Apply/verify Office conditions; --flight explicitly enables a PID hover demo."""
import argparse
import atexit
import datetime
import json
from pathlib import Path
import struct
import subprocess
import time
import urllib.request
import uuid
import random
import fcntl
import yaml
from conditions import validate,sensor_parameters

HERE=Path(__file__).resolve().parent
RUNTIME=HERE.parents[1]/"robot/ros_ws/ws2_runtime"


def scene_command(payload, timeout=30):
    identity=uuid.uuid4().hex
    payload=dict(payload,id=identity)
    temporary=RUNTIME/"scene_command.tmp"
    temporary.write_text(json.dumps(payload));temporary.replace(RUNTIME/"scene_command.json")
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        try:
            reply=json.loads((RUNTIME/"scene_reply.json").read_text())
            if reply.get("id")==identity:
                if not reply["ok"]:raise RuntimeError(reply["error"])
                return reply
        except (FileNotFoundError,json.JSONDecodeError):pass
        time.sleep(.1)
    raise TimeoutError("simulator did not acknowledge scene change")


def bridge_url():
    address=subprocess.check_output(["docker","inspect","airstack-robot-desktop-1","--format","{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}"],text=True).strip()
    return f"http://{address}:8765"


def get_frame(url):
    with urllib.request.urlopen(url+"/frame",timeout=3) as response: data=response.read()
    size=struct.unpack("!I",data[:4])[0]
    metadata=json.loads(data[4:4+size]);start=4+size
    return metadata,data[start:start+metadata["jpeg_bytes"]]


def apply(c,url):
    scene_command({"condition":c})
    # Wait for scene changes to reach the renderer before accepting camera frames.
    deadline=time.monotonic()+20
    settled_at=None
    while time.monotonic()<deadline:
        state=json.loads((RUNTIME/"scene_status.json").read_text())
        if state.get("condition")==c:
            if settled_at is None:settled_at=state["sim_time"]
            if state["sim_time"]-settled_at>=.5:break
        time.sleep(.05)
    else:raise TimeoutError("scene did not settle")
    subprocess.run(["docker","cp",str(HERE/"ros_control.py"),"airstack-robot-desktop-1:/tmp/ws2_ros_control.py"],check=True,capture_output=True)
    subprocess.run(["docker","exec","airstack-robot-desktop-1","bash","-lc",
                    'sws && python3 /tmp/ws2_ros_control.py "$1"',"ws2",json.dumps(sensor_parameters(c))],check=True,capture_output=True,text=True)
    deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        try:
            metadata,jpeg=get_frame(url)
            d=metadata["sensor_disturbance"]
            if all(d[k]==v for k,v in {"seed":c["seed"],"rgb_noise_stddev":c["rgb_noise"],"depth_noise_stddev_m":c["depth_noise"],"fixed_delay_s":c["delay"]}.items()):
                return metadata,jpeg
        except (OSError,KeyError):pass
        time.sleep(.15)
    raise TimeoutError("camera did not produce a sample under the requested condition")


def main():
    lock=(RUNTIME/'sequence.lock').open('w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise RuntimeError('another condition sequence is already running')
    if (RUNTIME/'flight_guard.json').exists():raise RuntimeError('review unresolved physics flight guard before running')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path,default=HERE/"demo.yaml")
    parser.add_argument("--one",help="single condition JSON instead of sequence")
    parser.add_argument("--dwell",type=float,help="simulation seconds per condition")
    parser.add_argument("--camera",choices=["overview","follow","fixed"])
    parser.add_argument("--random-count",type=int,default=0)
    parser.add_argument("--seed",type=int,default=42)
    parser.add_argument("--flight",action="store_true",help="default demo: take off after layout showcase, vary sensor/patch in hover, land")
    args=parser.parse_args()
    if args.flight and (args.one or args.random_count or args.config != HERE/'demo.yaml'):
        parser.error('--flight requires the default demonstration sequence')
    cfg=yaml.safe_load(args.config.read_text())
    conditions=[validate(json.loads(args.one))] if args.one else [validate(c) for c in cfg["conditions"]]
    if args.random_count:
        if not 1<=args.random_count<=100:raise ValueError("random count must be 1..100")
        rng=random.Random(args.seed)
        conditions=[validate(dict(name=f"Random condition {i+1}",layout=rng.choice(["furnished_a","furnished_b"]),layout_seed=rng.randrange(8),
                    seed=rng.randrange(2**31),light=rng.uniform(800,2400),rgb_noise=rng.uniform(0,30),
                    delay=rng.uniform(0,.3),patch_enabled=rng.choice([True,False]),
                    patch_strength=rng.uniform(.3,1),patch_size=rng.uniform(.5,.9),patch_height=rng.uniform(.9,1.5)))
                    for i in range(args.random_count)]
    dwell=args.dwell if args.dwell is not None else cfg.get("dwell_sim_s",8)
    if dwell<0:raise ValueError("dwell must be nonnegative")
    url=bridge_url()
    if args.camera:scene_command({"camera":args.camera})
    output=RUNTIME/"runs"/datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output.mkdir(parents=True,exist_ok=False)
    flight=None
    for i,c in enumerate(conditions):
        metadata,jpeg=apply(c,url)
        if args.flight and i==3:
            for n in ['hover_ready.json','land_request','flight_check_result.json']:
                (RUNTIME/n).unlink(missing_ok=True)
            subprocess.run(['docker','cp',str(HERE/'gt_hover.py'),'airstack-robot-desktop-1:/tmp/ws2_gt_hover.py'],check=True,capture_output=True)
            flight_log=(output/'flight.log').open('w')
            flight=subprocess.Popen(['docker','exec','airstack-robot-desktop-1','bash','-lc',
                'sws && python3 -u /tmp/ws2_gt_hover.py --height 1.2 --hold 180 --wait-for-land'],stdout=flight_log,stderr=subprocess.STDOUT)
            atexit.register(lambda: (RUNTIME/'land_request').touch())
            deadline=time.monotonic()+120
            while not (RUNTIME/'hover_ready.json').exists():
                if flight.poll() is not None:raise RuntimeError('takeoff failed; inspect flight.log')
                if time.monotonic()>deadline:
                    (RUNTIME/'land_request').touch();raise TimeoutError('takeoff timeout')
                time.sleep(.2)
            metadata,jpeg=get_frame(url)
        state=json.loads((RUNTIME/"scene_status.json").read_text())
        with urllib.request.urlopen(url+"/health",timeout=3) as response:health=json.load(response)
        evidence={"condition":c,"camera":metadata,"scene":state,"health":health,
                  "observed_image_age_s":health.get("odometry_stamp",0)-metadata["stamp"]}
        (output/f"{i:02d}.json").write_text(json.dumps(evidence,indent=2))
        (output/f"{i:02d}_input.jpg").write_bytes(jpeg)
        scene_command({"capture":f"condition_{i:02d}.png"})
        print(json.dumps({"index":i,"name":c["name"],"rgb_rmse":metadata["rgb_noise_rmse"],"delay":c["delay"],"output":str(output)}),flush=True)
        start=float(state["sim_time"]);wall_deadline=time.monotonic()+max(60,dwell*10)
        while time.monotonic()<wall_deadline:
            now=json.loads((RUNTIME/"scene_status.json").read_text())["sim_time"]
            if now-start>=dwell:break
            time.sleep(.2)
        else:raise TimeoutError("simulation clock stalled")
    if flight is not None:
        (RUNTIME/'land_request').touch()
        if flight.wait(timeout=120):raise RuntimeError('flight or landing failed')
        flight_log.close()
        for name in ['flight_check_result.json','flight_trace.json','hover_ready.json']:
            (output/name).write_bytes((RUNTIME/name).read_bytes())
    print("Condition sequence complete; video recording is handled by the user.",flush=True)


if __name__=="__main__":main()
