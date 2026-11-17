"""Private static fixture renderer: no robot, ROS, Pegasus or flight setup."""
import hashlib
import json
import sys
import time
from pathlib import Path

root = Path('/tmp/rrm-private-render-20261117/capture-03')
root.mkdir(parents=True, exist_ok=False)
phases = (root/'phases.jsonl').open('x', buffering=1)
def phase(name, **fields):
    phases.write(json.dumps(dict(phase=name, monotonic_s=time.monotonic(), **fields))+'\n')
phase('before_app')
from isaacsim import SimulationApp
app = SimulationApp(dict(headless=True, width=480, height=300, renderer='RaytracedLighting',
    active_gpu=0, multi_gpu=False, enable_crashreporter=False,
    extra_args=['--/app/settings/persistent=false', '--/app/livestream/enabled=false',
                '--/app/userConfigPath=/tmp/rrm-private-render-20261117/render.config.json']))
try:
    import traceback
    import numpy as np
    from PIL import Image
    import omni.usd
    import omni.replicator.core as rep
    from pxr import Gf, Usd, UsdGeom, UsdLux
    from uav_marker_fixtures import SCENARIOS, REFERENCE_CAMERA_M, fixture_spec, author_fixtures
    phase('app_started', usd_version=list(Usd.GetVersion()))
    records = []
    for name in SCENARIOS:
        phase('case_start', case=name)
        output = root/name
        output.mkdir()
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        spec = fixture_spec(name)
        author_fixtures(stage, spec)
        light = UsdLux.DomeLight.Define(stage, '/World/PrivateLight')
        light.CreateIntensityAttr(1000.)
        camera = UsdGeom.Camera.Define(stage, '/World/PrivateCamera')
        camera.CreateFocalLengthAttr(18.4)
        camera.CreateHorizontalApertureAttr(48.)
        camera.CreateVerticalApertureAttr(30.)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.05, 20.))
        position = Gf.Vec3d(*REFERENCE_CAMERA_M)
        matrix = Gf.Matrix4d().SetLookAt(position, position+Gf.Vec3d(1., 0., 0.), Gf.Vec3d(0., 0., 1.)).GetInverse()
        camera.AddTransformOp().Set(matrix)
        product = rep.create.render_product(str(camera.GetPath()), (480, 300))
        annotators = {key:rep.AnnotatorRegistry.get_annotator(key) for key in
                      ('rgb', 'distance_to_image_plane', 'camera_params')}
        for annotator in annotators.values():
            annotator.attach([product.path])
        try:
            for _ in range(15):
                app.update()
            rep.orchestrator.step(rt_subframes=8)
            metadata = dict(case=name, scope='private static camera, not robot camera',
                resolution=[480, 300], stage_metres_per_unit=UsdGeom.GetStageMetersPerUnit(stage),
                camera_position_world_m=list(position), camera_transform=list(matrix),
                focal_length=float(camera.GetFocalLengthAttr().Get()),
                horizontal_aperture=float(camera.GetHorizontalApertureAttr().Get()),
                vertical_aperture=float(camera.GetVerticalApertureAttr().Get()), frames=[])
            fx = metadata['focal_length']/metadata['horizontal_aperture']*480
            fy = metadata['focal_length']/metadata['vertical_aperture']*300
            intrinsic = [fx, 0., 240., 0., fy, 150., 0., 0., 1.]
            for index in range(3):
                rep.orchestrator.step(rt_subframes=8)
                data = {key:value.get_data() for key,value in annotators.items()}
                rgb = np.asarray(data['rgb'])
                depth = data['distance_to_image_plane']
                if isinstance(depth, dict):
                    depth = depth['data']
                depth = np.asarray(depth)
                if depth.ndim == 3 and depth.shape[2] == 1:
                    depth = depth[:, :, 0]
                if rgb.shape[:2] != (300, 480) or rgb.shape[2] not in (3, 4) or depth.shape != (300, 480):
                    raise ValueError('unexpected renderer output shape')
                if rgb.dtype != np.uint8 or not np.issubdtype(depth.dtype, np.floating):
                    raise ValueError('unexpected renderer output dtype')
                prefix = 'frame-'+str(index)
                Image.fromarray(rgb[:, :, :3]).save(output/(prefix+'.png'))
                np.savez_compressed(output/(prefix+'.npz'), depth_m=depth, intrinsic=np.array(intrinsic))
                params = data['camera_params']
                (output/(prefix+'-camera.json')).write_text(json.dumps(params, default=lambda value:value.tolist() if hasattr(value,'tolist') else str(value), indent=2))
                metadata['frames'].append(dict(prefix=prefix, wall_unix=time.time(),
                    input_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
                                  (output/(prefix+'.png'), output/(prefix+'.npz'), output/(prefix+'-camera.json'))}))
            stage.GetRootLayer().Export(str(output/'stage.usda'))
            (output/'metadata.json').write_text(json.dumps(metadata, default=lambda value:list(value), indent=2))
            (output/'authored-spec.json').write_text(json.dumps(spec, indent=2))
            records.append(dict(case=name, captured_frames=3))
            phase('case_captured', case=name)
        finally:
            for annotator in annotators.values():
                annotator.detach([product.path])
            product.destroy()
    (root/'result.json').write_text(json.dumps(dict(capture_complete=True, cases=records,
        renderer='RaytracedLighting', robot_or_ros_started=False,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()), indent=2))
    phase('capture_complete')
except BaseException as error:
    import traceback
    (root/'error.json').write_text(json.dumps(dict(type=type(error).__name__, message=str(error), traceback=traceback.format_exc()), indent=2))
    phase('capture_failed', error_type=type(error).__name__)
finally:
    phases.flush()
    app.close()
