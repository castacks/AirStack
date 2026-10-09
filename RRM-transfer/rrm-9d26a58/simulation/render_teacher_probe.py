#!/usr/bin/env python3
"""Bounded isolated renderer acquisition; no ROS, PX4, drone or live-stage access.

Run via Isaac python.sh with --source-root (RRM) and a NEW --output directory.
The static two-marker fixture is not the Office/drone camera or seven-case campaign.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import sys
import uuid
import zlib


def png_rgb(array):
    import numpy as np
    pixels = np.asarray(array)
    if pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] not in (3, 4):
        raise ValueError("invalid RGB renderer payload")
    height, width = pixels.shape[:2]
    def chunk(kind, value):
        return struct.pack(">I", len(value)) + kind + value + struct.pack(">I", zlib.crc32(kind + value))
    raster = b"".join(b"\0" + row[:, :3].tobytes() for row in pixels)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raster)) + chunk(b"IEND", b""))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--absent-entity", choices=("blue_marker", "orange_marker"),
                        help="Omit this prim in the isolated fixture; never modify a live stage")
    parser.add_argument("--color-profile", choices=("legacy", "emissive-v1"), default="legacy")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(args.source_root))
    from rrm.render_teacher import build_render_pair, qualify_probe_colors
    from rrm.visual_evaluation import canonical_sha256
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "active_gpu": 0, "multi_gpu": False,
                         "renderer": "RaytracedLighting"})
    try:
        import numpy as np
        import omni.usd
        import omni.replicator.core as rep
        from pxr import Gf, Sdf, UsdGeom, UsdShade
        from isaacsim.core.utils.semantics import add_update_semantics, get_semantics

        stage = omni.usd.get_context().get_stage()
        manifest = json.loads((args.source_root / 'examples/office_visual_eval/scene_manifest.json').read_text())
        manifest.update(scene_id="rrm-isolated-marker-render-probe-v1",
                        scene_shortname="isolated-render-probe", launch_script=Path(__file__).name,
                        teacher_source_ref="rrm-isolated-marker-render-probe-v1",
                        camera_topic="isaac-render://isolated-marker-probe/rgb", camera_frame_id="probe_camera")
        for record in manifest['markers'].values():
            record.pop('map_waypoint', None)
        if args.absent_entity:
            manifest['scene_id'] += '-' + args.absent_entity + '-absent'
            manifest['probe_variant'] = {'omitted_prim_entity': args.absent_entity}
        if args.color_profile != 'legacy':
            manifest['scene_id'] += '-' + args.color_profile
            manifest.setdefault('probe_variant', {})['color_profile'] = args.color_profile
        episode = uuid.uuid4().hex
        producer = "isaac-render-probe/v1:sha256:" + hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        launcher_hash = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        for index, (entity, record) in enumerate(manifest['markers'].items()):
            if entity == args.absent_entity:
                continue
            marker = UsdGeom.Cube.Define(stage, record['scene_prim'])
            marker.CreateSizeAttr(0.8)
            marker.AddTranslateOp().Set(Gf.Vec3d(4, index * 1.5 - 0.75, 1))
            marker.CreateDisplayColorAttr([Gf.Vec3f(*( (0.05, 0.2, 1) if index == 0 else (1, 0.22, 0.03) ))])
            if args.color_profile == 'emissive-v1':
                material = UsdShade.Material.Define(stage, '/World/ProbeMaterials/' + entity)
                shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild('Surface'))
                shader.CreateIdAttr('UsdPreviewSurface')
                shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0))
                shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(1.)
                shader.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).Set(
                    Gf.Vec3f(*((0, .02, .8) if entity == 'blue_marker' else (.8, .03, 0))))
                shader.CreateOutput('surface', Sdf.ValueTypeNames.Token)
                material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
                if not UsdShade.MaterialBindingAPI.Apply(marker.GetPrim()).Bind(material):
                    raise ValueError('probe material binding failed')
            marker.GetPrim().CreateAttribute('rrm:entity_id', Sdf.ValueTypeNames.String).Set(entity)
            marker.GetPrim().CreateAttribute('rrm:kind', Sdf.ValueTypeNames.String).Set(record['kind'])
            add_update_semantics(marker.GetPrim(), entity)
        camera = rep.create.camera(position=(0, 0, 1), look_at=(4, 0, 1))
        rep.create.light(light_type="dome", intensity=1500)
        product = rep.create.render_product(camera, (480, 300))
        reference_semantics = get_semantics

        class PairedWriter(rep.Writer):
            def __init__(self):
                self.annotators = [rep.AnnotatorRegistry.get_annotator('rgb'),
                    rep.AnnotatorRegistry.get_annotator('semantic_segmentation',
                        init_params={'colorize': False, 'semanticTypes': ['class']}),
                    rep.AnnotatorRegistry.get_annotator('ReferenceTime')]
                self.count = 0
                self.error = None
                self.color_report = None

            def write(self, data):
                # Only accept one paired callback. No sampling latest get_data()
                # or current World/ROS clock, which may name a different frame.
                if self.count or self.error:
                    return
                try:
                    segmentation = data['semantic_segmentation']
                    mask = np.asarray(segmentation['data']).squeeze()
                    if mask.ndim != 2 or mask.dtype != np.uint32:
                        raise ValueError('invalid native segmentation shape/type')
                    reference = data['ReferenceTime']
                    entities = []
                    for entity, record in manifest['markers'].items():
                        prim = stage.GetPrimAtPath(record['scene_prim'])
                        exists = bool(prim.IsValid() and prim.IsActive())
                        if exists:
                            if (prim.GetAttribute('rrm:entity_id').Get() != entity
                                    or prim.GetAttribute('rrm:kind').Get() != record['kind']
                                    or ('class', entity) not in reference_semantics(prim).values()):
                                raise ValueError('actual prim/semantic identity mismatch')
                        entities.append(dict(entity_id=entity, scene_prim=record['scene_prim'],
                                             kind=record['kind'], exists=exists))
                    image = png_rgb(data['rgb'])
                    pair = build_render_pair(image=image, semantic_pixels=mask.tolist(),
                        id_to_labels={str(key): value for key, value in segmentation['info']['idToLabels'].items()},
                        stage_entities=entities, manifest=manifest, launcher_sha256=launcher_hash,
                        producer_ref=producer, episode_id=episode, scene_revision=canonical_sha256(entities),
                        reference_numerator=int(reference['referenceTimeNumerator']),
                        reference_denominator=int(reference['referenceTimeDenominator']))
                    (args.output / 'image.png').write_bytes(image)
                    for name in ('frame', 'registration', 'teacher', 'pairing_report', 'render_reference'):
                        (args.output / (name + '.json')).write_text(json.dumps(pair[name], indent=2, sort_keys=True) + '\n')
                    np.save(args.output / 'semantic-pixels.npy', mask, allow_pickle=False)
                    (args.output / 'semantic-labels.json').write_text(json.dumps(segmentation['info']['idToLabels'], sort_keys=True))
                    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True))
                    if args.color_profile == 'emissive-v1':
                        rgb = np.asarray(data['rgb'])[:, :, :3]
                        samples = {}
                        for raw_id, label in segmentation['info']['idToLabels'].items():
                            entity = label.get('class')
                            if entity in manifest['markers'] and np.any(mask == int(raw_id)):
                                samples[entity] = rgb[mask == int(raw_id)].tolist()
                        self.color_report = qualify_probe_colors(samples)
                        (args.output / 'color_qualification.json').write_text(json.dumps(self.color_report, indent=2, sort_keys=True))
                    self.count += 1
                except Exception as error:
                    self.error = str(error)

        writer = PairedWriter()
        writer.attach(product)
        for _ in range(3):
            rep.orchestrator.step(rt_subframes=4, pause_timeline=True)
            if writer.count or writer.error:
                break
        rep.BackendDispatch.wait_until_done()
        report = dict(schema_version='rrm-render-teacher-probe/v1',
                      status='ACQUIRED' if writer.count else 'INCOMPLETE',
                      paired_callbacks=writer.count, error=writer.error,
                      fixture_id=manifest['scene_id'], episode_id=episode,
                      launcher_sha256=launcher_hash,
                      converter_sha256=hashlib.sha256((args.source_root / 'rrm/render_teacher.py').read_bytes()).hexdigest(),
                      execution_dispatch=False, ros_connected=False, active_stage_modified=False,
                      office_sensor_frame=False, numpy_version=np.__version__)
        report['omitted_prim_entity'] = args.absent_entity
        report['color_profile'] = args.color_profile
        report['color_qualification'] = writer.color_report
        (args.output / 'report.json').write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
        print(json.dumps(report), flush=True)
        writer.detach()
        return 0 if writer.count and (writer.color_report is None or writer.color_report['passed']) else 2
    finally:
        app.close()


if __name__ == '__main__':
    raise SystemExit(main())
