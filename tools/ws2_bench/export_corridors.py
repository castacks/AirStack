"""Export portable environment-only USD scenes without Isaac, ROS, or a target model."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import tempfile

from corridors import SHAPES, TIERS, tier_spec

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / 'scenarios/corridor_suite.yaml'


def export(output, shapes=SHAPES, tiers=TIERS, config=DEFAULT_CONFIG, seed=None):
    import yaml
    from pxr import Sdf, Usd, UsdGeom
    from corridor_scene import CorridorScene

    output = Path(output).absolute()
    if output.exists() or output.is_symlink():
        raise FileExistsError(f'Choose a new output directory: {output}')
    shapes, tiers = tuple(dict.fromkeys(shapes)), tuple(dict.fromkeys(tiers))
    if not shapes or not tiers or set(shapes) - set(SHAPES) or set(tiers) - set(TIERS):
        raise ValueError('Select at least one supported shape and difficulty')
    plan = yaml.safe_load(Path(config).read_text())
    seed = plan['layout_seed'] if seed is None else seed
    if type(seed) is not int or not 0 <= seed < 2**31:
        raise ValueError('Seed must be an integer in [0, 2**31)')
    light = float(plan['light'])
    if not math.isfinite(light) or light < 0:
        raise ValueError('Light intensity must be finite and nonnegative')
    output.parent.mkdir(parents=True, exist_ok=True)
    manifest = dict(schema_version=1, meters_per_unit=1.0, up_axis='Z',
                    layout_seed=seed, light=light, scenes=[])
    manifest['source_sha256'] = {
        name: hashlib.sha256((HERE / name).read_bytes()).hexdigest()
        for name in ('corridors.py', 'corridor_scene.py', 'export_corridors.py')}
    manifest['config_sha256'] = hashlib.sha256(Path(config).read_bytes()).hexdigest()

    with tempfile.TemporaryDirectory(prefix='.corridor-export-', dir=output.parent) as tmp:
        folder = Path(tmp)
        for shape in shapes:
            for tier in tiers:
                name = f'{shape}_{tier}'
                # Same map construction as corridor_suite.scenario(), without
                # importing the runner or its model/ROS/Docker dependencies.
                spec = tier_spec(shape, tier)
                spec.update(plan['map'])
                if shape == 'curve':
                    spec.pop('bend_radius', None)
                stage = Usd.Stage.CreateNew(str(folder / f'{name}.usda'))
                UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
                UsdGeom.SetStageMetersPerUnit(stage, 1.0)
                stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, '/World').GetPrim())
                scene = CorridorScene(stage, spec, seed, folder / 'textures')
                scene.illumination(light)
                # The live renderer writes absolute texture paths. Export each
                # as a layer-relative asset so moving the bundle remains valid.
                for prim in stage.Traverse():
                    for attr in prim.GetAttributes():
                        value = attr.Get()
                        if isinstance(value, Sdf.AssetPath) and value.path:
                            asset = Path(value.path).resolve(strict=True)
                            attr.Set(Sdf.AssetPath(asset.relative_to(folder).as_posix()))
                stage.GetRootLayer().Save()
                (folder / f'{name}.json').write_text(json.dumps(scene.data, indent=2) + '\n')
                manifest['scenes'].append(dict(
                    shape=shape, difficulty=tier, usd=f'{name}.usda',
                    geometry=f'{name}.json', placement_id=scene.data['placement_id'],
                    obstacles=len(scene.data['obstacles'])))
                del scene, stage

        (folder / 'README.txt').write_text(
            'Standalone WS2 corridor environments\n\n'
            'Keep this entire folder together, including textures/.\n'
            'In Isaac Sim, use File > Open and select a .usda file, or reference it\n'
            'inside your existing stage. No AirStack, ROS, PX4, model, LLM, or bench\n'
            'installation is needed to load these environment assets.\n\n'
            'Shapes: elbow (L), curve, zigzag, u_obstacles (U).\n'
            'Each file includes walls, floor, ceiling, obstacles, static collision\n'
            'shapes, materials, textures, and lights. It contains no robot, sensors,\n'
            'physics-scene settings, controllers, test runner, or attack injection.\n'
            'Add your own robot, physics scene, sensors, and task.\n\n'
            'Units: metres. Up axis: Z. Reference root: /World.\n'
            'The matching JSON contains exact geometry and the reference route.\n'
            'Its spawn Z is the original Iris setup; choose your own robot height.\n'
            'Difficulty names describe the layouts, not a guaranteed task difficulty.\n'
            'Use stage lighting to see the authored illumination. The Ceiling prim\n'
            'can be hidden for inspection; keep it enabled for matching rendering.\n\n'
            'bundle_manifest.json records scene identities and file checksums.\n'
            'Default exports match the fixed-suite map and seed in corridor_suite.yaml.\n'
            'Agent-generated layouts or other seeds require their own specifications.\n')
        manifest['files_sha256'] = {
            p.relative_to(folder).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(folder.rglob('*')) if p.is_file()}
        (folder / 'bundle_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        folder.rename(output)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New directory to share as a whole')
    parser.add_argument('--shapes', nargs='+', choices=SHAPES, default=list(SHAPES))
    parser.add_argument('--tiers', nargs='+', choices=TIERS, default=list(TIERS))
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--seed', type=int, default=None, help='Defaults to the suite layout_seed')
    args = parser.parse_args()
    try:
        result = export(args.output, args.shapes, args.tiers, args.config, args.seed)
    except ImportError as exc:
        parser.exit(1, f'Missing export dependency: {exc}\nInstall usd-core numpy Pillow PyYAML in a separate Python environment.\n')
    print(f"Exported {len(result['scenes'])} standalone scenes to {args.output.absolute()}")


if __name__ == '__main__':
    main()
