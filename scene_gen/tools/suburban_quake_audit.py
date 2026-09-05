"""Cold-read regression gates for the assembled suburban earthquake review."""
import argparse
import json
import math
from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics
from disaster.quake_suburban_bake import validate_settled_export
from disaster.quake_suburban_interactions import mesh_points
from tools.suburban_quake_build import house_keepout, tree_fits


def audit(directory):
    directory = Path(directory)
    report = json.loads((directory / 'scene_report.json').read_text())
    stage = Usd.Stage.Open(str(directory / 'review_scene.usda'))
    houses = report['houses']
    assert len(houses) == 60 and len(report['people']) == 12
    assert tuple(report['region']) == (-125, -125, 125, 125)
    dome = UsdLux.DomeLight.Get(stage, '/World/DomeLight')
    assert dome.GetExposureAttr().Get() == 0
    assert dome.GetTextureFileAttr().Get().path.endswith(
        '/NVIDIA/Assets/Skies/2022_1/Skies/Clear/noon_grass.hdr')
    active = [str(p.GetPath()) for p in stage.Traverse()
              if p.HasAPI(UsdPhysics.RigidBodyAPI)
              and UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Get()]
    assert not active, active[:5]
    tilts = {}
    for h in houses:
        if h['mode'] != 'foundation':
            continue
        structure = stage.GetPrimAtPath(h['prim'] + '/house/structure')
        assert structure.IsA(UsdGeom.Xform), str(structure.GetPath())
        child = next(p for p in Usd.PrimRange(structure, Usd.TraverseInstanceProxies())
                     if p.IsA(UsdGeom.Mesh))
        up = UsdGeom.XformCache().GetLocalToWorldTransform(child).TransformDir(
            Gf.Vec3d(0, 0, 1)).GetNormalized()
        angle = math.degrees(math.acos(max(-1., min(1., up[2]))))
        assert angle > 2., (h['id'], angle)
        tilts[h['id']] = angle
    floor = stage.GetPrimAtPath('/World/stage/generated/sqh_006/house/structure/house_floor_0_13')
    assert floor and not floor.IsActive()
    keepout = house_keepout(houses, stage=stage)
    bad_trees = [t['prim'] for t in report['trees']
                 if not tree_fits(stage, stage.GetPrimAtPath(t['prim']), keepout)]
    assert not bad_trees, bad_trees[:5]
    checked = set()
    bodies = 0
    for h in houses:
        cache = h['cache']
        if cache in checked:
            continue
        checked.add(cache)
        meta = json.loads(Path(cache).with_suffix('.json').read_text())
        assert meta['physics_ready']
        if not meta['loose_paths']:
            continue
        baked = Usd.Stage.Open(cache)
        minimum_z = min(p[2] for path in meta['loose_paths']
                        for p in mesh_points(baked.GetPrimAtPath(path)))
        validate_settled_export(meta['physics_bake'], minimum_z)
        bodies += len(meta['loose_paths'])
    result = dict(houses=len(houses), people=len(report['people']),
                  trees=len(report['trees']), active_bodies=len(active),
                  caches_checked=len(checked), cached_fragments_checked=bodies,
                  foundation_tilts_deg=tilts, unsupported_floor_inactive=True,
                  tree_clearance_violations=bad_trees, sky_exposure=0)
    print(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    audit(parser.parse_args().directory)
