"""Export small Office collision-layout bounds with the installed USD Python API."""
import argparse, hashlib, json
from pathlib import Path
from pxr import Usd, UsdGeom
from prepare_difficulty import obstacle_bounds

def export(source, output):
    occupied, floors = obstacle_bounds(source)
    # Placement is restricted to this source-frame ROI; distant Office props
    # cannot intersect it. Keep boundary-crossing structures and support slabs.
    roi=[[-5.,-5.,-.1],[5.,8.,6.]]
    intersects=lambda b: all(b[0][i]<roi[1][i] and roi[0][i]<b[1][i] for i in range(3))
    occupied={k:b for k,b in occupied.items() if intersects(b)}
    floors=[b for b in floors if intersects(b)]
    stage = Usd.Stage.Open(str(source))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render'])
    sources = {}
    for name, path in {'move':'/Root/SM_Plant7_463', 'plant':'/Root/SM_Plant8',
                       'column':'/Root/SM_ColumnA13'}.items():
        box = cache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
        sources[name] = {'path':path,'bounds':[list(box.GetMin()),list(box.GetMax())]}
    data = {'schema':1,'source_sha256':hashlib.sha256(Path(source).read_bytes()).hexdigest(),
            'coordinate_frame':'Office source Z-up metres; world=(source_y,-source_x,source_z)',
            'placement_roi_source_m':roi,'occupied':occupied,'floors':floors,'sources':sources}
    Path(output).write_text(json.dumps(data, indent=2)+'\n')
    print('Exported',len(occupied),'obstacle bounds and',len(floors),'floor bounds')

if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('output');a=p.parse_args()
    export(a.source,a.output)
