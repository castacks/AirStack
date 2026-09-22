"""Generate the validated layout catalogue from the collected Office USD."""
import argparse,json,tempfile
from pathlib import Path
from office_variants import build_variant

p=argparse.ArgumentParser();p.add_argument('office_usd',type=Path)
p.add_argument('--output',type=Path,default=Path(__file__).with_name('layouts.json'))
a=p.parse_args();catalog={}
for name in ['furnished_a','furnished_b']:
    entries=[]
    for seed in range(8):
        ops=[{'action':'move','source':'/Root/SM_Plant7_463','delta_range_m':[[-3,-3,0],[-1,0,0]]},
             {'action':'duplicate','source':'/Root/SM_Plant8','delta_range_m':[[-6,1,0],[-2,4,0]]},
             {'action':'duplicate','source':'/Root/SM_ColumnA13','delta_range_m':[[-5,-23,0],[0,-20,0]]}]
        if name=='furnished_b':ops.append({'action':'duplicate','source':'/Root/SM_Plant8','delta_range_m':[[-6,9,0],[-2,12,0]]})
        cfg={'seed':seed,'clearance_m':.1,'protected_regions_m':[[[-.8,-4.8,-.1],[.8,-3.2,2.2]]],'operations':ops}
        with tempfile.TemporaryDirectory() as directory:
            manifest=build_variant(a.office_usd,cfg,Path(directory)/'scene.usda')
        entry={k:c['delta_m'] for k,c in zip(['move','plant','column','plant2'],manifest['changes'])}
        entry.update(validation=manifest['validation'],seed=seed,source_sha256=manifest['source_sha256'])
        entries.append(entry)
    catalog[name]=entries
a.output.write_text(json.dumps(catalog,indent=2)+'\n')
print(f'Wrote {sum(map(len,catalog.values()))} validated layouts to {a.output}')
