"""Alternate decoder verification of frozen source-depth support, not visibility."""
import json,hashlib,sys
from pathlib import Path
import numpy as np
from PIL import Image


def main(root):
    r=Path(root);base=r/'results/natural-flow-reference-intake';m=json.loads((base/'inputs/metadata.json').read_text());report=json.loads((base/'report.json').read_text());lookup={(x['condition'],x['stem']):x for x in report['rows']};checks=0
    for x in m['rows']:
        p=Path(x['depth_path']);assert hashlib.sha256(p.read_bytes()).hexdigest()==x['depth_sha256'];a=np.asarray(Image.open(p));assert a.shape==(256,640) and a.dtype==np.uint16
        row=lookup[x['condition'],x['stem']];assert int(np.sum(a>0))==row['positive_pixels'];assert float(np.sum(a>0)/a.size)==row['positive_fraction'];checks+=4
    for c,s in report['summary'].items():
        rows=[x for x in report['rows'] if x['condition']==c];assert len(rows)==s['frames']==20;assert sum(x['positive_pixels'] for x in rows)==s['positive_pixels_total'];checks+=2
    out={'status':'PASS','checks':checks,'scope':'Pillow vs OpenCV numeric depth counts/schema/hash; no endpoint geometry/visibility truth or model execution.'};(base/'independent-audit.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
if __name__=='__main__':main(sys.argv[1])
