"""Fixed contiguous block displacement diagnostics; different origins, no GT claim."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from .td3_reference import matrices


def run(root):
    output=root/'results/gnss-block-localization'
    protocol=json.loads((output/'protocol.json').read_text())
    source=root/'results/gnss-and-tracking/gnss-002'
    plan=json.loads((source/'plan.json').read_text())
    hashes={};rows=[];arrays={}
    def freeze(path):
        hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    freeze(source/'plan.json');freeze(output/'protocol.json');freeze(Path(__file__))
    for name in protocol['cases']:
        case=next(c for c in plan['cases'] if c['case']==name)
        trajectory=Path(case['directory'])/'trajectory.npz';freeze(trajectory)
        with np.load(trajectory) as data:
            times=data['time_ns'].reshape(-1).copy()
            vo=matrices(data['poses_camera_ned'])[:,:3,3]
        boundaries=protocol['boundaries']
        assert len(times)==100 and np.all(np.diff(times)>0)
        assert boundaries[0]==0 and boundaries[-1]==len(times)-1
        assert [i for a,b in zip(boundaries,boundaries[1:]) for i in range(a,b)]==list(range(99))
        for offset in protocol['offsets_ns']:
            path=source/'points'/name/f'offset-{offset}.npz';freeze(path)
            with np.load(path) as data:
                assert np.array_equal(data['time_ns'],times)
                gps=data['gnss_local_positions'].copy()
                assert abs(np.linalg.norm(vo[-1]-vo[0])-data['vo_xyz'][-1])<1e-10
                assert abs(np.linalg.norm(gps[-1]-gps[0])-data['gnss_xyz'][-1])<1e-10
            arrays[f'{name}_{offset}_gnss']=gps
            arrays[f'{name}_{offset}_vo']=vo
            for block,(a,b) in enumerate(zip(boundaries,boundaries[1:])):
                gaps=np.diff(times[a:b+1])/1e9
                gnss=float(np.linalg.norm(gps[b]-gps[a]));estimate=float(np.linalg.norm(vo[b]-vo[a]))
                rows.append(dict(case=name,offset_ns=offset,block=block,start_frame=a,end_frame=b,
                    duration_s=float((times[b]-times[a])/1e9),gaps_gt_150ms=int((gaps>.15).sum()),
                    max_gap_s=float(gaps.max()),gnss_net_3d_m=gnss,vo_net_3d_m=estimate,
                    vo_minus_gnss_net_m=estimate-gnss))
    with (output/'blocks.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    np.savez_compressed(output/'positions.npz',**arrays)
    (output/'analysis.json').write_text(json.dumps(dict(protocol=protocol,rows=rows,files_sha256=hashes),indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(10,9),layout='constrained')
    for ax,name in zip(axes,protocol['cases']):
        selected=[r for r in rows if r['case']==name and r['offset_ns']==0]
        x=np.arange(5)
        ax.bar(x-.18,[r['gnss_net_3d_m'] for r in selected],width=.36,label='GNSS gps_frame')
        ax.bar(x+.18,[r['vo_net_3d_m'] for r in selected],width=.36,label='Thermal camera VO')
        for i,r in enumerate(selected):
            alternatives=[s['gnss_net_3d_m'] for s in rows if s['case']==name and s['block']==i]
            ax.plot([i-.18]*2,[min(alternatives),max(alternatives)],color='black',linewidth=2)
        ax.set(title=name,ylabel='Block net displacement (m)',xticks=x,
            xticklabels=[f"{r['start_frame']}–{r['end_frame']}\n{r['duration_s']:.2f}s; {r['gaps_gt_150ms']} gaps" for r in selected])
        ax.legend()
    fig.suptitle('Fixed blocks; gaps >150ms. Black: ±0.1s clock sensitivity, not confidence.\nDifferent physical origins; no axis/scale fit; not camera position error.',fontsize=11)
    fig.savefig(output/'block-displacements.png',dpi=160)
    print(json.dumps([r for r in rows if r['offset_ns']==0],indent=2))


if __name__=='__main__':
    run(Path('/workspace'))
