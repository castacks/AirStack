"""Read-only pairing/sensor/mission audit and optional trajectory plots."""
import argparse,json,math
from pathlib import Path
from audit_results import audit_episode
from episode import fingerprint

def read(p):return json.loads(Path(p).read_text())

def audit(root):
    history=read(root/'history.json');trials={};pairs=[]
    for entry in history:
        pair=entry['pairs'][0];folders=[Path(pair[role]['result_dir']) for role in ('clean','perturbed')]
        configs=[read(f/'scenario.json') for f in folders]
        evidence=[read(f/'condition_evidence.json') for f in folders];errors=[]
        for folder,c,e in zip(folders,configs,evidence):
            trials[str(folder)]=audit_episode(folder)
            if fingerprint(c)!=read(folder/'result.json')['configuration_hash']:errors.append('Result/config hash mismatch')
            if c['condition']['patch_enabled'] or e['scene']['condition']['patch_enabled']:errors.append('Patch was enabled')
            d=e['frame']['sensor_disturbance']
            for ck,dk in [('rgb_noise','rgb_noise_stddev'),('delay','fixed_delay_s'),('depth_noise','depth_noise_stddev_m')]:
                if c['condition'][ck]!=d[dk]:errors.append('Requested/observed sensor mismatch '+ck)
            rmse=e['frame']['rgb_noise_rmse']
            if (c['condition']['rgb_noise']==0 and rmse!=0) or (c['condition']['rgb_noise']>0 and rmse<=0):errors.append('Unexpected measured RGB noise RMSE')
        for key in configs[0]:
            if key not in ('name','condition') and configs[0][key]!=configs[1][key]:errors.append('Different mission '+key)
        for key in configs[0]['condition']:
            if key not in ('name','rgb_noise','delay') and configs[0]['condition'][key]!=configs[1]['condition'][key]:errors.append('Different condition '+key)
        if evidence[0]['scene']['realized_layout']!=evidence[1]['scene']['realized_layout']:errors.append('Different realized geometry')
        if math.dist(*(e['scene']['position'] for e in evidence))>.01:errors.append('Initial position differs by >1 cm')
        if configs[0]['condition']['rgb_noise'] or configs[0]['condition']['delay']:errors.append('Clean has sensor corruption')
        if sum(configs[1]['condition'][k]>0 for k in ('rgb_noise','delay'))!=1:errors.append('Expected exactly one attack factor')
        pairs.append({'round':entry['decision']['round'],'passed':not errors,'errors':errors,
            'rgb_rmse_clean_attack':[e['frame']['rgb_noise_rmse'] for e in evidence]})
    return {'passed':all(t['passed'] for t in [*trials.values(),*pairs]),'trials':list(trials.values()),'pairs':pairs}

def plot(root):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from generated_layouts import geometry
    report=read(root/'vulnerability_report.json');pairs=report['paired_comparisons']
    paths={r['directory']:read(Path(r['directory'])/'samples.json') for pair in pairs for r in (pair['clean'],pair['perturbed'])}
    points=[s['position_m'] for path in paths.values() for s in path]
    xlim=(min(-4.,*(p[0] for p in points))-1,max(4.,*(p[0] for p in points))+1)
    ylim=(min(-3.,*(p[1] for p in points))-1,max(2.,*(p[1] for p in points))+1)
    fig,axes=plt.subplots(1,len(pairs),figsize=(7*len(pairs),6),squeeze=False)
    for ax,pair in zip(axes[0],pairs):
        cond=pair['condition'];boxes=[b for path,b in geometry()['occupied'].items() if path!=geometry()['sources']['move']['path']]
        for box in boxes:
            if box[0][2]>.85 or box[1][2]<.85:continue
            ax.add_patch(Rectangle((box[0][1],-box[1][0]),box[1][1]-box[0][1],box[1][0]-box[0][0],color='#ccd2d7',alpha=.65))
        for kind,box in cond['placement']['bounds_source_m'].items():
            ax.add_patch(Rectangle((box[0][1],-box[1][0]),box[1][1]-box[0][1],box[1][0]-box[0][0],
                color='#507963' if 'plant' in kind or kind=='move' else '#c68948',alpha=.8))
        for role,color in [('clean','#0072b2'),('perturbed','#d55e00')]:
            r=pair[role];samples=paths[r['directory']]
            ax.plot([s['position_m'][0] for s in samples],[s['position_m'][1] for s in samples],
                    color=color,linestyle='--' if role=='clean' else '-',linewidth=2,
                    label=f"{role}: {r['outcome']}")
        ax.scatter([-4],[0],c='black',marker='o',s=25,label='Start')
        if report['target']=='mononav':ax.scatter([4],[0],c='black',marker='*',s=100,label='Goal')
        ax.set_title(f"Noise sigma {cond['rgb_noise']:g}, delay {cond['delay']:g} s\nPatch OFF; one matched pair")
        ax.set_xlabel('World x (m)');ax.set_ylabel('World y (m)');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8)
        ax.set_xlim(*xlim);ax.set_ylim(*ylim)
    fig.suptitle(report['target']+' — observed trajectories; selected obstacle bounds (not a complete scene map)')
    fig.tight_layout();fig.savefig(root/'trajectories.png',dpi=160);fig.savefig(root/'trajectories.pdf');plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--plot',action='store_true');a=p.parse_args()
    result=audit(a.root);(a.root/'artifact_audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    if a.plot:plot(a.root)
    if not result['passed']:raise SystemExit(1)
