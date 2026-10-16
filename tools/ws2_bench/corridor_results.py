"""Summarize saved corridor flights and plot measured paths, without Isaac."""
import argparse
import csv
import json
from pathlib import Path


def summarize(root):
    root=Path(root);rows=[];groups={}
    for file in sorted(root.rglob('result.json')):
        folder=file.parent
        if not (folder/'corridor_geometry.json').exists():continue
        result=json.loads(file.read_text());config=json.loads((folder/'scenario.json').read_text())
        geometry=json.loads((folder/'corridor_geometry.json').read_text())
        metrics=result['metrics'];c=config['condition']
        active=[k for k in ('rgb_noise','delay','patch_enabled') if c[k]]
        row={'run':str(folder.relative_to(root)),'planner':config['planner'],'shape':c['corridor']['shape'],
             'style':c['corridor'].get('style','hallway'),
             'variant':config.get('pulp_variant',config.get('kim_policy','')),
             'input_fov_deg':config.get('input_fov_deg'),
             'camera_pitch_deg':config.get('camera_pitch_deg',0.),
             'trajectory_horizon_s':config.get('trajectory_horizon'),
             'yaw_reference_mode':config.get('yaw_reference_mode'),
             'yaw_gain_deg_s':config.get('yaw_gain_deg_s'),
             'configured_speed_m_s':config.get('velocity') if config['planner']=='mononav' else config.get('maximum_speed'),
             'depth_diagnostic':config.get('diagnostic_depth_source')=='ground-truth',
             'width_m':c['corridor']['width'],'blocks':c['corridor']['obstacles'],
             'placement_id':geometry['placement_id'],
             'condition':'+'.join(active) or 'clean','outcome':result['outcome'],
             **{k:metrics.get(k) for k in ('route_progress_percent','route_gates_passed','route_gate_count',
                         'path_length_m','mission_duration_sim_s','minimum_obstacle_clearance_m','stationary_fraction')},
             'contact':';'.join((result.get('termination',{}).get('detail') or {}).get('objects',[]))
                       if isinstance(result.get('termination',{}).get('detail'),dict) else ''}
        rows.append(row)
        key=geometry['placement_id']
        group=groups.setdefault(key,{'geometry':geometry,'runs':[]})
        group['runs'].append((folder,config,result))
    if not rows:raise ValueError('No completed corridor artifacts found')
    with (root/'trials.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    (root/'summary.json').write_text(json.dumps({'flights':rows},indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    import math
    colors={'mononav':'#2267b3','kim':'#df7b20','pulp_dronet':'#28835a'}
    for key,group in groups.items():
        g=group['geometry'];fig,ax=plt.subplots(figsize=(8,7))
        for kind,color in [('walls','#a4a7ab'),('obstacles','#ad835a')]:
            for b in g[kind]:
                x,y,_=b['center'];dx,dy,_=b['size'];a=b['yaw']
                points=[(x+u*math.cos(a)-v*math.sin(a),y+u*math.sin(a)+v*math.cos(a))
                        for u,v in [(-dx/2,-dy/2),(dx/2,-dy/2),(dx/2,dy/2),(-dx/2,dy/2)]]
                ax.add_patch(Polygon(points,color=color))
        ax.plot(*zip(*g['route']),':',color='#b9b9b9',lw=1,label='Scoring route (not reactive input)')
        # Most recent clean attempt per target. Failures remain explicit and
        # all attempts, including development cutoffs, are in trials.csv.
        latest={}
        for folder,c,r in sorted(group['runs'],key=lambda item:(item[0]/'result.json').stat().st_mtime):
            if c.get('diagnostic_depth_source')!='ground-truth' and not any(c['condition'][k] for k in ('rgb_noise','delay','patch_enabled')):
                latest[c['planner']]=(folder,c,r)
        for model,(folder,c,r) in latest.items():
            samples=json.loads((folder/'samples.json').read_text());xy=[s['position_m'][:2] for s in samples]
            if not xy:continue
            label=model+(' '+c.get('pulp_variant','resblock') if model=='pulp_dronet' else '')
            ax.plot(*zip(*xy),color=colors.get(model,'black'),lw=1.9,label=f'{label}: {r["outcome"]}')
            ax.scatter(*xy[-1],marker='o' if r['outcome']=='route_completed' else 'x',color=colors.get(model,'black'),s=65,zorder=5)
        ax.scatter(*g['route'][0],marker='^',color='black',s=65,label='Start',zorder=5)
        ax.scatter(*g['route'][-1],marker='*',color='black',s=110,label='Finish cross-section',zorder=5)
        ax.set_aspect('equal');ax.autoscale();ax.margins(.07);ax.grid(alpha=.15)
        ax.set(xlabel='World x (m)',ylabel='World y (m)',title=f'{g["spec"]["shape"]} | {g["spec"]["width"]}m wide | {len(g["obstacles"])} blocks\nMost recent clean attempt per target; measured simulator trajectories')
        ax.legend(fontsize=8,loc='best');fig.tight_layout()
        fig.savefig(root/f'paths_{g["spec"]["shape"]}_{key[:8]}.png',dpi=160);plt.close(fig)
    return rows


def plot_pairs(root):
    """Plot paired measured trajectories; refuse geometry/target mismatches."""
    root=Path(root)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    import math
    outputs=[]
    for history_file in root.rglob('history.json'):
        for entry in json.loads(history_file.read_text()):
            for pair in entry.get('pairs',[]):
                folders=[Path(pair[role]['result_dir']) for role in ('clean','perturbed')]
                if not all((p/'corridor_geometry.json').exists() for p in folders):continue
                geometries=[json.loads((p/'corridor_geometry.json').read_text()) for p in folders]
                configs=[json.loads((p/'scenario.json').read_text()) for p in folders]
                frozen=[{k:v for k,v in c.items() if k not in ('name','condition')} for c in configs]
                environment=[{k:v for k,v in c['condition'].items()
                              if k not in ('name','rgb_noise','depth_noise','delay','patch_enabled')} for c in configs]
                if geometries[0]['placement_id']!=geometries[1]['placement_id'] or frozen[0]!=frozen[1] or environment[0]!=environment[1]:
                    raise ValueError('Cannot plot an unmatched corridor pair')
                g=geometries[0];fig,ax=plt.subplots(figsize=(8,7))
                for kind,color in [('walls','#a4a7ab'),('obstacles','#ad835a')]:
                    for b in g[kind]:
                        x,y,_=b['center'];dx,dy,_=b['size'];a=b['yaw']
                        points=[(x+u*math.cos(a)-v*math.sin(a),y+u*math.sin(a)+v*math.cos(a))
                                for u,v in [(-dx/2,-dy/2),(dx/2,-dy/2),(dx/2,dy/2),(-dx/2,dy/2)]]
                        ax.add_patch(Polygon(points,color=color))
                for role,folder,color in zip(('clean','perturbed'),folders,('#2267b3','#c34e38')):
                    result=pair[role];m=result['metrics']
                    xy=[s['position_m'][:2] for s in json.loads((folder/'samples.json').read_text())]
                    if xy:
                        ax.plot(*zip(*xy),color=color,lw=2,label=f'{role}: {result["outcome"]} ({m.get("route_progress_percent",0):.1f}%)')
                        ax.scatter(*xy[-1],color=color,marker='o' if result['outcome']=='route_completed' else 'x',s=75)
                c=configs[1]['condition'];factors=', '.join(f'{k}={c[k]}' for k in ('rgb_noise','delay','patch_enabled') if c[k])
                ax.set_aspect('equal');ax.autoscale();ax.margins(.07);ax.grid(alpha=.15)
                ax.set(xlabel='World x (m)',ylabel='World y (m)',title=f'{pair["planner"]} | {g["spec"]["shape"]} | {factors}\nMatched environment and target settings; one pair, not a robustness estimate')
                ax.legend(fontsize=8,loc='best');fig.tight_layout()
                output=history_file.parent/f'pair_{entry["decision"]["round"]:02d}_{pair["planner"]}.png'
                fig.savefig(output,dpi=160);plt.close(fig);outputs.append(output)
    return outputs


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    a=p.parse_args();rows=summarize(a.root)
    for row in rows:print(row['run'],row['outcome'],row['route_progress_percent'])
    for output in plot_pairs(a.root):print('Pair plot:',output)
