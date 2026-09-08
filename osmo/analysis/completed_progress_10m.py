"""Progress-only NAS evaluation; completed cells and evaluated cells are distinct."""
import concurrent.futures
import json
from pathlib import Path
import team_progress as tp

ROOT = Path('/media/share/coa-sei')
OUT = Path('osmo/results/completed_progress_10m.json')
METHODS = {'frontier', 'lawnmower', 'vlfm', 'conavgpt2_team'}


def discover():
    chosen = {}
    for meta in sorted(ROOT.glob('*/*/iter_*/iteration.json')):
        if any(s in str(meta) for s in ('failed_attempt', '_rtf_', '1robot', 'diagnostic_')):
            continue
        try:
            if json.loads(meta.read_text()).get('status') != 'passed':
                continue
            scene, method, key = tp.identity(meta.parent)
            if key not in METHODS:
                continue
            # Latest accepted iteration per planned scene/method cell. Reruns
            # do not increase completed/planned experiment counts.
            previous = chosen.get((scene, method))
            if previous is None or meta.parent.parent.name > previous.parent.name:
                chosen[scene, method] = meta.parent
        except (ValueError, OSError):
            continue
    return chosen


def evaluate(path):
    scene, method, key = tp.identity(path)
    bag = tp.find_bag(path)
    annotation = tp.annotation_name(scene)
    candidate = Path('/tmp/eval_gt') / annotation
    if not candidate.exists():
        candidate = Path('osmo/results/gt_annotations') / annotation
    gt, _, plans, _, _ = tp.load_static_and_plan_starts(
        bag, candidate if candidate.exists() else None)
    starts, shared = tp.parse_starts(path)
    robots = set(plans) | set(starts)
    if len(robots) != 8:
        raise ValueError(f'expected eight robots, found {sorted(robots)}')
    times = {n: shared if shared is not None else starts.get(n, plans.get(n)) for n in robots}
    if any(t is None for t in times.values()):
        raise ValueError('missing timed-search start')
    hits, _, _, messages = tp.load_search_evidence(bag, gt, times, (10.0,), 600, 50)
    return dict(scene=scene, method=method, iter_dir=str(path), radius_m=10,
                gt_victims=len(gt), detected_victims=len(hits[10.0]),
                progress=len(hits[10.0])/len(gt), marker_messages=messages)


def main():
    chosen = discover()
    old = json.loads(OUT.read_text()) if OUT.exists() else {}
    cached = {r['iter_dir']: r for r in old.get('rows', []) if r.get('radius_m') == 10}
    # A strict subset radius cannot find a target absent at 12 m. Reuse only
    # the exact same accepted iteration, never a different rerun of the scene.
    prior = Path('osmo/results/team_progress_sep8.json')
    if prior.exists():
        for r in json.loads(prior.read_text())['rows']:
            if r['detected_victims'] == 0:
                cached.setdefault(r['iter_dir'], dict(scene=r['scene'], method=r['method'],
                    iter_dir=r['iter_dir'], radius_m=10, gt_victims=r['gt_victims'],
                    detected_victims=0, progress=0, evidence='zero at 12 m implies zero at 10 m'))
    rows = [cached[str(p)] for p in chosen.values() if str(p) in cached]
    errors = []

    def save():
        summary = []
        for method in sorted({m for _, m in chosen}):
            completed = sum(m == method for _, m in chosen)
            evaluated = [r for r in rows if r['method'] == method]
            summary.append(dict(method=method, completed_runs=completed,
                planned_8robot_runs=24, planned_all_fleet_sizes=48,
                analyzed_runs=len(evaluated), average_progress=(
                    sum(r['progress'] for r in evaluated)/len(evaluated)
                    if len(evaluated) == completed and completed else None)))
        payload = dict(radius_m=10, association='world XY Euclidean distance <= 10 m; no IoU',
            budget_s=600, chunk_stride=50, final_persistent_markers_included=True,
            analysis_complete=len(rows) == len(chosen), rows=rows,
            summary=summary, errors=errors,
            completed_iterations=[str(p) for p in chosen.values()])
        tmp = OUT.with_suffix('.tmp')
        tmp.write_text(json.dumps(payload, indent=2)+'\n')
        tmp.replace(OUT)

    save()
    print('Inventory', len(chosen), 'completed cells;', len(rows), 'cached zero rows', flush=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        futures = {pool.submit(evaluate, p): p for p in chosen.values() if str(p) not in cached}
        for future in concurrent.futures.as_completed(futures):
            path = futures[future]
            try:
                rows.append(future.result())
                print('DONE', len(rows), '/', len(chosen), path, flush=True)
            except Exception as exc:
                errors.append(dict(iter_dir=str(path), error=str(exc)))
                print('ERROR', path, exc, flush=True)
            save()
    return bool(errors)


if __name__ == '__main__':
    raise SystemExit(main())
