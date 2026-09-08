"""Rescore final detector-circle progress for an already evaluated cohort."""
import argparse
import concurrent.futures
import json
from pathlib import Path
import team_progress as tp


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--source', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--radius', type=float, required=True)
    a = p.parse_args()
    old = json.loads(Path(a.source).read_text())['rows']
    assert 0 < a.radius <= min(r['target_circle_radius_m'] for r in old)

    def score(r):
        count = 0
        if r['detected_victims']:
            bag = Path(r['bag'])
            gt, _, plans, _, _ = tp.load_static_and_plan_starts(bag)
            starts, shared = tp.parse_starts(Path(r['iter_dir']))
            times = {n: shared if shared is not None else starts.get(n, plans.get(n))
                     for n in sorted(set(plans) | set(starts))}
            assert times and all(t is not None for t in times.values())
            hits, _, _, _ = tp.load_search_evidence(
                bag, gt, times, (a.radius,), r['budget_s'], 50)
            assert len(gt) == r['gt_victims']
            count = len(hits[a.radius])
        return dict(scene=r['scene'], method=r['method'], iter_dir=r['iter_dir'],
                    gt_victims=r['gt_victims'], detected_victims=count,
                    progress=count/r['gt_victims'] if r['gt_victims'] else 0,
                    radius_m=a.radius)

    rows = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        futures = [pool.submit(score, r) for r in old]
        for f in concurrent.futures.as_completed(futures):
            rows.append(f.result())
            print(f'{len(rows)}/{len(old)}', flush=True)
    averages = []
    for method in sorted({r['method'] for r in rows}):
        group = [r for r in rows if r['method'] == method]
        averages.append(dict(method=method, evaluated_runs=len(group),
                             average_progress=sum(r['progress'] for r in group)/len(group)))
    Path(a.output).write_text(json.dumps(dict(rows=rows, averages=averages,
        analysis_complete=True, radius_m=a.radius), indent=2)+'\n')
    print(json.dumps(averages), flush=True)


if __name__ == '__main__':
    main()
