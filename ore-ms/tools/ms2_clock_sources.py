"""EXP139 bounded read-only public-source snapshot. Never execute fetched code."""
import argparse
import concurrent.futures
import datetime
import hashlib
import json
from pathlib import Path
import re
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/clock-evidence/source-001'
REPOS = ['MS2-MultiSpectralStereoDataset', 'SupDepth4Thermal']


def collect(out=OUT):
    out.mkdir(parents=True, exist_ok=False)
    records = []
    def fetch(url, relative):
        request = urllib.request.Request(url, headers={'User-Agent': 'ore-ms-source-evidence-read-only'})
        with urllib.request.urlopen(request, timeout=45) as r:
            data = r.read(15_000_001)
            if len(data) > 15_000_000: raise ValueError('bounded public source too large')
            final = r.url
        path = out/relative; path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(data)
        record = {'url': url, 'resolved_url': final, 'path': str(path.relative_to(ROOT)),
                  'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest(),
                  'retrieved_utc': datetime.datetime.now(datetime.timezone.utc).isoformat()}
        return data, record
    def get(url, relative):
        data, record = fetch(url, relative); records.append(record); return data
    def pages(repo, endpoint, label):
        rows = []; count = 0
        for page in range(1, 21):
            url = f'https://api.github.com/repos/UkcheolShin/{repo}/{endpoint}'
            url += ('&' if '?' in url else '?')+f'per_page=100&page={page}'
            values = json.loads(get(url, f'{repo}/{label}-{page:02d}.json'))
            if not isinstance(values, list): raise ValueError('expected paginated list')
            rows.extend(values); count += 1
            if len(values) < 100: break
        else: raise ValueError('pagination bound reached; refuse completeness claim')
        return rows, count
    summary = []
    for repo in REPOS:
        commit = json.loads(get(f'https://api.github.com/repos/UkcheolShin/{repo}/commits/main', f'{repo}/head.json'))
        revision = commit['sha']
        tree = json.loads(get(f'https://api.github.com/repos/UkcheolShin/{repo}/git/trees/{revision}?recursive=1', f'{repo}/tree.json'))
        if tree['truncated'] is not False: raise ValueError('truncated source tree')
        paths = [x['path'] for x in tree['tree'] if x['type'] == 'blob']
        chosen = [p for p in paths if (p.lower().endswith('.md')
                  or (('dataloader' in p.lower() or re.search(r'acqui|timestamp|synch|rosbag|export', p, re.I))
                      and Path(p).suffix.lower() in ['.py', '.sh', '.txt', '.yaml', '.yml', '.xml', '.cpp', '.h', '.launch']))]
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            jobs = [pool.submit(fetch, f'https://raw.githubusercontent.com/UkcheolShin/{repo}/{revision}/{p}', f'{repo}/files/{p}') for p in chosen]
            for job in jobs:
                _, record = job.result(); records.append(record)
        issues, ip = pages(repo, 'issues?state=all', 'issues')
        comments, cp = pages(repo, 'issues/comments', 'comments')
        ids = [x['number'] for x in issues if 'pull_request' not in x]
        if len(ids) != len(set(ids)): raise ValueError('duplicate issue pagination')
        for issue in issues:
            actual = sum(c['issue_url'] == issue['url'] for c in comments)
            if actual != issue['comments']: raise ValueError('incomplete/concurrently changed comment collection')
        summary.append({'repo': repo, 'revision': revision, 'commit_date': commit['commit']['committer']['date'],
                        'tree_truncated': False, 'blob_count': len(paths), 'selected_files': chosen,
                        'issues_excluding_prs': len(ids), 'issue_numbers': sorted(ids),
                        'issue_pages': ip, 'comments': len(comments), 'comment_pages': cp,
                        'scope': 'full tree path list; all Markdown and loader/relevant-name text files; all issue bodies/comments'})
    for label, url in [('paper.html', 'https://arxiv.org/html/2503.22060v1'),
                       ('dataset-site.html', 'https://sites.google.com/view/multi-spectral-stereo-dataset')]:
        get(url, label)
    registry = {'retrieved_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                'repos': summary, 'sources': sorted(records, key=lambda x: x['path']),
                'scope': 'bounded public source refresh; not proof that acquisition code exists nowhere',
                'scientific_payload_access': False, 'contact_sent': False}
    (out/'registry.json').write_text(json.dumps(registry, indent=2)+'\n')
    print(json.dumps({'snapshots': len(records), 'repositories': summary}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('--output', type=Path, default=OUT)
    collect(parser.parse_args().output)
