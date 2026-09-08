#!/usr/bin/env python3
"""Defect-mining enumerator for the ICRA 2027 AirStack paper, Sec. VI-B
("Where Bugs Are Found: Defect-Discovery Shift").

The study has two stages.  This script is STAGE 1 and is fully deterministic:
given a repository, a commit window and (optionally) a GitHub repo, it
enumerates *everything a classifier must look at* and records the
denominators the paper reports:

  commits.tsv            every commit in the window (sha, date, author, +/-, files, subject)
  candidates.md          full message + numstat of every commit whose message matches the
                         fix-keyword regex (plus any commit with --all-messages)
  pr_timeline.json/.md   (GitHub) every merged PR in the window: its commits in order, the
                         workflow runs that ran on each commit and their conclusions, review
                         and issue comments -- with RED->FIX sequences flagged, i.e. a failing
                         automated check followed by a later push to the same PR
  issues.json            (GitHub) issues closed in the window
  denominators.json      counts of all of the above + the exact commands used + tool versions

STAGE 2 -- reading each candidate's diff/thread and classifying it on the two
axes of defect_mining_prompt.md (defect class x discovery venue) --
is a judgement step done by a person or a coding agent following that prompt.
Its output (defects.csv + mining_report.md) must cite only artifacts that
appear in the Stage-1 files, which is what makes every row auditable.
summarize_defects.py then aggregates the per-project CSVs.

Usage examples:
  # AirStack core, CI-era window, with GitHub PR/CI history
  python3 mine_defects.py --project AirStack-core --repo repos/AirStack \
      --base <develop@2026-04-28> --head origin/develop --github castacks/AirStack \
      --since 2026-04-28 --until 2026-09-08 --out results/airstack-core

  # A team branch (no PRs): everything after the fork point
  python3 mine_defects.py --project Shimizu --repo repos/AirStack \
      --base 39e5e698cf34 --head origin/junbin/planning_demo --out results/shimizu

Requires: git; python3 >= 3.8; `gh` authenticated (only for --github).
Read-only: the script never modifies the repository.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import re
import subprocess
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, List, Optional

FIX_RE = re.compile(r"\b(fix|fixes|fixed|fixing|bug|bugs|broken|breaks?|repair|revert|hotfix|"
                    r"crash|regression|wrong|incorrect|fail(?:s|ed|ing|ure)?|error|issue|typo|"
                    r"workaround|patch)\b", re.I)
CMDS: List[str] = []


def run(cmd: List[str], check=True) -> str:
    CMDS.append(" ".join(cmd))
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if check and r.returncode:
        raise RuntimeError(f"{' '.join(cmd)}\n{r.stderr}")
    return r.stdout


def git(repo: str, *a: str) -> str:
    return run(["git", "-C", repo, *a])


def gh_api(path: str, paginate=True, jq: Optional[str] = None) -> str:
    cmd = ["gh", "api", path]
    if paginate:
        cmd.append("--paginate")
    if jq:
        cmd += ["--jq", jq]
    return run(cmd)


def gh_api_json(path: str, paginate=True) -> list:
    out = gh_api(path, paginate=paginate, jq=".")
    # --paginate concatenates JSON documents; split them
    items: list = []
    dec = json.JSONDecoder()
    i = 0
    out = out.strip()
    while i < len(out):
        obj, j = dec.raw_decode(out, i)
        items.append(obj)
        i = j
        while i < len(out) and out[i].isspace():
            i += 1
    return items


# ---------------------------------------------------------------------------
def enumerate_commits(repo: str, base: str, head_refs: List[str], since: Optional[str], until: Optional[str]) -> List[dict]:
    rng = [*head_refs, "--not", base] if base else head_refs
    extra = []
    if since:
        extra.append(f"--since={since}")
    if until:
        extra.append(f"--until={until}T23:59:59")
    fmt = "%H%x1f%aI%x1f%an%x1f%s%x1f%b%x1e"
    raw = git(repo, "log", "--no-merges", f"--format={fmt}", *extra, *rng)
    merges = git(repo, "rev-list", "--merges", "--count", *extra, *rng).strip()
    commits = []
    for rec in raw.split("\x1e"):
        rec = rec.strip("\n")
        if not rec.strip():
            continue
        sha, date, author, subject, body = rec.split("\x1f")
        stat = git(repo, "show", "--numstat", "--format=", sha).strip().splitlines()
        add = dele = 0
        files = []
        for line in stat:
            a, d, path = line.split("\t", 2)
            files.append(path)
            if a != "-":
                add += int(a)
                dele += int(d)
        text = subject + "\n" + body
        commits.append({"sha": sha, "date": date[:10], "author": author, "subject": subject.strip(),
                        "body": body.strip(), "added": add, "deleted": dele, "files": files,
                        "keyword_hit": bool(FIX_RE.search(text)),
                        "keywords": sorted({m.lower() for m in FIX_RE.findall(text)})})
    commits.sort(key=lambda c: c["date"])
    return commits, int(merges)


def write_commits(commits: List[dict], out: Path, all_messages: bool, github: Optional[str]):
    with (out / "commits.tsv").open("w") as f:
        f.write("sha\tdate\tauthor\tadded\tdeleted\tfiles\tkeyword_hit\tsubject\n")
        for c in commits:
            f.write(f"{c['sha']}\t{c['date']}\t{c['author']}\t{c['added']}\t{c['deleted']}\t{len(c['files'])}\t"
                    f"{int(c['keyword_hit'])}\t{c['subject']}\n")
    L = ["# Candidate commits", "",
         "Every commit in the window whose message matches the fix-keyword regex"
         + (" (plus all others, --all-messages)" if all_messages else "")
         + ". Read the diff before classifying; the message alone is `low` confidence.", ""]
    for c in commits:
        if not (c["keyword_hit"] or all_messages):
            continue
        url = f"https://github.com/{github}/commit/{c['sha']}" if github else c["sha"]
        L += [f"## {c['sha'][:10]} — {c['date']} — {c['author']} — {'KEYWORD ' + ','.join(c['keywords']) if c['keyword_hit'] else 'no keyword'}",
              "", f"**{c['subject']}**", "", url, ""]
        if c["body"]:
            L += ["```", c["body"], "```", ""]
        L += [f"+{c['added']} / −{c['deleted']} in {len(c['files'])} files:", ""]
        L += [f"- `{p}`" for p in c["files"][:40]]
        if len(c["files"]) > 40:
            L.append(f"- … {len(c['files']) - 40} more")
        L.append("")
    (out / "candidates.md").write_text("\n".join(L))


# ---------------------------------------------------------------------------
def github_history(github: str, since: str, until: str, bases: List[str], out: Path, commits_by_sha: Dict[str, dict]) -> dict:
    den: dict = {"github": github, "ci_reachable": False}
    # --- workflow runs in window, indexed by head_sha -------------------------------------
    runs_raw = gh_api_json(f"repos/{github}/actions/runs?per_page=100&created={since}..{until}")
    runs = [r for page in runs_raw for r in page.get("workflow_runs", [])]
    den["ci_reachable"] = True
    den["workflow_runs_total"] = len(runs)
    den["workflow_runs_by_name_event_conclusion"] = dict(Counter(
        f"{r['name']}|{r['event']}|{r['conclusion']}" for r in runs).most_common())
    by_sha: Dict[str, List[dict]] = defaultdict(list)
    for r in runs:
        by_sha[r["head_sha"]].append({"id": r["id"], "name": r["name"], "event": r["event"],
                                      "conclusion": r["conclusion"], "created_at": r["created_at"],
                                      "url": r["html_url"], "branch": r.get("head_branch")})
    # --- merged PRs ------------------------------------------------------------------------
    prs = []
    for b in bases:
        prs_json = run(["gh", "pr", "list", "--repo", github, "--state", "merged", "--limit", "500",
                        "--search", f"merged:{since}..{until} base:{b}",
                        "--json", "number,title,mergedAt,baseRefName,headRefName,author,url,additions,deletions"])
        prs += [p for p in json.loads(prs_json) if since <= p["mergedAt"][:10] <= until and p["baseRefName"] == b]
    den["merged_prs_in_window"] = len(prs)
    den["merged_prs_by_base"] = dict(Counter(p["baseRefName"] for p in prs))
    timeline = []
    red_fix = 0
    review_comments_total = 0
    for p in sorted(prs, key=lambda p: p["mergedAt"]):
        n = p["number"]
        pcommits = [c for page in gh_api_json(f"repos/{github}/pulls/{n}/commits?per_page=100") for c in page]
        rc = [c for page in gh_api_json(f"repos/{github}/pulls/{n}/comments?per_page=100") for c in page]
        ic = [c for page in gh_api_json(f"repos/{github}/issues/{n}/comments?per_page=100") for c in page]
        review_comments_total += len(rc)
        seq = []
        for c in pcommits:
            sha = c["sha"]
            seq.append({"sha": sha, "date": c["commit"]["author"]["date"][:10],
                        "subject": c["commit"]["message"].splitlines()[0][:120],
                        "runs": by_sha.get(sha, []),
                        "in_history": sha in commits_by_sha})
        # RED -> FIX: a failing run on commit i, and a later commit j>i in the same PR
        flags = []
        for i, s in enumerate(seq):
            fails = [r for r in s["runs"] if r["conclusion"] == "failure"]
            if fails and i + 1 < len(seq):
                flags.append({"failing_sha": s["sha"], "failing_runs": [f["url"] for f in fails],
                              "workflows": sorted({f["name"] for f in fails}),
                              "next_commits": [{"sha": t["sha"], "subject": t["subject"]} for t in seq[i + 1:i + 4]]})
        red_fix += len(flags)
        timeline.append({"number": n, "title": p["title"], "url": p["url"], "merged_at": p["mergedAt"][:10],
                         "base": p["baseRefName"], "head": p["headRefName"], "author": p["author"]["login"],
                         "additions": p["additions"], "deletions": p["deletions"],
                         "n_commits": len(seq), "n_review_comments": len(rc), "n_issue_comments": len(ic),
                         "commits": seq, "red_then_fix": flags,
                         "review_comments": [{"path": c.get("path"), "body": c["body"][:600], "user": c["user"]["login"],
                                              "url": c["html_url"]} for c in rc],
                         "issue_comments": [{"body": c["body"][:600], "user": c["user"]["login"], "url": c["html_url"]}
                                            for c in ic if not c["user"]["login"].endswith("[bot]")]})
    den["pr_review_comments_total"] = review_comments_total
    den["red_then_fix_sequences"] = red_fix
    (out / "pr_timeline.json").write_text(json.dumps(timeline, indent=1) + "\n")
    # --- issues ----------------------------------------------------------------------------
    issues_json = run(["gh", "issue", "list", "--repo", github, "--state", "all", "--limit", "1000",
                       "--json", "number,title,state,createdAt,closedAt,url,labels"])
    issues = [i for i in json.loads(issues_json) if since <= (i.get("closedAt") or i["createdAt"])[:10] <= until]
    den["issues_in_window"] = len(issues)
    (out / "issues.json").write_text(json.dumps(issues, indent=1) + "\n")
    # --- markdown view for the classifier ------------------------------------------------
    L = [f"# Merged PRs into {', '.join(bases)} of {github}, {since}..{until}", "",
         f"{len(prs)} PRs; {len(runs)} workflow runs in window; {red_fix} RED→FIX sequences "
         "(a failing automated check on a PR commit followed by a later push to the same PR).", ""]
    for t in timeline:
        L += [f"## PR #{t['number']} — {t['title']} ({t['merged_at']}, → {t['base']}, @{t['author']})", "",
              t["url"], "",
              f"{t['n_commits']} commits, +{t['additions']}/−{t['deletions']}, {t['n_review_comments']} review comments, "
              f"{t['n_issue_comments']} issue comments" + (f", **{len(t['red_then_fix'])} RED→FIX**" if t["red_then_fix"] else ""), ""]
        for s in t["commits"]:
            concl = ", ".join(f"{r['name'].replace(' ', '')}={r['conclusion']}" for r in s["runs"]) or "no runs"
            L.append(f"- `{s['sha'][:10]}` {s['date']} {s['subject']}  ⟶ {concl}")
        for f in t["red_then_fix"]:
            L += ["", f"  **RED→FIX** after `{f['failing_sha'][:10]}` ({', '.join(f['workflows'])}): " +
                  "; ".join(f['failing_runs']), "  followed by: " +
                  "; ".join(f"`{c['sha'][:10]}` {c['subject']}" for c in f["next_commits"])]
        if t["review_comments"]:
            L += ["", "  Review comments:"]
            for c in t["review_comments"][:30]:
                L.append(f"  - @{c['user']} `{c['path']}`: {c['body'].splitlines()[0][:200] if c['body'] else ''}  ({c['url']})")
        if t["issue_comments"]:
            L += ["", "  Conversation:"]
            for c in t["issue_comments"][:20]:
                L.append(f"  - @{c['user']}: {c['body'].splitlines()[0][:200] if c['body'] else ''}  ({c['url']})")
        L.append("")
    (out / "pr_timeline.md").write_text("\n".join(L))
    return den


# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--project", required=True)
    ap.add_argument("--repo", required=True, help="local clone (read-only)")
    ap.add_argument("--base", help="exclusive lower bound commit (fork point / window start); omit for whole history")
    ap.add_argument("--head", action="append", required=True, help="tip ref(s); repeatable")
    ap.add_argument("--github", help="owner/repo for PR / CI / issue history via gh")
    ap.add_argument("--links", help="owner/repo used only to render commit URLs (default: --github)")
    ap.add_argument("--pr-base", action="append", help="PR base branches to include (default: develop, main)")
    ap.add_argument("--since", help="ISO date lower bound for PRs/runs/issues (and for commits only when no --base is given)")
    ap.add_argument("--until", help="ISO date upper bound (default today)")
    ap.add_argument("--all-messages", action="store_true", help="include every commit in candidates.md, not just keyword hits")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    until = a.until or dt.date.today().isoformat()

    base = git(a.repo, "rev-parse", a.base).strip() if a.base else None
    heads = [git(a.repo, "rev-parse", h).strip() for h in a.head]
    # With a --base the window is topological (base..head); --since/--until then bound only the
    # GitHub history, so rebased commits carrying older author dates are not dropped.
    commits, merges = enumerate_commits(a.repo, base, heads, None if base else a.since, None if base else a.until)
    write_commits(commits, out, a.all_messages, a.links or a.github)
    den = {
        "project": a.project, "repo": a.repo, "github": a.github,
        "base": base, "heads": dict(zip(a.head, heads)), "since": a.since, "until": until,
        "mined_on": dt.date.today().isoformat(),
        "git_version": run(["git", "--version"]).strip(),
        "commits_in_window": len(commits), "merge_commits_in_window": merges,
        "commits_by_author": dict(Counter(c["author"] for c in commits).most_common()),
        "keyword_candidates": sum(c["keyword_hit"] for c in commits),
        "keyword_regex": FIX_RE.pattern,
    }
    if a.github:
        den["gh_version"] = run(["gh", "--version"]).splitlines()[0]
        den.update(github_history(a.github, a.since or commits[0]["date"], until,
                                  a.pr_base or ["develop", "main"], out, {c["sha"]: c for c in commits}))
    den["commands"] = sorted(set(CMDS))
    (out / "denominators.json").write_text(json.dumps(den, indent=2) + "\n")
    print(json.dumps({k: v for k, v in den.items() if k not in ("commands", "workflow_runs_by_name_event_conclusion",
                                                                 "commits_by_author", "keyword_regex")}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
