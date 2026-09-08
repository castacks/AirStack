#!/usr/bin/env python3
"""Lines-of-code reuse study for the AirStack case studies (ICRA 2027, Sec. V / Table I).

For every case-study project the manifest (projects.yaml) pins one or more
*components*: a git repository, a BASE ref (the pre-existing code the project
started from -- usually the AirStack fork point) and a HEAD ref (the project's
final code).  For each component this script measures, from `git diff
--numstat` alone:

  * how many lines the project AUTHORED  (added in new files + added/removed
    in modified files), split by file category (code / config / docs / data),
  * how many of those lines are VENDORED (third-party or copied code the
    manifest lists explicitly, with a rationale),
  * how many pre-existing BASE files/lines were MODIFIED in place,
  * the size of the BASE tree under the same category rules,

and derives the reuse ratios reported in the paper.  Everything is computed
from pinned SHAs, so anyone with read access to the repositories reproduces
the numbers bit-for-bit.  Vendored patterns and base exclusions are data in
the manifest, never hard-coded, so a reader can audit or change them.

Usage:
    python3 loc_reuse.py projects.yaml --repos-dir ./repos --out ./results
    python3 loc_reuse.py projects.yaml --only shimizu --json-only

Requires: python3 >= 3.8, PyYAML, git >= 2.20.  No other dependencies.
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import os
import subprocess
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.exit("PyYAML is required: pip install pyyaml")

EMPTY_TREE = "4b825dc642cb6eb9a060e54bf8d69288fbee4904"  # git's well-known empty tree

# ---------------------------------------------------------------------------
# File categories.  "source" (= code + config) is the headline denominator the
# paper uses; docs/data/other are reported alongside so either can be used.
# ---------------------------------------------------------------------------
CODE_EXT = {"py", "cpp", "c", "cc", "cxx", "h", "hpp", "hh", "cu", "js", "ts", "tsx",
            "sh", "bash", "zsh", "java", "rs", "go"}
CONFIG_EXT = {"yaml", "yml", "xml", "launch", "cmake", "txt", "cfg", "env", "ini",
              "toml", "urdf", "xacro", "sdf", "srv", "msg", "action", "idl", "repos",
              "conf", "properties", "gitmodules", "gitignore", "bashrc"}
DOCS_EXT = {"md", "rst", "adoc", "tex", "bib"}
DATA_EXT = {"json", "rviz", "ui", "csv", "obj", "dae", "stl", "usd", "usda", "patch",
            "ipynb", "svg", "html", "css", "lock", "log", "pgm", "map", "bt", "kml"}
BASENAME_CATEGORY = {"Dockerfile": "config", "CMakeLists.txt": "config", "Makefile": "config",
                     "LICENSE": "docs", "COPYING": "docs", "AGENTS.md": "docs", "CLAUDE.md": "docs"}


def categorize(path: str) -> str:
    base = path.rsplit("/", 1)[-1]
    if base in BASENAME_CATEGORY:
        return BASENAME_CATEGORY[base]
    if base.startswith("Dockerfile"):
        return "config"
    if base.startswith("."):
        stem = base[1:]
        if stem in CONFIG_EXT:
            return "config"
    ext = base.rsplit(".", 1)[-1].lower() if "." in base else ""
    if ext in CODE_EXT:
        return "code"
    if ext in CONFIG_EXT:
        return "config"
    if ext in DOCS_EXT:
        return "docs"
    if ext in DATA_EXT:
        return "data"
    return "other"


CATEGORIES = ["code", "config", "docs", "data", "other"]
SOURCE = ("code", "config")


def matches_any(path: str, patterns: List[str]) -> bool:
    return any(fnmatch.fnmatch(path, p) or fnmatch.fnmatch("/" + path, p) for p in patterns)


# ---------------------------------------------------------------------------
# git plumbing
# ---------------------------------------------------------------------------
def git(repo: Path, *args: str) -> str:
    return subprocess.run(["git", "-C", str(repo), *args], check=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True).stdout


def resolve(repo: Path, ref: str) -> str:
    return git(repo, "rev-parse", "--verify", ref + "^{commit}").strip()


@dataclass
class FileDelta:
    path: str
    status: str            # A M D R T
    added: int             # lines added ('-' binary -> 0, binary=True)
    deleted: int
    binary: bool
    gitlink: bool          # submodule pointer (mode 160000) -- never counted
    category: str
    vendored: bool
    excluded: bool         # matched a base_exclude pattern (third-party bundled in base)


def diff_files(repo: Path, base: str, head: str, vendored: List[str], excludes: List[str],
               paths: Optional[List[str]] = None) -> List[FileDelta]:
    """One FileDelta per path in base..head, combining --raw (modes/status) and --numstat (lines)."""
    extra = ["--"] + paths if paths else []
    raw = git(repo, "diff", "--raw", "-z", "--no-renames", base, head, *extra)
    num = git(repo, "diff", "--numstat", "-z", "--no-renames", base, head, *extra)
    modes: Dict[str, tuple] = {}
    toks = raw.split("\0")
    i = 0
    while i < len(toks) - 1:
        meta, path = toks[i], toks[i + 1]
        i += 2
        if not meta:
            continue
        parts = meta.lstrip(":").split()
        old_mode, new_mode, status = parts[0], parts[1], parts[4][0]
        modes[path] = (old_mode, new_mode, status)
    out: List[FileDelta] = []
    for rec in num.strip("\0").split("\0") if num.strip("\0") else []:
        a, d, path = rec.split("\t", 2)
        binary = a == "-"
        old_mode, new_mode, status = modes.get(path, ("?", "?", "M"))
        gitlink = "160000" in (old_mode, new_mode)
        out.append(FileDelta(path=path, status=status,
                             added=0 if binary else int(a), deleted=0 if binary else int(d),
                             binary=binary, gitlink=gitlink, category=categorize(path),
                             vendored=matches_any(path, vendored),
                             excluded=matches_any(path, excludes)))
    return out


def by_category(deltas: List[FileDelta], key) -> Dict[str, int]:
    c: Dict[str, int] = {k: 0 for k in CATEGORIES}
    for d in deltas:
        c[d.category] += key(d)
    c["source"] = c["code"] + c["config"]
    c["total"] = sum(c[k] for k in CATEGORIES)
    return c


# ---------------------------------------------------------------------------
# Measurement of one component
# ---------------------------------------------------------------------------
def measure_component(repo: Path, comp: dict, global_excludes: List[str]) -> dict:
    base_ref = comp.get("base") or EMPTY_TREE
    head_ref = comp["head"]
    base = EMPTY_TREE if base_ref == EMPTY_TREE else resolve(repo, base_ref)
    head = resolve(repo, head_ref)
    vendored = comp.get("vendored", []) or []
    vendored_patterns = [v["pattern"] if isinstance(v, dict) else v for v in vendored]
    excludes = list(global_excludes) + list(comp.get("base_excludes", []) or [])
    paths = comp.get("paths")

    deltas = [d for d in diff_files(repo, base, head, vendored_patterns, excludes, paths)
              if not d.gitlink]
    base_files = [d for d in diff_files(repo, EMPTY_TREE, base, vendored_patterns, excludes, paths)
                  if not d.gitlink and not d.binary] if base != EMPTY_TREE else []

    new = [d for d in deltas if d.status == "A"]
    modified = [d for d in deltas if d.status in ("M", "T")]
    deleted = [d for d in deltas if d.status == "D"]
    authored = [d for d in deltas if not d.vendored]
    vend = [d for d in deltas if d.vendored]

    base_kept = [d for d in base_files if not d.excluded]
    base_excluded = [d for d in base_files if d.excluded]

    res = {
        "repo": comp["repo"],
        "role": comp.get("role", ""),
        "base_ref": base_ref, "base_sha": base, "head_ref": head_ref, "head_sha": head,
        "paths": paths,
        "commits_base_to_head": (int(git(repo, "rev-list", "--count", f"{base}..{head}").strip())
                                 if base != EMPTY_TREE else int(git(repo, "rev-list", "--count", head).strip())),
        "files": {
            "changed": len(deltas), "new": len(new), "modified": len(modified),
            "deleted": len(deleted), "binary": sum(d.binary for d in deltas),
        },
        # Everything the project put into the tree, vendored or not
        "delta_added": by_category(deltas, lambda d: d.added),
        "delta_deleted": by_category(deltas, lambda d: d.deleted),
        # Lines the project wrote itself (vendored excluded)
        "authored_added": by_category(authored, lambda d: d.added),
        "authored_deleted": by_category(authored, lambda d: d.deleted),
        "authored_in_new_files": by_category([d for d in authored if d.status == "A"], lambda d: d.added),
        "authored_in_modified_files_added": by_category([d for d in authored if d.status in ("M", "T")], lambda d: d.added),
        "authored_in_modified_files_deleted": by_category([d for d in authored if d.status in ("M", "T")], lambda d: d.deleted),
        # Third-party / copied code brought in (listed in the manifest)
        "vendored_added": by_category(vend, lambda d: d.added),
        "vendored_files": len(vend),
        "vendored_patterns": vendored,
        # Base tree
        "base_files": len(base_kept),
        "base_loc": by_category(base_kept, lambda d: d.added),
        "base_excluded_files": len(base_excluded),
        "base_excluded_loc": by_category(base_excluded, lambda d: d.added),
        "base_exclude_patterns": excludes,
        # Pre-existing base files touched in place (vendored irrelevant here: they're base files)
        "base_modified_files": [
            {"path": d.path, "added": d.added, "deleted": d.deleted, "category": d.category,
             "base_size": next((b.added for b in base_files if b.path == d.path), None)}
            for d in sorted(modified, key=lambda d: -(d.added + d.deleted))],
        "base_deleted_files": [{"path": d.path, "deleted": d.deleted} for d in deleted],
        "new_files_by_dir": top_dirs(new, depth=comp.get("dir_depth", 4)),
        "authored_by_ext": by_ext(authored),
    }
    m_src = [d for d in modified if d.category in SOURCE]
    res["base_modified_source"] = {
        "files": len(m_src),
        "lines_changed": sum(d.added + d.deleted for d in m_src),
        "added": sum(d.added for d in m_src), "deleted": sum(d.deleted for d in m_src),
    }
    return res


def top_dirs(deltas: List[FileDelta], depth: int) -> List[dict]:
    agg: Dict[str, List[int]] = defaultdict(lambda: [0, 0, 0])
    for d in deltas:
        parts = d.path.split("/")
        key = "/".join(parts[:depth]) if len(parts) > depth else "/".join(parts[:-1]) or "(root)"
        agg[key][0] += d.added
        agg[key][1] += d.deleted
        agg[key][2] += 1
    return [{"dir": k, "added": v[0], "deleted": v[1], "files": v[2]}
            for k, v in sorted(agg.items(), key=lambda kv: -kv[1][0])]


def by_ext(deltas: List[FileDelta]) -> List[dict]:
    agg: Dict[str, List[int]] = defaultdict(lambda: [0, 0, 0])
    for d in deltas:
        base = d.path.rsplit("/", 1)[-1]
        ext = base.rsplit(".", 1)[-1] if "." in base and not base.startswith(".") else base
        agg[ext][0] += d.added
        agg[ext][1] += d.deleted
        agg[ext][2] += 1
    return [{"ext": k, "added": v[0], "deleted": v[1], "files": v[2]}
            for k, v in sorted(agg.items(), key=lambda kv: -kv[1][0])]


# ---------------------------------------------------------------------------
# Project-level aggregation and ratios
# ---------------------------------------------------------------------------
def aggregate(project: dict, comps: List[dict]) -> dict:
    """Paper ratios.  'base' = pre-existing code the team started from (AirStack fork
    point plus any pre-existing library they extended); 'authored' = lines the team wrote;
    'vendored' = third-party/copied lines they brought in.  All on the SOURCE
    (code+config) category unless stated."""
    def s(key, cat="source"):
        return sum(c[key][cat] for c in comps)

    airstack = [c for c in comps if c.get("role") == "airstack"]
    a = airstack[0] if airstack else None
    authored_src = s("authored_added")
    authored_all = s("authored_added", "total")
    vendored_src = s("vendored_added")
    base_src = s("base_loc")
    airstack_base_src = a["base_loc"]["source"] if a else None
    final_src = base_src - s("authored_deleted") + authored_src + vendored_src
    out = {
        "project": project["name"],
        "components": [c["repo"] for c in comps],
        "authored_source_loc": authored_src,
        "authored_total_loc": authored_all,
        "authored_source_deleted": s("authored_deleted"),
        "authored_by_category": {k: s("authored_added", k) for k in CATEGORIES},
        "vendored_source_loc": vendored_src,
        "vendored_total_loc": s("vendored_added", "total"),
        "base_source_loc": base_src,
        "airstack_base_source_loc": airstack_base_src,
        "final_source_loc": final_src,
        "ratios": {
            "authored_over_base": authored_src / base_src if base_src else None,
            "base_share_of_final": (base_src - s("authored_deleted")) / final_src if final_src else None,
            "authored_share_of_final": authored_src / final_src if final_src else None,
            "vendored_share_of_final": vendored_src / final_src if final_src else None,
        },
    }
    if a:
        bm = a["base_modified_source"]
        out["airstack_base_modified_in_place"] = {
            "files": bm["files"], "lines_changed": bm["lines_changed"],
            "share_of_base": bm["lines_changed"] / a["base_loc"]["source"] if a["base_loc"]["source"] else None,
            "files_share_of_base": bm["files"] / a["base_files"] if a["base_files"] else None,
            "deleted_files": len(a["base_deleted_files"]),
        }
        out["airstack_authored_source_loc"] = a["authored_added"]["source"]
        out["airstack_additive_share"] = (a["authored_in_new_files"]["source"] / a["authored_added"]["source"]
                                          if a["authored_added"]["source"] else None)
    return out


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------
def pct(x: Optional[float]) -> str:
    if x is None:
        return "n/a"
    return f"{100 * x:.2f}%" if 0 < x < 0.01 else f"{100 * x:.1f}%"


def fmt(n: Optional[int]) -> str:
    return "n/a" if n is None else f"{n:,}"


def render_project_md(project: dict, comps: List[dict], agg: dict) -> str:
    L = [f"# {project['name']}: lines-of-code reuse vs. base", ""]
    if project.get("notes"):
        L += [project["notes"].strip(), ""]
    L += ["Generated by `loc_reuse.py (notebook/012-paper-measurement-studies/reuse)` from the pinned refs below; "
          "re-run the script to regenerate.", ""]
    L += ["## Components", "",
          "| role | repo | base (pre-existing) | head (final) | commits |", "|---|---|---|---|---|"]
    for c in comps:
        L.append(f"| {c['role']} | `{c['repo']}` | `{c['base_ref']}` → `{c['base_sha'][:10]}` | "
                 f"`{c['head_ref']}` → `{c['head_sha'][:10]}` | {c['commits_base_to_head']} |")
    L += ["", "## Headline (source = code + config files; see Method)", "",
          "| metric | value |", "|---|---:|",
          f"| Lines authored by the project (source) | {fmt(agg['authored_source_loc'])} |",
          f"| Lines authored by the project (all text files) | {fmt(agg['authored_total_loc'])} |",
          f"| Vendored / copied third-party lines brought in (source) | {fmt(agg['vendored_source_loc'])} |",
          f"| Pre-existing base the project started from (source) | {fmt(agg['base_source_loc'])} |",
          f"| — of which AirStack at the fork point | {fmt(agg['airstack_base_source_loc'])} |",
          f"| Final system (source) | {fmt(agg['final_source_loc'])} |",
          f"| Authored / base | {pct(agg['ratios']['authored_over_base'])} |",
          f"| Unmodified base as share of final | {pct(agg['ratios']['base_share_of_final'])} |",
          f"| Authored as share of final | {pct(agg['ratios']['authored_share_of_final'])} |",
          f"| Vendored as share of final | {pct(agg['ratios']['vendored_share_of_final'])} |"]
    if "airstack_base_modified_in_place" in agg:
        m = agg["airstack_base_modified_in_place"]
        L += [f"| AirStack base files modified in place | {m['files']} ({pct(m['files_share_of_base'])} of base files) |",
              f"| AirStack base source lines changed in place (+/−) | {fmt(m['lines_changed'])} ({pct(m['share_of_base'])} of base) |",
              f"| AirStack base files deleted | {m['deleted_files']} |",
              f"| Share of AirStack-side authored source in brand-new files | {pct(agg['airstack_additive_share'])} |"]
    L.append("")
    L += ["Authored lines by category: " + ", ".join(f"{k} {fmt(v)}" for k, v in agg["authored_by_category"].items()), ""]
    for c in comps:
        L += [f"## Component `{c['repo']}` ({c['role']})", ""]
        f = c["files"]
        L += [f"`git diff --numstat {c['base_sha'][:10]} {c['head_sha'][:10]}`"
              + (f" -- {' '.join(c['paths'])}" if c.get("paths") else "")
              + f": {f['changed']} files ({f['new']} new, {f['modified']} modified, {f['deleted']} deleted, {f['binary']} binary), "
              f"+{fmt(c['delta_added']['total'])} / −{fmt(c['delta_deleted']['total'])} text lines.", ""]
        L += ["| category | authored + | authored − | of which in new files | vendored + | base LOC | base excluded (3rd-party) |",
              "|---|---:|---:|---:|---:|---:|---:|"]
        for k in CATEGORIES + ["source", "total"]:
            L.append(f"| {k} | {fmt(c['authored_added'][k])} | {fmt(c['authored_deleted'][k])} | "
                     f"{fmt(c['authored_in_new_files'][k])} | {fmt(c['vendored_added'][k])} | "
                     f"{fmt(c['base_loc'][k])} | {fmt(c['base_excluded_loc'][k])} |")
        L.append("")
        if c["vendored_patterns"]:
            L += ["Vendored / copied patterns (excluded from *authored*):", ""]
            for v in c["vendored_patterns"]:
                if isinstance(v, dict):
                    L.append(f"- `{v['pattern']}` — {v.get('why', '')}")
                else:
                    L.append(f"- `{v}`")
            L.append("")
        if c["base_modified_files"]:
            L += ["Pre-existing files modified in place:", "",
                  "| + | − | base size | file |", "|---:|---:|---:|---|"]
            for m in c["base_modified_files"]:
                L.append(f"| {m['added']} | {m['deleted']} | {fmt(m['base_size'])} | `{m['path']}` |")
            L.append("")
        if c["base_deleted_files"]:
            L += ["Pre-existing files deleted: " + ", ".join(f"`{d['path']}` (−{d['deleted']})" for d in c["base_deleted_files"]), ""]
        L += ["New files by directory (top 15):", "", "| + | − | files | directory |", "|---:|---:|---:|---|"]
        for t in c["new_files_by_dir"][:15]:
            L.append(f"| {fmt(t['added'])} | {fmt(t['deleted'])} | {t['files']} | `{t['dir']}` |")
        L += ["", "Authored lines by extension (top 12):", "", "| + | − | files | ext |", "|---:|---:|---:|---|"]
        for t in c["authored_by_ext"][:12]:
            L.append(f"| {fmt(t['added'])} | {fmt(t['deleted'])} | {t['files']} | `{t['ext']}` |")
        L.append("")
    L += ["## Method", "",
          "- *Base* is the pre-existing tree the team started from: the AirStack fork point "
          "(`git merge-base` with `castacks/AirStack`) and, where a team extended another "
          "pre-existing library in its own repo, that library's trunk.",
          "- *Authored* = lines added in `base..head` minus lines under the manifest's vendored "
          "patterns (third-party or copied code; each pattern carries a rationale).",
          "- *Source* = code (`.py .cpp .h .hpp .c .js .sh …`) + config (`.yaml .xml .launch .cmake "
          "CMakeLists.txt Dockerfile .env .urdf .srv .msg .action …`). Docs (`.md .rst`), data "
          "(`.json .rviz .ui .obj .usd .csv .patch`) and binaries are reported but not in *source*.",
          "- *Base excluded* = third-party code bundled inside the base tree (bundled `glad`, "
          "`stb_image`, `xdot`, generated Foxglove `dist/` bundles, OpenVDB `Find*.cmake`, the vendored "
          "`natnet_ros2` driver); listed in the manifest and removed from every denominator.",
          "- Git submodule pointers (mode 160000) are skipped on both sides; submodule contents are "
          "not counted in any figure.",
          "- Renames are disabled (`--no-renames`) so a moved file counts as delete + add, like a "
          "reader running plain `git diff --numstat` would see.",
          "- `final = base − authored_deleted + authored + vendored`; *unmodified base share* = "
          "(base − authored_deleted) / final.", ""]
    return "\n".join(L)


def render_summary_md(rows: List[dict], reported: List[dict]) -> str:
    L = ["# Case-study reuse accounting — summary", "",
         "Generated by `loc_reuse.py (notebook/012-paper-measurement-studies/reuse)`. Source = code + config lines; see per-project files for method and per-file detail.", "",
         "| project | authored source LOC | vendored LOC | AirStack base LOC | base files / lines modified in place | unmodified base share of final | authored share of final | AirStack-side additive share |",
         "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for r in rows:
        m = r.get("airstack_base_modified_in_place", {})
        L.append(f"| {r['project']} | {fmt(r['authored_source_loc'])} | {fmt(r['vendored_source_loc'])} | "
                 f"{fmt(r['airstack_base_source_loc'])} | "
                 f"{m.get('files', 'n/a')} / {fmt(m.get('lines_changed'))} ({pct(m.get('share_of_base'))}) | "
                 f"{pct(r['ratios']['base_share_of_final'])} | {pct(r['ratios']['authored_share_of_final'])} | "
                 f"{pct(r.get('airstack_additive_share'))} |")
    for r in reported:
        L.append(f"| {r['name']} (author-reported†) | {fmt(r['authored_source_loc'])} | {fmt(r.get('vendored_source_loc', 0))} | "
                 f"{fmt(r['airstack_base_source_loc'])} | {r['base_modified_files']} / {fmt(r['base_modified_lines'])} "
                 f"({pct(r['base_modified_lines'] / r['airstack_base_source_loc'])}) | "
                 f"{pct(r['base_share_of_final'])} | {pct(r['authored_share_of_final'])} | {pct(r.get('additive_share'))} |")
    if reported:
        L += ["", "† Repository not accessible to the paper authors; numbers copied verbatim from the "
              "team's own report (path in `projects.yaml`), which used the same git-diff-vs-fork-point method "
              "but its own source-file definition. The base LOC of the shared AirStack fork point was "
              "recomputed here where the SHA is public."]
    return "\n".join(L) + "\n"


def render_latex_rows(rows: List[dict], reported: List[dict], order: List[str]) -> str:
    """Rows for Table~\\ref{tab:casestudies}: one cell per project in `order`."""
    cells = {r["project"]: r for r in rows}
    rep = {r["name"]: r for r in reported}

    def k(n):  # 21,037 -> 21.0k
        return f"{n / 1000:.1f}k" if n >= 1000 else str(n)

    def lp(x):  # percent with a LaTeX-escaped sign
        return pct(x).replace("%", "\\%")

    r1, r2, r3 = [], [], []
    for name in order:
        if name in cells:
            r = cells[name]
            m = r.get("airstack_base_modified_in_place", {})
            r1.append(k(r["authored_source_loc"]))
            r2.append(f"{m.get('files', '--')} / {m.get('lines_changed', '--')} ({lp(m.get('share_of_base'))})")
            r3.append(lp(r["ratios"]["base_share_of_final"]))
        elif name in rep:
            r = rep[name]
            r1.append(k(r["authored_source_loc"]) + r"$^{\dagger}$")
            r2.append(f"{r['base_modified_files']} / {r['base_modified_lines']} "
                      f"({lp(r['base_modified_lines'] / r['airstack_base_source_loc'])})")
            r3.append(lp(r["base_share_of_final"]))
        else:
            r1.append(r"\todo{}"); r2.append(r"\todo{}"); r3.append(r"\todo{}")
    return ("% generated by loc_reuse.py (notebook/012-paper-measurement-studies/reuse) -- paste into tab:casestudies\n"
            "Source LOC written                    & " + " & ".join(r1) + r" \\" + "\n"
            "Base modified in place (files / LOC)  & " + " & ".join(r2) + r" \\" + "\n"
            "Base reused unmodified (\\% of final)  & " + " & ".join(r3) + r" \\" + "\n")


# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("manifest")
    ap.add_argument("--repos-dir", default="repos", help="where run_reuse_study.sh cloned the repositories")
    ap.add_argument("--out", default="results")
    ap.add_argument("--only", action="append", help="project key(s) to run")
    ap.add_argument("--json-only", action="store_true")
    args = ap.parse_args()

    manifest = yaml.safe_load(Path(args.manifest).read_text())
    global_excludes = manifest.get("base_excludes", []) or []
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rows, reported = [], []
    for key, project in manifest["projects"].items():
        if args.only and key not in args.only:
            continue
        project.setdefault("name", key)
        if project.get("reported"):
            r = dict(project["reported"])
            r["name"] = project["name"]
            # recompute the public base denominator if we can
            base_comp = project.get("base_only")
            if base_comp:
                repo = Path(args.repos_dir) / base_comp["repo_dir"]
                if repo.exists():
                    c = measure_component(repo, {"repo": base_comp["repo"], "base": None,
                                                 "head": base_comp["ref"], "role": "airstack",
                                                 "vendored": global_excludes}, global_excludes)
                    # base_only: HEAD *is* the base tree; report its LOC under our rules
                    r["airstack_base_source_loc_recomputed"] = c["authored_added"]["source"]
                    r["airstack_base_total_loc_recomputed"] = c["authored_added"]["total"]
                    r["airstack_base_excluded_loc_recomputed"] = c["vendored_added"]["source"]
            reported.append(r)
            (out / f"{key}.json").write_text(json.dumps(r, indent=2) + "\n")
            print(f"[{key}] author-reported numbers copied from {project['reported'].get('source_file', '?')}")
            continue
        comps = []
        for comp in project["components"]:
            repo = Path(args.repos_dir) / comp["repo_dir"]
            if not repo.exists():
                print(f"[{key}] SKIP: {repo} not present (private repo or clone failed); see run_reuse_study.sh", file=sys.stderr)
                comps = None
                break
            comps.append(measure_component(repo, comp, global_excludes))
        if comps is None:
            continue
        agg = aggregate(project, comps)
        rows.append(agg)
        (out / f"{key}.json").write_text(json.dumps({"project": agg, "components": comps}, indent=2) + "\n")
        if not args.json_only:
            (out / f"{key}.md").write_text(render_project_md(project, comps, agg))
        print(f"[{key}] authored source {agg['authored_source_loc']:,} | vendored {agg['vendored_source_loc']:,} | "
              f"base {agg['base_source_loc']:,} | base share of final {pct(agg['ratios']['base_share_of_final'])}")
    if not args.json_only:
        (out / "summary.md").write_text(render_summary_md(rows, reported))
        (out / "table_rows.tex").write_text(render_latex_rows(rows, reported, manifest.get("table_order", [r["project"] for r in rows])))
    (out / "summary.json").write_text(json.dumps({"projects": rows, "reported": reported}, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
