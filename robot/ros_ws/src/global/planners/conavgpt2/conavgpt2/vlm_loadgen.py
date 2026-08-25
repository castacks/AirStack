#!/usr/bin/env python3
"""Load generator that answers "can one VLM serve N drones?" with numbers.

Fires CoNavGPT-shaped rounds — the real vendored system prompt, 6 synthetic
480x480 frontier top views sized to the ~135 KiB/image the node actually emits,
`response_format: json_object`, `temperature 0.1`, `max_tokens 100` — at a
concurrency sweep, and prints latency, queue wait and makespan per level.

The worst case is the one that matters: every drone's round lands at the same
instant, so each level fires all its clients simultaneously and waits for the
last answer. That makespan is what has to fit inside `round_period_s`.

    /opt/lvlm-venv/bin/python -m conavgpt2.vlm_loadgen \
        --base-url http://localhost:8000/v1 --levels 1,2,5 --rounds 3
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import statistics
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List, Optional

import numpy as np
import requests
from PIL import Image, ImageDraw

try:  # the real prompt if the package is importable, so the token count is real
    from conavgpt2.vendor.system_prompt import system_prompt as SYSTEM_PROMPT
except Exception:  # noqa: BLE001
    SYSTEM_PROMPT = (
        "You are a multi-robot navigation agent equipped with a vision-language "
        "model. Assign one frontier to each robot. You should only respond in JSON "
        'format: {"robot_0": "frontier_1", "reason": "why"}\n')


def make_frontier_image(idx: int, seed: int, size: int = 480,
                        target_kib: float = 135.0) -> bytes:
    """A stand-in for upstream's get_all_candidate_maps() render.

    Only two properties matter for a latency measurement: the pixel dimensions
    (they set the visual-token count) and the encoded size (it sets the request
    body and the base64/decode cost). Structure is faked; JPEG quality is solved
    for so the bytes land where the real render lands.
    """
    rng = np.random.default_rng(seed)
    # explored/free white, unknown grey, obstacles speckled dark - the texture is
    # what makes the real render expensive to encode, so keep it.
    arr = np.full((size, size, 3), 230, dtype=np.uint8)
    arr[:] = rng.integers(200, 255, size=(size, size, 3), dtype=np.uint8)
    for _ in range(28):
        h, w = rng.integers(20, 90, size=2)
        y, x = rng.integers(0, size - max(h, w) - 1, size=2)
        patch = rng.integers(40, 120, size=(h, w, 3), dtype=np.uint8)
        arr[y:y + h, x:x + w] = patch
    img = Image.fromarray(arr)
    d = ImageDraw.Draw(img)
    # one thick red frontier polyline + the red frontier id + the black robot mark
    pts = [(int(rng.integers(0, size)), int(rng.integers(0, size))) for _ in range(6)]
    d.line(pts, fill=(255, 0, 0), width=5)
    d.text((10, 10), str(idx), fill=(255, 0, 0))
    d.text((size // 2, size // 2), "R0", fill=(0, 0, 0))

    lo, hi, best = 20, 97, None
    for _ in range(9):  # binary search quality onto the target size
        q = (lo + hi) // 2
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=q)
        n = buf.tell()
        best = buf.getvalue()
        if abs(n / 1024.0 - target_kib) < target_kib * 0.05:
            return best
        if n / 1024.0 < target_kib:
            lo = q + 1
        else:
            hi = q - 1
        if lo > hi:
            break
    return best


def build_request(model: str, images: List[bytes], goal: str, num_robots: int,
                  max_tokens: int, temperature: float) -> Dict[str, Any]:
    """Byte-identical in shape to chat_utils.message_prepare + chat_with_gpt4v."""
    parts: List[Dict[str, Any]] = [{
        "type": "text",
        "text": f"{num_robots} robots need to find a {goal}",
    }]
    for raw in images:
        b64 = base64.b64encode(raw).decode("utf-8")
        parts.append({"type": "image_url",
                      "image_url": {"url": f"data:image/jpeg;base64,{b64}"}})
    return {
        "model": model,
        "messages": [{"role": "system", "content": SYSTEM_PROMPT},
                     {"role": "user", "content": parts}],
        "response_format": {"type": "json_object"},
        "temperature": temperature,
        "max_tokens": max_tokens,
    }


def fire(session: requests.Session, url: str, payload: Dict[str, Any],
         client_id: str, timeout: float) -> Dict[str, Any]:
    t0 = time.perf_counter()
    try:
        r = session.post(url, json=payload, timeout=timeout,
                         headers={"X-Client-Id": client_id,
                                  "Authorization": "Bearer EMPTY"})
        dt = time.perf_counter() - t0
        if r.status_code != 200:
            return {"client": client_id, "ok": False, "latency_s": dt,
                    "error": f"HTTP {r.status_code}: {r.text[:200]}"}
        body = r.json()
        usage = body.get("usage", {})
        tim = body.get("x_timings", {})
        content = body["choices"][0]["message"]["content"]
        parsed = None
        try:
            parsed = json.loads(content)
        except Exception:  # noqa: BLE001
            pass
        return {"client": client_id, "ok": True, "latency_s": dt,
                "prompt_tokens": usage.get("prompt_tokens", 0),
                "completion_tokens": usage.get("completion_tokens", 0),
                "image_tokens": usage.get("prompt_tokens_details", {}).get("image_tokens", 0),
                "queue_wait_s": tim.get("queue_wait_s", 0.0),
                "prefill_s": tim.get("prefill_s", 0.0),
                "decode_s": tim.get("decode_s", 0.0),
                "infer_s": tim.get("infer_s", 0.0),
                "server_total_s": tim.get("total_s", 0.0),
                "json_ok": parsed is not None,
                "content": content}
    except Exception as exc:  # noqa: BLE001
        return {"client": client_id, "ok": False,
                "latency_s": time.perf_counter() - t0,
                "error": f"{type(exc).__name__}: {exc}"}


def _agg(vals: List[float]) -> Dict[str, float]:
    if not vals:
        return {"mean": 0.0, "p50": 0.0, "p95": 0.0, "max": 0.0}
    s = sorted(vals)
    def pct(q):
        if len(s) == 1:
            return s[0]
        i = q * (len(s) - 1)
        lo, hi = int(i), min(int(i) + 1, len(s) - 1)
        return s[lo] + (s[hi] - s[lo]) * (i - lo)
    return {"mean": statistics.fmean(s), "p50": pct(0.5), "p95": pct(0.95), "max": s[-1]}


def main(argv=None):
    p = argparse.ArgumentParser(
        description="concurrency sweep against the conavgpt2 VLM server",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--base-url", default="http://localhost:8000/v1")
    p.add_argument("--model", default="Qwen/Qwen2.5-VL-3B-Instruct")
    p.add_argument("--levels", default="1,2,5",
                   help="comma-separated concurrency levels = simultaneous drones")
    p.add_argument("--rounds", type=int, default=3,
                   help="repeats per level; every repeat fires all clients at once")
    p.add_argument("--images", type=int, default=6,
                   help="frontier images per round; upstream caps at 6")
    p.add_argument("--image-kib", type=float, default=135.0,
                   help="target JPEG size per frontier image")
    p.add_argument("--image-size", type=int, default=480,
                   help="render edge in px; must match map_cells to be honest")
    p.add_argument("--goal", default="person")
    p.add_argument("--num-robots", type=int, default=2,
                   help="the count named in the user text, as message_prepare does")
    p.add_argument("--max-tokens", type=int, default=100)
    p.add_argument("--temperature", type=float, default=0.1)
    p.add_argument("--timeout", type=float, default=600.0)
    p.add_argument("--round-period-s", type=float, default=30.0,
                   help="the node's cadence; a makespan above this means stale "
                        "assignments, which is the failure that matters")
    p.add_argument("--out", default="",
                   help="write per-request results as JSONL here")
    p.add_argument("--no-warmup", dest="warmup", action="store_false", default=True)
    args = p.parse_args(argv)

    url = args.base_url.rstrip("/") + "/chat/completions"
    metrics_url = args.base_url.rstrip("/").removesuffix("/v1") + "/metrics"

    print(f"preflight {args.base_url}/models ...", flush=True)
    r = requests.get(args.base_url.rstrip("/") + "/models", timeout=30)
    r.raise_for_status()
    served = [m["id"] for m in r.json()["data"]]
    print(f"  serves: {served}")
    if args.model not in served:
        print(f"  WARNING: {args.model} not in served list", file=sys.stderr)

    print(f"building {args.images} frontier images at {args.image_size}px "
          f"~{args.image_kib} KiB ...", flush=True)
    images = [make_frontier_image(i, seed=1000 + i, size=args.image_size,
                                  target_kib=args.image_kib)
              for i in range(args.images)]
    payload = build_request(args.model, images, args.goal, args.num_robots,
                            args.max_tokens, args.temperature)
    body_bytes = len(json.dumps(payload).encode())
    print(f"  images: {[len(b) // 1024 for b in images]} KiB "
          f"(sum {sum(len(b) for b in images) // 1024} KiB)")
    print(f"  request body: {body_bytes / 1024 / 1024:.2f} MB\n", flush=True)

    levels = [int(x) for x in args.levels.split(",") if x.strip()]
    sessions: Dict[str, requests.Session] = {}
    lock = threading.Lock()

    def session_for(cid: str) -> requests.Session:
        with lock:
            if cid not in sessions:
                sessions[cid] = requests.Session()
            return sessions[cid]

    if args.warmup:
        print("warmup round (excluded from the numbers) ...", flush=True)
        w = fire(session_for("warmup"), url, payload, "warmup", args.timeout)
        print(f"  {w.get('latency_s', 0):.2f} s ok={w['ok']} "
              f"{w.get('error', '')}\n", flush=True)

    all_rows: List[Dict[str, Any]] = []
    table: List[Dict[str, Any]] = []

    for level in levels:
        try:
            requests.post(metrics_url + "/reset", timeout=10)
        except Exception:  # noqa: BLE001
            pass
        rows: List[Dict[str, Any]] = []
        makespans: List[float] = []
        print(f"── level {level} concurrent client(s), {args.rounds} round(s) ──",
              flush=True)
        for rnd in range(args.rounds):
            with ThreadPoolExecutor(max_workers=level) as pool:
                t0 = time.perf_counter()
                futs = [pool.submit(fire, session_for(f"drone_{k}"), url, payload,
                                    f"drone_{k}", args.timeout)
                        for k in range(level)]
                res = [f.result() for f in futs]
                makespans.append(time.perf_counter() - t0)
            for x in res:
                x["level"] = level
                x["round"] = rnd
            rows.extend(res)
            bad = [x for x in res if not x["ok"]]
            print(f"  round {rnd}: makespan {makespans[-1]:.2f} s | "
                  f"latencies {[round(x['latency_s'], 2) for x in res]}"
                  + (f" | {len(bad)} FAILED: {bad[0].get('error')}" if bad else ""),
                  flush=True)

        ok = [x for x in rows if x["ok"]]
        all_rows.extend(rows)
        lat = _agg([x["latency_s"] for x in ok])
        qw = _agg([x["queue_wait_s"] for x in ok])
        inf = _agg([x["infer_s"] for x in ok])
        server = {}
        try:
            server = requests.get(metrics_url, timeout=10).json()
        except Exception:  # noqa: BLE001
            pass
        row = {
            "level": level,
            "n": len(ok),
            "failed": len(rows) - len(ok),
            "latency_mean": lat["mean"], "latency_p50": lat["p50"],
            "latency_p95": lat["p95"], "latency_max": lat["max"],
            "queue_mean": qw["mean"], "queue_p95": qw["p95"], "queue_max": qw["max"],
            "infer_mean": inf["mean"],
            "makespan_mean": statistics.fmean(makespans),
            "makespan_max": max(makespans),
            "throughput_req_s": len(ok) / max(sum(makespans), 1e-9),
            "prompt_tokens": statistics.fmean([x["prompt_tokens"] for x in ok]) if ok else 0,
            "image_tokens": statistics.fmean([x["image_tokens"] for x in ok]) if ok else 0,
            "completion_tokens": statistics.fmean([x["completion_tokens"] for x in ok]) if ok else 0,
            "json_ok": sum(1 for x in ok if x["json_ok"]),
            "server_max_concurrency": server.get("max_concurrency_observed"),
            "server_gpu_utilisation": server.get("gpu_utilisation"),
        }
        table.append(row)
        print(f"  → latency mean {row['latency_mean']:.2f} p95 {row['latency_p95']:.2f} s | "
              f"queue mean {row['queue_mean']:.2f} max {row['queue_max']:.2f} s | "
              f"makespan mean {row['makespan_mean']:.2f} s | "
              f"{row['throughput_req_s']:.3f} req/s | "
              f"json_ok {row['json_ok']}/{row['n']}\n", flush=True)

    print("\n" + "=" * 108)
    print(f"{'drones':>6} {'lat_mean':>9} {'lat_p50':>8} {'lat_p95':>8} "
          f"{'queue_mean':>11} {'queue_max':>10} {'infer':>7} {'makespan':>9} "
          f"{'req/s':>7} {'prompt_tok':>11} {'out_tok':>8} {'json':>6}")
    print("-" * 108)
    for row in table:
        print(f"{row['level']:>6} {row['latency_mean']:>9.2f} {row['latency_p50']:>8.2f} "
              f"{row['latency_p95']:>8.2f} {row['queue_mean']:>11.2f} "
              f"{row['queue_max']:>10.2f} {row['infer_mean']:>7.2f} "
              f"{row['makespan_mean']:>9.2f} {row['throughput_req_s']:>7.3f} "
              f"{row['prompt_tokens']:>11.0f} {row['completion_tokens']:>8.0f} "
              f"{row['json_ok']:>3}/{row['n']:<2}")
    print("=" * 108)

    if table:
        base = table[0]["makespan_mean"]
        print(f"\nscaling vs 1 drone (makespan, i.e. time until the LAST drone has "
              f"its assignment):")
        for row in table:
            print(f"  {row['level']} drone(s): {row['makespan_mean']:6.2f} s "
                  f"= {row['makespan_mean'] / max(base, 1e-9):4.2f}x  "
                  f"| fits in round_period_s={args.round_period_s:.0f}? "
                  f"{'YES' if row['makespan_max'] < args.round_period_s else 'NO'} "
                  f"(worst {row['makespan_max']:.2f} s)")

    if args.out:
        with open(args.out, "w") as f:
            for x in all_rows:
                x.pop("content", None)
                f.write(json.dumps(x) + "\n")
            f.write(json.dumps({"summary": table}) + "\n")
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
