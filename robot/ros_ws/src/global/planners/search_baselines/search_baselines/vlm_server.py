#!/usr/bin/env python3
"""OpenAI-compatible chat-completions server for a local Qwen2.5-VL, instrumented
for the "can one VLM serve N drones?" question.

`conavgpt2/vlm_client.py` points the vendored Co-NavGPT2 chat layer at any
OpenAI-compatible `/v1`; upstream's request is plain OpenAI vision (`text` +
`image_url` parts carrying `data:image/jpeg;base64,...`, `response_format`
json_object, `temperature`, `max_tokens`). This serves exactly that shape from
transformers + bitsandbytes, because vLLM/AWQ is not installed in the robot
image and this card is shared with Isaac.

Two things it does that a stock example server does not:

1. **Serialises the GPU.** One model on one card cannot run two forward passes at
   once; letting FastAPI interleave them would corrupt the batch and produce
   nonsense. Decode/preprocess runs in a small pool, then every generate takes a
   single lock, FIFO. Time blocked on that lock is the reported `queue_wait_s`.
2. **Measures.** Per request: queue wait, prefill, decode, image count and bytes,
   real token counts, and which client sent it — to `/metrics` as aggregates and
   to a JSONL file for after-the-fact analysis. The ROS node records the `usage`
   block per round, so the counts here are the real ones from the tokeniser, not
   estimates.

Run:
    /opt/lvlm-venv/bin/python -m search_baselines.vlm_server --port 8000
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import binascii
import io
import json
import logging
import os
import re
import statistics
import threading
import time
import uuid
from collections import defaultdict, deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

# Module scope, not inside build_app: `from __future__ import annotations` turns
# every annotation into a string and FastAPI resolves those against the module
# globals, so a function-local `Request` becomes an unresolvable query parameter.
from fastapi import FastAPI, Header, HTTPException, Request
from fastapi.responses import JSONResponse

LOG = logging.getLogger("vlm_server")

# 14 px patches merged 2x2, so one visual token covers a 28x28 block. Qwen2.5-VL's
# dynamic resolution means max_pixels is really a per-image token budget; keeping
# the arithmetic here is what makes --max-pixels legible in tokens.
PIXELS_PER_VISUAL_TOKEN = 28 * 28

DEFAULT_MODEL = "Qwen/Qwen2.5-VL-3B-Instruct"
# 6 x 480x480 top views is a full CoNavGPT round; 640 tokens/image leaves the
# 480x480 renders untouched (289 tokens each) while capping anything larger.
DEFAULT_MAX_PIXELS = PIXELS_PER_VISUAL_TOKEN * 640
DEFAULT_MIN_PIXELS = PIXELS_PER_VISUAL_TOKEN * 16


# ── request records ──────────────────────────────────────────────────────────

@dataclass
class Record:
    """One request, start to finish. Written to JSONL and aggregated by /metrics."""

    request_id: str
    client: str
    model: str
    t_wall: float
    concurrency_on_arrival: int
    concurrency_peak_during: int = 0
    num_images: int = 0
    image_bytes: int = 0
    image_sizes: List[List[int]] = field(default_factory=list)
    visual_tokens: int = 0
    prompt_tokens: int = 0
    completion_tokens: int = 0
    total_tokens: int = 0
    max_tokens: int = 0
    json_mode: bool = False
    json_extract: str = "n/a"
    finish_reason: str = "stop"
    prep_s: float = 0.0
    queue_wait_s: float = 0.0
    prefill_s: float = 0.0
    decode_s: float = 0.0
    infer_s: float = 0.0
    total_s: float = 0.0
    decode_tok_s: float = 0.0
    error: Optional[str] = None

    def as_dict(self) -> Dict[str, Any]:
        d = self.__dict__.copy()
        for k in ("prep_s", "queue_wait_s", "prefill_s", "decode_s", "infer_s",
                  "total_s", "decode_tok_s"):
            d[k] = round(d[k], 4)
        return d


class Metrics:
    """Lifetime counters plus a bounded window for percentiles."""

    def __init__(self, window: int, jsonl_path: Optional[str]):
        self._lock = threading.Lock()
        self._window: deque = deque(maxlen=window)
        self._per_client: Dict[str, Dict[str, float]] = defaultdict(
            lambda: {"count": 0, "total_s": 0.0, "queue_wait_s": 0.0, "errors": 0})
        self.t_start = time.time()
        self.count = 0
        self.errors = 0
        self.completion_tokens = 0
        self.prompt_tokens = 0
        self.images = 0
        self.inflight = 0
        self.max_concurrency = 0
        self.gpu_busy_s = 0.0
        self._jsonl = None
        if jsonl_path:
            os.makedirs(os.path.dirname(os.path.abspath(jsonl_path)) or ".", exist_ok=True)
            self._jsonl = open(jsonl_path, "a", buffering=1)  # line-buffered: a killed run keeps its records
        self.jsonl_path = jsonl_path

    def arrive(self) -> int:
        with self._lock:
            self.inflight += 1
            self.max_concurrency = max(self.max_concurrency, self.inflight)
            return self.inflight

    def peak_now(self) -> int:
        with self._lock:
            return self.inflight

    def depart(self, rec: Record) -> None:
        with self._lock:
            self.inflight -= 1
            self.count += 1
            if rec.error:
                self.errors += 1
            self.completion_tokens += rec.completion_tokens
            self.prompt_tokens += rec.prompt_tokens
            self.images += rec.num_images
            self.gpu_busy_s += rec.infer_s
            self._window.append(rec)
            c = self._per_client[rec.client]
            c["count"] += 1
            c["total_s"] += rec.total_s
            c["queue_wait_s"] += rec.queue_wait_s
            if rec.error:
                c["errors"] += 1
            if self._jsonl is not None:
                self._jsonl.write(json.dumps(rec.as_dict()) + "\n")

    def reset(self) -> None:
        with self._lock:
            self._window.clear()
            self._per_client.clear()
            self.t_start = time.time()
            self.count = self.errors = 0
            self.completion_tokens = self.prompt_tokens = self.images = 0
            self.max_concurrency = self.inflight
            self.gpu_busy_s = 0.0

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            recs = list(self._window)
            uptime = max(time.time() - self.t_start, 1e-9)
            base = {
                "window_s": round(uptime, 2),
                "count": self.count,
                "errors": self.errors,
                "inflight": self.inflight,
                "max_concurrency_observed": self.max_concurrency,
                "gpu_busy_s": round(self.gpu_busy_s, 3),
                "gpu_utilisation": round(self.gpu_busy_s / uptime, 3),
                "throughput_req_s": round(self.count / uptime, 4),
                "throughput_completion_tok_s": round(self.completion_tokens / uptime, 3),
                "prompt_tokens_total": self.prompt_tokens,
                "completion_tokens_total": self.completion_tokens,
                "per_client": {
                    k: {"count": int(v["count"]),
                        "errors": int(v["errors"]),
                        "mean_total_s": round(v["total_s"] / max(v["count"], 1), 3),
                        "mean_queue_wait_s": round(v["queue_wait_s"] / max(v["count"], 1), 3)}
                    for k, v in self._per_client.items()},
            }
        base["latency_s"] = _stats([r.total_s for r in recs])
        base["queue_wait_s"] = _stats([r.queue_wait_s for r in recs])
        base["infer_s"] = _stats([r.infer_s for r in recs])
        base["prefill_s"] = _stats([r.prefill_s for r in recs])
        base["decode_s"] = _stats([r.decode_s for r in recs])
        base["prep_s"] = _stats([r.prep_s for r in recs])
        base["prompt_tokens"] = _stats([float(r.prompt_tokens) for r in recs])
        base["completion_tokens"] = _stats([float(r.completion_tokens) for r in recs])
        base["visual_tokens"] = _stats([float(r.visual_tokens) for r in recs])
        base["images_per_request"] = _stats([float(r.num_images) for r in recs])
        base["image_bytes"] = _stats([float(r.image_bytes) for r in recs])
        base["decode_tok_s"] = _stats([r.decode_tok_s for r in recs if r.decode_tok_s > 0])
        return base


def _stats(vals: List[float]) -> Dict[str, float]:
    if not vals:
        return {"n": 0}
    s = sorted(vals)
    return {
        "n": len(s),
        "mean": round(statistics.fmean(s), 4),
        "p50": round(_pct(s, 0.50), 4),
        "p95": round(_pct(s, 0.95), 4),
        "min": round(s[0], 4),
        "max": round(s[-1], 4),
    }


def _pct(sorted_vals: List[float], q: float) -> float:
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    idx = q * (len(sorted_vals) - 1)
    lo, hi = int(idx), min(int(idx) + 1, len(sorted_vals) - 1)
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (idx - lo)


# ── message parsing ──────────────────────────────────────────────────────────

_DATA_URL = re.compile(r"^data:(image/[a-zA-Z0-9.+-]+)?;base64,(.*)$", re.DOTALL)


def parse_messages(messages: List[Dict[str, Any]], max_images: int
                   ) -> Tuple[List[Dict[str, Any]], List[Any], int, List[List[int]]]:
    """OpenAI content parts -> (Qwen chat, PIL images, raw image bytes, sizes).

    Only data: URLs are accepted. A server that fetched http image_urls would let
    a prompt reach the network from inside the robot container; the client always
    inlines base64 anyway.
    """
    from PIL import Image

    chat: List[Dict[str, Any]] = []
    images: List[Any] = []
    nbytes = 0
    sizes: List[List[int]] = []

    for msg in messages:
        role = msg.get("role", "user")
        content = msg.get("content")
        if content is None:
            chat.append({"role": role, "content": ""})
            continue
        if isinstance(content, str):
            chat.append({"role": role, "content": content})
            continue

        parts: List[Dict[str, Any]] = []
        for part in content:
            ptype = part.get("type")
            if ptype in ("text", "input_text"):
                parts.append({"type": "text", "text": part.get("text", "")})
            elif ptype in ("image_url", "input_image"):
                url = part.get("image_url", part).get("url") if isinstance(
                    part.get("image_url", part), dict) else part.get("image_url")
                if not isinstance(url, str):
                    raise ValueError("image_url part has no string url")
                m = _DATA_URL.match(url.strip())
                if not m:
                    raise ValueError(
                        "only data:image/...;base64 image_url values are served; "
                        f"got {url[:40]!r}")
                try:
                    raw = base64.b64decode(m.group(2), validate=False)
                except (binascii.Error, ValueError) as exc:
                    raise ValueError(f"undecodable base64 image: {exc}") from exc
                nbytes += len(raw)
                img = Image.open(io.BytesIO(raw))
                img = img.convert("RGB")
                sizes.append([img.width, img.height])
                images.append(img)
                parts.append({"type": "image"})
                if len(images) > max_images:
                    raise ValueError(
                        f"{len(images)} images in one request exceeds --max-images "
                        f"{max_images}; a CoNavGPT round carries at most 6")
            else:
                raise ValueError(f"unsupported content part type {ptype!r}")
        chat.append({"role": role, "content": parts})

    return chat, images, nbytes, sizes


def extract_json_object(text: str) -> Optional[str]:
    """First balanced {...} in the text, string-aware.

    `response_format: json_object` without guided decoding is a request, not a
    guarantee: a 3B model wraps its answer in ```json fences or a sentence often
    enough to matter. Upstream feeds the content straight to ast.literal_eval, so
    the object has to come back bare.
    """
    start = text.find("{")
    if start < 0:
        return None
    depth = 0
    in_str = False
    esc = False
    for i in range(start, len(text)):
        ch = text[i]
        if in_str:
            if esc:
                esc = False
            elif ch == "\\":
                esc = True
            elif ch == '"':
                in_str = False
            continue
        if ch == '"':
            in_str = True
        elif ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
    return None


# ── engine ───────────────────────────────────────────────────────────────────

class Engine:
    """The model plus the one lock that keeps forward passes off each other."""

    def __init__(self, args):
        self.args = args
        self.model = None
        self.processor = None
        self.quant_report: Dict[str, Any] = {}
        self.gpu_lock = asyncio.Lock()
        self.gpu_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vlm-gpu")
        self.prep_pool = ThreadPoolExecutor(max_workers=args.prep_workers,
                                            thread_name_prefix="vlm-prep")
        self.load_s = 0.0
        self.mem_after_load = {}
        self.ready = False

    def load(self):
        import torch
        from transformers import AutoProcessor, BitsAndBytesConfig

        a = self.args
        t0 = time.time()
        dev = a.device
        if torch.cuda.is_available():
            # Bind the context before any memory query: the stats APIs raise
            # "Invalid device argument" against a device that was never selected.
            torch.cuda.set_device(torch.device(dev))
            torch.cuda.init()
            torch.cuda.reset_peak_memory_stats(dev)
            free0, total = torch.cuda.mem_get_info(dev)
        else:
            free0 = total = 0

        compute_dtype = {"float16": torch.float16, "bfloat16": torch.bfloat16}[a.compute_dtype]
        if a.quantization == "nf4":
            quant = BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_quant_type="nf4",
                bnb_4bit_compute_dtype=compute_dtype,
                bnb_4bit_use_double_quant=True)
        elif a.quantization == "int8":
            quant = BitsAndBytesConfig(load_in_8bit=True)
        else:
            quant = None

        LOG.info("loading %s | quantization=%s compute_dtype=%s device=%s",
                 a.model, a.quantization, a.compute_dtype, dev)

        # Qwen2.5-VL needs its own class; AutoModelForImageTextToText resolves it
        # on 4.57 but the explicit import fails loudly on an older transformers.
        from transformers import Qwen2_5_VLForConditionalGeneration as _VLM
        kwargs = dict(dtype=compute_dtype, low_cpu_mem_usage=True,
                      attn_implementation=a.attn_implementation)
        if quant is not None:
            kwargs["quantization_config"] = quant
        # device_map pins the whole model to one card: an accelerate "auto" split
        # would happily offload to CPU when Isaac has the VRAM, turning a 2 s
        # round into a 40 s one with no error to explain it.
        kwargs["device_map"] = {"": dev}
        self.model = _VLM.from_pretrained(a.model, **kwargs).eval()

        self.processor = AutoProcessor.from_pretrained(
            a.model, min_pixels=a.min_pixels, max_pixels=a.max_pixels, use_fast=True)
        # Left padding is what batched/streamed decode expects; we generate one at
        # a time, but a right-padded tokenizer silently degrades if that changes.
        if getattr(self.processor, "tokenizer", None) is not None:
            self.processor.tokenizer.padding_side = "left"

        # Qwen ships sampling defaults in generation_config; leaving them set makes
        # every greedy request log "generation flags are not valid".
        for _k in ("temperature", "top_p", "top_k"):
            setattr(self.model.generation_config, _k, None)

        # Proof the quantisation actually took: a silently-unquantised load still
        # "works", it just eats 3x the VRAM and blames the next process that OOMs.
        n4, n8, nfull, quant_bytes, full_bytes = 0, 0, 0, 0, 0
        for mod in self.model.modules():
            cls = type(mod).__name__
            if cls == "Linear4bit":
                n4 += 1
            elif cls in ("Linear8bitLt", "Int8Params"):
                n8 += 1
            elif cls == "Linear":
                nfull += 1
        for prm in self.model.parameters():
            nb = prm.numel() * prm.element_size()
            if type(prm).__name__ == "Params4bit" or prm.dtype == torch.uint8:
                quant_bytes += nb
            else:
                full_bytes += nb
        qcfg = getattr(self.model.config, "quantization_config", None)
        if isinstance(qcfg, dict):
            method = qcfg.get("quant_method"), qcfg.get("bnb_4bit_quant_type")
        else:
            method = (getattr(qcfg, "quant_method", None),
                      getattr(qcfg, "bnb_4bit_quant_type", None))
        self.quant_report = {
            "linear4bit_modules": n4, "linear8bit_modules": n8,
            "unquantised_linear_modules": nfull,
            "quantised_param_gib": round(quant_bytes / 2**30, 3),
            "unquantised_param_gib": round(full_bytes / 2**30, 3),
            "quant_method": str(method[0]), "quant_type": str(method[1]),
        }

        self.load_s = time.time() - t0
        if torch.cuda.is_available():
            free1, _ = torch.cuda.mem_get_info(dev)
            self.mem_after_load = {
                "torch_allocated_gib": round(torch.cuda.memory_allocated(dev) / 2**30, 3),
                "torch_reserved_gib": round(torch.cuda.memory_reserved(dev) / 2**30, 3),
                "process_attributable_gib": round((free0 - free1) / 2**30, 3),
                "device_free_gib": round(free1 / 2**30, 3),
                "device_total_gib": round(total / 2**30, 3),
            }
        LOG.info("loaded in %.1f s | quantisation: %s", self.load_s,
                 json.dumps(self.quant_report))
        LOG.info("gpu after load: %s", json.dumps(self.mem_after_load))
        self.ready = True

    def gpu_memory(self) -> Dict[str, Any]:
        import torch
        if not torch.cuda.is_available():
            return {}
        free, total = torch.cuda.mem_get_info(self.args.device)
        return {
            "torch_allocated_gib": round(torch.cuda.memory_allocated(self.args.device) / 2**30, 3),
            "torch_reserved_gib": round(torch.cuda.memory_reserved(self.args.device) / 2**30, 3),
            "torch_peak_reserved_gib": round(
                torch.cuda.max_memory_reserved(self.args.device) / 2**30, 3),
            "device_free_gib": round(free / 2**30, 3),
            "device_used_gib": round((total - free) / 2**30, 3),
            "device_total_gib": round(total / 2**30, 3),
            "after_load": self.mem_after_load,
        }

    # -- prep: CPU only, runs concurrently; nothing here touches the GPU --------

    def prepare(self, chat, images, json_prefill: bool):
        import torch

        text = self.processor.apply_chat_template(
            chat, tokenize=False, add_generation_prompt=True)
        if json_prefill:
            # Opening the assistant turn with "{" is the cheapest substitute for
            # guided decoding: it removes the "Sure, here is..." preamble that a
            # 3B model otherwise puts in front of the object.
            text = text + "{"
        inputs = self.processor(text=[text], images=images or None,
                                padding=True, return_tensors="pt")
        visual_tokens = 0
        grid = inputs.get("image_grid_thw")
        if grid is not None:
            merge = getattr(self.processor.image_processor, "merge_size", 2)
            visual_tokens = int(grid.prod(dim=-1).sum().item() // (merge * merge))
        return inputs, visual_tokens, int(inputs["input_ids"].shape[1])

    # -- generate: serialised, one at a time -----------------------------------

    def generate(self, inputs, max_tokens: int, temperature: float,
                 top_p: float, stop: Optional[List[str]]):
        import torch
        from transformers import LogitsProcessor, LogitsProcessorList

        a = self.args
        dev = a.device
        inputs = {k: (v.to(dev) if hasattr(v, "to") else v) for k, v in inputs.items()}
        n_in = int(inputs["input_ids"].shape[1])

        marks: Dict[str, float] = {}

        class _FirstToken(LogitsProcessor):
            """Timestamps the first decode step, which is the end of prefill."""

            def __call__(self, input_ids, scores):
                if "first" not in marks:
                    marks["first"] = time.perf_counter()
                return scores

        do_sample = temperature is not None and temperature > 0.0
        gen_kwargs = dict(
            max_new_tokens=max_tokens,
            do_sample=do_sample,
            logits_processor=LogitsProcessorList([_FirstToken()]),
            pad_token_id=self.processor.tokenizer.pad_token_id
            or self.processor.tokenizer.eos_token_id,
        )
        if do_sample:
            gen_kwargs["temperature"] = float(temperature)
            gen_kwargs["top_p"] = float(top_p if top_p is not None else 1.0)

        torch.cuda.synchronize(dev) if torch.cuda.is_available() else None
        t0 = time.perf_counter()
        with torch.inference_mode():
            out = self.model.generate(**inputs, **gen_kwargs)
        torch.cuda.synchronize(dev) if torch.cuda.is_available() else None
        t1 = time.perf_counter()

        new_ids = out[0][n_in:]
        n_new = int(new_ids.shape[0])
        text = self.processor.tokenizer.decode(new_ids, skip_special_tokens=True)

        finish = "length" if n_new >= max_tokens else "stop"
        if stop:
            for s in stop:
                idx = text.find(s)
                if idx >= 0:
                    text = text[:idx]
                    finish = "stop"
        prefill = marks.get("first", t1) - t0
        return {
            "text": text,
            "n_new": n_new,
            "prefill_s": prefill,
            "decode_s": max(t1 - marks.get("first", t1), 0.0),
            "infer_s": t1 - t0,
            "finish_reason": finish,
        }


# ── app ──────────────────────────────────────────────────────────────────────

def build_app(engine: Engine, metrics: Metrics, args):
    app = FastAPI(title="search_baselines local VLM", version="1.0")

    def _auth(authorization: Optional[str]):
        if not args.api_key:
            return
        want = f"Bearer {args.api_key}"
        if authorization != want:
            raise HTTPException(status_code=401, detail="invalid api key")

    @app.middleware("http")
    async def _limit_body(request: Request, call_next):
        # A 6-frontier round is ~1.1 MB of base64; the cap exists only so a runaway
        # client cannot OOM the box, and it must sit well above that.
        cl = request.headers.get("content-length")
        if cl and cl.isdigit() and int(cl) > args.max_body_mb * 1024 * 1024:
            return JSONResponse(
                status_code=413,
                content={"error": {"message": f"body exceeds --max-body-mb {args.max_body_mb}",
                                   "type": "invalid_request_error"}})
        return await call_next(request)

    @app.get("/health")
    async def health():
        return {"status": "ok" if engine.ready else "loading",
                "model": args.served_model_name,
                "load_s": round(engine.load_s, 2),
                "quantisation": engine.quant_report,
                "gpu": engine.gpu_memory()}

    @app.get("/v1/models")
    async def models():
        # vlm_client.preflight() calls this and raises if the requested model id is
        # absent, so served_model_name must equal the node's vlm_model exactly.
        created = int(engine_start)
        return {"object": "list",
                "data": [{"id": args.served_model_name, "object": "model",
                          "created": created, "owned_by": "local",
                          "permission": []}]}

    @app.get("/metrics")
    async def get_metrics():
        snap = metrics.snapshot()
        snap["model"] = args.served_model_name
        snap["backend"] = {
            "hf_model": args.model,
            "quantization": args.quantization,
            "compute_dtype": args.compute_dtype,
            "min_pixels": args.min_pixels,
            "max_pixels": args.max_pixels,
            "max_visual_tokens_per_image": args.max_pixels // PIXELS_PER_VISUAL_TOKEN,
            "prep_workers": args.prep_workers,
            "load_s": round(engine.load_s, 2),
            "quantisation": engine.quant_report,
        }
        snap["gpu"] = engine.gpu_memory()
        snap["jsonl"] = metrics.jsonl_path
        return snap

    @app.post("/metrics/reset")
    async def reset_metrics():
        metrics.reset()
        return {"status": "reset"}

    @app.post("/v1/chat/completions")
    async def chat_completions(request: Request,
                               authorization: Optional[str] = Header(default=None)):
        _auth(authorization)
        if not engine.ready:
            raise HTTPException(status_code=503, detail="model still loading")

        t_recv = time.perf_counter()
        body = await request.json()
        req_id = "chatcmpl-" + uuid.uuid4().hex[:24]
        client = (request.headers.get("x-client-id")
                  or body.get("user")
                  or (request.client.host if request.client else "unknown"))

        rec = Record(request_id=req_id, client=str(client),
                     model=args.served_model_name, t_wall=time.time(),
                     concurrency_on_arrival=metrics.arrive())
        try:
            messages = body.get("messages")
            if not isinstance(messages, list) or not messages:
                raise ValueError("messages must be a non-empty list")
            if body.get("stream"):
                raise ValueError("streaming is not implemented; set stream=false")

            rfmt = body.get("response_format") or {}
            json_mode = isinstance(rfmt, dict) and rfmt.get("type") in (
                "json_object", "json_schema")
            rec.json_mode = json_mode
            max_tokens = int(body.get("max_tokens") or body.get("max_completion_tokens")
                             or args.default_max_tokens)
            max_tokens = max(1, min(max_tokens, args.max_tokens_cap))
            rec.max_tokens = max_tokens
            temperature = body.get("temperature", 0.0)
            top_p = body.get("top_p", 1.0)
            stop = body.get("stop")
            if isinstance(stop, str):
                stop = [stop]

            loop = asyncio.get_running_loop()
            chat, images, nbytes, sizes = parse_messages(messages, args.max_images)
            rec.num_images = len(images)
            rec.image_bytes = nbytes
            rec.image_sizes = sizes

            prefill_json = json_mode and args.json_prefill
            t_prep0 = time.perf_counter()
            inputs, visual_tokens, n_prompt = await loop.run_in_executor(
                engine.prep_pool, engine.prepare, chat, images, prefill_json)
            rec.prep_s = time.perf_counter() - t_prep0
            rec.visual_tokens = visual_tokens
            rec.prompt_tokens = n_prompt

            t_q0 = time.perf_counter()
            async with engine.gpu_lock:
                rec.queue_wait_s = time.perf_counter() - t_q0
                rec.concurrency_peak_during = metrics.peak_now()
                out = await loop.run_in_executor(
                    engine.gpu_pool, engine.generate, inputs, max_tokens,
                    temperature, top_p, stop)

            text = out["text"]
            if prefill_json:
                text = "{" + text
            rec.completion_tokens = out["n_new"]
            rec.total_tokens = rec.prompt_tokens + rec.completion_tokens
            rec.prefill_s = out["prefill_s"]
            rec.decode_s = out["decode_s"]
            rec.infer_s = out["infer_s"]
            rec.finish_reason = out["finish_reason"]
            rec.decode_tok_s = (out["n_new"] / out["decode_s"]) if out["decode_s"] > 0 else 0.0

            if json_mode:
                obj = extract_json_object(text)
                if obj is not None:
                    rec.json_extract = "ok" if obj.strip() == text.strip() else "extracted"
                    text = obj
                else:
                    rec.json_extract = "failed"

            rec.total_s = time.perf_counter() - t_recv
            payload = {
                "id": req_id,
                "object": "chat.completion",
                "created": int(time.time()),
                "model": args.served_model_name,
                "choices": [{
                    "index": 0,
                    "message": {"role": "assistant", "content": text},
                    "logprobs": None,
                    "finish_reason": rec.finish_reason,
                }],
                "usage": {
                    "prompt_tokens": rec.prompt_tokens,
                    "completion_tokens": rec.completion_tokens,
                    "total_tokens": rec.total_tokens,
                    # Non-standard but honest: how much of the prompt was pixels.
                    "prompt_tokens_details": {
                        "image_tokens": rec.visual_tokens,
                        "text_tokens": rec.prompt_tokens - rec.visual_tokens,
                        "cached_tokens": 0,
                    },
                },
                "x_timings": {
                    "queue_wait_s": round(rec.queue_wait_s, 4),
                    "prep_s": round(rec.prep_s, 4),
                    "prefill_s": round(rec.prefill_s, 4),
                    "decode_s": round(rec.decode_s, 4),
                    "infer_s": round(rec.infer_s, 4),
                    "total_s": round(rec.total_s, 4),
                    "concurrency_on_arrival": rec.concurrency_on_arrival,
                },
            }
            LOG.info("%s | client=%s | %d img %.0f KiB | prompt %d (%d img) + %d tok | "
                     "queue %.2f prep %.2f prefill %.2f decode %.2f total %.2f s | %s",
                     req_id[-8:], rec.client, rec.num_images, rec.image_bytes / 1024,
                     rec.prompt_tokens, rec.visual_tokens, rec.completion_tokens,
                     rec.queue_wait_s, rec.prep_s, rec.prefill_s, rec.decode_s,
                     rec.total_s, rec.json_extract)
            return payload

        except Exception as exc:  # noqa: BLE001 - every failure must still be recorded
            rec.error = f"{type(exc).__name__}: {exc}"
            rec.total_s = time.perf_counter() - t_recv
            LOG.error("%s failed after %.2f s: %s", req_id[-8:], rec.total_s, rec.error)
            status = 400 if isinstance(exc, (ValueError, KeyError)) else 500
            if isinstance(exc, HTTPException):
                status = exc.status_code
            return JSONResponse(
                status_code=status,
                content={"error": {"message": str(exc), "type": "invalid_request_error"
                                   if status == 400 else "server_error"}})
        finally:
            metrics.depart(rec)

    return app


engine_start = time.time()


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="OpenAI-compatible Qwen2.5-VL server for conavgpt2",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--model", default=DEFAULT_MODEL,
                   help="HF repo id or local path to load")
    p.add_argument("--served-model-name", default=None,
                   help="model id reported by /v1/models; must match the node's "
                        "vlm_model or its preflight refuses to start "
                        "(default: --model)")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--device", default="cuda:0")
    p.add_argument("--quantization", default="nf4", choices=["nf4", "int8", "none"],
                   help="nf4 = bitsandbytes 4-bit NF4 with double quant")
    p.add_argument("--compute-dtype", default="float16", choices=["float16", "bfloat16"])
    p.add_argument("--attn-implementation", default="sdpa",
                   choices=["sdpa", "eager", "flash_attention_2"])
    p.add_argument("--min-pixels", type=int, default=DEFAULT_MIN_PIXELS,
                   help=f"per-image floor; {PIXELS_PER_VISUAL_TOKEN} px = 1 visual token")
    p.add_argument("--max-pixels", type=int, default=DEFAULT_MAX_PIXELS,
                   help="per-image ceiling. Qwen2.5-VL's dynamic resolution makes "
                        "this the real token budget: a 6-image round costs up to "
                        "6 x max_pixels/784 visual tokens")
    p.add_argument("--max-images", type=int, default=6,
                   help="reject prompts carrying more; upstream Frontier_Det() caps at 6")
    p.add_argument("--default-max-tokens", type=int, default=128)
    p.add_argument("--max-tokens-cap", type=int, default=1024,
                   help="upper bound on a client's max_tokens, so one request "
                        "cannot monopolise the serialised GPU")
    p.add_argument("--json-prefill", dest="json_prefill", action="store_true", default=True,
                   help="open the assistant turn with '{' when response_format is "
                        "json_object")
    p.add_argument("--no-json-prefill", dest="json_prefill", action="store_false")
    p.add_argument("--prep-workers", type=int, default=4,
                   help="threads for base64/JPEG decode and the image processor. "
                        "CPU only - the GPU stays serialised regardless")
    p.add_argument("--metrics-jsonl", default="/tmp/conavgpt2/vlm_server_requests.jsonl",
                   help="one JSON object appended per request; '' disables")
    p.add_argument("--metrics-window", type=int, default=2000,
                   help="requests retained for percentile stats")
    p.add_argument("--max-body-mb", type=int, default=32)
    p.add_argument("--api-key", default=os.environ.get("VLM_SERVER_API_KEY", ""),
                   help="if set, require 'Authorization: Bearer <key>'")
    p.add_argument("--warmup", dest="warmup", action="store_true", default=True,
                   help="run one throwaway generate at startup so the first real "
                        "request is not paying for CUDA graph/kernel autotune")
    p.add_argument("--no-warmup", dest="warmup", action="store_false")
    p.add_argument("--log-level", default="info")
    args = p.parse_args(argv)
    if args.served_model_name is None:
        args.served_model_name = args.model
    return args


def _warmup(engine: Engine):
    from PIL import Image
    LOG.info("warmup...")
    img = Image.new("RGB", (480, 480), (255, 255, 255))
    chat = [{"role": "user", "content": [{"type": "image"},
                                         {"type": "text", "text": "Reply with {}"}]}]
    t0 = time.time()
    inputs, _, _ = engine.prepare(chat, [img], False)
    engine.generate(inputs, 8, 0.0, 1.0, None)
    LOG.info("warmup done in %.1f s", time.time() - t0)


def main(argv=None):
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s [vlm_server] %(message)s")

    metrics = Metrics(args.metrics_window, args.metrics_jsonl or None)
    engine = Engine(args)
    engine.load()
    if args.warmup:
        _warmup(engine)

    app = build_app(engine, metrics, args)
    import uvicorn
    LOG.info("serving %s on http://%s:%d/v1 | jsonl=%s",
             args.served_model_name, args.host, args.port, args.metrics_jsonl)
    uvicorn.run(app, host=args.host, port=args.port,
                log_level=args.log_level, access_log=False)


if __name__ == "__main__":
    main()
