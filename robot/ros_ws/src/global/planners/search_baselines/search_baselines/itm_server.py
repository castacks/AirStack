"""BLIP-2 ITM as a shared scoring service — the scorer VLFM actually uses.

VLFM (Yokoyama et al., ICRA 2024) scores a view by image-text matching, not by
asking a generative model. That distinction is not cosmetic: ITM returns a
calibrated match probability in one forward pass, where a generative VLM has to
emit tokens and be parsed. Measured on this box, same image, same target:

    Qwen2.5-VL-7B nf4, "give me a score 0-10"   ~2500 ms    0.4 Hz   5.93 GB
    BLIP-2 ITM (Salesforce/blip2-itm-vit-g)       17.4 ms   57.5 Hz   2.50 GB

143x faster and smaller, which is what makes ONE scorer enough for a fleet
rather than one per robot. Batched it reaches ~124 img/s (8.0 ms/image at
batch 8), so the practical ceiling at a 5 Hz per-robot keyframe rate is roughly
a dozen robots unbatched and a couple of dozen batched.

Stateless by construction — VLFM scores single frames with no history — so
requests from different robots can be interleaved freely with no cross-talk.
That is the property that makes sharing safe, and it is worth stating because it
is NOT true of the generative endpoint conavgpt2 uses, where a shared server
still serialises on one GPU lock.

The ITM head is the right one. `use_image_text_matching_head=True` gives a
2-logit match/no-match head whose softmax is a probability; the ITC (cosine)
head is a retrieval similarity that is not calibrated the same way and needs its
own scaling before it can be painted into a value map.
"""

import argparse
import base64
import io
import json
import logging
import threading
import time
from typing import List, Optional

import torch
import uvicorn
from fastapi import FastAPI, HTTPException
from PIL import Image
from pydantic import BaseModel
from transformers import AutoProcessor, Blip2ForImageTextRetrieval

DEFAULT_MODEL = "Salesforce/blip2-itm-vit-g"
# VLFM's own prompt shape: a statement to be matched against, not a question.
DEFAULT_PROMPT = "Seems like there is a {target} ahead."

log = logging.getLogger("itm_server")


class ScoreRequest(BaseModel):
    images: List[str]                 # base64 JPEG/PNG, no data: prefix needed
    text: Optional[str] = None        # full prompt; wins over `target`
    target: Optional[str] = None      # filled into the prompt template


class Engine:
    def __init__(self, model_id, device, dtype, prompt_template):
        self.model_id = model_id
        self.device = device
        self.prompt_template = prompt_template
        self.dtype = torch.float16 if dtype == "float16" else torch.bfloat16
        self.processor = None
        self.model = None
        self.load_s = 0.0
        # One GPU, one forward at a time. Held briefly (~17 ms), unlike the
        # generative endpoint where it is held for seconds.
        self._lock = threading.Lock()
        self.n_calls = 0
        self.total_s = 0.0

    def load(self):
        t0 = time.time()
        self.processor = AutoProcessor.from_pretrained(self.model_id)
        self.model = Blip2ForImageTextRetrieval.from_pretrained(
            self.model_id, dtype=self.dtype).to(self.device).eval()
        self.load_s = time.time() - t0
        log.info("loaded %s in %.1f s | %.2f GiB",
                 self.model_id, self.load_s,
                 torch.cuda.memory_allocated() / 2 ** 30
                 if self.device.startswith("cuda") else 0.0)

    def score(self, images, text):
        with self._lock:
            t0 = time.time()
            inp = self.processor(images=images, text=[text] * len(images),
                                 return_tensors="pt").to(self.device, self.dtype)
            with torch.no_grad():
                out = self.model(**inp, use_image_text_matching_head=True)
            # logits are [no-match, match]; softmax[:, 1] is P(match) in [0, 1],
            # directly paintable into a value map with no rescaling.
            probs = torch.softmax(out.logits_per_image.float(), dim=1)[:, 1]
            dt = time.time() - t0
            self.n_calls += 1
            self.total_s += dt
            return [float(p) for p in probs.cpu()], dt


def _decode(b64):
    if "," in b64[:64] and b64.lstrip().startswith("data:"):
        b64 = b64.split(",", 1)[1]
    return Image.open(io.BytesIO(base64.b64decode(b64))).convert("RGB")


def build_app(engine, args):
    app = FastAPI(title="blip2-itm")

    @app.get("/health")
    def health():
        return {
            "status": "ok",
            "model": engine.model_id,
            "load_s": round(engine.load_s, 2),
            "device": engine.device,
            "prompt_template": engine.prompt_template,
            "gpu_gib": (round(torch.cuda.memory_allocated() / 2 ** 30, 3)
                        if engine.device.startswith("cuda") else None),
        }

    @app.get("/metrics")
    def metrics():
        n = engine.n_calls
        return {
            "calls": n,
            "total_s": round(engine.total_s, 3),
            "mean_ms": round(1000.0 * engine.total_s / n, 2) if n else None,
            "throughput_hz": round(n / engine.total_s, 1) if engine.total_s else None,
        }

    @app.post("/score")
    def score(req: ScoreRequest):
        if not req.images:
            raise HTTPException(400, "no images")
        if len(req.images) > args.max_batch:
            raise HTTPException(400, f"batch > {args.max_batch}")
        text = req.text or engine.prompt_template.format(
            target=(req.target or "object"))
        try:
            imgs = [_decode(b) for b in req.images]
        except Exception as exc:
            raise HTTPException(400, f"image decode failed: {exc}")
        scores, dt = engine.score(imgs, text)
        rec = {"scores": scores, "text": text, "server_s": round(dt, 4),
               "batch": len(imgs)}
        if args.metrics_jsonl:
            try:
                with open(args.metrics_jsonl, "a") as fh:
                    fh.write(json.dumps({"t": time.time(), **rec}) + "\n")
            except OSError:
                pass
        return rec

    return app


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--model", default=DEFAULT_MODEL)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8100)
    p.add_argument("--device", default="cuda:0")
    p.add_argument("--dtype", default="float16", choices=["float16", "bfloat16"])
    p.add_argument("--prompt-template", default=DEFAULT_PROMPT)
    p.add_argument("--max-batch", type=int, default=16)
    p.add_argument("--metrics-jsonl", default="")
    p.add_argument("--log-level", default="info")
    args = p.parse_args(argv)

    logging.basicConfig(level=args.log_level.upper(),
                        format="%(asctime)s %(levelname)s [blip2_itm] %(message)s")
    engine = Engine(args.model, args.device, args.dtype, args.prompt_template)
    # Load BEFORE binding the port, so a port that answers means "ready" and a
    # readiness gate cannot race the weight load.
    engine.load()
    engine.score([Image.new("RGB", (224, 224))], "warmup")
    log.info("serving %s on http://%s:%d", args.model, args.host, args.port)
    uvicorn.run(build_app(engine, args), host=args.host, port=args.port,
                log_level=args.log_level)


if __name__ == "__main__":
    main()
