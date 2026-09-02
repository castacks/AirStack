"""Minimal OpenAI-compatible VLM client for the LVLM-guided behaviour.

stdlib only (`urllib`) so the robot image needs nothing new; the JPEG encode
uses OpenCV, which the robot image already has (`cv2` ships with the ROS
desktop image and with `search_baselines`). The request runs on a worker
thread — `RavenNavNode`'s tick must never block on a model.

Endpoint contract (matches `search_baselines/vlm_server.py`, i.e. what
`offboard-compute` serves):
    GET  {base_url}/models             -> {"data": [{"id": ...}, ...]}
    POST {base_url}/chat/completions   -> {"choices":[{"message":{"content":...}}]}
with the image passed as a `data:image/jpeg;base64,...` `image_url` part —
that server explicitly refuses remote URLs.
"""
from __future__ import annotations

import base64
import json
import os
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import List, Optional

# Precedence for the endpoint: explicit param, then VLM_URL, then
# OPENAI_BASE_URL (what search_baselines/vlm_client.py reads), then the
# offboard-compute default.
DEFAULT_BASE_URL = 'http://offboard-compute:8000/v1'
DEFAULT_API_KEY = 'EMPTY'


def resolve_base_url(explicit: str = '') -> str:
    for cand in (explicit,
                 os.getenv('VLM_URL', ''),
                 os.getenv('OPENAI_BASE_URL', ''),
                 DEFAULT_BASE_URL):
        if cand:
            return str(cand).rstrip('/')
    return DEFAULT_BASE_URL


def resolve_api_key() -> str:
    return os.getenv('OPENAI_API_KEY', '') or DEFAULT_API_KEY


def resolve_model(explicit: str = '') -> str:
    return str(explicit or os.getenv('CONAVGPT2_VLM_MODEL', '') or '')


def list_models(base_url: str, timeout: float = 10.0) -> List[str]:
    """GET {base_url}/models. Raises urllib errors when unreachable."""
    req = urllib.request.Request(
        f'{base_url.rstrip("/")}/models',
        headers={'Authorization': f'Bearer {resolve_api_key()}'})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        payload = json.loads(resp.read().decode('utf-8'))
    data = payload.get('data') or []
    return [str(m.get('id', '')) for m in data if m.get('id')]


def encode_jpeg(image_bgr, quality: int = 85) -> bytes:
    """BGR uint8 ndarray -> JPEG bytes (OpenCV; imported lazily)."""
    import cv2  # noqa: PLC0415  (optional dependency, only on the request path)
    ok, buf = cv2.imencode('.jpg', image_bgr,
                           [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError('cv2.imencode failed')
    return bytes(buf.tobytes())


def build_chat_payload(model: str, prompt: str,
                       jpeg_bytes: Optional[bytes],
                       max_tokens: int = 64,
                       temperature: float = 0.0) -> dict:
    content: List[dict] = []
    if jpeg_bytes:
        b64 = base64.b64encode(jpeg_bytes).decode('ascii')
        content.append({'type': 'image_url',
                        'image_url': {'url': f'data:image/jpeg;base64,{b64}'}})
    content.append({'type': 'text', 'text': prompt})
    return {
        'model': model,
        'messages': [{'role': 'user', 'content': content}],
        'max_tokens': int(max_tokens),
        'temperature': float(temperature),
    }


def parse_answer(payload: dict) -> str:
    choices = payload.get('choices') or []
    if not choices:
        return ''
    msg = choices[0].get('message') or {}
    content = msg.get('content')
    if isinstance(content, list):
        return ' '.join(str(c.get('text', '')) for c in content).strip()
    return str(content or '').strip()


@dataclass
class VlmResult:
    ok: bool
    answer: str = ''
    error: str = ''
    latency_s: float = 0.0
    prompt: str = ''
    model: str = ''
    ts: float = 0.0


class VlmClient:
    """Blocking client. Use `AsyncVlmClient` from the node."""

    def __init__(self, base_url: str = '', model: str = '',
                 timeout_s: float = 60.0) -> None:
        self.base_url = resolve_base_url(base_url)
        self.model = resolve_model(model)
        self.timeout_s = float(timeout_s)

    def preflight(self, timeout: float = 10.0) -> List[str]:
        """GET /models; also fills in `self.model` from the first served model
        when it was not configured. Raises on an unreachable endpoint."""
        models = list_models(self.base_url, timeout=timeout)
        if not self.model and models:
            self.model = models[0]
        return models

    def chat(self, prompt: str, jpeg_bytes: Optional[bytes] = None) -> VlmResult:
        started = time.time()
        body = json.dumps(build_chat_payload(self.model, prompt, jpeg_bytes))
        req = urllib.request.Request(
            f'{self.base_url}/chat/completions',
            data=body.encode('utf-8'),
            headers={'Content-Type': 'application/json',
                     'Authorization': f'Bearer {resolve_api_key()}'},
            method='POST')
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                payload = json.loads(resp.read().decode('utf-8'))
        except Exception as exc:                      # noqa: BLE001
            return VlmResult(ok=False, error=f'{type(exc).__name__}: {exc}',
                             latency_s=time.time() - started, prompt=prompt,
                             model=self.model, ts=time.time())
        return VlmResult(ok=True, answer=parse_answer(payload),
                         latency_s=time.time() - started, prompt=prompt,
                         model=self.model, ts=time.time())


class AsyncVlmClient:
    """One in-flight request at a time, on a daemon thread.

    `submit` is a no-op while a request is outstanding; `poll` returns the
    finished `VlmResult` exactly once.
    """

    def __init__(self, client: VlmClient) -> None:
        self.client = client
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._result: Optional[VlmResult] = None

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def submit(self, prompt: str, jpeg_bytes: Optional[bytes]) -> bool:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return False
            self._result = None

            def _run():
                res = self.client.chat(prompt, jpeg_bytes)
                with self._lock:
                    self._result = res

            self._thread = threading.Thread(target=_run, daemon=True,
                                            name='raven-vlm')
            self._thread.start()
            return True

    def poll(self) -> Optional[VlmResult]:
        with self._lock:
            res, self._result = self._result, None
            return res
