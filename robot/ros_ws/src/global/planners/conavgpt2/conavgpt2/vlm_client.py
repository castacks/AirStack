"""OpenAI-compatible VLM backend wiring for the vendored Co-NavGPT2 chat layer.

Upstream calls `client.chat.completions.create(...)` against api.openai.com. Any
server that speaks the OpenAI chat-completions API with `image_url` content parts
(vLLM, llama.cpp, Ollama, SGLang) is a drop-in: point `base_url` at it and pass a
placeholder key. Nothing about the request shape changes.

This module owns (a) the parameters, (b) the preflight that fails loudly before
the first mission tick rather than silently degrading to "frontier_0" 20 minutes
in, and (c) writing the settings into the vendored module's CONFIG.
"""

import os

from conavgpt2.vendor.utils import chat_utils

# Defaults assume a locally served open-weight VLM, not OpenAI. Qwen2.5-VL is the
# closest open comparable to the GPT-4o upstream used for this task: read a
# rendered top-down map with numbered frontiers, answer in strict JSON. 3B is the
# default only because the local smoke test shares one 16 GB card with Isaac;
# 7B is the intended size once this moves to the 2x48 GB OSMO box.
DEFAULT_BASE_URL = "http://localhost:8000/v1"
DEFAULT_MODEL = "Qwen/Qwen2.5-VL-3B-Instruct"
DEFAULT_API_KEY = "EMPTY"


def resolve(base_url=None, model=None, api_key=None):
    """ROS parameter < environment < built-in default.

    The env names are the ones the openai SDK itself honours, so an operator who
    exports OPENAI_BASE_URL for other tooling gets the same endpoint here.
    """
    base_url = base_url or os.environ.get("OPENAI_BASE_URL") or DEFAULT_BASE_URL
    model = model or os.environ.get("CONAVGPT2_VLM_MODEL") or DEFAULT_MODEL
    api_key = api_key or os.environ.get("OPENAI_API_KEY") or DEFAULT_API_KEY
    return base_url, model, api_key


def configure(base_url, model, api_key, num_agents, timeout=60.0):
    """Install the backend settings into the vendored chat_utils module."""
    chat_utils.CONFIG.base_url = base_url
    chat_utils.CONFIG.model = model
    chat_utils.CONFIG.api_key = api_key
    chat_utils.CONFIG.num_agents = int(num_agents)
    chat_utils.CONFIG.timeout = float(timeout)
    chat_utils.reset_client()


def preflight(logger=None, timeout=10.0):
    """Reach the endpoint once at startup and raise with an actionable message.

    A wrong base_url otherwise shows up only as the fallback assignment inside
    chat_with_gpt4v, which looks like a bad VLM answer rather than a dead server.
    """
    cfg = chat_utils.CONFIG
    where = f"base_url={cfg.base_url} model={cfg.model}"
    try:
        client = chat_utils.get_client()
        models = [m.id for m in client.models.list(timeout=timeout).data]
    except Exception as exc:
        raise RuntimeError(
            f"conavgpt2: VLM endpoint unreachable ({where}): {exc}\n"
            "Start an OpenAI-compatible server first, e.g.\n"
            f"    vllm serve {cfg.model} --port 8000\n"
            "then set the vlm_base_url parameter (or OPENAI_BASE_URL) to its /v1 URL."
        ) from exc

    if models and cfg.model not in models:
        raise RuntimeError(
            f"conavgpt2: VLM endpoint is up but does not serve '{cfg.model}' ({where}).\n"
            f"Models it does serve: {models}\n"
            "Set the vlm_model parameter to one of those, or serve the expected model."
        )
    if logger is not None:
        logger.info(f"conavgpt2: VLM endpoint OK ({where})")
    return models
