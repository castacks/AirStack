#!/usr/bin/env python3
"""Run local Cosmos Reason2 inference for RRM cognition; never dispatches a robot."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

from rrm.contracts import CapabilityDeclaration
from rrm.cosmos_reason2 import CosmosReasoningInput, parse_cosmos_candidate, render_cosmos_prompt
from rrm.state_contracts import StateSnapshot
from rrm.task_contracts import TaskRequest


def load_context(path: Path) -> CosmosReasoningInput:
    payload = json.loads(path.read_text(encoding="utf-8"))
    capability = CapabilityDeclaration(
        embodiment_id=payload["capabilities"]["embodiment_id"],
        revision=payload["capabilities"]["revision"],
        operations=frozenset(payload["capabilities"]["operations"]),
        resources=frozenset(payload["capabilities"]["resources"]),
        available_resources=frozenset(payload["capabilities"]["available_resources"]),
        limits_ref=payload["capabilities"]["limits_ref"],
    )
    return CosmosReasoningInput(
        task=TaskRequest.model_validate(payload["task"]),
        snapshot=StateSnapshot.model_validate(payload["snapshot"]),
        capabilities=capability,
        now_monotonic_s=payload["now_monotonic_s"],
    )


def generate(*, model_path: str, prompt: str, image_path: str | None,
             video_path: str | None, max_new_tokens: int) -> str:
    """Lazy heavyweight import: core contracts remain CPU/unit-testable."""
    import torch
    import transformers

    if image_path and video_path:
        raise ValueError("provide at most one image or video")
    media: list[dict[str, str]] = []
    if image_path:
        media.append({"type": "image", "image": image_path})
    if video_path:
        media.append({"type": "video", "video": video_path})
    conversation = [
        {"role": "system", "content": [{"type": "text", "text": "Follow the requested JSON schema."}]},
        {"role": "user", "content": [*media, {"type": "text", "text": prompt}]},
    ]
    processor = transformers.Qwen3VLProcessor.from_pretrained(model_path)
    model = transformers.Qwen3VLForConditionalGeneration.from_pretrained(
        model_path, dtype=torch.bfloat16, device_map="auto", attn_implementation="sdpa",
    )
    inputs = processor.apply_chat_template(
        conversation, tokenize=True, add_generation_prompt=True, return_dict=True,
        return_tensors="pt", fps=4,
    ).to(model.device)
    generated = model.generate(**inputs, max_new_tokens=max_new_tokens, do_sample=False)
    trimmed = [out[len(source):] for source, out in zip(inputs.input_ids, generated, strict=False)]
    return processor.batch_decode(trimmed, skip_special_tokens=True,
                                  clean_up_tokenization_spaces=False)[0]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="C01/C02/C03 JSON bundle")
    parser.add_argument("--model-path", required=True, help="local Cosmos Reason2 snapshot")
    parser.add_argument("--output", type=Path, required=True, help="replayable result JSON")
    parser.add_argument("--image", help="optional scene image")
    parser.add_argument("--video", help="optional scene video")
    parser.add_argument("--max-new-tokens", type=int, default=768)
    args = parser.parse_args()
    if args.max_new_tokens <= 0:
        raise SystemExit("--max-new-tokens must be positive")
    if args.output.exists():
        raise SystemExit("output already exists; choose a new run path to preserve evidence")
    context = load_context(args.input)
    prompt = render_cosmos_prompt(context)
    started = time.monotonic()
    raw = generate(model_path=args.model_path, prompt=prompt, image_path=args.image,
                   video_path=args.video, max_new_tokens=args.max_new_tokens)
    candidate = parse_cosmos_candidate(raw, context)
    record = {
        "model_path": args.model_path,
        "input": str(args.input),
        "image": args.image,
        "video": args.video,
        "prompt": prompt,
        "raw_response": raw,
        "candidate": candidate.model_dump(mode="json"),
        "execution_dispatch": False,
        "input_sha256": hashlib.sha256(args.input.read_bytes()).hexdigest(),
        "media_sha256": hashlib.sha256(Path(args.image or args.video).read_bytes()).hexdigest()
            if args.image or args.video else None,
        "inference_wall_s": time.monotonic() - started,
        "generation": {"max_new_tokens": args.max_new_tokens, "do_sample": False},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"candidate_status={candidate.status.value} output={args.output} execution_dispatch=false")


if __name__ == "__main__":
    main()
