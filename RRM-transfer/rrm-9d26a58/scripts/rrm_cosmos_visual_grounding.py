#!/usr/bin/env python3
"""Run local Cosmos visual grounding into C02 evidence; never plans or dispatches."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from rrm.state_contracts import StateSnapshot
from rrm.visual_world_builder import (
    MediaArtifact, VisualGroundingInput, parse_visual_candidate, render_visual_prompt,
    score_visual_snapshot,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _generate(*, model_path: str, prompt: str, image_path: str | None,
              video_path: str | None, max_new_tokens: int) -> str:
    import torch
    import transformers

    if bool(image_path) == bool(video_path):
        raise ValueError("provide exactly one image or video")
    media = [{"type": "image", "image": image_path}] if image_path else [
        {"type": "video", "video": video_path},
    ]
    conversation = [
        {"role": "system", "content": [{"type": "text", "text": "Return only requested JSON."}]},
        {"role": "user", "content": [*media, {"type": "text", "text": prompt}]},
    ]
    model = transformers.Qwen3VLForConditionalGeneration.from_pretrained(
        model_path, dtype=torch.bfloat16, device_map="auto", attn_implementation="sdpa",
    )
    processor = transformers.Qwen3VLProcessor.from_pretrained(model_path)
    inputs = processor.apply_chat_template(conversation, tokenize=True, add_generation_prompt=True,
                                           return_dict=True, return_tensors="pt", fps=4).to(model.device)
    generated = model.generate(**inputs, max_new_tokens=max_new_tokens, do_sample=False)
    trimmed = [out[len(source):] for source, out in zip(inputs.input_ids, generated, strict=False)]
    return processor.batch_decode(trimmed, skip_special_tokens=True,
                                  clean_up_tokenization_spaces=False)[0]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    media_group = parser.add_mutually_exclusive_group(required=True)
    media_group.add_argument("--image", type=Path)
    media_group.add_argument("--video", type=Path)
    parser.add_argument("--media-ref", required=True, help="durable dataset/scene capture identifier")
    parser.add_argument("--catalog", type=Path, required=True, help="JSON object: entity ID -> kind")
    parser.add_argument("--task-id", required=True)
    parser.add_argument("--episode-id", required=True)
    parser.add_argument("--state-revision", required=True)
    parser.add_argument("--observed-monotonic-s", type=float, required=True)
    parser.add_argument("--received-monotonic-s", type=float, required=True)
    parser.add_argument("--max-age-s", type=float, default=1.0)
    parser.add_argument("--model-ref", default="nvidia/Cosmos-Reason2-8B")
    parser.add_argument("--model-path", required=True)
    parser.add_argument("--teacher", type=Path, help="optional frozen C02 ground-truth snapshot JSON")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-new-tokens", type=int, default=768)
    args = parser.parse_args()
    if args.max_new_tokens <= 0:
        raise SystemExit("--max-new-tokens must be positive")
    media_path = args.image or args.video
    if not media_path.is_file():
        raise SystemExit(f"media file does not exist: {media_path}")
    catalog = json.loads(args.catalog.read_text(encoding="utf-8"))
    if not isinstance(catalog, dict) or not all(isinstance(key, str) and isinstance(value, str)
                                                for key, value in catalog.items()):
        raise SystemExit("--catalog must be a JSON object mapping entity IDs to kinds")
    context = VisualGroundingInput(
        task_id=args.task_id, episode_id=args.episode_id, state_revision=args.state_revision,
        entity_catalog=catalog,
        media=MediaArtifact(source_ref=args.media_ref, sha256=_sha256(media_path),
                            observed_monotonic_s=args.observed_monotonic_s),
        received_monotonic_s=args.received_monotonic_s, max_age_s=args.max_age_s,
        model_ref=args.model_ref,
    )
    prompt = render_visual_prompt(context)
    raw = _generate(model_path=args.model_path, prompt=prompt,
                    image_path=str(args.image) if args.image else None,
                    video_path=str(args.video) if args.video else None,
                    max_new_tokens=args.max_new_tokens)
    candidate = parse_visual_candidate(raw, context)
    score = None
    if args.teacher and candidate.snapshot is not None:
        teacher = StateSnapshot.model_validate(json.loads(args.teacher.read_text(encoding="utf-8")))
        score = score_visual_snapshot(candidate.snapshot, teacher,
                                      now_monotonic_s=args.received_monotonic_s,
                                      entity_catalog=catalog)
    record = {
        "media_path": str(media_path), "media_ref": args.media_ref,
        "media_sha256": context.media.sha256, "model_path": args.model_path,
        "model_ref": args.model_ref, "prompt": prompt, "raw_response": raw,
        "candidate": candidate.model_dump(mode="json"),
        "teacher_score": None if score is None else score.model_dump(mode="json"),
        "planning_or_execution_dispatch": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"candidate_status={candidate.status.value} output={args.output} "
          "planning_or_execution_dispatch=false")


if __name__ == "__main__":
    main()
