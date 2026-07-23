"""Qwen3 client for llm_nav: model hosting, prompts, constrained-JSON parsing.

Runs inside /opt/lvlm-venv (transformers + bitsandbytes on system torch),
following the lvlm_baseline hosting pattern: 8-bit quantized load on a
background thread so the ROS node stays responsive, blocking generation only
ever called from the planner's worker thread.

Every call is logged in full (prompt, raw response, parse outcome, latency)
to the MissionLog.
"""

import json
import re
import time

DECIDE_SYSTEM = (
    'You are the navigation brain of a search drone. A text digest of the '
    "drone's semantic map is provided: mapped object instances (V#) and ray "
    'leads (R#) pointing at things seen from afar. Your job is to find the '
    'TARGET. You reply with ONLY a JSON object — no prose, no markdown.')

DECIDE_INSTRUCTIONS = (
    'Choose what the drone should do next to find the target "{target}".\n'
    'Valid choices — you MUST pick exactly one:\n'
    '- "goto_instance" (fly to a mapped object), valid ids: {instance_ids}\n'
    '- "goto_ray" (follow a ray lead to extend the map that way), valid ids: '
    '{ray_ids}\n'
    '- "goto_frontier" (fly to unexplored area), valid ids (compass '
    'directions): {frontier_ids}\n'
    '- "survey" (climb high and circle in place to scan the horizon for new '
    'leads), valid: {survey_ok} — use when nothing above looks promising\n'
    '- "retune" (adjust the voxel clustering parameters), valid: {retune_ok} '
    '— use ONLY if the object list looks wrong: many tiny fragments that are '
    'probably one {target}, or one implausibly huge blob spanning several\n'
    'Ids marked VISITED or BLOCKED are already done and are NOT valid choices. '
    'Only visit an object if it might BE a {target} — do not visit fences, '
    'poles or other clutter; if no unvisited {target} candidate exists, '
    'explore instead (ray lead, frontier, or survey).\n'
    'Mapped objects are partial views that grow as the drone sees more angles '
    '— small fragments near a strong candidate are usually part of the same '
    'object, so prefer the strongest/largest candidate over its fragments.\n'
    'Reply with ONLY this JSON, action FIRST:\n'
    '{{"action": {{"type": "goto_instance"|"goto_ray"|"goto_frontier"|"survey"'
    '|"retune", "id": "<V#, R#, compass direction, or empty>", '
    '"reason": "<short why>"}}, '
    '"seeing": "<ONE short sentence: what the map shows>"}}')

PERCEPTION_SYSTEM = (
    'You tune the perception layer of a drone that clusters 0.5m semantic '
    'voxels into object instances. Reply with ONLY a JSON object — no prose, '
    'no markdown.')

PERCEPTION_INSTRUCTIONS = (
    'Target object: "{target}".\n'
    '{context}'
    'Set clustering parameters appropriate for the physical nature of a '
    '{target}:\n'
    '- "min_voxels": smallest voxel cluster worth listing as a candidate '
    'instance (noise floor). Think about how many 0.5m voxels a partial view '
    'of a {target} occupies — a car door ~10, a house wall ~100s.\n'
    '- "score_floor": minimum similarity for a voxel to count as an object '
    'at all (raw cos-sim). Below ~0.08 background clutter usually scores in '
    'and BRIDGES separate objects into one giant blob; 0.08-0.15 works for '
    'most targets, higher only for very distinctive ones.\n'
    '- "merge_gap_m": two same-label voxel patches closer than this are ONE '
    'object (bridges occlusion splits, e.g. a house cut in two by a tree in '
    'front; too large merges neighboring {target}s together!).\n'
    'Reply with ONLY this JSON: {{"min_voxels": <int>, "score_floor": '
    '<float>, "merge_gap_m": <float>, "reason": "<short>"}}')

ALIASES_SYSTEM = (
    'You classify object part-whole relations for a robot mapping system. '
    'Reply with ONLY a JSON object — no prose, no markdown.')

ALIASES_INSTRUCTIONS = (
    'Target object: "{target}".\n'
    'Label list: {labels}\n'
    'Which labels from the list name PARTS of a {target} — components that are '
    'physically attached to / built into a {target} (e.g. "roof" is part of '
    '"house")? Nearby-but-separate objects (e.g. "fence", "driveway") are NOT '
    'parts.\n'
    'Reply with ONLY this JSON: {{"parts": ["<label>", ...]}} '
    '(empty list if none).')

NARRATE_SYSTEM = (
    'You are the narration voice of a search drone. A text digest of the '
    "drone's semantic map is provided. You reply with ONLY a JSON object — "
    'no prose, no markdown.')

NARRATE_INSTRUCTIONS = (
    'The drone is flying to its committed goal and cannot change goals right '
    'now. Briefly describe what the semantic map currently shows that is '
    'relevant to finding "{target}".\n'
    'Reply with ONLY this JSON:\n'
    '{{"seeing": "<1-2 short sentences>", "notes": "<anything noteworthy for later, '
    'or empty string>"}}')


def extract_json(text: str) -> dict:
    """Parse the first complete JSON object in text (brace matching)."""
    # Qwen3 may wrap output in ```json fences or emit <think> blocks even with
    # thinking disabled in odd cases — strip both defensively.
    text = re.sub(r'<think>.*?</think>', '', text, flags=re.DOTALL)
    text = text.replace('```json', '').replace('```', '')
    start = text.find('{')
    if start < 0:
        raise ValueError('no JSON object in response')
    depth = 0
    in_str = False
    esc = False
    for i in range(start, len(text)):
        c = text[i]
        if in_str:
            if esc:
                esc = False
            elif c == '\\':
                esc = True
            elif c == '"':
                in_str = False
            continue
        if c == '"':
            in_str = True
        elif c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                return json.loads(text[start:i + 1])
    raise ValueError('unterminated JSON object in response')


class LLMClient:

    def __init__(self, model_path: str, ros_logger, mission_log,
                 enable_thinking: bool = False, max_new_tokens: int = 384,
                 quantization: str = '4bit'):
        self._model_path = model_path
        self._log = ros_logger
        self._mlog = mission_log
        self._enable_thinking = enable_thinking
        self._max_new_tokens = max_new_tokens
        self._quantization = quantization  # '4bit' | '8bit' | 'none'
        self.model = None
        self.tokenizer = None
        self.ready = False

    def load(self):
        """Blocking model load — call from a background thread."""
        self._mlog.event('model_load_start', model=self._model_path)
        try:
            import torch
            from transformers import (AutoModelForCausalLM, AutoTokenizer,
                                      BitsAndBytesConfig)
            try:
                import bitsandbytes as _bnb  # noqa: F401
                import accelerate as _acc  # noqa: F401
                deps = f'bitsandbytes={_bnb.__version__} accelerate={_acc.__version__}'
            except Exception as de:
                deps = f'MISSING ({de}) — rebuild/push the robot-desktop image'
            self._log.info(
                f'Loading Qwen3 model ({self._model_path}) | deps: {deps} | '
                f'cuda_available={torch.cuda.is_available()}...')
            t0 = time.monotonic()
            self.tokenizer = AutoTokenizer.from_pretrained(self._model_path)
            # VRAM is shared with Isaac Sim (~7 GB) + rayfronts (~5.6 GB) on
            # single-GPU sim hosts, leaving ~2.5 GB: bf16 (~3.5 GB) OOMs there,
            # so default to nf4 (~1.8 GB incl. activations; also much faster
            # than bnb int8). 'none' (bf16) is best on a dedicated GPU.
            if self._quantization == '4bit':
                quant = dict(quantization_config=BitsAndBytesConfig(
                    load_in_4bit=True,
                    bnb_4bit_quant_type='nf4',
                    bnb_4bit_compute_dtype=torch.bfloat16))
            elif self._quantization == '8bit':
                quant = dict(quantization_config=BitsAndBytesConfig(load_in_8bit=True))
            else:
                quant = {}
            self.model = AutoModelForCausalLM.from_pretrained(
                self._model_path,
                torch_dtype=torch.bfloat16,
                low_cpu_mem_usage=True,
                device_map='auto', **quant).eval()
            self.ready = True
            dt = time.monotonic() - t0
            self._log.info(f'Qwen3 model loaded! ({dt:.1f}s)')
            self._mlog.event('model_load_done', seconds=round(dt, 1))
        except Exception:
            import traceback
            tb = traceback.format_exc()
            self._log.error(
                'Qwen3 model load FAILED — node will not navigate:\n' + tb)
            self._mlog.event('model_load_failed', traceback=tb)

    # ── generation ────────────────────────────────────────────────────────────

    def _generate(self, system: str, user: str) -> tuple:
        """Greedy generation; returns (text, latency_s)."""
        import torch
        messages = [{'role': 'system', 'content': system},
                    {'role': 'user', 'content': user}]
        prompt = self.tokenizer.apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True,
            enable_thinking=self._enable_thinking)
        inputs = self.tokenizer([prompt], return_tensors='pt').to(
            self.model.device)
        t0 = time.monotonic()
        with torch.no_grad():
            out = self.model.generate(
                **inputs,
                max_new_tokens=self._max_new_tokens,
                do_sample=False,
                pad_token_id=self.tokenizer.eos_token_id)
        latency = time.monotonic() - t0
        text = self.tokenizer.decode(
            out[0][inputs.input_ids.shape[1]:], skip_special_tokens=True)
        return text, latency

    def _call(self, mode: str, system: str, user: str,
              validate) -> 'dict | None':
        """Generate + parse + validate, with one retry carrying the error back."""
        for attempt in (1, 2):
            try:
                text, latency = self._generate(system, user)
            except Exception as e:
                self._log.error(f'LLM generate failed ({mode}): {e}')
                self._mlog.event('llm_call', mode=mode, attempt=attempt,
                                 prompt=user, error=str(e))
                return None
            self._mlog.event('llm_call', mode=mode, attempt=attempt,
                             prompt=user, response=text,
                             latency_s=round(latency, 2))
            try:
                parsed = extract_json(text)
                err = validate(parsed)
                if err:
                    raise ValueError(err)
                self._mlog.event('llm_parsed', mode=mode, parsed=parsed)
                return parsed
            except Exception as e:
                self._mlog.event('llm_parse_error', mode=mode, attempt=attempt,
                                 error=str(e), response=text)
                self._log.warn(f'LLM {mode} reply invalid ({e}) — '
                               f'{"retrying" if attempt == 1 else "giving up"}')
                user = (user + f'\n\nYour previous reply was invalid ({e}). '
                        'Reply with ONLY the JSON object.')
        return None

    # ── public API ────────────────────────────────────────────────────────────

    def perception_params(self, target: str,
                          current: dict, stats: str = '') -> 'dict | None':
        """LLM sets the clustering params for this target — at startup and on
        `retune`. `stats` carries the current digest summary for retunes."""
        def validate(p):
            try:
                mv = int(p.get('min_voxels'))
                sf = float(p.get('score_floor'))
                mg = float(p.get('merge_gap_m'))
            except (TypeError, ValueError):
                return ('need numeric "min_voxels", "score_floor", '
                        '"merge_gap_m"')
            if not (1 <= mv <= 2000):
                return 'min_voxels out of range [1, 2000]'
            if not (0.0 <= sf <= 0.5):
                return 'score_floor out of range [0.0, 0.5]'
            if not (0.0 <= mg <= 6.0):
                return 'merge_gap_m out of range [0.0, 6.0]'
            return None
        context = ''
        if current:
            context += (f'Current parameters: {json.dumps(current)}.\n')
        if stats:
            context += f'Current clustering result: {stats}\n'
        user = PERCEPTION_INSTRUCTIONS.format(target=target, context=context)
        parsed = self._call('perception', PERCEPTION_SYSTEM, user, validate)
        if parsed is None:
            return None
        return dict(min_voxels=int(parsed['min_voxels']),
                    score_floor=float(parsed['score_floor']),
                    merge_gap_m=float(parsed['merge_gap_m']),
                    reason=str(parsed.get('reason', '')))

    def decide(self, digest_text: str, target: str,
               selectable_instances: set, selectable_rays: set,
               frontier_sectors: set, allow_survey: bool,
               allow_retune: bool = False) -> 'dict | None':
        """Ask for a goal decision. Returns validated dict or None."""
        def validate(p):
            act = p.get('action')
            if not isinstance(act, dict):
                return 'missing "action" object'
            typ, iid = act.get('type'), str(act.get('id', ''))
            if typ == 'goto_instance':
                if iid not in selectable_instances:
                    return (f'id "{iid}" is not a selectable instance; choose '
                            f'from {sorted(selectable_instances)}')
            elif typ == 'goto_ray':
                if iid not in selectable_rays:
                    return (f'id "{iid}" is not a selectable ray lead; choose '
                            f'from {sorted(selectable_rays)}')
            elif typ == 'goto_frontier':
                if iid not in frontier_sectors:
                    return (f'"{iid}" is not an unexplored direction; choose '
                            f'from {sorted(frontier_sectors)}')
            elif typ == 'survey':
                if not allow_survey:
                    return ('survey was done recently and is on cooldown; '
                            'pick another action')
            elif typ == 'retune':
                if not allow_retune:
                    return ('retune was done recently and is on cooldown; '
                            'pick another action')
            else:
                return ('action.type must be one of goto_instance, goto_ray, '
                        'goto_frontier, survey, retune')
            return None
        user = (digest_text + '\n\n' + DECIDE_INSTRUCTIONS.format(
            target=target,
            instance_ids=', '.join(sorted(selectable_instances)) or '(none)',
            ray_ids=', '.join(sorted(selectable_rays)) or '(none)',
            frontier_ids=', '.join(sorted(frontier_sectors)) or '(none)',
            survey_ok='yes' if allow_survey else 'no (on cooldown)',
            retune_ok='yes' if allow_retune else 'no (on cooldown)'))
        return self._call('decide', DECIDE_SYSTEM, user, validate)

    def part_aliases(self, target: str, labels: list) -> 'dict | None':
        """One-shot: which bank labels are PARTS of the target? Returns an
        alias map {part_label: target} or None on failure."""
        def validate(p):
            if not isinstance(p.get('parts'), list):
                return 'missing "parts" list'
            bad = [x for x in p['parts']
                   if str(x).lower() not in {l.lower() for l in labels}]
            if bad:
                return f'labels not in the list: {bad}'
            return None
        user = ALIASES_INSTRUCTIONS.format(
            target=target, labels=', '.join(labels))
        parsed = self._call('aliases', ALIASES_SYSTEM, user, validate)
        if parsed is None:
            return None
        return {str(x).lower(): target.lower()
                for x in parsed['parts'] if str(x).lower() != target.lower()}

    def narrate(self, digest_text: str, target: str) -> 'dict | None':
        def validate(p):
            return None if isinstance(p.get('seeing'), str) \
                else 'missing "seeing" string'
        user = (digest_text + '\n\n'
                + NARRATE_INSTRUCTIONS.format(target=target))
        return self._call('narrate', NARRATE_SYSTEM, user, validate)
