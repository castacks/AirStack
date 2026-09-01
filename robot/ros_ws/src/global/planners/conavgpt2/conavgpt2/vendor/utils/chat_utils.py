import numpy as np
import ast
import time
import requests
import json
import re
import base64
import openai
from openai import OpenAI
from io import BytesIO
import ast
import cv2

import conavgpt2.vendor.utils.visualization as vu

class _Config:
    """Backend settings, filled in by conavgpt2.vlm_client.configure() before the
    first request. Upstream read these from argparse at import time, which is not
    available under `ros2 run`."""

    base_url = None       # None -> the openai SDK default / OPENAI_BASE_URL
    api_key = None        # None -> OPENAI_API_KEY
    model = "gpt-4o"
    num_agents = 2
    timeout = 60.0


CONFIG = _Config()
_CLIENT = None


def get_client():
    """Deferred client construction: OpenAI() raises when no key is configured,
    so building it at import time makes the whole package unimportable."""
    global _CLIENT
    if _CLIENT is None:
        _CLIENT = OpenAI(
            base_url=CONFIG.base_url,
            api_key=CONFIG.api_key,
            timeout=CONFIG.timeout,
        )
    return _CLIENT


def reset_client():
    global _CLIENT
    _CLIENT = None


gpt_name = [
            'text-davinci-003',
            'gpt-3.5-turbo-0125',
            'gpt-4o',
            'gpt-4o-mini'
        ]           
def transform_rgb_bgr(image):
    return image[:, :, [2, 1, 0]]


def get_all_candidate_maps(target_edge_map, top_view_map, pose):
    # show paths in map
    candidate_map_list = []
    for i in range(int(target_edge_map.max())):
        map_with_frontier = top_view_map.copy()
        path_map = np.zeros(target_edge_map.shape)
        path_map[target_edge_map == i+1] = 1
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5, 5))
        path_map = cv2.dilate((path_map).astype('uint8'), kernel)
        map_with_frontier[path_map == 1] = [255, 0 , 0]
        map_with_frontier = np.flipud(map_with_frontier)
        map_with_pose = vu.write_number(map_with_frontier, pose, i)
        buffered = BytesIO()
        map_with_pose.save(buffered, format="JPEG")
        candidate_map_list.append(buffered)

    return candidate_map_list

def get_all_candidate_obs_maps(target_edge_map, top_view_map, pose):
    # show paths in map
    candidate_map_list = []
    for i in range(int(target_edge_map.max())):
        map_with_frontier = top_view_map.copy()
        mask = np.any(top_view_map != 0, axis=-1)
        map_with_frontier[mask] = [255, 255, 255]
        path_map = np.zeros(target_edge_map.shape)
        path_map[target_edge_map == i+1] = 1
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5, 5))
        path_map = cv2.dilate((path_map).astype('uint8'), kernel)
        map_with_frontier[path_map == 1] = [255, 0 , 0]
        map_with_frontier = np.flipud(map_with_frontier)
        map_with_pose = vu.write_number(map_with_frontier, pose, i)
        buffered = BytesIO()
        map_with_pose.save(buffered, format="JPEG")
        candidate_map_list.append(buffered)
        
    return candidate_map_list

def get_all_candidate_full_maps(image_id, target_edge_map, top_view_map, pose):
    # show paths in map
    candidate_map_list = []
    for i in range(int(target_edge_map.max())):
        map_with_frontier = top_view_map.copy()
        path_map = np.zeros(target_edge_map.shape)
        path_map[target_edge_map == i+1] = 1
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5, 5))
        path_map = cv2.dilate((path_map).astype('uint8'), kernel)
        map_with_frontier[path_map == 1] = [255, 0 , 0]
        map_with_frontier = np.flipud(map_with_frontier)
        
        frontier_image = image_id[i]
        np_image = np.asarray(frontier_image)
        resized_image1 = cv2.resize(np_image, (480, 480))
        resized_image2 = cv2.resize(np.asarray(map_with_frontier), (480, 480))
        
        combined_image = np.hstack((resized_image2, resized_image1))
        
        map_with_pose = vu.write_number_full(combined_image, pose, i)
        
        buffered = BytesIO()
        map_with_pose.save(buffered, format="JPEG")
        candidate_map_list.append(buffered)
        
    return candidate_map_list

def message_prepare(prompt, candidate_map_list, navigation_instruct):
    base64_image_list = []
    for image_candidate in candidate_map_list:
        base64_image_list.append(base64.b64encode(image_candidate.getvalue()).decode("utf-8"))


    message = []
    message.append({"role": "system", "content": prompt})

    image_contents = []
    # Upstream hardcodes "two robots" here, because the paper is a two-robot
    # method. Left as a literal it is a LIE to the model on any other team size,
    # and the model acts on it: at num_agents=1 it reserves a frontier for a
    # robot that does not exist ("Robot 1 should explore the other frontier to
    # cover the remaining area") and so declines to send the one real robot to
    # the best frontier. CONFIG.num_agents is already the team size everywhere
    # else in this file; use it.
    n = max(1, int(getattr(CONFIG, "num_agents", 2) or 1))
    subject = "1 robot needs" if n == 1 else f"{n} robots need"
    image_contents.append({
        "type": "text",
        "text": f"{subject} to find a " + navigation_instruct,
    })
    for base64_image in base64_image_list:
        image_contents.append({
            "type": "image_url",
            "image_url": {
                "url": f"data:image/jpeg;base64,{base64_image}"
            }
        })
    message.append({"role": "user", "content": image_contents})
    
    return message


# Per-call telemetry, read by the ROS wrapper after every round. Latency and
# prompt size grow with the FRONTIER count (one top-view image per candidate), so
# this is what says whether a round still fits inside its period.
LAST_CALL = {}


def _reset_last_call(model, num_frontier):
    LAST_CALL.clear()
    LAST_CALL.update({
        "model": model,
        "num_frontier": num_frontier,
        "num_agents": CONFIG.num_agents,
        "attempts": 0,
        "server_s": 0.0,
        "last_server_s": None,
        "prompt_tokens": None,
        "completion_tokens": None,
        "total_tokens": None,
        "parse": "failed",
        "lenient": False,
        "invalid_ids": [],
        "errors": [],
        "response_text": None,
    })


_ASSIGN_RE = re.compile(r'"robot_(\d+)"\s*:\s*"frontier_(\d+)"')


def parse_assignment(text, num_agents, num_frontier):
    """`{robot_i: frontier_k}` out of a model reply, or None.

    Strict first (`ast.literal_eval` of the whole JSON object), then LENIENT:
    the `"robot_i": "frontier_k"` pairs pulled by regex. The lenient path is
    what makes a TRUNCATED reply usable — a small VLM answers for one robot
    per image it was shown (robot_0..robot_6 for seven BEVs) and then starts
    a "reason", so at max_tokens the JSON is cut mid-string and the strict
    parse raises `unterminated string literal`. Measured on the 250 m suburb
    with Qwen2.5-VL-3B: 12 of 13 rounds failed that way, each after FIVE
    26-second retries with the planner blocked. Every one of them carried a
    complete, valid `robot_0` assignment in its first line.

    Extra robots the model invented are dropped; a missing or out-of-range id
    for a robot that exists is a failure (None), as before.
    """
    pairs = None
    try:
        obj = ast.literal_eval(text)
        if isinstance(obj, dict):
            pairs = {k: str(v) for k, v in obj.items() if str(k).startswith("robot_")}
    except (SyntaxError, ValueError, TypeError):
        pairs = None
    lenient = pairs is None
    if pairs is None:
        pairs = {f"robot_{a}": f"frontier_{b}" for a, b in _ASSIGN_RE.findall(text or "")}
    out = {}
    for i in range(num_agents):
        v = pairs.get(f"robot_{i}")
        if v is None:
            LAST_CALL["parse"] = "too_few_robots"
            return None
        try:
            k = int(str(v).split('_')[1])
        except (IndexError, ValueError):
            LAST_CALL["parse"] = "bad_json"
            return None
        if k < 0 or k >= num_frontier:
            LAST_CALL["invalid_ids"].append({"robot": i, "frontier": v})
            LAST_CALL["parse"] = "invalid_frontier_id"
            return None
        out[f"robot_{i}"] = f"frontier_{k}"
    if lenient:
        LAST_CALL["lenient"] = True
        m = re.search(r'"reason"\s*:\s*"([^"]*)', text or "")
        if m:
            out["reason"] = m.group(1)
    else:
        try:
            r = ast.literal_eval(text).get("reason")
            if r:
                out["reason"] = str(r)
        except Exception:
            pass
    return out


def chat_with_gpt4v(chat_history, model=None):
    num_frontier = len(chat_history[1]['content'])-1
    model = model or CONFIG.model
    client = get_client()
    _reset_last_call(model, num_frontier)
    # TWO attempts, not five. A failed generation costs a full decode (26 s
    # on the 3B model) and nothing flies while it runs; the lenient parser
    # above already recovers the truncated case, so retries are only for a
    # dead endpoint.
    retries = 2
    while retries > 0:
        try: 
            LAST_CALL["attempts"] += 1
            _t0 = time.time()
            response = client.chat.completions.create(
                model=model, 
                response_format = { "type": "json_object" },
                messages=chat_history,
                temperature=0.1,
                # 160, not upstream's 100: the assignment lines plus a short
                # reason. The lenient parser copes with a cut-off, but a reply
                # that fits is a reply that carries its reason into the log.
                max_tokens=160,
            )
            _dt = time.time() - _t0
            LAST_CALL["last_server_s"] = _dt
            LAST_CALL["server_s"] += _dt
            _usage = getattr(response, "usage", None)
            if _usage is not None:
                LAST_CALL["prompt_tokens"] = getattr(_usage, "prompt_tokens", None)
                LAST_CALL["completion_tokens"] = getattr(_usage, "completion_tokens", None)
                LAST_CALL["total_tokens"] = getattr(_usage, "total_tokens", None)

            response_message = response.choices[0].message.content
            LAST_CALL["response_text"] = (response_message or "")[:512]
            print(model + " response: ")
            print(response_message)
            ground_json = parse_assignment(response_message or "",
                                           CONFIG.num_agents, num_frontier)
            if ground_json is not None:
                LAST_CALL["parse"] = "ok" if LAST_CALL["attempts"] == 1 else "retried"
                if LAST_CALL.get("lenient"):
                    LAST_CALL["parse"] += "_lenient"
                return ground_json
            LAST_CALL["errors"].append(f"parse: {LAST_CALL['parse']}: "
                                       f"{(response_message or '')[:80]!r}")
        except openai.APIError as e:
            #Handle API error here, e.g. retry or log
            LAST_CALL["errors"].append(f"api: {e}")
            print(f"OpenAI API returned an API Error: {e}")
            pass
        except openai.APIConnectionError as e:
            #Handle connection error here
            LAST_CALL["errors"].append(f"connection: {e}")
            print(f"Failed to connect to VLM endpoint {CONFIG.base_url}: {e}")
            pass
        except openai.RateLimitError as e:
            #Handle rate limit error (we recommend using exponential backoff)
            LAST_CALL["errors"].append(f"rate_limit: {e}")
            print(f"OpenAI API request exceeded rate limit: {e}")
            pass
        retries -=1
            
    # print(ground_json)
    LAST_CALL["parse"] = "failed"
    # SPREAD THE FLEET, DO NOT STACK IT. Upstream fell back to
    #     {f"robot_{i}": "frontier_0" for i in range(CONFIG.num_agents)}
    # which sends EVERY robot to the SAME frontier. On a failed round the whole
    # team converges on one point, re-explores ground the others already hold,
    # and the search area stops growing -- the opposite of what a multi-robot
    # method is for. Measured on the 2026-09-01 tornado L1 team cell: 3 of 44
    # rounds took this path.
    #
    # Round-robin over the frontiers that actually exist instead. It is still a
    # degraded fallback -- nothing here knows which frontier suits which robot,
    # that is what the VLM was for -- but it preserves DISPERSION, which is the
    # property the fleet cannot recover on its own. num_frontier is whatever the
    # caller offered this round; guard it so a zero can never modulo-divide.
    n_front = max(1, int(num_frontier or 1))
    print(f"VLM assignment failed after retries (model={model}, "
          f"base_url={CONFIG.base_url}); falling back to a round-robin over "
          f"{n_front} frontier(s) to keep the robots spread out")
    ground_json = {f"robot_{i}": f"frontier_{i % n_front}"
                   for i in range(CONFIG.num_agents)}
    return ground_json
