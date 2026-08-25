# search_baselines

One launch file per search method, so a run is named after the METHOD:

```bash
ros2 launch search_baselines vlfm.launch.xml      scene_params_file:=<scene>.yaml
ros2 launch search_baselines conavgpt2.launch.xml scene_params_file:=<scene>.yaml
ros2 launch search_baselines nearest.launch.xml   scene_params_file:=<scene>.yaml
```

## Why one node, three methods

The executable these launch is `search_planner`, and it lives HERE. It is built
around the **vendored upstream Co-NavGPT2** library in the `conavgpt2` package
(`Global_Map_Proc`, `Frontier_Det`, `ROS_Agent`), so the package name says what
the code IS. VLFM and the geometric control reuse that machinery — the RGBD
occupancy map, the frontier extraction, the search-area and altitude filters,
the droan actuation — and differ ONLY in how a frontier is chosen:

| method | selection |
|---|---|
| `conavgpt2` | a generative VLM picks from numbered top-down BEV images |
| `vlfm` | BLIP-2 ITM scores the live RGB into a value map; argmax value |
| `nearest` | closest frontier, no model at all |

Sharing the map is not a shortcut, it is the experiment: because the three build
the SAME map from the SAME frames and act through the SAME controller, a
difference in the result is attributable to the selection policy and to nothing
else. Two separate implementations would each need their own mapping to be
argued equivalent before any comparison meant anything.

`conavgpt2` is a LEAF: it holds the vendored third-party code its name refers
to and depends on nothing in this stack.

## What each method needs running

| method | service | VRAM |
|---|---|---|
| `conavgpt2` | `ros2 run search_baselines vlm_server` (generative, OpenAI-compatible) on :8000 | ~5.9 GiB at 7B nf4 |
| `vlfm` | `ros2 run search_baselines itm_server` (BLIP-2 ITM) on :8100 | ~2.5 GiB |
| `nearest` | nothing | — |

`vlfm` does **not** preflight or contact the generative endpoint, so a VLFM run
needs only the ITM service.
