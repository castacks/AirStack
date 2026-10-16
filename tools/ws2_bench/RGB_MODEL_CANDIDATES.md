# RGB-camera UAV navigation candidates

Checked 14 October 2026 against author papers and official repositories. This is
an artifact/experimental-evidence review, **not an AirStack flight comparison**.
No candidate below is guaranteed to outperform MonoNav/Kim in our Office scene.

## Recommendation

**Subsequent local checks:** The initial Office/ResBlock integration failed
qualification in10development flights (8collisions,2insufficient-progress).
The later Tiny/FOV78 corridor profile completed four clean U-course traversals
and a separate zigzag trial; its matched noise trial collided and delay trial
completed. These are different scenes and variants, not interchangeable results.
See [Office evidence](PULP_DRONET_VALIDATION.md) and
[current corridor evidence](CORRIDOR_VALIDATION.md). The candidate recommendation
below was not a prediction of general AirStack flight reliability.

For the fastest third-model integration attempt, try **PULP-DroNet v3**, with its tiny
variant as a second checkpoint. It has PyTorch weights, a small inference model,
and real closed-loop obstacle/corridor experiments. Its neural input is **grayscale
converted from the camera**, not three-channel RGB, and it is a reactive policy
without a goal. If three-channel RGB is essential, investigate GRaD-Nav or SOUS
VIDE, but neither is a verified ready-to-run general Office avoidance checkpoint.

Do not replace the target merely because its paper video looks better. First
check original inference/preprocessing, adapter conventions, and a short clean
qualification on a straight route, a turn and an obstacle requiring a detour.
Keep failed qualification results and do not compensate by changing success rules.

## Verified shortlist

| Candidate | Actual input/output | Available artifacts | Evidence and practical limitation |
|---|---|---|---|
| **PULP-DroNet v3 / Tiny-PULP-DroNet v3**, *Distilling Tiny and Ultra-fast Deep Neural Networks for Autonomous Navigation on Nano-UAVs*, IEEE IoT Journal, 2024 | Monocular grayscale image → steering/yaw and collision probability; control wrapper sets speed | Official PyTorch code **and .pth weights in repository**; Apache-2.0 code, dataset has a separate license | Real Crazyflie U-shaped corridor with obstacles. Static course at target 0.5 m/s: v3 **4/5**, tiny **5/5**; at 1 m/s v3 **3/5**, tiny **0/5**. Small samples and task-specific results; not a high-speed general solution. |
| **GRaD-Nav**, IROS 2025, and **GRaD-Nav++**, RA-L | RGB plus dynamical/proprioceptive context (and language for ++); low-level control | Official simulator/training code, scene data instructions; **no policy .pt/.pth/.ckpt in inspected repository tree or explicit pretrained-policy link in README** | Original GRaD-Nav real gate tasks: **7/10, 7/10, 6/10** for three gate placements. ++ reports real trained/unseen-task success **67%/50%**. Strong research candidate, but task-specific training and body-rate/thrust integration are substantial work. |
| **SOUS VIDE**, 2025 manuscript revision | RGB, optical flow and IMU history → thrust/body rates | Official training/simulation code and example notebooks; README still labels real deployment tutorial “coming soon”; pretrained policies not verified as downloadable | **105 hardware trials** examine scene/dynamics perturbations. Policies are trained for reconstructed scenes and desired routes; this is not a general pretrained Office obstacle-avoidance policy. |
| **MoNA Bench**, *Drones*, 2024 | RGB → learned depth, **ToF altitude for metric scale**, ORB-SLAM2 and Fast-Planner → velocity | Official ROS1 code, underlying MDE model setup; no single ready policy checkpoint in repository | Real MAV obstacle avoidance/target tracking. Useful modular reference for metric-depth scale and classical replanning. **Learning-based perception with a classical planner**, not an end-to-end avoidance network; ROS1 porting and scale assumptions matter. |
| **DroNet: Learning to Fly by Driving**, RA-L 2018 | Camera image converted to grayscale → steering and collision probability | Author code and trained real-experiment weights | Real street/corridor/parking flight; old TensorFlow/Keras stack. Prefer v3 for a new adapter rather than assuming the original handles dense Office clutter. |

PULP results above are **requested target speeds**, not measured average speeds.
On its static course the reported successful averages at target 0.5 m/s were
0.39 m/s (v3) and 0.45 m/s (tiny). Faster inference alone is not faster successful
flight. Source: [paper, Tables VII–VIII](https://arxiv.org/html/2407.12675v1).

## Sources and availability checks

- PULP: [paper](https://arxiv.org/abs/2407.12675),
  [official code](https://github.com/pulp-platform/pulp-dronet),
  [weights](https://github.com/pulp-platform/pulp-dronet/tree/master/tiny-pulp-dronet-v3/model).
  Inspected tree `98f15cfe2e9c34c3a546358e22e45b3b09f11a79` contains
  `pulp-dronet-v3-resblock-1.0.pth` (1,298,645 bytes) and
  `tiny-pulp-dronet-v3-dw-pw-0.125.pth` (24,085 bytes).
- GRaD-Nav: [paper and real-flight table](https://arxiv.org/html/2503.03984v2),
  [++ paper](https://arxiv.org/html/2506.14009v2),
  [code](https://github.com/Qianzhong-Chen/grad_nav).
  Inspected tree `95218abe9a5d355519cdaab78d915365303066fe`.
  A README path to `examples/logs/.../best_policy.pt` is a locally trained output,
  not proof of a downloadable checkpoint. The [repository license](https://github.com/Qianzhong-Chen/grad_nav/blob/main/LICENSE.md)
  restricts use to non-commercial research/evaluation: call this publicly available
  research code, not unrestricted open-source software.
- SOUS VIDE: [author project](https://stanfordmsl.github.io/SousVide/),
  [paper](https://arxiv.org/abs/2412.16346),
  [code](https://github.com/StanfordMSL/SousVide).
  Tree `a2400aa6e100c81b388bd105fdbe153710479004`: `cohorts/` only has `.gitignore`;
  example notebook includes training. The external scene archive was not
  downloaded; do not infer whether it also contains a usable policy.
- MoNA: [paper](https://www.mdpi.com/2504-446X/8/2/66),
  [code, inputs and launch instructions](https://github.com/npu-ius-lab/MoNA-Bench).
  Tree `e5a20ac47dbdff8051432fed0184243e94506e73`. GPLv3 framework plus component
  licenses. External MDE checkpoint availability was not fully verified.
- Original DroNet: [author page and weights](https://rpg.ifi.uzh.ch/dronet.html),
  [code](https://github.com/uzh-rpg/rpg_public_dronet).

## Candidates that do not currently meet the ready-open-source requirement

- **Seeing Through Pixel Motion: Learning Obstacle Avoidance from Optical Flow
  with One Camera** reports real flights up to 6 m/s. A public official navigation
  repository and trained policy were **not found** in the paper/author links and
  searches. This is not proof that the authors cannot share it. Request their
  inference/training code, policy checkpoint and camera/control specifications;
  otherwise a reproduction is required. Do not confuse the separate
  depth-input DiffPhysDrone project with this optical-flow policy.
  [Paper](https://arxiv.org/abs/2411.04413), [author page](https://zyayoung.github.io/).
- **SKYTOPIA** uses monocular RGB with state/goal information and shows hardware
  flight, but its official page currently marks code pending.
  [Project](https://eugshang.github.io/Skytopia/), [paper](https://arxiv.org/abs/2609.26007).
- Depth-input methods such as Agile Autonomy/YOPO are not evidence of RGB-only
  deployment; RGB-D VLN and fixed-wing SITL demos do not answer the same question.

## A fair third-target integration

1. Pin code/weights and preserve original image normalization, crop and history.
2. Implement the adapter, reset/state, timing and output contract. For reactive
   DroNet use collision-free useful travel, not goal arrival.
3. Validate camera orientation/FOV and turn sign, then clean flight reliability
   in meaningful avoidance scenes at fixed speeds. Paper rates do not transfer
   directly between a nano-drone and our larger vehicle/controller.
4. Register only applicable attacks. Rui's FCRN patch must not automatically be
   enabled for this new target. Start with separated noise/delay/environment tests.
5. Compare agent/random/search only after the clean controls work. Report failure
   discovery and reproducibility, not just the convenience of a working GUI.
