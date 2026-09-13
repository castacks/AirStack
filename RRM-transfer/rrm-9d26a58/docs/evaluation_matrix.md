# RRM-1 Evaluation Matrix

Target definitions for the full evaluation programme. The implemented subset and its
reproducibility rules live in `docs/benchmarks.md`.

| Category           | Metric                         | Definition                                           |              Target |
| ------------------ | ------------------------------ | ---------------------------------------------------- | ------------------: |
| **Task Success**   | Task Success Rate              | Successful missions / total missions                 |                >85% |
|                    | Partial Completion Rate        | Missions partially completed / total missions        |                >90% |
|                    | Goal Verification Accuracy     | Correct identification of task completion            |                >90% |
| **Reasoning**      | Task Decomposition Accuracy    | Correct required subtasks / expected subtasks        |                >85% |
|                    | Step Ordering Accuracy         | Correct action ordering / total dependencies         |                >90% |
|                    | Spatial Reasoning Accuracy     | Correct spatial relationships / total tested         |                >90% |
|                    | Temporal Reasoning Accuracy    | Correct temporal dependencies / total tested         |                >90% |
| **World Model**    | Object-State Accuracy          | Correct object states / total objects                |                >90% |
|                    | Spatial State Accuracy         | Correct object positions/relations                   |                >90% |
|                    | State Transition Accuracy      | Correct state updates after actions                  |                >90% |
|                    | Memory Consistency             | Correct historical state retrieval                   |                >90% |
| **Planning**       | Plan Validity Rate             | Executable plans / generated plans                   |                >95% |
|                    | Planning Time                  | Time required to generate a valid plan               |            Minimize |
|                    | Action Efficiency              | Required actions / executed actions                  |                >85% |
|                    | Replanning Rate                | Tasks requiring replanning / total tasks             |                <20% |
| **Execution**      | Action Success Rate            | Successful actions / attempted actions               |                >90% |
|                    | Manipulation Success           | Successful grasps/placements / attempts              |                >90% |
|                    | Navigation Success             | Successful navigation tasks / attempts               |                >95% |
|                    | Execution Time                 | Time from first action to completion                 |            Minimize |
| **Safety**         | Safety Violation Rate          | Unsafe actions / total actions                       |                 ~0% |
|                    | Collision Rate                 | Collisions / total trials                            |                 ~0% |
|                    | Safety Verifier Recall         | Unsafe actions correctly rejected                    |                >99% |
|                    | Safety Verifier Precision      | Rejected actions that were actually unsafe           |                >95% |
|                    | False-Negative Rate            | Unsafe actions incorrectly permitted                 |                 <1% |
| **Recovery**       | Failure Detection Rate         | Detected failures / injected failures                |                >90% |
|                    | Recovery Success Rate          | Recovered failures / detected failures               |                >80% |
|                    | Recovery Time                  | Failure detection → successful recovery              |            Minimize |
|                    | Recovery Action Overhead       | Additional actions required after failure            |            Minimize |
| **Generalization** | Environment Generalization     | Performance on unseen environments                   |       >75% retained |
|                    | Object Generalization          | Performance on unseen objects                        |       >75% retained |
|                    | Instruction Generalization     | Performance on novel language formulations           |       >80% retained |
|                    | Embodiment Transfer            | Performance on unseen robot embodiment               |       >70% retained |
| **Robustness**     | Sensor Robustness              | Performance under sensor noise/dropout               |       >80% retained |
|                    | Occlusion Robustness           | Performance under partial visual occlusion           |       >80% retained |
|                    | Dynamic Environment Robustness | Performance with moving obstacles/people             |       >80% retained |
|                    | Disturbance Recovery           | Successful recovery after environmental perturbation |                >80% |
| **Sim-to-Real**    | Sim Success Rate               | Success rate in simulation                           |                >85% |
|                    | Real Success Rate              | Success rate on physical robot                       |                >75% |
|                    | Sim-to-Real Gap                | Simulation performance − real performance            |                <15% |
| **Efficiency**     | Inference Latency              | Model inference time per reasoning cycle             |            Minimize |
|                    | GPU Memory                     | Peak GPU memory usage                                |            Minimize |
|                    | Compute Cost                   | Compute required per completed task                  |            Minimize |
|                    | Token/Model Calls              | AI calls required per task                           |            Minimize |
| **Long-Horizon**   | Success vs. Task Horizon       | Success rate as action count increases               | >70% at 10+ actions |
|                    | State Consistency              | Correct world state throughout task                  |                >90% |
|                    | Long-Horizon Recovery          | Recovery rate during multi-step tasks                |                >80% |
| **Overall**        | Weighted RRM Score             | Weighted aggregate of primary categories             |                >85% |

## Primary Weighted Score

| Category       |   Weight |
| -------------- | -------: |
| Task Success   |      25% |
| Reasoning      |      15% |
| World Model    |      15% |
| Safety         |      15% |
| Recovery       |      10% |
| Generalization |      10% |
| Efficiency     |      10% |
| **Total**      | **100%** |

## Required Baseline Comparisons

| System                          | Task Success | Reasoning | World Model | Safety | Recovery | Generalization | Efficiency |
| ------------------------------- | -----------: | --------: | ----------: | -----: | -------: | -------------: | ---------: |
| Classical Robotics              |              |           |             |        |          |                |            |
| LLM + Classical Planner         |              |           |             |        |          |                |            |
| VLM + Classical Planner         |              |           |             |        |          |                |            |
| VLA                             |              |           |             |        |          |                |            |
| RRM without World Model         |              |           |             |        |          |                |            |
| RRM without Predictive Planning |              |           |             |        |          |                |            |
| **RRM-1**                       |              |           |             |        |          |                |            |
