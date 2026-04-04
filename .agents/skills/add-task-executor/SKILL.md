---
name: add-task-executor
description: Implement a new task executor as a ROS 2 action server in AirStack. Use when adding a new goal-directed task (coverage, search, counting, trajectory following, etc.) that a user can trigger with parameters, monitor via feedback, and cancel. Reference implementation is random_walk_planner (ExplorationTask).
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Add a New Task Executor

## When to Use

A **task executor** is a module that:
- Is activated on demand with user-specified parameters (not always running)
- Has a well-defined completion condition (time limit, area covered, object found, etc.)
- Should support cancellation mid-flight
- Provides progress feedback to the caller

Examples: coverage survey, object search, object counting, semantic search, fixed trajectory following, exploration.

**Do NOT use this skill for continuous modules** (state estimation, depth estimation, trajectory controller). Those are always-running nodes that don't need an action server.

## Prerequisites

- Identify which `task_msgs` action type your executor should handle (see `common/ros_packages/msgs/task_msgs/action/`)
- Know the algorithm the executor will run and what inputs it needs (map, odometry, etc.)
- Understand the task completion condition

## Available Task Action Types

| Action File | Use For |
|-------------|---------|
| `ExplorationTask.action` | Random or systematic area exploration |
| `CoverageTask.action` | Systematic lawnmower-pattern coverage survey |
| `ObjectSearchTask.action` | Finding instances of a named object class |
| `ObjectCountingTask.action` | Counting all instances of an object class in an area |
| `FixedTrajectoryTask.action` | Following a pre-defined waypoint trajectory |
| `SemanticSearchTask.action` | Finding a location described in natural language |

If none of these fits, add a new `.action` file to `task_msgs` following the same pattern (see Step 0 below).

## Steps

### 0. (Optional) Add a new action type to `task_msgs`

Only if no existing action type fits your task.

**File:** `common/ros_packages/msgs/task_msgs/action/YourTask.action`

```
# Goal — task parameters specified by the user
geometry_msgs/Polygon search_area
float32 min_altitude_agl
float32 max_altitude_agl
float32 min_flight_speed
float32 max_flight_speed
float32 time_limit_sec           # 0 = no limit
# ... task-specific fields
---
# Result — returned when task completes or is cancelled
bool success
string message
# ... task-specific result fields
---
# Feedback — streamed ~1 Hz while task is active
string status
float32 progress                 # 0.0–1.0
geometry_msgs/Point current_position
# ... task-specific feedback fields
```

Register it in `common/ros_packages/msgs/task_msgs/CMakeLists.txt`:
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  ...
  "action/YourTask.action"
  DEPENDENCIES action_msgs geometry_msgs airstack_msgs
)
```

Rebuild `task_msgs`:
```bash
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select task_msgs"
```

### 1. Create the package

Follow [add-ros2-package](../add-ros2-package) to create the package under the appropriate layer (e.g., `robot/ros_ws/src/global/planners/your_executor/`).

Add these dependencies to `package.xml`:
```xml
<depend>rclcpp</depend>
<depend>rclcpp_action</depend>
<depend>task_msgs</depend>
<depend>geometry_msgs</depend>
<!-- ... other deps -->
```

Add to `CMakeLists.txt`:
```cmake
find_package(rclcpp_action REQUIRED)
find_package(task_msgs REQUIRED)

ament_target_dependencies(your_node
  rclcpp rclcpp_action task_msgs geometry_msgs
  # ...
)
```

### 2. Implement the action server

The required pattern — four components:

#### 2a. Type aliases (header)
```cpp
#include "rclcpp_action/rclcpp_action.hpp"
#include "task_msgs/action/your_task.hpp"

using YourTask = task_msgs::action::YourTask;
using GoalHandle = rclcpp_action::ServerGoalHandle<YourTask>;
```

#### 2b. Members (header)
```cpp
rclcpp_action::Server<YourTask>::SharedPtr action_server_;
std::atomic<bool> task_active_{false};
std::atomic<bool> cancel_requested_{false};
rclcpp::Time task_start_time_;
float task_time_limit_sec_ = 0.0f;
```

#### 2c. Four callbacks

**`handle_goal`** — reject if busy, accept otherwise:
```cpp
rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const YourTask::Goal> goal)
{
    if (task_active_) {
        RCLCPP_WARN(get_logger(), "Rejecting goal: task already active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    task_active_ = true;
    RCLCPP_INFO(get_logger(), "Accepted YourTask goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
```

**`handle_cancel`** — always accept, set flag:
```cpp
rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>)
{
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
}
```

**`handle_accepted`** — spawn execute thread (never block here):
```cpp
void handle_accepted(std::shared_ptr<GoalHandle> goal_handle)
{
    std::thread{std::bind(&YourNode::execute, this, std::placeholders::_1), goal_handle}.detach();
}
```

**`execute`** — the planning loop (runs in its own thread):
```cpp
void execute(std::shared_ptr<GoalHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    task_start_time_ = now();
    task_time_limit_sec_ = goal->time_limit_sec;
    cancel_requested_ = false;

    rclcpp::Rate rate(1.0);  // 1 Hz feedback

    while (rclcpp::ok()) {
        // 1. Check cancellation
        if (cancel_requested_) {
            auto result = std::make_shared<YourTask::Result>();
            result->success = false;
            result->message = "Task cancelled";
            task_active_ = false;
            goal_handle->canceled(result);
            return;
        }

        // 2. Check time limit
        if (task_time_limit_sec_ > 0.0f) {
            double elapsed = (now() - task_start_time_).seconds();
            if (elapsed >= task_time_limit_sec_) {
                auto result = std::make_shared<YourTask::Result>();
                result->success = true;
                result->message = "Time limit reached";
                task_active_ = false;
                goal_handle->succeed(result);
                return;
            }
        }

        // 3. Check task-specific completion condition
        if (task_is_complete()) {
            auto result = std::make_shared<YourTask::Result>();
            result->success = true;
            result->message = "Task complete";
            // ... populate result fields
            task_active_ = false;
            goal_handle->succeed(result);
            return;
        }

        // 4. Publish feedback (~1 Hz)
        auto feedback = std::make_shared<YourTask::Feedback>();
        feedback->status = "executing";
        feedback->progress = compute_progress();
        feedback->current_position = current_position_;
        goal_handle->publish_feedback(feedback);

        // 5. Run one step of the algorithm
        run_planning_step(goal);

        rate.sleep();
    }

    // Node shutting down
    auto result = std::make_shared<YourTask::Result>();
    result->success = false;
    result->message = "Node shutting down";
    task_active_ = false;
    goal_handle->abort(result);
}
```

#### 2d. Create the server (constructor)
```cpp
action_server_ = rclcpp_action::create_server<YourTask>(
    this, "~/your_task",
    std::bind(&YourNode::handle_goal,    this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&YourNode::handle_cancel,  this, std::placeholders::_1),
    std::bind(&YourNode::handle_accepted,this, std::placeholders::_1));
```

#### 2e. Executor in main

Because `execute()` runs in a **detached thread**, the main executor is never blocked by it — subscriber callbacks (map, odometry) keep running regardless of which executor you use. `rclcpp::spin()` (single-threaded) is the safe default:

```cpp
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YourNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

Only switch to `MultiThreadedExecutor` if you have independent callbacks that genuinely need concurrent execution **and** all shared resources are thread-safe. Nodes that use OpenGL, CUDA, or other thread-affine resources **must** use `rclcpp::spin()` to keep callbacks serialized.

### 3. Add remap to bringup launch

In the layer bringup launch file (e.g., `global_bringup/launch/global.launch.xml`):
```xml
<remap from="~/your_task" to="/$(env ROBOT_NAME)/tasks/your_task_name" />
```

**Convention:** all task action servers are remapped to `/{robot_name}/tasks/{task_name}`.

### 4. Document the executor

In the package `README.md`, include a **Task Executor** section:

```markdown
## Task Executor

This node is a **task executor**: it runs as a ROS 2 action server and is
activated on demand via an action goal.

**Action server:** `/{robot_name}/tasks/exploration` (type: `task_msgs/action/ExplorationTask`)

### Goal parameters
| Field | Type | Description |
|-------|------|-------------|
| `min_altitude_agl` | float32 | Minimum flight altitude above ground (m) |
| ...   | ...  | ... |

### Feedback (published ~1 Hz)
| Field | Type | Description |
|-------|------|-------------|
| `progress` | float32 | Task progress 0.0–1.0 |
| ...   | ...  | ... |

### Result
| Field | Type | Description |
|-------|------|-------------|
| `success` | bool | True if task completed normally |
| ...   | ...  | ... |

### CLI test
```bash
ros2 action send_goal /robot1/tasks/your_task task_msgs/action/YourTask \
  '{min_altitude_agl: 3.0, max_altitude_agl: 8.0, time_limit_sec: 30.0}' --feedback
```
```

### 5. Build and test

```bash
# Build
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select your_package_name"

# Verify action server appears
docker exec airstack-robot-desktop-1 bash -c "sws && ros2 action list"
# Expected: /robot1/tasks/your_task_name

# Send a test goal with feedback
docker exec airstack-robot-desktop-1 bash -c \
  "sws && ros2 action send_goal /robot1/tasks/your_task_name \
   task_msgs/action/YourTask '{...}' --feedback"

# Verify cancellation works (Ctrl-C in ros2 action send_goal sends a cancel)
```

## Reference Implementation

`robot/ros_ws/src/global/planners/random_walk/` — implements `ExplorationTask.action`.
Study this package for a complete working example of all four callbacks, the execute loop,
`task_active_` / `cancel_requested_` flags, and bringup integration.

## Common Pitfalls

- ❌ **Blocking in `handle_accepted`** — always spawn a thread; blocking the executor prevents subscriber callbacks from running
- ❌ **Forgetting `task_active_ = false`** at every exit point in `execute()` — the node will reject all future goals
- ❌ **Using `MultiThreadedExecutor` with non-thread-safe resources** (OpenGL, CUDA, etc.) — the detach pattern means `rclcpp::spin()` never starves subscriber callbacks; use `MultiThreadedExecutor` only when callbacks truly need concurrency and all shared state is thread-safe
- ❌ **Missing action server remap in bringup** — action won't be discoverable at the expected topic
- ❌ **Calling `goal_handle->succeed()` after `goal_handle->canceled()`** — check the cancel flag before the completion condition
