# Remaining blockers

- Establish availability of `airlab-docker.andrew.cmu.edu/airstack/airstack:v0.20.8_robot-x86-64_dev` to the workflow's registry credential. The registry returned artifact not found for that exact tag. The evidence does not establish whether the artifact is absent or inaccessible to the task.
- Retrieve sanitized OSMO workspace stdout for the startup command and earliest task milestones, plus the outer image identity if OSMO exposes it. Retain the failed attempt as cold-start evidence.
- A separately authorized healthy cold start is required before sensor, ROS time and TF, sustained stability and external visualization can be qualified. Workflow submissions and cancellations remain user-controlled.
- Scientific geometry and downstream milestones remain outside this verification.
