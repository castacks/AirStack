# Planners

Planners are the core of the `planner` module. They are responsible for generating a plan for the robot to follow. The `planner` module provides a number of planners that can be used to generate plans for the robot to follow. The planners are divided into two categories: trivial planners and advanced planners.


## Trivial Planners

### Random Walk planner
The random walk planner generates

The random walk planner replans when the robot is getting close to the goal. The random walk planner is a trivial planner that generates a plan by randomly selecting a direction to move in. The random walk planner is useful for testing the robot's ability to follow a plan.