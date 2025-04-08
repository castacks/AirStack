# TakeoffLandingPlanner Launch Tests

This file contains integration tests for the TakeoffLandingPlanner ROS2 node using the ROS2 launch testing framework. The tests verify that the node correctly handles takeoff and landing commands, generates appropriate trajectories, and tracks completion states.

## Overview

The test harness launches the TakeoffLandingPlanner node with test-specific parameters and performs a series of tests to verify its behavior. It simulates sensor data, sends commands, and checks the responses to ensure the node operates as expected.

## Test Configuration

The test uses a specific YAML configuration file:

```
test/config/test_takeoff_landing_planner.yaml
```

This configuration file contains test-specific parameters for the TakeoffLandingPlanner node, separate from the default parameters used in production. This allows for controlled testing without modifying the production configuration.

The parameter file is passed to both the target TakeoffLandingPlanner node via the `ParameterFile` class in the launch description, and is also parsed using PyYAML within the test class itself:

```python
# Send parameters to the node being tested
takeoff_landing_planner = Node(
    package="takeoff_landing_planner",
    executable="takeoff_landing_planner",
    parameters=[
        ParameterFile(
            param_file=param_file,
            allow_substs=True,
        )
    ],
    # ...
)

# Later in the test class, parse the same file for test verification
with open(param_file, "r") as file:
    param_file = yaml.safe_load(file)
cls.params = param_file["/**"]["ros__parameters"]
```

This dual use ensures that the test logic operates with the same parameter values as the node being tested.

## Test Execution Process

The test follows the ROS2 launch testing pattern as described in the [ROS2 Launch Testing Documentation](https://github.com/ros2/launch/tree/master/launch_testing/doc):

1. A test description is generated using `generate_test_description()`
2. The TakeoffLandingPlanner node is launched with test-specific parameters
3. A timer ensures a 1-second delay before starting tests ([recommended practice](https://github.com/ros2/launch/blob/master/launch_testing/doc/nodes.md#allowing-nodes-time-to-start)) to allow the node to initialize
4. Tests are run against the live node

## Unit Tests

The test suite includes the following test cases:

### `test_takeoff_parameters`

Verifies that all required parameters are properly loaded from the configuration file:

- `takeoff_height`
- `high_takeoff_height`
- `takeoff_landing_velocity`
- `takeoff_acceptance_distance`
- And others...

### `test_node_active`

Ensures that the node is active and its services are available by checking that the service client can connect within a timeout period.

### `test_service_response`

Tests the service interface by:

1. Sending takeoff, landing, and reset commands
2. Verifying that responses indicate acceptance
3. Verifying correct response types

### `test_takeoff_and_landing`

A comprehensive integration test that:

1. Simulates the full takeoff sequence:
   - Publishes initial ground position
   - Sends takeoff command
   - Checks generated trajectory parameters
   - Simulates in-progress flight
   - Verifies "TAKING_OFF" state
   - Simulates completion conditions
   - Verifies "COMPLETE" state
2. Simulates the full landing sequence:
   - Sends landing command
   - Checks generated trajectory
   - Simulates in-progress landing
   - Verifies "LANDING" state
   - Simulates landed condition
   - Verifies "COMPLETE" state

## Helper Methods

The test includes several helper methods:

- `call_service(command)`: Sends commands to the node and verifies response
- `reset_states()`: Resets internal test state between test cases
- `yaw_to_quat(yaw)`: Converts yaw angle to quaternion
- `get_fake_odom(kind, yaw, curr_odom)`: Generates simulated odometry messages
- `get_fake_ref_point(kind)`: Generates simulated reference points
- `check_takeoff_trajectory()`: Verifies trajectory parameters
- `wait_for(secs)`: Waits for specified time while processing ROS callbacks

## Simulated Message Types

The test creates fake data of different types to simulate various conditions:

- `TYPE_ODOM_GROUNDED`: Robot on the ground
- `TYPE_ODOM_TAKEOFF_PENDING`: Robot in process of taking off
- `TYPE_ODOM_TAKEOFF_COMPLETE`: Robot successfully taken off
- `TYPE_ODOM_LAND_PENDING`: Robot in process of landing
- `TYPE_ODOM_LAND_COMPLETE`: Robot successfully landed

## Further Reading

For more information on ROS2 launch testing:

- [Launch Testing Nodes Documentation](https://github.com/ros2/launch/blob/master/launch_testing/doc/nodes.md)
- [Launch Testing Examples](https://github.com/ros2/launch/tree/master/launch_testing/examples)
- [Launch Testing API Reference](https://github.com/ros2/launch/blob/master/launch_testing/doc/api.md)

## CMake Integration

The tests are integrated into the CMake build system using isolated launch testing, which provides better isolation between test runs. The CMakeLists.txt contains the following configuration:

```cmake
find_package(ament_cmake_ros REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

# Isolated Integration Testing from (https://arnebaeyens.com/blog/2024/ros2-integration-testing/)
function(add_ros_isolated_launch_test path)
  set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
  add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
endfunction()

if(BUILD_TESTING)
  # Linter configuration is skipped for brevity

  add_ros_isolated_launch_test(
    test/scripts/test_takeoff_landing_planner.py
  )
endif()
```

This setup:

1. Requires the necessary ROS2 testing packages
2. Defines a custom function `add_ros_isolated_launch_test` that uses a special runner script for better test isolation
3. Registers our test script with this isolated runner

The isolated testing approach helps prevent interference between test runs and improves test reliability, as described in [this reference article](https://arnebaeyens.com/blog/2024/ros2-integration-testing/).

## Running the Tests

To run these tests:

```bash
colcon test --packages-select takeoff_landing_planner --event-handlers=console_direct+
```

For more verbose output:

```bash
colcon test --packages-select takeoff_landing_planner --event-handlers=console_direct+ --pytest-args -v
```
