# Testing

For testing in general, all tests will be required to be exectued by colcon test for ROS2. All tests should be located in the ros_ws/ folder and should follow the similar format located [here](https://github.com/ros2/launch_ros/blob/master/launch_testing_ros/test/examples/talker_listener_launch_test.py). Any end to end testing with the simulator or systems should be written in Python, and should spin up nodes or launch files with rclpy. For unit tests, these should be written in GTest, for C++ logic, and Pytest for python logic.

All testing should be automated by Github actions. Since all tests should be run by entering `colcon test` in the command line, Github actions should be able to run these and pass if all tests return Passed. This is currently being integrated.

## End to End Testing

All end to end testing procedures will live in the end2end_testing package. These end to end tests are meant to be run with the simulation and test full system functionality like obstacle avoidance, successfully navigating to a goal point, or executing an instructed task. 

Currently the end to end testing waits for the simulation to start up in headless mode, and passes if this occurs. Specific tests can be added by loading different usd files in the simulator. 

### Github Actions

## System Tests

### Unit Tests



