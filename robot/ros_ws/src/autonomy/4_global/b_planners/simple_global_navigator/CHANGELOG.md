# Changelog

## [1.1.0] - Time Constraint Support

### Added
- **Time Constraint Compliance**: Added support for `max_planning_seconds` constraint from NavigationTask
- **Dynamic Time Allocation**: Remaining planning time is dynamically allocated to each goal in multi-goal scenarios
- **Best Effort Planning**: Returns best path found when time limit is reached, even if goal not exactly reached
- **Fallback Strategy**: If exact goal unreachable within time limit, returns path to closest viable point within 3x goal tolerance
- **Enhanced Logging**: Added detailed timing information and timeout notifications
- **Test Script**: Added `test_time_constraint.py` for validating time constraint functionality

### Changed
- **RRT* Algorithm**: Modified `plan_rrt_star_path()` to accept `max_planning_time` parameter
- **Main Execution Loop**: Added global timeout checking in navigation execution
- **Result Handling**: Enhanced result messages to include completion status and timeout information
- **Path Planning**: Improved path planning to track best solution found so far during iterations

### Technical Details

#### Time Constraint Implementation
1. **Global Timeout**: The entire navigation task respects `max_planning_seconds`
2. **Per-Goal Allocation**: Remaining time is calculated and passed to each goal's planning phase
3. **Iterative Improvement**: RRT* continues to improve path quality until time limit reached
4. **Graceful Termination**: Planning stops cleanly when timeout occurs, returning best path found

#### Fallback Behavior
- If exact goal cannot be reached within time limit, planner searches for closest reachable point
- Fallback is only used if closest point is within 3x the normal goal tolerance
- Clear logging indicates when fallback strategy is employed

#### Performance Considerations
- Time checking occurs once per RRT* iteration to minimize overhead
- Planning time tracking uses high-resolution ROS2 time for accuracy
- Timeout detection has minimal computational impact on algorithm performance

### Files Modified
- `include/simple_global_navigator/simple_global_navigator.hpp`: Added time parameter to planning method
- `src/simple_global_navigator.cpp`: Implemented time constraint logic throughout
- `README.md`: Updated documentation with time constraint examples and behavior
- `CMakeLists.txt`: Added test script installation
- `launch/simple_global_navigator.launch.py`: Launch file remains compatible

### Testing
- Added comprehensive test script with multiple time constraint scenarios
- Tests verify timeout compliance within 1-second tolerance
- Includes tests for short, medium, and unlimited time constraints

### Backward Compatibility
- Negative `max_planning_seconds` values disable timeout (maintains existing behavior)
- Default parameter value of -1.0 ensures existing code continues to work
- All existing functionality preserved when no time constraint specified