To implement a set of actions (sequence of movements) for the robot, I will create a C++ demo node within the `gluon_moveit_config` package. This node will use the `MoveGroupInterface` to command the robot to move between the named poses defined in the SRDF (`home_pose`, `test_pose-1`, `test_pose-2`).

Here is the implementation plan:

1.  **Create Source File**: Create a new C++ file `src/move_group_demo.cpp` inside `gluon_moveit_config`. This file will contain the code to initialize the ROS 2 node and execute the sequence of movements.
2.  **Update Dependencies**: Modify `package.xml` to add dependencies on `rclcpp` and `moveit_ros_planning_interface`.
3.  **Update Build Configuration**: Modify `CMakeLists.txt` to compile the new C++ node (`move_group_demo`) and install it.
4.  **Build and Verify**: Rebuild the package and run the demo node to verify the robot executes the sequence of actions.

This approach ensures a robust execution using the standard MoveIt 2 C++ interface, as Python bindings (`moveit_py`) might not be fully configured in this environment.
