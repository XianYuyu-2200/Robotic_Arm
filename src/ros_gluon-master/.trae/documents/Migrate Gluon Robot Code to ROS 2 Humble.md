I will help you migrate the `gluon` and `gluon_control` packages to **ROS 2 Humble**. This involves converting the build system to `ament_cmake`, updating the C++ source code to use `rclcpp` and `ros2_control`, and creating Python launch files.

**Important Note**: The `ActuatorController_SDK` seems to be missing from the file list I retrieved. I will assume it exists at the configured path and proceed with the code refactoring, but you may need to ensure the SDK libraries are correctly placed and linked.

### Plan: ROS 2 Migration

#### 1. Migrate `gluon` Package (Basic Driver Node)
*   **Build System**: Update `package.xml` and `CMakeLists.txt` to use `ament_cmake` and `rclcpp`.
*   **Source Code**: Refactor `src/gluonControl.cpp` to use `rclcpp::Node`.
    *   Replace `ros::init`, `ros::NodeHandle` with `rclcpp::init`, `rclcpp::Node`.
    *   Replace `ros::Subscriber` with `rclcpp::Subscription`.
    *   Update logging macros (`ROS_INFO` -> `RCLCPP_INFO`).
*   **Launch**: Create `launch/start_node.launch.py` to replace the old XML launch files.

#### 2. Migrate `gluon_control` Package (ros2_control Interface)
*   **Architecture Change**: Rewrite `GluonHWInterface` to inherit from `hardware_interface::SystemInterface` (the standard for ROS 2 robots) instead of `ros_control_boilerplate`.
*   **Implementation**:
    *   Implement `on_init`, `on_configure`, `on_activate`, `on_deactivate`.
    *   Implement `read` (read motor positions) and `write` (send commands).
    *   Implement `export_state_interfaces` (publish joint positions) and `export_command_interfaces` (accept commands).
*   **Dependencies**: Update `package.xml` and `CMakeLists.txt` to depend on `hardware_interface`, `pluginlib`, and `rclcpp`.
*   **Plugin Registration**: Export the hardware interface as a plugin.

#### 3. Configuration & Launch
*   Create a standard `ros2_control` URDF tag (in a separate xacro file or appended to existing URDF) to load the `GluonSystemInterface`.
*   Create a `bringup.launch.py` to load the robot description and start the `ros2_control_node`.

I will start by migrating the `gluon` package, then move to the more complex `gluon_control` package.
