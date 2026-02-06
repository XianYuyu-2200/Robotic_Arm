# Gluon Robot ROS 2 Humble Workspace

This repository contains the ROS 2 Humble migration and configuration for the Gluon robot arm. It includes the hardware interface, MoveIt 2 configuration, and a C++ demo application for motion planning.

## 🤖 Features

*   **ROS 2 Humble Support**: Fully migrated packages compatible with ROS 2 Humble on Ubuntu 22.04.
*   **MoveIt 2 Integration**: Complete MoveIt 2 configuration (`gluon_moveit_config`) including SRDF, kinematics, and OMPL planning pipeline.
*   **Hardware Interface**: `ros2_control` hardware interface for Gluon actuators.
*   **Demo Application**: A C++ `move_group` demo node that executes a sequence of predefined poses.

## 🛠️ Prerequisites

*   **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
*   **ROS Distro**: ROS 2 Humble Hawksbill
*   **Dependencies**:
    *   `moveit`
    *   `ros2_control`
    *   `ros2_controllers`
    *   `xacro`

## 📦 Installation & Build

1.  **Clone the repository**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    # Clone this repository into src/
    git clone <your-repository-url> .
    ```

2.  **Install dependencies**
    ```bash
    sudo apt update
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the workspace**
    ```bash
    colcon build
    ```

4.  **Source the environment**
    ```bash
    source install/setup.bash
    ```

## 🚀 Usage

### 1. Launch Robot & MoveIt 2
This launch file starts the `ros2_control` node, `robot_state_publisher`, MoveIt 2 `move_group`, and RViz.

```bash
ros2 launch gluon_moveit_config demo.launch.py
```

### 2. Run Motion Planning Demo
In a separate terminal, run the C++ demo node. This node commands the robot to move through a sequence of poses: **Home -> Test Pose 1 -> Test Pose 2 -> Home**.

```bash
source install/setup.bash
ros2 launch gluon_moveit_config move_group_demo.launch.py
```

## 📂 Package Overview

*   **`gluon`**: Contains the URDF description, meshes, and basic configuration.
*   **`gluon_control`**: Implements the `SystemInterface` for `ros2_control` to communicate with the hardware SDK.
*   **`gluon_moveit_config`**: Generated and customized MoveIt 2 configuration package.
    *   `launch/`: Launch files for demo and real hardware.
    *   `config/`: SRDF, kinematics, and controller configurations.
    *   `src/move_group_demo.cpp`: Example C++ node for trajectory execution.

## 🔧 Troubleshooting

*   **"Address already in use" Error**:
    If you encounter this error when restarting launch files, it means previous nodes haven't shut down completely. Run:
    ```bash
    pkill -f ros2_control_node
    pkill -f move_group
    pkill -f robot_state_publisher
    pkill -f spawner
    ```

*   **RViz Display Issues**:
    If running in a headless environment or without a display, RViz might fail to launch. This is expected and does not affect the core motion planning functionality.

## 📝 License

This project is licensed under the BSD License.
