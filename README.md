# Gluon 机械臂 ROS 2 Humble 工作空间

本仓库包含 Gluon 机械臂在 Ubuntu 22.04 上适配 ROS 2 Humble 的完整配置。项目整合了底层硬件接口、MoveIt 2 运动规划配置，并提供了一个 C++ 编写的运动控制演示程序。

## 🤖 主要功能

*   **支持 ROS 2 Humble**：所有软件包已完全迁移并兼容 Ubuntu 22.04 环境下的 ROS 2 Humble。
*   **集成 MoveIt 2**：提供完整的 MoveIt 2 配置包 (`gluon_moveit_config`)，包含 SRDF、运动学参数及 OMPL 规划管线。
*   **硬件接口**：基于 `ros2_control` 实现的 Gluon 执行器硬件通信接口。
*   **演示程序**：包含一个基于 C++ `move_group` 接口的演示节点，可执行预定义的动作序列。

## 🛠️ 环境要求

*   **操作系统**: Ubuntu 22.04 LTS (Jammy Jellyfish)
*   **ROS 版本**: ROS 2 Humble Hawksbill
*   **依赖包**:
    *   `moveit`
    *   `ros2_control`
    *   `ros2_controllers`
    *   `xacro`

## 📦 安装与编译

1.  **克隆仓库**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    # 将本仓库克隆到 src/ 目录下
    git clone <仓库地址> .
    ```

2.  **安装依赖**
    ```bash
    sudo apt update
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **编译工作空间**
    ```bash
    colcon build
    ```

4.  **配置环境变量**
    ```bash
    source install/setup.bash
    ```

## 🚀 使用指南

### 1. 启动机械臂与 MoveIt 2
此启动文件会同时加载 `ros2_control` 节点、`robot_state_publisher`、MoveIt 2 `move_group` 以及 RViz 可视化界面。

```bash
ros2 launch gluon_moveit_config demo.launch.py
```

### 2. 运行运动规划演示
在新的终端窗口中运行以下命令。该演示程序将控制机械臂执行：**复位 (Home) -> 测试姿态 1 -> 测试姿态 2 -> 复位 (Home)** 的循环动作。

```bash
source install/setup.bash
ros2 launch gluon_moveit_config move_group_demo.launch.py
```

## 📂 软件包概览

*   **`gluon`**: 包含机械臂的 URDF 描述文件、模型网格 (Meshes) 及基础配置。
*   **`gluon_control`**: 实现了 `ros2_control` 的 `SystemInterface`，用于与底层 SDK 进行通信。
*   **`gluon_moveit_config`**: 生成并定制化的 MoveIt 2 配置包。
    *   `launch/`: 包含演示和实机运行的启动文件。
    *   `config/`: SRDF、运动学参数和控制器配置文件。
    *   `src/move_group_demo.cpp`: 轨迹规划与执行的 C++ 示例代码。

## 🔧 常见问题排查

*   **"Address already in use" (地址已被占用)**：
    如果在重启 launch 文件时遇到此错误，通常意味着上一次运行的节点没有完全退出。请运行以下命令清理残留进程：
    ```bash
    pkill -f ros2_control_node
    pkill -f move_group
    pkill -f robot_state_publisher
    pkill -f spawner
    ```

*   **RViz 显示问题**：
    如果在无头模式（无显示器）的服务器环境运行，RViz 启动失败是正常现象，不会影响核心的运动规划功能。

## 📝 许可证

本项目采用 BSD 许可证。
