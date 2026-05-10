# Gluon Robotic Arm for ROS 2 Humble

这是一个面向 `ROS 2 Humble` 的机械臂工作空间，包含机械臂模型、`ros2_control` 硬件接口、MoveIt 2 运动规划配置，以及一个基于 YOLO 的视觉检测节点。

项目主要目录位于 `src/ros_gluon-master`，另外保留了 `src/innfos-cpp-sdk-master` 作为底层 SDK 参考代码。

## 项目特性

- 支持 `Ubuntu 22.04 + ROS 2 Humble`
- 提供 Gluon 机械臂的 `URDF`、网格模型和基础启动文件
- 集成 `ros2_control`，可用于控制器加载与状态发布
- 集成 `MoveIt 2`，可用于机械臂规划、仿真与 RViz 可视化
- 提供 `move_group_demo` 示例程序
- 提供 `gluon_vision` YOLO 检测节点，可发布 `vision_msgs/Detection2DArray`

## 目录结构

```text
Robotic_Arm-main/
├─ src/
│  ├─ ros_gluon-master/
│  │  ├─ gluon/                 # 机械臂描述、URDF、mesh、基础节点
│  │  ├─ gluon_control/         # ros2_control 硬件接口
│  │  ├─ gluon_moveit_config/   # MoveIt 2 配置与 launch
│  │  └─ gluon_vision/          # YOLO 视觉检测节点
│  └─ innfos-cpp-sdk-master/    # 底层 SDK 示例与库文件
├─ weights/                     # YOLO 模型权重目录（默认不上传到 GitHub）
├─ build/
├─ install/
└─ log/
```

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- colcon
- rosdep
- MoveIt 2
- ros2_control
- ros2_controllers
- xacro

如果你需要运行视觉检测，还需要：

- Python 3
- `ultralytics`
- `opencv-python`
- `cv_bridge`
- 相机图像话题输入

## 安装与编译

1. 克隆仓库

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/XianYuyu-2200/Robotic_Arm.git
```

2. 安装依赖

```bash
cd ~/ros2_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

3. 编译工作空间

```bash
colcon build
```

4. 加载环境

```bash
source install/setup.bash
```

## 启动 MoveIt 2 演示

这个启动文件会加载：

- `robot_state_publisher`
- `ros2_control_node`
- `joint_state_broadcaster`
- `joint_trajectory_controller`
- MoveIt `move_group`
- RViz

启动命令：

```bash
source install/setup.bash
ros2 launch gluon_moveit_config demo.launch.py
```

默认参数里启用了 `use_fake_hardware=true`，适合先做仿真和联调。

## 运行运动规划示例

项目中带有一个 `move_group_demo` 示例节点：

```bash
source install/setup.bash
ros2 launch gluon_moveit_config move_group_demo.launch.py
```

这个示例会加载机械臂描述、SRDF 和运动学参数，并运行 `gluon_moveit_config` 包中的 `move_group_demo` 可执行程序。

## 运行 YOLO 视觉检测

视觉节点位于 `gluon_vision` 包中，默认参数如下：

- 模型路径：`weights/best.pt`
- 输入图像：`/camera/camera/color/image_raw`
- 检测输出：`/detector/detections`
- 调试图像：`/detector/debug_image`

启动命令：

```bash
source install/setup.bash
ros2 launch gluon_vision yolo_detector.launch.py
```

如果需要手动指定模型：

```bash
ros2 launch gluon_vision yolo_detector.launch.py model_path:=/absolute/path/to/best.pt
```

也可以通过环境变量指定：

```bash
export GLUON_MODEL_PATH=/absolute/path/to/best.pt
```

## 重要说明

- `weights/best.pt` 没有随仓库上传，因为 GitHub 对普通文件有 100MB 限制。如果你要在其他机器运行 YOLO，请自行准备权重文件。
- `build/`、`install/`、`log/` 属于本地构建产物，不需要提交到仓库。
- 仓库中包含部分 SDK 动态库与示例文件，跨平台使用时请注意系统环境和依赖版本。

## 常见问题

### 1. `git clone` 后没有检测模型怎么办？

把你的 `best.pt` 放到项目根目录的 `weights/` 下，或者在启动时通过 `model_path:=...` 指定。

### 2. RViz 没有正常显示机械臂怎么办？

先确认已经执行：

```bash
source install/setup.bash
```

并检查 `robot_state_publisher`、`move_group`、`controller_manager` 是否都成功启动。

### 3. 控制器没有正常加载怎么办？

可以先检查以下话题和节点：

```bash
ros2 node list
ros2 topic list
ros2 control list_controllers
```

## License

本项目使用仓库中提供的 [LICENSE](./LICENSE)。
