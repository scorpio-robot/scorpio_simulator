# scorpio_simulator

## 1. Overview

scorpio_simulator 是基于 Gazebo (Ignition 字母版本) 的仿真环境

目前 scorpio_simulator 提供以下功能：

- warehouse, turtlebot3_dqn, turtlebot3_house 等仿真世界模型

- ydlidar, RealSense D435i, Livox mid360 等传感器的仿真

- GlobalOdometryPublisher 插件：发布机器人精确位姿（ground truth）用于对比和验证

## 2. Quick Start

### 2.1 Setup Environment

Ubuntu 22.04: [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

> [!NOTE]
> 由于使用了 RGLGazeboPlugin 用于仿真 mid360 点云,仿真包仅可运行在带 Nvidia GPU 的电脑

### 2.2 Create Workspace

### 2.2.1 Clone

```sh
pip3 install vcs2l
pip3 install xmacro
```

```sh
mkdir -p ~/scorpio_ws
cd ~/scorpio_ws
```

```sh
git clone https://github.com/scorpio-robot/scorpio_simulator src/scorpio_simulator
```

```sh
vcs import src < src/scorpio_simulator/dependencies.repos
```

#### 2.2.3 Build

```sh
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
```

### 2.3 Running

启动仿真环境

```sh
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

> [!NOTE]
> **注意：需要点击 Gazebo 左下角橙红色的 `启动` 按钮**

#### 2.3.1 Test Commands

控制机器人移动

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 2.3.2 查看 Ground Truth 数据

查看机器人精确位姿（如果模型加载了 GlobalOdometryPublisher 插件）：

```sh
ros2 topic echo /ground_truth/odom
```

#### 2.3.3 切换仿真世界

修改 [gz_world.yaml](./rmu_gazebo_simulator/config/gz_world.yaml) 中的 `world`。当前可选: `rmul_2024`, `rmuc_2024`, `rmul_2025`, `rmuc_2025`


## 3. Gazebo Plugins

### 3.1 GlobalOdometryPublisher Plugin

GlobalOdometryPublisher 是一个用于发布机器人任意 link 在世界坐标系下的位姿和速度的插件

#### 功能特性

- 发布精确的 3D 位姿和速度信息（无噪声或可配置噪声）
- 支持世界坐标系或相对于其他链接的坐标系
- 可配置的更新频率
- 支持位姿偏移
- 灵活的坐标系配置（分离 Gazebo 内部坐标系和 ROS 消息坐标系）

#### 参数说明

| 参数名 | 类型 | 必需 | 默认值 | 说明 |
| ------ | ---- | ---- | ------ | ---- |
| `robot_namespace` | string | 否 | "" | ROS 节点命名空间 |
| `gazebo_child_frame` | string | 是 | - | Gazebo 中要跟踪的链接名称 |
| `gazebo_frame` | string | 否 | "world" | Gazebo 参考坐标系（world 或其他链接） |
| `topic_name` | string | 是 | - | ROS 话题名称 |
| `ros_frame_id` | string | 否 | "odom" | ROS Odometry 消息的 frame_id |
| `ros_child_frame_id` | string | 否 | gazebo_child_frame | ROS Odometry 消息的 child_frame_id |
| `local_twist` | bool | 否 | false | 是否在局部坐标系中计算速度 |
| `xyz_offset` | Vector3d | 否 | [0, 0, 0] | 位置偏移 (米) |
| `rpy_offset` | Vector3d | 否 | [0, 0, 0] | 姿态偏移 (弧度) |
| `gaussian_noise` | double | 否 | 0.0 | 高斯噪声标准差 |
| `update_rate` | double | 否 | 0.0 | 更新频率 (Hz)，0 表示最快 |

#### 使用示例

```xml
<plugin filename="GlobalOdometryPublisher" name="ignition::gazebo::systems::GlobalOdometryPublisher">
  <!-- Gazebo 坐标系配置 -->
  <gazebo_child_frame>base_link</gazebo_child_frame>
  <gazebo_frame>world</gazebo_frame>

  <!-- ROS 话题和坐标系配置 -->
  <topic_name>ground_truth/odom</topic_name>
  <ros_frame_id>odom</ros_frame_id>
  <ros_child_frame_id>base_footprint</ros_child_frame_id>

  <!-- 更新设置 -->
  <update_rate>50</update_rate>
  <gaussian_noise>0.01</gaussian_noise>

  <!-- 可选的偏移 -->
  <xyz_offset>0 0 0.1</xyz_offset>
  <rpy_offset>0 0 0</rpy_offset>
</plugin>
```
