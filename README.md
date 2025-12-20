# scorpio_simulator

## 1. Overview

scorpio_simulator 是基于 Gazebo (Ignition 字母版本) 的仿真环境

目前 rmu_gazebo_simulator 提供以下功能：

- warehouse, turtlebot3_dqn, turtlebot3_house 仿真世界模型

- ydlidar, RealSense D435i, Livox mid360 等传感器的仿真

| rmul_2024 | rmuc_2024 |
|:-----------------:|:--------------:|
|![spin_nav.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/spin_nav.1ove3nw63o.gif)|![rmuc_fly.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/rmuc_fly_image.1aoyoashvj.gif)|

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

#### 2.3.3 切换仿真世界

修改 [gz_world.yaml](./rmu_gazebo_simulator/config/gz_world.yaml) 中的 `world`。当前可选: `rmul_2024`, `rmuc_2024`, `rmul_2025`, `rmuc_2025`
