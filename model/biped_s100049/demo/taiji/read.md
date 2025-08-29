#  全身打太极动作案例

- [全身打太极动作案例](#全身打太极动作案例)

  - [1. 描述](#1-描述)

  - [2. 文件结构](#2-文件结构)

  - [3. 运行步骤](#3-运行步骤)

  - [4. ROS接口](#4-ros接口)

    - [4.1 发布的话题](#41-发布的话题)

    - [`/humanoid_mpc_foot_pose_target_trajectories`](#humanoid_mpc_foot_pose_target_trajectories)

    - [`/kuavo_arm_target_poses`](#kuavo_arm_target_poses)

    - [4.2 订阅的话题](#42-订阅的话题)

    - [`/humanoid_mpc_observation`](#humanoid_mpc_observation)

    - [`/humanoid_mpc_gait_time_name`](#humanoid_mpc_gait_time_name)

    - [4.3 服务](#43-服务)

    - [`/humanoid_change_arm_ctrl_mode`](#humanoid_change_arm_ctrl_mode)

  - [5. 使用指南](#5-使用指南)

    - [5.1 动作文件格式说明](#51-动作文件格式说明)

    - [5.1.1 步态控制数据](#511-步态控制数据)

    - [5.1.2 手臂动作数据](#512-手臂动作数据)

    - [5.2 动作执行流程](#52-动作执行流程)

  - [6. 补充说明](#6-补充说明)

## 1. 描述

本案例是一个ROS节点，用于播放预定义的机器人动作序列。动作序列存储在JSON文件中，包括步态控制（足部运动）和手臂动作。节点通过ROS话题和服务与MPC控制器及手臂控制器通信，实现动作的按序执行。

- ***建议先进行全身机器人关节标定，确保机器人状况良好，先在挂绳的条件下运行此案例，足够稳定后再考虑脱绳运行。并且由于机器人会有小幅度下蹲，移位机需要下降至挂绳架子与机器人头部约1cm高度。运行此案例时机器人会向左踏出一步，注意不要碰到障碍物，如果需要多次重复运行此案例，需要在启动下位机程序后面增加参数 <joystick_type:=h12>（用遥控器启动机器人无需再次启动下位机程序） ，每重复运行2~3次太极动作需要通过遥控器控制机器人原地踏步一会以矫正机器人姿态。***

- 示例代码位于：`<kuavo-ros-opensource>/src/demo/taiji/action_player`

- 主要文件：

- `action_player.py`: 动作播放器主程序

- `taiji.json`: 预定义的太极动作序列文件
## 2. 文件结构

```

taiji/

├── action_player.py          # 动作播放器主程序

└── taiji.json                # 预定义的太极动作序列

```

## 3. 运行步骤

1. 终端1：启动下位机程序（实物运行，若已使用操控器启动机器人无需执行此步骤）：

```bash

cd ~/kuavo-ros-opensource && sudo su
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real.launch

```

2. 终端1：启动下位机程序（仿真运行）：

```bash

cd ~/kuavo-ros-opensource && sudo su
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch

```

3. 终端2：运行动作播放器节点：

```bash

cd ~/kuavo-ros-opensource
source devel/setup.bash
python3 src/demo/taiji/actions_player.py

```

注意：在运行前，确保`taiji.json`文件位于同一目录下，或者修改代码中的路径。

- 运行效果

<iframe src="//player.bilibili.com/player.html?isOutside=true&aid=114737803170975&bvid=BV1QdKgzjEnn&cid=30673735033&p=1" 
        width="320" height="320" 
        scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true">
</iframe>

## 4. ROS接口

### 4.1 发布的话题

#### `/humanoid_mpc_foot_pose_target_trajectories`

- **消息类型**: `kuavo_msgs/footPoseTargetTrajectories`

- **描述**: 发布足部目标轨迹，用于MPC控制器控制机器人的步态。

#### `/kuavo_arm_target_poses`

- **消息类型**: `kuavo_msgs/armTargetPoses`

- **描述**: 发布手臂目标位姿，控制手臂关节角度。

### 4.2 订阅的话题

#### `/humanoid_mpc_observation`

- **消息类型**: `ocs2_msgs/mpc_observation`

- **描述**: 订阅MPC的观测信息，获取当前MPC时间。

#### `/humanoid_mpc_gait_time_name`

- **消息类型**: `kuavo_msgs/gaitTimeName`

- **描述**: 订阅步态时间信息，获取步态开始时间。

### 4.3 服务

#### `/humanoid_change_arm_ctrl_mode`

- **服务类型**: `kuavo_msgs/changeArmCtrlMode`

- **描述**: 调用此服务设置手臂控制模式为外部控制（模式2）。

## 5. 使用指南

### 5.1 动作文件格式说明

动作文件为JSON格式，包含两个主要部分：`step_control`（步态控制）和`arm_motion`（手臂动作）。

#### 5.1.1 步态控制数据

`step_control`是一个数组，每个元素代表一个步态控制点：

- `time`: 该步态的持续时间（<=1秒）

- `comment`: 注释

- `mode`: 步态模式（SS: 双脚支撑, FS: 左脚单支撑, SF: 右脚单支撑）

- `torso_pose`: 躯干位姿 [x, y, z, roll, pitch, yaw]（单位：米和度）

- `foot_positions`: 脚的位置 [x, y, z, yaw]（单位：米和度）


#### 5.1.2 手臂动作数据

`arm_motion`是一个数组，每个元素代表一个手臂动作点：

- `time`: 该动作点的持续时间（秒）

- `comment`: 注释

- `angles`: 14个手臂关节的角度值（度），顺序为左臂7个关节，右臂7个关节。

### 5.2 动作执行流程

1. 加载JSON动作文件。

2. 初始化ROS节点，创建发布者和订阅者。

3. 设置手臂为外部控制模式（通过服务调用）。

4. 生成足部轨迹并发布到`/humanoid_mpc_foot_pose_target_trajectories`。

5. 等待接收到步态开始时间。

6. 生成手臂轨迹并发布到`/kuavo_arm_target_poses`（时间基于步态开始时间）。

7. 等待动作执行完成。

## 6. 补充说明

- 本案例中的动作文件`taiji.json`包含了一个完整的太极拳动作序列，包括躯干移动、脚步移动和手臂动作。

- 在实际运行中，需要确保MPC控制器和手臂控制器正常运行，并且机器人处于合适的初始状态。

- 动作执行过程中，节点会实时监听MPC的观测信息和步态时间，确保动作的同步。