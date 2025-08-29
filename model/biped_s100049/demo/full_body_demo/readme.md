### 太极、跳舞分支
- 没有合并到主分支之前为`fandes/dev/add-taiji-demo`

  ```bash
  git checkout fandes/dev/add-taiji-demo
  ```
### 环境配置和确认
- kuavo-ros-control能够正常运行
- 额外依赖
  - 升级pandas版本(重要)
  ```bash
  sudo su
  pip install pandas==2.0.3

  ```
  > ⚠️ 遥控器启动必须在root下执行`pip install pandas==2.0.3`，不能只安装在lab用户目录下，也可以都装一次！
  - 安装 robot_localization 包相关依赖
  ```bash
  sudo apt-get install libgeographic-dev ros-noetic-geographic* -y

  ```
- 确认手臂config增益
    - 执行 `cat ~/.config/lejuconfig/config.yaml`打印手臂电机配置，确认`parameter`项和下列一致
    ```bash
    parameter:
        # 关节参数[vel, kp_pos, kd_pos, tor, kp_vel, kd_vel, ki_vel]
        Left_joint_arm_1: [0, 25, 8, 0, 0, 0, 0]
        Left_joint_arm_2: [0, 20, 6, 0, 0, 0, 0]
        Left_joint_arm_3: [0, 20, 6, 0, 0, 0, 0] 
        Left_joint_arm_4: [0, 10, 3, 0, 0, 0, 0]
        Left_joint_arm_5: [0, 10, 3, 0, 0, 0, 0]
        Left_joint_arm_6: [0, 10, 3, 0, 0, 0, 0]
        Right_joint_arm_1: [0, 25, 8, 0, 0, 0, 0]
        Right_joint_arm_2: [0, 20, 6, 0, 0, 0, 0]
        Right_joint_arm_3: [0, 20, 6, 0, 0, 0, 0] 
        Right_joint_arm_4: [0, 10, 3, 0, 0, 0, 0]
        Right_joint_arm_5: [0, 10, 3, 0, 0, 0, 0]
        Right_joint_arm_6: [0, 10, 3, 0, 0, 0, 0]
        Head_joint_low: [0, 4, 3, 0, 0, 0, 0]
        Head_joint_high: [0, 10, 6, 0, 0, 0, 0]
    ```


### 编译
```bash
catkin build humanoid_controllers # 在仓库目录执行
```


## 使用案例
### 通过命令行运行跳舞

1. 启动机器人程序，并站立
```bash
sudo su
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real.launch
```
2. 启动跳舞程序
   - 启动自动程序，会自动执行手势舞、太极、跳舞，各段动作时间配置在`src/demo/full_body_demo/scripts/action_sequence.yaml`中
   ```bash
   source devel/setup.bash
   python3 ./src/demo/full_body_demo/scripts/timed_action_executor.py
   ```

   ！！！注意，这里的太极由于有单脚站立的动作，会导致机器人摔倒，所以无法完整运行，其余动作均可完整运行。

   - 单独启动各段动作,
   ```bash
   source devel/setup.bash
   # 太极
   python3 ./src/demo/csv2body_demo/step_player_csv_ocs2.py src/demo/csv2body_demo/actions/taiji_wuhan_step_part.csv
   # 手势舞
   python3 ./src/demo/full_body_demo/csv_trajectory_publisher.py ./src/demo/full_body_demo/motions/rbd_state_motions1.csv
   # 跳舞
   python3 ./src/demo/full_body_demo/csv_trajectory_publisher.py ./src/demo/full_body_demo/motions/rbd_states_0314_5deg_ik_8_full.csv
   ```
### 通过遥控器运行
  - 配置遥控器自启动程序
     ```bash
      cd <kuavo-ros-control>/src/humanoid-control/h12pro_controller_node/scripts
      sudo su
      ./deploy_autostart.sh
      ```

      ⚠️ **注意: h12pro 遥控器的程序以及机器人程序，vr 程序都会使用部署时的 ROS_MASTER_URI 与 ROS_IP，请确保部署时的 ROS_MASTER_URI 与 ROS_IP 正确。**

      执行后，会自动安装依赖，并启动 h12pro 遥控器程序。
  - 配置遥控器按键执行程序
    - 修改`src/humanoid-control/h12pro_controller_node/config/customize_config.json`
    - 将命令行启动跳舞的命令添加到想要的按键下，类型改为`shell`, 例如绑定按键为`拨杆都摆到右侧+D键`可以这样配置
      ```json
      "customize_action_RR_D": {
        "type": "shell",
        "command": "python3 /home/lab/kuavo-ros-control/src/demo/full_body_demo/scripts/timed_action_executor.py > /home/lab/executor_output.txt 2>&1"
      },
      ```
    - 修改后保存，重新启动遥控器服务之后即可生效
      ```bash
      sudo systemctl restart ocs2_h12pro_monitor.service
      ```
