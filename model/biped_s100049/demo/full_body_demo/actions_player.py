#!/usr/bin/env python3
import rospy
import json
import numpy as np
import os
from kuavo_sdk.msg import footPose, footPoseTargetTrajectories, armTargetPoses
from kuavo_msgs.msg import gaitTimeName
from kuavo_msgs.srv import changeArmCtrlMode
from ocs2_msgs.msg import mpc_observation

from std_msgs.msg import Float64MultiArray

class ActionPlayer:
    """动作播放器，用于控制机器人执行预定义的动作序列"""
    
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('action_player', anonymous=True)
        
        # 创建发布者
        self.foot_pose_pub = rospy.Publisher(
            '/humanoid_mpc_foot_pose_target_trajectories', 
            footPoseTargetTrajectories, 
            queue_size=10
        )
        self.arm_target_pub = rospy.Publisher(
            '/kuavo_arm_target_poses',
            armTargetPoses,
            queue_size=10
        )
        
        # 等待手臂控制模式服务
        rospy.loginfo("等待手臂控制模式服务...")
        rospy.wait_for_service('/humanoid_change_arm_ctrl_mode')
        self.change_arm_mode = rospy.ServiceProxy('/arm_traj_change_mode', changeArmCtrlMode)
        rospy.loginfo("手臂控制模式服务已就绪")
        
        # 动作数据
        self.step_control = None
        self.arm_motion = None
        
        # MPC时间
        self.mpc_time = None
        self.mpc_time_received = False
        
        # 步态执行时间
        self.gait_start_time = None
        self.gait_start_time_received = False
        
        # 订阅MPC观测话题
        self.mpc_obs_sub = rospy.Subscriber(
            '/humanoid_mpc_observation',
            mpc_observation,
            self.mpc_observation_callback
        )
        
        # 订阅步态时间话题
        self.gait_time_sub = rospy.Subscriber(
            '/humanoid_mpc_gait_time_name',
            gaitTimeName,
            self.gait_time_callback
        )
        
    def mpc_observation_callback(self, msg):
        """MPC观测回调函数"""
        self.mpc_time = msg.time
        self.mpc_time_received = True
        
    def gait_time_callback(self, msg):
        """步态时间回调函数"""
        if self.mpc_time_received and msg.gait_name == "custom_gait":
            self.gait_start_time = msg.start_time
            self.gait_start_time_received = True
            rospy.loginfo(f"收到步态开始时间: {self.gait_start_time}, MPC当前时间: {self.mpc_time}")
        
    def load_action(self, json_file):
        """从JSON文件加载动作数据"""
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
                
            self.step_control = data['step_control']
            self.arm_motion = data['arm_motion']
            return True
        except Exception as e:
            rospy.logerr(f"加载动作数据失败: {str(e)}")
            return False
            
    def generate_foot_trajectory(self):
        """生成足部轨迹，使用累积时间"""
        if not self.step_control:
            rospy.logerr("未加载步态控制数据")
            return None
            
        msg = footPoseTargetTrajectories()
        msg.timeTrajectory = []
        msg.footIndexTrajectory = []
        msg.footPoseTrajectory = []
        msg.swingHeightTrajectory = []  # 初始化摆动高度轨迹
        
        current_time = 0.0
        for step in self.step_control:
            # 累加持续时间
            current_time += step['time']
            mode = step['mode']
            torso_pose = step['torso_pose']
            foot_pos = step['foot_positions']
            foot_pos[3] = foot_pos[3] * np.pi / 180.0
            
            msg.timeTrajectory.append(current_time)
            msg.swingHeightTrajectory.append(step['swing_height'])  # 添加摆动高度
            
            # 根据支撑模式确定移动的脚
            if mode == "SS":  # 双脚支撑
                msg.footIndexTrajectory.append(2)
            elif mode == "SF": 
                msg.footIndexTrajectory.append(1) 
            else:  # "FS" 
                msg.footIndexTrajectory.append(0)
            
            # 创建脚部位姿消息
            foot_pose = footPose()
            foot_pose.footPose = foot_pos  # [x, y, z, yaw]
            foot_pose.torsoPose = [torso_pose[0],torso_pose[1],torso_pose[2],torso_pose[5]*np.pi/180]  # [x, y, z, roll, pitch, yaw]
            msg.footPoseTrajectory.append(foot_pose)
        print(msg)
        return msg
    
    def generate_arm_trajectory(self, start_time):
        """生成手臂轨迹
        
        参数:
            start_time: float, 步态开始执行的时间
        """
        if not self.arm_motion:
            rospy.logerr("未加载手臂动作数据")
            return None
            
        msg = armTargetPoses()
        msg.times = []
        msg.values = []
        
        current_time = start_time
        for motion in self.arm_motion:
            # 累加持续时间
            current_time += motion['time']
            msg.times.append(current_time)
            # 将角度值转换为弧度
            angles_rad = [angle for angle in motion['angles']]
            msg.values.extend(angles_rad)
        print(msg)
        return msg
    
    def set_arm_external_control(self):
        """设置手臂为外部控制模式"""
        try:
            # 2 表示外部控制模式
            response = self.change_arm_mode(2)
            if hasattr(response, 'result'):
                if response.result:
                    rospy.loginfo("成功切换到手臂外部控制模式")
                    return True
            elif hasattr(response, 'success'):
                if response.success:
                    rospy.loginfo("成功切换到手臂外部控制模式")
                    return True
            rospy.logerr("切换手臂控制模式失败")
            return False
        except rospy.ServiceException as e:
            rospy.logerr(f"调用手臂控制模式服务失败: {str(e)}")
            return False
            
    def execute_action(self):
        """执行动作序列"""
        if not self.step_control or not self.arm_motion:
            rospy.logerr("未加载完整的动作数据")
            return
            
        rospy.loginfo("开始执行动作序列...")
        
        # 设置手臂为外部控制模式
        if not self.set_arm_external_control():
            rospy.logerr("无法切换到手臂外部控制模式，终止执行")
            return
            
        # 等待接收到MPC时间
        timeout = rospy.Duration(5.0)  # 5秒超时
        start_wait = rospy.Time.now()
        while not self.mpc_time_received:
            if (rospy.Time.now() - start_wait) > timeout:
                rospy.logerr("等待MPC时间超时")
                return
            rospy.sleep(0.1)
            
        # 重置步态时间标志
        self.gait_start_time_received = False
        
        # 生成并发布足部轨迹
        foot_traj = self.generate_foot_trajectory()
        if foot_traj:
            self.foot_pose_pub.publish(foot_traj)
            rospy.loginfo("已发送足部轨迹")
            
        # 等待接收到步态开始时间
        timeout = rospy.Duration(5.0)  # 5秒超时
        start_wait = rospy.Time.now()
        while not self.gait_start_time_received:
            if (rospy.Time.now() - start_wait) > timeout:
                rospy.logerr("等待步态开始时间超时")
                return
            rospy.sleep(0.1)
        rospy.loginfo(f"步态开始时间: {self.gait_start_time}, 等待手臂轨迹发布")
        
        # 等待到达步态开始时间
        while self.mpc_time < self.gait_start_time:
            rospy.sleep(0.1)
        rospy.loginfo("开始发布手臂轨迹")
            
        # 生成并发布手臂轨迹（基于步态开始时间）
        arm_traj = self.generate_arm_trajectory(0)
        if arm_traj:
            self.arm_target_pub.publish(arm_traj)
            rospy.loginfo(f"已发送手臂轨迹，起始时间: {self.gait_start_time}")
            
        # 计算总持续时间
        total_step_time = sum(step['time'] for step in self.step_control)
        total_arm_time = sum(motion['time'] for motion in self.arm_motion)
        max_time = max(total_step_time, total_arm_time)
        
        rospy.loginfo(f"等待动作完成，预计耗时: {max_time}秒")
        rospy.sleep(max_time)
        rospy.loginfo("动作序列执行完成")

def list_action_files(actions_dir):
    """列出actions目录下的所有JSON文件
    
    参数:
        actions_dir: str, actions目录的路径
    返回:
        list: 所有JSON文件的列表
    """
    json_files = []
    try:
        for file in os.listdir(actions_dir):
            if file.endswith('.json'):
                json_files.append(file)
        return sorted(json_files)
    except Exception as e:
        rospy.logerr(f"读取动作文件列表失败: {str(e)}")
        return []

def select_action_file(actions_dir):
    """让用户选择要执行的动作文件
    
    参数:
        actions_dir: str, actions目录的路径
    返回:
        str: 选择的动作文件的完整路径，如果选择无效则返回None
    """
    json_files = list_action_files(actions_dir)
    if not json_files:
        rospy.logerr("actions目录中没有找到JSON文件")
        return None
        
    print("\n可用的动作文件:")
    for i, file in enumerate(json_files, 1):
        print(f"{i}. {file}")
        
    while True:
        try:
            choice = input("\n请选择要执行的动作文件 (直接回车选择第一个，q退出): ")
            if choice.lower() == 'q':
                return None
            if choice == '':  # 直接回车
                return os.path.join(actions_dir, json_files[0])
                
            index = int(choice) - 1
            if 0 <= index < len(json_files):
                return os.path.join(actions_dir, json_files[index])
            else:
                print("无效的选择，请重试")
        except ValueError:
            print("请输入有效的数字")
        except KeyboardInterrupt:
            print("\n操作已取消")
            return None

def main():
    """主函数"""
    player = ActionPlayer()
    
    # 获取脚本所在目录的路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    actions_dir = os.path.join(script_dir, "actions")
    
    while True:
        # 让用户选择动作文件
        action_file = select_action_file(actions_dir)
        if not action_file:
            print("程序已退出")
            return
            
        rospy.loginfo(f"已选择动作文件: {os.path.basename(action_file)}")
        
        # 加载动作数据
        if not player.load_action(action_file):
            continue
            
        # 等待ROS系统就绪
        rospy.sleep(1)
        
        try:
            # 执行动作序列
            player.execute_action()
            print("\n动作执行完成！")
        except rospy.ROSInterruptException:
            rospy.loginfo("动作执行被中断")
            break
        except Exception as e:
            rospy.logerr(f"执行动作时出错: {str(e)}")
            continue

if __name__ == '__main__':
    main()
