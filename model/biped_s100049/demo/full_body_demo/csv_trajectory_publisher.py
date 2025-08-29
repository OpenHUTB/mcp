#!/usr/bin/env python3

import rospy
import numpy as np
import subprocess
import sys
# 检查pandas版本
try:
    import pandas as pd
    current_version = pd.__version__
    required_version = '2.0.3'
    
    if current_version != required_version:
        rospy.logwarn(f"当前pandas版本 {current_version} 与要求版本 {required_version} 不符")
        rospy.loginfo(f"正在安装pandas {required_version}")
        subprocess.check_call([sys.executable, "-m", "pip", "install", f"pandas=={required_version}"])
        rospy.loginfo("pandas安装完成，请重新运行程序")
    import pandas as pd
except ImportError:
    rospy.logwarn("未检测到pandas，正在安装...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", f"pandas==2.0.3"])
    rospy.loginfo("pandas安装完成，请重新运行程序")
    import pandas as pd

import argparse
from kuavo_msgs.msg import fullBodyTargetTrajectories, qv
from ocs2_msgs.msg import mpc_observation

class CsvTrajectoryPublisher:
    """CSV轨迹发布器,用于读取CSV文件中的轨迹数据并发布到ROS话题"""
    
    def __init__(self, dt=0.01, publish_rate=100, start_frame=None, end_frame=None, time_offset=None):
        # 初始化ROS节点
        if not rospy.core.is_initialized():
            rospy.init_node('csv_trajectory_publisher', anonymous=True)
        
        # 创建发布者
        self.trajectory_pub = rospy.Publisher(
            '/humanoid_mpc_fullbody_target_trajectories',
            fullBodyTargetTrajectories,
            queue_size=10
        )
        
        # 保存参数
        self.publish_rate = publish_rate  # Hz
        self.dt = dt  # 时间步长
        self.start_frame = start_frame
        self.end_frame = end_frame
        self.time_offset = time_offset  # 重命名：起始时间改为时间偏移
        
        # MPC时间相关
        self.mpc_time = None
        self.mpc_time_received = False
        
        # 预加载的消息
        self.trajectory_msg = None
        
        # 订阅MPC观测话题
        self.mpc_obs_sub = rospy.Subscriber(
            '/humanoid_mpc_observation',
            mpc_observation,
            self.mpc_observation_callback
        )
        
        # 定义接触模式映射关系
        self.contact_mode_map = {
            0: 2,  # 双脚支撑 -> 2
            1: 1,  # 左脚支撑 -> 0
            2: 0   # 右脚支撑 -> 1
        }

    def mpc_observation_callback(self, msg):
        """MPC观测回调函数"""
        self.mpc_time = msg.time
        self.mpc_time_received = True
        
    def wait_for_mpc_time(self, timeout=5.0):
        """等待接收到MPC时间
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            bool: 是否成功接收到MPC时间
        """
        timeout_time = rospy.Time.now() + rospy.Duration(timeout)
        while not self.mpc_time_received:
            if rospy.Time.now() >= timeout_time:
                rospy.logerr("等待MPC时间超时")
                return False
            rospy.sleep(0.1)
        return True

    def load_csv_data(self, csv_file):
        """加载CSV文件数据并验证数据格式"""
        try:
            # 读取CSV文件
            df = pd.read_csv(csv_file)
            
            # 验证数据维度
            base_columns = 1+33+32  # 1(接触相) + 33(广义位置) + 32(广义速度)
            head_columns = 4  # 头部数据的列数
            expected_columns = base_columns + head_columns  # 完整数据的列数
            
            if len(df.columns) == base_columns:
                rospy.loginfo("检测到无头部数据，将自动补充零值")
                # 在广义位置和广义速度后分别添加两列零值
                q_head_zeros = pd.DataFrame(0, index=df.index, columns=['head_q1', 'head_q2'])
                v_head_zeros = pd.DataFrame(0, index=df.index, columns=['head_v1', 'head_v2'])
                
                # 分割原始数据
                contact_df = df.iloc[:, 0]
                q_df = df.iloc[:, 1:34]  # 33列广义位置
                v_df = df.iloc[:, 34:]   # 32列广义速度
                
                # 重新组合数据
                df = pd.concat([contact_df, q_df, q_head_zeros, v_df, v_head_zeros], axis=1)
                
            elif len(df.columns) != expected_columns:
                rospy.logerr(f"数据维度错误: 期望{expected_columns}列或{base_columns}列，实际{len(df.columns)}列")
                return None
                
            # 验证接触相并转换为整数
            valid_contacts = {0, 1, 2}  # 有效的接触相值
            contact_phase = df.iloc[:, 0].round().astype(int)  # 四舍五入并转为整数
            invalid_contacts = set(contact_phase.unique()) - valid_contacts
            if invalid_contacts:
                rospy.logerr(f"错误：发现无效的接触相值: {invalid_contacts}")
                return None
                
            # 更新第一列为转换后的整数值
            df.iloc[:, 0] = contact_phase
            
            # 应用帧范围过滤
            if self.start_frame is not None or self.end_frame is not None:
                start_idx = self.start_frame if self.start_frame is not None else 0
                end_idx = self.end_frame if self.end_frame is not None else len(df)
                
                if start_idx < 0:
                    start_idx = len(df) + start_idx
                if end_idx < 0:
                    end_idx = len(df) + end_idx
                    
                df = df.iloc[start_idx:end_idx].reset_index(drop=True)
                rospy.loginfo(f"截取数据帧范围: [{start_idx}, {end_idx}), 总帧数: {len(df)}")
            
            rospy.loginfo(f"成功加载CSV文件: {csv_file}")
            return df
        except Exception as e:
            rospy.logerr(f"加载CSV文件失败: {str(e)}")
            return None
            
    def create_qv_msg(self, values):
        """创建qv消息"""
        msg = qv()
        msg.value = values.tolist()
        return msg

    def process_trajectory_data(self, df):
        """处理轨迹数据，转换为Drake格式的广义位置和速度
        
        Drake格式:
        - 广义位置q: [quat_w, quat_x, quat_y, quat_z, pos_x, pos_y, pos_z, joints...]
        - 广义速度v: [angular_vel_x, angular_vel_y, angular_vel_z, vel_x, vel_y, vel_z, joint_velocities...]
        
        Returns:
            tuple: (foot_indices, q_trajectories, v_trajectories)
        """
        n_q = 35  # 广义位置的维度
        q_start = 1  # 位置数据开始的列索引
        v_start = q_start + n_q  # 速度数据开始的列索引
        
        # 获取接触模式序列并进行映射转换
        raw_indices = df.iloc[:, 0].astype(int).tolist()
        foot_indices = [self.contact_mode_map[idx] for idx in raw_indices]
        
        # 准备轨迹列表
        q_trajectories = []
        v_trajectories = []
        
        rospy.loginfo(f"处理数据: 总帧数={len(df)}, 每帧数据长度={len(df.iloc[0, :])}")
        
        for i in range(len(df)):
            # 提取原始数据
            raw_q = df.iloc[i, q_start:q_start+n_q].values
            raw_v = df.iloc[i, v_start:v_start+n_q].values
            
            # 重排广义位置顺序: [quat_w, quat_x, quat_y, quat_z, pos_x, pos_y, pos_z, joints...]
            drake_q = np.zeros_like(raw_q)
            drake_q[0:4] = raw_q[3:7]      # 四元数 [w,x,y,z]
            drake_q[4:7] = raw_q[0:3]      # 位置 [x,y,z]
            drake_q[7:] = raw_q[7:]        # 关节角度
            
            # 重排广义速度顺序: [angular_vel_x, angular_vel_y, angular_vel_z, vel_x, vel_y, vel_z, joint_velocities...]
            drake_v = np.zeros_like(raw_v)
            drake_v[0:3] = raw_v[3:6]      # 角速度 [wx,wy,wz]
            drake_v[3:6] = raw_v[0:3]      # 线速度 [vx,vy,vz]
            drake_v[6:] = raw_v[6:]        # 关节速度
            
            # 创建消息并添加到轨迹列表
            q_trajectories.append(self.create_qv_msg(drake_q))
            v_trajectories.append(self.create_qv_msg(drake_v))
            
        print(f"q_trajectories: {len(q_trajectories)}")
        print(f"v_trajectories: {len(v_trajectories)}")
        print(f"foot_indices: {len(foot_indices)}")
        # print(f"q_trajectories[0]: {q_trajectories[0]}")
        # print(f"v_trajectories[0]: {v_trajectories[0]}")
        # print(f"原始接触模式: {raw_indices[0]} -> 映射后: {foot_indices[0]}")
        return foot_indices, q_trajectories, v_trajectories
        
    def create_trajectory_msg(self, df):
        """创建轨迹消息"""
        msg = fullBodyTargetTrajectories()
        
        # 生成时间序列
        num_frames = len(df)
        msg.timeTrajectory = [i * self.dt for i in range(num_frames)]
        
        # 处理轨迹数据
        foot_indices, q_trajectories, v_trajectories = self.process_trajectory_data(df)
        
        # 填充消息
        msg.footIndexTrajectory = foot_indices
        msg.fullBodyQTrajectory = q_trajectories
        msg.fullBodyVTrajectory = v_trajectories
        
        rospy.loginfo(f"轨迹总帧数: {num_frames}, 总时长: {num_frames * self.dt:.2f}秒")
        return msg

    def load_action_with_csv(self, csv_file, dt=None, start_frame=None, end_frame=None):
        """加载CSV文件并生成消息
        
        Args:
            csv_file: CSV文件路径
            dt: 时间步长，如果为None则使用初始化时的值
            start_frame: 起始帧索引，如果为None则使用初始化时的值
            end_frame: 结束帧索引，如果为None则使用初始化时的值
            
        Returns:
            bool: 是否成功加载并生成消息
        """
        start_time = rospy.Time.now()
        if not csv_file:
            rospy.logerr("未指定CSV文件路径")
            return False
            
        # 更新参数
        if dt:
            self.dt = dt
        if start_frame is not None:
            self.start_frame = start_frame
        if end_frame is not None:
            self.end_frame = end_frame
            
        # 加载数据
        df = self.load_csv_data(csv_file)
        if df is None:
            return False
        
        load_time = rospy.Time.now()
        # 创建消息
        self.trajectory_msg = self.create_trajectory_msg(df)
        create_time = rospy.Time.now()
        print(f"加载数据完成，耗时: {load_time.to_sec() - start_time.to_sec():.2f}秒")
        print(f"创建消息完成，耗时: {create_time.to_sec() - load_time.to_sec():.2f}秒")
        return True

    def run(self, time_offset=None):
        """运行发布器
        
        Args:
            time_offset: 时间偏移量，如果为None则使用初始化时的值
        """
        start_time = rospy.Time.now()
        # 检查是否已加载消息
        if self.trajectory_msg is None:
            rospy.logerr("未加载CSV文件，请先调用load_action_with_csv")
            return False
            
        # 更新时间偏移
        if time_offset is not None:
            self.time_offset = time_offset
            
        # 设置调度参数
        if self.time_offset is not None and self.time_offset > 0:
            rospy.set_param('/mpc/schedule/enable', True)
            rospy.set_param('/mpc/schedule/start_time', self.mpc_time + self.time_offset)
            rospy.loginfo(f"设置调度参数: 启用=True, 起始时间={self.mpc_time + self.time_offset:.2f}")
        else:
            rospy.set_param('/mpc/schedule/enable', False)
        
        rospy.loginfo("开始发布轨迹数据...")
        self.trajectory_pub.publish(self.trajectory_msg)
        print(f"发布轨迹数据完成，耗时: {rospy.Time.now().to_sec() - start_time.to_sec():.2f}秒")
        rospy.loginfo("轨迹数据发布完成")
        return True

def main():
    # 创建命令行参数解析器
    parser = argparse.ArgumentParser(description='CSV轨迹发布器')
    parser.add_argument('csv_file', type=str, help='CSV文件路径')
    parser.add_argument('--dt', type=float, default=0.01, help='时间步长(秒),默认0.01')
    parser.add_argument('--rate', type=float, default=100, help='发布频率(Hz),默认100')
    parser.add_argument('-s', '--start-frame', type=int, help='起始帧索引(支持负数表示倒数)')
    parser.add_argument('-t', '--end-frame', type=int, help='结束帧索引(支持负数表示倒数)')
    parser.add_argument('--time-offset', type=float, help='轨迹开始时间的偏移量(秒)。正值表示在当前MPC时间基础上延迟执行，负值或当前时间之前的值将被忽略（使用默认值-1）')
    
    # 解析命令行参数
    args = parser.parse_args()
    
    try:
        publisher = CsvTrajectoryPublisher(
            dt=args.dt,
            publish_rate=args.rate,
            start_frame=args.start_frame,
            end_frame=args.end_frame,
            time_offset=args.time_offset
        )
        
        # 等待ROS系统就绪
        rospy.sleep(1)
        
        # 加载CSV文件并运行
        if publisher.load_action_with_csv(args.csv_file):
            publisher.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 
