#!/usr/bin/env python3
import numpy as np
import json
from typing import List, Dict, Tuple
import bvh
from transforms3d.euler import euler2mat, mat2euler
from transforms3d.affines import compose
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
plt.rcParams['font.sans-serif'] = ['SimHei']  # 指定默认字体为SimHei
plt.rcParams['axes.unicode_minus'] = False  # 解决保存图像时负号'-'显示为方块的问题

class BVHToTaichiConverter:
    def __init__(self):
        self.bvh_data = None
        self.joint_names = []
        self.num_frames = 0
        self.frame_time = 0.0
        
    def read_bvh(self, filename: str) -> bool:
        """读取BVH文件并验证关键关节的存在性"""
        try:
            with open(filename, 'r') as f:
                self.bvh_data = bvh.Bvh(f.read())
            
            # 获取基本信息
            self.joint_names = self.bvh_data.get_joints_names()
            self.num_frames = self.bvh_data.nframes
            self.frame_time = self.bvh_data.frame_time
            
            print("joint_names", self.joint_names)
            print("num_frames", self.num_frames)
            print("frame_time", self.frame_time)
            print("total_time", self.num_frames * self.frame_time)
            
            # 验证必需的关节是否存在
            required_joints = ["Hips", "LeftFoot", "RightFoot"]
            for joint in required_joints:
                if joint not in self.joint_names:
                    print(f"错误：找不到必需的关节 {joint}")
                    return False
            
            # 打印关节层级和通道信息
            print("\n=== 关节信息 ===")
            for joint in required_joints:
                offset = self.bvh_data.joint_offset(joint)
                channels = self.bvh_data.joint_channels(joint)
                parent = self.bvh_data.joint_parent(joint)
                print(f"\n{joint}:")
                print(f"  父关节: {parent}")
                print(f"  偏移量: {offset}")
                print(f"  通道: {channels}")
            
            return True
            
        except Exception as e:
            print(f"读取BVH文件失败: {str(e)}")
            return False
    
    def get_joint_transform_chain(self, frame: int, joint_name: str) -> List[Tuple[np.ndarray, np.ndarray]]:
        """获取从根节点到指定关节的变换链
        
        Args:
            frame: 帧索引
            joint_name: 关节名称
            
        Returns:
            List[Tuple[np.ndarray, np.ndarray]]: 变换链列表，每个元素是(position, rotation_matrix)
        """
        transforms = []
        current_joint = joint_name
        
        # 首先收集从末端到根节点的变换
        temp_transforms = []
        while current_joint is not None:
            try:
                # 获取关节偏移和通道数据
                offset = np.array(self.bvh_data.joint_offset(current_joint))
                channel_names = self.bvh_data.joint_channels(current_joint)
                channels = self.bvh_data.frame_joint_channels(frame, current_joint, channel_names)
                
                # 初始化位置和旋转
                position = np.copy(offset)  # 使用copy确保不修改原始数据
                rotation = np.zeros(3)
                
                # 处理通道数据
                for i, channel in enumerate(channel_names):
                    value = channels[i]
                    if channel.endswith('position'):
                        if current_joint == "Hips":  # 只有根节点有位置通道
                            if channel == 'Xposition':
                                position[0] = value
                            elif channel == 'Yposition':
                                position[1] = value
                            elif channel == 'Zposition':
                                position[2] = value
                    elif channel.endswith('rotation'):
                        if channel == 'Xrotation':
                            rotation[0] = np.radians(value)
                        elif channel == 'Yrotation':
                            rotation[1] = np.radians(value)
                        elif channel == 'Zrotation':
                            rotation[2] = np.radians(value)
                
                # 创建旋转矩阵（注意旋转顺序）
                rot_matrix = euler2mat(rotation[2], rotation[0], rotation[1], 'rzxy')
                temp_transforms.append((position, rot_matrix))
                
                # 移动到父关节
                parent = self.bvh_data.joint_parent(current_joint)
                current_joint = parent.name if parent is not None else None
                
            except Exception as e:
                print(f"处理关节 {current_joint} 的变换时出错: {str(e)}")
                break
        
        # 反转变换链，使其从根节点到末端
        transforms = list(reversed(temp_transforms))
        
        return transforms
    
    def compute_world_transform(self, transform_chain: List[Tuple[np.ndarray, np.ndarray]]) -> Tuple[np.ndarray, np.ndarray]:
        """计算世界坐标系中的位置和旋转
        
        Args:
            transform_chain: 从根节点到目标关节的变换链列表，每个元素是(position, rotation_matrix)元组
                           position: 局部坐标系中的位置偏移
                           rotation_matrix: 局部旋转矩阵
        
        Returns:
            Tuple[np.ndarray, np.ndarray]: (world_position, world_rotation_matrix)
                world_position: 世界坐标系中的位置 [x, y, z]
                world_rotation_matrix: 世界坐标系中的旋转矩阵 (3x3)
        """
        if not transform_chain:
            return np.zeros(3), np.eye(3)
        
        # 初始化世界变换
        world_position = transform_chain[0][0].copy()  # 根节点位置
        world_rotation = transform_chain[0][1].copy()  # 根节点旋转
        
        # 从第二个变换开始累积（因为第一个是根节点）
        for i in range(1, len(transform_chain)):
            local_pos, local_rot = transform_chain[i]
            
            # 1. 将局部位置转换到世界坐标系
            world_offset = world_rotation @ local_pos
            
            # 2. 更新世界位置
            world_position = world_position + world_offset
            
            # 3. 更新世界旋转
            world_rotation = world_rotation @ local_rot
            
            # 调试信息
            # print(f"\n调试 - 变换 {i}:")
            # print(f"局部位置: {local_pos}")
            # print(f"世界偏移: {world_offset}")
            # print(f"当前世界位置: {world_position}")
            # print(f"当前世界旋转:\n{world_rotation}")
        
        return world_position, world_rotation

    def get_joint_world_transform(self, frame: int, joint_name: str) -> Tuple[np.ndarray, np.ndarray]:
        """获取指定关节在世界坐标系中的位置和旋转
        
        Args:
            frame: 帧索引
            joint_name: 关节名称
        
        Returns:
            Tuple[np.ndarray, np.ndarray]: (world_position, world_rotation_matrix)
        """
        # 获取从根节点到目标关节的变换链
        transform_chain = self.get_joint_transform_chain(frame, joint_name)
        
        # 计算世界坐标系变换
        return self.compute_world_transform(transform_chain)
    
    def test_joint_transforms(self, frame: int = 0):
        """测试关节变换计算"""
        joints = ["Hips", "LeftFoot", "RightFoot"]
        
        print(f"\n=== 测试第 {frame} 帧的关节变换 ===")
        for joint in joints:
            print(f"\n{joint}:")
            # 获取变换链
            transform_chain = self.get_joint_transform_chain(frame, joint)
            print(f"变换链长度: {len(transform_chain)}")
            
            # 计算世界坐标系变换
            position, rotation = self.compute_world_transform(transform_chain)
            
            # 从旋转矩阵提取欧拉角 (返回rx, ry, rz)
            euler_angles = mat2euler(rotation, 'rzxy')
            rx, ry, rz = euler_angles
            
            print(f"世界坐标系位置: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
            print(f"旋转矩阵:\n{rotation}")
            print(f"欧拉角(弧度): [{rx:.3f}, {ry:.3f}, {rz:.3f}]")
            print(f"欧拉角(度): X={np.degrees(rx):.2f}, Y={np.degrees(ry):.2f}, Z={np.degrees(rz):.2f}")
            

    def convert_to_motion_data(self) -> Dict:
        """将BVH数据转换为太极动作格式
        
        Returns:
            Dict: 包含以下字段的字典：
                - step_control: 腿部和躯干控制列表
                    - time: 持续时间
                    - mode: 支撑模式 (SS: 双脚支撑, SF: 左脚支撑, FS: 右脚支撑)
                    - torso_pose: 躯干6D姿态 [x, y, z, roll, pitch, yaw]
                    - foot_positions: 悬空腿位置 [x, y, z, yaw]
                    - swing_height: 抬脚高度（相对于上次落脚点的最大高度）
                - arm_motion: 手臂运动列表
                    - time: 持续时间
                    - angles: 手臂关节角度列表
        """
        motion_data = {
            "step_control": [],
            "arm_motion": []
        }
        
        # 获取初始状态作为基准
        initial_hips_pos, initial_hips_rot = self.get_joint_world_transform(0, "Hips")
        initial_left_foot_pos, _ = self.get_joint_world_transform(0, "LeftFoot")
        initial_right_foot_pos, _ = self.get_joint_world_transform(0, "RightFoot")
        initial_left_arm_rot = self.get_joint_world_transform(0, "LeftArm")[1]
        initial_right_arm_rot = self.get_joint_world_transform(0, "RightArm")[1]
        initial_left_forearm_rot = self.get_joint_world_transform(0, "LeftForeArm")[1]
        initial_right_forearm_rot = self.get_joint_world_transform(0, "RightForeArm")[1]
        
        # 将初始位置从厘米转换为米
        initial_hips_pos = initial_hips_pos / 100.0
        initial_left_foot_pos = initial_left_foot_pos / 100.0
        initial_right_foot_pos = initial_right_foot_pos / 100.0
        initial_left_foot_pos[1] += -0.1
        initial_right_foot_pos[1] += 0.1
        
        # 首先获取所有帧的相位信息
        phases = []
        foot_positions = []  # 存储每帧的足端位置
        torso_poses = []    # 存储每帧的躯干姿态
        print("detecting contact state:", self.num_frames)
        for frame in range(self.num_frames):
            print(f"frame: {frame}/{self.num_frames}", end="\r")
            # 获取当前帧的关节位置和旋转
            hips_pos, hips_rot = self.get_joint_world_transform(frame, "Hips")
            left_foot_pos, _ = self.get_joint_world_transform(frame, "LeftFoot")
            right_foot_pos, _ = self.get_joint_world_transform(frame, "RightFoot")
            
            # 将位置从厘米转换为米
            hips_pos = hips_pos / 100.0
            left_foot_pos = left_foot_pos / 100.0
            right_foot_pos = right_foot_pos / 100.0
            
            # 计算相对于初始状态的位置差异
            rel_hips_pos = hips_pos - initial_hips_pos
            
            # 计算躯干的欧拉角
            euler_angles = mat2euler(hips_rot @ np.linalg.inv(initial_hips_rot), 'rzxy')
            rz, rx, ry = euler_angles
            
            # 构建躯干姿态 [x, y, z, roll, pitch, yaw]
            torso_pose = [
                float(rel_hips_pos[0]),     # x (m)
                float(rel_hips_pos[1]),     # y (m)
                float(rel_hips_pos[2]),     # z (m)
                float(np.degrees(rx)),      # roll (X rotation)
                float(np.degrees(ry)),      # pitch (Y rotation)
                float(np.degrees(rz))       # yaw (Z rotation)
            ]
            torso_poses.append(torso_pose)
            
            # 存储足端位置
            foot_positions.append((left_foot_pos, right_foot_pos))
            
            # 计算支撑相位
            phase = self.get_support_phase(left_foot_pos[2], right_foot_pos[2])
            phases.append(phase)
        
        # 根据相位变化划分动作段
        current_phase = phases[0]
        phase_start = 0
        last_left_height = initial_left_foot_pos[2]
        last_right_height = initial_right_foot_pos[2]
        max_swing_height = 0.0
        print("generating steps:",len(phases))
        
        for frame in range(1, len(phases)):
            print(f"frame: {frame}/{len(phases)}", end="\r")
            current_duration = (frame - phase_start) * self.frame_time
            
            # 检查是否需要分割当前步态
            need_split = False
            if current_phase == 2:  # 双脚支撑状态
                if current_duration >= 0.4:  # 超过0.4秒
                    need_split = True
            
            if phases[frame] != current_phase or frame == len(phases) - 1 or need_split:
                # 计算这段时间内的最大抬脚高度
                if current_phase == 0:  # 左脚支撑
                    swing_heights = [foot_positions[i][1][2] - last_right_height 
                                   for i in range(phase_start, frame)]
                    max_swing_height = max(swing_heights) if swing_heights else 0.0
                    last_right_height = foot_positions[frame-1][1][2]
                elif current_phase == 1:  # 右脚支撑
                    swing_heights = [foot_positions[i][0][2] - last_left_height 
                                   for i in range(phase_start, frame)]
                    max_swing_height = max(swing_heights) if swing_heights else 0.0
                    last_left_height = foot_positions[frame-1][0][2]
                
                # 获取这段时间的最终状态
                final_torso = torso_poses[frame-1]
                final_left_pos = foot_positions[frame-1][0]
                final_right_pos = foot_positions[frame-1][1]
                
                # 确定支撑模式和足端位置
                mode = "SS" if current_phase == 2 else ("SF" if current_phase == 0 else "FS")
                
                # 获取当前躯干的yaw角度
                torso_yaw = final_torso[5]  # torso_pose中的yaw角度已经是度数
                
                if mode == "SS":
                    foot_pos = [0.0, 0.0, 0.0, 0.0]
                elif mode == "SF":
                    rel_right_foot = final_right_pos - initial_right_foot_pos
                    foot_pos = [
                        float(rel_right_foot[0]),  # 已经是米为单位
                        float(rel_right_foot[1]),
                        0,
                        float(torso_yaw)  # 使用躯干的yaw角度
                    ]
                else:  # FS
                    rel_left_foot = final_left_pos - initial_left_foot_pos
                    foot_pos = [
                        float(rel_left_foot[0]),  # 已经是米为单位
                        float(rel_left_foot[1]),
                        0,
                        float(torso_yaw)  # 使用躯干的yaw角度
                    ]
                
                # 添加动作段数据
                step_data = {
                    "time": current_duration,
                    "comment": f"Frames {phase_start}-{frame}",
                    "mode": mode,
                    "torso_pose": final_torso,
                    "foot_positions": foot_pos,
                    "swing_height": float(max_swing_height)  # 已经是米为单位
                }
                motion_data["step_control"].append(step_data)
                
                # 更新状态
                if need_split and not (phases[frame] != current_phase or frame == len(phases) - 1):
                    # 如果是因为超时分割的，保持相同的相位
                    phase_start = frame
                else:
                    current_phase = phases[frame]
                    phase_start = frame
                max_swing_height = 0.0
                
        print("total steps:",len(motion_data["step_control"]))
        print("generating arms:")
        
        # 处理手臂运动（精简为每个步态的关键点）
        motion_data["arm_motion"] = []
        
        # 获取所有步态的时间点
        step_times = []
        current_time = 0
        for step in motion_data["step_control"]:
            step_times.append(current_time + step["time"] / 2)  # 中间点
            step_times.append(current_time + step["time"])      # 结束点
            current_time += step["time"]
            
        print(f"start processing arm keypoints: {len(step_times)}")
        
        # 对每个关键时间点生成手臂姿态
        for time_point in step_times:
            print(f"keypoint: {time_point:.2f}s", end="\r")
            # 将时间转换为最近的帧索引
            frame = int(time_point / self.frame_time)
            frame = min(frame, self.num_frames - 1)  # 确保不超过最大帧数
            
            try:
                # 获取手臂关节的世界坐标变换
                _, left_arm_rot = self.get_joint_world_transform(frame, "LeftArm")
                _, right_arm_rot = self.get_joint_world_transform(frame, "RightArm")
                _, left_forearm_rot = self.get_joint_world_transform(frame, "LeftForeArm")
                _, right_forearm_rot = self.get_joint_world_transform(frame, "RightForeArm")
                
                # 计算相对于初始状态的旋转
                rel_left_arm_rot = left_arm_rot @ np.linalg.inv(initial_left_arm_rot)
                rel_right_arm_rot = right_arm_rot @ np.linalg.inv(initial_right_arm_rot)
                rel_left_forearm_rot = left_forearm_rot @ np.linalg.inv(initial_left_forearm_rot)
                rel_right_forearm_rot = right_forearm_rot @ np.linalg.inv(initial_right_forearm_rot)
                
                # 从旋转矩阵提取欧拉角
                la_angles = mat2euler(rel_left_arm_rot, 'rzxy')
                ra_angles = mat2euler(rel_right_arm_rot, 'rzxy')
                lf_angles = mat2euler(rel_left_forearm_rot, 'rzxy')
                rf_angles = mat2euler(rel_right_forearm_rot, 'rzxy')
                
                # 构建手臂角度数据（转换为角度）
                arm_angles = [
                    float(np.degrees(la_angles[0])),  # 左臂X
                    float(np.degrees(la_angles[1])),  # 左臂Y
                    float(np.degrees(la_angles[2])),  # 左臂Z
                    float(np.degrees(lf_angles[0])),  # 左前臂X
                    float(np.degrees(lf_angles[1])),  # 左前臂Y
                    float(np.degrees(lf_angles[2])),  # 左前臂Z
                    0.0,                              # 左手腕
                    float(np.degrees(ra_angles[0])),  # 右臂X
                    float(np.degrees(ra_angles[1])),  # 右臂Y
                    float(np.degrees(ra_angles[2])),  # 右臂Z
                    float(np.degrees(rf_angles[0])),  # 右前臂X
                    float(np.degrees(rf_angles[1])),  # 右前臂Y
                    float(np.degrees(rf_angles[2])),  # 右前臂Z
                    0.0,                              # 右手腕
                ]
                
                # 计算相对于上一个关键点的时间间隔
                if len(motion_data["arm_motion"]) > 0:
                    time_interval = time_point - (current_time - motion_data["arm_motion"][-1]["time"])
                else:
                    time_interval = time_point
                
                arm_data = {
                    "time": float(time_interval),
                    "comment": f"Step keypoint at time {time_point:.2f}s",
                    "angles": arm_angles
                }
                motion_data["arm_motion"].append(arm_data)
                
            except Exception as e:
                print(f"处理手臂运动关键点 {time_point:.2f}s 时出错: {str(e)}")
                continue
                
        print("total arm keypoints:", len(motion_data["arm_motion"]))
        return motion_data

    def save_taichi_motion(self, output_file: str):
        """保存为太极动作JSON文件"""
        motion_data = self.convert_to_motion_data()
        print("writing json file:", output_file)
        with open(output_file, 'w') as f:
            json.dump(motion_data, f, indent=4)
        print(f"动作数据已保存到: {output_file}")

    def visualize_joints(self, frame: int):
        """可视化指定帧的所有关节位置
        
        Args:
            frame: 要可视化的帧索引
        """
        # 创建3D图形
        fig = plt.figure(figsize=(12, 12))
        ax = fig.add_subplot(111, projection='3d')
        
        # 存储所有关节的位置和名称
        positions = {}
        
        # 获取所有关节的世界坐标位置
        for joint in self.joint_names:
            try:
                position, rotation = self.get_joint_world_transform(frame, joint)
                # 调整坐标轴映射：x->x, y->y, z->z
                positions[joint] = position
                if joint in ["Hips", "LeftFoot", "RightFoot", "LeftHand", "RightHand"]:
                    print(f"{joint} position: {position}")
            except Exception as e:
                print(f"获取关节 {joint} 的位置时出错: {str(e)}")
                continue
        
        # 定义骨骼连接关系（父子关系）
        connections = []
        for joint in self.joint_names:
            parent = self.bvh_data.joint_parent(joint)
            if parent is not None:
                connections.append((parent.name, joint))
        
        # 绘制骨骼连接线
        for parent, child in connections:
            if parent in positions and child in positions:
                parent_pos = positions[parent]
                child_pos = positions[child]
                ax.plot([parent_pos[0], child_pos[0]], 
                       [parent_pos[1], child_pos[1]], 
                       [parent_pos[2], child_pos[2]], 
                       'r-', linewidth=1)
        
        # 绘制关节点
        for joint, pos in positions.items():
            ax.scatter(pos[0], pos[1], pos[2], c='b', marker='o')
            # 为重要关节添加标签
            if joint in ["Hips", "LeftFoot", "RightFoot", "LeftHand", "RightHand", "Head"]:
                ax.text(pos[0], pos[1], pos[2], joint)
        
        # 获取所有点的坐标范围
        all_positions = np.array(list(positions.values()))
        max_range = np.array([all_positions[:,0].ptp(), all_positions[:,1].ptp(), all_positions[:,2].ptp()]).max() / 2.0
        
        mid_x = (all_positions[:,0].max() + all_positions[:,0].min()) * 0.5
        mid_y = (all_positions[:,1].max() + all_positions[:,1].min()) * 0.5
        mid_z = (all_positions[:,2].max() + all_positions[:,2].min()) * 0.5
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        # 设置坐标轴标签
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        # 设置标题
        plt.title(f'人体骨骼可视化 (第{frame}帧)')
        
        # 调整视角
        ax.view_init(elev=20, azim=45)
        
        # 显示网格
        ax.grid(True)
        
        # 显示图形
        plt.show()
        
    def visualize_animation(self, start_frame: int = 0, end_frame: int = None, step: int = 1):
        """可视化动画序列，在同一个窗口中动态更新
        
        Args:
            start_frame: 起始帧
            end_frame: 结束帧（如果为None，则使用最后一帧）
            step: 帧间隔
        """
        if end_frame is None:
            end_frame = self.num_frames
            
        # 创建图形窗口
        plt.ion()  # 打开交互模式
        fig = plt.figure(figsize=(12, 12))
        ax = fig.add_subplot(111, projection='3d')
        
        for frame in range(start_frame, end_frame, step):
            ax.clear()  # 清除当前帧
            
            # 存储所有关节的位置和名称
            positions = {}
            
            # 获取所有关节的世界坐标位置
            for joint in self.joint_names:
                try:
                    position, _ = self.get_joint_world_transform(frame, joint)
                    positions[joint] = position
                except Exception as e:
                    print(f"获取关节 {joint} 的位置时出错: {str(e)}")
                    continue
            
            # 定义骨骼连接关系
            connections = []
            for joint in self.joint_names:
                parent = self.bvh_data.joint_parent(joint)
                if parent is not None:
                    connections.append((parent.name, joint))
            
            # 绘制骨骼连接线
            for parent, child in connections:
                if parent in positions and child in positions:
                    parent_pos = positions[parent]
                    child_pos = positions[child]
                    ax.plot([parent_pos[0], child_pos[0]], 
                           [parent_pos[1], child_pos[1]], 
                           [parent_pos[2], child_pos[2]], 
                           'r-', linewidth=1)
            
            # 绘制关节点
            for joint, pos in positions.items():
                ax.scatter(pos[0], pos[1], pos[2], c='b', marker='o')
                if joint in ["Hips", "LeftFoot", "RightFoot", "LeftHand", "RightHand", "Head"]:
                    ax.text(pos[0], pos[1], pos[2], joint)
            
            # 获取所有点的坐标范围
            all_positions = np.array(list(positions.values()))
            max_range = np.array([all_positions[:,0].ptp(), all_positions[:,1].ptp(), 
                                all_positions[:,2].ptp()]).max() / 2.0
            
            mid_x = (all_positions[:,0].max() + all_positions[:,0].min()) * 0.5
            mid_y = (all_positions[:,1].max() + all_positions[:,1].min()) * 0.5
            mid_z = (all_positions[:,2].max() + all_positions[:,2].min()) * 0.5
            
            # 设置坐标轴范围
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)
            
            # 设置标签
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            
            # 设置标题
            plt.title(f'人体骨骼动画 (第{frame}帧)')
            
            # 调整视角
            ax.view_init(elev=20, azim=45)
            
            # 显示网格
            ax.grid(True)
            
            # 更新显示
            plt.draw()
            plt.pause(0.01)  # 暂停一小段时间，让动画更流畅
            
        plt.ioff()  # 关闭交互模式
        plt.show()  # 保持窗口打开

    def plot_feet_height(self, start_frame: int = 0, end_frame: int = None, step: int = 3):
        """绘制足端高度随时间变化的曲线
        
        Args:
            start_frame: 起始帧
            end_frame: 结束帧
            step: 帧间隔
        """
        if end_frame is None:
            end_frame = self.num_frames
        
        frames = list(range(start_frame, end_frame, step))
        left_foot_heights = []
        right_foot_heights = []
        
        # 收集足端高度数据
        for frame in frames:
            left_foot_pos, _ = self.get_joint_world_transform(frame, "LeftFoot")
            right_foot_pos, _ = self.get_joint_world_transform(frame, "RightFoot")
            left_foot_heights.append(left_foot_pos[2])  # Z轴为高度
            right_foot_heights.append(right_foot_pos[2])
        
        # 创建图形
        plt.figure(figsize=(12, 6))
        plt.plot(frames, left_foot_heights, 'b-', label='左脚')
        plt.plot(frames, right_foot_heights, 'r-', label='右脚')
        plt.xlabel('帧数')
        plt.ylabel('高度 (Z轴)')
        plt.title('足端高度变化曲线')
        plt.legend()
        plt.grid(True)
        plt.show()

    def get_support_phase(self, left_z: float, right_z: float, diff_threshold: float = 0.03) -> int:
        """根据左右脚的高度确定支撑相位
        
        Args:
            left_z: 左脚Z轴高度（米）
            right_z: 右脚Z轴高度（米）
            diff_threshold: 左右脚高度差的阈值（米，默认3cm）
        
        Returns:
            int: 支撑相位 (0: 左脚支撑, 1: 右脚支撑, 2: 双脚支撑)
        """
        height_diff = abs(left_z - right_z)
        
        # 如果高度差小于阈值，认为是双脚支撑
        if height_diff < diff_threshold:
            return 2
        # 左脚高度更低，认为是左脚支撑
        elif left_z < right_z:
            return 0
        # 右脚高度更低，认为是右脚支撑
        else:
            return 1

    def visualize_animation_with_curves(self, start_frame: int = 0, end_frame: int = None, step: int = 1):
        """同时显示动画和足端高度曲线
        
        Args:
            start_frame: 起始帧
            end_frame: 结束帧（如果为None，则使用最后一帧）
            step: 帧间隔
        """
        if end_frame is None:
            end_frame = self.num_frames
            
        # 创建包含三个子图的图形窗口
        plt.ion()
        fig = plt.figure(figsize=(18, 12))
        
        # 3D动画子图
        ax_3d = fig.add_subplot(221, projection='3d')
        # 足端高度曲线子图
        ax_height = fig.add_subplot(222)
        # 支撑相位曲线子图
        ax_phase = fig.add_subplot(212)
        
        # 初始化数据
        frames = list(range(start_frame, end_frame, step))
        left_heights = []
        right_heights = []
        phases = []
        
        for frame in range(start_frame, end_frame, step):
            # 清除3D子图
            ax_3d.clear()
            
            # 存储所有关节的位置和名称
            positions = {}
            
            # 获取所有关节的世界坐标位置
            for joint in self.joint_names:
                try:
                    position, _ = self.get_joint_world_transform(frame, joint)
                    positions[joint] = position
                except Exception as e:
                    print(f"获取关节 {joint} 的位置时出错: {str(e)}")
                    continue
            
            # 更新足端高度数据
            left_foot_pos = positions.get("LeftFoot")
            right_foot_pos = positions.get("RightFoot")
            if left_foot_pos is not None and right_foot_pos is not None:
                left_heights.append(left_foot_pos[2])
                right_heights.append(right_foot_pos[2])
                # 计算当前支撑相位
                phase = self.get_support_phase(left_foot_pos[2], right_foot_pos[2], diff_threshold=3)
                phases.append(phase)
            
            # 绘制3D骨骼
            connections = []
            for joint in self.joint_names:
                parent = self.bvh_data.joint_parent(joint)
                if parent is not None:
                    connections.append((parent.name, joint))
            
            for parent, child in connections:
                if parent in positions and child in positions:
                    parent_pos = positions[parent]
                    child_pos = positions[child]
                    ax_3d.plot([parent_pos[0], child_pos[0]], 
                             [parent_pos[1], child_pos[1]], 
                             [parent_pos[2], child_pos[2]], 
                             'r-', linewidth=1)
            
            for joint, pos in positions.items():
                ax_3d.scatter(pos[0], pos[1], pos[2], c='b', marker='o')
                if joint in ["Hips", "LeftFoot", "RightFoot", "LeftHand", "RightHand", "Head"]:
                    ax_3d.text(pos[0], pos[1], pos[2], joint)
            
            # 设置3D视图属性
            all_positions = np.array(list(positions.values()))
            max_range = np.array([all_positions[:,0].ptp(), all_positions[:,1].ptp(), 
                                all_positions[:,2].ptp()]).max() / 2.0
            
            mid_x = (all_positions[:,0].max() + all_positions[:,0].min()) * 0.5
            mid_y = (all_positions[:,1].max() + all_positions[:,1].min()) * 0.5
            mid_z = (all_positions[:,2].max() + all_positions[:,2].min()) * 0.5
            
            ax_3d.set_xlim(mid_x - max_range, mid_x + max_range)
            ax_3d.set_ylim(mid_y - max_range, mid_y + max_range)
            ax_3d.set_zlim(mid_z - max_range, mid_z + max_range)
            
            ax_3d.set_xlabel('X')
            ax_3d.set_ylabel('Y')
            ax_3d.set_zlabel('Z')
            ax_3d.set_title(f'Human Skeleton Animation (Frame {frame})')
            ax_3d.view_init(elev=20, azim=45)
            ax_3d.grid(True)
            
            # 更新足端高度曲线
            ax_height.clear()
            current_frames = frames[:len(left_heights)]
            ax_height.plot(current_frames, left_heights, 'b-', label='Left Foot')
            ax_height.plot(current_frames, right_heights, 'r-', label='Right Foot')
            ax_height.axvline(x=frame, color='g', linestyle='--', label='Current Frame')
            ax_height.set_xlabel('Frame')
            ax_height.set_ylabel('Height (Z-axis)')
            ax_height.set_title('Foot Height Change Curve')
            ax_height.legend()
            ax_height.grid(True)
            
            # 更新支撑相位曲线
            ax_phase.clear()
            ax_phase.plot(current_frames, phases, 'k-', linewidth=2)
            ax_phase.axvline(x=frame, color='g', linestyle='--', label='Current Frame')
            ax_phase.set_xlabel('Frame')
            ax_phase.set_ylabel('Support Phase')
            ax_phase.set_title('Support Phase Change (0: Left Foot, 1: Right Foot, 2: Both Feet)')
            ax_phase.set_yticks([0, 1, 2])
            ax_phase.set_yticklabels(['Left Foot Support', 'Right Foot Support', 'Both Feet Support'])
            ax_phase.grid(True)
            
            # 调整布局
            plt.tight_layout()
            
            # 更新显示
            plt.draw()
            plt.pause(0.01)
            
        plt.ioff()
        plt.show()

def main():
    converter = BVHToTaichiConverter()
    
    # 读取BVH文件
    bvh_file = "/home/fandes/workspace/kuavo-ros-controldev/lisishu-test1-Body0.bvh"
    if not converter.read_bvh(bvh_file):
        return
    
    # 测试关节变换
    converter.test_joint_transforms(0)
    
    # 可视化动画和足端高度曲线（每10帧显示一次）
    # converter.visualize_animation_with_curves(start_frame=0, end_frame=converter.num_frames, step=10)
    
    # 单独显示足端高度曲线
    # converter.plot_feet_height(start_frame=0, end_frame=converter.num_frames, step=10)
    
    # 导出数据
    converter.save_taichi_motion("converted_motion_data.json")
if __name__ == "__main__":
    main()
