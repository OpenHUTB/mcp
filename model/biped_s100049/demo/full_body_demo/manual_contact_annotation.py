import numpy as np
import pandas as pd
import time
import sys
import argparse
import termios
import tty
import select
from pydrake.all import (
    MultibodyPlant, Parser, AddMultibodyPlantSceneGraph, DiagramBuilder,
    Meshcat, MeshcatVisualizer, MeshcatVisualizerParams
)

class NonBlockingInput:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        
    def __enter__(self):
        tty.setraw(sys.stdin.fileno())
        return self
        
    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
    
    def get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            return sys.stdin.read(1)
        return None

class ContactAnnotator:
    def __init__(self, urdf_path, csv_path, dt=1/60):
        # 创建Drake环境
        builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-2)
        
        # 加载机器人模型
        parser = Parser(self.plant)
        self.model = parser.AddModelFromFile(urdf_path)
        self.plant.Finalize()
        
        # 创建可视化器
        self.meshcat = Meshcat()
        self.visualizer = MeshcatVisualizer.AddToBuilder(
            builder=builder,
            scene_graph=self.scene_graph,
            meshcat=self.meshcat)
        
        # 构建系统
        self.diagram = builder.Build()
        self.context = self.diagram.CreateDefaultContext()
        self.plant_context = self.plant.GetMyContextFromRoot(self.context)
        
        # 加载动作数据
        self.df = pd.read_csv(csv_path)
        self.current_frame = 0
        self.total_frames = len(self.df)
        self.playing = True
        self.playback_speed = 1.0
        self.dt = dt
        
        # 初始化接触状态数组：0=双脚接触，1=左脚接触，2=右脚接触，3=无接触
        self.contact_states = np.zeros(self.total_frames, dtype=np.int32)
        
        # 当前接触模式：0=双脚接触，1=左脚接触，2=右脚接触，3=无接触
        self.contact_mode = 0
        
        print("加载了 {} 帧数据".format(self.total_frames))
        print("\n控制说明:")
        print("p: 播放/暂停")
        print("a/d: 前一帧/后一帧")
        print("w/s: 加快/减慢播放速度")
        print("l: 切换到左脚接触模式")
        print("r: 切换到右脚接触模式")
        print("空格: 切换到双脚接触模式")
        print("n: 切换到无接触模式")
        print("v: 保存标注结果")
        print("q: 退出程序")
    
    def process_generalized_position(self, row):
        q = np.zeros_like(row[:35])
        q[0:4] = row[3:7]      # w,x,y,z
        q[4:7] = row[0:3]      # x,y,z
        q[7:] = row[7:35]      # joints
        return q
    
    def update_contact_state(self):
        """根据当前接触模式更新接触状态"""
        self.contact_states[self.current_frame] = self.contact_mode
    
    def display_frame(self, frame_idx):
        if 0 <= frame_idx < self.total_frames:
            q = self.process_generalized_position(self.df.iloc[frame_idx])
            self.plant.SetPositions(self.plant_context, q)
            self.context.SetTime(frame_idx * self.dt)
            self.diagram.ForcedPublish(self.context)
            
            self.current_frame = frame_idx
            self.update_contact_state()
            
            # 显示当前接触状态和模式
            mode_names = ['双脚', '左脚', '右脚', '无接触']
            current_state = self.contact_states[frame_idx]
            contact_info = f"当前状态: {mode_names[current_state]} | 当前模式: {mode_names[self.contact_mode]}"
            
            sys.stdout.write("\r帧:{:4d}/{:4d} ".format(frame_idx, self.total_frames) + 
                           "速度:{:3.1f}x | ".format(self.playback_speed) +
                           contact_info)
            sys.stdout.flush()
    
    def save_annotations(self):
        # 将接触状态拼接到原始数据前面
        new_data = np.hstack((self.contact_states.reshape(-1, 1), self.df.values))
        
        # 保存到新文件，不包含表头
        output_filename = 'motion_with_contacts.csv'
        np.savetxt(output_filename, new_data, delimiter=',', fmt='%g')
        print(f"\n已保存标注结果到文件: {output_filename}")
    
    def run(self):
        last_time = time.time()
        frame_time = self.dt
        
        with NonBlockingInput() as kb:
            while True:
                current_time = time.time()
                elapsed = current_time - last_time
                
                key = kb.get_key()
                if key:
                    if key == 'q':
                        print("\n退出程序")
                        break
                    elif key == 'p':
                        self.playing = not self.playing
                    elif key == 'd' and not self.playing:
                        self.display_frame(self.current_frame + 1)
                    elif key == 'a' and not self.playing:
                        self.display_frame(self.current_frame - 1)
                    elif key == 'w':
                        self.playback_speed = min(self.playback_speed + 0.5, 10.0)
                    elif key == 's':
                        self.playback_speed = max(self.playback_speed - 0.5, 0.1)
                    elif key == 'l':
                        self.contact_mode = 1  # 左脚接触模式
                    elif key == 'r':
                        self.contact_mode = 2  # 右脚接触模式
                    elif key == ' ':
                        self.contact_mode = 0  # 双脚接触模式
                    elif key == 'n':
                        self.contact_mode = 3  # 无接触模式
                    elif key == 'v':
                        self.save_annotations()
                    self.update_contact_state()
                
                if self.playing and elapsed >= frame_time / self.playback_speed:
                    self.display_frame((self.current_frame + 1) % self.total_frames)
                    last_time = current_time
                
                time.sleep(0.001)

def main():
    parser = argparse.ArgumentParser(description='手动标注接触状态工具')
    parser.add_argument('--urdf', type=str, 
                      default="my-robot-data/robots/biped_s46/urdf/drake/biped_v3_full.urdf",
                      help='URDF文件路径')
    parser.add_argument('--csv', type=str, 
                      default='taichi_easy_cali_drake.csv',
                      help='动作数据CSV文件路径')
    
    args = parser.parse_args()
    
    annotator = ContactAnnotator(args.urdf, args.csv)
    annotator.run()

if __name__ == "__main__":
    main() 
