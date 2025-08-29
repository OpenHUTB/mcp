import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.interpolate import interp1d
import argparse
import os

class InteractiveCsvEditor:
    def __init__(self, csv_path):
        # 检查文件是否存在
        if not os.path.exists(csv_path):
            raise FileNotFoundError(f"CSV文件不存在: {csv_path}")
            
        # 读取CSV数据
        self.data = pd.read_csv(csv_path)
        self.columns = self.data.columns
        self.current_column = self.columns[0]
        self.selected_point = None
        self.influence_range_before = 5
        self.influence_range_after = 5
        self.input_path = csv_path
        # 添加平移相关的变量
        self.panning = False
        self.pan_start_x = None
        self.pan_start_y = None
        self.setup_plot()
        
    def setup_plot(self):
        # 创建图形和轴
        self.fig, self.ax = plt.subplots(figsize=(15, 8))  # 增大整体图形大小
        plt.subplots_adjust(bottom=0.2, left=0.05, right=0.98, top=0.95)  # 调整边距，使绘图区更大
        
        # 创建列选择滑块
        ax_col = plt.axes([0.2, 0.1, 0.6, 0.03])  # 调整控件位置
        self.col_slider = Slider(
            ax_col, 'Column', 0, len(self.columns)-1,
            valinit=0, valstep=1
        )
        
        # 创建前向插值范围滑块
        ax_before = plt.axes([0.2, 0.06, 0.6, 0.03])  # 调整控件位置
        self.before_slider = Slider(
            ax_before, 'Before Range', 1, 50,
            valinit=self.influence_range_before, valstep=1
        )
        
        # 创建后向插值范围滑块
        ax_after = plt.axes([0.2, 0.02, 0.6, 0.03])  # 调整控件位置
        self.after_slider = Slider(
            ax_after, 'After Range', 1, 50,
            valinit=self.influence_range_after, valstep=1
        )
        
        # 绘制初始数据
        self.update_plot(True)  # True表示是初始化调用
        
        # 添加事件处理
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.col_slider.on_changed(self.on_column_change)
        self.before_slider.on_changed(self.on_range_change)
        self.after_slider.on_changed(self.on_range_change)
        
    def update_plot(self, is_init=False):
        # 保存当前的视图范围
        if hasattr(self.ax, 'get_xlim') and not is_init:
            prev_xlim = self.ax.get_xlim()
            prev_ylim = self.ax.get_ylim()
        else:
            prev_xlim = None
            prev_ylim = None
            
        self.ax.clear()
        x = np.arange(len(self.data))
        y = self.data[self.current_column].values
        self.line, = self.ax.plot(x, y, 'b-')
        self.points, = self.ax.plot(x, y, 'bo')
        self.ax.set_title(f'Editing: {self.current_column}')
        self.ax.grid(True)
        
        # 如果是初始化，设置合适的显示范围
        if is_init:
            self.ax.set_xlim(-len(self.data)*0.02, len(self.data)*1.02)  # 留出2%的边距
            y_min, y_max = np.min(y), np.max(y)
            y_margin = (y_max - y_min) * 0.05  # 留出5%的边距
            self.ax.set_ylim(y_min - y_margin, y_max + y_margin)
        # 否则恢复之前的视图范围
        elif prev_xlim is not None:
            self.ax.set_xlim(prev_xlim)
            self.ax.set_ylim(prev_ylim)
            
        plt.draw()
        
    def interpolate_points(self, x_idx, new_y):
        # 使用三次样条插值
        y_data = self.data[self.current_column].values
        x_data = np.arange(len(y_data))
        
        # 使用滑块设定的插值范围
        start_idx = max(0, x_idx - self.influence_range_before)
        end_idx = min(len(y_data), x_idx + self.influence_range_after + 1)
        
        # 更新当前点
        y_data[x_idx] = new_y
        
        # 对影响范围内的点进行插值
        x_interp = x_data[start_idx:end_idx]
        if len(x_interp) > 3:  # 确保有足够的点进行插值
            f = interp1d(
                [x_data[start_idx], x_idx, x_data[end_idx-1]], 
                [y_data[start_idx], new_y, y_data[end_idx-1]], 
                kind='quadratic'
            )
            y_data[start_idx:end_idx] = f(x_interp)
        
        self.data[self.current_column] = y_data
        self.update_plot()
        
    def on_click(self, event):
        if event.inaxes != self.ax:
            return
            
        if event.button == 2:  # 鼠标中键
            self.panning = True
            self.pan_start_x = event.xdata
            self.pan_start_y = event.ydata
        elif event.button == 1:  # 鼠标左键
            # 找到最近的点
            x_data = np.arange(len(self.data))
            y_data = self.data[self.current_column].values
            distances = np.sqrt((x_data - event.xdata)**2 + (y_data - event.ydata)**2)
            nearest_idx = np.argmin(distances)
            
            if distances[nearest_idx] < 0.5:  # 点击足够接近某个点
                self.selected_point = nearest_idx

    def on_motion(self, event):
        if event.inaxes != self.ax:
            return
            
        if self.panning and event.button == 2:  # 鼠标中键拖动
            # 计算拖动距离
            dx = event.xdata - self.pan_start_x
            dy = event.ydata - self.pan_start_y
            
            # 更新视图范围
            cur_xlim = self.ax.get_xlim()
            cur_ylim = self.ax.get_ylim()
            
            self.ax.set_xlim(cur_xlim - dx)
            self.ax.set_ylim(cur_ylim - dy)
            
            # 更新起始位置
            self.pan_start_x = event.xdata
            self.pan_start_y = event.ydata
            
            plt.draw()
        elif self.selected_point is not None:  # 拖动数据点
            self.interpolate_points(self.selected_point, event.ydata)

    def on_release(self, event):
        if event.button == 2:  # 鼠标中键
            self.panning = False
            self.pan_start_x = None
            self.pan_start_y = None
        elif event.button == 1:  # 鼠标左键
            self.selected_point = None
        
    def on_column_change(self, val):
        self.current_column = self.columns[int(val)]
        self.update_plot()
        
    def on_range_change(self, val):
        self.influence_range_before = int(self.before_slider.val)
        self.influence_range_after = int(self.after_slider.val)
        
    def on_scroll(self, event):
        if event.inaxes is None:
            return
            
        # 设置缩放因子
        base_scale = 1.1
        if event.button == 'up':
            # 放大
            scale_factor = 1/base_scale
        else:
            # 缩小
            scale_factor = base_scale

        # 获取当前视图范围
        cur_xlim = self.ax.get_xlim()
        cur_ylim = self.ax.get_ylim()
        
        # 获取子图的位置信息
        bbox = self.ax.get_position()
        # 计算鼠标在图形中的相对位置
        fig_height = self.fig.get_figheight()
        fig_width = self.fig.get_figwidth()
        mouse_x = event.x / self.fig.dpi / fig_width
        mouse_y = event.y / self.fig.dpi / fig_height
        
        # 定义边缘区域的大小（相对于子图大小的比例）
        edge_ratio = 0.1
        
        # 检查鼠标位置
        if event.inaxes == self.ax:
            # 判断是否在子图的边缘区域
            in_left_edge = mouse_x < (bbox.x0 + bbox.width * edge_ratio)
            in_right_edge = mouse_x > (bbox.x1 - bbox.width * edge_ratio)
            in_bottom_edge = mouse_y < (bbox.y0 + bbox.height * edge_ratio)
            in_top_edge = mouse_y > (bbox.y1 - bbox.height * edge_ratio)
            
            if in_left_edge or in_right_edge:
                # 在左右边缘，只缩放y轴，以鼠标y位置为中心
                ydata = event.ydata
                new_height = (cur_ylim[1] - cur_ylim[0]) * scale_factor
                self.ax.set_ylim([ydata - new_height * (ydata - cur_ylim[0])/(cur_ylim[1] - cur_ylim[0]),
                                 ydata + new_height * (cur_ylim[1] - ydata)/(cur_ylim[1] - cur_ylim[0])])
            elif in_bottom_edge or in_top_edge:
                # 在上下边缘，只缩放x轴
                center = (cur_xlim[1] + cur_xlim[0]) / 2
                new_width = (cur_xlim[1] - cur_xlim[0]) * scale_factor
                self.ax.set_xlim([center - new_width/2, center + new_width/2])
            else:
                # 在中央区域，同时缩放两个轴
                xdata = event.xdata
                ydata = event.ydata
                
                # 计算新的范围
                new_width = (cur_xlim[1] - cur_xlim[0]) * scale_factor
                new_height = (cur_ylim[1] - cur_ylim[0]) * scale_factor
                
                # 设置新的视图范围
                self.ax.set_xlim([xdata - new_width * (xdata - cur_xlim[0])/(cur_xlim[1] - cur_xlim[0]),
                                 xdata + new_width * (cur_xlim[1] - xdata)/(cur_xlim[1] - cur_xlim[0])])
                self.ax.set_ylim([ydata - new_height * (ydata - cur_ylim[0])/(cur_ylim[1] - cur_ylim[0]),
                                 ydata + new_height * (cur_ylim[1] - ydata)/(cur_ylim[1] - cur_ylim[0])])
        
        plt.draw()
        
    def save_data(self, output_path):
        self.data.to_csv(output_path, index=False)
        
    def show(self):
        plt.show()

def main():
    # 创建参数解析器
    parser = argparse.ArgumentParser(description='交互式CSV数据编辑器')
    parser.add_argument('input_csv', type=str, help='输入CSV文件的路径')
    parser.add_argument('-o', '--output', type=str, help='输出CSV文件的路径。如果不指定，将在输入文件名后添加_edited后缀')
    
    # 解析命令行参数
    args = parser.parse_args()
    
    # 设置输出文件路径
    if args.output:
        output_path = args.output
    else:
        # 自动生成输出文件路径
        base_name, ext = os.path.splitext(args.input_csv)
        output_path = f"{base_name}_edited{ext}"
    
    try:
        # 创建编辑器实例
        editor = InteractiveCsvEditor(args.input_csv)
        editor.show()
        # 保存编辑后的数据
        editor.save_data(output_path)
        print(f"数据已保存到: {output_path}")
    except FileNotFoundError as e:
        print(f"错误: {e}")
        return 1
    except Exception as e:
        print(f"发生错误: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())
