#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rm_ros_interfaces.msg import Sixforce
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import collections
import time

class ForceVisualizer(Node):
    def __init__(self):
        super().__init__('force_visualizer')
        
        # 内存保留最近 200 个点
        self.max_pts = 200  
        self.time_data = collections.deque(maxlen=self.max_pts)
        # 3轴力数据
        self.fx_data = collections.deque(maxlen=self.max_pts)
        self.fy_data = collections.deque(maxlen=self.max_pts)
        self.fz_data = collections.deque(maxlen=self.max_pts)
        # 3轴力矩数据
        self.mx_data = collections.deque(maxlen=self.max_pts)
        self.my_data = collections.deque(maxlen=self.max_pts)
        self.mz_data = collections.deque(maxlen=self.max_pts)
        
        self.start_time = time.time()

        self.subscription = self.create_subscription(
            Sixforce,
            '/rm_driver/udp_six_zero_force',
            self.force_callback,
            10)

        # 创建 2x1 的子图布局
        self.fig, (self.ax_f, self.ax_m) = plt.subplots(2, 1, figsize=(10, 8))
        
        # 定义线条
        self.line_fx, = self.ax_f.plot([], [], 'r-', label='Fx', alpha=0.8)
        self.line_fy, = self.ax_f.plot([], [], 'g-', label='Fy', alpha=0.8)
        self.line_fz, = self.ax_f.plot([], [], 'b-', label='Fz', linewidth=2)
        
        self.line_mx, = self.ax_m.plot([], [], 'r--', label='Mx')
        self.line_my, = self.ax_m.plot([], [], 'g--', label='My')
        self.line_mz, = self.ax_m.plot([], [], 'b--', label='Mz')

        self.setup_plot()

    def setup_plot(self):
        self.ax_f.set_title("Real-time Force Interaction (N)")
        self.ax_f.grid(True)
        self.ax_f.legend(loc='upper right', ncol=3)
        
        self.ax_m.set_title("Real-time Torque Interaction (Nm)")
        self.ax_m.set_xlabel("Time (s)")
        self.ax_m.grid(True)
        self.ax_m.legend(loc='upper right', ncol=3)
        plt.tight_layout()

    def force_callback(self, msg):
        self.time_data.append(time.time() - self.start_time)
        # 填充 3 轴力
        self.fx_data.append(msg.force_fx)
        self.fy_data.append(msg.force_fy)
        self.fz_data.append(msg.force_fz)
        # 填充 3 轴力矩
        self.mx_data.append(msg.force_mx)
        self.my_data.append(msg.force_my)
        self.mz_data.append(msg.force_mz)

    def update_plot(self, frame):
        if not self.time_data: return self.line_fz,
        
        t = list(self.time_data)
        # 更新力图层
        self.line_fx.set_data(t, list(self.fx_data))
        self.line_fy.set_data(t, list(self.fy_data))
        self.line_fz.set_data(t, list(self.fz_data))
        
        # 更新力矩图层
        self.line_mx.set_data(t, list(self.mx_data))
        self.line_my.set_data(t, list(self.my_data))
        self.line_mz.set_data(t, list(self.mz_data))

        # 动态调整坐标轴范围
        for ax, data_list in [(self.ax_f, [self.fx_data, self.fy_data, self.fz_data]), 
                             (self.ax_m, [self.mx_data, self.my_data, self.mz_data])]:
            ax.set_xlim(t[0], t[-1] + 0.5)
            all_vals = [v for d in data_list for v in d]
            ax.set_ylim(min(all_vals)-1, max(all_vals)+1)

        return self.line_fz, self.line_mz

def main():
    rclpy.init()
    node = ForceVisualizer()
    # interval=50 表示每秒刷新 20 次，兼顾实时性和 CPU
    ani = FuncAnimation(node.fig, node.update_plot, interval=50, blit=False)
    plt.show()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()