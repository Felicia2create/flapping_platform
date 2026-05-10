#!/usr/bin/env python3
"""
转台速度控制 UI (tkinter)
发布 Float64MultiArray 到 /turntable_velocity_controller/commands

用法:
  source install/setup.bash
  ros2 run flapping_platform_bringup turntable_control.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import tkinter as tk
from tkinter import ttk
import threading
import math


class TurntableControlNode(Node):
    """ROS2 节点：发布转台速度命令，监听转台关节角度"""

    def __init__(self):
        super().__init__('turntable_control_node')
        self.pub = self.create_publisher(Float64MultiArray, '/turntable_velocity_controller/commands', 10)
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)
        self.current_angle = 0.0  # 度
        self.velocity = 0.0  # 当前发布的速度

    def joint_states_cb(self, msg):
        """从 joint_states 中提取转台关节角度"""
        # 尝试常见的转台关节名称
        turntable_joint_names = ['turntable_joint', 'base_link_to_turntable', 'turntable']
        for name, position in zip(msg.name, msg.position):
            if name in turntable_joint_names:
                self.current_angle = math.degrees(position)
                break

    def set_velocity(self, velocity):
        """发布速度命令"""
        msg = Float64MultiArray()
        msg.data = [velocity]
        self.pub.publish(msg)
        self.velocity = velocity


class TurntableControlGUI:
    """转台控制 GUI（tkinter）"""

    def __init__(self, node: TurntableControlNode):
        self.node = node

        # 创建主窗口
        self.root = tk.Tk()
        self.root.title("转台速度控制")
        self.root.geometry("420x260")
        self.root.resizable(False, False)

        style = ttk.Style()
        style.theme_use('clam')

        # === 速度显示与滑块 ===
        frame = ttk.Frame(self.root, padding="10")
        frame.pack(fill=tk.BOTH, expand=True)

        # 标题
        title = ttk.Label(frame, text="转台速度控制", font=("Arial", 14, "bold"))
        title.pack(pady=(0, 8))

        # 速度值显示
        self.velocity_var = tk.DoubleVar(value=0.0)
        velocity_label = ttk.Label(frame, text="速度 (rad/s):", font=("Arial", 10))
        velocity_label.pack(anchor=tk.W)

        value_frame = ttk.Frame(frame)
        value_frame.pack(fill=tk.X, pady=2)

        self.velocity_value = ttk.Label(value_frame, text="0.00", font=("Arial", 12, "bold"),
                                        foreground="#2196F3", width=6, anchor=tk.CENTER)
        self.velocity_value.pack(side=tk.LEFT)

        # 滑块
        self.slider = ttk.Scale(
            frame, from_=-1.0, to=1.0, orient=tk.HORIZONTAL,
            variable=self.velocity_var, command=self.on_slider_change
        )
        self.slider.pack(fill=tk.X, pady=(0, 2))

        # 滑块刻度标签
        scale_label = ttk.Label(frame, text="-1.0                         0.0                         1.0",
                                font=("Arial", 8), foreground="gray")
        scale_label.pack()

        # === 控制按钮 ===
        btn_frame = ttk.Frame(frame)
        btn_frame.pack(pady=8)

        btn_ccw = ttk.Button(btn_frame, text="◀ 逆时针", command=lambda: self.set_velocity(-0.5),
                             width=12)
        btn_ccw.pack(side=tk.LEFT, padx=4)

        btn_stop = ttk.Button(btn_frame, text="■ 停止", command=lambda: self.set_velocity(0.0),
                              width=12)
        btn_stop.pack(side=tk.LEFT, padx=4)

        btn_cw = ttk.Button(btn_frame, text="顺时针 ▶", command=lambda: self.set_velocity(0.5),
                            width=12)
        btn_cw.pack(side=tk.LEFT, padx=4)

        # === 状态与角度显示 ===
        status_frame = ttk.Frame(frame)
        status_frame.pack(fill=tk.X, pady=(8, 0))

        self.angle_var = tk.StringVar(value="角度: 0.0°")
        angle_label = ttk.Label(status_frame, textvariable=self.angle_var,
                                font=("Arial", 11), foreground="#4CAF50")
        angle_label.pack(side=tk.LEFT)

        self.status_var = tk.StringVar(value="状态: 停止")
        status_label = ttk.Label(status_frame, textvariable=self.status_var,
                                 font=("Arial", 10))
        status_label.pack(side=tk.RIGHT)

        # 键盘绑定
        self.root.bind('<Up>', lambda e: self.nudge_velocity(0.1))
        self.root.bind('<Down>', lambda e: self.nudge_velocity(-0.1))
        self.root.bind('<space>', lambda e: self.set_velocity(0.0))
        self.root.bind('<Left>', lambda e: self.set_velocity(-0.5))
        self.root.bind('<Right>', lambda e: self.set_velocity(0.5))

        # 定时更新 UI
        self.update_ui()

        # 窗口关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def set_velocity(self, vel):
        """设置速度并更新滑块"""
        vel = max(-1.0, min(1.0, vel))
        self.velocity_var.set(vel)
        self.node.set_velocity(vel)
        self.update_status(vel)

    def on_slider_change(self, event=None):
        """滑块拖动回调"""
        vel = self.velocity_var.get()
        self.node.set_velocity(vel)
        self.update_status(vel)

    def nudge_velocity(self, delta):
        """微调速度"""
        vel = self.velocity_var.get() + delta
        self.set_velocity(vel)

    def update_status(self, vel):
        """更新状态文字"""
        if abs(vel) < 0.01:
            status = "状态: 停止"
        elif vel > 0:
            status = f"状态: 顺时针 (速度: {vel:.2f})"
        else:
            status = f"状态: 逆时针 (速度: {abs(vel):.2f})"
        self.status_var.set(status)
        self.velocity_value.config(text=f"{vel:+.2f}")

    def update_ui(self):
        """定时更新 UI 元素（角度显示）"""
        self.angle_var.set(f"角度: {self.node.current_angle:.1f}°")
        self.root.after(100, self.update_ui)

    def on_close(self):
        """窗口关闭时停止转台并退出"""
        self.node.set_velocity(0.0)
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = TurntableControlNode()

    # 在独立线程中运行 ROS2 spin
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    # 启动 GUI（主线程）
    gui = TurntableControlGUI(node)
    gui.run()

    # 清理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
