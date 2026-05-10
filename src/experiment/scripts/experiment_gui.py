#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QLineEdit, QPushButton, QTextEdit, QGroupBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty

class ExperimentGUINode(QWidget):
    def __init__(self):
        super().__init__()
        # --- 初始化 ROS 2 节点 ---
        rclpy.init()
        self.node = Node('experiment_gui_node')
        # 创建发布者，发布到 'target_arm_pose' 话题
        self.pose_pub = self.node.create_publisher(Pose, 'target_arm_pose', 10)
        self.exec_pub = self.node.create_publisher(Empty, 'execute_arm_plan', 10)
        # 设置中文字体
        font = QFont()
        font.setFamily("SimHei") 
        font.setPointSize(10)
        QApplication.setFont(font)
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # --- 输入区：运动参数 ---
        input_group = QGroupBox("实验运动输入")
        input_layout = QVBoxLayout()
        
        # 线速度 v
        h1 = QHBoxLayout()
        h1.addWidget(QLabel('线速度 v (m/s):'))
        self.input_v = QLineEdit('0.5')
        h1.addWidget(self.input_v)
        input_layout.addLayout(h1)

        # 角速度 w
        h2 = QHBoxLayout()
        h2.addWidget(QLabel('角速度 ω (rad/s):'))
        self.input_w = QLineEdit('1.0')
        h2.addWidget(self.input_w)
        input_layout.addLayout(h2)
        
        input_group.setLayout(input_layout)
        layout.addWidget(input_group)

        # --- 设置区：末端姿态 (角度制) ---
        pose_group = QGroupBox("末端姿态设置 (Euler Angles)")
        pose_layout = QVBoxLayout()
        
        h3 = QHBoxLayout()
        h3.addWidget(QLabel('Roll (deg):'))
        self.input_roll = QLineEdit('0.0')
        h3.addWidget(self.input_roll)
        
        h4 = QHBoxLayout()
        h4.addWidget(QLabel('Pitch (deg):'))
        self.input_pitch = QLineEdit('0.0')
        h4.addWidget(self.input_pitch)
        
        h5 = QHBoxLayout()
        h5.addWidget(QLabel('Yaw (deg):'))
        self.input_yaw = QLineEdit('0.0')
        h5.addWidget(self.input_yaw)
        
        pose_layout.addLayout(h3)
        pose_layout.addLayout(h4)
        pose_layout.addLayout(h5)
        pose_group.setLayout(pose_layout)
        layout.addWidget(pose_group)

        # --- 按钮 ---
        self.btn_calc = QPushButton('计算末端位姿')
        self.btn_calc.setStyleSheet("height: 40px; background-color: #0078D7; color: white;")
        self.btn_calc.clicked.connect(self.on_verify_clicked)
        layout.addWidget(self.btn_calc)
        self.btn_execute = QPushButton("确认执行 (Execute)", self)
        self.btn_execute.clicked.connect(self.send_execute_signal)
        layout.addWidget(self.btn_execute)
        # --- 输出区 ---
        self.result_display = QTextEdit()
        self.result_display.setReadOnly(True)
        layout.addWidget(self.result_display)

        self.setLayout(layout)
        self.setWindowTitle('experiment for control')
        self.resize(450, 550)
        self.show()

    # ======================================================
    # 公式预留区：请在这里根据你的实验台物理模型进行修改
    # ======================================================
    def calculate_pose(self, v, w, r_deg, p_deg, y_deg):
        # 1. 计算旋转半径 R
        # 注意：防止角速度为 0 导致除零错误
        radius = v / w if w != 0 else 0.0

        # 2. 根据半径得到机械臂伸出长度 L (根据你的物理连接修改此公式)
        # 示例：假设伸出长度直接等于旋转半径
        L = radius - 1.21704

        # 3. 计算末端 X, Y, Z (这里假设在 X 轴方向伸出)
        x = L
        y = 0.0
        z = 0.7  # 假设一个固定工作高度

        # 4. 将姿态角 (Euler) 转换为四元数 (Quaternion)
        # 这里预留简单的转换逻辑
        qx, qy, qz, qw = self.euler_to_quaternion(r_deg, p_deg, y_deg)

        return radius, L, (x, y, z), (qx, qy, qz, qw)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """将角度制欧拉角转换为四元数"""
        r = math.radians(roll)
        p = math.radians(pitch)
        y = math.radians(yaw)

        cy = math.cos(y * 0.5)
        sy = math.sin(y * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def on_verify_clicked(self):
        try:
            v = float(self.input_v.text())
            w = float(self.input_w.text())
            roll = float(self.input_roll.text())
            pitch = float(self.input_pitch.text())
            yaw = float(self.input_yaw.text())

            R, L, pos, quat = self.calculate_pose(v, w, roll, pitch, yaw)

            res_text = (f"--- 计算验证 ---\n"
                        f"旋转半径 R: {R:.4f} m\n"
                        f"伸出长度 L: {L:.4f} m\n\n"
                        f"--- 目标位姿 (Pose) ---\n"
                        f"Position: [x:{pos[0]:.3f}, y:{pos[1]:.3f}, z:{pos[2]:.3f}]\n"
                        f"Orientation: [x:{quat[0]:.3f}, y:{quat[1]:.3f}, z:{quat[2]:.3f}, w:{quat[3]:.3f}]")
            self.result_display.setText(res_text)
            # 发送消息节点
            msg = Pose()
            msg.position.x = pos[0]
            msg.position.y = pos[1]
            msg.position.z = pos[2]
            msg.orientation.x = quat[0]
            msg.orientation.y = quat[1]
            msg.orientation.z = quat[2]
            msg.orientation.w = quat[3]
            self.pose_pub.publish(msg) # 发送给 C++ 节点
            self.result_display.append("\n[ROS 2] 已发送位姿至控制节点...")
            
        except Exception as e:
            self.result_display.setText(f"错误: {str(e)}")
    # 按钮回调函数
    def send_execute_signal(self):
        msg = Empty()
        self.exec_pub.publish(msg)
        self.result_display.append("[UI] 已发送执行指令。")
    
if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = ExperimentGUINode()
    sys.exit(app.exec_())