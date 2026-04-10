#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2025 <Ming2zun:https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system>
#                <喵了个水蓝蓝:https://www.bilibili.com/video/BV1kzEwzuEFw?spm_id_from=333.788.videopod.sections&vd_source=134c12873ff478ea447a06d652426f8f>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.




import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math
import numpy as np
from scipy.spatial import KDTree
from nav_msgs.msg import Path
import yaml
import os
import random

class DWAController:
    def __init__(self):
        # 1. 机器人运动学极限 (根据 URDF 提取)
        self.v_max = 0.8       # 最大线速度 m/s
        self.v_min = -0.2       # 最小线速度 (不允许倒车逃生)
        self.w_max = 0.8       # 最大角速度 rad/s (限制震荡)
        self.w_min = -0.8      # 最小角速度
        
        # 加速度限制 (决定了动态窗口的大小)
        self.v_acc = 1.0       # 线加速度 m/s^2
        self.w_acc = 1.5       # 角加速度 rad/s^2
        
        # 采样分辨率 (越小算得越准，但越占 CPU)
        self.v_res = 0.1      
        self.w_res = 0.1      

        # 2. 仿真推演参数
        self.dt = 0.1          # 每次推演的步长
        self.predict_time = 1.0 # 往未来看 1.5 秒
        self.safe_radius = 0.20 # 致命碰撞半径 (基于URDF radius 0.1m + 0.15m缓冲)

        # 3. 评价函数权重 (多目标优化的灵魂)
        self.alpha = 1.0  # Heading: 终点指向得分权重
        self.beta = 1.5   # Distance: 避障安全得分权重 (调高点更怕死)
        self.gamma = 1.0  # Velocity: 速度得分权重 (鼓励开快点)

    def calculate_best_velocity(self, pose, v_c, w_c, target, obstacles):
        # [核心步骤 1]：计算当前周期的动态窗口 (Dynamic Window)
        Vs = [self.v_min, self.v_max, self.w_min, self.w_max] # 绝对物理限制
        Vd = [v_c - self.v_acc * self.dt, v_c + self.v_acc * self.dt,  # 当前能达到的加速度限制
              w_c - self.w_acc * self.dt, w_c + self.w_acc * self.dt]
        
        # 窗口交集
        v_min_win = max(Vs[0], Vd[0])
        v_max_win = min(Vs[1], Vd[1])
        w_min_win = max(Vs[2], Vd[2])
        w_max_win = min(Vs[3], Vd[3])

        best_v, best_w = 0.0, 0.0
        max_score = -float('inf')

        # [核心步骤 2 & 3 & 4]：网格化采样 -> 轨迹预测 -> 模拟打分
        v = v_min_win
        while v <= v_max_win:
            w = w_min_win
            while w <= w_max_win:
                # 预测这条 (v,w) 指令在未来 1.5 秒的轨迹
                traj = self.predict_trajectory(pose, v, w)
                
                # 对这条轨迹打分
                score = self.evaluate_trajectory(traj, target, obstacles, v)
                
                # 选出最高分
                if score > max_score:
                    max_score = score
                    best_v = v
                    best_w = w
                w += self.w_res
            v += self.v_res

        # 防死锁保护：如果所有路都是死路 (max_score为负无穷)
        if max_score == -float('inf'):
            # 放弃原地转圈，直接挂倒挡缓慢倒车，把车头空间让出来！
            return -0.15, 0.0

    def predict_trajectory(self, pose, v, w):
        """推演运动学轨迹"""
        traj = []
        x, y, yaw = pose
        time = 0.0
        while time <= self.predict_time:
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt
            traj.append([x, y, yaw])
            time += self.dt
        return traj

    def evaluate_trajectory(self, traj, target, obstacles, v):
        """三项全能打分器"""
        end_pose = traj[-1]
        
        # 1. Heading 评价：轨迹终点朝向与目标点的角度差 (越小越好)
        dx = target[0] - end_pose[0]
        dy = target[1] - end_pose[1]
        target_angle = math.atan2(dy, dx)
        angle_diff = abs(target_angle - end_pose[2])
        while angle_diff > math.pi: angle_diff -= 2*math.pi
        while angle_diff < -math.pi: angle_diff += 2*math.pi
        heading_score = math.pi - abs(angle_diff) # 归一化：完全对准得 pi 分

        # 2. Distance 评价：轨迹到障碍物的最近距离
        min_dist = float('inf')
        # 仅抽取轨迹中部分点进行碰撞检测以节省算力
        for p in traj[::2]: 
            for obs in obstacles:
                dist = math.hypot(p[0] - obs[0], p[1] - obs[1])
                if dist < min_dist:
                    min_dist = dist
                    
        # 【一票否决权】如果轨迹碰到了致命半径，直接枪毙这条路线
        if min_dist < self.safe_radius:
            return -float('inf') 
            
        # 核心修复 1：加上距离封顶限制！超过 1.0 米就视为满分，消除“旷野恐惧症”
        dist_score = min(min_dist, 1.0)

        # 3. Velocity 评价：在线速度上鼓励开快点
        vel_score = v

        # 计算总分 (可根据现象微调权重)
        return self.alpha * heading_score + self.beta * dist_score + self.gamma * vel_score
# ROS 2节点
class PathFollowingNode(Node):
    def __init__(self):
        super().__init__('path_following_node')
        # 创建纯追踪控制器
        self.dwa_controller = DWAController()
        # 记录当前实际速度，DWA推演需要用到
        self.current_v = 0.0
        self.current_w = 0.0
        # 创建路径点
        self.path_points = None
        # 创建订阅者
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        # 创建发布者
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # 创建路径点订阅者
        self.path_subscriber = self.create_subscription(Path, '/path', self.path_callback, 10)
        # 变量初始化
        self.current_odom = None
        # 订阅 3D 激光雷达话题 /points_raw
        self.scan_subscriber = self.create_subscription(PointCloud2, '/points_raw', self.scan_callback, 10)
        self.latest_scan = None
        
        
        self.stop_flag = False  # 新增的停止标志
        self.path_received = False  # 添加路径是否已接收的标志
        # self.get_logger().info('ready--------ok----to---nav')

    def path_callback(self, msg):
        self.path_points_list = [[point.pose.position.x, point.pose.position.y] for point in msg.poses]
        # 将列表转换为 numpy 数组
        self.path_points = np.array(self.path_points_list)
        assert self.path_points.ndim == 2, "path_points must be a 2D array"
        # 对路径点进行插值
        self.path_points = self.interpolate_path(self.path_points)
        self.path_received = True  # 设置路径接收标志
        # print('received path ready to nav-------------')

    def interpolate_path(self, points, segment_length=0.1):
        interpolated_points = []
        for i in range(len(points) - 1):
            start_point = points[i]
            end_point = points[i+1]
            # 计算两点之间的距离
            distance = np.linalg.norm(end_point - start_point)
            # 计算所需点的数量（包括起点）
            num_points = int(distance / segment_length) + 1
            # 生成线性插值点
            t_values = np.linspace(0, 1, num_points)
            interpolated_segment = start_point + (end_point - start_point)[np.newaxis, :] * t_values[:, np.newaxis]
            interpolated_points.append(interpolated_segment)
        # 将所有插值点合并成一个数组
        return np.vstack(interpolated_points)
    def scan_callback(self, msg):
        # 只要雷达刷新，就把最新的点云数据存下来给 APF 用
        self.latest_scan = msg

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def odometry_callback(self, msg):
        self.current_xy = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        if not self.path_received:
            return  # 如果还没有接收到路径，则直接返回
        self.current_odom = msg
        # 提取位置和朝向
        pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, self.quaternion_to_yaw(msg.pose.pose.orientation)]
        
      # ==========================================================
        # 🚀 3D 点云实时动态避障模块 🚀
        real_obstacles = []
        if self.latest_scan is not None:
            yaw = pose[2]  # 小车当前的绝对朝向角
            
            # 读取 3D 点云的 X, Y, Z 数据
            point_generator = pc2.read_points(self.latest_scan, field_names=("x", "y", "z"), skip_nans=True)
            
            for p in point_generator:
                local_x, local_y, local_z = p[0], p[1], p[2]
                
                # 过滤掉地面（Z太低）和天花板（Z太高）的无效噪点
                if -0.5 < local_z < 0.5:
                    
                    # 算一下这个点离小车有多远 (2D平面距离)
                    dist_2d = math.hypot(local_x, local_y)
                    
                    # 只提取 0.1米 到 1.5米 内的危险障碍物
                    if 0.1 < dist_2d < 0.6:
                        # 把相对坐标旋转平移成全局绝对坐标
                        global_x = pose[0] + local_x * math.cos(yaw) - local_y * math.sin(yaw)
                        global_y = pose[1] + local_x * math.sin(yaw) + local_y * math.cos(yaw)
                        real_obstacles.append([global_x, global_y])
            
            # 性能锁：3D点云数据量太大，如果发现的危险点超过20个，就随机抽20个算斥力，防止系统卡死
            if len(real_obstacles) > 20:
                real_obstacles = random.sample(real_obstacles, 20)
                
        
                
        # ==========================================================
        # 🚀 核心修复 2：寻找前方 0.8 米的预瞄点 (Lookahead Point) 🚀
        lookahead_target = self.path_points[-1] # 默认是终点
        for p in self.path_points:
            # 找到绿线上距离当前小车大于 0.8 米的第一个点
            if math.hypot(p[0] - pose[0], p[1] - pose[1]) > 0.8:
                lookahead_target = p
                break

        # 将预瞄点传给 DWA 控制器
        speed, steering_angle = self.dwa_controller.calculate_best_velocity(
            pose, self.current_v, self.current_w, lookahead_target, real_obstacles)

        # 更新底盘的速度记忆（这是 DWA 预测下一秒轨迹的绝对前提！）
        self.current_v = speed
        self.current_w = steering_angle
        # ==========================================================
        
        # 计算到路径终点的距离
        distance_to_end = np.linalg.norm(np.array(pose[:2]) - self.path_points[-1])
        
        # 🚀 核心修改点 2：极其清爽的发布逻辑 🚀
        # 停止条件
        if distance_to_end < 0.2:  # 0.2m作为接近阈值
            speed = 0.0
            steering_angle = 0.0
            self.path_received = False
            self.current_v = 0.0   # 停车时也清空记忆
            self.current_w = 0.0
            # self.get_logger().info('Naving node.success..')
            
        # [旧的 else 分支和 math.sin 速度计算代码已被彻底删除，因为 DWA 已经给了最完美的 speed]

        # 发布最终的速度和转向角
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = speed
        cmd_vel_msg.angular.z = steering_angle
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        # 可以取消注释下面这行来观察 DWA 的输出
        # self.get_logger().info(f'v: {speed:.2f}, ang: {steering_angle:.2f}, dist_to_end: {distance_to_end:.2f}')
        
    
       
def main(args=None):
    rclpy.init(args=args)
    node = PathFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    