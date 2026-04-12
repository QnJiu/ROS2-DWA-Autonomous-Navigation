import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math
import numpy as np
from nav_msgs.msg import Path
import random

class DWAController:
    def __init__(self):
        # 1. 机器人运动学极限
        self.v_max = 0.8       
        self.v_min = -0.2      
        self.w_max = 0.8       
        self.w_min = -0.8      
        
        # 加速度限制
        self.v_acc = 1.0       
        self.w_acc = 1.5       
        
        # 采样分辨率
        self.v_res = 0.05      
        self.w_res = 0.05      

        # 2. 仿真推演参数
        self.dt = 0.1          
        self.predict_time = 1.0 
        self.safe_radius = 0.20 
        self.startup_safe_radius = 0.15
        self.startup_flag = True  

        # 3. 评价函数权重 (归一化后的权重比例)
        self.alpha = 0.5  # 朝向权重
        self.beta = 0.4   # 避障权重
        self.gamma = 0.8  # 速度权重

        # 4. 死锁检测状态变量
        self.deadlock_count = 0
        self.deadlock_threshold = 20  
    

    def calculate_best_velocity(self, pose, v_c, w_c, target, obstacles):
        # 动态窗口计算
        Vs = [self.v_min, self.v_max, self.w_min, self.w_max]
        Vd = [v_c - self.v_acc * self.dt, v_c + self.v_acc * self.dt,
              w_c - self.w_acc * self.dt, w_c + self.w_acc * self.dt]
        
        v_min_win = max(Vs[0], Vd[0])
        v_max_win = min(Vs[1], Vd[1])
        w_min_win = max(Vs[2], Vd[2])
        w_max_win = min(Vs[3], Vd[3])

                # ==========================================================
        # 【平滑限速修复版】解决“刚开始慢，突然猛加速”的问题
        dx = target[0] - pose[0]
        dy = target[1] - pose[1]
        target_angle = math.atan2(dy, dx)
        # 计算目标点相对车头的角度差（0~3.14弧度）
        angle_diff = abs(math.atan2(math.sin(target_angle - pose[2]), math.cos(target_angle - pose[2])))
        
        # 只要角度差大于 30度（0.5弧度），就开始施加平滑限速
        if angle_diff > 0.5:
            # 将角度差归一化到 0~1 之间 (0.5弧度对应0，3.14弧度对应1)
            ratio = (angle_diff - 0.5) / (math.pi - 0.5)
            # 根据比例计算限速值：角度越大，限速越低
            # 角度差180度时(ratio=1)，限速 0.15
            # 角度差30度时(ratio=0)，限速 0.6
            limit_v = 0.6 - ratio * 0.45
            v_max_win = min(v_max_win, limit_v)
        # ==========================================================

        best_v, best_w = 0.0, 0.0
        max_score = -float('inf')
        current_safe_radius = self.startup_safe_radius if self.startup_flag else self.safe_radius

        # 网格化采样
        v = v_min_win
        while v <= v_max_win + 0.001:
            w = w_min_win
            while w <= w_max_win + 0.001:
                traj = self.predict_trajectory(pose, v, w)
                score = self.evaluate_trajectory(traj, target, obstacles, v, current_safe_radius)
                if score > max_score:
                    max_score = score
                    best_v = v
                    best_w = w
                w += self.w_res
            v += self.v_res

        # 死锁检测逻辑
        if abs(best_v) < 0.05 and abs(best_w) > 0.6:
            self.deadlock_count += 1
        else:
            self.deadlock_count = 0  

        if max_score == -float('inf') or self.deadlock_count >= self.deadlock_threshold:
            print(f'[DWA Warning] Deadlock! Reversing...')
            self.deadlock_count = 0  
            return -0.15, 0.0
        
        if best_v > 0.1:
            self.startup_flag = False

        return best_v, best_w

    def predict_trajectory(self, pose, v, w):
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

    def evaluate_trajectory(self, traj, target, obstacles, v, safe_radius):
        end_pose = traj[-1]
        
        # 1. 朝向评价 (归一化，修复角度跳变)
        dx = target[0] - end_pose[0]
        dy = target[1] - end_pose[1]
        target_angle = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(target_angle - end_pose[2]), math.cos(target_angle - end_pose[2]))
        heading_score = (math.pi - abs(angle_diff)) / math.pi  

        # 2. 避障评价 (去掉截断，恢复区分度)
        min_dist = float('inf')
        for p in traj[::2]: 
            for obs in obstacles:
                dist = math.hypot(p[0] - obs[0], p[1] - obs[1])
                if dist < min_dist:
                    min_dist = dist
        if min_dist < safe_radius:
            return -float('inf') 
        dist_score = min(min_dist / 2.0, 1.0)

        # 3. 速度评价 (归一化)
        vel_score = max(0.0, v) / self.v_max

        total_score = (self.alpha * heading_score + self.beta * dist_score + self.gamma * vel_score)
        return total_score


# ROS 2节点
class PathFollowingNode(Node):
    def __init__(self):
        super().__init__('path_following_node')
        self.dwa_controller = DWAController()
        self.current_v = 0.0
        self.current_w = 0.0
        self.path_points = None
        
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/path', self.path_callback, 10)
        self.scan_subscriber = self.create_subscription(PointCloud2, '/points_raw', self.scan_callback, 10)
        
        self.current_odom = None
        self.latest_scan = None
        self.path_received = False
        self.get_logger().info('DWA Node Started (Ultimate Version)!')

    def path_callback(self, msg):
        # 收到空路径才真正停止，防止状态锁死
        if len(msg.poses) == 0:
            self.path_received = False
            return

        self.path_points_list = [[point.pose.position.x, point.pose.position.y] for point in msg.poses]
        self.path_points = np.array(self.path_points_list)
        assert self.path_points.ndim == 2, "path_points must be a 2D array"
        self.path_points = self.interpolate_path(self.path_points)
        self.path_received = True
        self.dwa_controller.startup_flag = True
        self.dwa_controller.deadlock_count = 0

    def interpolate_path(self, points, segment_length=0.1):
        interpolated_points = []
        for i in range(len(points) - 1):
            start_point = points[i]
            end_point = points[i+1]
            distance = np.linalg.norm(end_point - start_point)
            num_points = int(distance / segment_length) + 1
            t_values = np.linspace(0, 1, num_points)
            interpolated_segment = start_point + (end_point - start_point)[np.newaxis, :] * t_values[:, np.newaxis]
            interpolated_points.append(interpolated_segment)
        return np.vstack(interpolated_points)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odometry_callback(self, msg):
        self.current_xy = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        
        # 没有路径时：只发0速度，绝对不 return，保证节点活着
        if not self.path_received or self.path_points is None:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            self.current_v = 0.0
            self.current_w = 0.0
            return

        pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, self.quaternion_to_yaw(msg.pose.pose.orientation)]
        
        # 点云处理 (带CPU防爆机制)
        real_obstacles = []
        if self.latest_scan is not None:
            yaw = pose[2]
            point_generator = pc2.read_points(self.latest_scan, field_names=("x", "y", "z"), skip_nans=True)
            for p in point_generator:
                local_x, local_y, local_z = p[0], p[1], p[2]
                if 0.1 < local_z < 0.8:
                    dist_2d = math.hypot(local_x, local_y)
                    if 0.3 < dist_2d < 2.0:
                        global_x = pose[0] + local_x * math.cos(yaw) - local_y * math.sin(yaw)
                        global_y = pose[1] + local_x * math.sin(yaw) + local_y * math.cos(yaw)
                        real_obstacles.append([global_x, global_y])
            
            # 防爆：先砍到300个，再排序取最近60个
            if len(real_obstacles) > 300:
                real_obstacles = random.sample(real_obstacles, 300)
            if len(real_obstacles) > 60:
                real_obstacles.sort(key=lambda obs: math.hypot(obs[0]-pose[0], obs[1]-pose[1]))
                real_obstacles = real_obstacles[:60]
                
        # 预瞄点逻辑 (基于偏离距离动态调整，防止起步走丢)
        lookahead_target = self.path_points[-1] # 默认终点
        distances_to_path = np.linalg.norm(self.path_points - np.array(pose[:2]), axis=1)
        nearest_idx = np.argmin(distances_to_path)
        deviation_dist = distances_to_path[nearest_idx]
        
        target_lookahead_dist = 1.0  # 正常看 1.0 米
        if deviation_dist > 1.0:     # 严重偏离看 2.0 米找方向
            target_lookahead_dist = 2.0
            
        temp_dist = 0.0
        for i in range(nearest_idx, len(self.path_points) - 1):
            temp_dist += np.linalg.norm(self.path_points[i+1] - self.path_points[i])
            if temp_dist > target_lookahead_dist:  
                lookahead_target = self.path_points[i+1]
                break

        # DWA计算
        try:
            speed, steering_angle = self.dwa_controller.calculate_best_velocity(
                pose, self.current_v, self.current_w, lookahead_target, real_obstacles)
            self.current_v = speed
            self.current_w = steering_angle
        except Exception as e:
            self.get_logger().error(f'DWA error: {e}')
            speed, steering_angle = 0.0, 0.0
            self.current_v = 0.0
            self.current_w = 0.0

        # 停止条件 (不锁死节点)
        distance_to_end = np.linalg.norm(np.array(pose[:2]) - self.path_points[-1])
        if distance_to_end < 0.2:
            speed = 0.0
            steering_angle = 0.0
            self.current_v = 0.0
            self.current_w = 0.0
            
        # 发布指令
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = speed
        cmd_vel_msg.angular.z = steering_angle
        self.cmd_vel_publisher.publish(cmd_vel_msg)

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