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

# ================= 卡尔曼滤波器类 =================
class SimpleKalmanFilter:
    def __init__(self, dt):
        self.dt = dt
        self.X = np.zeros(4) # [x, y, vx, vy]
        self.F = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])
        self.P = np.eye(4) * 10.0
        self.Q = np.eye(4) * 0.01
        self.R = np.eye(2) * 0.1
        self.is_initialized = False

    def update(self, z):
        if not self.is_initialized:
            self.X = np.array([z[0], z[1], 0.0, 0.0])
            self.is_initialized = True
            return self.X
        X_pred = self.F @ self.X
        P_pred = self.F @ self.P @ self.F.T + self.Q
        y = z - (self.H @ X_pred)
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        self.X = X_pred + K @ y
        self.P = (np.eye(4) - K @ self.H) @ P_pred
        return self.X
# ========================================================

class DWAController:
    def __init__(self):
        self.v_max = 0.8       
        self.v_min = -0.2      
        self.w_max = 0.8       
        self.w_min = -0.8      
        self.v_acc = 1.0       
        self.w_acc = 1.5       
        self.v_res = 0.05      
        self.w_res = 0.05      
        self.dt = 0.1          
        self.predict_time = 1.0 
        self.safe_radius = 0.20 
        self.startup_safe_radius = 0.15
        self.startup_flag = True  
        self.alpha = 0.5  
        self.beta = 0.4   
        self.gamma = 0.8  
        self.deadlock_count = 0
        self.deadlock_threshold = 20  
        self.predicted_dynamic_obs = []

    def calculate_best_velocity(self, pose, v_c, w_c, target, obstacles, predicted_obs, dynamic_safe_radius):
        Vs = [self.v_min, self.v_max, self.w_min, self.w_max]
        Vd = [v_c - self.v_acc * self.dt, v_c + self.v_acc * self.dt,
              w_c - self.w_acc * self.dt, w_c + self.w_acc * self.dt]
        
        v_min_win = max(Vs[0], Vd[0])
        v_max_win = min(Vs[1], Vd[1])
        w_min_win = max(Vs[2], Vd[2])
        w_max_win = min(Vs[3], Vd[3])

        # 平滑限速
        dx = target[0] - pose[0]
        dy = target[1] - pose[1]
        target_angle = math.atan2(dy, dx)
        angle_diff = abs(math.atan2(math.sin(target_angle - pose[2]), math.cos(target_angle - pose[2])))
        
        if angle_diff > 0.5:
            ratio = (angle_diff - 0.5) / (math.pi - 0.5)
            limit_v = 0.6 - ratio * 0.45
            v_max_win = min(v_max_win, limit_v)

        best_v, best_w = 0.0, 0.0
        max_score = -float('inf')
        current_safe_radius = self.startup_safe_radius if self.startup_flag else dynamic_safe_radius

        v = v_min_win
        while v <= v_max_win + 0.001:
            w = w_min_win
            while w <= w_max_win + 0.001:
                traj = self.predict_trajectory(pose, v, w)
                score = self.evaluate_trajectory(traj, target, obstacles, v, current_safe_radius, predicted_obs)
                if score > max_score:
                    max_score = score
                    best_v = v
                    best_w = w
                w += self.w_res
            v += self.v_res

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

    def evaluate_trajectory(self, traj, target, obstacles, v, safe_radius, predicted_obs):
        end_pose = traj[-1]
        
        # 1. 朝向评价
        dx = target[0] - end_pose[0]
        dy = target[1] - end_pose[1]
        target_angle = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(target_angle - end_pose[2]), math.cos(target_angle - end_pose[2]))
        heading_score = (math.pi - abs(angle_diff)) / math.pi  

        # 2. 静态真实避障评价 (硬约束保命)
        min_dist = float('inf')
        for p in traj[::2]: 
            for obs in obstacles:
                dist = math.hypot(p[0] - obs[0], p[1] - obs[1])
                if dist < min_dist:
                    min_dist = dist
        if min_dist < safe_radius:
            return -float('inf') 
        dist_score = min(min_dist / 2.0, 1.0)

        # 3. 动态预测避障评价 (软约束排斥 + 时间碰撞死刑)
        dynamic_penalty = 0.0
        if len(predicted_obs) > 0:
            min_future_dist = float('inf')
            for p in traj[::2]: 
                for ghost_obs in predicted_obs:
                    dist = math.hypot(p[0] - ghost_obs[0], p[1] - ghost_obs[1])
                    if dist < min_future_dist:
                        min_future_dist = dist
            
            # 【关键修复1】只有当小车有速度(>0.05)时才判死刑，允许它原地停车等！
            if min_future_dist < 0.4 and abs(v) > 0.05:
                return -float('inf')
            elif min_future_dist < 2.0:
                dynamic_penalty = (2.0 - min_future_dist) / 2.0 * 4.0 

        # 4. 速度评价
        vel_score = max(0.0, v) / self.v_max

        # 【关键修复2】之前漏掉了这两行，导致返回 None 报错小车不动
        total_score = (self.alpha * heading_score + self.beta * dist_score + self.gamma * vel_score - dynamic_penalty)
        return total_score


class PathFollowingNode(Node):
    def __init__(self):
        super().__init__('path_following_node')
        self.dwa_controller = DWAController()
        self.current_v = 0.0
        self.current_w = 0.0
        self.path_points = None
        self.path_received = False
        self.kf_tracker = SimpleKalmanFilter(dt=0.1) 
        
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/path', self.path_callback, 10)
        self.scan_subscriber = self.create_subscription(PointCloud2, '/points_raw', self.scan_callback, 10)
        
        self.current_odom = None
        self.latest_scan = None
        self.get_logger().info('DWA Node Started (Ultimate Safe Wait Version)!')

    def path_callback(self, msg):
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
        
        if not self.path_received or self.path_points is None:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            self.current_v = 0.0
            self.current_w = 0.0
            return

        pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, self.quaternion_to_yaw(msg.pose.pose.orientation)]
        
        # 点云处理
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
            
            if len(real_obstacles) > 300:
                real_obstacles = random.sample(real_obstacles, 300)
            if len(real_obstacles) > 60:
                real_obstacles.sort(key=lambda obs: math.hypot(obs[0]-pose[0], obs[1]-pose[1]))
                real_obstacles = real_obstacles[:60]
                
        # ================= 基于密度的智能聚类与预测 =================
        self.dwa_controller.predicted_dynamic_obs = [] 
        current_dynamic_safe_radius = self.dwa_controller.safe_radius
        is_dynamic_locked = False 

        if len(real_obstacles) > 0:
            for pt_candidate in real_obstacles:
                if is_dynamic_locked:
                    break 
                
                count_neighbors = 0
                for obs in real_obstacles:
                    if math.hypot(obs[0] - pt_candidate[0], obs[1] - pt_candidate[1]) < 0.3:
                        count_neighbors += 1
                
                if 3 <= count_neighbors <= 20:
                    cluster_points = []
                    for obs in real_obstacles:
                        if math.hypot(obs[0] - pt_candidate[0], obs[1] - pt_candidate[1]) < 0.4:
                            cluster_points.append(obs)
                    
                    if len(cluster_points) >= 3:
                        obs_array = np.array(cluster_points)
                        centroid_x = np.mean(obs_array[:, 0])
                        centroid_y = np.mean(obs_array[:, 1])
                        
                        estimated_state = self.kf_tracker.update(np.array([centroid_x, centroid_y]))
                        obs_vx = estimated_state[2]
                        obs_vy = estimated_state[3]
                        speed_magnitude = math.hypot(obs_vx, obs_vy)
                        
                        if speed_magnitude > 0.15:
                            is_dynamic_locked = True
                            self.get_logger().info(f'【锁定动态目标】速度: {speed_magnitude:.2f} m/s')
                            
                            # 生成幽灵点
                            predict_time = 1.0 
                            steps = 3 
                            for i in range(1, steps + 1):
                                dt_future = (predict_time / steps) * i
                                future_x = centroid_x + obs_vx * dt_future
                                future_y = centroid_y + obs_vy * dt_future
                                self.dwa_controller.predicted_dynamic_obs.append([future_x, future_y])
                            
                            # 动态膨胀安全半径
                            vec_to_obs_x = centroid_x - pose[0]
                            vec_to_obs_y = centroid_y - pose[1]
                            dist_to_obs = math.hypot(vec_to_obs_x, vec_to_obs_y)
                            
                            if dist_to_obs > 0.01:
                                relative_approach_speed = - (obs_vx * (vec_to_obs_x/dist_to_obs) + obs_vy * (vec_to_obs_y/dist_to_obs))
                                if relative_approach_speed > 0.1: 
                                    expanded_radius = self.dwa_controller.safe_radius + relative_approach_speed * 0.4
                                    expanded_radius = min(expanded_radius, 0.6) 
                                    current_dynamic_safe_radius = expanded_radius
        else:
            self.kf_tracker.is_initialized = False
        # ===============================================================
        
        # 预瞄点逻辑
        lookahead_target = self.path_points[-1]
        distances_to_path = np.linalg.norm(self.path_points - np.array(pose[:2]), axis=1)
        nearest_idx = np.argmin(distances_to_path)
        deviation_dist = distances_to_path[nearest_idx]
        
        target_lookahead_dist = 1.0
        if deviation_dist > 1.0:
            target_lookahead_dist = 2.0
            
        temp_dist = 0.0
        for i in range(nearest_idx, len(self.path_points) - 1):
            temp_dist += np.linalg.norm(self.path_points[i+1] - self.path_points[i])
            if temp_dist > target_lookahead_dist:  
                lookahead_target = self.path_points[i+1]
                break

        try:
            speed, steering_angle = self.dwa_controller.calculate_best_velocity(
                pose, self.current_v, self.current_w, lookahead_target, 
                real_obstacles, self.dwa_controller.predicted_dynamic_obs,
                current_dynamic_safe_radius)
            self.current_v = speed
            self.current_w = steering_angle
        except Exception as e:
            self.get_logger().error(f'DWA error: {e}')
            speed, steering_angle = 0.0, 0.0
            self.current_v = 0.0
            self.current_w = 0.0

        distance_to_end = np.linalg.norm(np.array(pose[:2]) - self.path_points[-1])
        if distance_to_end < 0.2:
            speed = 0.0
            steering_angle = 0.0
            self.current_v = 0.0
            self.current_w = 0.0
            
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