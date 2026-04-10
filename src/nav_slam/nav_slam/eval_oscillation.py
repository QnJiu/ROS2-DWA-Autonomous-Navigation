import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

class OscillationEvaluator(Node):
    def __init__(self):
        super().__init__('oscillation_evaluator')
        # 订阅底盘的底层控制指令
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.angular_z_data = []
        self.is_recording = False
        self.get_logger().info('震荡评估节点已启动，等待小车移动...')

    def cmd_vel_callback(self, msg):
        # 如果线速度 > 0.05，说明小车正在通道内行驶
        if msg.linear.x > 0.05:
            if not self.is_recording:
                self.get_logger().info('小车开始移动，正在记录角速度数据...')
                self.is_recording = True
            self.angular_z_data.append(msg.angular.z)
            
        # 如果线速度降为 0，说明小车停下了（到达终点或卡死）
        elif self.is_recording and msg.linear.x <= 0.05:
            self.get_logger().info('小车停止，开始结算震荡指标...')
            self.evaluate_data()
            self.is_recording = False
            self.angular_z_data = [] # 清空列表，为下一次测试做准备

    def evaluate_data(self):
        if len(self.angular_z_data) < 10:
            self.get_logger().warn('收集到的数据太少，无法进行有效评估！')
            return

        # 调用 Numpy 计算标准差（核心量化指标）
        std_dev = np.std(self.angular_z_data)
        max_val = np.max(self.angular_z_data)
        min_val = np.min(self.angular_z_data)

        self.get_logger().info('\n' + '='*40)
        self.get_logger().info('      A/B 测试角速度震荡报告      ')
        self.get_logger().info('='*40)
        self.get_logger().info(f'有效采样点数 : {len(self.angular_z_data)}')
        self.get_logger().info(f'最大向左角速度: {max_val:.4f} rad/s')
        self.get_logger().info(f'最大向右角速度: {min_val:.4f} rad/s')
        self.get_logger().info(f'>>> 角速度标准差 (Std Dev): {std_dev:.4f} <<<')
        self.get_logger().info('='*40 + '\n')

def main(args=None):
    rclpy.init(args=args)
    node = OscillationEvaluator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()