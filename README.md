# ROS2 移动机器人自主导航系统 (Enhanced A* & DWA)

本项目基于 ROS2 (Humble) 和 Gazebo 仿真环境，构建了一个完整的移动机器人建图与导航框架。在原开源项目的基础上，针对原有的 APF（人工势场法）容易陷入局部极小值的问题，**重点重构了局部路径规划算法**，实现了具备运动学约束的 **DWA（动态窗口法）** 结合 **A* 全局搜索**的混合导航策略。


相比于原始版本，本项目主要完成了以下深度优化：

1. **局部规划器重构 (APF -> DWA)：**
   * 废弃了原有的纯追踪与人工势场法，从零实现 DWA 算法。
   * 引入了小车底盘的物理运动学约束（最大线/角速度、加速度），使得生成的轨迹完全符合实体机器人的物理极限，消除了原地剧烈震荡现象。
   * 自定义了包含 `Heading` (航向)、`Distance` (避障) 和 `Velocity` (速度) 的三维评价函数。

2. **死锁防范与倒车**
   * 针对极度拥挤的 U 型障碍物通道，加入了防死锁判断及小车掉头角度进行调参优化



3. **全局 A* 算法微调：**
   * 引入B采样插值，提高A*算法的平滑度


## 🛠️ 环境依赖与运行
* Ubuntu 22.04 + ROS2 Humble
* `ros-humble-gazebo-ros-pkgs`
* `scipy`, `numpy`

**运行指令：**
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch nav_slam 2dpoints.launch.py
## 安装运行

```
git clone https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system.git
```

## 运行测试
###  启动差速仿真
ros2 launch gazebo_modele gazebo.launch.py

###  启动导航
```
ros2 launch nav_slam 2dpoints.launch.py
```
## 特别致谢：本项目参考https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system.git对其纯跟踪控制以及A*全局规划进行修改

