# Pure-tracking-slam-automatic-navigation-system
simulate，ros2，gazebo，navigation，slam



<!--
 * @作者: boxing
 * @b号: 喵了个水蓝蓝
 * @描述: README
-->
## 注意当前分支代码为humble版本

# 基于ROS2实现的差速机器人，slam（建图定位），路径规划（A*），导航控制（纯追踪）

![image](https://github.com/user-attachments/assets/baac6889-d251-4891-8d21-c47fa4b45a33)


# 还包含su7模型的阿克曼底盘
<img width="2344" height="1400" alt="image" src="https://github.com/user-attachments/assets/9a933d84-e864-4fbf-98d5-312a79838130" />
此模型放在：
链接: https://pan.baidu.com/s/1geyZbNclzaeNOMe9bcLUvg?pwd=rwqb 提取码: rwqb


## 安装运行

```
git clone https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system.git
```

## 运行测试
###  启动差速仿真
```
ros2 launch gazebo_modele gazebo.launch.py
```
###  启动阿克曼su7仿真
```
ros2 launch four_wheeled_vehicle vehicle_gazebo_ok.launch.py
```
###  启动导航
```
ros2 launch nav_slam 2dpoints.launch.py
```
## 演示视频
https://www.bilibili.com/video/BV1kzEwzuEFw?spm_id_from=333.788.videopod.sections&vd_source=134c12873ff478ea447a06d652426f8f

联系：clibang2022@163.com
