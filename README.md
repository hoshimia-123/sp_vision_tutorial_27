# 27算法组导航方向招新大作业 - 迷宫点击导航

机器人从迷宫左下角 (0.9, 0.9) 出发，在 RViz 里对右上角终点 (14.1, 14.1) 点击一次，经过上层行为树、下层行为树和 A* 规划后，由我写的控制器插件跟踪路径走到终点。

## 运行方法

环境：Ubuntu 22.04 + ROS 2 Humble，依赖按作业说明安装。仿真器还需要 Python 版 OpenCV：

```bash
sudo apt install -y python3-opencv
```

编译：

```bash
cd ~/sp_vision_tutorial_27
colcon build --symlink-install
```

启动：

```bash
bash bash/sim_nav_all_start.sh
```

等 RViz 和仿真窗口都打开后，用 RViz 工具栏里的 2D Goal Pose 在右上角终点的通道中间点一次。

## 整体流程

1. RViz 发布 /goal_pose，sp_decision 收到后执行上层行为树 click_nav.xml
2. 上层行为树锁定云台、设置底盘跟随，然后通过 navigate_to_pose action 发出导航任务
3. nav_interface_node 执行下层行为树 default_nav_with_fallback.xml：通过 TF 获取 map 到 base_link 的位姿，调用 make_plan 服务
4. planner_server 用 A* 规划路径，发布到 /global_path
5. controller_server 加载我的 PidController 插件跟踪路径，把速度发到 /sentry/cmd_vel
6. sim_robot_node 接收速度，更新机器人位置，并发布 /Odometry 和 TF

## 我做的修改

### 1. 补全配置

把 sp_nav_bringup/config/nav_params.yaml 里空着的话题、坐标系、插件参数补全，让各个节点之间对得上：

- 全局代价地图 /global_costmap，全局路径 /global_path，速度指令 /sentry/cmd_vel
- 坐标系 map -> lidar_odom -> base_link，里程计 /Odometry

### 2. 控制器 pid_controller.cpp

一开始我按差速小车的思路写，用 angular.z 转向，结果机器人一直绕小圈，走得很慢，到拐角就卡住。看了仿真器的代码后发现两点：

- 仿真机器人是全向底盘，sim_robot_node 只读 cmd_vel 的 linear.x 和 linear.y，根本不读 angular.z
- base_link 的朝向跟着云台转，云台默认是全向扫描模式，大约每秒转 1 rad，而速度指令是按 base_link 坐标系执行的

所以我把控制器改成这样：

1. 在路径上找离机器人最近的点，再往前找第一个距离不小于 lookahead_distance 的点，作为前视点
2. 用 PID 算速度大小，误差是机器人到前视点的距离：speed = kp * d + ki * 积分 + kd * 微分，再限制在 0 到 max_linear_speed 之间
3. 速度方向指向前视点，得到 map 坐标系下的速度 (vx_map, vy_map)
4. 用当前 yaw 把速度旋转到 base_link 坐标系：
   vx = cos(yaw) * vx_map + sin(yaw) * vy_map
   vy = -sin(yaw) * vx_map + cos(yaw) * vy_map
5. angular.z 设为 0，进入 goal_tolerance 范围就停车

快到终点时，到前视点的距离会变短，速度会自己降下来。

### 3. 上层行为树 click_nav.xml

在 NavToPose 前面加了两个节点：

```xml
<GimbalControl mode="2"/>
<ChassisMode mode="1" is_stop="false" rotate_velocity="0.0"/>
```

mode 2 是云台 yaw 锁定，mode 1 是底盘跟随。原因是 base_link 一直转的时候，控制器用的是上一次收到的 yaw，位姿只要有一点延迟，算出来的速度方向就会偏，拐角处离墙近就容易蹭墙。底盘小陀螺转起来时，方形底盘的角也容易扫到墙。导航结束后后面原有的节点会把模式恢复。

### 4. A* 规划器 astar_planner.cpp

下层行为树会不停地从机器人当前位置重新规划。机器人过弯贴墙时，当前位置会落在代价地图的膨胀区里（cost = 100），原来的代码会直接返回空路径，导航就一直卡着。我改成了在 1 m 范围内找最近的可通行格子当起点，这样能把机器人重新引回走廊中间。

## 参数

PidController 的参数在 nav_params.yaml 里：

| 参数 | 值 | 说明 |
|---|---|---|
| linear_kp | 3.0 | 直道上 kp 乘前视距离大于最大速度，可以按最大速度走 |
| linear_ki / linear_kd | 0.0 | 前视点误差基本不变，不需要积分和微分 |
| max_linear_speed | 1.0 | 单位 m/s，兼顾用时和过弯安全 |
| lookahead_distance | 0.5 | 单位 m，太短会抖，太长拐角会切弯贴墙 |
| goal_tolerance | 0.15 | 单位 m，控制器停车距离，行为树判定到达是 0.3 m |

## RViz 显示

- 全局代价地图 /global_costmap
- 局部代价地图 /local_costmap/costmap
- 全局路径 /global_path（绿线）
- 机器人位姿（TF 里的 base_link）
- 起点和终点标记 /plan_markers

## 测试结果

| 项目 | 结果 |
|---|---|
| 是否到达 | 是 |
| 用时 | 50.26s |
| 最终离终点距离 | 0.294m |

在我的虚拟机上调试时遇到一个问题：仿真器发布局部代价地图的那段 Python 循环太慢，要求 30 Hz，实际只有 3 Hz 左右，把仿真器的 ROS 线程占满了，导致 /Odometry 发不出来、速度指令也处理不了，机器人看起来不动或者走走停停。这是虚拟机性能不够，和导航代码无关，在性能正常的电脑上用原版仿真参数就能正常跑。

## 还可以改进的地方

- 对 A* 路径做平滑，减少折线拐角
- 根据路径曲率规划速度，拐弯前减速，直道加速
- 尝试 LQR 或 MPC 控制器