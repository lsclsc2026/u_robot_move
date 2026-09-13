# 单目标导航

## 1. 预演

```bash
ros2 launch u_robot_navigation navigation.launch.py \
  map:=/home/unitree/data/maps/site.yaml dry_run:=true
```

默认自动激活 Nav2 生命周期节点，但不发送 Sport 请求。设置[初始位姿](localization.md)后运行：

```bash
python3 scripts/check_navigation_ready.py
```

`READY` 表示脚本检查到地图、定位、点云及目标门禁符合其条件，不等于真实场地已经完成全部验收。

## 2. 设置目标

Foxglove 3D 面板使用 `map` 参考系，2D Pose 工具向 `/operator/goal_pose` 或 `/move_base_simple/goal` 发送 `geometry_msgs/msg/PoseStamped`。在空闲区域按下并拖动，指定位置与最终朝向。

门禁检查地图边界、占用状态、0.55 m 目标净空、有效定位、TF 与新鲜点云，随后调用 Nav2。直接向 Nav2 的其他目标接口发消息可能绕过本工程的目标校验，不作为操作入口。

观察 `/plan`、costmap、机器人 footprint 和 `/cmd_vel_safe`。dry-run 下机器人不运动，因此不能用它测量真实到达精度或完整闭环巡逻成功率。

## 3. 真实运动

操作者先确认场地、定位、遥控和停止手段，再停止 dry-run 会话并启动：

```bash
ros2 launch u_robot_navigation navigation.launch.py \
  map:=/home/unitree/data/maps/site.yaml dry_run:=false
```

重新设置初始位姿并检查状态。在第二个已加载工作区的终端显式启用驱动：

```bash
ros2 service call /a2_driver/enable_control std_srvs/srv/SetBool "{data: true}"
```

先验证近距离单目标，再使用多点巡逻。驱动最大绝对速度配置为 x 0.20 m/s、y 0.15 m/s、yaw 0.45 rad/s；Nav2 和速度平滑器还可能进一步限制。不要把采样器的 `vx_max=0.22` 当作最终硬件输出上限。

## 4. 取消与停止

取消当前目标，同时请求停止巡逻：

```bash
ros2 topic pub --once /operator/cancel_navigation std_msgs/msg/Empty "{}"
```

禁用导航底盘控制：

```bash
ros2 service call /a2_driver/enable_control std_srvs/srv/SetBool "{data: false}"
```

任务取消、软件禁用与实体急停是不同层次的操作。软件链路异常时应使用现场已验证的遥控/硬件停止手段。退出会话前先停止任务与控制。

## 5. 参数调整原则

先从日志区分定位、地图、点云、规划器或执行器模型问题。不要缩小 footprint 掩盖近障碰撞，也不要针对某一个 waypoint 字母写运动特例。完整配置见[参数与接口](configuration.md)，本次上传未调整运动行为。
