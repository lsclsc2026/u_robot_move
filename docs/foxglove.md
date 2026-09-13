# Foxglove 配置

## 连接

桥接默认只监听机器人端 `127.0.0.1:9000`。在操作电脑执行：

```bash
ssh -N -L 9000:127.0.0.1:9000 unitree@ROBOT_HOST
```

将 `ROBOT_HOST` 替换为实际地址或 SSH 别名。Foxglove 连接 `ws://localhost:9000`。若本地 9000 已占用，可转发到其他本地端口，连接地址相应修改。

## 推荐面板

| 面板 | 设置 |
|---|---|
| 地图 3D | 固定/显示参考系 `map`；显示地图、scan、路径、footprint、点位 |
| Image | `/camera/front/image/compressed` |
| 模型 3D | 显示机器人模型和 TF，不叠加地图；参考系按现场 TF 选择 |
| Diagnostics | 感知、定位和目标门禁状态 |
| Raw Messages / Plot | 巡逻状态、定位质量及速度链调试 |

调试时分别查看局部和全局 costmap，避免图层重叠导致误判。模型依赖 SDK 中的官方 A2 meshes，不能只凭 TF 正常就判断模型资源齐全。

## 点击工具和权限

| 操作 | 话题 | 消息类型 |
|---|---|---|
| 设置初始位姿 | `/operator/initialpose` | `geometry_msgs/msg/PoseStamped` |
| 单目标导航 | `/operator/goal_pose` 或 `/move_base_simple/goal` | `geometry_msgs/msg/PoseStamped` |
| 添加巡逻点 | `/operator/waypoint` | `geometry_msgs/msg/PointStamped` 或 `PoseStamped` |
| 巡逻命令 | `/operator/patrol_command` | `std_msgs/msg/String` |
| 取消任务 | `/operator/cancel_navigation` | `std_msgs/msg/Empty` |

初始位姿使用 PoseStamped 适配入口，不是直接让 Foxglove 发布 AMCL 协方差消息。建图默认不开放定位和导航写入；定位开放初始位姿；导航额外开放目标、取消与巡逻。源码中还开放独立语音模块的限定接口；语音未启动时没有相应服务。

浏览器不开放任意速度、Sport API 或导航驱动 enable 服务；真实底盘启用需从 ROS 终端进行。上述限制描述项目桥接配置，不替代网络访问控制。

## 其他显示入口

`u_robot_bringup/live_visualization.launch.py` 运行只读里程计/模型显示；`u_robot_description/a2_urdf_preview.launch.py` 提供站姿模型预览。模型预览默认使用独立 domain/端口，具体参数请用 `--show-args` 查看，避免与正式会话混用。
