# 多点巡逻

![点位与路径](media/patrol.jpg)

巡逻节点保存地图坐标下的位姿，逐个提交 Nav2 `NavigateToPose` Action。启动巡逻会产生导航任务；真实运动仍取决于导航驱动状态。

## 启动与打点

```bash
ros2 launch u_robot_navigation navigation.launch.py \
  map:=/home/unitree/data/maps/site.yaml \
  waypoints_file:=/home/unitree/data/maps/site_waypoints.yaml \
  dry_run:=true
```

若省略 `waypoints_file`，程序按地图名称推导独立点位文件。先设置定位，再配置 Foxglove 的 `/operator/waypoint`：

- `PointStamped`（2D Point）：沿路线自动推导朝向。
- `PoseStamped`（2D Pose）：保留手动拖出的朝向。

按期望行走顺序添加，点位显示为 A/B/C… 并立即保存。未知/占用区域、越界、净空不足或距离已有点过近会被拒绝；默认净空 0.55 m、重复距离 0.20 m、上限 100 点。使用一种消息类型配置面板即可。

## 操作接口

全部命令既可通过 `/operator/patrol_command` 的 `std_msgs/msg/String` 发送，也可通过 `/waypoint_patrol/<命令>` 的 `std_srvs/srv/Trigger` 调用。

| 命令 | 用途 |
|---|---|
| `start_once` | 依次访问一轮 |
| `start_loop` | 循环访问点位 |
| `pause` | 暂停当前任务 |
| `resume` | 恢复暂停任务 |
| `stop` | 停止巡逻并取消当前目标 |
| `undo` | 撤销最后添加的点 |
| `clear` | 清空保存的路线 |
| `reload` | 从当前点位文件重新读取 |
| `add_current_pose` | 将机器人当前地图位姿添加到路线 |

```bash
ros2 service call /waypoint_patrol/start_loop std_srvs/srv/Trigger "{}"
ros2 service call /waypoint_patrol/pause std_srvs/srv/Trigger "{}"
ros2 service call /waypoint_patrol/resume std_srvs/srv/Trigger "{}"
ros2 service call /waypoint_patrol/stop std_srvs/srv/Trigger "{}"
```

Foxglove 发布消息示例：`{"data":"start_loop"}`。`/waypoint_patrol/status` 输出 JSON 字符串状态，`/waypoint_patrol/active` 表示任务活动状态，`/navigation/waypoints` 为可视化标记。

## 行为与失败处理

成功到达后默认等待 1 秒，再请求 AMCL 静止观测并等待 1 秒，之后继续下一点。目标失败时默认间隔 5 秒重试，最多 3 次；达到上限暂停在当前点，不默默跳过。

默认质量评分作为辅助诊断；它不能替代碰撞监控，也不保证所有定位异常会自动停止巡逻。近后方目标在距离不超过 3 m、方位角超过约 120° 等条件下可选择专用倒退路径。路线配置不等于任何指定点一定可达。

需修改点位时先停止任务，备份点位 YAML。`clear` 和重新保存会改变路线数据。真实巡逻前按[导航说明](navigation.md)完成单目标检查。
