# 故障排查

| 现象 | 检查顺序 |
|---|---|
| 无宇树话题 | 物理网络、网卡、设备侧发布者、DDS domain、Cyclone 配置；检查是否混用了原生 SDK DDS 库 |
| 地图有但机器人位姿不对 | 初始位姿、地图版本、`map → odom → base_link`、里程计、时间戳 |
| 点云过滤不健康 | 原始点云是否刷新、腿部实时 TF、动态自体遮罩、输出点数与时间 |
| 点击初始位姿无效果 | 是否向 `/operator/initialpose` 发 PoseStamped、面板是否 map、定位适配器是否启动 |
| 点击目标被拒绝 | 查看目标门禁诊断、占用/未知区、净空、定位协方差、TF 和点云时效 |
| 路径存在但不运动 | dry-run、驱动 enable、速度链、Collision Monitor、运动锁、硬件状态 |
| 巡逻暂停 | 当前目标失败/重试情况、状态 JSON、路线数据、障碍与定位；处理后再 resume |
| 狭窄处恢复反复 | 真实 footprint、近障点云、局部轨迹、定位偏差；不要盲目缩小模型或清图 |
| 相机黑屏 | RTP 多播网卡、UDP 1720、GStreamer 插件、数据是否到达；相机缺失不自动证明导航故障 |
| Foxglove 无模型 | SDK 模型资源、安装路径、assets 能力、TF、模型显示图层 |
| 同路径修改不生效 | 先核对容器是否挂载源码，再确认 build/install 和 source 的工作区 |
| 会话锁冲突 | 停止自己启动的上一会话；确认残留进程，不按进程名批量终止机器人任务 |

## 诊断命令

在已加载正确工作区的终端：

```bash
python3 scripts/check_navigation_ready.py
ros2 topic echo /diagnostics --once
ros2 topic echo /waypoint_patrol/status --once
ros2 topic info /navigation/obstacles
ros2 param get /a2_driver dry_run
```

日志判断需结合现场状态。导航无法保证在所有障碍物材质、空间和定位条件下通行；异常时先停止当前任务，再保留记录分析。
