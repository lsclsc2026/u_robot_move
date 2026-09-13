# 安装与环境

## 前提

- Unitree A2 Pro 的高层运动接口、里程计、融合点云和前视相机可用。
- 机器人网络接口默认为 `eth0`，原生 DDS domain 默认为 0。
- Ubuntu 22.04、ROS 2 Humble；ROS 使用发行版 CycloneDDS，原生 SDK 后端使用独立进程。
- 准备 [unitree_docker](https://github.com/lsclsc2026/unitree_docker) 所述环境和 SDK，保持同级目录布局。

主要原始输入为 `/dog_odom`、`/unitree/slam_lidar/points`、原生 `rt/lf/lowstate` 和相机 RTP/H.264 多播。宇树设备侧点云融合服务不由本仓库实现；仅启动此工程不会自动补齐缺失的设备驱动。

## 构建

在配置好的容器中：

```bash
cd /home/unitree/unitree_robot_development/u_robot_move
./scripts/build.sh
source install/setup.bash
unset CYCLONEDDS_URI
```

构建脚本同时收集 `src/` 与 `../sdk/unitree_ros2/cyclonedds_ws/src/unitree/` 下的 `unitree_api`、`unitree_go`、`unitree_hg` 消息包。模型来自 `../sdk/unitree_ros/robots/a2_description`。不要单独复制 `src/` 而遗漏共享 SDK。

SDK2 的原生 DDS 依赖由 CMake 为各后端定位。不要把 SDK 自带的 DDS 库目录全局加入 ROS 的 `LD_LIBRARY_PATH`，否则可能与 ROS 自带 CycloneDDS 混用。

## 路径约定

| 容器路径 | 用途 |
|---|---|
| `/home/unitree/unitree_robot_development/u_robot_move` | 源码及构建工作区 |
| `/home/unitree/unitree_robot_development/sdk` | 第三方依赖 |
| `/home/unitree/data/maps` | 地图、位姿图、巡逻点位 |
| `/home/unitree/data/rosbags` | 采集数据 |
| `/home/unitree/data/logs` | 日志和共享运动控制锁 |

当前 `nav2_params.yaml` 中行为树路径，以及录制、遥控脚本中的默认路径依赖上述布局。首版按标准容器路径使用；若迁移到其他用户目录，需要同步修改并验证这些入口。

已有容器不一定挂载源码，即使宿主机路径与容器相同也不能认定文件同步。发布用的 Docker 入口采用明确挂载；检查现有部署时先查看 `docker inspect <容器名>` 的 Mounts。

## 环境检查

```bash
./scripts/check_environment.sh
ros2 topic list
ros2 topic info /dog_odom
ros2 topic info /unitree/slam_lidar/points
```

这些步骤是只读检查。成功列出话题不等于数据新鲜或 TF 正确；启动导航后还应执行 `python3 scripts/check_navigation_ready.py`。不连接机器人时，部分输入缺失是预期情况。

## 编译后检查

```bash
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
```

具体本次执行结果见[验证记录](validation.md)。自动测试不覆盖完整场地通行和硬件停车行为。
