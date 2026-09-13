# 参数与 ROS 接口

以下为此次发布源配置的关键值；运行时参数以加载文件和 `ros2 param get/dump` 为准。距离单位 m、速度 m/s、角度 rad、角速度 rad/s、时间 s，除非另有标注。

| 配置 | 参数 | 默认值与意义 |
|---|---|---|
| `u_robot_bringup/config/a2_driver.yaml` | `dry_run` | true，不发 Sport 请求 |
| 同上 | `command_timeout_ms` | 250，过期命令停止 |
| 同上 | `max_vx / max_vy / max_wz` | 0.20 / 0.15 / 0.45，驱动限幅 |
| `u_robot_navigation/config/nav2_params.yaml` | `controller_frequency` | 20 Hz |
| 同上 | `xy_goal_tolerance / yaw_goal_tolerance` | 0.30 / 0.30，目标容差，不是实测精度 |
| 同上 | MPPI `time_steps / model_dt` | 72 / 0.05，预测时域约 3.6 s |
| 同上 | costmap `resolution` | 0.05；局部窗口 6×6 m |
| 同上 | `footprint_padding` | 0.03，围绕配置多边形增加余量 |
| 同上 | 局部/全局 `inflation_radius` | 0.65 / 2.50，软代价范围，不是碰撞边界 |
| `operator_goal_adapter.yaml` | `goal_clearance` | 0.55，目标点净空 |
| 同上 | `obstacle_timeout` | 0.50，目标检查的点云新鲜度 |
| `velocity_deadband_adapter.yaml` | `minimum_vx / minimum_vy` | 0.08 / 0.08，非零速度死区补偿 |
| 同上 | `minimum_wz` | 0.12；平移中最小 yaw 默认 0 |
| `collision_monitor.yaml` | `time_before_collision` | 1.0，预测接近碰撞的时间 |
| 同上 | `source_timeout` | 2.0，Collision Monitor 数据源超时 |
| `u_robot_perception/config/navigation_cloud_filter.yaml` | `min_height / max_height` | -0.24 / 0.80，相对 base_link 高度范围 |
| 同上 | `max_range / voxel_size` | 6.0 / 0.05 |
| 同上 | `require_dynamic_self_mask` | true，需要实时腿部 TF |
| `u_robot_localization/config/localization.yaml` | `min_particles / max_particles` | 800 / 4000 |
| 同上 | `set_initial_pose` | false |
| `u_robot_camera_bridge/config/front_camera.yaml` | 输出 | 960×540，JPEG 82，15 Hz |

所有导航配置位于 `src/<包>/config/`。`u_robot_patrol/config/patrol.yaml` 的点位、间隔及重试含义见[巡逻文档](patrol.md)。`u_robot_mapping/config/slam_toolbox.yaml` 和点云转 scan 的 mapping/navigation 配置应与实际激光安装及地图保持一致。

## 主要接口

| 话题/服务 | 用途 |
|---|---|
| `/dog_odom` → `/odometry/a2` | 原始/标准化里程计 |
| `/unitree/slam_lidar/points` → `/navigation/obstacles` | 融合输入与过滤障碍点云 |
| `/scan` | 建图与定位 LaserScan |
| `/map`、`/amcl_pose`、`/tf` | 地图、定位与坐标关系 |
| `/plan`、`/local_costmap/costmap`、`/global_costmap/costmap` | 路径和环境代价 |
| `/cmd_vel_nav`、`/cmd_vel`、`/cmd_vel_shaped`、`/cmd_vel_safe` | 四级速度链 |
| `/a2_driver/enable_control` | `std_srvs/srv/SetBool`，导航驱动开关 |
| `/diagnostics` | 感知、定位、门禁等诊断 |
| `/camera/front/image/compressed` | 前视相机 JPEG |

运行 `ros2 launch <包> <launch> --show-args` 查看顶层参数。当前导航入口没有暴露所有底层网络参数，非 `eth0` 部署应检查并调整相机/原生状态/运动后端一致的配置后再运行。
