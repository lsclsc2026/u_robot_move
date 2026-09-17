# 来源、依赖与许可

本仓库是现有 Unitree A2 Pro 工程的私有审阅快照。各源文件和 package.xml 已有的作者/许可声明保留；本次整理不另行选择或授予新的仓库级开源许可证。对外公开前需核对原创代码和设备交付材料的许可。

| 组件 | 来源与用途 |
|---|---|
| Unitree SDK2 | `unitreerobotics/unitree_sdk2`，A2 原生接口及 DDS；版本由 Docker 仓库依赖清单固定 |
| Unitree ROS 2 | `unitreerobotics/unitree_ros2`，消息定义 |
| Unitree ROS 模型 | `unitreerobotics/unitree_ros`，官方 A2 URDF/mesh，构建时保留 LICENSE |
| Nav2 / AMCL | ROS 2 Humble 依赖，规划、控制、定位与碰撞监控 |
| SLAM Toolbox | ROS 2 Humble 依赖，平面 SLAM |
| Foxglove Bridge | 可视化桥接 |
| OpenCV / GStreamer | 相机解码与图像发布 |
| 遥控接收器二进制 | 设备原有程序；[来源与 SHA256](src/u_robot_teleop/vendor/PROVENANCE.md)，原始源码待取得 |

共享 SDK 未复制到本仓库，使用 [宇树机器人容器化开发环境](https://github.com/lsclsc2026/unitree_docker) 的固定版本获取方式。Nav2、SLAM 和 SDK 能力分别归属于上游，本项目提供设备适配、配置、任务逻辑及工具。

源码主要来自正在使用的容器工作区，另补入同版本宿主机已有的遥控部署与启动脚本。完整源文件校验清单见 `docs/source-manifest.json`。构建产物、bag、云端账号、交接提示词和原始视频不进入仓库。
