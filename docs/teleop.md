# 独立遥控接收器

`u_robot_teleop` 封装已有 PC1 BLE/UDP → 机器人接收器链路。导航和遥控通过共享运动锁互斥；本仓库没有完整 Windows BLE sender 或接收器原始 C++ 工程。

## 当前包含

- ROS launch、接收器进程监督、配置校验与退出清理。
- 设备原有 `a2_network_bridge` 二进制及 SHA256 来源记录。
- 宿主机部署/启动脚本、PC1 SSH 心跳入口和离线测试。

详见 [接收器来源](../src/u_robot_teleop/vendor/PROVENANCE.md)。这里的 colcon 构建默认安装已有二进制，不等于从源码重编译该接收器。该部分独立于 Nav2 巡逻，缺少 PC1 外部组件时不能重现完整 BLE 控制。

## 运行示例

在标准容器路径构建完成后：

```bash
ros2 launch u_robot_teleop teleop.launch.py pc1_ip:=192.0.2.10
```

`192.0.2.10` 是文档示例地址，必须替换为实际 PC1 网络接口地址。默认 dry-run。真实遥控显式加 `dry_run:=false`，其控制入口与导航的 `/a2_driver/enable_control` 不同。

宿主机便捷入口：

```bash
bash scripts/run_teleop_docker.sh --container unitree-review --pc1-ip 192.0.2.10
```

`--real-control` 会启用真实接收器控制；首次仅检查通信时不要添加。停止使用 Ctrl+C。控制锁位于 `/home/unitree/data/logs/a2_sport_control.lock`，导航与遥控必须共享同一持久化目录。

此入口默认容器为 `unitree-review`，可通过首个 `--container` 参数或 `TELEOP_CONTAINER` 环境变量指定；显式参数优先。宿主机仓库可以位于任意目录，容器内工作区默认独立设为 `/home/unitree/unitree_robot_development/u_robot_move`。自定义容器挂载位置时用 `TELEOP_CONTAINER_WORKSPACE` 指定容器中的工作区根目录，例如：

```bash
TELEOP_CONTAINER_WORKSPACE=/opt/workspaces/u_robot_move \
  bash scripts/run_teleop_docker.sh --container custom-review --pc1-ip 192.0.2.10
```

## PC1 接入

`scripts/pc1_ble_docker.py` 保留原部署的标准宿主机路径约定，是已有 PC1 工程的迁移辅助入口。它在 SSH 目标宿主机查找 `/home/unitree/unitree_robot_development/u_robot_move/scripts/run_teleop_docker.sh`，不是到容器内查找；仅在宿主机确有该路径时使用。新仓库克隆到其他宿主机目录时，这一旧入口不能直接按通用快速开始运行。其原默认容器仍为 `unitree-dev`，对新审阅容器必须显式传入 `--container unitree-review`，并配置 SSH 与实际 PC1 IP。

该入口等待机器人端就绪后才启动可选 sender 子命令，通过 SSH stdin 心跳管理本次接收器。现有 sender 的实际命令和协议需从 PC1 获取，仓库不臆造其参数。

`scripts/deploy_teleop_docker.sh` 也是原标准路径下的迁移工具，要求宿主机与容器内仓库绝对路径相同，并准备原设备接收器文件。它保留 `unitree-dev` 默认目标，切换容器需设置 `TELEOP_CONTAINER`。脚本会复制并编译包，不属于只读检查，也不是新 Docker 挂载布局的通用安装步骤。新环境应按 Docker 仓库构建挂载工作区；仅在满足旧路径约定且明确管理的开发容器上使用此迁移脚本。

## 验证范围

自动测试覆盖部分配置、共享锁、dry-run UDP、重复启动和 EOF 清理。完整 BLE、SDK、ACK、实际通信看门狗和实体停车仍需要 PC1 与机器人现场验证。本次发布不启动这些真实控制链路。
