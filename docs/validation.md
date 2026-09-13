# 验证与运行记录

## 本次发布边界

2026-09-13 发布整理以运行容器的导航源码为基础，补入宿主机已有的两个遥控部署/启动脚本。独立音频包放在另一个仓库，旧交接提示词与历史进度文档已改写成面向用户的说明。

本次不启动真实运动、不重启现有容器，也不重新做实机导航。离线检查结果记录在 [release-validation.md](release-validation.md)。演示视频与每一次算法修改的精确对应关系尚未完整建立，视频不证明所有最新参数已经通过连续巡逻验收。

## 自动检查

在准备好的独立 ROS 环境中构建并测试：

```bash
./scripts/build.sh
source install/setup.bash
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
```

现有测试涵盖驱动命令保护、里程计与原生关节包、定位健康逻辑、遥控监督等。参数解析、行为树 XML 和 Python/Bash 语法也纳入发布检查。未接通硬件的测试无法判断定位精度、现场避障效果或任务成功率。

## 实机试验记录

每次试验记录源码提交、镜像/依赖版本、地图/点位、起始位姿、场地变化、目标结果、人工介入和异常原因。先启动导航，再开始录制，之后设置初始位姿和巡逻：

```bash
NAV_TRIAL_ROOT=/home/unitree/data/rosbags/navigation_trials \
  ./scripts/record_navigation_trial.sh patrol_review /home/unitree/data/maps/site.yaml
```

人工介入时显式标记：

```bash
./scripts/mark_navigation_intervention.sh BEGIN "人工遥控接管，记录原因"
./scripts/mark_navigation_intervention.sh END "接管结束，记录结果"
```

录制停止后分析实际生成的试验目录：

```bash
python3 scripts/analyze_navigation_trial.py /home/unitree/data/rosbags/navigation_trials/TRIAL_DIRECTORY
```

`TRIAL_DIRECTORY` 替换为录制工具输出的目录。记录包含参数快照、地图相关信息、路径、各级速度、里程计、AMCL、TF、costmap、点云和相机等。原始 bag 不进入 Git。

## 待实机补充

- 在新场地确认地图几何、初始定位与持续定位稳定性。
- 单目标、转弯、窄通道、动态障碍与后向目标测试。
- 多轮巡逻、失败重试、暂停/恢复、取消和控制禁用。
- 人工接管和遥控/导航互斥在完整硬件链路中的表现。
- 对应源码提交的第三人称视频与无人工介入试验记录。

已有近障碰撞、漂移和恢复策略相关历史问题不能仅凭编译通过宣告解决。准确率/成功率等数值只在具有明确样本、失败标准和原始记录时报告。
