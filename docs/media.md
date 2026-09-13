# 演示素材

现有三段视频来自项目实机界面录制，均为 H.264、30 FPS，无音轨。原片未改写，也未提交到 Git。

| 素材 | 原时长/大小 | 展示版 |
|---|---|---|
| 建图 | 325.17 s / 320.9 MiB | mapping-10x.mp4，10 倍速 |
| 定位与快速打点 | 133.23 s / 103.4 MiB | localization-3x.mp4，3 倍速 |
| 打点巡航 | 378.70 s / 353.9 MiB | patrol-10x.mp4，10 倍速 |

视频下载见 [v0.1.0-review Release](https://github.com/lsclsc2026/u_robot_move/releases/tag/v0.1.0-review)。私有仓库附件需要登录有权限的账号。README 使用封面链接和短 GIF；MP4 链接的播放/下载方式由浏览器决定。

## 重新制作

安装 FFmpeg 后执行：

```bash
bash scripts/prepare_demo_media.sh /path/to/original/videos /path/to/new/output
```

输入文件名需与原片名称一致：`狗子建图1.mp4`、`狗子定位+快速打点.mp4`、`狗子打点巡航.mp4`。工具输出 1440 宽、H.264、30 FPS、yuv420p、faststart 展示版及截图/GIF，拒绝覆盖已存在文件。视频和 GIF 左上角标明播放倍速，文件名和 README 同样标注。FFmpeg 需支持 drawtext 并能找到字体；精简系统可安装 `fonts-dejavu-core`。需要 15 倍速时调整脚本对应 encode 的 speed 参数，并输出到新的目录。

不要把帧率改变当作唯一的时间加速方法；脚本使用 setpts 缩短时间轴后重新编码。播放时长和编码质量须通过 ffprobe 与实际观看检查。

## 等待补录

| 后续素材 | 建议内容 | 文档位置 |
|---|---|---|
| 第三人称实机巡逻 | 20–40 秒，能看见机体、路径环境、转弯及到点 | README 演示表新增实拍链接 |
| 对应版本的整轮验收 | 注明源码提交、地图、是否人工接管 | validation.md |
| 硬件照片 | 机体与传感器布局 | installation.md |

在提供素材前保留上述文字位置，不使用伪造画面或失效图片链接。
