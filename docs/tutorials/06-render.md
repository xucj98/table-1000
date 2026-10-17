# 06 渲染专家轨迹与导出 LeRobot

把第五章录制的轨迹渲染成视频、机器人相机图像和评审图，再导出 LeRobot v3.0 数据集。渲染按录制的状态逐步恢复场景，不重新仿真，画面与录制的轨迹一致。

**输入：** 第五章的 `trajectories.hdf5`。

**输出：** 每条轨迹的视频、评审图、机器人相机图像；LeRobot 数据集。

## 1. 渲染

```bash
"$ISAAC_PYTHON" scripts/scenes/render.py outputs/scenes/scene-000000/gen/trajectories.hdf5 --output outputs/scenes/scene-000000/render --count 5 --gpu 0
```

`--count 5` 随机选 5 条（`--seed` 固定选择），`--demos demo_3 demo_7` 指定，默认全部。

每一步渲染前：

1. 用 `scene.reset_to(states[t])` 恢复状态；
2. 调用 PhysX 的 `update_transformations` 把位姿写进 USD（不推进物理时渲染器读不到新位姿）；
3. 渲染两次，保留第二次（RT2 在时间上降噪，第一次会混入上一个状态）。

每条轨迹 `<output>/<demo>/`：

| 文件 | 内容 |
| --- | --- |
| `wide.mp4`、`close.mp4`、`top.mp4` | 场景的固定视角，25 fps 即 1 倍速，标注时间与子任务 |
| `sheet.jpg` | 第一个视角在整条轨迹上均匀取的 12 帧，标注子任务，供大模型逐条评审、人工抽查 |
| `frames/<相机>/%05d.jpg` | 机器人相机每一步之前的图像（该步动作对应的观测），不加标注 |

本例每条轨迹（约 700 步、3 个视角与 3 路机器人相机）渲染约 RENDER_TIME。只需要评审图时加 `--views wide --cameras` 省去机器人相机。

## 2. 导出 LeRobot

LeRobot 需要的包版本与 Isaac Lab 冲突（如 `packaging`），装在单独的环境里：

```bash
UV_PROJECT_ENVIRONMENT=.venv-data uv sync --locked --extra data
.venv-data/bin/python scripts/scenes/to_lerobot.py outputs/scenes/scene-000000/gen/trajectories.hdf5 --frames outputs/scenes/scene-000000/render --output outputs/scenes/scene-000000/lerobot
```

已渲染的每条轨迹成为一个 LeRobot 回合（25 fps）：

| 字段 | 内容 |
| --- | --- |
| `observation.state` | 关节位置（机器人关节，再加底盘虚拟关节；名字写在 `meta/info.json`） |
| `action` | 每步的动作，名字为 `<动作组>.<序号>`，动作组见轨迹文件的 `action_layout` |
| `observation.images.<相机>` | 每路机器人相机，存成视频 |
| `task` | 该步所属子任务的英文指令；`--task instruction` 改用高层指令 `tidy up the table` |

读取：

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset
dataset = LeRobotDataset('table-1000/scene-000000', root='outputs/scenes/scene-000000/lerobot', video_backend='pyav')
```

LeRobot 默认用 torchcodec 解码视频，它需要系统里有 FFmpeg 4–7 的共享库；没有时用 `video_backend='pyav'`。
