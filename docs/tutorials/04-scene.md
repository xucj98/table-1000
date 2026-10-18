# 04 场景与专家轨迹

用第 2 章的物体和第 3 章的机器人搭一个场景，编写专家程序，批量生成专家轨迹，渲染后评审，再导出 LeRobot 数据集。以[场景 000000](../../asset_sources/scenes/000000/) 为例：X1Pro 站在桌前，桌上一个三抽屉柜和一支笔；机器人打开最下层抽屉，把笔放进去，再关上。各文件的要求见[场景与专家轨迹规范](../designs/scenes.md)。

![场景 000000 的 25 个初态](../../asset_sources/scenes/000000/initial_states.jpg)

## 交付物

源码目录 `asset_sources/scenes/<编号>/`，由 AI 按规范编写，提交到 Git：

| 文件 | 作用 |
| --- | --- |
| `scene.py` | 场景：桌子、机器人及起始姿态、物体、录像视角，以及每个 seed 怎样随机摆放（`initialize`） |
| `expert.py` | 专家程序 `solve(ctx)`：按英文子任务组织的阻塞式代码，用运动原语、几何查询和抓取几何完成任务 |
| `initial_states.jpg` | 25 个初态的拼图（缩小），作为场景的缩略图 |

生成的结果不进 Git：

| 位置 | 内容 |
| --- | --- |
| `assets/scenes/<编号>/preview/` | 初态检查 `initial.json` 与拼图 `initial_states.jpg`；每条渲染过的轨迹一个 `demo_<seed>/`：评审图、视频、机器人相机图像 |
| 生成目录（`--output`） | 轨迹文件 `trajectories.hdf5`（成功）、`trajectories_failed.hdf5`（失败），每个 seed 的报告 `episodes/<seed>.json`，汇总 `summary.json` |
| LeRobot 目录（`--output`） | LeRobot v3.0 数据集 |

轨迹文件是 Isaac Lab 录制器的格式，可在物理中重放，也可逐步恢复状态；带图片的数据只来自渲染后的 LeRobot 数据集。

**本例场景**：抽屉柜放在桌子右侧，位置 ±5 cm、偏航 ±15°；笔在机器人右手前方，位置 ±5 cm、朝向任意。专家分四个子任务：`open the bottom drawer of the cabinet`、`pick up the pen`、`put the pen into the drawer`、`close the drawer`。笔朝向任意，所以抓笔和放笔各有几种物理上等价的做法，按顺序尝试，见 [expert.py](../../asset_sources/scenes/000000/expert.py)。

## 命令

**检查初态**：对 seed 0–24 做与生成时相同的初始化（无穿透检查、静置 1 s），结果写入预览目录，约 2 分钟：

```bash
uv run python scripts/scenes/check_scene.py 000000 --gpu 0
```

**生成轨迹**：一个进程连续跑一段 seed。100 个 seed 分 4 个进程在 4 张卡上约 30 分钟：

```bash
uv run python scripts/scenes/generate.py 000000 --seeds 0:25 --output outputs/scenes/000000/gen/part0 --gpu 0
uv run python scripts/scenes/generate.py 000000 --seeds 25:50 --output outputs/scenes/000000/gen/part1 --gpu 1
# ……其余各段同理，然后汇总：
uv run python scripts/scenes/summarize.py outputs/scenes/000000/gen/part* --output outputs/scenes/000000/gen/summary.json
```

批量运行用固定提交的独立检出，运行期间不改代码、场景和资产。

**渲染**：按录制的状态逐步恢复场景再渲染，不重新仿真。先为所有成功轨迹渲染评审图（每条约 1 s，另有约 1 分钟启动），再随机选 5 条渲染视频和机器人相机图像（每条约 2.5 分钟）：

```bash
uv run python scripts/scenes/render.py outputs/scenes/000000/gen/part0/trajectories.hdf5 --sheets --gpu 0
uv run python scripts/scenes/render.py outputs/scenes/000000/gen/part0/trajectories.hdf5 --count 5 --gpu 0
```

- 每次给一个轨迹文件；分段生成时每段各运行一次，或用 `--demos demo_27 demo_51` 指定轨迹。
- `--count` 随机选择，`--seed` 固定选法；`--views`、`--cameras` 选择视角和相机。
- `--sheets` 只替换 `sheet.jpg`，已渲染的视频和图像保留。

**导出 LeRobot**：把预览目录中渲染过机器人相机图像的轨迹写成数据集，本例 5 条约 7.5 分钟：

```bash
uv run python scripts/scenes/to_lerobot.py outputs/scenes/000000/gen/part*/trajectories.hdf5 --output outputs/scenes/000000/lerobot
```

`task` 默认是每步所属子任务的英文指令，`--task instruction` 改用高层指令 `tidy up the table`。读取时用 `video_backend='pyav'`：LeRobot 默认的 torchcodec 需要系统里有 FFmpeg 4–7 的共享库。

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset
dataset = LeRobotDataset('table-1000/000000', root='outputs/scenes/000000/lerobot', video_backend='pyav')
```

## 验收

**初态**：先看预览目录的 `initial.json`，再看拼图。

| 内容 | 本例 | 要求 |
| --- | --- | --- |
| `invalid` | 0 | 没有无法初始化的 seed |
| `mean_attempts` | 1.0 | 越接近 1 越好；偏大说明摆放区域太挤或静置不稳 |
| `seeds.<seed>.attempts[].settle` | 静置时笔最多移动 0.4 mm、转动 0.2°，柜子不动，机器人无接触 | 移动不超过 3 mm、2° |
| `initial_states.jpg` | 见本章开头 | 摆放合理，变化足够（位置、朝向都在变） |

**生成**：看汇总 `summary.json`，不设硬性门槛，用于判断专家是否可用、比较不同版本：

| 字段 | 本例（seed 0–99） | 看什么 |
| --- | --- | --- |
| `success_rate` | 0.98 | 不要求 100%，失败的轨迹不进数据集 |
| `failures` | 2 条，都在 `close the drawer` | 按子任务和原因分组，集中出现的原因值得修 |
| `with_unexpected_contacts` | 0 | 机器人除手指外碰到物体的轨迹数，应为 0 |
| `mean_duration_s` | 27.7 s | 轨迹时长 |
| `mean_wall_s` | 63 s（最长 152 s） | 每条墙钟耗时，其中规划约 32 s |

单条轨迹的细节在 `episodes/<seed>.json`：每次尝试的初态、子任务起止、每次规划、手指碰过的部件、换用的候选、物体终态。

**评审**：

1. 每条成功轨迹的 `sheet.jpg`（12 帧，标注子任务）由大模型逐条查看：流程完整，物体到位（笔在抽屉里），抽屉最后关好、手已撤开。本例 98 条全部通过。
2. 随机 5 条的视频人工查看：手、臂与物体无可见穿模，夹持稳定，物体无异常滑移或跳动。
3. LeRobot 数据集能读回，帧数与轨迹步数一致。

不通过时：摆放问题改 `scene.py` 的 `initialize`，专家问题改 `expert.py`，然后重新检查初态、生成和评审。同一 seed 单独重跑时动作相同，但规划有时限，负载不同时路径可能不同，复现以录制的轨迹文件为准。
