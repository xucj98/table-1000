# 04 场景与专家轨迹

用第 2 章的物体和第 3 章的机器人搭一个场景，编写专家程序，批量生成专家轨迹，渲染后评审，再导出 LeRobot 数据集。以[场景 000000](../../asset_sources/scenes/000000/) 为例：X1Pro 站在桌前，桌上一个三抽屉柜和一支笔；机器人打开最下层抽屉，把笔放进去，再关上。各文件的要求见[场景与专家轨迹规范](../designs/asset-pipeline/scenes.md)。

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
| `assets/scenes/<编号>/summary.json` | 验收入口：初态检查、各后端的生成统计、已渲染的轨迹 |
| `assets/scenes/<编号>/preview/` | 初态检查 `initial.json` 与拼图 `initial_states.jpg`；每条渲染过的轨迹一个 `demo_<seed>/`：评审图、视频、机器人相机图像 |
| `outputs/scenes/<编号>/<后端>/` | 轨迹：每段 `part<k>/` 有 `trajectories.hdf5`（成功）、`trajectories_failed.hdf5`（失败）、每个 seed 的报告 `episodes/<seed>.json` 和日志 `run.log`；汇总 `summary.json`、`summary.csv` |
| LeRobot 目录（`--output`） | LeRobot v3.0 数据集 |

轨迹文件是 Isaac Lab 录制器的格式，可在物理中重放，也可逐步恢复状态；带图片的数据只来自渲染后的 LeRobot 数据集。

**本例场景**：抽屉柜放在桌子右侧，位置 ±5 cm、偏航 ±15°；笔在机器人右手前方，位置 ±5 cm、朝向任意。专家分四个子任务：`open the bottom drawer of the cabinet`、`pick up the pen`、`put the pen into the drawer`、`close the drawer`。笔朝向任意，所以抓笔和放笔各有几种物理上等价的做法，按顺序尝试，见 [expert.py](../../asset_sources/scenes/000000/expert.py)。

## 命令

**检查初态**：对 seed 0–24 做与生成时相同的初始化（无穿透检查、静置 1 s），约 2 分钟：

```bash
uv run python scripts/scenes/check_scene.py 000000 --gpu 0
```

**生成轨迹**（省略的参数有默认值）：

```bash
uv run python scripts/scenes/generate.py 000000 --seeds 0:100 --physics isaac-cpu --gpus 0 1 2 3
```

- `--physics` 必填，列出物理后端，与第 2 章的命名一致；场景生成目前只验证了 `isaac-cpu`。
- seed 按 `--gpus` 分成连续的几段，每段一个进程，每张卡同时跑一段；某一段失败不影响其他段。本例 100 个 seed 在 4 张卡上约 30 分钟。
- 可以同时给多个场景。每次运行会替换 `outputs/scenes/<编号>/<后端>/` 中原有的结果；`--output` 可以换根目录。
- 批量运行用固定提交的独立检出，运行期间不改代码、场景和资产；`summary.json` 记下所用的提交。

**渲染**：按录制的状态逐步恢复场景再渲染，不重新仿真。先为所有成功轨迹渲染评审图，再随机选 5 条渲染视频和机器人相机图像：

```bash
uv run python scripts/scenes/render.py outputs/scenes/000000/isaac-cpu/part*/trajectories.hdf5 --sheets --gpus 0 1 2 3
uv run python scripts/scenes/render.py outputs/scenes/000000/isaac-cpu/part*/trajectories.hdf5 --count 5 --gpus 0 1 2 3
```

- 选中的轨迹分到 `--gpus` 列出的卡上，每张卡同时一个进程。评审图每条约 1 s，视频和相机图像每条约 2.5 分钟，每个进程另有约 1 分钟启动。
- `--count` 随机选择，`--seed` 固定选法；`--demos demo_27 demo_51` 指定轨迹；`--views`、`--cameras` 选择视角和相机。
- `--sheets` 只替换同一轨迹的 `sheet.jpg`，通过 `render.json` 指纹匹配的视频和图像保留；换批次或旧预览没有来源记录时，清理该 demo 的旧产品。
- `--output DIR` 指定自定义预览目录，摘要记录实际路径；导出时对应使用 `--frames DIR`。重新生成轨迹后摘要标为 `needs_render`，重新渲染后只统计与输入轨迹匹配的产品。

**导出 LeRobot**：把预览目录中渲染过机器人相机图像的轨迹写成数据集，本例 5 条约 7.5 分钟：

```bash
uv run python scripts/scenes/to_lerobot.py outputs/scenes/000000/isaac-cpu/part*/trajectories.hdf5 --output outputs/scenes/000000/lerobot
```

导出前会校验轨迹指纹、相机集合和完整帧编号。旧预览缺少 `render.json`、与轨迹不匹配或缺帧时会报错，请先不带 `--sheets` 重新渲染相应轨迹，再导出。

`task` 默认是每步所属子任务的英文指令，`--task instruction` 改用高层指令 `tidy up the table`。读取时用 `video_backend='pyav'`：LeRobot 默认的 torchcodec 需要系统里有 FFmpeg 4–7 的共享库。

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset
dataset = LeRobotDataset('table-1000/000000', root='outputs/scenes/000000/lerobot', video_backend='pyav')
```

## 验收

生成失败的段或渲染失败的进程会让脚本以非零退出码结束。先看 `assets/scenes/<编号>/summary.json`，不设硬性门槛，用于判断场景和专家是否可用、比较不同版本：

| 字段 | 本例 | 看什么 |
| --- | --- | --- |
| `initial.invalid`、`without_state` | 0 | 没有无法初始化的 seed |
| `initial.mean_attempts` | 1.0 | 越接近 1 越好；偏大说明摆放区域太挤或静置不稳 |
| `initial.settle` | 笔最多移动 0.4 mm、转动 0.2°，柜子不动 | 每件物体移动不超过 3 mm、2° |
| `initial.robot_contacts` | 0 | 静置时机器人不碰任何东西 |
| `generation.<后端>.commit` | 生成所用的提交 | 不带 `+dirty` |
| `generation.<后端>.success_rate` | 0.98 | 不要求 100%，失败的轨迹不进数据集 |
| `generation.<后端>.failures` | 2 条都在关抽屉：没握住把手、推动了柜子 | 按子任务和原因分组，集中出现的原因值得修 |
| `generation.<后端>.with_unexpected_contacts` | 0 | 机器人除手指外碰到物体的轨迹数，应为 0 |
| `generation.<后端>.missing_seeds`、`failed_parts` | 空 | 每个 seed 都有结果 |
| `generation.<后端>.mean_duration_s`、`mean_wall_s` | 27.7 s、64 s | 轨迹时长与每条墙钟耗时 |
| `preview` | 98 条评审图，5 条视频 | 评审图覆盖所有成功轨迹 |

表中数值是修复前 bbc5e0e 对应批次的历史结果，不能代替修改后逐刚体静置检查和来源校验的验收；新批次以实际生成的摘要为准。静置阈值现在应用到每个刚体，包括笔帽和钢珠。

单条轨迹的细节在 `episodes/<seed>.json`：每次尝试的初态、子任务起止、每次规划、手指碰过的部件、换用的候选、物体终态。

然后看图片和视频：

1. `preview/initial_states.jpg`：摆放合理，变化足够（位置、朝向都在变），见本章开头。
2. 每条成功轨迹的 `preview/demo_<seed>/sheet.jpg`（12 帧，标注子任务）由大模型逐条查看：流程完整，物体到位（笔在抽屉里），抽屉最后关好、手已撤开。
3. 有视频的 5 条人工查看 `wide.mp4`、`close.mp4`：手、臂与物体无可见穿模，夹持稳定，物体无异常滑移或跳动。
4. LeRobot 数据集能读回，帧数与轨迹步数一致。

不通过时：摆放问题改 `scene.py` 的 `initialize`，专家问题改 `expert.py`，然后重新检查初态、生成和评审。同一 seed 单独重跑时动作相同，但规划有时限，负载不同时路径可能不同，复现以录制的轨迹文件为准。
