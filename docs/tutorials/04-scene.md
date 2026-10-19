# 04 场景与专家轨迹

用第 2 章的物体和第 3 章的机器人搭一个场景，编写专家程序，批量生成专家轨迹，渲染后评审，再导出 LeRobot 数据集。以[场景 000000](../../asset_sources/scenes/000000/) 为例：X1Pro 站在桌前，桌上一个三抽屉柜和一支笔；机器人打开最下层抽屉，把笔放进去，再关上。各文件的要求见[场景与专家轨迹规范](../designs/asset-pipeline/scenes.md)。

![场景 000000：seed 0 的初态，机器人头部相机](../../asset_sources/scenes/000000/thumbnail.jpg)

## 交付物

源码目录 `asset_sources/scenes/<编号>/`，由 AI 按规范编写，提交到 Git：

| 文件 | 作用 |
| --- | --- |
| `scene.py` | 场景：桌子、机器人及起始姿态、物体、录像视角，以及每个 seed 怎样随机摆放（`initialize`） |
| `expert.py` | 专家程序 `solve(ctx)`：按英文子任务组织的阻塞式代码，用运动原语、几何查询和抓取几何完成任务 |
| `thumbnail.jpg` | seed 0 初态的头部相机图像，作为场景的缩略图，由 `check_scene.py` 生成 |

生成的结果不进 Git：验收时看 `assets/scenes/<编号>/` 这一层（`summary.json`、5 × 5 初态拼图、5 条录像），轨迹和 LeRobot 数据集在 `data/scenes/<编号>/`。每个文件见[资产布局](../designs/asset-pipeline/README.md#资产布局)。

轨迹文件是 Isaac Lab 录制器的格式，可在物理中重放，也可逐步恢复状态；带图片的数据只来自渲染后的 LeRobot 数据集。

**本例场景**：抽屉柜放在桌子右侧，位置 ±5 cm、偏航 ±15°；笔在机器人右手前方，位置 ±5 cm、朝向任意。专家分四个子任务：`open the bottom drawer of the cabinet`、`pick up the pen`、`put the pen into the drawer`、`close the drawer`。笔朝向任意，所以抓笔和放笔各有几种物理上等价的做法，按顺序尝试，见 [expert.py](../../asset_sources/scenes/000000/expert.py)。

## 命令

**检查初态**：对 seed 0–24 做与生成时相同的初始化（无穿透检查、静置 1 s），用机器人头部相机拍成 5 × 5 拼图，并把 seed 0 的图缩小写成 `thumbnail.jpg`，约 2 分钟：

```bash
uv run python scripts/scenes/check_scene.py 000000 --gpu 0
```

**生成轨迹**（省略的参数有默认值）：

```bash
uv run python scripts/scenes/generate.py 000000 --seeds 0:100 --physics isaac-cpu --gpus 0 1 2 3
```

- `--physics` 必填，列出物理后端，与第 2 章的命名一致；场景生成目前只验证了 `isaac-cpu`。
- seed 按 `--gpus` 分成连续的几段，每段一个进程，每张卡同时跑一段；某一段失败不影响其他段。本例 100 个 seed 在 4 张卡上约 30 分钟。
- 可以同时给多个场景。每次运行会替换 `data/scenes/<编号>/<后端>/` 中原有的结果，并删除 `assets/scenes/<编号>/` 中旧批次的录像和 `demos/`；`--output` 可以换轨迹的根目录。
- 批量运行用固定提交的独立检出，运行期间不改代码、场景和资产；`summary.json` 记下所用的提交。

**渲染**：按录制的状态逐步恢复场景再渲染，不重新仿真。先为所有成功轨迹渲染评审图，再随机选 5 条渲染录像和机器人相机图像。录像是 `assets/scenes/<编号>/demo_<seed>.mp4`，`wide` 与 `close` 视角左右并排：

```bash
uv run python scripts/scenes/render.py data/scenes/000000/isaac-cpu/part*/trajectories.hdf5 --sheets --gpus 0 1 2 3
uv run python scripts/scenes/render.py data/scenes/000000/isaac-cpu/part*/trajectories.hdf5 --count 5 --gpus 0 1 2 3
```

- 选中的轨迹分到 `--gpus` 列出的卡上，每张卡同时一个进程。评审图每条约 1 s，视频和相机图像每条约 2.5 分钟，每个进程另有约 1 分钟启动。
- `--count` 随机选择，`--seed` 固定选法；`--demos demo_27 demo_51` 指定轨迹；`--views`、`--cameras` 选择视角和相机。
- `--sheets` 只替换 `sheet.jpg`，保留该轨迹已有的录像和相机图像。重新生成轨迹时旧批次的渲染会被删除。
- `--output DIR` 代替场景目录，布局相同，摘要记录实际路径；导出时对应使用 `--frames DIR`。重新生成轨迹后摘要标为 `needs_render`，重新渲染后统计输入轨迹中已完成的产品。

**导出 LeRobot**：把 `demos/` 中渲染过机器人相机图像的轨迹写成数据集，本例 5 条约 7.5 分钟：

```bash
uv run python scripts/scenes/to_lerobot.py data/scenes/000000/isaac-cpu/part*/trajectories.hdf5 --output data/scenes/000000/lerobot
```

导出前会检查相机集合和帧编号是否完整，渲染中断或缺帧时会报错，请先不带 `--sheets` 重新渲染相应轨迹，再导出。

`task` 默认是每步所属子任务的英文指令，`--task instruction` 改用高层指令 `tidy up the table`。读取时用 `video_backend='pyav'`：LeRobot 默认的 torchcodec 需要系统里有 FFmpeg 4–7 的共享库。

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset
dataset = LeRobotDataset('table-1000/000000', root='data/scenes/000000/lerobot', video_backend='pyav')
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

表中数值是修复前 bbc5e0e 对应批次的历史结果，不能代替修改后逐刚体静置检查的验收；新批次以实际生成的摘要为准。静置阈值现在应用到每个刚体，包括笔帽和钢珠。

单条轨迹的细节在 `episodes/<seed>.json`：每次尝试的初态、子任务起止、每次规划、手指碰过的部件、换用的候选、物体终态。

然后看图片和视频：

1. `initial_states.jpg`：摆放合理，变化足够（位置、朝向都在变）。
2. 每条成功轨迹的 `demos/demo_<seed>/sheet.jpg`（12 帧，标注子任务）由大模型逐条查看：流程完整，物体到位（笔在抽屉里），抽屉最后关好、手已撤开。
3. 5 条录像 `demo_<seed>.mp4` 人工查看：手、臂与物体无可见穿模，夹持稳定，物体无异常滑移或跳动。
4. LeRobot 数据集能读回，帧数与轨迹步数一致。

不通过时：摆放问题改 `scene.py` 的 `initialize`，专家问题改 `expert.py`，然后重新检查初态、生成和评审。同一 seed 单独重跑时动作相同，但规划有时限，负载不同时路径可能不同，复现以录制的轨迹文件为准。
