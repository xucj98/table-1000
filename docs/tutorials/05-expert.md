# 05 编写专家轨迹脚本

为 scene-000000 编写专家程序并批量生成轨迹：机器人打开抽屉柜最下层抽屉，把笔放进去，再关上。专家是每个场景自己的 Python 代码（code as policy，可由大模型编写），用框架提供的运动原语、几何查询和子任务标记；规范见[场景与专家轨迹规范](../designs/scenes.md#专家程序-expertpy)。

**输入：** 第四章的场景。

**输出：** `asset_sources/scenes/scene-000000/expert.py`；轨迹文件 `trajectories.hdf5`（成功）、`trajectories_failed.hdf5`（失败），每个 seed 的报告与汇总。

## 1. 专家程序

[expert.py](../../asset_sources/scenes/scene-000000/expert.py) 定义 `solve(ctx)`，按四个英文子任务组织：

```python
def solve(ctx):
    expert = Expert(ctx)
    with ctx.subtask('open the bottom drawer of the cabinet'):
        ...
    with ctx.subtask('pick up the pen'):
        ...
    with ctx.subtask('put the pen into the drawer'):
        ...
    with ctx.subtask('close the drawer'):
        ...
```

用到的几层接口：

| 层 | 本例用法 |
| --- | --- |
| 规划世界 | `add_table()`；`add_object('cabinet', ...)`：每个刚体一个规划物体，把手单独分出（手指可以碰）；`add_object('pen', ...)`：整支笔合成一个凸体 |
| 运动原语 | 到把手前方用 `move_to_pose_with_RRTConnect`，贴近、拉动、后撤用 `move_to_pose_with_screw`；`open_gripper`、`close_gripper` |
| 接触许可 | 抓把手前 `allow(['cabinet:cabinet.drawer3.handle'])`，松开后撤销；拉抽屉时 `carry` 把抽屉当作随手移动的物体；笔夹起后 `attach` |
| 几何查询 | 把手、抽屉前板、内腔、底板的包围盒（`ctx.geometry['cabinet'].bounds(部件, 刚体)`），笔的质心（`ctx.center_of_mass`），抽屉开度（`ctx.joint('cabinet', 'cabinet_drawer3_slide')`） |
| 抓取几何 | `build_grasp_pose(接近方向, 闭合方向, 中心)`：把手从前方斜向下 30° 接近，两指夹住把手两端；笔从上方接近，在质心处横夹 |

所有目标都由物体当下的位姿和资产几何算出，不按 seed 写特例。重试只有列明的几种：

- 后撤距离与抬起高度按候选依次尝试（`first_reachable`）；
- 拉抽屉分 3 段，每段检查抽屉是否跟上，跟不上就松手重抓，最多 2 次。

失败用 `ctx.fail(原因)` 结束，记录子任务。

## 2. 生成轨迹

```bash
"$ISAAC_PYTHON" scripts/scenes/generate.py scene-000000 --seeds 0:100 --output outputs/scenes/scene-000000/gen --gpu 0
```

每个 seed 依次：

1. 按第四章生成初态并静置；
2. 开始录制；
3. 运行 `solve`；
4. 把轨迹写进 HDF5，成功和失败分开存放。

一个进程连续跑多个 seed，每个从复位开始。需要更快时，按 seed 区间在多个进程或 GPU 上分别运行，各自输出到不同目录，再汇总：

```bash
"$ISAAC_PYTHON" scripts/scenes/summarize.py outputs/scenes/scene-000000/gen/part* --output outputs/scenes/scene-000000/gen/summary.json
```

本例结果：在提交 8f4f0ff 上，seed 0–99 分 4 个进程并行生成。

- 初态全部第一次通过。
- 成功 97 条，平均 27.4 s（684 步）。各子任务平均用时：开抽屉 9.5 s，拿笔 4.7 s，放笔 4.8 s，关抽屉 8.4 s。
- 没有意外接触；手指只碰过把手和笔，只有一条失败轨迹碰到了抽屉前板。
- 每条墙钟约 54 s：规划 22 s（每条 3 次 RRTConnect，每次约 6.8 s；另有 16 次直线规划），执行 24 s，初态 1.8 s。
- 失败 3 条，都在 `close the drawer`：笔放进抽屉后一端翘起 2–4°；之前抽屉已被推进一些，关抽屉时被卡住。其中两次专家松手重抓后继续推，把整个柜子推动了 10–15 cm。

之后专家加了一项检查：柜子移动超过 1 cm 就停止。用改后的代码复查 seed 30、35、90：

- 35、90 成功；
- 30 在柜子移动 2.6 cm 时停止。

两条对照（seed 0、36）仍成功，步数相差 1–2 步，见第 3 节。

| 输出 | 内容 |
| --- | --- |
| `trajectories.hdf5` | Isaac Lab 录制器的格式，每条 `data/demo_<seed>`：`initial_state`、`actions`、`states`、`obs/*`；属性有场景、seed、指令、子任务起止步。可用 `env.reset_to(initial_state)` 加动作在物理中重放，或用 `states` 逐步恢复 |
| `trajectories_failed.hdf5` | 失败的轨迹，格式相同，属性里有失败的子任务与原因 |
| `episodes/<seed>.json` | 初态各次尝试、子任务、失败、每次规划（类型、耗时、结果）、机器人接触、物体终态（位置与关节）、耗时 |
| `summary.json`、`summary.csv` | 成功率、失败按子任务与原因分布、平均初态尝试次数、有意外接触的轨迹数、平均时长与墙钟耗时 |

报告里的机器人接触分两类，都只作为指标记录，不作为失败条件：

- `fingers`：手指碰到的物体部件，用来核对是否只碰了该碰的部件。
- `unexpected`：除此之外、也不是两指之间的接触。

第一版专家在自由空间移动到把手之前就允许了手指碰把手。RRTConnect 的路径因此可以穿过把手，把抽屉推进去约 3 cm，随后抓空。现在只在最后的直线接近时允许。

## 3. 可重复性

初态由 seed 完全确定。专家的规划用 MPlib，每条轨迹开始前用 seed 设置它的随机数（`mplib.pymp.set_global_seed`）：

- 实测同一 seed（0、30）两次运行的动作逐位相同。
- 设置之前，同一 seed 重跑的步数会差 1–2 步。

RRTConnect 有规划时限，机器负载不同时仍可能走出不同的路径。所以复现轨迹以录制的文件为准。
