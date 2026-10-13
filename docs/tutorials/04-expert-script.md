# 04 编写专家轨迹脚本

以 scene-000000 的“开抽屉、放笔、关抽屉”为例，写一段对每个随机初始摆放都适用的固定脚本：**任务分解 → 抓取与放置坐标 → 规划与执行 → 单次运行 → 批量验收**。

**输入：** 第三章的场景和机器人资产。

**输出：** 每次运行的 `result.json`、`trace.csv`、`contacts.json`，批量的 `summary.json/csv`；可选视频见第五章。

## 1. 任务分解

[drawer_pen.py](../../table_1000/tasks/drawer_pen.py) 的 `Solver.solve()` 参照 ManiSkill3 motionplanning solutions 的写法，一个阶段一行：

| 阶段 | 动作 |
| --- | --- |
| open | 预抓把手 → 沿接近方向靠近 → 合爪 → 分三段沿抽屉轴拉开 0.12 m → 松爪后撤 |
| pick | 手先竖直升高、离开抽屉侧壁 → 在笔质心处俯抓 → 合爪 → 竖直抬起 |
| place | 直线移到抽屉开口上方 → 下降到松手位 → 张爪让笔落下 → 撤手 |
| close | 再抓把手 → 分三段推回 → 松爪后撤 |

每一步的目标位姿都由**当下**的物体真实位姿和资产自身的碰撞几何算出，写成 `T_world_tcp = T_world_frame · T_frame_tcp`，见 [drawer_pen_geometry.py](../../table_1000/tasks/drawer_pen_geometry.py)。没有按 seed 或世界坐标写的特例。

- **把手 frame**：原点取把手碰撞体中心，轴取抽屉刚体的轴（−Y 为拉出方向）。手指沿把手长度方向夹住两端，接近方向向下倾 30°，指尖停在抽屉面板前 6 mm。
- **笔抓取**：竖直向下，开合方向垂直笔轴，抓在笔的质心处（运行时从物理引擎读取）。抓在质心外时，笔在两指间转动，规划用的持笔几何随之失准。
- **松手位**：取抽屉内腔露出部分的中心、侧壁上方 12 mm。松手位的手部姿态由合爪后实测的手-笔相对位姿反算，使笔轴沿抽屉宽度方向。
- **拉与推**：沿实时抽屉轴移动 TCP。每段后检查抽屉是否跟上（偏差 1.5 cm 以内）。没跟上就松手、重读把手位姿、重新抓取，全程最多 2 次。

## 2. 规划与执行

[motion.py](../../table_1000/tasks/motion.py) 是 MPlib 与 Isaac Sim 之间的薄层：

- **空程**：`move_to_pose_with_RRTConnect` 用 IK 加 RRTConnect，接触动作用 `move_to_pose_with_screw`（[闭环直线螺旋路径](../../table_1000/planning/screw.py)）。两者都用 TOPP 按 4 ms 采样时间参数化。
- **规划世界**：包括桌子、抽屉盒各刚体（把手单独成一个对象）、笔和机器人全身（另一臂冻结在初始姿态）。物体几何取自资产的 `model.json`，每次规划前按物理引擎中的实时位姿更新。
- **接触许可**：只在相应阶段给手指与把手、手指与笔开放；拉推抽屉时，把抽屉作为被手带动的物体挂在 TCP 上一起规划。
- **余量**：机器人凸块（手指除外）加厚 2 mm；空程规划时桌子和抽屉盒再加厚 3 mm。RRTConnect 的路径另按 0.01 rad 关节步长逐点复查，有碰撞就重规划。
- **执行**：轨迹逐个物理步下发关节位置和速度目标，到终点后等待关节误差小于 0.01 rad。机器人连杆不受重力，相当于理想重力补偿；手中的笔仍受重力。
- **失败传播**：IK、碰撞、关节限位、TOPP、跟踪任一失败都抛出带阶段名和原因的异常。只在明确的几种情况下换用较短动作再试：把手后撤距离、开抽屉后的升高高度、笔抬起高度、撤手方向。

MPlib 0.2.1 摆放挂接物体时不计 articulation 的基座位姿，所以规划在机器人 base_link 坐标系中进行，接口对外仍用世界坐标。

## 3. 单次运行

```bash
"$ISAAC_PYTHON" scripts/tasks/drawer_pen.py --offsets 0 0 0 0 --output outputs/drawer-pen/center --gpu 0
"$ISAAC_PYTHON" scripts/tasks/drawer_pen.py --seed 3 --output outputs/drawer-pen/seed-3 --gpu 0
```

`--offsets CX CY PX PY` 直接给出两件物体的偏移，`--seed` 按场景的 `placement()` 抽样。运行在受监督的子进程中：结果落盘后写 `shutdown.json`；`app.close()` 超过期限时，父进程依次发 TERM、KILL，真实退出码写入 `parent-lifecycle.json`。

`result.json` 主要字段：

| 字段 | 内容 |
| --- | --- |
| `status` | PASS 需同时满足：流程无失败、抽屉合上（开度 ≤ 5 mm）、笔在抽屉内腔、手离把手 5 cm 以上、无意外接触 |
| `layout`、`initial_poses` | 本次偏移、布局检查结果、两件物体的初始真实位姿 |
| `failure` | 失败阶段、原因与规划细节（如碰撞对、IK 状态） |
| `stages`、`plans` | 各阶段起始时刻；每次规划的类型、耗时、时长、最大跟踪误差 |
| `evaluation` | 抽屉开度、笔在抽屉坐标系中的位置、手到把手距离、意外接触数 |
| `timings` | 启动、加载、规划、物理步进、渲染、编码各项耗时 |

`contacts.json` 按时间段记录机器人每个连杆与桌子、抽屉盒部件、笔、地面以及自身的实际接触（PhysX 接触点间距小于 0.5 mm）。预期接触只有三类：手指与把手、手指与笔、轮子与地面；其余都计为意外接触。`trace.csv` 每 0.04 s 记录一行：阶段、关节实际值与目标、手指间距、抽屉开度、笔位姿。

## 4. 批量验收

```bash
"$ISAAC_PYTHON" scripts/tasks/drawer_pen_batch.py corners --output outputs/drawer-pen/corners --gpus 0 1 --jobs 8
"$ISAAC_PYTHON" scripts/tasks/drawer_pen_batch.py seeds --first 0 --count 40 --output outputs/drawer-pen/seeds --gpus 0 1 --jobs 8
```

- **corners**：两个区域各取中心、四个边中点和四个角，两两组合共 81 例，两件物体独立变化。
- **seeds**：连续 seed 的独立随机摆放。

每例是一次独立运行，已有结果的例子不重跑。全部例子（含失败）汇总到 `summary.json` 和 `summary.csv`，列出 seed、偏移、结果、失败阶段与原因、重抓次数、意外接触数和耗时。验收批次运行期间不改代码和场景。

本例结果见任务报告。
