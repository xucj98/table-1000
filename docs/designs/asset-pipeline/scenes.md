# 场景与专家轨迹规范

规定一个场景交付什么：场景源码、随机初始化、专家程序、轨迹数据，以及验收方式。以场景 [scenes/000000](../../../asset_sources/scenes/000000/)（三抽屉柜与笔）为例，命令见[第四章](../../tutorials/04-scene.md)，各文件的位置见[资产布局](README.md#资产布局)。

## 定位

- 每个场景只有一条高层指令：`tidy up the table`（整理好桌面）。专家按实际操作把轨迹分成子任务，每个子任务有一条英文指令（如 `open the bottom drawer of the cabinet`），数据同时提供高层与子任务两级指令。
- 每个场景有自己的专家程序，由大模型按本规范编写（code as policy）。场景和专家都不按任务类型命名，也不写成功判据（谓词）；轨迹好不好由人工抽查录像。自动化只做硬性检查和量化指标。
- 尽量使用现成框架：环境、状态读写、录制与重放用 Isaac Lab，运动规划求解器与抓取几何沿用 ManiSkill3 的写法。本仓库只补这些框架没有的部分：资产几何查询、初态检查、子任务记录。
- 生产分三步：构建场景（采样并检查初态，存下静置后的状态）、生成轨迹（从存下的初态运行专家）、渲染录像（按录制的状态）。

## 场景源码 `scene.py`

普通 Python 模块，不另设配置格式：

- **世界系**：原点在桌面中心正下方的地面，+Z 向上，单位 m、rad。
- `TABLE`：尺寸、高度、板厚、腿尺寸、摩擦。
- `ROBOT`：机器人资产（`robots/<name>`，描述取自其 `robot.py`）、底盘位置与偏航、起始关节（未列出的为 0）。格式上可以有多个机器人，环境目前只支持一个。
- `OBJECTS`：`{名字: {'asset': 'objects/<类别>/<编号>'}}`。尺寸、碰撞几何、关节都从资产读取，场景里不手写。场景原样引用资产的构建产物，不缩放、不覆盖质量、材料或驱动。
- `VIEWS`：录像用的固定视角，渲染时与机器人相机一样按名字选用。初态图片默认用机器人的主相机（`robot.py` 中第一个 RGB 相机），不另设视角。
- `initialize(layout)`：随机初始化，见下节。

资产按结构接入 Isaac Lab：带 articulation root 的资产（抽屉柜）是一个 articulation；刚体之间有关节但没有 articulation root 的资产（笔帽靠钢珠卡在笔身上）整体加载，每个刚体各是一个 rigid object。这样 Isaac Lab 的状态读写和录制覆盖每个刚体。物体刚体的名字取自资产的 `table1000:name`，特权状态里写作 `<物体>:<刚体>`（如 `cabinet:cabinet.drawer3`）。

## 随机初始化

`initialize(layout)` 用 `layout.rng` 抽样，写出各物体的位姿（资产坐标系）和关节值。同一 seed 必须得到同一结果，不能用其他随机源。接口分三层，场景按需选用：

| 层 | 接口 |
| --- | --- |
| 状态 | `set_pose(物体, 位姿, on=支撑)`、`set_joint(物体, 关节, 值)`：直接写入，无约束 |
| 几何查询 | `surface_height(on)`（支撑面高度）；`resting_pose(物体, xy, yaw, on)`（离给定朝向最近的稳定放置朝向与高度，由碰撞凸包算出）；`penetration(物体, 位姿)`（与桌子和已放物体的穿透深度）；`geometry[物体]`（部件包围盒、关节轴等） |
| 便捷组合 | `place(物体, region, yaw, on)`：在区域内均匀采样原点位置（偏航可给范围），放到稳定朝向，与已放物体穿透就重采 |

框架在最后统一检查（`table_1000/scenes/initial.py`）：

1. **无穿透**：已放物体之间、物体与桌面之间按碰撞几何检查，接触允许，穿透不超过 1 mm。
2. **静置稳定**：复位后机器人保持不动、静置 1 s。每件物体的每个刚体相对其支撑（桌子即世界）平移不超过 3 mm、转动不超过 2°，包括非 articulation 的笔帽和钢珠；物体关节变化不超过 3 mm（或 rad），机器人不碰任何东西。比较位置而不是速度：笔帽钢珠有微米级抖动，笔永远不休眠，但不会移动。

不通过就换下一个子 seed（`default_rng([seed, attempt])`）重新运行 `initialize`，最多 20 次，并记录每次的原因。`default_rng` 对整个列表做哈希，子 seed `[s, k]` 与其他 seed 的序列互不重叠。

通过的初态存进 `initial_states.hdf5`：每个有效 seed 一组，名称为 seed，属性 `seed`、`attempt`，内容为静置后的场景状态（格式同轨迹的 `initial_state`）；文件属性记录场景、机器人、物理后端和提交。专家轨迹从这些状态开始，不再重新采样；同一 seed 的摆放由代码确定，静置后的状态可能有微小差异，所以初态以文件为准。

例：笔的资产原始朝向是一个棱顶朝下，直接平放会滚动 7.5° 到相邻的面上，静置检查因此不通过；`resting_pose` 改用最近的稳定朝向后，静置移动降到 0.4 mm、0.2°。

## 专家程序 `expert.py`

定义 `solve(ctx)`：阻塞式代码，每个动作执行完才返回。`ctx` 提供：

| 层 | 接口 |
| --- | --- |
| 运动原语（与物体无关） | `ctx.solver(arm)` 的 `move_to_pose_with_RRTConnect`、`move_to_pose_with_screw`、`open_gripper`、`close_gripper`、`first_reachable`（依次尝试候选目标）；规划世界 `add_table`、`add_object`（由碰撞几何生成，可按部件分组）、`allow`（允许手指接触某部件）、`attach`/`detach`（手中物体）、`carry`/`release`（随手移动的关节部件，如被拉的抽屉）；`ctx.hold`、`ctx.step` |
| 状态与几何查询 | `ctx.pose(物体, 刚体)`、`ctx.joint(物体, 关节)`、`ctx.center_of_mass`、`ctx.part_box`（部件在刚体坐标系中的包围盒）、`ctx.geometry[物体]`（部件包围盒、关节轴）、`ctx.tcp()` |
| 抓取几何 | `table_1000.experts.grasps`：`build_grasp_pose`、`compute_grasp_info_by_obb`（ManiSkill 的同名函数）、`standoff`、`translated` |
| 原子技能 | `table_1000.experts.skills`（见下文）：`grasp`、`pick`、`move_by_displacement`、`push`、`release`、`place`、`make_room`、`ready`、`retry` |
| 子任务 | `with ctx.subtask('put the pen into the drawer'):` 标记这段步数属于哪条英文指令 |
| 随机 | `ctx.rng`：需要变化的选择（抓取位置、顺序）用它，不用其他随机源 |

要求：

- 每个目标位姿由物体**当下**的实际位姿和资产几何算出，不按 seed 或固定世界坐标写特例。
- 手指只在需要的动作里允许接触相应部件；接触结束后撤销。例：先允许碰把手再用 RRTConnect 去把手前方，路径可能穿过把手，把抽屉推回去后抓空；应只在最后的直线接近时允许。
- 可以重试，但要写明候选（后撤距离、抬起高度；物理上等价的抓法与放法，如对称的手腕转向、绕圆柱轴侧倾、物体放置的两个方向）或次数上限（重新抓取），用 `ctx.note` 记下换用的候选。其余失败用 `ctx.fail(原因)` 或规划异常结束，记录子任务与原因。
- TCP 约定：+X 为接近方向，+Y 为手指张开方向。

规划用 MPlib（ManiSkill 也用它）：自由空间用 RRTConnect 到 IK 解，规划路径按 1 cm 关节步长复查碰撞，不过就重新规划；接触动作用直线 screw 路径。规划好的路径每个控制步（25 Hz）发送一个关节位置。RRTConnect 规划时碰撞几何膨胀 1 cm，起点和终点处已有的接触只在本次规划中允许，路径再按未膨胀的几何复查。底盘移动后，规划世界随之更新坐标系。

### 原子技能

`table_1000.experts.skills` 在运动原语之上提供以物体为单位的技能，接口仿照 RoboTwin 2.0 的任务 API，但和 `solve` 一样阻塞：每个调用在仿真中执行完、检查结果后才返回，做不到时以 `ctx.fail`（原因如 `no reachable grasp`、`object not held`、`no placement`）或规划异常结束；什么都够不着（没有可行的抓取、推或放置）时异常类型为 `Unreachable`。技能按物体的实时位姿和资产几何自己选择手臂、抓取候选和路径，每次换用候选都用 `ctx.note` 记下。场景 [000002](../../../asset_sources/scenes/000002/expert.py) 只用这些技能写成。

| 技能 | 作用 |
| --- | --- |
| `grasp(ctx, 物体, 部件)` | 抓住物体或其一个部件（如把手）。可给接近方向 `approach`、与它的最大夹角 `max_angle`（给一组时依次放宽，够不着才用下一个）、抓住后的位移 `then`（候选须让这段位移也有 IK 解）。先试肩部离物体近的手臂；闭合后没夹住（手指间隙小于物体宽度）就退出重选，至多 `MAX_REGRASPS` 次；可达范围内没有候选时先移动底盘 |
| `pick(ctx, 物体)` | `grasp` 后抬起 `height`；物体没跟着抬起时松手并失败（`object not lifted`） |
| `move_by_displacement(ctx, 手臂, x, y, z)` | TCP 沿直线移动。`drag` 指定随手移动的关节部件（拉抽屉）；`least` 允许整段规划不通时只走一部分，返回走过的比例 |
| `push(ctx, 物体, 部件, direction, distance)` | 闭合指尖抵住部件的面推动（推抽屉关上） |
| `release(ctx, 手臂)` | 张开手指并沿接近方向退出 |
| `place(ctx, 物体, region, near, keep_out)` | 把手中物体放在表面上的矩形区域内：`region = (表面坐标系, 低角, 高角)`，表面为坐标系的 z = 0 平面；朝向不限，按离 `near` 的距离、再按转角排序采样；物体轮廓避开 `keep_out` 矩形。可达范围内没有可行位置时先移动底盘 |
| `make_room(ctx, 位姿)` | 移动底盘（相对起始位姿后退、转向）到能以关节余量够到给定 TCP 位姿的最小移动处；`path=True` 表示这些位姿是一条直线运动上的连续位姿。机器人没有底盘时不动 |
| `ready(ctx)` | 空着的手臂回到起始姿态 |
| `retry(ctx, 步骤)` | 步骤失败时先 `ready` 再重试，至多 `ATTEMPTS` 次；回合（由步骤组成的专家为当前步骤）的仿真时间超过 `RETRY_BUDGET`（90 s）后不再重试，按仿真时间计，与机器负载无关；`Unreachable` 不重试：规划已经尝试过所有候选，这类失败比例高说明规划参数、碰撞或上一步留下的状态有问题，应优化规划而不是重试 |

### 步骤

专家可以把子任务分成步骤：`Step(名字, run, check)`，`run(ctx)` 执行其中的子任务，`check(ctx)` 检查结束状态是否满足下一步的起始条件，不满足就 `ctx.fail`。步骤之间手里不拿东西。`solve` 写成 `run_steps(ctx, STEPS)`。

### 逐步运行

由步骤组成的专家，每一步通过 `check` 后保存一个检查点：完整场景状态（`InteractiveScene.get_state`，格式同 `initial_state`：物体位姿与关节，机器人的底盘、躯干、手臂和夹爪），以及该 seed 回合开始时机器人的关节和底盘位姿。一批的检查点按步骤存为 `checkpoints/step<k>.hdf5`，每个 seed 一组（属性 `seed`、`attempt`、`at` 即这一步结束时的控制步；`state`、`start`），文件属性记录场景、机器人、后端、提交、步骤序号和名字。

`run_expert.py <场景> --steps k[-m]` 只运行第 k 到 m 步：k 为 1 时从初态开始；k 大于 1 时须给 `--from <批次目录>`，从那一批第 k − 1 步的检查点开始，先静置 0.2 s。有检查点的 seed 都会运行，输出格式与完整批次相同（默认目录 `<后端>-steps<k>-<m>`），并写出新的检查点。恢复后，空闲手臂回到的姿态和底盘的起始位姿仍取该 seed 回合开始时的，而不是检查点处的。从第 k 步开始时 `ctx.rng` 为 `default_rng([seed, attempt, 1, k])`，MPlib 的随机数也随 k 改变。

`summary.json` 的 `steps` 给出每步的起始 seed 数、完成数、成功率（完成 / 起始）和逐步完成率（progressive completion rate，ManiSkill-HAB 的统计方式：这一步及之前各步都完成的 seed 占第 1 步起始 seed 的比例）；从检查点开始的批次沿用来源批次前几步的行，分母也取来源批次的。

`join_steps.py <批次> ... --output <目录>` 把逐步运行的各批按顺序接成完整轨迹：每段在下一批开始用的检查点处截断，接上下一批同一 seed 的轨迹，`subtasks` 和 `steps` 平移到接好后的位置，`parts` 记录每段的批次、提交、步骤、起点和长度。输出目录的格式与一批轨迹相同，可直接渲染。逐步迭代的方法见[操作手册 05](../../tutorials/05-step-by-step-expert.md)。

## 轨迹数据

**无图片、可重放**：直接使用 Isaac Lab 录制器的 HDF5（`trajectories.hdf5` 存成功的，`trajectories_failed.hdf5` 存失败的）。每条轨迹 `data/demo_<seed>`：

| 内容 | 说明 |
| --- | --- |
| `initial_state` | 静置后的场景状态（`InteractiveScene.get_state`：每个 articulation 的根位姿、速度、关节位置与速度，每个 rigid object 的位姿与速度） |
| `actions` `[T, A]` | 每个控制步的动作；各动作组的区间见文件的 `env_args.action_layout` |
| `states` | 每步之后的场景状态 |
| `obs/*` | 每步之前的机器人观测（`qpos`、`qvel`、`base_pose`、`tcp_<arm>`） |
| 属性 | `scene`、`seed`、`attempt`、`instruction`、`subtasks`（每段的指令与起止步）、`status`、`failure`；由步骤组成的专家另有 `steps`（完成的步骤及其结束的控制步）和 `from`（开始用的检查点文件，从初态开始时为空） |

`env.reset_to(initial_state)` 后依次执行动作可在物理中重放；`scene.reset_to(states[t])` 精确恢复第 t 步，用于渲染。每条轨迹开始前用 seed 设置 MPlib 的随机数（`mplib.pymp.set_global_seed`），专家的 `ctx.rng` 为 `default_rng([seed, attempt, 1])`；RRTConnect 有规划时限，负载不同时路径仍可能不同。

一批轨迹的进程各自录制，结束时按 seed 顺序合并成一个 `trajectories.hdf5` 和一个 `trajectories_failed.hdf5`。每条轨迹另有报告 `episodes/<seed>.json`：子任务起止、每次规划、手指碰过的部件、换用的候选、失败原因、物体终态与耗时。

**渲染**：`render.py` 按状态逐步恢复并渲染指定的相机（机器人的 RGB 相机或场景的固定视角），不推进物理。每一步：`scene.reset_to(states[t])`；调用 PhysX 的 `update_transformations` 把位姿写进 USD（不推进物理时渲染器读不到新位姿）；渲染一次。RT2 在时间上降噪，相邻控制步只差 40 ms，混入的上一帧与收敛画面的平均像素差不到 2（0–255）；每条的第 0 帧接在另一条轨迹之后，多渲染几次。T 步的轨迹录像为 T 帧，第 t 帧是第 t 步动作之前的状态，与 `obs[t]`、`actions[t]` 对应。录像与轨迹放在同一目录：成功的在 `videos/`，失败的在 `fails/`，每条轨迹一个目录，每个相机一条（`<seed>/<相机>.mp4`），或合成一条（`<seed>/merge.mp4`）；标注可选。每次只写本次渲染的录像，已有的其他录像保留；每个文件写完整后才放到最终位置，中断不会留下半成品。生成新批次时整个目录被替换，旧的渲染随之删除。

带图片的数据集（如 LeRobot）可以由 `trajectories.hdf5` 逐步恢复状态、渲染机器人相机得到，目前不在生产流程中。

### 复用边界

环境与状态恢复使用 Isaac Lab 的 `InteractiveScene.get_state/reset_to`，静置检查复用 `RobotEnv.privileged()` 已从 Isaac Lab 刚体和关节数据取得的全量位姿。录制使用官方 `RecorderManager`、动作和状态 recorder、`HDF5DatasetFileHandler`；项目只补 agent 观测组与任务属性。

重放按 HDF5 行读取当前状态：官方 `HDF5DatasetFileHandler.load_episode` 会一次加载整条轨迹，渲染只需要当前状态。初始化的候选位置检查发生在写入仿真之前，并需与 MPlib 的规划碰撞几何一致，复用已有 FCL 与资产几何适配；稳定放置复用 trimesh。

抽屉滑动除有限次重抓外，另设每次调用的总动作次数上限，覆盖剩余行程小于滑脱容差的停滞；到达目标的最后一次动作仍可正常返回，超限用 `ctx.fail` 结束该轨迹，继续后续 seed。

## 验收

**自动化硬性检查**：初态无穿透、静置稳定（见上）。

**量化指标**（不设硬性门槛）：

- 初态（`initial.json`）：无法初始化的 seed、平均尝试次数、静置时的最大移动；
- 轨迹（批次的 `summary.json`）：成功率（分母为全部 seed），失败的 seed 按“子任务: 原因”列出，原因来自专家的检查（`ctx.fail`）、规划失败或专家代码出错，进程中途退出时没有结果的 seed 记为 `-: process exited`；什么都够不着（`Unreachable`）的 seed 另列在 `unreachable`，及其比例 `unreachable_rate`；成功轨迹的平均时长、每条墙钟耗时；
- 每条轨迹的报告：规划次数、换用的候选、手指碰过的部件。手指以外的接触不统计：密集摆放时手臂碰到物体是正常的。

**评审**：

- 初态拼图（`build_scene.py --grid`，默认机器人主相机），人工查看摆放是否合理、多样；最小有效 seed 的图缩小后作为场景缩略图提交；
- 前 5 条成功轨迹录制视频（机器人三个相机与 `wide` 视角合成一条，带标注），人工查看：流程完整，物体到位，手、臂与物体无可见穿模，夹持稳定，物体无异常滑移或跳动。失败的轨迹也录制视频（标注失败原因），用来判断是否值得修改专家。

批量运行从固定提交的独立检出进行，运行期间不改代码、场景和资产；`initial.json` 与 `summary.json` 记下所用的提交和 seed。三个脚本都分成 `--jobs` 个进程并行，本机平均分到 `--gpus` 列出的显卡上，或经 `--launcher` 交给作业调度器：seed 分成连续的段，要渲染的轨迹按帧数分配，每个进程一份；某个进程失败不影响其他进程，其中没有结果的 seed 按 `--retries` 重跑。不渲染时物理在 CPU 上，每个进程只占约 0.5 GB 显存，可以在一张卡上并发多个。轨迹的后端用 `--physics` 指定，命名与物理测试一致（`isaac-cpu`；场景生成尚未在 GPU 上验证）；初态固定用 `isaac-cpu` 生成。

## 已知限制

- 只在 CPU PhysX、单环境上运行；GPU 与多环境见 [issue #11](https://github.com/xucj98/table-1000/issues/11)。
- 初始化的几何检查按资产静止构型（关节为 0）；用 `set_joint` 打开的部件不参与采样时的穿透检查，由复位后的静置检查兜底。
- 支撑面只有桌面和物体碰撞包围盒的顶面；碗内底、抽屉内底等支撑面以后按需加入几何查询。
- 笔的帽与钢珠一直有微米级抖动，不休眠（不影响任务）。
- `make_room` 和技能内部的底盘选位只考虑相对起始位姿后退（0–0.2 m）和转向（±20°），不侧移或前进。场景 000002 因此把柜子的横向范围限制在 ±0.22 m。
- 摆放由 seed 完全确定，静置后的状态以 `initial_states.hdf5` 为准。专家轨迹：MPlib 的随机数按 seed 设置，但 RRTConnect 有规划时限，负载不同时路径仍可能不同。复现轨迹以录制文件为准。
