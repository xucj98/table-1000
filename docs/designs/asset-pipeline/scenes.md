# 场景与专家轨迹规范

规定一个场景交付什么：场景源码、随机初始化、专家程序、轨迹数据，以及验收方式。以场景 [scenes/000000](../../../asset_sources/scenes/000000/)（三抽屉柜与笔）为例，命令见[第四章](../../tutorials/04-scene.md)，各文件的位置见[资产布局](README.md#资产布局)。

## 定位

- 每个场景只有一条高层指令：`tidy up the table`（整理好桌面）。专家按实际操作把轨迹分成子任务，每个子任务有一条英文指令（如 `open the bottom drawer of the cabinet`），数据同时提供高层与子任务两级指令。
- 每个场景有自己的专家程序，由大模型按本规范编写（code as policy）。场景和专家都不按任务类型命名，也不写成功判据（谓词）；轨迹好不好由大模型逐条评审、人工抽查。自动化只做硬性检查和量化指标。
- 尽量使用现成框架：环境、状态读写、录制与重放用 Isaac Lab，运动规划求解器与抓取几何沿用 ManiSkill3 的写法，带图片的数据用 LeRobot。本仓库只补这些框架没有的部分：资产几何查询、初态检查、子任务记录。

## 场景源码 `scene.py`

普通 Python 模块，不另设配置格式：

- **世界系**：原点在桌面中心正下方的地面，+Z 向上，单位 m、rad。
- `TABLE`：尺寸、高度、板厚、腿尺寸、摩擦。
- `ROBOT`：机器人资产（`robots/<name>`，描述取自其 `robot.py`）、底盘位置与偏航、起始关节（未列出的为 0）。格式上可以有多个机器人，环境目前只支持一个。
- `OBJECTS`：`{名字: {'asset': 'objects/<类别>/<编号>'}}`。尺寸、碰撞几何、关节都从资产读取，场景里不手写。场景原样引用资产的构建产物，不缩放、不覆盖质量、材料或驱动。
- `VIEWS`：录像用的固定视角，录像中左右并排。初态拼图用机器人头部相机，不另设视角。
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

不通过就换下一个子 seed（`default_rng([seed, attempt])`）重新运行 `initialize`，最多 20 次，并记录每次的原因。轨迹从静置后的实际状态开始。

例：笔的资产原始朝向是一个棱顶朝下，直接平放会滚动 7.5° 到相邻的面上，静置检查因此不通过；`resting_pose` 改用最近的稳定朝向后，静置移动降到 0.4 mm、0.2°。

## 专家程序 `expert.py`

定义 `solve(ctx)`：阻塞式代码，每个动作执行完才返回。`ctx` 提供：

| 层 | 接口 |
| --- | --- |
| 运动原语（与物体无关） | `ctx.solver(arm)` 的 `move_to_pose_with_RRTConnect`、`move_to_pose_with_screw`、`open_gripper`、`close_gripper`、`first_reachable`（依次尝试候选目标）；规划世界 `add_table`、`add_object`（由碰撞几何生成，可按部件分组）、`allow`（允许手指接触某部件）、`attach`/`detach`（手中物体）、`carry`/`release`（随手移动的关节部件，如被拉的抽屉）；`ctx.hold`、`ctx.step` |
| 状态与几何查询 | `ctx.pose(物体, 刚体)`、`ctx.joint(物体, 关节)`、`ctx.center_of_mass`、`ctx.part_box`（部件在刚体坐标系中的包围盒）、`ctx.geometry[物体]`（部件包围盒、关节轴）、`ctx.tcp()` |
| 抓取几何 | `table_1000.experts.grasps`：`build_grasp_pose`、`compute_grasp_info_by_obb`（ManiSkill 的同名函数）、`standoff`、`translated` |
| 子任务 | `with ctx.subtask('put the pen into the drawer'):` 标记这段步数属于哪条英文指令 |
| 随机 | `ctx.rng`：需要变化的选择（抓取位置、顺序）用它，不用其他随机源 |

要求：

- 每个目标位姿由物体**当下**的实际位姿和资产几何算出，不按 seed 或固定世界坐标写特例。
- 手指只在需要的动作里允许接触相应部件；接触结束后撤销。例：先允许碰把手再用 RRTConnect 去把手前方，路径可能穿过把手，把抽屉推回去后抓空；应只在最后的直线接近时允许。
- 可以重试，但要写明候选（后撤距离、抬起高度；物理上等价的抓法与放法，如对称的手腕转向、绕圆柱轴侧倾、物体放置的两个方向）或次数上限（重新抓取），用 `ctx.note` 记下换用的候选。其余失败用 `ctx.fail(原因)` 或规划异常结束，记录子任务与原因。
- TCP 约定：+X 为接近方向，+Y 为手指张开方向。

规划用 MPlib（ManiSkill 也用它）：自由空间用 RRTConnect 到 IK 解，规划路径按 1 cm 关节步长复查碰撞，不过就重新规划；接触动作用直线 screw 路径。规划好的路径每个控制步（25 Hz）发送一个关节位置。

## 轨迹数据

**无图片、可重放**：直接使用 Isaac Lab 录制器的 HDF5（`trajectories.hdf5` 存成功的，`trajectories_failed.hdf5` 存失败的）。每条轨迹 `data/demo_<seed>`：

| 内容 | 说明 |
| --- | --- |
| `initial_state` | 静置后的场景状态（`InteractiveScene.get_state`：每个 articulation 的根位姿、速度、关节位置与速度，每个 rigid object 的位姿与速度） |
| `actions` `[T, A]` | 每个控制步的动作；各动作组的区间见文件的 `env_args.action_layout` |
| `states` | 每步之后的场景状态 |
| `obs/*` | 每步之前的机器人观测（`qpos`、`qvel`、`base_pose`、`tcp_<arm>`） |
| 属性 | `scene`、`seed`、`attempt`、`instruction`、`subtasks`（每段的指令与起止步）、`status`、`failure` |

`env.reset_to(initial_state)` 后依次执行动作可在物理中重放；`scene.reset_to(states[t])` 精确恢复第 t 步，用于渲染。生成时每条轨迹开始前用 seed 设置 MPlib 的随机数（`mplib.pymp.set_global_seed`），同一 seed 单独重跑时动作逐位相同；RRTConnect 有规划时限，负载不同时仍可能不同。

**带图片**：`render.py` 按状态逐步恢复并渲染机器人相机。每一步：`scene.reset_to(states[t])`；调用 PhysX 的 `update_transformations` 把位姿写进 USD（不推进物理时渲染器读不到新位姿）；渲染两次保留第二次（RT2 在时间上降噪，第一次会混入上一个状态）。机器人相机图像是每一步之前的观测，与该步动作对应。`to_lerobot.py` 写成 LeRobot v3.0 数据集：`observation.state`（关节位置）、`action`、`observation.images.<相机>`（视频）、`task`（子任务指令，或 `--task instruction` 用高层指令），25 fps。LeRobot 在 `data` 依赖组，与 Isaac Lab 装在同一环境。

### 渲染与导出

渲染属于生成它的那一批轨迹：生成新批次时，验收摘要的 `preview` 置为 `needs_render`，旧批次的渲染一并删除。每条渲染过的轨迹有一份 `render.json`，只列已完成的评审图、录像视角和相机；渲染开始前先失效本次要重建的产品，完成后原子更新，中断留下的半成品不会被导出。`--sheets` 只重建评审图，保留该轨迹已有的录像和相机图像。

LeRobot 导出在创建或替换数据集之前，检查每条有相机图像的轨迹的完成记录、相机集合和帧编号，渲染中断或缺帧会报错，要求不带 `--sheets` 重新渲染；只有评审图的轨迹跳过。多份输入包含相同 demo 名称时拒绝处理。

### 复用边界

环境与状态恢复继续使用 Isaac Lab 的 `InteractiveScene.get_state/reset_to`，静置检查复用 `RobotEnv.privileged()` 已从 Isaac Lab 刚体和关节数据取得的全量位姿。录制继续使用官方 `RecorderManager`、动作和状态 recorder、`HDF5DatasetFileHandler`；项目只补 agent 观测组与任务属性。渲染产物的位置与完成记录集中在一个模块，渲染与导出共享子任务解析。

重放保留按 HDF5 行读取的薄适配：官方 `HDF5DatasetFileHandler.load_episode` 会一次加载整条轨迹，本项目渲染只需要当前状态，避免为长轨迹增加常驻内存。物理场景查询可用 Isaac Sim/PhysX，但初始化的候选位置检查发生在写入仿真之前，并需与 MPlib 的规划碰撞几何一致，继续复用已有 FCL 与资产几何适配；稳定放置复用 trimesh。无需再引入另一套场景、状态或运动规划框架。

抽屉滑动除有限次重抓外，另设每次调用的总动作次数上限，覆盖剩余行程小于滑脱容差的停滞；到达目标的最后一次动作仍可正常返回，超限用 `ctx.fail` 结束该轨迹，继续后续 seed。

## 验收

**自动化硬性检查**：初态无穿透、静置稳定（见上）。

**量化指标**（不设硬性门槛，写入场景的 `summary.json`）：

- 初态平均尝试次数、无法初始化的 seed；
- 生成成功率，失败按子任务和原因分布；
- 每条轨迹的规划次数与换用的候选；机器人接触分两类记录：`fingers`（手指碰过的物体部件，核对是否只碰了该碰的）和 `unexpected`（除此之外、也不是两指之间的接触）及其最大穿透；
- 轨迹时长、墙钟耗时。

**评审**：

- seed 0–24 的初态用头部相机渲染成 5 × 5 拼图（`check_scene.py`，每格 384 × 216，相机原图 1280 × 720），人工查看摆放是否合理、多样；seed 0 的图缩小后作为场景缩略图提交；
- 每条轨迹由大模型看评审图（`render.py` 的 `sheet.jpg`，12 帧带子任务），判断流程是否完整、物体是否到位、有无异常；人工抽查；
- 随机 5 条轨迹录制视频（固定视角左右并排），人工查看：流程完整，手、臂与物体无可见穿模，夹持稳定，物体无异常滑移或跳动。

批量运行从固定提交的独立检出进行，运行期间不改代码、场景和资产；`summary.json` 记下所用的提交。`generate.py` 与 `render.py` 和物体物理测试一样按 `--gpus` 分进程并行：seed（或要渲染的轨迹）分成几段，每段一个进程，每张卡同时一个；某一段失败不影响其他段。后端用 `--physics` 指定，命名与物理测试一致（`isaac-cpu`；场景生成尚未在 GPU 上验证）。

## 已知限制

- 只在 CPU PhysX、单环境上运行；GPU 与多环境见 [issue #11](https://github.com/xucj98/table-1000/issues/11)。
- 初始化的几何检查按资产静止构型（关节为 0）；用 `set_joint` 打开的部件不参与采样时的穿透检查，由复位后的静置检查兜底。
- 支撑面只有桌面和物体碰撞包围盒的顶面；碗内底、抽屉内底等支撑面以后按需加入几何查询。
- 笔的帽与钢珠一直有微米级抖动，不休眠（不影响任务）。
- 初态由 seed 完全确定。专家轨迹：MPlib 的随机数按 seed 设置，实测同一 seed 重跑动作逐位相同；但 RRTConnect 有规划时限，负载不同时仍可能不同。复现轨迹以录制文件为准。
