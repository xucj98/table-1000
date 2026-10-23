# 场景与专家轨迹规范

规定一个场景交付什么：场景源码、随机初始化、终态、专家程序、轨迹数据，以及验收方式。以场景 [scenes/000000](../../../asset_sources/scenes/000000/)（三抽屉柜与笔）、[scenes/000100](../../../asset_sources/scenes/000100/)（38 件物体的办公桌与十个终态）、[scenes/000101](../../../asset_sources/scenes/000101/)（62 件物体的电子维修工作台，含垃圾桶与桌面杂物）和 [scenes/000102](../../../asset_sources/scenes/000102/)（30 件物体的梳妆台）为例，命令见[第四章](../../tutorials/04-scene.md)，各文件的位置见[资产布局](README.md#资产布局)。

## 定位

- 每个场景只有一条高层指令：`tidy up the table`（整理好桌面）。专家按实际操作把轨迹分成子任务，每个子任务有一条英文指令（如 `open the bottom drawer of the cabinet`），数据同时提供高层与子任务两级指令。
- 每个场景有自己的专家程序，由大模型按本规范编写（code as policy）；场景也可以先只提供初态和终态，专家程序以后补（如 000100）。场景和专家都不按任务类型命名，也不写成功判据（谓词）；轨迹好不好由人工抽查录像。自动化只做硬性检查和量化指标。
- 尽量使用现成框架：环境、状态读写、录制与重放用 Isaac Lab，运动规划求解器与抓取几何沿用 ManiSkill3 的写法。本仓库只补这些框架没有的部分：资产几何查询、初态检查、子任务记录。
- 生产分三步：构建场景（采样并检查初态，存下静置后的状态）、生成轨迹（从存下的初态运行专家）、渲染录像（按录制的状态）。

## 场景源码 `scene.py`

普通 Python 模块，不另设配置格式：

- **世界系**：原点在桌面中心正下方的地面，+Z 向上，单位 m、rad。
- `TABLE`：尺寸、高度、板厚、腿尺寸、摩擦。
- `ROBOT`：机器人资产（`robots/<name>`，描述取自其 `robot.py`）、底盘位置与偏航、起始关节（未列出的为 0）。格式上可以有多个机器人，环境目前只支持一个。
- `OBJECTS`：`{名字: {'asset': 'objects/<类别>/<编号>'}}`。尺寸、碰撞几何、关节都从资产读取，场景里不手写。场景原样引用资产的构建产物，不缩放、不覆盖质量、材料或驱动。
- `VIEWS`：录像用的固定视角，渲染时与机器人相机一样按名字选用。
- `IMAGE_VIEW`（可选）：构建初态、终态图片默认用的视角，取 `VIEWS` 中的名字；不设时用机器人的主相机（`robot.py` 中第一个 RGB 相机）。
- `initialize(layout)`：随机初始化，见下节。
- `END_STATES`：可选，同一批物体摆好的若干终态，见[终态](#终态)。

资产按结构接入 Isaac Lab：带 articulation root 的资产（抽屉柜）是一个 articulation；刚体之间有关节但没有 articulation root 的资产（笔帽靠钢珠卡在笔身上）整体加载，每个刚体各是一个 rigid object。这样 Isaac Lab 的状态读写和录制覆盖每个刚体。物体刚体的名字取自资产的 `table1000:name`，特权状态里写作 `<物体>:<刚体>`（如 `cabinet:cabinet.drawer3`）。

## 随机初始化

`initialize(layout)` 用 `layout.rng` 抽样，写出各物体的位姿（资产坐标系）和关节值。同一 seed 必须得到同一结果，不能用其他随机源。接口分三层，场景按需选用：

| 层 | 接口 |
| --- | --- |
| 状态 | `set_pose(物体, 位姿, on=支撑)`、`set_joint(物体, 关节, 值)`：直接写入，无约束；支撑可以是桌子、物体或桌旁地面 `'floor'`（地上的物体不算掉下桌） |
| 几何查询 | `surface_height(on)`（支撑面高度）；`resting_pose(物体, xy, yaw, on)`（离给定朝向最近的稳定放置朝向与高度，由碰撞凸包和资产显式给出的质心算出）；`penetration(物体, 位姿)`（与桌子和已放物体的穿透深度）；`geometry[物体]`（部件包围盒、关节轴等） |
| 便捷组合 | `place(物体, region, yaw, on)`：在区域内均匀采样原点位置（偏航可给范围），放到稳定朝向，与已放物体穿透就重采；`drop(物体, xy, yaw, rotation, frame)`：从上方竖直落下，停在最先碰到的桌面或物体上（落进容器、抽屉，叠到本子上），返回位姿和支撑；`scatter(物体, region, yaw, rotation, frame, onto)`：在区域内采样落点，直到落在 `onto` 指定的支撑上且整件物体在桌面范围内；`row(物体列表, start, direction, yaw, gap, frame)`：沿一个方向把物体按碰撞外形依次排成一行，间隔 `gap` |

多个场景共用的摆放组合在 `table_1000/scenes/arrange.py`：`stand_in`（立放进笔筒等容器）、`into`（随机落进容器）、`line_up_in`（在容器里排成一行）、`drawer_contents`（放进抽屉柜的某个抽屉，再推到给定开度）、`strew`（撒在一片区域）等。`drawer_contents` 适用于按约定命名的抽屉柜：刚体 `<类别>.housing` 与 `<类别>.drawer<i>`（1 为最上层），抽屉部件 `front`、`side1`、`side2`、`bottom`，滑动关节 `<类别>.drawer<i>.slide` 向 −Y 拉出；抽屉内可放的范围由这些部件的几何算出（两侧板之间、拉出部分里上层拉手之前）。

区域、落点和朝向可以在某个已放物体的坐标系（`frame`，只取它的偏航）中给出，例如笔筒内的落点、抽屉拉出部分的范围。几何随 `set_joint` 设定的关节值变化：拉开的抽屉按拉开后的位置参与穿透和落点计算。

`drop` 与 `scatter` 放下的物体是**松放的**：刚接触支撑，未必已经放稳（斜靠在笔筒壁上的笔）。`layout.free_landing = True` 表示松放物体可以落到任何地方（凌乱桌面上笔从笔筒里倒出来也算合理）；默认要求它们留在原来的支撑上。

框架在最后统一检查（`table_1000/scenes/initial.py`）：

1. **无穿透**：已放物体之间、物体与桌面之间按碰撞几何检查，接触允许，穿透不超过 1 mm。
2. **落稳**（有松放物体时）：复位后静置 3 s。松放物体可以移动，但着地点（最低 2 mm 内碰撞点的中心）要留在托着它的物体上（沿支撑链找到第一个非松放的物体，如容器或桌面，在其碰撞包围盒的水平范围内，关节取摆放时的值，拉开的抽屉也算）；`free_landing` 时不限。其他物体按下一步的限值检查。任何物体都不能掉下桌面，机器人不碰任何东西。
3. **静置稳定**：机器人保持不动、再静置 1 s。每件物体的每个刚体相对其支撑（桌子即世界）平移不超过 3 mm、转动不超过 2°，包括非 articulation 的笔帽和钢珠；物体关节变化不超过 3 mm（或 rad），机器人不碰任何东西。比较位置而不是速度：笔帽钢珠有微米级抖动，笔永远不休眠，但不会移动。

不通过就换下一个子 seed（`default_rng([seed, attempt])`）重新运行 `initialize`，最多 20 次，并记录每次的原因。`default_rng` 对整个列表做哈希，子 seed `[s, k]` 与其他 seed 的序列互不重叠。

通过的初态存进 `initial_states.hdf5`：每个有效 seed 一组，名称为 seed，属性 `seed`、`attempt`，内容为静置后的场景状态（格式同轨迹的 `initial_state`）；文件属性记录场景、机器人、物理后端和提交。专家轨迹从这些状态开始，不再重新采样；同一 seed 的摆放由代码确定，静置后的状态可能有微小差异，所以初态以文件为准。

例：笔的资产原始朝向是一个棱顶朝下，直接平放会滚动 7.5° 到相邻的面上，静置检查因此不通过；`resting_pose` 改用最近的稳定朝向后，静置移动降到 0.4 mm、0.2°。

## 终态

`END_STATES`：`{名字: {'quality': 'good' | 'bad', 'description': 英文说明, 'arrange': 函数}}`。每个终态是同一批物体的一种摆法：`arrange(layout)` 与 `initialize` 用同一套接口写出位姿和关节，经过同样的检查（seed 为 0，不通过时同样换子 seed 重试）。`quality` 标明这是整理得好的还是差的方案，`description` 写明怎么摆，供人评与模型评价对照。

好的终态之间应在分组、收纳位置或排列方式上有明显区别；差的终态物理上可行，但整理质量差（堆成一堆、放错容器、只整理了一半）。终态不需要配套轨迹。

构建结果存进 `end_states.hdf5`：每个终态一组，名称为终态名，属性 `state`、`attempt`，内容格式同初态。

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
- 轨迹（批次的 `summary.json`）：成功率（分母为全部 seed），失败的 seed 按“子任务: 原因”列出，原因来自专家的检查（`ctx.fail`）、规划失败或专家代码出错，进程中途退出时没有结果的 seed 记为 `-: process exited`；成功轨迹的平均时长、每条墙钟耗时；
- 每条轨迹的报告：规划次数、换用的候选、手指碰过的部件。手指以外的接触不统计：密集摆放时手臂碰到物体是正常的。

**评审**：

- 初态拼图（`build_scene.py --grid`，默认机器人主相机），人工查看摆放是否合理、多样；最小有效 seed 的图缩小后作为场景缩略图提交；
- 前 5 条成功轨迹录制视频（机器人三个相机与 `wide` 视角合成一条，带标注），人工查看：流程完整，物体到位，手、臂与物体无可见穿模，夹持稳定，物体无异常滑移或跳动。失败的轨迹也录制视频（标注失败原因），用来判断是否值得修改专家。

批量运行从固定提交的独立检出进行，运行期间不改代码、场景和资产；`initial.json` 与 `summary.json` 记下所用的提交和 seed。三个脚本都按 `--gpus` 和 `--jobs-per-gpu` 分进程并行：seed（或要渲染的轨迹）分成 `显卡数 × jobs-per-gpu` 段，每段一个进程；某一段失败不影响其他段。不渲染时物理在 CPU 上，每个进程只占约 0.5 GB 显存，可以在一张卡上并发多个。轨迹的后端用 `--physics` 指定，命名与物理测试一致（`isaac-cpu`；场景生成尚未在 GPU 上验证）；初态固定用 `isaac-cpu` 生成。

## 已知限制

- 只在 CPU PhysX、单环境上运行；GPU 与多环境见 [issue #11](https://github.com/xucj98/table-1000/issues/11)。
- `surface_height`、`resting_pose`、`place` 的支撑面只有桌面和物体碰撞包围盒的顶面；放进容器、抽屉或叠放用 `drop` / `scatter`。
- `drop` 停在第一次接触的位置，物体未必放稳，由落稳检查交给物理决定最终位置；落稳后的状态以文件为准。
- 笔的帽与钢珠一直有微米级抖动，不休眠（不影响任务）。
- 摆放由 seed 完全确定，静置后的状态以 `initial_states.hdf5` 为准。专家轨迹：MPlib 的随机数按 seed 设置，但 RRTConnect 有规划时限，负载不同时路径仍可能不同。复现轨迹以录制文件为准。
