# physics_test.py

带盖笔与三层抽屉柜使用本规范的 Python 测试入口；其他资产的测试按需补充。

资产目录中的 `physics_test.py` 验证当前资产，允许加载该资产的多个实例。不同资产的组合测试不属于本阶段验收。测试脚本随 Git 保存，调用 `table_1000.physics.testing` 的公共工具及官方仿真 API；不再定义测试 JSON 语言。

验收关注两项：视频及选定曲线是否符合预期，以及在明确条件下测得的实时因子 RTF。自动执行完成不代表物理表现已经通过人工验收。

## 统一入口

脚本导出 `TESTS: dict[str, Callable[[TestContext], None]]`。键为测试名称，也是输出名称；每个函数完成一个实验。脚本导入时不启动仿真。

```python
from table_1000.physics.testing import TestContext


def pull(ctx: TestContext):
    pen = ctx.asset
    body = pen.body("body")
    cap = pen.body("cap")

    ctx.camera(position=(0.3, -0.5, 0.2), target=(0, 0, 0))
    ctx.observe_body(cap, fields=("position", "linear_velocity"))
    ctx.plots("pull.force.x", "asset.cap.position.x",
              "asset.cap.linear_velocity.x")

    with ctx.fixed(body):
        ctx.force("pull", cap, frame="world", keyframes=[
            {"time": 0, "force": (0, 0, 0)},
            {"time": 1, "force": (-3, 0, 0)},
            {"time": 1.2, "force": (0, 0, 0)},
        ])
        ctx.run(2)


TESTS = {"pull": pull}
```

示例参数仅演示接口，不规定资产的受力阈值。执行器接收资产目录、可选测试名及输出目录；默认读取同目录脚本，依次运行全部测试。入口负责启动仿真、加载资产及官方行为组件、独立复位、采样和结果写入。测试函数不自行启动或关闭 Isaac。

执行器通过共享 Isaac Lab `start_app()` 启动隔离用户配置的应用，物理上下文使用 `SimulationContext`。`--physics cpu|gpu` 选择 CPU 或 CUDA PhysX，默认 `cpu`；`--gpu` 选择物理显卡编号，CUDA 逻辑设备为 `cuda:0`。CPU/GPU 必须分别输出结果，不能把渲染显卡编号当作物理后端。

`--num-envs N` 在同一场景中并行运行 N 份相同的实验，默认 CPU 为 1、GPU 为 4。各环境接收相同的动作、夹持轨迹和限值，夹持按各自环境的状态反馈；记录、曲线和视频取第一个环境，测试脚本无需修改。接触临界或限力切换等敏感过程中，各环境可能分化成不同的结果，`result.json` 的 `environments` 记录其他环境与第一个环境的终态最大位置差。GPU 上第一个环境的结果也可能随环境数略有不同，比较时保持相同的环境数。

```bash
uv run python scripts/assets/physics_test.py assets/objects/pen/000002 --script asset_sources/objects/pen/000002/physics_test.py --physics gpu --gpu 0 --output outputs/physics/pen-gpu
```

每个测试开始时，仅加载默认实例 `ctx.asset`，名称为 `asset`，恢复资产保存的初态，时间为零。不同测试以及计时重复之间不残留上一次的实例、约束或行为状态。

## 实例与公共工具

`ctx.asset_path` 为被测资产入口；`ctx.spawn(ctx.asset_path, name="pen_b", position=..., rotation=...)` 加载另一实例。返回的实例提供 `body(name)`、`joint(name)`，名称在该实例内解析；记录使用实例名前缀。实例随上下文统一释放，不手工导入行为。

公共工具覆盖下列反复使用的操作；特殊实验可直接访问官方 USD/Isaac 对象，不为其扩展新的配置语言。

| 操作 | 约定 |
| --- | --- |
| 初态设置 | 资产整体位姿、关节位置，以及指定刚体的世界位姿/速度；仅初始化时设置 |
| `ctx.fixed(body, position=None, rotation=None)` | 把刚体刚性锁在一个静止的世界位姿（默认为声明时位姿），不能移动；退出作用域时释放 |
| `ctx.grip(body, name=None)` | 理想的六轴夹持，用于让物体按轨迹运动、引导或测力，见下文 |
| 关节锁定与驱动 | 初始化前声明结构；运行时通过张量设置限位或驱动参数 |
| `ctx.force()`、`ctx.torque()` | 注册具名关键帧动作；同时记录实际施加的世界力/力矩及作用点 |
| `ctx.run(duration)`、`ctx.step()` | 推进指定时长或一个物理步；统一执行动作、行为、采样及到期渲染 |
| `ctx.observe_body()`、关节观测 | 记录指定状态，单位由物理量确定 |
| `ctx.observe()` | 注册名称到取值函数的自定义观测；自定义量需注明单位，不能按名称猜测单位 |
| `ctx.plots(*columns)` | 选择已登记的标量记录列，每列一个面板，共享时间轴 |
| `ctx.camera()`、地面工具 | 设置测试相机及地面；地面参数、夹具和相机不写回资产 |

所有资产、夹具、接触报告属性及物理结构须在首次 `run/step/ensure_initialized` 前声明。运行期间只写 PhysX 张量状态、目标、参数与外力，不能增删关节或写 USD 物理属性。相机与灯光等视觉结构不受此限制。

### 夹持

`ctx.grip(body)` 是一个理想的六轴夹持：通过关节直接连接被夹刚体，没有几何、不参与碰撞、不会打滑。它在初始化前声明，初始时以声明位姿夹住刚体；被夹刚体是 articulation 根时（如抽屉柜外壳），整棵 articulation 随之运动，其余关节保持自由。

```python
def gravity_tilt(ctx):
    grip = ctx.grip(ctx.asset.body("cabinet.housing"))
    ctx.run(1)
    grip.move_to(rotation=(cos(a/2), sin(a/2), 0, 0), duration=15)
    ctx.run(2)
```

| 方法 | 约定 |
| --- | --- |
| `move_to(*, duration, position=None, rotation=None, wait=True)` | 从上一段运动的终点出发，平移走直线、旋转绕固定轴，按五次多项式随时间平滑，起止速度和加速度为零 |
| `follow(path, *, duration, wait=True)` | 跟踪 `path(t) -> (position, quaternion)`，`t ∈ [0, duration]`；`path(0)` 必须等于上一段运动的终点 |
| `track(position, quaternion, linear_velocity=0, angular_velocity=0)` | 从下一步起保持给定位姿与速度，清空排队的运动；用于需要每步反馈的实验 |
| `limit(force=inf, torque=inf)` | 夹持施加的合力（N）与合力矩（N·m）的大小上限，可为数值或时间的函数；上限起作用时夹持以该大小推向目标，物体落后于轨迹 |
| `axes(linear=(1, 1, 1), angular=True)` | 逐个开关世界平移轴及转动；关闭的轴不施力 |
| `release()` / `grasp()` | 松开，刚体完全自由；在刚体当前位姿处静止地重新夹住 |
| `position` / `quaternion` | 上一段运动的终点，即下一段运动的起点 |

- 位姿、速度、力与力矩都在世界坐标系中，位置是刚体原点，旋转为四元数 `(w, x, y, z)`。
- `duration` 必须给出，且为物理步长的整数倍。阻塞式调用推进仿真直到运动结束，期间动作、行为与采样照常执行。多段运动按顺序衔接，可以在两段之间读取实时状态计算下一段。`wait=False` 只排队并返回运动对象，用 `ctx.wait(*motions)` 让多个夹持同时运动。
- 增益由被夹 articulation 树的质量与惯量自动确定，并补偿其重力与轨迹加速度；物体以连续的真实速度跟随轨迹，静止夹持时不因重力下垂。所用增益写入结果的 `grips`。
- 每步记录 `<name>.target.position.*`、`<name>.force.*`、`<name>.torque.*`（夹持施加给被夹树的力与对夹持点的力矩）及 `<name>.error.position` / `<name>.error.angle`。力在 CPU 与 GPU 上都可用于测量；GPU 上的力矩为估算值，约偏低 5%。
- 位移控制测力（如以恒定速度拔帽并取力的峰值）用 `follow`；力控制（如一只逐渐用力的手）用 `limit` 配合远处的目标。上限针对合力：夹持保持的横向约束力也计入其中，只关心某一方向的推力时，用 `axes` 只保留该方向（如沿笔轴合帽）。已知外载荷作用下的实验仍用 `ctx.force()`。


实例句柄、测试函数和时间规则是统一接口；其余辅助方法随实际实验提取，避免为未使用的能力预建框架。

### 初态与时间

Python 接口统一使用 SI 单位、弧度和 wxyz 四元数。默认重力为 `[0, 0, -9.81]`；地面显式创建，摩擦和恢复系数记录在报告中。

先恢复保存姿态和关节位置，再应用资产整体位姿，最后覆盖单独刚体的世界位姿。整体角速度按最终质心相对资产原点的位置计算各刚体线速度 `v + ω × r`，最后应用单体速度覆盖。有关节连接的初态不得违反连接及限位。

初态位置表示资产原点的位置，不是距地面的最低点高度。测试开始后不通过写入姿态模拟受力运动。

动作时间从本次实验初始化完成后计，单位为秒。关键帧首尾定义生效区间 `[首帧时间, 末帧时间)`，区间外停止施力。每个动作至少两帧，时间递增并对齐物理步；省略字段继承上一帧，数组整体替换。力、位置线性插值，旋转使用最短路径 SLERP。

力与力矩可选 `world` 或当前 `body` 坐标，作用点单独指定 `point_frame`；省略作用点时作用于当前质心。多个力/力矩相加。夹具释放保留当时位姿和速度，不复位刚体；临时约束不修改源资产。

## 视频、曲线与结果

按物理步记录状态，统一使用仿真时间；动作列表示该步实际施加值，不冒充接触力或夹具反力。状态采样标明积分前/后时刻，视频与同一时间戳的姿态一致。

相机参数包括位置、目标、up、分辨率和 fps。每帧间隔包含整数个物理步，首尾帧均采集，视频按正常速度播放。轨迹重渲染须标为回放，成本不混入在线仿真耗时。

每个测试只生成一张 `plots.jpg`，按选择顺序上下排列；标签使用列名，单位由观测定义确定，长度可显示为 m 或 mm，CSV 保持 SI。不选择曲线时不出图。已有 CSV 可以独立重画，不重新仿真。

默认写入资产目录下的 `physics_test/`，与 `preview/` 并列；可覆盖输出目录。测试 `pull` 的结果为：

```text
physics_test/
  pull.mp4
  pull/
    trace.csv
    plots.jpg
    result.json
```

重跑仅清理对应测试的结果。报告包含测试名、资产与软件版本、生效物理/渲染条件、实例数、自动检查及性能数据。`execution_status` 为 `completed` 或 `failed`；`review_status` 初始为 `pending`，人工确认后为 `accepted` 或 `needs_changes`。脚本可使用普通断言检查明确要求，不需要另建断言语言。

测试把测得的关键物理量写入字典 `ctx.experiment_details`（SI 单位，键名带单位后缀，如 `peak_pull_N`），结果原样保存在 `result.json` 的 `experiment`。验收完成后，将这些量汇总到资产 README 的物理实测表，见 [README 与缩略图](README.md#readme-与缩略图)。

人工查看完整视频与关注曲线，判断是否符合实验预期和物理常识。观察到的限位、接触误差、振荡及步长敏感性如实记录；数值稳定不等同于真实材料参数已标定。

## RTF 基准

`RTF = simulated_seconds / wall_seconds`，无量纲，越大越快。RTF=1 表示与真实时间等速。本阶段规定测量条件，不设置所有资产通用的最低 RTF。

| 条件 | 标准基准 |
| --- | --- |
| 引擎 | Isaac Sim 5.1 / PhysX；记录实际完整版本 |
| 物理步长 | `0.004 s`（250 Hz），每次推进一个物理步，无隐藏子步 |
| 求解器 | CPU PhysX、TGS，位置/速度迭代 16/4；CCD 开启 |
| 环境及实例 | CPU 1 个环境，GPU 4 个并行环境；默认 1 个资产实例，多实例另列实际数量，不与单实例直接比较 |
| CPU | 参考 AMD EPYC 7542；Kit/PhysX 工作线程目标 4，OMP/BLAS 目标 1；记录生效线程数与 CPU 型号 |
| GPU | 单张 RTX 4090；CPU 参考使用其渲染，GPU 对照使用 CUDA 刚体求解；分别记录后端、设备、驱动及其他负载 |
| 视频 | RTX Real-Time 2.0（`RealTimePathTracing`），960 × 480、25 FPS，即每 10 个物理步一帧；关闭运动模糊，1 spp，CPU/GPU 每次采集执行 2 次不推进物理的 Isaac Lab 渲染，总反弹与镜面/透射反弹上限均为 12 |
| 灯光 | 统一使用 `DomeLight` 强度 900 和 `DistantLight` 强度 2500；太阳光 `RotateXYZ = (-40, 20, 0)` 度 |

接触裕量、材料、阻尼和相机采用当前测试与资产的实际值并记录；灯光使用上述统一配置。同一实验的比较保持这些条件一致。保存实际渲染模式及采样/降噪设置，不能只报告“RTX”。指定条件无法满足时报告偏差，不标为同条件基准。

GPU 对照使用场景最小位置/速度迭代 64/16，TGS 开启逐次施加外力；GPU PhysX 不支持 CCD，因此关闭并披露。笔的 GPU 跌落实验单独使用 PGS，关闭不受 PGS 支持的逐次施加外力：TGS 在钢珠的 maximal-coordinate 关节撞击时出现异常能量增长。该选择保持资产材料、关节参数、4 ms 步长和验收阈值不变，报告记录实际求解器；它不属于与 CPU 同求解设置的性能比较。

必须分别报告：

1. **无渲染 RTF**：包含物理推进、资产行为、测试动作、状态读取与内存采样；关闭相机采集和渲染，不以单独 `World.step` 时间冒充完整测试成本。先在独立场景执行一次完整实验（至少 1 s）作为预热，销毁后用全新场景测量。每轮累计至少 10 s 仿真时间，测量 3 轮并报告各轮与中位数。短实验以独立复位重复累计，复位耗时单列且不计入积分区间，不把实验尾部任意延长为静止计时。
2. **含视频端到端 RTF**：对正式实验从首帧采集开始到最后视频/CSV 写入完成计时，包含动作、行为、采样、渲染和编码；同时报告仿真时长和墙钟时长。采用同一测试输入独立运行，核对物理结果的一致性。
多环境时，RTF 按单个环境的仿真时间计算，另报告所有环境合计的 `environment_seconds_per_second`（RTF × 环境数）。两种 RTF 的测量区间内同时采样资源占用：本进程 CPU 占用（核数）、整机 CPU 利用率、所用显卡的 GPU 利用率和显存。GPU 数值为整卡数据，包含同卡其他进程；报告同时记录该卡上的计算进程数（含本进程）。

3. **准备成本**：进程启动、资产加载/初始化、首次渲染预热及复位单列，不混入稳态无渲染 RTF；端到端测量前的预热必须披露。

计时采用单调墙钟时钟。更细步长或不同硬件结果作为独立对照，不外推标准 RTF；不能从带渲染耗时中简单减去渲染耗时得到无渲染结果。

参考：[官方独立 Python 工作流](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/introduction/workflows.html)、[Isaac Lab 刚体示例](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/01_assets/run_rigid_object.html)。
