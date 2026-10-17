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

```bash
"$ISAAC_PYTHON" scripts/assets/physics_test.py assets/objects/pen/000002 --script asset_sources/objects/pen/000002/physics_test.py --physics gpu --gpu 0 --output outputs/physics/pen-gpu
```

每个测试开始时，仅加载默认实例 `ctx.asset`，名称为 `asset`，恢复资产保存的初态，时间为零。不同测试以及计时重复之间不残留上一次的实例、约束或行为状态。

## 实例与公共工具

`ctx.asset_path` 为被测资产入口；`ctx.spawn(ctx.asset_path, name="pen_b", position=..., rotation=...)` 加载另一实例。返回的实例提供 `body(name)`、`joint(name)`，名称在该实例内解析；记录使用实例名前缀。实例随上下文统一释放，不手工导入行为。

公共工具覆盖下列反复使用的操作；特殊实验可直接访问官方 USD/Isaac 对象，不为其扩展新的配置语言。

| 操作 | 约定 |
| --- | --- |
| 初态设置 | 资产整体位姿、关节位置，以及指定刚体的世界位姿/速度；仅初始化时设置 |
| `ctx.fixed(body, ...)` | 初始化前声明六轴 articulation 夹具；作用域内将限位锁在目标坐标，退出时通过张量释放 |
| `ctx.clamp_root(body)` | 永久固定已有 articulation 的根，通过张量根位姿移动；不增加辅助质量或自由度，不支持运行时释放 |
| 弹性夹具 | 使用原生约束与驱动，提供目标位置/旋转、平移/旋转刚度和阻尼；允许偏移，不自行实现一套积分器 |
| 关节锁定与驱动 | 初始化前声明结构；运行时通过张量设置限位或驱动参数 |
| `ctx.force()`、`ctx.torque()` | 注册具名关键帧动作；同时记录实际施加的世界力/力矩及作用点 |
| `ctx.run(duration)`、`ctx.step()` | 推进指定时长或一个物理步；统一执行动作、行为、采样及到期渲染 |
| `ctx.observe_body()`、关节观测 | 记录指定状态，单位由物理量确定 |
| `ctx.observe()` | 注册名称到取值函数的自定义观测；自定义量需注明单位，不能按名称猜测单位 |
| `ctx.plots(*columns)` | 选择已登记的标量记录列，每列一个面板，共享时间轴 |
| `ctx.camera()`、地面工具 | 设置测试相机及地面；地面参数、夹具和相机不写回资产 |

所有资产、夹具、接触报告属性及物理结构须在首次 `run/step/ensure_initialized` 前声明。运行期间只写 PhysX 张量状态、目标、参数与外力，不能增删关节或写 USD 物理属性。相机与灯光等视觉结构不受此限制。

可释放夹具使用无碰撞的 XYZ 平移及 XYZ 旋转串联关节，被测刚体直接作为末端 link。六个辅助 link 各有 10 μg 质量、1e-11 kg·m² 惯量并禁用重力；释放后保留这部分惯性。旋转关节使用大范围有限限位以支持跨整圈目标。夹具继承资产求解迭代次数及原 articulation 自碰撞设置，报告披露辅助质量与驱动参数。它与旧 D6 夹具不保证轨迹相同；迁移必须对比 CPU 基线、报告物理偏差，不能自动放宽验收阈值。抽屉柜的永久固定与倾斜使用根夹具，保留原有三个抽屉自由度，避免辅助关节影响滑出角度。

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

人工查看完整视频与关注曲线，判断是否符合实验预期和物理常识。观察到的限位、接触误差、振荡及步长敏感性如实记录；数值稳定不等同于真实材料参数已标定。

## RTF 基准

`RTF = simulated_seconds / wall_seconds`，无量纲，越大越快。RTF=1 表示与真实时间等速。本阶段规定测量条件，不设置所有资产通用的最低 RTF。

| 条件 | 标准基准 |
| --- | --- |
| 引擎 | Isaac Sim 5.1 / PhysX；记录实际完整版本 |
| 物理步长 | `0.004 s`（250 Hz），每次推进一个物理步，无隐藏子步 |
| 求解器 | CPU PhysX、TGS，位置/速度迭代 16/4；CCD 开启 |
| 环境及实例 | 1 个环境、默认 1 个资产实例；多实例另列实际数量，不与单实例直接比较 |
| CPU | 参考 AMD EPYC 7542；Kit/PhysX 工作线程目标 4，OMP/BLAS 目标 1；记录生效线程数与 CPU 型号 |
| GPU | 单张 RTX 4090；CPU 参考使用其渲染，GPU 对照使用 CUDA 刚体求解；分别记录后端、设备、驱动及其他负载 |
| 视频 | RTX Real-Time 2.0（`RealTimePathTracing`），960 × 480、25 FPS，即每 10 个物理步一帧；关闭运动模糊，1 spp，CPU/GPU 每次采集执行 2 次不推进物理的 Isaac Lab 渲染，总反弹与镜面/透射反弹上限均为 12 |
| 灯光 | 统一使用 `DomeLight` 强度 900 和 `DistantLight` 强度 2500；太阳光 `RotateXYZ = (-40, 20, 0)` 度 |

接触裕量、材料、阻尼和相机采用当前测试与资产的实际值并记录；灯光使用上述统一配置。同一实验的比较保持这些条件一致。保存实际渲染模式及采样/降噪设置，不能只报告“RTX”。指定条件无法满足时报告偏差，不标为同条件基准。

GPU 对照使用场景最小位置/速度迭代 64/16，TGS 开启逐次施加外力；GPU PhysX 不支持 CCD，因此关闭并披露。笔的 GPU 跌落实验单独使用 PGS，关闭不受 PGS 支持的逐次施加外力：TGS 在钢珠的 maximal-coordinate 关节撞击时出现异常能量增长。该选择保持资产材料、关节参数、4 ms 步长和验收阈值不变，报告记录实际求解器；它不属于与 CPU 同求解设置的性能比较。

必须分别报告：

1. **无渲染 RTF**：包含物理推进、资产行为、测试动作、状态读取与内存采样；关闭相机采集和渲染，不以单独 `World.step` 时间冒充完整测试成本。先在独立场景执行一次完整实验（至少 1 s）作为预热，销毁后用全新场景测量。每轮累计至少 10 s 仿真时间，测量 3 轮并报告各轮与中位数。短实验以独立复位重复累计，复位耗时单列且不计入积分区间，不把实验尾部任意延长为静止计时。
2. **含视频端到端 RTF**：对正式实验从首帧采集开始到最后视频/CSV 写入完成计时，包含动作、行为、采样、渲染和编码；同时报告仿真时长和墙钟时长。采用同一测试输入独立运行，核对物理结果的一致性。
3. **准备成本**：进程启动、资产加载/初始化、首次渲染预热及复位单列，不混入稳态无渲染 RTF；端到端测量前的预热必须披露。

计时采用单调墙钟时钟。更细步长或不同硬件结果作为独立对照，不外推标准 RTF；不能从带渲染耗时中简单减去渲染耗时得到无渲染结果。

参考：[官方独立 Python 工作流](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/introduction/workflows.html)、[Isaac Lab 刚体示例](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/01_assets/run_rigid_object.html)。
