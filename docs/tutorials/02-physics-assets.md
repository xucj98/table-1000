# 02 物理属性、USD 导出与运动测试

本章完成 **Blender 模型 → 编写物理层与行为 → 构建 USD 资产 → Python 仿真测试 → 人工验收**。以 [带盖笔 pen/000006](../../asset_sources/objects/pen/000006/) 为例：笔身和笔帽是两个独立刚体，使用附加保持力模拟拔合手感。

**输入：** 第一章验收后的 `object.blend`，以及资产源目录中的 `physics.usda`、可选 `behavior.py` 和 `physics_test.py`。

**输出：** 可加载的 USD 资产，以及测试视频、曲线、CSV 和包含实时因子 RTF 的报告。本章使用无界面 Isaac Sim；GUI 加载尚未验证。

## 1. 准备模型与环境

命令均在仓库根目录执行。准备 Blender、FFmpeg，以及已安装 Isaac Sim 5.1 的独立 Python 环境。该环境还需提供 NumPy、SciPy、Pillow、Matplotlib 和 threadpoolctl；不向项目的轻量 uv 环境安装 Isaac。

将下面的路径替换成该环境的 Python 可执行文件：

```bash
export ISAAC_PYTHON=/absolute/path/to/isaac-environment/bin/python
export OMNI_KIT_ACCEPT_EULA=YES
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
```

脚本入口会加入仓库源码路径，无需把公共模块复制到资产目录。环境和资产源码应使用兼容的项目版本。

本例模型位于 `assets/objects/pen/000006/object.blend`。若尚未生成，先执行：

```bash
blender -b --python-exit-code 1 -P scripts/assets/build_assets.py -- --assets pen/000006
uv run python scripts/assets/validate_and_preview.py assets/objects/pen/000006/object.blend \
  --views asset_sources/objects/pen/000006/preview.json
```

按第一章查看报告和预览，通过后继续。已有通过验收且与源码一致的模型可直接使用。

## 2. 编写物理层与行为

编辑源目录中的 [physics.usda](../../asset_sources/objects/pen/000006/physics.usda)，按[物理属性规范](../designs/asset-pipeline/physics.md)配置质量、质心、惯量、接触材料及需要的驱动。引用模型导出的实际节点路径；本例的笔身和笔帽分别为 `/Asset/body`、`/Asset/cap`，接触材料绑定在各部件上。

物理层不重复定义几何。形状、部件划分或关节结构有问题时，返回第一章修改 `object.py`；质量、摩擦和驱动问题在物理层调整。

原生刚体、接触和关节不能表达所需行为时，再提供 [behavior.py](../../asset_sources/objects/pen/000006/behavior.py)，遵循[行为规范](../designs/asset-pipeline/behaviors.md)。本例通过脚本成对施加保持力，自动发现兼容笔身与笔帽，脱离后停止保持。它是等效力模型，不是真实塑料形变，也未经过实物受力标定。

## 3. 构建物理资产

使用 Isaac 环境运行[构建入口](../../scripts/assets/build_physics.py)：

```bash
"$ISAAC_PYTHON" scripts/assets/build_physics.py assets/objects/pen/000006 \
  --source asset_sources/objects/pen/000006 --gpu 0
```

`--gpu` 选择可用显卡。默认输出到输入资产目录；可用 `--output DIR` 指定其他目录。

构建后主要产物为：

| 文件 | 用途 |
| --- | --- |
| `geometry.usdc` | 从 Blender 导出的几何、刚体层级和关节 |
| `physics.usda` | 源物理层快照 |
| `object.usdz` | 组合后的物理资产 |
| `object.usda`、`behavior.py` | 带行为资产的加载入口与旁置脚本 |
| `physics_test.py` | 测试脚本快照 |
| `physics_build.json` | 节点路径映射、刚体和碰撞数量、材料绑定 |
| `model.json` | 导出中间数据，供几何统计等公共工具使用 |

检查 `physics_build.json`：本例应有 **2 个刚体、7 个部件、36 个碰撞形状**。构建器会重新打开 USDZ 检查刚体数量、质量和碰撞材料绑定。

使用资产时，带行为的笔应加载整个目录中的 **`object.usda`**，并在应用入口统一启用官方脚本组件。它引用 `object.usdz`，自动挂载同目录的行为；测试入口已完成这项设置。只打开 USDZ 不会执行旁置 Python。分发时保留整个资产目录及相对引用，详见[行为分发](../designs/asset-pipeline/behaviors.md#分发)。

## 4. 编写并运行测试

### 4.1 设计 physics_test.py

编辑源目录中的 [physics_test.py](../../asset_sources/objects/pen/000006/physics_test.py)，按[物理测试规范](../designs/asset-pipeline/physics-tests.md)编写测试函数并登记到 `TESTS`。每项设置初态、相机、外力或夹具、运行时长，以及关注的曲线。常用操作使用 `TestContext`，特殊操作可直接使用官方 API。

本例提供以下测试：

| 测试名 | 内容 |
| --- | --- |
| `pull` | 固定笔身，弱拉保持、强拉拔开，撤力后笔帽继续运动 |
| `close` | 固定笔身，弱压与强压对照，到位后撤力并观察保持 |
| `pressure_hold` | 持续施压的稳定性对照 |
| `free_cap` | 从已分离初态开始，无夹具，偏心短脉冲后自由平移和旋转 |
| `drop_0.2m_4ms`、`drop_0.5m_4ms`、`drop_1m_4ms` | 三种初始原点高度下的自由跌落 |
| `drop_1m_2ms`、`drop_1m_1ms` | 同一 1 m 初态的步长对照 |
| `two_instances` | 两支笔同时加载，对其中一支施加弱拉力，检查实例隔离 |
| `exchanged_caps` | 初始时交换两支笔的盖子，检查跨实例自动配对；不包含交换操作过程 |

曲线只选关心的量。例如拔帽同时显示外力 X、笔帽位置 X 和速度 X，每个量一个面板，共用时间轴。其他已记录状态仍可从 CSV 分析。

### 4.2 执行仿真

先运行拔帽和合帽：

```bash
"$ISAAC_PYTHON" scripts/assets/physics_test.py assets/objects/pen/000006 \
  --script asset_sources/objects/pen/000006/physics_test.py --tests pull close --gpu 0
```

去掉 `--tests pull close` 则运行全部测试。`--script` 指定当前源码，省略时使用资产目录的测试快照。物理层或行为修改后必须重新构建，再测试。

执行器自动进行独立重复的无渲染计时和一次带视频运行；具体条件见 [RTF 基准](../designs/asset-pipeline/physics-tests.md#rtf-基准)。默认物理步长为 4 ms、视频为 960 × 480 / 25 FPS。较小步长的跌落测试单独报告，不混作同条件性能比较。

默认结果在资产的 `physics_test/` 中，与第一章的 `preview/` 并列：

```text
assets/objects/pen/000006/physics_test/
  pull.mp4
  pull/
    trace.csv
    plots.jpg
    result.json
    frames/
  close.mp4
  close/
    ...
```

`--output DIR` 可覆盖结果位置。重跑前清理所选测试的旧结果，其他测试保留；失败会写入该测试的 `result.json` 并退出非零。

只重画已有曲线、不重新仿真时执行：

```bash
"$ISAAC_PYTHON" scripts/assets/physics_test.py assets/objects/pen/000006 \
  --tests pull close --plots-only
```

该命令使用已有 `result.json` 中记录的曲线列与单位，不会重新执行测试脚本中的 `ctx.plots()`。

## 5. 查看结果并验收

打开各测试的 MP4 和 `plots.jpg`，结合实验预期判断运动与曲线是否合理。再查看 `result.json`：确认执行完成、实际物理与渲染条件，以及无渲染和含视频两种 RTF。`execution_status=completed` 只表示程序运行完成，不能替代人工验收。

当前笔示例的验收结论：

- 拔帽、合帽、已分离笔帽的自由运动、多实例及交换配对通过。合帽撤力后约 0.409 mm 的缝隙，符合本例接受的 1 mm 标准。
- 持续施压仍有抖动，作为已知限制保留，不算稳定性通过。
- 五组跌落均未脱帽，宏观运动粗测通过；仍有步长差异，不认为已收敛，也不据此预测实物的脱帽高度。

本例 4 ms 单笔拔合的无渲染 RTF 约 1.6，含视频约 0.53–0.60；这是所记录硬件与设置下的实测参考，不是所有资产的通过阈值。比较性能时核对完整[基准条件](../designs/asset-pipeline/physics-tests.md#rtf-基准)。

不通过时，根据问题修改模型、物理层、行为或测试设置，重新构建并运行受影响的测试。保留未解决问题，记录人工验收结论。源码和生成物按[资产布局](../designs/asset-pipeline/README.md#资产布局)留存；不将生成的 USD、视频和报告提交到 Git。
