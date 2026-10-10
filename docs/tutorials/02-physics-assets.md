# 02 物理属性、USD 导出与运动测试

以 [带盖笔 pen/000006](../../asset_sources/objects/pen/000006/) 和 [三抽屉盒 cabinet/000002](../../asset_sources/objects/cabinet/000002/) 为例，完成 **配置物理属性 → 构建 USD 资产 → 仿真测试与验收**。

**输入：** 第一章验收后的各资产 `object.blend`。

**输出：** USD 物理资产，以及测试视频、曲线、CSV 和实时因子 RTF 报告。

## 1. 准备环境

使用已安装 Isaac Sim 5.1 的独立 Python 环境，另需 Blender、FFmpeg，以及该 Python 环境中的 NumPy、SciPy、Pillow、Matplotlib、threadpoolctl。命令均在仓库根目录执行，将解释器路径替换为实际位置：

```bash
export ISAAC_PYTHON=/absolute/path/to/isaac-environment/bin/python
export OMNI_KIT_ACCEPT_EULA=YES
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
```

## 2. 编辑并构建物理资产

在资产源目录中准备以下文件：

| 文件 | 编写内容 |
| --- | --- |
| [physics.py](../../asset_sources/objects/pen/000006/physics.py) | 按[物理属性规范](../designs/asset-pipeline/physics.md)通过官方 API 配置质量、接触材料与驱动；质心和惯量可由引擎计算 |
| [behavior.py](../../asset_sources/objects/pen/000006/behavior.py)（可选） | 按[行为规范](../designs/asset-pipeline/behaviors.md)补充原生物理无法表达的行为；本例为笔帽等效保持力 |
| [physics_test.py](../../asset_sources/objects/pen/000006/physics_test.py) | 按[测试规范](../designs/asset-pipeline/physics-tests.md)定义初态、相机、受力或夹具、时长及关注曲线，将测试函数登记到 `TESTS` |

运行构建：

```bash
"$ISAAC_PYTHON" scripts/assets/build_physics.py --assets pen/000006 cabinet/000002 --gpu 0
```

`--source`、`--output` 默认为 `asset_sources/objects`、`assets/objects`；模型从输出根下对应资产目录读取。`--assets` 可指定多个相对资产目录，省略时构建所有包含 `physics.py` 的资产。`--gpu` 选择可用显卡。产物写入对应资产目录，包括 `geometry.usdc`、生成的 `physics.usda`、`object.usdz`、源码快照及 `physics_build.json`。检查报告：本例应有 **2 个刚体、7 个部件、36 个碰撞形状**，三抽屉盒应有 **4 个刚体、26 个部件、33 个碰撞形状**。构建器自动核对质量和碰撞材料绑定。

带行为资产另生成 `object.usda`，引用 USDZ 并挂载旁置脚本。使用时加载该入口，保留整个资产目录；测试入口已启用官方脚本组件，详见[行为分发](../designs/asset-pipeline/behaviors.md#分发)。

## 3. 运行测试并验收

先运行拔帽和合帽：

```bash
"$ISAAC_PYTHON" scripts/assets/physics_test.py assets/objects/pen/000006 \
  --script asset_sources/objects/pen/000006/physics_test.py --tests pull close --gpu 0
```

去掉 `--tests pull close` 运行全部测试。省略 `--script` 时使用资产目录中的测试快照；修改 `physics.py` 或行为后需重新构建。

| 测试 | 内容 |
| --- | --- |
| `pull`、`close` | 固定笔身，弱力与强力拔合，撤力后观察运动与保持 |
| `pressure_hold` | 持续施压稳定性 |
| `free_cap` | 已分离笔帽受偏心短脉冲后的自由运动 |
| `drop_0.2m_4ms`、`drop_0.5m_4ms`、`drop_1m_4ms` | 不同初始原点高度的跌落 |
| `drop_1m_2ms` | 1 m 跌落的步长对照 |
| `drop_zero_friction_*` | 去除笔帽保持摩擦及笔内接触摩擦的跌落对照 |
| `exchanged_caps` | 两帽拔出、交换、插回并释放夹具 |
| `loose_cap_rotation` | 夹持两支笔缓慢旋转，对照紧帽与松帽 |

三抽屉盒使用原生关节摩擦和阻尼，无需行为脚本：

```bash
"$ISAAC_PYTHON" scripts/assets/physics_test.py assets/objects/cabinet/000002 --gpu 0
```

测试覆盖三个抽屉拉动后撤力、上下限位持续受力、到限位后继续拉动整个盒子，以及 0°/15°/35° 倾斜下的重力滑动。

结果默认写入资产的 `physics_test/`，与 `preview/` 并列。例如：

```text
physics_test/
  pull.mp4
  pull/
    trace.csv
    plots.jpg
    result.json
```

`--output DIR` 可指定其他输出目录。重跑会先清理所选测试的旧结果，其他测试保留。

**验收分两步：** 查看完整视频和关注曲线，判断是否符合实验预期；查看 `result.json` 中的执行状态、实际条件及无渲染/含视频两种 RTF。标准条件与计时方法见 [RTF 基准](../designs/asset-pipeline/physics-tests.md#rtf-基准)。

当前笔示例的拔合和自由运动测试通过，合帽缝隙约 0.258 mm。持续施压仍有抖动；跌落仅通过宏观粗测，步长结果未收敛。零摩擦对照在撞地前已脱帽，属于失败诊断。保持力是未标定的等效模型，GUI 加载尚未验证。抽屉示例已通过上述运动粗测，质量与摩擦参数同样未经实物标定。

有问题时修改对应源码，重新构建并运行受影响的测试。按[资产布局](../designs/asset-pipeline/README.md#资产布局)保存源码和生成物。
