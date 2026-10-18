# 02 物理属性、USD 导出与运动测试

以 [带盖笔 pen/000002](../../asset_sources/objects/pen/000002/) 和 [三抽屉盒 cabinet/000000](../../asset_sources/objects/cabinet/000000/) 为例，完成 **配置物理属性 → 构建 USD 资产 → 仿真测试与验收**。

**输入：** 第一章验收后的各资产 `object.blend`。

**输出：** USD 物理资产，以及测试视频、曲线、CSV 和实时因子 RTF 报告。

## 1. 准备环境

仿真环境是项目 uv 环境的 `sim` 依赖组（Isaac Sim 5.1，见 [README](../../README.md#开发环境)），另需 Blender 和 FFmpeg。Isaac Sim 已包含在项目环境中，直接使用 `uv run python`，无需另设解释器。命令均在仓库根目录执行；已有依赖时可跳过 `uv sync`：

```bash
uv sync --locked --extra sim
export OMNI_KIT_ACCEPT_EULA=YES
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
```

## 2. 编辑并构建物理资产

在资产源目录中准备以下文件：

| 文件 | 编写内容 |
| --- | --- |
| [physics.py](../../asset_sources/objects/pen/000002/physics.py) | 按[物理属性规范](../designs/asset-pipeline/physics.md)通过官方 API 配置质量、接触材料与驱动；质心和惯量可由引擎计算 |
| `behavior.py`（可选） | 按[行为规范](../designs/asset-pipeline/behaviors.md)补充原生物理无法表达的行为；本章两个资产均不需要 |
| [physics_test.py](../../asset_sources/objects/pen/000002/physics_test.py) | 按[测试规范](../designs/asset-pipeline/physics-tests.md)定义初态、相机、受力或夹具、时长及关注曲线，将测试函数登记到 `TESTS` |

笔帽通过三颗钢珠与笔颈凸棱的真实接触保持。每颗球通过径向滑动关节连接帽壳，关节驱动提供弹簧力；`physics.py` 设置质量、接触材料和驱动参数，无需脚本控制吸附、配对或分离。

运行构建：

```bash
uv run python scripts/assets/build_physics.py --assets pen/000002 cabinet/000000 --gpu 0
```

`--source`、`--output` 默认为 `asset_sources/objects`、`assets/objects`；模型从输出根下对应资产目录读取。`--assets` 可指定多个相对资产目录，省略时构建所有包含 `physics.py` 的资产。`--gpu` 选择可用显卡。产物写入对应资产目录，包括 `geometry.usdc`、生成的 `physics.usda`、`object.usdz`、源码快照及 `physics_build.json`。检查报告：本例应有 **5 个刚体、11 个部件、79 个碰撞形状**（其中 3 个为球体），三抽屉盒应有 **4 个刚体、26 个部件、33 个碰撞形状**。构建器自动核对质量和碰撞材料绑定。

带行为资产另生成 `object.usda`，引用 USDZ 并挂载旁置脚本。使用时加载该入口，保留整个资产目录；测试入口已启用官方脚本组件，详见[行为分发](../designs/asset-pipeline/behaviors.md#分发)。

## 3. 运行测试并验收

测试使用 Isaac Lab，`--physics cpu|gpu` 选择物理后端（默认 CPU），`--gpu` 选择物理显卡。GPU 后端默认并行运行 4 个相同的环境，记录和视频取第一个环境；`--num-envs` 可改变环境数。分别运行笔帽的 CPU、GPU 验收：

```bash
uv run python scripts/assets/physics_test.py assets/objects/pen/000002 \
  --script asset_sources/objects/pen/000002/physics_test.py \
  --physics cpu --gpu 0
uv run python scripts/assets/physics_test.py assets/objects/pen/000002 \
  --script asset_sources/objects/pen/000002/physics_test.py \
  --physics gpu --gpu 0
```

使用 `--tests` 可选择测试名；省略时运行全部四项。省略 `--script` 时使用资产目录中的测试快照；修改几何后需重新执行建模与预览；修改 `physics.py` 或行为后需重新构建物理资产。

| 测试 | 内容 |
| --- | --- |
| `pull_comparison` | 同场景紧松两笔，固定笔身，夹持笔帽以 5 mm/s 沿 −X 匀速拉出 45 mm，再放稳松开。记录夹持拉力（取峰值为拔帽力）及帽的 X 位置 |
| `close` | 固定笔身，夹持笔帽只沿笔轴推向盖合位置：先限力 0.3 N，应停在卡扣前；再逐渐提高上限直到盖紧，然后松开。记录推力及帽的 X 位置 |
| `drop_heights` | 同场景三笔从 0.2、0.5、1 m 同时落地，观察冲击后运动与恢复 |
| `gravity_rotation` | 同场景紧松两笔，从帽朝上的竖直姿态慢转 180°，观察紧帽保持、松帽滑落 |

三抽屉盒使用原生关节摩擦和阻尼，无需行为脚本：

```bash
uv run python scripts/assets/physics_test.py assets/objects/cabinet/000000 \
  --script asset_sources/objects/cabinet/000000/physics_test.py \
  --physics cpu --gpu 0
uv run python scripts/assets/physics_test.py assets/objects/cabinet/000000 \
  --script asset_sources/objects/cabinet/000000/physics_test.py \
  --physics gpu --gpu 0
```

测试覆盖三个抽屉拉动后撤力、上下限位持续受力、到限位后继续拉动整个盒子，以及夹持外壳从水平连续下倾至 45° 的重力滑动（`gravity_tilt`）。记录三个抽屉达到 0.5 mm 相对位移时的实际角度；本例使用原生关节轴静/动摩擦力 0.50/0.35 N，静摩擦对应的解析起滑角约 18.6°。实测 CPU 约 18.9°、GPU 约 15.8–16.6° 起滑，满足至少 15° 的设计目标；GPU 偏早约 2.5°。

静止锁定 `ctx.fixed` 与夹持 `ctx.grip` 在初始化前声明，运行中移动、驱动与释放，不改变资产材料、质量、惯量及原有关节参数；夹持的增益写入 `result.json` 的 `grips`。GPU 使用更高求解迭代数，关闭不支持的 CCD，笔跌落单独使用 PGS；实际配置记录在 `result.json`，详见[测试规范](../designs/asset-pipeline/physics-tests.md)。

结果按后端写入资产的 `physics_test/cpu/` 和 `physics_test/gpu/`，与 `preview/` 并列。例如：

```text
assets/objects/pen/000002/physics_test/
  cpu/
    pull_comparison.mp4
    pull_comparison/
      plots.jpg
      result.json
      trace.csv
  gpu/
    ...
```

`--output DIR` 可指定其他输出目录。重跑会先清理所选测试的旧结果，其他测试保留。

**验收：** CPU 与 GPU 的结果分别检查，笔 4 项、抽屉柜 6 项，每项看三个文件：

1. `<测试名>.mp4`：完整视频，过程是否符合上表的实验意图；
2. `<测试名>/plots.jpg`：关注曲线，趋势和峰值是否与视频一致；
3. `<测试名>/result.json`：`execution_status` 应为 `completed`（测试中断言失败或出错时为 `failed`，并在 `error` 中给出原因），再看 `experiment` 中的实测量和 `performance` 中的 RTF。

细则见[人工验收](../designs/asset-pipeline/physics-tests.md#人工验收)。默认每项测试只运行一遍，`result.json` 给出含视频 RTF 和同一遍中的物理推进 RTF，用来确认速度没有明显异常。需要在 README 记录性能时，加 `--benchmark` 执行标准测量（预热后测三轮、每轮累计至少 10 s），标准条件见 [RTF 基准](../designs/asset-pipeline/physics-tests.md#rtf-基准)。RTF 为仿真时间除以实际耗时，1 表示实时；小场景的 GPU 吞吐不一定高于 CPU，不同求解配置不能直接作为同条件加速比。

笔帽采用未标定的钢珠弹簧代理，每颗半径 0.8 mm 的球使用 1 g 质量。主要通过视频验收紧松帽拔出、分段施力盖合和重力翻转，结合力与位置曲线判断过程。载荷和动作速度应根据实际响应调整，保留上述测试意图；高速跌落允许短时穿透，但应检查后续能否稳定恢复，不能据此推断真实脱帽高度。抽屉的质量与摩擦参数同样未经实物标定。

验收后，把 `result.json` 中 `experiment` 记录的关键物理量（本例的拔帽力、合帽力和抽屉起滑角），以及 `--benchmark` 测得的 RTF 和资源占用，更新到资产 README 的“物理实测”一节，见 [README 规范](../designs/asset-pipeline/README.md#readme-与缩略图)。

有问题时修改对应源码，重新构建并运行受影响的测试。按[资产布局](../designs/asset-pipeline/README.md#资产布局)保存源码和生成物。
