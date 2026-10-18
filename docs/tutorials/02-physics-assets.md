# 02 物理资产与物理测试

为第 1 章的模型配置质量、摩擦和关节驱动，导出 USD 物理资产，并在 Isaac Sim 中用 CPU 和 GPU 两种物理后端测试。以[带盖笔 pen/000002](../../asset_sources/objects/pen/000002/) 和[三抽屉盒 cabinet/000000](../../asset_sources/objects/cabinet/000000/) 为例。

## 交付物

源码目录 `asset_sources/objects/<类别>/<编号>/` 中新增两个文件，由 AI 按规范编写：

| 文件 | 作用 |
| --- | --- |
| `physics.py` | 物理属性：质量、接触材料、关节摩擦与驱动（[规范](../designs/asset-pipeline/physics.md)） |
| `physics_test.py` | 物理测试：每项测试的初始状态、施力或夹持动作、相机和关注曲线（[规范](../designs/asset-pipeline/physics-tests.md)） |

原生物理表达不了的行为另写 `behavior.py`（[规范](../designs/asset-pipeline/behaviors.md)），本章两个资产都不需要。

构建结果在 `assets/objects/<类别>/<编号>/`：`object.usdz` 是仿真加载的资产，`physics_build.json` 是构建报告；测试结果在 `physics_test/cpu/` 和 `physics_test/gpu/`。

## 命令

构建物理资产（省略 `--assets` 时构建所有带 `physics.py` 的资产）：

```bash
uv run python scripts/assets/build_physics.py --assets pen/000002 cabinet/000000 --gpu 0
```

构建报告中，笔应有 5 个刚体、11 个部件、79 个碰撞形状，盒子应有 4 个刚体、26 个部件、33 个碰撞形状。

运行物理测试，CPU 和 GPU 各跑一次：

```bash
uv run python scripts/assets/physics_test.py assets/objects/pen/000002 --physics cpu --gpu 0
uv run python scripts/assets/physics_test.py assets/objects/pen/000002 --physics gpu --gpu 0
```

- `--tests close pull_comparison` 只运行指定的测试。
- `--script asset_sources/objects/pen/000002/physics_test.py` 使用源码目录中正在修改的测试，默认用构建时复制的快照。
- GPU 默认同时模拟 4 个相同的环境，结果取第一个环境；`--num-envs` 可修改。
- 默认每项测试只跑一遍；需要在 README 记录性能时加 `--benchmark`，按标准方法测量 RTF（实时因子），耗时约为默认的 3–5 倍。

## 测试项与关键指标

**带盖笔**：笔帽靠三颗钢珠弹簧卡在笔颈的凸棱上。

| 测试 | 内容 | 关键指标（CPU / GPU） |
| --- | --- | --- |
| `pull_comparison` | 固定笔身，夹住笔帽以 5 mm/s 拔出，紧帽和松帽各一支 | 拔帽力：紧帽 2.36 / 2.73 N，松帽约 0 N |
| `close` | 夹住笔帽沿笔轴推：先限力 0.3 N，应停在卡扣前；再逐渐加大推力直到盖紧 | 合帽力 3.74 / 2.14 N |
| `drop_heights` | 三支笔从 0.2、0.5、1 m 同时落地 | 落地后能稳定下来；笔帽被震出的距离：CPU 0.3 / 4.3 / 12.9 mm，GPU 0.4 / 2.2 / 3.5 mm |
| `gravity_rotation` | 紧帽和松帽两支笔从帽朝上慢慢转到帽朝下 | 紧帽保持，松帽滑落 |

**三抽屉盒**：抽屉滑轨用关节摩擦，静摩擦 0.50 N、动摩擦 0.35 N。

| 测试 | 内容 | 关键指标（CPU / GPU） |
| --- | --- | --- |
| `pull_release_drawer1..3` | 逐渐拉开一个抽屉后松手 | 松手后由摩擦减速停下 |
| `travel_stops` | 在全开和全关两端持续推拉 | 停在限位，不越界 |
| `pull_cabinet` | 盒子靠橡胶脚垫放在地面上，拉力逐渐加到 8 N 拉第三个抽屉 | 抽屉拉到底（0.173 m）后，盒子在地面上被拉动 15.3 / 15.9 cm |
| `gravity_tilt` | 夹住外壳，从水平慢慢前倾到 45° | 抽屉起滑角 18.9° / 15.8–16.6°（解析值约 18.6°，要求不低于 15°） |

这些参数都是未经实物标定的仿真值。CPU 与 GPU 的合帽力相差较大，见 [#15](https://github.com/xucj98/table-1000/issues/15)。

## 验收

结果目录 `physics_test/cpu/` 和 `physics_test/gpu/` 分别检查，每项测试看三个文件：

1. `<测试名>.mp4`：完整过程是否符合上表的测试意图，有没有穿透、抖动、飞出等异常；
2. `<测试名>/plots.jpg`：关注曲线的趋势和峰值是否与视频一致；
3. `<测试名>/result.json`：`execution_status` 应为 `completed`，测试出错时为 `failed`，`error` 中写明原因；`experiment` 是测得的关键指标，`performance` 是 RTF。

验收通过后，把关键指标和 `--benchmark` 测得的性能更新到资产 README 的“物理实测”一节。不通过时，修改 `physics.py` 或 `physics_test.py`，重新构建并重跑受影响的测试。
