# 透明三格笔筒

![preview](three_quarter.jpg)

透明塑料笔筒，120 × 80 × 100 mm。三列直通空腔，两块隔板到筒口，前沿中段有取物缺口。底板、外壁和隔板组成一个完整刚体，无关节；原点为底面中心，+X 宽度、+Y 背面、+Z 向上。

```text
pen_holder
  container [rigid]
    divider1..2
    floor
    walls
```

## 物理配置

一个完整空筒刚体，质量 0.155 kg，无关节或行为脚本。碰撞体积约 130.45 cm³，按塑料密度约 1180 kg/m³ 估算并取整；非实物称重。质心和惯量由保留空腔的碰撞体计算。资产静/动摩擦为 0.65/0.50、恢复系数 0，平均组合，接触裕量 0.1 mm、静止偏移 0。CPU 使用 TGS、16/4 迭代、CCD 与 speculative CCD。

## 物理实测

Isaac Sim 5.1.0 / Isaac Lab 测试框架，2026-10-20 本机实测；所有数值未经实物标定。CPU 为 1 环境、4 ms；GPU 为 4 环境，接触实验采用 1 ms，自由冲量采用 4 ms，场景最小迭代 64/16，CCD 不支持并关闭。两后端不是同求解条件对比。

| 物理量 | CPU | GPU | 来源与方法 |
|---|---:|---:|---|
| 正立末倾角（deg） | 5.193e-06 | 8.769e-05 | `drop_orientations` / `upright_end_tilt` |
| 侧倒末倾角（deg） | 90 | 90 | `drop_orientations` / `side_end_tilt` |
| 最大几何穿地（mm） | 1.012 | 0.04502 | `drop_orientations` / `ground_penetration` |
| 低位0.5mm起滑施加力（N） | 0.885 | 0.8463 | `push_height` / `low_slide_force` |
| 高位45度倾倒施加力（N） | 0.98 | 0.9837 | `push_height` / `high_tip_force` |
| 最大几何穿地（mm） | 0.01023 | 0.5524 | `push_height` / `ground_penetration` |
| 探针最大穿透（mm） | 0.00202 | 0.0003583 | `cavity_probes` / `probe_penetration` |
| 各环境探针末速（mm/s） | 0.06819 | 5.46 | `cavity_probes` / `probe_end_speed` |
| 线动量相对误差（%） | 6.031e-05 | 0.00166 | `free_impulse` / `momentum_error` |
| 闲置实例速度（m/s） | 0 | 0 | `free_impulse` / `idle_speed` |

数据来自 `physics_test/summary.json`，明细和全程视频位于 `physics_test/isaac-cpu/`、`physics_test/isaac-gpu/`。逐环境断言检查正立/侧倒、低推滑移/高推倾倒、探针四面内壁与底板、末速和自由冲量隔离；画面仅显示环境 0。

透明三格款 GPU 4 ms 的落放/高推分别出现约 3.08/2.79 mm 瞬时穿地，超过 2 mm 接受条件；2 ms 高推仍约 2.25 mm。正式接触实验使用通过相同门槛的 1 ms，不宣称标准 4 ms GPU 接触通过。该数值边界不适于精确冲击预测。

理想测试球半径 4 mm、质量 5 g，接触生成裕量 2 mm、零静止偏移、角阻尼 20/s；阻尼是测试夹具条件，不属于笔筒资产。探针摩擦 0.1/0.1，四向加载 0.024 N。 夹具在初始化前声明于环境子树，通过统一张量路径复制、施力和释放；不在运行中修改 USD 物理属性。接触反力未采样，记录力为施加外力。

仅验证空筒刚体和理想球探针；真实笔装入后的重心、承载和倾倒另做组合验收。不模拟塑料弹性、损伤或真实材料标定。GPU 探针存在小幅底面滚动，末速仍须小于 10 mm/s；不能把数值稳定当成实物正确。

## 运行与性能

从仓库根目录使用第二章教程指定环境执行：

```bash
uv run python scripts/assets/physics_test.py --assets pen_holder/000000 --physics isaac-cpu isaac-gpu --gpus 0 --scripts asset_sources/objects
uv run python scripts/assets/physics_test.py --assets pen_holder/000000 --physics isaac-cpu isaac-gpu --gpus 0 --scripts asset_sources/objects --benchmark
```

加载入口为生成目录中的 `object.usdz`，无需 `behavior.py`。本轮仅修复测试接口，未改变几何与资产物理参数；有效产物与测试源码快照一同保留。

当前环境为总控指定的 Python 3.11.16、Isaac Sim 5.1.0.0、Isaac Lab 源码版本 2.3.2（扩展元数据 0.54.3，源码提交 afca7b09d60d8beb9c1cb28b43066499940b969b）；不能据此宣称与锁定 wheel 2.3.2.post1 严格等价。

新版 `--benchmark` 三轮独立测量完成。CPU 1 环境/4 ms；GPU 4 环境接触 1 ms、自由冲量 4 ms；无渲染每轮累计至少 10 s，准备成本单列。硬件 AMD EPYC 7542 / RTX 4090。PhysX 目标 4 线程；实际 Kit carb_tasking/TBB 为 32/32，未绑核隔离，不属于严格四线程基准。

| 测试 | CPU 无渲染/视频 RTF | GPU 单环境无渲染/视频 RTF | GPU 总吞吐（env·s/s） |
|---|---:|---:|---:|
| `drop_orientations` | 1.197 / 0.2751 | 0.08538 / 0.06701 | 0.3415 |
| `push_height` | 1.079 / 0.2628 | 0.0775 / 0.06082 | 0.31 |
| `cavity_probes` | 0.8962 / 0.2467 | 0.05066 / 0.04198 | 0.2026 |
| `free_impulse` | 0.9074 / 0.2502 | 0.2775 / 0.1534 | 1.11 |

各项独立无渲染与视频终态误差为零，视频运行轨迹与已审功能验证完全一致。GPU 四环境在接触临界可发生轨迹分化，各环境分别验证原合同，不保证轨迹一致。
isaac-cpu 无渲染测量的平均进程 CPU 占用范围 2.72–3.55 核，平均整机 CPU 24.1–25.6%，平均 GPU 利用率 5.44–10.9%。
isaac-gpu 无渲染测量的平均进程 CPU 占用范围 1.32–1.48 核，平均整机 CPU 22.4–25%，平均 GPU 利用率 10–46%。

原报告记录 GPU 计算进程数为 2，现场原始 nvidia-smi 为同一 PID 的重复条目，未见其他仿真竞争；保留原报告。GPU 接触性能基于细步长，与 CPU 4 ms 不作同条件加速比较。

本批测量条件补充：OMP/MKL 环境变量为 1，OPENBLAS_NUM_THREADS 为 32，运行时读回的数学线程池为 1；不能将环境变量与实际线程池混为一谈。渲染为 RealTimePathTracing，direct/dome light 实际采样为 2 spp（规范目标 1 spp），相机通常每帧使用 2 次不推进物理的 render，部分首次 RGB 预热为 3 次（capture_render_pass_counts 实际读回为 2/3）；所有原始读回保留在 result.json。含视频性能按该偏差解释，不能声称标准 1 spp 或整机空闲条件。

生命周期证据为本版 benchmark 的独立场景预热、每次短实验重新初始化/关闭、三轮重复末态与视频末态一致；三实例自由冲量覆盖闲置隔离。无 behavior，无 GUI 行为加载项目；不将独立场景重建冒称为新版 GUI 同场景 stop/reset 专项。几何后代整体性沿用相同 USD 的历史结构检查，未把历史动力学结果记为新版通过。

当前阶段为本组自审通过、待总控独立验收。本地交付清单位于仓库 outputs/asset-physics/pen_holder/closure-2026-10-20/delivery-manifest.json，逐项记录输入哈希、公共依赖、正式结果和本次完整媒体审查；其为本机审计材料，不是资产运行依赖。
