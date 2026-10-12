# 03 设计场景

以 [scene-000000](../../asset_sources/scenes/scene-000000/) 为例，把机器人和第二章的两件资产放到同一张桌子上：**准备机器人资产 → 编写场景源码 → 按可达范围选区域 → 静态检查**。

**输入：** 第二章构建好的 `assets/objects/cabinet/000000`、`assets/objects/pen/000002`；机器人 URDF 源码 [asset_sources/robots/x1pro](../../asset_sources/robots/x1pro/)。

**输出：** `assets/robots/x1pro/robot.usda`；场景源码 `scene.py`，含按 seed 的初始摆放与布局检查。

环境同第二章（`ISAAC_PYTHON`、`OMNI_KIT_ACCEPT_EULA`）；该 Python 环境另需 MPlib 0.2.1、toppra、trimesh，机器人缩略图需要 Blender。

## 1. 机器人资产

机器人源码目录只放 URDF、它按相对路径引用的网格、`physics.py` 和 README，见 [X1Pro README](../../asset_sources/robots/x1pro/README.md)：

| 内容 | 做法 |
| --- | --- |
| 运动学 | 取真机控制容器中展开的 URDF，关节、限位、质量、惯量保持真机值；夹爪电机关节改为固定，两指各自按米驱动 |
| 刚体 | 固定关节连接的零件合并进所属运动刚体（22 个），否则导入后有 78 个刚体，仿真慢且接近 articulation 上限 |
| 外观 | 真机 STL 二次误差简化到约 25 万面 |
| 碰撞 | 每个外观网格用 CoACD 分成 1–4 个凸块，外观全部包在碰撞体内 |
| 物理 | `physics.py` 在导入结果上叠加驱动、求解迭代、自碰撞、材料和重力补偿，并关闭两指之间、头与躯干之间的碰撞 |

两指的凸块互相重叠时，自碰撞会把手指卡死在闭合位置；这类真机上不会接触的组合在 `physics.py` 里用 `FilteredPairsAPI` 排除。

构建、检查并渲染缩略图：

```bash
"$ISAAC_PYTHON" scripts/assets/build_robot.py x1pro --gpu 0
"$ISAAC_PYTHON" tests/robots/isaac_x1pro.py --robot assets/robots/x1pro/robot.usda --output outputs/x1pro-check --gpu 0
"$ISAAC_PYTHON" -m table_1000.robots.preview asset_sources/robots/x1pro --joints '{"lift_joint": 0.4}'
```

检查脚本在 4 ms 步长下保持一个姿态 2 s，`result.json` 应为 PASS：22 个刚体、86 个碰撞形状、每步约 1 ms、关节误差小于 0.02 rad。

## 2. 编写场景源码

场景是一个 Python 模块，不另设配置格式。[scene.py](../../asset_sources/scenes/scene-000000/scene.py) 写明：

- 桌子尺寸、高度和摩擦；
- 机器人资产、base_link 位姿和全部初始关节值（升降 0.40 m、头俯仰、双臂初始姿态）；
- 每件物体的资产路径、区域中心、高度和朝向，以及 ±0.05 m 的随机范围；
- `placement(seed=None, offsets=None)`：按 seed 独立抽两件物体的 XY 偏移，返回各自的 4×4 位姿；
- `check(layout)`：返回违反的布局约束，空表示有效。

运行时由 `table_1000/scenes/stage.py` 把场景组装进 Isaac Sim 舞台：桌子用基本体创建，机器人和物体原样引用构建产物，只在引用 prim 上设置位姿。

## 3. 按可达范围选区域

区域位置由机器人能否完成每个关键姿态决定，不靠试跑。离线检查只需 MPlib，不启动 Isaac Sim：

```bash
"$ISAAC_PYTHON" scripts/tasks/drawer_pen_reach.py --output outputs/drawer-pen/reach
```

[reach.py](../../table_1000/tasks/reach.py) 对两个区域的 3 × 3 代表点组合（81 种），用与求解器相同的坐标计算，对任务的每个关键 TCP 位姿求带碰撞检查的逆解，约 2 分钟。`reach.json` 列出每个组合每个位姿是否可达。本例的关键位姿为：

- 把手预抓与抓取；
- 拉开后的把手位置及其后撤点；
- 笔的预抓、抓取与抬起；
- 抽屉上方的松手位。

当前布局 81 种组合全部可达。调整区域时先跑这一步，再跑第四章的批量仿真。本例从中得到的选址约束：

1. 桌子前沿离机器人躯干约 4 cm，躯干下沿低于桌面，机器人不能再靠前。
2. 抽屉盒太近时，拉开后的把手离肩太近，手臂折叠，上臂和前臂相碰；盒子区域中心放在 base_link 前方约 0.86 m。
3. 竖直向下的夹爪在 0.86 m 以上高度只能到 base_link 前方约 0.6 m；放笔时手向抽屉里侧倾斜 40° 解决。
4. 笔区靠右放在双臂之间，并在 X 方向与抽屉拉出的范围错开。

## 4. 静态检查

```bash
"$ISAAC_PYTHON" -m pytest -q tests/scenes
```

[test_scene_000000.py](../../tests/scenes/test_scene_000000.py) 检查：

- 同一 seed 结果相同；
- 两件物体的偏移相互独立，且在区域内；
- 朝向不变；
- 81 种代表组合全部满足：物体整体在桌面内，笔不进入抽屉完全拉出时扫过的范围，离机器人一侧桌边至少 0.05 m。

动态的可行性（拉开抽屉需要多大力、笔能否放稳）在第四章的整条流程里验证。
