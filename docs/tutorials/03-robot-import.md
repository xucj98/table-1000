# 03 机器人导入与验收

以双臂移动机器人 [X1Pro](../../asset_sources/robots/x1pro/) 为例，把厂商的 URDF 包变成仿真可用的机器人：可以导入 Isaac Sim 的模型、运动学与规划、关节和末端位姿控制接口，并在仿真中验证行为和性能。各文件的要求见[机器人资产规范](../designs/asset-pipeline/robots.md)。

**输入：** 真机导出的 URDF 包 `ex001_render`（193 MB）。

**输出：** 不超过 10 MB 的机器人源码 `asset_sources/robots/x1pro/`，以及构建产物 `assets/robots/x1pro/robot.usda`。

![X1Pro：左为外观，右为碰撞凸块（每个刚体一种颜色）](../../asset_sources/robots/x1pro/three_quarter.jpg)

## 交付物

源码目录 `asset_sources/robots/x1pro/`，提交到 Git：

| 文件 | 作用 |
| --- | --- |
| `robot.urdf`、`meshes/` | 由厂商包生成：简化后的外观网格和凸块碰撞 |
| `prepare.py` | 转换选择：用哪份 URDF、每个连杆的凸块数、对厂商数据的修改 |
| `physics.py` | 物理层：关节驱动、自碰撞过滤、材料、夹爪耦合、底盘 |
| `robot.py` | 机器人描述 `SPEC`：手臂、升降、头部、底盘、关键姿态和相机 |
| `robot_test.py` | 验收设置：检查场景中的桌子、摆放位置和要运行的用例 |
| `README.md`、`three_quarter.jpg` | 说明（原始包、改动、关节表、实测数据）和缩略图 |

厂商原始包不进 Git，存档在 Hugging Face 私有数据集，版本记录在 [assets.lock.json](../../assets.lock.json)。

## 使用机器人

做场景、任务或其他资产时，只需要仓库中的源码，不需要原始包：

```bash
uv run python scripts/robots/build_robot.py x1pro --gpu 0
```

约 40 s，写出 `assets/robots/x1pro/robot.usda`，场景按 `robots/x1pro` 引用。构建失败时退出码为 1，错误写在 `assets/robots/build-error.txt`。代码中用 `table_1000.robots.load_spec('x1pro')` 取机器人描述，`RobotModel` 做运动学与规划，`RobotEnv` 在仿真中控制机器人（25 Hz 一个动作），见[规范](../designs/asset-pipeline/robots.md#机器人层运动学规划与控制)。

## 开发机器人

修改转换选择或更新厂商数据时，先下载原始包，再重新生成源码（约 10 分钟）：

```bash
uv run hf auth login
uv run python scripts/assets/remote.py pull robots/x1pro/raw/ex001_render
uv run python scripts/robots/prepare_robot.py x1pro
```

生成后会自动检查外观误差和碰撞覆盖，也可单独运行 `prepare_robot.py x1pro --check`（约 10 s，不需要原始包）。上传新的原始包用 `remote.py push <assets 下的目录> -m "说明"`，然后提交更新后的清单。

## 验收

```bash
uv run python scripts/robots/prepare_robot.py x1pro --check
uv run python scripts/robots/check_robot.py x1pro --gpu 0 --output outputs/robots/x1pro-check --video
uv run python scripts/robots/check_robot.py x1pro --physics gpu --gpu 0 --output outputs/robots/x1pro-gpu
uv run python scripts/robots/check_parallel.py x1pro --num-envs 4 --gpu 0 --output outputs/robots/x1pro-parallel
```

依次是：资产检查、CPU 物理用例（带录像）、GPU 物理用例、单卡 4 个环境的并行检查。结果在 `report.json`，失败时退出码非零；`--video` 为外部视角和机器人每路相机各录一个视频（`wide.mp4`、`face_view.mp4` 等）。CPU 用例连同录像约 12 分钟。

**资产检查**

| 项目 | 本例结果 | 通过条件 |
| --- | --- | --- |
| `check.json` | 外观越出碰撞最多 0.19 mm，目录 8.5 MB | 不超过 0.5 mm，目录不超过 10 MB |
| `prepare.json` | 网格最大误差 0.91 mm，22 个刚体 | 每个网格误差不超过 1 mm，刚体数与 README 一致 |
| 缩略图 | 见本章开头 | 外观无破面、塌陷；碰撞覆盖完整，刚体划分合理 |

**物理用例**：本例 12 个用例在 CPU 和 GPU 上全部通过。

| 用例 | 结果要点 |
| --- | --- |
| 关键姿态 | 零位与 `ready` 无自碰撞；静置 1 s 关节漂移为 0 |
| 传感器 | 四路图像中标记物位置与镜头模型预测相差最多 0.7 px，深度误差最多 0.8 mm |
| 手臂撞固定块 | 被挡住，最大穿透 0.1 mm；撤块后到达目标 |
| 手臂推自由物 | 方块和圆柱被推开，仍在桌上，最大穿透 0.2 mm |
| 物体落到机身上 | 停在底盘和头上；撞击穿透 3.0–4.7 mm，在 10 mm 以内 |
| 躯干升入隔板 | 头顶碰到隔板后停住，穿透 0.5 mm，能降回 |
| 自碰撞 | 右臂摆向立柱时在接触处停下，能退回 |
| 夹持与晃动 | 方块、圆柱和直径 14 mm 的细棒晃动后相对 TCP 漂移不超过 0.2 mm |
| 底盘撞墙 | 停在墙前，不推墙，能开走 |
| 底盘连续转向 | 以 1 rad/s 原地转 1.25 圈，转角误差 0.002 rad |
| 持物行驶 | 前进、急停、原地转后手中方块漂移 0.01 mm |
| 关节限位 | 升降与头部停在限位，不越界 |

**对比指标**（不设门槛，用于比较不同方案或机器人）：

| 指标 | 本例 |
| --- | --- |
| 模型与仿真 TCP 一致性（30 个随机姿态） | 位置差 < 0.001 mm，角度差 < 2 µrad |
| IK（每臂 100 个随机可达位姿） | 成功率 99–100%，中位耗时约 0.05 s |
| 自碰撞规划（每臂 20 组随机起止） | 成功率 95–100%，中位耗时 5–6.5 s |
| 关节轨迹跟踪 | 最大误差 0.5 mrad |
| 底盘 | 0.4 m/s 指令实测 0.400 m/s，停车误差 < 0.01 mm |
| 每物理步耗时 | 静止 2.6–2.7 ms，手臂运动 4.8–5.0 ms |

并行检查比较 1 个和 4 个环境在相同输入下的结果、不同输入时各环境的独立跟踪，以及单个环境复位不影响其他环境。

**遥控**：在浏览器中用键盘操作机器人，人工检查行为：

```bash
uv run python scripts/robots/teleop.py x1pro --output outputs/robots/x1pro-teleop --gpu 0 --port 8765
```

打开 `http://<服务器>:8765/`（或 `ssh -L 8765:localhost:8765 <服务器>` 后打开本机地址），点击页面后操作：

| 按键 | 动作 |
| --- | --- |
| W/S、A/D | 底盘前进后退、转向 |
| R/F | 升降 |
| 方向键 | 转头 |
| Tab | 切换手臂 |
| I/K、J/L、U/O | 沿基座坐标系移动 TCP |
| Z/X、C/V、B/N | 转动 TCP |
| 空格 | 开合夹爪 |
| Backspace | 复位 |
| Shift | 按住加速 |

会话动作写入 `actions.jsonl`，可用 `teleop.replay` 回放。

不通过时：外观或碰撞问题改 `prepare.py` 后重新生成；驱动、过滤、材料或夹爪问题改 `physics.py` 后重新构建。

## 同一台机器人换夹爪

真机现用的转动夹爪 rm001 是另一台机器人 [`x1pro_rm001`](../../asset_sources/robots/x1pro_rm001/)：目录中只有夹爪的 4 个网格（0.9 MB），其余网格和 `.py` 文件引用 x1pro。生成、构建与验收用同样的命令，把 `x1pro` 换成 `x1pro_rm001`：

```bash
uv run python scripts/robots/prepare_robot.py x1pro_rm001
uv run python scripts/robots/build_robot.py x1pro_rm001 --gpu 0
uv run python scripts/robots/check_robot.py x1pro_rm001 --gpu 0 --output outputs/robots/x1pro_rm001-check --video
```

`prepare.py` 设 `BASE = 'x1pro'`，原始包中 x1pro 也用到的网格不再生成，所以修改 x1pro 的网格后要重新生成 x1pro_rm001。转动手指同样按指尖间距控制，换算与验收结果见其 README。
