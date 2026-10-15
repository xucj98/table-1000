# 03 机器人导入与验收

以双臂移动机器人 [X1Pro](../../asset_sources/robots/x1pro/) 为例，把厂商的 URDF 包变成仿真可用的机器人：**取回原始包 → 编写转换选择 → 生成机器人源码 → 物理层与构建 → 机器人描述与控制接口 → 验收与留存**。交付物包括可以导入仿真器的模型，规划器、控制接口（关节控制和末端位姿控制），并在仿真中验证行为和性能。各文件的要求见[机器人资产规范](../designs/asset-pipeline/robots.md)。

**输入（开发机器人时）：** 真机导出的 URDF 包 `ex001_render`（193 MB：URDF、44 个外观 STL 共 157.7 MB、厂商碰撞网格）。

**输出：** 不超过 10 MB 的机器人源码 `asset_sources/robots/x1pro/`，以及构建产物 `assets/robots/x1pro/robot.usda`。

![X1Pro：左为外观，右为碰撞凸块（每个刚体一种颜色）](../../asset_sources/robots/x1pro/three_quarter.jpg)

## 两种用法

| 角色 | 需要 | 步骤 |
| --- | --- | --- |
| **使用机器人**：场景、任务和其他资产的开发者 | 仓库中的 `asset_sources/robots/x1pro/` 与 `table_1000/`；不需要原始包，也不需要数据集权限 | 构建（第 4 节），之后按需运行验收与遥控（第 6 节） |
| **开发机器人**：修改转换选择、重新生成网格、更新厂商数据 | 另需厂商原始包（Hugging Face 私有数据集） | 第 1–6 节全部 |

使用机器人需要项目环境的 `sim` 与 `robots` 依赖组（版本见 [README](../../README.md#开发环境) 与 `uv.lock`），以及第二章的 `ISAAC_PYTHON`、`OMNI_KIT_ACCEPT_EULA`：

```bash
uv sync --locked --extra sim --extra robots
export ISAAC_PYTHON=$PWD/.venv/bin/python
```

之后只需三条命令：

```bash
"$ISAAC_PYTHON" scripts/robots/build_robot.py x1pro --gpu 0
"$ISAAC_PYTHON" scripts/robots/check_robot.py x1pro --output outputs/robots/x1pro-check --gpu 0
"$ISAAC_PYTHON" scripts/robots/teleop.py x1pro --output outputs/robots/x1pro-teleop --gpu 0 --port 8765
```

构建把 `assets/robots/x1pro/robot.usda` 写到已忽略的 `assets/` 下，场景按 `robots/x1pro` 引用它；代码里用 `table_1000.robots.load_spec('x1pro')` 取机器人描述，`RobotModel` 做运动学与规划，`RobotEnv` 控制仿真中的机器人（第 5 节）。构建、源码检查、验收和遥控都只读源码目录；只有第 3 节的重新生成需要原始包。

## 1. 准备环境与原始包（开发机器人）

开发机器人另需 `dev` 依赖组（open3d、coacd、huggingface_hub），缩略图需要 Blender：

```bash
uv sync --locked --all-extras
```

厂商原始包不进 Git，存档在 Hugging Face 私有数据集中，数据集名、固定版本和路径记录在仓库根目录的 [assets.lock.json](../../assets.lock.json)。首次使用先登录有访问权限的账号，再把原始包下载到 `assets/`：

```bash
"$(dirname "$ISAAC_PYTHON")/hf" auth login
"$ISAAC_PYTHON" scripts/assets/remote.py pull robots/x1pro/raw/ex001_render
```

原始包落在 `assets/robots/x1pro/raw/ex001_render/`。`remote.py list` 显示清单；`remote.py push <assets 下的目录> -m "说明"` 上传一个目录并把新版本写入清单，随后提交清单。

## 2. 编写转换选择（开发机器人）

机器人目录中的 [prepare.py](../../asset_sources/robots/x1pro/prepare.py) 只写这台机器人的选择，转换流程由 [table_1000/robots/prepare.py](../../table_1000/robots/prepare.py) 统一完成：

| 名称 | 本例内容 |
| --- | --- |
| `RAW`、`URDF` | 原始包在 `assets/` 下的位置；选用平动夹爪配置 `ex001_robot_fx001_h_evt1.urdf` |
| `PIECES` | 每个连杆的凸块数：底盘、躯干、头和手臂各节 4 块，其余（传感器、轮子）1 块 |
| `NO_COLLISION` | 包在夹爪座内部的 `arm_link6` 与电机坐标系，不设碰撞 |
| `edit(root)` | 夹爪电机关节改为固定、两指去掉 mimic 各自驱动，升降关节力与速度上限 |

厂商 URDF 自带的碰撞网格是非凸网格，PhysX 的动态刚体只能用凸体，所以不直接使用；碰撞凸块从简化后的外观网格生成，保证外观在碰撞体内。

## 3. 生成机器人源码（开发机器人）

```bash
"$ISAAC_PYTHON" scripts/robots/prepare_robot.py x1pro
```

约 10 分钟，重写 `robot.urdf` 和 `meshes/`，README、`prepare.py`、`physics.py` 与缩略图保持不变：

1. 删除厂商用作占位的 1 mm 方块。
2. 外观网格二次误差简化：面数从 25 万总预算按 √(原面数) 分配起步，误差（双向表面距离 99 分位）超过 1 mm 就加倍。只按预算简化时，躯干外壳在 2.7 万面时被压坏，误差约 19 mm；按误差简化后躯干约 10 万面、误差 0.28 mm。
3. 每个外观网格用 CoACD 分为凸块，按质心排序写出，结果不随线程调度变化。
4. 应用 `edit(root)`，并把固定关节连接的零件并入所属运动刚体（78 个连杆 → 22 个刚体），质量与惯量按平行轴定理合并。Isaac 导入器只合并无质量连杆，不合并时导入后有 78 个刚体，仿真慢且接近 articulation 上限。

转换结束后自动检查，也可单独运行（约 10 s，只读源码，不需要原始包）：

```bash
"$ISAAC_PYTHON" scripts/robots/prepare_robot.py x1pro --check
```

`assets/robots/x1pro/prepare.json` 记录每个网格的原面数、简化后面数与误差；`check.json` 记录每个外观网格越出其碰撞凸块的最大距离和目录大小。本例：44 个外观网格共约 37 万面，最大误差 0.91 mm（轮子）；64 个凸块文件，越出最多 0.19 mm（夹爪座）；目录 8.5 MB。

## 4. 物理层与构建

[physics.py](../../asset_sources/robots/x1pro/physics.py) 在导入结果上叠加：

- **驱动**：力驱动，增益按弧度或米给出；
- **求解**：articulation 迭代 16/4；
- **自碰撞与过滤**：开启自碰撞，只用 `FilteredPairsAPI` 排除真机上不会接触、凸块却重叠的组合：两指之间（重叠时自碰撞会把手指卡死在闭合位置）、头与躯干之间、零位折叠时前臂（link4）与上臂（link2）之间（两者外观相贴，joint2、joint3 都在限位上，不能更近）。规划模型的 SRDF 排除同样的组合；
- **材料**：机身与指垫摩擦，指垫另设 6 mm 扭转摩擦半径；
- **夹爪**：右指通过 mimic 关节跟随左指（真机一个电机带两指），只驱动左指；
- **底盘**：虚拟底盘关节的刚度和推力上限（取两个轮电机能给出的 116 N、28 N·m）；
- **重力补偿**：连杆不受重力，相当于理想的重力补偿控制。

增益和摩擦是仿真取值，未对照真机标定。

构建 USD：

```bash
"$ISAAC_PYTHON" scripts/robots/build_robot.py x1pro --gpu 0
```

约 40 s（含 Isaac 启动）。移动机器人先写 `mobile.urdf`：在 `base_link` 前加世界 x、y、偏航三个虚拟关节，仿真通过它们移动底盘，机器人 prim 留在原点。构建在 `assets/robots/x1pro/` 写入 `mobile.urdf`、`import/`（URDF 导入器输出）、`robot.usda`（加载入口）、`robot.urdf` 与 `physics.py` 快照和 `build.json`。构建只删除并重建 `import/`，不触及 `raw/`。

## 5. 机器人描述与控制接口

机器人目录中的 [robot.py](../../asset_sources/robots/x1pro/robot.py) 的 `SPEC` 写这台机器人的关节分组：两只手臂（关节、TCP、手指、安装连杆；第一只右臂为主手臂）、升降、头部（偏航、俯仰）、差速底盘（轮距、轮半径、速度、加速度、推力上限）、关键姿态与相机。零位（全部关节为 0，真机最常用的起始姿态）总是关键姿态，另有 `ready`（躯干升起、两臂举在胸前）。升降、头部、底盘都是可选的：固定底座的单臂（如 Franka）只写手臂，机器人放在场景给定的位姿上。其余都是通用代码，结构参照 ManiSkill3：

- [kinematics.py](../../table_1000/robots/kinematics.py) 的 `RobotModel`：一个 MPlib 模型包含全部身体关节，每只手臂一个规划器。提供正运动学、IK、头部 look-at、RRTConnect 与直线规划，只做自碰撞检查。
- 仿真环境基于 Isaac Lab 的 `ManagerBasedEnv`：[table_1000/sim/isaaclab/env.py](../../table_1000/sim/isaaclab/env.py) 的 `RobotEnv(scene, spec, assets)` 由场景源码和 `SPEC` 生成 Isaac Lab 的场景、动作与观测配置；`reset(layout)` 写入物体摆放与机器人关节，`step(action)` 执行一个控制周期。检查、遥控和后续章节的专家脚本都通过它运行，数组带环境维 N。
- 动作 25 Hz 一个，每个动作覆盖 10 个 4 ms 物理步（[actions.py](../../table_1000/sim/isaaclab/actions.py)）：底盘速度、升降与头部位置、手臂关节位置（或 Isaac Lab 的差分 IK 做 TCP 增量）、夹爪间距，见[机器人规范](../designs/asset-pipeline/robots.md#机器人层运动学规划与控制)。
- 目前在 CPU PhysX 上验收（N = 1）；GPU 仿真只需把 `device` 换成 `cuda`，验证见 [issue #11](https://github.com/xucj98/table-1000/issues/11)。

**感知接口**：`SPEC.cameras` 的每一路图像是一个 `CameraSpec`，由传感器参数（`Sensor`：分辨率、内参 `(fx, fy, cx, cy)`、OpenCV 镜头模型（`pinhole` 带径向和切向畸变，或 `fisheye`）、图像类型 `rgb` 或以米为单位的 `depth`）和安装位置组成。安装位置写 URDF 中的光学坐标系名（OpenCV 约定 +Z 向前、+Y 向下），或 `(link, xyz, rpy)` 直接装在某个连杆上，不必改 URDF。实验可以用 `override_cameras(cameras, config)` 改参数或增删相机，规则同 ManiSkill 的 `sensor_configs`：相机名下的设置只改这一路，其余键改全部相机，`resize=(w, h)` 按比例缩放内参。名字与 robot-bridge 中 X1Pro 的视角一致：

| 名字 | 坐标系 | 图像 | 参数来源 |
| --- | --- | --- | --- |
| `face_view` | `camera_head_front_color_optical_frame` | RGB 1280×720，针孔加畸变 | 头部相机（Orbbec DaBai DCW2）出厂标定，2026-10-10 读取 |
| `face_depth` | `camera_head_front_depth_optical_frame` | 深度 640×400，针孔无畸变 | 同上；与 RGB 相距 12 mm |
| `left_wrist_view`、`right_wrist_view` | `<side>_arm_gripper_camera_color_frame` | RGB 640×480，鱼眼 | 分辨率为真机值；标定未知，暂用水平视场约 130° 的理想等距鱼眼代替，拿到 `camera_info` 后替换 |

仿真用同一镜头模型渲染（RTX 的 OpenCV 针孔与鱼眼镜头），所以 `table_1000.robots.cameras.project` 按 `Sensor` 投影的像素就是仿真图像中的位置。观测按 ManiSkill 的结构组织，只含真机也能测到的量：`agent`（关节位置与速度、底盘位姿、各手臂 TCP 位姿）和 `sensor_param`（每路相机的内参与光学坐标系的世界位姿）。`RobotEnv.images(names)` 在当前状态渲染相机，返回图像（RGB 为 uint8，深度为 float32 米，无返回处为 0）；图像只在调用时渲染，控制步不渲染。物体位姿、接触等仿真才有的量通过 `privileged()` 等方法取得，只给专家、验收和评测用。

## 6. 验收

```bash
"$ISAAC_PYTHON" scripts/robots/prepare_robot.py x1pro --check
"$ISAAC_PYTHON" scripts/robots/check_robot.py x1pro --output outputs/robots/x1pro-check --gpu 0 --video
"$ISAAC_PYTHON" -m table_1000.robots.preview asset_sources/robots/x1pro --joints '{"lift_joint": 0.4, "head_pitch_joint": 0.3, "right_arm_joint2": 0.4, "right_arm_joint3": -1.0, "left_arm_joint2": 0.4, "left_arm_joint3": -1.0, "right_arm_gripper_left_joint": 0.03, "right_arm_gripper_right_joint": -0.03, "left_arm_gripper_left_joint": 0.03, "left_arm_gripper_right_joint": -0.03}'
```

**资产检查**

| 项目 | 本例结果 | 通过条件 |
| --- | --- | --- |
| `check.json` | PASS：越出最多 0.19 mm，8.5 MB | 越出不超过 0.5 mm，目录不超过 10 MB |
| `prepare.json` | 最大误差 0.91 mm，22 个刚体 | 每个网格误差不超过 1 mm，刚体数与 README 一致 |
| 缩略图 | 见本章开头 | 外观无破面、塌陷；碰撞覆盖完整，刚体划分合理 |

**物理常识用例**：[check_robot.py](../../scripts/robots/check_robot.py) 在检查场景（机器人前方一张桌子，几个盒、球、圆柱）里依次运行下列用例。用例本身是通用的（[checks.py](../../table_1000/robots/checks.py)），这台机器人的设置写在 [robot_test.py](../../asset_sources/robots/x1pro/robot_test.py)：桌子、机器人摆放与起始姿态、桌面上主手臂前方的工作点（各用例的目标位姿相对它给出）、物体下落的目标连杆和要运行的用例，与物体资产各自的 `physics_test.py` 同理。需要升降或移动底盘的用例只用于有这些部件的机器人。每个用例从 `reset` 开始，结果写入 `report.json`。`--video` 时每个视角把全部用例录成一个视频：外部视角 `wide.mp4`，以及机器人每路相机一个（`face_view.mp4`、`face_depth.mp4` 以灰度表示深度、`left_wrist_view.mp4`、`right_wrist_view.mp4`），用相机描述中的分辨率和镜头渲染。用例与判据见[机器人规范](../designs/asset-pipeline/robots.md#验收)。本例 11 个用例全部通过：

| 用例 | 结果 | 要点 |
| --- | --- | --- |
| 关键姿态 | PASS | 零位与 `ready`：规划模型无碰撞对；仿真静置 1 s 无机器人内部接触，关节漂移 0 |
| 传感器 | PASS | 四路图像在 `ready` 和头臂各转 0.2 rad 后：标记物中心与镜头模型预测相差最多 0.7 px（头部）、0.05 px（腕部）；深度误差最多 0.8 mm；画面中机器人自身占 0%（头部）、约 9%（腕部，夹爪） |
| 手臂撞固定块 | PASS | 被挡住，最大穿透 0.1 mm，去趋势抖动 0.8 mrad；撤块后到达目标 |
| 手臂推自由物 | PASS | 方块和圆柱被推开，仍在桌上，最大穿透 0.2 mm |
| 物体落到机身上 | PASS | 停在底盘、头上的方块离外观表面 2.9 mm 和 0.2 mm；砸到躯干和头部时的冲击穿透 3.0–4.7 mm，在 10 mm 以内 |
| 躯干升入隔板 | PASS | 头顶到隔板后停住，穿透 0.6 mm，能降回 |
| 自碰撞 | PASS | 右臂摆向立柱时在接触处停下，能退回 |
| 夹持与晃动 | PASS | 方块、圆柱、直径 14 mm 的细棒晃动后相对 TCP 漂移不超过 0.2 mm |
| 底盘撞墙 | PASS | 停在墙前，不推墙，能开走 |
| 持物行驶 | PASS | 前进、急停、原地转后手中方块漂移 0.01 mm |
| 关节限位 | PASS | 升降与头部停在限位，不越界 |

下落撞击的判据放宽到 10 mm：4 ms 步长下高速撞击时的轻微穿透可以接受；躯干和头部由几个互相重叠的凸块组成，物体停在一个凸块表面时，PhysX 也可能报告它进入相邻凸块的深度（停在头上的方块离外观 0.2 mm，报告穿透 1.75 mm）。慢速与静止接触仍按 2 mm。

用例曾发现两处物理问题，已修正：两指各自驱动、都顶在力上限时没有回复力，原地转向时手中方块连同两指侧移 13 mm，改为 mimic 耦合；指垫点接触没有扭转摩擦，晃动时圆柱绕两指连线转动，加了扭转摩擦半径。

**对比指标**（不设门槛，用于比较碰撞方案、参数或机器人）：

| 指标 | 本例 |
| --- | --- |
| 模型与仿真 TCP 一致性（30 个随机姿态） | 位置差 < 0.001 mm，角度差 < 2 µrad |
| IK（100 个随机可达位姿/臂） | 成功率 99–100%，中位耗时约 0.05 s，精度 < 0.01 mm |
| 自碰撞规划（20 组随机起止/臂） | 成功率 95–100%；耗时中位 6.5 s（右）、5.2–6.0 s（左），95 分位 10–19 s |
| 关节轨迹跟踪 | 最大误差 0.5 mrad |
| 底盘 | 0.4 m/s 指令实测 0.400 m/s；原地转向平移漂移可忽略；停车误差 < 0.01 mm |
| 每物理步耗时 | 静止 2.6–2.7 ms，手臂运动 4.8–5.0 ms（检查场景） |

随机起止的规划耗时偏长。[碰撞方案研究](https://github.com/xucj98/table-1000/blob/7660d8c0c0763cee752152c25ddbbc255aef495a/docs/research/robot-collision.md)（ref `refs/research/robot-collision`，只含源码）比较了凸块块数、顶点上限和盒子与圆柱几种碰撞：碰撞形状几乎不影响仿真耗时；规划的大头是自碰撞检查，耗时取决于要检查的连杆对数而不是形状，提速应减少检查的对数。IK 和规划的指标在两次运行间有随机波动，表中为范围。用例和指标一次约 12 分钟（含录像）。

**遥控**：在浏览器中用键盘操作，供人工验收和测试：

```bash
"$ISAAC_PYTHON" scripts/robots/teleop.py x1pro --output outputs/robots/x1pro-teleop --gpu 0 --port 8765
```

打开 `http://<服务器>:8765/`（或 `ssh -L 8765:localhost:8765 <服务器>` 后打开本机地址），页面显示外部视角和机器人自己的 RGB 相机（头部、两个腕部），点击页面后操作：W/S 前进后退，A/D 转向，R/F 升降，方向键转头，Tab 切换手臂，I/K、J/L、U/O 沿基座坐标系移动 TCP，Z/X、C/V、B/N 转动 TCP，空格开合夹爪，Backspace 复位，按住 Shift 加速。状态行显示 TCP、底盘位姿、IK 失败和实时倍率；会话动作写入 `actions.jsonl`，可用 `teleop.replay` 回放。服务器无界面运行，画面每 `--render-every` 个控制周期（默认 5）渲染一次，不保存图片，渲染决定运行速度。

不通过时：外观或碰撞问题改 `prepare.py` 的选择（或转换工具）后重新生成；驱动、过滤、材料或夹爪问题改 `physics.py` 后重新构建。碰撞或物理参数变化后，依赖它的场景任务要重新批量验收。

## 7. 留存成果

| 位置 | 内容 | 留存方式 |
| --- | --- | --- |
| `asset_sources/robots/x1pro/` | `robot.urdf`、`meshes/`、`prepare.py`、`physics.py`、`robot.py`、`robot_test.py`、`README.md`、`three_quarter.jpg` | Git |
| `table_1000/robots/`、`table_1000/sim/`、`scripts/robots/`、`tests/robots/` | 与机器人无关的转换、构建、预览、运动学、控制器、环境与后端、验收用例与遥控；单元测试（固定底座单臂）与每台机器人的源码测试 | Git |
| `assets.lock.json` | 原始包的数据集版本 | Git |
| `assets/robots/x1pro/raw/` | 厂商原始包 | Hugging Face 数据集 |
| `assets/robots/x1pro/` 其余 | `import/`、`robot.usda`、快照、`build.json`、`prepare.json`、`check.json` | 本地，可由源码重建 |
| `outputs/robots/x1pro-check/` | 检查报告与视频 | 本地；结论写入任务报告 |

README 写明原始包和所选配置、关节表、对厂商数据的全部改动、几何与碰撞的实测数据和物理取值。
