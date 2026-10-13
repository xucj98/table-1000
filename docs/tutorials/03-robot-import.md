# 03 机器人导入与验收

以双臂移动机器人 [X1Pro](../../asset_sources/robots/x1pro/) 为例，把厂商的 URDF 包变成仿真可用的机器人资产：**取回原始包 → 编写转换选择 → 生成机器人源码 → 物理层与构建 → 验收与留存**。各文件的要求见[机器人资产规范](../designs/asset-pipeline/robots.md)。

**输入：** 真机导出的 URDF 包 `ex001_render`（193 MB：URDF、44 个外观 STL 共 157.7 MB、厂商碰撞网格）。

**输出：** 不超过 10 MB 的机器人源码 `asset_sources/robots/x1pro/`，以及构建产物 `assets/robots/x1pro/robot.usda`。

![X1Pro：左为外观，右为碰撞凸块（每个刚体一种颜色）](../../asset_sources/robots/x1pro/three_quarter.jpg)

## 1. 准备环境与原始包

环境同第二章（`ISAAC_PYTHON`、`OMNI_KIT_ACCEPT_EULA`）。该 Python 环境另需 trimesh、open3d、coacd、huggingface_hub，缩略图需要 Blender。

厂商原始包不进 Git，存档在 Hugging Face 私有数据集中，数据集名、固定版本和路径记录在仓库根目录的 [assets.lock.json](../../assets.lock.json)。首次使用先登录有访问权限的账号，再把原始包下载到 `assets/`：

```bash
"$(dirname "$ISAAC_PYTHON")/hf" auth login
"$ISAAC_PYTHON" scripts/assets/remote.py pull robots/x1pro/raw/ex001_render
```

原始包落在 `assets/robots/x1pro/raw/ex001_render/`。`remote.py list` 显示清单；`remote.py push <assets 下的目录> -m "说明"` 上传一个目录并把新版本写入清单，随后提交清单。

## 2. 编写转换选择

机器人目录中的 [prepare.py](../../asset_sources/robots/x1pro/prepare.py) 只写这台机器人的选择，转换流程由 [table_1000/robots/prepare.py](../../table_1000/robots/prepare.py) 统一完成：

| 名称 | 本例内容 |
| --- | --- |
| `RAW`、`URDF` | 原始包在 `assets/` 下的位置；选用平动夹爪配置 `ex001_robot_fx001_h_evt1.urdf` |
| `PIECES` | 每个连杆的凸块数：底盘、躯干、头和手臂各节 4 块，其余（传感器、轮子）1 块 |
| `NO_COLLISION` | 包在夹爪座内部的 `arm_link6` 与电机坐标系，不设碰撞 |
| `edit(root)` | 夹爪电机关节改为固定、两指去掉 mimic 各自驱动，升降关节力与速度上限 |

厂商 URDF 自带的碰撞网格是非凸网格，PhysX 的动态刚体只能用凸体，所以不直接使用；碰撞凸块从简化后的外观网格生成，保证外观在碰撞体内。

## 3. 生成机器人源码

```bash
"$ISAAC_PYTHON" scripts/assets/prepare_robot.py x1pro
```

约 10 分钟，重写 `robot.urdf` 和 `meshes/`，README、`prepare.py`、`physics.py` 与缩略图保持不变：

1. 删除厂商用作占位的 1 mm 方块。
2. 外观网格二次误差简化：面数从 25 万总预算按 √(原面数) 分配起步，误差（双向表面距离 99 分位）超过 1 mm 就加倍。只按预算简化时，躯干外壳在 2.7 万面时被压坏，误差约 19 mm；按误差简化后躯干约 10 万面、误差 0.28 mm。
3. 每个外观网格用 CoACD 分为凸块，按质心排序写出，结果不随线程调度变化。
4. 应用 `edit(root)`，并把固定关节连接的零件并入所属运动刚体（78 个连杆 → 22 个刚体），质量与惯量按平行轴定理合并。Isaac 导入器只合并无质量连杆，不合并时导入后有 78 个刚体，仿真慢且接近 articulation 上限。

转换结束后自动检查，也可单独运行（约 10 s）：

```bash
"$ISAAC_PYTHON" scripts/assets/prepare_robot.py x1pro --check
```

`assets/robots/x1pro/prepare.json` 记录每个网格的原面数、简化后面数与误差；`check.json` 记录每个外观网格越出其碰撞凸块的最大距离和目录大小。本例：44 个外观网格共约 37 万面，最大误差 0.91 mm（轮子）；64 个凸块文件，越出最多 0.19 mm（夹爪座）；目录 8.5 MB。

## 4. 物理层与构建

[physics.py](../../asset_sources/robots/x1pro/physics.py) 在导入结果上叠加：

- **驱动**：力驱动，增益按弧度或米给出；
- **求解**：articulation 迭代 16/4；
- **自碰撞与过滤**：开启自碰撞，只用 `FilteredPairsAPI` 排除两指之间、头与躯干之间这类真机上不会接触、凸块却重叠的组合（两指凸块重叠时自碰撞会把手指卡死在闭合位置）；
- **材料**：机身与指垫摩擦；
- **重力补偿**：连杆不受重力，相当于理想的重力补偿控制。

增益和摩擦是仿真取值，未对照真机标定。任务与规划共用的约定（手臂与手指关节、开合换算、TCP、头部相机挂载）写在 [table_1000/robots/x1pro.py](../../table_1000/robots/x1pro.py)，规划模型直接取源码 `robot.urdf` 和碰撞凸块。

构建 USD：

```bash
"$ISAAC_PYTHON" scripts/assets/build_robot.py x1pro --gpu 0
```

约 40 s（含 Isaac 启动），在 `assets/robots/x1pro/` 写入 `import/`（URDF 导入器输出）、`robot.usda`（加载入口）、`robot.urdf` 与 `physics.py` 快照和 `build.json`。构建只删除并重建 `import/`，不触及 `raw/`。

## 5. 验收

```bash
"$ISAAC_PYTHON" tests/robots/isaac_x1pro.py --robot assets/robots/x1pro/robot.usda --output outputs/x1pro-check --gpu 0
"$ISAAC_PYTHON" -m table_1000.robots.preview asset_sources/robots/x1pro --joints '{"lift_joint": 0.4, "head_pitch_joint": 0.3, "right_arm_joint2": 0.4, "right_arm_joint3": -1.0, "left_arm_joint2": 0.4, "left_arm_joint3": -1.0, "right_arm_gripper_left_joint": 0.03, "right_arm_gripper_right_joint": -0.03, "left_arm_gripper_left_joint": 0.03, "left_arm_gripper_right_joint": -0.03}'
```

| 项目 | 本例结果 | 通过条件 |
| --- | --- | --- |
| `check.json` | PASS：越出最多 0.19 mm，8.5 MB | 越出不超过 0.5 mm，目录不超过 10 MB |
| `prepare.json` | 最大误差 0.91 mm，22 个刚体 | 每个网格误差不超过 1 mm，刚体数与 README 一致 |
| 保持姿态 `result.json` | PASS：22 个刚体、86 个碰撞形状，每步约 0.6 ms，跟踪误差小于 1 mrad | 4 ms 步长保持 2 s，误差小于 0.02 rad |
| 缩略图 | 见本章开头 | 外观无破面、塌陷；碰撞覆盖完整，刚体划分合理 |

缩略图 1920 × 960：左为外观网格，右为碰撞凸块，每个刚体一种颜色。看外观时注意大件薄壳（躯干、底盘）有无塌陷和裂缝；看碰撞时确认颜色边界与关节划分一致，例如立柱与底盘同属一个刚体，胸部和两肩横梁属于升降的躯干。

不通过时：外观或碰撞问题改 `prepare.py` 的选择（或转换工具）后重新生成；驱动、过滤或材料问题改 `physics.py` 后重新构建。碰撞凸块变化后，依赖它的场景任务要重新批量验收（第四章）。

## 6. 留存成果

| 位置 | 内容 | 留存方式 |
| --- | --- | --- |
| `asset_sources/robots/x1pro/` | `robot.urdf`、`meshes/`、`prepare.py`、`physics.py`、`README.md`、`three_quarter.jpg` | Git |
| `table_1000/robots/`、`scripts/assets/` | 转换、构建、预览工具与 `x1pro.py` 规划接口 | Git |
| `assets.lock.json` | 原始包的数据集版本 | Git |
| `assets/robots/x1pro/raw/` | 厂商原始包 | Hugging Face 数据集 |
| `assets/robots/x1pro/` 其余 | `import/`、`robot.usda`、快照、`build.json`、`prepare.json`、`check.json` | 本地，可由源码重建 |
| `outputs/x1pro-check/` | 保持姿态测试结果 | 本地 |

README 写明原始包和所选配置、关节表、对厂商数据的全部改动、几何与碰撞的实测数据和物理取值。
