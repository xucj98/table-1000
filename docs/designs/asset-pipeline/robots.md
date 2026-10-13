# 机器人资产规范

规定从厂商 URDF 包得到仿真用机器人资产的文件、几何、碰撞、物理与验收要求。以 [X1Pro](../../../asset_sources/robots/x1pro/) 为例；执行命令见[第三章](../../tutorials/03-robot-import.md)。

## 流程与存储

```text
厂商 URDF 包（assets/robots/<name>/raw/，存档于 Hugging Face 数据集）
    ──prepare.py 的选择 + 转换工具──> robot.urdf + meshes/visual/ + meshes/collision/
                                     └──检查──> check.json
robot.urdf + physics.py ──导入与物理层──> import/robot.usd + robot.usda
robot.urdf ──Blender──> three_quarter.jpg
```

| 位置 | 内容 | Git |
| --- | --- | --- |
| `asset_sources/robots/<name>/` | `robot.urdf`、`meshes/visual/`、`meshes/collision/`、`prepare.py`、`physics.py`、`README.md`、`three_quarter.jpg` | 是 |
| `table_1000/robots/<name>.py` | 规划接口：手臂与夹爪关节、夹爪换算、TCP、相机挂载 | 是 |
| `assets/robots/<name>/raw/` | 厂商原始包，原样保存 | 否；存档于数据集，版本记录在 `assets.lock.json` |
| `assets/robots/<name>/` 其余 | `import/`、`robot.usda`、源码快照、`build.json`、`prepare.json`、`check.json` | 否；可由源码重建 |

源码目录不超过 10 MB，不依赖上游 ROS 包即可构建：URDF 只用相对路径，不用 `package://`。README 写明原始包名称、所选 URDF 配置和对厂商数据的全部改动。

## 运动学与刚体

- 关节名、类型、限位、质量与惯量保持厂商值；改动集中在 `prepare.py` 的 `edit(root)`，并在 README 中逐条列出（例如 mimic 改为独立驱动、电机关节改为固定、限位调整）。
- 固定关节连接的零件并入所属运动刚体，质量与惯量按平行轴定理合并；合并后的连杆保留为固定在刚体上的空坐标系，TCP、相机等位置由它们的固定关节偏移给出。Isaac 导入器只合并无质量连杆，不能代替这一步。
- README 写明刚体数；构建后核对。

## 外观

- 每个外观网格以二次误差简化，面数从总预算按 √(原面数) 分配起步，不满足误差就加倍：简化前后表面距离（双向采样）的 99 分位不超过 **1 mm**。
- 只按面数预算简化会压坏薄壳：X1Pro 躯干在 2.7 万面时误差约 19 mm。
- 外观网格存为 PLY，删除厂商用作占位的 1 mm 方块。

## 碰撞

- 只用凸块。每个外观网格用 CoACD（阈值 0.3）分为 1–4 块，块数在 `prepare.py` 的 `PIECES` 中按连杆给出；传感器、轮子等用一个凸包。凸块按质心排序后写出，使结果不随 CoACD 线程调度变化。
- 外观顶点须在碰撞凸块内，越出不超过 **0.5 mm**；由检查工具测量并写入 `check.json`。
- 厂商 URDF 自带的碰撞网格是非凸网格，不直接使用。
- 包在其他刚体内部、不会接触外界的连杆（如夹爪座内的电机）不设碰撞，列在 `NO_COLLISION`。

## 物理

`physics.py` 的 `author(import_path, output_path)` 在导入结果上写 `robot.usda`，只包含：

| 内容 | 要求 |
| --- | --- |
| 关节驱动 | 刚度、阻尼按弧度或米给出，转换到 USD 的角度单位；力与速度上限取 URDF |
| 求解 | articulation 位置/速度迭代次数 |
| 自碰撞与过滤 | 开启自碰撞；`FilteredPairsAPI` 只排除真机上不会接触、凸块却重叠的刚体对，逐对注明原因 |
| 材料 | 机身与夹爪指垫的静/动摩擦 |
| 重力补偿 | 如采用理想重力补偿（连杆不受重力），在 README 中写明；手中物体仍受重力 |

未对照真机标定的数值在文件和 README 中注明“仿真取值，未标定”。

## 规划接口

`table_1000/robots/<name>.py` 定义任务和规划共用的常量与函数：手臂关节名、手指关节与开合换算、TCP 相对夹爪的偏移、相机挂载位置，以及从源码 URDF 构建规划模型的函数。规划用的机器人模型取源码 `robot.urdf` 和碰撞凸块，不另做一套几何。

## 验收

| 项目 | 通过条件 |
| --- | --- |
| 源码检查 `check.json` | 外观越出碰撞凸块不超过 0.5 mm；目录不超过 10 MB |
| 转换报告 `prepare.json` | 每个外观网格误差不超过 1 mm；刚体数与 README 一致 |
| 构建 | 刚体数、碰撞形状数与 README 一致 |
| 保持姿态 | 4 ms 步长在地面保持初始姿态 2 s，关节跟踪误差小于 0.02 rad；记录每步耗时 |
| 缩略图 | 左为外观，右为按刚体分色的碰撞凸块；人工确认外观无破面、碰撞覆盖完整、刚体划分合理 |

修改 `prepare.py` 的选择或转换工具后，重新生成源码并逐项重验；碰撞凸块变化后，依赖它的场景任务需重新批量验收。
