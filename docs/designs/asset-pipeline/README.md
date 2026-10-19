# 资产生产规范

本目录规定 Table-1000 对象资产的文件结构与接口，覆盖从 Blender 建模到物理仿真资产的生产和验收。

README 说明整体流程，以及每个文件放在哪里、由哪份规范约定。各文件的字段、单位、默认值与约束见对应规范；具体执行命令见[操作手册](../../tutorials/README.md)。

## 生产流程

两个阶段都按“源码 → 构建产物 → 测试结果”组织。

**第一阶段：建模与形状验收。** `object.py` 生成包含视觉、碰撞几何和关节的 `object.blend`；`preview.json` 指定观察视角和几何姿态，供验收工具检查并渲染图片、视频。

```text
object.py
    ──构建──> object.blend
            + preview.json
            └──检查与渲染──> JPG / MP4 / acceptance.json
```

**第二阶段：物理属性与运动验收。** 将模型导出为 USD 几何层，由 `physics.py` 生成 `physics.usda` 并组合，必要时挂载 `behavior.py`；`physics_test.py` 使用公共工具与官方 API 执行单资产（含多实例）实验，输出视频、曲线和 RTF。

```text
object.blend ──导出──> geometry.usdc
physics.py ──配置──> physics.usda
    ──组合与打包──> object.usdz + 可选 object.usda / behavior.py
                  + physics_test.py
                  └──仿真测试──> MP4 / trace.csv / plots.jpg / result.json
```

**机器人** 以厂商 URDF 包为原始输入：`prepare.py` 写明所选配置与碰撞划分，转换工具生成仓库中的 URDF 与网格；Isaac Sim URDF 导入器生成 USD，`physics.py` 在其上叠加驱动、求解器与接触材料层 `robot.usda`。详见[机器人资产规范](robots.md)。

```text
raw/ 厂商 URDF 包 + prepare.py ──转换与检查──> robot.urdf + meshes/ + check.json
robot.urdf + meshes/ ──导入──> import/robot.usd
physics.py ──配置──> robot.usda
robot.urdf ──Blender 渲染──> three_quarter.jpg
```

模型保存刚体划分、几何和关节定义；物理配置补充质量、材料与驱动。带行为资产使用生成的 `object.usda` 作为加载入口，见[行为分发](behaviors.md#分发)。几何预览配置与物理测试脚本均作为资产源码保存。已提供带盖笔与三抽屉盒的构建和测试示例。

## 资产布局

仓库顶层按“由谁产生、是否进 Git”分成四个目录。三个忽略目录体积大，多个检出可以用软链接指向同一份。

| 目录 | 放什么 | Git |
| --- | --- | --- |
| `asset_sources/` | 资产源码：建模、物理、场景、专家程序，以及很小的缩略图 | 提交 |
| `assets/` | 由源码构建或下载的资产，以及验收要看的预览和测试结果 | 忽略 |
| `data/` | 生成的数据：专家轨迹（HDF5）、LeRobot 数据集 | 忽略 |
| `outputs/` | 日志和临时实验 | 忽略 |

同一资产在 `asset_sources/` 与 `assets/` 下使用相同的相对路径：对象 `objects/<category>/<六位编号>`、机器人 `robots/<名称>`、场景 `scenes/<六位编号>`；编号从 `000000` 开始。下面按资产类型列出每个文件，括号内为规范文档。

**对象**

```text
asset_sources/objects/cabinet/000000/
  object.py           建模入口：坐标系、刚体子树、视觉与碰撞、关节（建模）
  preview.json        几何姿态、相机、关键帧与预览输出（几何预览）
  physics.py          质量、接触材料、驱动（物理属性）
  physics_test.py     物理实验的初态、动作与记录（物理测试）
  behavior.py         可选，随资产加载的附加力或状态转换（行为插件）
  metadata.json       UUID，见下文
  README.md           外观、尺寸、资产树、物理实测，见下文
  three_quarter.jpg   480 × 240 缩略图
assets/objects/cabinet/000000/
  object.blend        object.py 构建的模型
  geometry.usdc       导出的 USD 几何层
  physics.usda        physics.py 生成的物理层
  object.usdz         加载入口；带行为时另有 object.usda 与行为依赖
  *.py、*.json、README.md、three_quarter.jpg    源码快照
  preview/            第一阶段验收：几何检查 acceptance.json、图片与视频
  physics_test/       第二阶段验收：summary.json，isaac-cpu/、isaac-gpu/ 各后端结果
```

**机器人**

```text
asset_sources/robots/x1pro/
  robot.urdf、meshes/ 转换后的 URDF 与视觉、碰撞网格（机器人）
  prepare.py          从厂商 URDF 包选取配置、划分碰撞
  physics.py          驱动、求解器与接触材料
  robot.py            机器人描述：关节、末端、相机
  robot_test.py       验收设置
  README.md、three_quarter.jpg
assets/robots/x1pro/
  raw/                厂商原始 URDF 包，存档于 Hugging Face 数据集，版本见 assets.lock.json
  import/             Isaac Sim URDF 导入结果
  robot.usda          加载入口
  prepare.json、check.json、源码快照
```

**场景**

```text
asset_sources/scenes/000000/
  scene.py            桌子、机器人、物体、录像视角、随机初始化（场景）
  expert.py           专家程序
  thumbnail.jpg       seed 0 初态的头部相机图像，480 × 270
assets/scenes/000000/            验收时看这一层
  summary.json        验收入口：初态检查、生成统计、已渲染的轨迹
  initial.json、initial_states.jpg    seed 0–24 的初态检查与 5 × 5 拼图
  demo_<seed>.mp4     抽查的录像，固定视角左右并排
  demos/demo_<seed>/  每条渲染过的轨迹：评审图 sheet.jpg、机器人相机图像 frames/、完成记录 render.json
data/scenes/000000/
  <后端>/part<k>/     轨迹 trajectories.hdf5（成功）、trajectories_failed.hdf5，每个 seed 的报告 episodes/、日志 run.log
  <后端>/summary.json、summary.csv    该后端的生成汇总
  lerobot/            LeRobot 数据集
```

规范：[建模](modeling.md)、[几何预览](preview.md)、[物理属性](physics.md)、[行为插件](behaviors.md)、[物理测试](physics-tests.md)、[机器人](robots.md)、[场景与专家轨迹](scenes.md)。

统一单位：m、kg、s、rad、N、N·m；四元数顺序为 `[w, qx, qy, qz]`。各文件另行注明坐标系。USD 原生属性遵循各 Schema 的单位，详见[物理属性](physics.md)。源码快照重建依赖同版本的 `table_1000` 包，共用模块不复制到资产目录。外部资源引用须注明版本、来源和许可，不使用私有绝对路径。

## 名称压缩表示

资产树与项目公共名称解析工具支持 `name1..N` 表示法：`cabinet.drawer1..3` 展开为 `cabinet.drawer1`、`cabinet.drawer2`、`cabinet.drawer3`，包含首尾。范围后可接名称后缀，例如 `cabinet.drawer1..3.handle`。

- 每个名称最多一个递增整数范围，不支持通配符或正则表达式；补零编号须保持相同宽度，例如 `collision00..15`。
- 配置展开后的名称必须存在且符合引用类型。同一字段内不得重复引用同一对象；需要不同配置时拆开范围单独填写。
- 资产树自动压缩仅合并同父级、名称仅连续显式编号不同、角色/碰撞类型/材质及归一化子树结构相同的对象；`.001` 自动后缀不参与归并。不按外观猜测同类部件。手写配置范围不要求各对象几何相同。

压缩只影响文字展示或配置书写，模型中的对象仍各自使用完整名称，实际层级不变。需要范围展开的脚本共用公共解析工具，不处理动作名、文件名等普通字符串。USDA 使用原生 USD 语法和实际 prim 路径，不直接支持此缩写。

## README 与缩略图

`README.md` 包含 `![preview](three_quarter.jpg)`、简短外观与尺寸说明、资产树及刚体数；有关节时说明类型、限位和方向。部件名称遵循[建模规范](modeling.md#部件命名与资产树)。

资产树用 `[rigid]` 标注刚体根，子节点可使用局部名称，重复组件可用[名称压缩表示](#名称压缩表示)，例如：

```text
cabinet
  housing [rigid]
    rubber_pad1..4
  drawer1..3 [rigid]
    handle
```

资产树展示所有刚体及其部件，省略部件下的视觉与碰撞网格；使用空格缩进表示实际父子层级，森林保留多个根节点。

带 `physics_test.py` 的资产另列[物理实测](physics-tests.md#人工验收)表，记录各测试测得的关键物理量，如拔出力、合紧力、起滑角：每行写明物理量、CPU 与 GPU 实测值、来源测试及测量方式。数值取自 `physics_test/summary.json`，注明软件版本，并标明为未标定的仿真值；修改几何、`physics.py` 或测试后重新运行并更新。已知后端差异在表下说明或链接 issue。

同一节另列典型条件下的性能：CPU 后端 1 个环境、GPU 后端 4 个并行环境，分别给出无渲染和含视频的 RTF 范围，以及测量期间的本进程 CPU 占用和 GPU 利用率。数值取自 `--benchmark` 运行的 `result.json` 中的 `performance`，注明机器负载等条件，测量方法见 [RTF 基准](physics-tests.md#rtf-基准)。

第一阶段的[连续变形几何](modeling.md#第一阶段连续变形几何)另以 `[deformable]` 标注归属节点，README 单列其数量、控制量和仅几何预览的限制；不得计入刚体数或宣称物理导出已支持。仍使用上述五种第一阶段源文件，几何回调放在 `object.py` 中，随源码快照分发。

`three_quarter.jpg` 是四分之三视角的 480 × 240 的缩略图，约 5 - 10 KB，左视觉、右碰撞。

## metadata.json

现有格式：

```json
{"uuid": "<object-uuid>"}
```

UUID 的用途、修改规则及资产版本/发布机制尚未确定；此处仅记录现有格式。
