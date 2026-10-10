# 资产生产规范

这里集中定义对象资产的源码接口、配置格式、构建产物与验收约定。操作手册说明如何执行流程，通过链接引用本目录，不另维护一套字段定义。规范目录使用 `asset-pipeline`，覆盖源码、导出和测试三个阶段，不绑定某个仿真器。

| 文档 | 内容 | 状态 |
| --- | --- | --- |
| [目录与留存](layout.md) | 资产编号、元数据、源码与生成物、Git 边界 | 第一阶段已实现；第二阶段布局为设计 |
| [object.py 与 object.blend](modeling.md) | 建模入口、刚体子树、坐标系、视觉与碰撞、关节 | 已实现 |
| [preview.json](preview.md) | 几何姿态、相机、关键帧、检查与预览 | 已实现 |
| [physics.json](physics.md) | 质量、材料、驱动与 USDZ 导出 | 设计稿，尚未实现统一入口 |
| [behavior.py](behaviors.md) | 随资产分发的可选运行时行为 | 设计稿，尚未实现统一加载器 |
| [physics_test.json](physics-tests.md) | 初始状态、受力动作、视频和物理验收 | 设计稿，尚未实现统一测试器 |

## 两阶段流程

```text
第一阶段
object.py + metadata.json + preview.json
    → 构建 → object.blend + 源码/配置快照
    → validate_and_preview → JPG / MP4 + acceptance.json

第二阶段（设计）
object.blend + physics.json + 可选 behavior.py + physics_test.json
    → 构建 → object.usdz + 可选 runtime/ + 配置快照
    → 物理仿真测试 → MP4 + acceptance.json + trace.csv
```

两类测试配置都属于输入源码，构建时复制，不根据模型自动生成验收意图。第一阶段验证形状和运动空间；第二阶段验证物理参数与行为。第一阶段通过不等于物理模型通过。

## 数据归属

| 内容 | 唯一来源 |
| --- | --- |
| 几何、刚体划分、保存零位、关节类型/连接/轴/锚点/限位、禁碰关系 | `object.py` 生成的 `object.blend` |
| 预览相机与指定姿态 | `preview.json` |
| 质量、质心/惯量覆盖、材料、驱动、行为绑定 | `physics.json` |
| 原生物理属性不能表达的附加力或状态转换 | 可选 `behavior.py` |
| 实验初态、重力、地面、步长、外力与观察方式 | `physics_test.json` |
| 对象身份 | `metadata.json` |

同一参数不在多个文件重复定义。仿真导出不能重新猜测建模阶段的刚体与关节，也不再次自动分解已经明确给出的凸碰撞网格。

## 坐标、单位与职责

资产坐标系和刚体/关节坐标见[模型规范](modeling.md)。JSON 和行为接口统一采用米、千克、秒、弧度、牛顿和牛顿米；四元数顺序为 `[w, qx, qy, qz]`。导出器/后端适配器负责转换 USD 或引擎 API 的单位，例如旋转限位和驱动参数涉及的角度单位；转换不能只改目标角而漏掉刚度与阻尼。

核心包保持轻量；Blender、OpenUSD 导出器和仿真后端在各自环境执行。`scripts/` 只提供薄入口，共用实现放在 `table_1000/`，具体模块做到时再确定。本项目负责资产和资产测试，不重复实现 robot-bridge 的策略或机器人控制能力。

第一阶段的执行步骤见[第一章](../../tutorials/01-blender-modeling.md)。第二阶段先实现普通抽屉和带盖笔的构建、加载与受力测试，再按实际需要扩展字段；本目录的第二阶段示例目前不是可执行命令或已完成的验收证据。
