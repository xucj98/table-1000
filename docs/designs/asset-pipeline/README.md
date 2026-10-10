# 资产生产规范

本目录规定 Table-1000 对象资产的文件结构与接口，覆盖从 Blender 建模到物理仿真资产的生产和验收。

README 说明整体流程、文件分工和存储位置。各文件的字段、单位、默认值与约束见对应规范；具体执行命令见[操作手册](../../tutorials/README.md)。

## 生产流程

两个阶段都按“源码 → 构建产物 → 测试结果”组织。

**第一阶段：建模与形状验收。** `object.py` 生成包含视觉、碰撞几何和关节的 `object.blend`；`preview.json` 指定观察视角和几何姿态，供验收工具检查并渲染图片、视频。

```text
object.py
    ──构建──> object.blend
            + preview.json
            └──检查与渲染──> JPG / MP4 / acceptance.json
```

**第二阶段：物理属性与运动验收。** 在模型上补充 `physics.json`，必要时提供 `behavior.py`，生成 USDZ 与运行时行为包；`physics_test.json` 指定初态、外力和观察方式，驱动物理仿真测试。

```text
object.blend + physics.json + 可选 behavior.py
    ──构建──> object.usdz + 可选 runtime/
            + physics_test.json
            └──仿真测试──> MP4 / trace.csv / acceptance.json
```

模型保存刚体划分、几何和关节定义；物理配置补充质量、材料与驱动。两类测试配置分别描述几何预览和受力实验，均作为资产源码保存。第一阶段已实现；第二阶段的文件格式与接口目前为草案。

## 文件分工

| 文件 | 内容 | 规范 |
| --- | --- | --- |
| `object.py`、`object.blend` | 建模入口、坐标系、刚体子树、视觉与碰撞、关节 | [建模](modeling.md) |
| `preview.json` | 几何姿态、相机、关键帧与预览输出 | [几何预览](preview.md) |
| `physics.json`、`object.usdz` | 质量、接触材料、驱动与仿真导出 | [物理属性](physics.md) |
| `behavior.py`、`runtime/manifest.json` | 随资产加载的附加力或状态转换 | [行为插件](behaviors.md) |
| `physics_test.json` | 实验初态、测试动作、视频与结果记录 | [物理测试](physics-tests.md) |
| `metadata.json` | 现有 UUID 字段；身份与版本规则待定 | 见下文 |

统一单位：m、kg、s、rad、N、N·m；四元数顺序为 `[w, qx, qy, qz]`。各文件另行注明坐标系。后端适配器负责单位转换。

## 名称压缩表示

资产树与所有脚本中的对象名称引用共用 `name1..N` 表示法：`cabinet.drawer1..3` 展开为 `cabinet.drawer1`、`cabinet.drawer2`、`cabinet.drawer3`，包含首尾。范围后可接名称后缀，例如 `cabinet.drawer1..3.handle`。

- 每个名称最多一个递增整数范围，不支持通配符或正则表达式；补零编号须保持相同宽度，例如 `collision00..15`。
- 配置中各对象分别获得相同配置，例如 `drawer1..3` 的 `mass: 0.12` 表示每个抽屉均为 0.12 kg。
- 配置展开后的名称必须存在且符合引用类型。同一字段内不得重复引用同一对象；需要不同配置时拆开范围单独填写。
- 资产树自动压缩仅合并同父级、名称仅连续显式编号不同、角色/碰撞类型/材质及归一化子树结构相同的对象；`.001` 自动后缀不参与归并。不按外观猜测同类部件。手写配置范围不要求各对象几何相同。

压缩只影响文字展示或配置书写，模型中的对象仍各自使用完整名称，实际层级不变。所有脚本通过统一公共解析工具展开名称引用，不在各入口重复实现；此规则适用于引用对象的名称键、名称值和名称列表，不处理动作名、文件名等普通字符串。

## 资产布局

对象路径为 `objects/<category>/<六位编号>`，场景路径为 `scenes/scene-<六位编号>`；编号从 `000000` 开始。

| 位置 | 内容 | Git |
| --- | --- | --- |
| `asset_sources/objects/<category>/<id>/` | object.py、preview.json、metadata.json、README.md、three_quarter.jpg；第二阶段增加 physics.json、physics_test.json、可选 behavior.py | 提交 |
| `assets/objects/<category>/<id>/` | object.blend、object.usdz、源码及配置快照、README.md、已有 three_quarter.jpg、可选 runtime/、preview/ | 忽略 |
| `outputs/` | 物理测试视频、轨迹、报告与日志 | 忽略 |

源码快照重建依赖同版本的 `table_1000` 包，共用模块不复制到资产目录。外部资源引用须注明版本、来源和许可，不使用私有绝对路径。

### README 与缩略图

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

`three_quarter.jpg` 是四分之三视角的 480 × 240 的缩略图，约 5 - 10 KB，左视觉、右碰撞。

### metadata.json

现有格式：

```json
{"uuid": "<object-uuid>"}
```

UUID 的用途、修改规则及资产版本/发布机制尚未确定；此处仅记录现有格式。
