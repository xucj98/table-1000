# physics.usda 与 object.usdz

本规范规定第二阶段的目标接口，原 JSON 构建流程正在迁移。

`physics.usda` 是人工或 AI 编辑的 USD 物理层，通过原生 `UsdPhysics` 和 `PhysxSchema` 为几何层添加质量、接触材料和驱动。不再定义平行的物理 JSON 字段。源文件随 Git 提交，生成的几何层和 USDZ 不提交。

## 分层与节点

```text
object.blend ──导出──> geometry.usdc
                          + physics.usda
                          └──组合与打包──> object.usdz
```

几何层保存视觉、碰撞、刚体划分、关节连接/轴/锚点/限位及禁碰关系。物理层通过 `over` 修改已有节点，新增物理材质和驱动；不重复保存网格。构建入口组合两层，不要求物理层独立包含完整资产。

- `/Asset` 为资产参考系 Xform，不是刚体。
- 刚体根映射为带 `RigidBodyAPI` 的 Xform；部件映射为普通 Xform；视觉与碰撞网格保留在部件下。
- USD 保留实际父子层级。各层使用局部名称，并转换成合法 USD 标识符；转换后的同胞名称不得冲突。导出节点保存原 Blender 全局名称，供测试按名称查找。
- 物理层引用实际 USD prim 路径，例如 `/Asset/cabinet/housing`。USD 文本不解释项目的范围缩写；需要批量修改时使用 Python 的公共名称解析和官方 USD API。

例如，给已有笔身设置质量：

```usda
#usda 1.0

over "Asset"
{
    over "body" (
        prepend apiSchemas = ["PhysicsMassAPI"]
    )
    {
        float physics:mass = 0.012
    }
}
```

## 属性约定

| 内容 | 原生表达 |
| --- | --- |
| 总质量 | 刚体上的 `PhysicsMassAPI`：`physics:mass` |
| 质心 | `physics:centerOfMass`，刚体局部坐标 |
| 主惯量与主轴 | `physics:diagonalInertia`、`physics:principalAxes` |
| 接触材料 | `PhysicsMaterialAPI`：`physics:staticFriction`、`physics:dynamicFriction`、`physics:restitution` |
| 关节驱动 | `PhysicsDriveAPI` 的目标、刚度、阻尼、最大力和驱动类型 |
| PhysX 专用属性 | 对应 `PhysxSchema` 属性，不再经过自定义白名单映射 |

每个刚体明确给出总质量；质量、质心、惯量只配置在刚体根。质心和主惯量省略时由物理引擎按碰撞形状及总质量估算；显式覆盖时一并给出质心、主惯量和主轴。不同接触材料不改变质量分布。

同一刚体可有多种接触材料。使用原生 `MaterialBindingAPI` 的 physics purpose，将材料绑定到部件并由其碰撞网格继承；同一部件内所有碰撞网格使用同一接触材料。需要不同材料时拆分部件。视觉材质绑定与接触材料分开。

Stage 使用米、千克和 Z 向上。USDA 属性遵循原生 Schema 单位，尤其转动关节/驱动的角度使用度；Python 测试接口使用弧度，辅助工具负责转换。不得把 JSON 时代的弧度参数原样写入角度属性。

## 编辑与交付

可直接编辑 USDA，也可通过官方 `pxr.UsdPhysics`、`pxr.PhysxSchema` API 或 Isaac 编辑器修改该层。几何重建不覆盖源物理层。Python 只是编辑工具，不同时维护另一份权威 JSON 配置。

USDZ 包含组合后的物理资产与视觉资源；带行为时通过外层 `object.usda` 引用 USDZ，具体挂载和交付见[行为规范](behaviors.md)。构建需重新打开产物核对刚体、碰撞、关节和材质绑定，并使用适用于该类资产的官方验证规则；机器人专用规则不作为普通物体的强制条件。

参考：[Isaac 资产分层](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup/asset_structure.html)、[MassAPI](https://openusd.org/release/api/class_usd_physics_mass_a_p_i.html)、[DriveAPI](https://openusd.org/release/api/class_usd_physics_drive_a_p_i.html)、[资产验证](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup/asset_validation.html)。
