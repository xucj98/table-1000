# physics.py 与物理资产

`physics.py` 是物理属性的权威源码，通过官方 `UsdPhysics`、`PhysxSchema` API 为几何层配置质量、接触材料和驱动。源码随 Git 提交；生成的 `physics.usda`、几何层及 USDZ 不提交，也不手工维护第二份配置。

## 构建接口

资产模块提供 `author(geometry_path, output_path)`，导入时不启动仿真。统一构建入口启动环境、导出几何，再调用该函数，将物理属性写入指定的 USDA 层并保存：

```text
object.blend ──导出──> geometry.usdc
physics.py ──读取几何并配置──> physics.usda
              两层组合与打包──> object.usdz
```

编辑时将几何层放在弱层、物理层放在强层，并把编辑目标设为物理层。函数不修改输入几何，不将网格复制进物理层。原生 USD API 为属性接口，不另定义物理 JSON 字段。

## 节点与名称

几何层保存视觉、碰撞、刚体划分，以及关节连接、轴、锚点、限位和禁碰关系。

- `/Asset` 为资产参考系；刚体根带 `RigidBodyAPI`，部件为普通 Xform，网格位于部件下。
- USD 保留实际层级，局部名称转换为合法 USD 标识符，同胞名称不得冲突。
- 导出节点的 `table1000:name` 保留 Blender 全局名称，`table1000:role` 标识刚体、部件、视觉或碰撞。脚本可按名称配置特定部件，或按角色批量设置碰撞属性，不逐个枚举凸块。

## 属性约定

| 内容 | 原生表达 |
| --- | --- |
| 总质量 | 刚体上的 `MassAPI`：`physics:mass` |
| 质心 | `physics:centerOfMass`，刚体局部坐标 |
| 主惯量与主轴 | `physics:diagonalInertia`、`physics:principalAxes` |
| 接触材料 | `MaterialAPI`：静摩擦、动摩擦、恢复系数 |
| 关节驱动 | `DriveAPI`：目标、刚度、阻尼、最大力和驱动类型 |
| PhysX 专用属性 | 对应 `PhysxSchema` API |

每个刚体明确给出总质量。默认只设置质量，由引擎根据碰撞形状估算质心和惯量；需要特殊质量分布时，一并显式给出质心、主惯量与主轴。这些属性只配置在刚体根；当前资产仅填写质量，不显式覆盖质心、惯量和主轴。

同一刚体可有多种接触材料。使用 `MaterialBindingAPI` 的 physics purpose 绑定到部件，由其碰撞网格继承；需要不同材料时拆分部件。视觉材质与接触材料分开，不用接触材料推断质量分布。

Stage 使用米、千克和 Z 向上。USD 原生转动关节/驱动角度使用度；Python 测试接口使用弧度，辅助工具负责转换。

## 验证与交付

构建后重新打开 USDZ，核对刚体、碰撞、关节及材料绑定。仅改变源码表达时，比较组合后有效属性、Schema、关系和材料解析结果；等价则无需重复动力学测试。改变质量、惯量或其他物理参数时，重新验证受影响的测试。

产物目录保存 `physics.py` 快照和生成的 `physics.usda`。带行为资产的入口与分发见[行为规范](behaviors.md)。

参考：[Isaac 资产分层](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup/asset_structure.html)、[MassAPI](https://openusd.org/release/api/class_usd_physics_mass_a_p_i.html)、[DriveAPI](https://openusd.org/release/api/class_usd_physics_drive_a_p_i.html)。
