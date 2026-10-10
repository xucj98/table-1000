# object.py 与 object.blend

## 入口

`main(argv=None)`，参数 `--output DIR`。在 Blender Python 中执行，每个脚本生成一个资产的 `object.blend`。

## 坐标与结构

- 一个资产一个 `.blend`；Blender 世界坐标系作为资产局部坐标系，原点与轴向按物体常识定义。
- 米制：`unit_settings.system = "METRIC"`、`scale_length = 1`；应用对象缩放和碰撞修改器。
- 每个刚体对应一棵明确标识的完整子树，整体允许树或森林。
- 每个刚体至少包含一个部件；部件包含一个或多个视觉或碰撞网格，允许仅视觉部件。视觉与碰撞不要求一一对应；空腔与活动间隙必须保留。

| 对象 | 要求 |
| --- | --- |
| 刚体根 | 零顶点 Mesh Object；`rigid_body_root = True`；`collision_shape = "COMPOUND"` |
| 部件节点 | 刚体根的直接子级；零顶点 Mesh Object；`semantic_part = True`；`collision_shape = "COMPOUND"`；不标记 `rigid_body_root` |
| 视觉对象 | `geometry_role = "visual"`；部件节点的子级；有网格面和材质；不设刚体；渲染可见 |
| 碰撞对象 | `geometry_role = "collision"`；部件节点的子级；设置刚体碰撞形状；`hide_render = True` |
| 碰撞形状 | 当前支持 `BOX`、显式闭合凸网格 `CONVEX_HULL`；`use_margin = True`、`collision_margin = 0` |
| Scene | `penetration_tolerance_m`，单位 m，默认 0.0002 |

TODO：用现成碰撞引擎替换当前粗测，扩展其他基础形状支持。

## 部件命名与资产树

具有独立功能或材质的部件须独立命名，对象名称在该资产内全局唯一。采用点号分隔的完整名称，例如 `cabinet.drawer1.handle`；重复组件显式编号，不依赖 Blender 自动添加的 `.001` 后缀。

名称各段使用英文字母、数字和下划线。点号是命名分隔符，实际父子关系由 Object 的 `parent` 定义；资产前缀不要求创建共同根对象，森林仍然允许。

同一部件使用独立视觉与碰撞对象时，分别命名为 `cabinet.drawer1.handle.visual`、`cabinet.drawer1.handle.collision`；多个凸块使用 `.collision1`、`.collision2` 等。它们共同挂在 `cabinet.drawer1.handle` 部件节点下。部件的 Compound 聚合其碰撞几何，整个部件仍属于上层刚体，不拥有独立的质量、运动或关节。需要不同接触材料的区域须有独立碰撞对象。

对象 README 提供[资产树](README.md#readme-与缩略图)，标明刚体与部件。第一阶段验收核对部件划分、完整名称、刚体归属及 README 与模型的一致性。此项为新增验收要求，当前自动检查器尚未检查命名语义与 README 一致性，由人工核对；已有资产需在采用此规范时整理名称。

## 资产树导出

从 `object.blend` 的实际 `parent` 层级导出树或森林，刚体根标注 `[rigid]`。默认展示组织节点、刚体根和部件，用两个空格缩进表示每级父子关系；子节点名称省略与实际父节点相同的前缀。`--geometry` 额外展示视觉与碰撞网格。相机、灯光和独立关节辅助对象不展示；旧资产没有部件标记时只展示组织节点与刚体根。

默认不压缩编号；可选压缩遵循[名称压缩表示](README.md#名称压缩表示)。压缩输出同时生成 `<输出名>.expanded.txt`，保留完整展开名称以便核对。

## 关节

原生 Rigid Body Constraint，`preview_joint = True`，对象名作为关节名；`object1` 为上游刚体、`object2` 为运动刚体。

| 类型 | 运动轴 | 单位 | 限位字段 |
| --- | --- | --- | --- |
| `SLIDER` | 约束对象局部 X | m | `use_limit_lin_x`、`limit_lin_x_lower/upper` |
| `HINGE` | 约束对象局部 Z | rad | `use_limit_ang_z`、`limit_ang_z_lower/upper` |

保存装配姿态为零位；启用限位时须包含 0。关节连接无环，每个运动刚体最多一个上游关节。`disable_collisions` 定义两连接刚体之间的禁碰关系。
