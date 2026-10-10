# physics.json 与 object.usdz

状态：草案。

定义质量、接触材料、驱动及行为绑定。`rigid_bodies`、`colliders`、`joints` 为并列的顶层字段，分别引用 Blender 刚体根、部件节点、约束对象的全局唯一完整名称。部件所属刚体由 `object.blend` 的父子层级确定，不在 JSON 中重复声明。

## 格式

```json
{
  "materials": {
    "plastic": {
      "static_friction": 0.4,
      "dynamic_friction": 0.3,
      "restitution": 0.05
    },
    "rubber": {
      "static_friction": 0.8,
      "dynamic_friction": 0.6,
      "restitution": 0.1
    }
  },
  "defaults": {"material": "plastic"},
  "rigid_bodies": {
    "cabinet.housing": {"mass": 0.8},
    "cabinet.drawer1..3": {"mass": 0.12}
  },
  "colliders": {
    "cabinet.housing.rubber_pad1..4": {"material": "rubber"}
  },
  "joints": {
    "cabinet.drawer1..3.slide": {
      "drive": {"target_position": 0, "stiffness": 0, "damping": 0.2, "max_force": 10}
    }
  }
}
```

| 字段 | 含义 |
| --- | --- |
| `materials` | 命名接触材料；静摩擦、动摩擦、恢复系数均无量纲，区别于视觉材质 |
| `defaults.material` | 未单独指定时，所有刚体碰撞形状使用的材料名 |
| `rigid_bodies` | 仅配置刚体属性；覆盖所有刚体；每个刚体明确给出 `mass`，不沿用 Blender 的占位质量 |
| `rigid_bodies.<name>.mass` | 整个刚体的总质量，kg |
| `rigid_bodies.<name>.center_of_mass` | 可选，刚体根局部坐标中的质心 `[x, y, z]`，m |
| `rigid_bodies.<name>.inertia` | 可选，关于质心的主惯量与主轴方向，见下文 |
| `colliders.<part_name>.material` | 可选，将该部件下所有碰撞网格统一绑定到指定接触材料 |
| `joints` | 可选，给模型中已有的关节补充驱动参数，不重复定义连接和几何限位 |
| `behaviors`、`interfaces` | 可选，声明行为实例与装配接口，见[行为与装配接口](#行为与装配接口) |
| `backends.<name>` | 可选，仅供对应后端读取的已支持参数，不是任意 USD 属性透传 |

## 质量属性

`mass`、`center_of_mass`、`inertia` 仅绑定刚体，描述整个刚体的总质量、质心和惯量；不在视觉网格或碰撞形状上单独配置。

总质量必填。质心与惯量覆盖一起提供；省略时按该刚体的碰撞形状和总质量采用均匀密度近似计算。不同接触材料不改变质量分布。

`inertia` 格式：

```json
{
  "diagonal_inertia": [0.001, 0.002, 0.0025],
  "principal_axes": [1, 0, 0, 0]
}
```

- `diagonal_inertia`：必填，沿三个主轴的主惯量，单位 kg·m²。
- `principal_axes`：主轴坐标系相对于刚体根局部坐标系的旋转，单位四元数，顺序为 `wxyz`；省略时为 `[1, 0, 0, 0]`，即两套轴向一致。

## 接触材料

`colliders` 的键为部件完整名称，不带 `.collision`。配置作用于该部件下所有 `geometry_role = "collision"` 的网格，不改变视觉材质。仅视觉部件无需配置。材料优先级为：

`colliders.<part_name>.material` → `defaults.material`。

同一刚体的不同部件可使用不同材料；同一部件内需要不同接触材料时，拆为不同部件。示例中四个脚垫使用橡胶，其余部件使用默认塑料。两接触面的材料组合规则由后端明确设置并记录。

## drive

省略表示无驱动。配置时采用力驱动：

| 字段 | 滑动关节 | 转动关节 |
| --- | --- | --- |
| `target_position` | m | rad |
| `stiffness` | N/m | N·m/rad |
| `damping` | N·s/m | N·m·s/rad |
| `max_force` | N | N·m |

## 行为与装配接口

`behaviors` 是可选的顶层字典，键为资产内唯一的行为实例名；省略时不加载附加行为。每项通过 `entry` 指定 Python 类，`bindings` 绑定模型对象或装配接口，`parameters` 提供该插件的参数。

| 字段 | 含义 |
| --- | --- |
| `entry` | 资产源目录内的 `文件.py:类名` |
| `bindings` | 插件逻辑名称到本资产刚体、关节或接口名的映射 |
| `parameters` | 插件参数，字段由对应插件定义 |

`interfaces` 是可选的顶层字典，键为资产内唯一的接口名。接口归属某个刚体，供行为跨资产查找兼容部件；它不创建物理关节。下面是笔盖配置节选，`parameters` 的完整必需项由 `CapFit` 定义：

```json
{
  "interfaces": {
    "tip": {
      "body": "body",
      "role": "male",
      "compatible": "marker_18mm",
      "pose": [-0.032, 0, 0, 1, 0, 0, 0]
    },
    "cap_mouth": {
      "body": "cap",
      "role": "female",
      "compatible": "marker_18mm",
      "pose": [0, 0, 0, 1, 0, 0, 0]
    }
  },
  "behaviors": {
    "cap_fit": {
      "entry": "behavior.py:CapFit",
      "bindings": {"interface": "cap_mouth"},
      "parameters": {"static_retention_N": 2, "dynamic_retention_N": 1.4}
    }
  }
}
```

| 字段 | 含义 |
| --- | --- |
| `body` | 所属刚体根名称 |
| `role` | `male` 或 `female` |
| `compatible` | 兼容类别 |
| `pose` | 刚体根局部位姿 `[x, y, z, w, qx, qy, qz]`；+X 为插合轴，对齐时两接口同向 |

配对条件为兼容类别及相对姿态。注册器保证同一仿真环境内一对一配对，脱离后释放；每对只计算一次内力，不限于原资产内的部件。

行为不转换为 USD 原生物理属性，而是随资产打包并由统一运行时加载。生命周期与分发规则见[行为插件](behaviors.md)。

## Isaac Sim / USD 映射

字段统一使用 snake_case，单位由本规范约定，不写入字段名。

| 本规范 | USD 属性 |
| --- | --- |
| `mass` | `physics:mass` |
| `center_of_mass` | `physics:centerOfMass` |
| `inertia.diagonal_inertia` | `physics:diagonalInertia` |
| `inertia.principal_axes` | `physics:principalAxes` |
| `static_friction` / `dynamic_friction` | `physics:staticFriction` / `physics:dynamicFriction` |
| `restitution` | `physics:restitution` |
| `drive.target_position` | `drive:<轴>:physics:targetPosition` |
| `drive.stiffness` / `damping` / `max_force` | 对应 DriveAPI 的 `stiffness` / `damping` / `maxForce` |

显式惯量直接写入主惯量与主轴方向；自动计算得到惯量矩阵时，先分解为这两项。转动驱动的角度与刚度、阻尼须按 USD 的角度单位转换。驱动类型固定为 `force`，`target_position` 明确表示位置目标。

参考：[MassAPI](https://openusd.org/release/api/class_usd_physics_mass_a_p_i.html)、[DriveAPI](https://openusd.org/release/api/class_usd_physics_drive_a_p_i.html)、[Isaac Sim PhysicsMaterial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.api/docs/index.html)。

## object.usdz

- 资产总 Xform 表示资产坐标系，不作为额外刚体。
- 每个刚体子树映射为一个刚体 Xform；部件映射为普通 Xform，不添加 RigidBodyAPI。保留部件下的多个碰撞形状，凸块逐块导出。
- 质量属性写入刚体节点；按上述优先级解析接触材料并绑定到各碰撞形状。
- 几何、关节类型/连接/轴/锚点/限位及禁碰关系来自 `object.blend`；质量、材料和驱动来自 `physics.json`。
- 包含所用视觉资源；Python 行为采用独立[运行时包](behaviors.md#分发与自动加载)。
- 导出器负责坐标与单位转换；后端不支持的属性报错。
