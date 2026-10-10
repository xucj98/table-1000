# physics_test.json

状态：草案。定义资产的动力学测试，输入为 `object.usdz` 与可选运行时包。

## 格式

顶层为 `defaults` 和 `tests`；测试键为 MP4 文件名，各测试独立复位。

```json
{
  "defaults": {
    "simulation": {"backend": "isaac", "dt": 0.004, "gravity": [0, 0, -9.81]},
    "duration": 4,
    "ground": {"z": 0, "static_friction": 0.5, "dynamic_friction": 0.4, "restitution": 0.05},
    "video": {"fps": 25, "size": [1280, 720]},
    "camera": {"position": [1.5, -2, 1.2], "target": [0, 0, 0.5], "up": [0, 0, 1]}
  },
  "tests": {
    "drop_020.mp4": {
      "initial": {"rotation": [0.9238795, 0, 0.3826834, 0], "clearance_above_ground": 0.2}
    },
    "drop_050.mp4": {
      "initial": {"rotation": [0.9238795, 0, 0.3826834, 0], "clearance_above_ground": 0.5}
    },
    "drop_100.mp4": {
      "initial": {"rotation": [0.9238795, 0, 0.3826834, 0], "clearance_above_ground": 1.0}
    }
  }
}
```

测试项与 `defaults` 递归合并；数组/标量整体替换，`null` 关闭可选项。合并后须给出 `simulation`、`duration`、`video`、`camera`；省略 `ground` 或设为 `null` 表示无地面。

## 初态与仿真

| 字段 | 含义 |
| --- | --- |
| `simulation.backend` | 后端标识；首版实现 `isaac`，不表示其他后端已可用 |
| `simulation.dt` | 实际物理步长，秒；配置值必须与实际引擎步进一致 |
| `simulation.gravity` | 世界坐标重力加速度，m/s² |
| `simulation.backend_options` | 可选，对应后端支持的接触/求解设置；实际生效值写入报告 |
| `duration` | 实验时长，秒，不包括加载、预热 |
| `initial.position` | 资产原点在仿真世界的平移，默认 `[0, 0, 0]` |
| `initial.rotation` | 资产整体旋转，wxyz 四元数，默认单位旋转 |
| `initial.joints` | 可选，初始关节位置；未指定时为保存零位 |
| `initial.clearance_above_ground` | 可选，摆好关节并旋转后，整体沿世界 Z 平移，使最低碰撞几何点距地面达到指定高度；覆盖 `position` 的 Z 分量 |
| `initial.linear_velocity` | 可选，世界坐标共同初始线速度，默认零 |
| `initial.angular_velocity` | 可选，世界坐标共同初始角速度 rad/s，默认零 |


`initial` 可省略。整体角速度初始化时，各刚体线速度增加绕资产原点旋转的 `ω × r`。正式采样从复位后的初态开始。

## 动作

`actions` 为字典，键是当前测试内唯一的动作名，供 `observe.actions` 引用。以下为 `tests` 中的一项：

```json
{
  "pull_and_release.mp4": {
    "duration": 3,
    "initial": {"joints": {"cabinet.drawer1.slide": 0}},
    "actions": {
      "pull_drawer": {
        "type": "force",
        "body": "cabinet.drawer1",
        "frame": "world",
        "point_frame": "body",
        "keyframes": [
          {"time": 0.5, "value": [0, 0, 0], "point": [0, -0.02, 0]},
          {"time": 1.5, "value": [0, -2, 0]}
        ]
      }
    },
    "observe": {
      "joints": ["cabinet.drawer1.slide"],
      "rigid_bodies": ["cabinet.housing", "cabinet.drawer1"],
      "actions": ["pull_drawer"]
    }
  }
}
```

| `type` | 内容 |
| --- | --- |
| `force` | 固定 `body`、`frame` 和可选 `point_frame`；关键帧给出三维力 `value`（N）和可选作用点 `point`（m） |
| `torque` | 固定 `body`、`frame`；关键帧给出三维力矩 `value`（N·m），无作用点 |
| `joint_drive` | 固定 `joint`；关键帧给出 `target_position`，临时覆盖已有 drive 的位置目标，结束后恢复资产配置 |


每个动作均提供至少两个按时间递增的 `keyframes`，`time` 为从测试开始计的秒数。首尾时间定义生效区间 `[首帧时间, 末帧时间)`，区间外不施加该动作；不另设 `start`、`end`。时间位于测试时长内并对齐物理步。

关键帧字段省略遵循[预览关键帧规则](preview.md#关键帧)：省略字段继承上一帧，数组整体替换；`time` 每帧必填。首帧提供 `value` 或 `target_position`，使用指定作用点时同时提供 `point`。继承完成后，对 `value`、`point` 或 `target_position` 逐分量线性插值。恒定作用的末帧只需填写 `time`。`type`、`body`、`joint`、`frame`、`point_frame` 写在动作层，不随关键帧变化。

力与力矩的 `frame` 必填：`world` 表示世界坐标轴，`body` 表示当前刚体局部轴。力动作使用指定作用点时，在首帧填写 `point`，并在动作层提供 `point_frame`：`world` 相对世界原点，`body` 相对当前刚体根原点。不提供作用点时始终施加于当前质心。插值在所选坐标系内进行，再按当前刚体位姿转换到世界坐标。

多个力/力矩相加，同一关节的位置目标覆盖区间不重叠。省略 `actions` 表示无测试动作。

## 观测

`observe` 按名称选择 `rigid_bodies`、`joints` 和 `actions`；动作名来自当前测试的 `actions` 字典。省略整个 `observe` 时全部记录；提供时，未列出的类别或空列表不记录。

| 类别 | 记录内容 |
| --- | --- |
| `rigid_bodies` | 世界坐标位置、旋转四元数（wxyz）、线速度、角速度 |
| `joints` | 关节位置与速度 |
| `actions` | 每个具名动作的生效状态；力/力矩记录插值并转换到世界坐标后的实际施加值，力同时记录作用点；关节驱动记录位置目标 |

各项按物理步采样并带仿真时间，保存到 `trace.csv`，列名包含类别和对象或动作名。多个动作同时作用时分别记录，不将它们合并为单个动作。动作记录不包含接触力或关节反力；区间外标记为未生效，力与力矩为零。


## 视频与输出

`camera.position/target/up` 采用世界坐标，使用固定透视相机。`video.size` 为 `[宽, 高]` 像素；`fps` 满足 `1 / (dt × fps)` 为整数。首尾帧均包含，画面与状态时间同步。

输出 `<测试名>.mp4`、`<测试名>/trace.csv`、`<测试名>/acceptance.json`。视频标注实验名、时间、测试施力状态；从轨迹重渲染时标注“物理轨迹回放”。

报告包含生效配置、引擎版本、物理属性、行为、采样结果及耗时。`RTF = simulated_seconds / wall_seconds`；物理推进（含插件）、采样、渲染编码与启动分别计时。

| 报告字段 | 取值 |
| --- | --- |
| `execution_status` | `completed` / `failed` |
| `review_status` | 初始 `pending`；人工检查后 `accepted` / `needs_changes` |

自动检查加载、配置生效及数值状态；受力行为由人工结合轨迹和视频验收。
