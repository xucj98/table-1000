# physics_test.json

状态：第二阶段设计稿。测试配置随源码提交，构建时复制到资产目录；运行对象是导出的 USDZ 与随资产分发的行为包，不再用 Blender 姿态动画代替动力学。

## 文件结构

顶层包含 `defaults` 和 `tests`。`tests` 的键是输出 MP4 文件名，每项是一段独立实验；所有实验都从指定初态重新加载或复位，不延续上一个实验的状态。

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

每项与 `defaults` 按字典递归合并，数组和标量整体替换。动作列表不拼接，`null` 明确关闭可选项，例如 `ground: null` 表示无地面。`simulation`、`duration`、`video`、`camera` 必须在合并后完整给出，不暗藏依赖机器的默认步长或相机。

## 仿真与初始状态

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

`initial` 本身可省略，此时使用表中的默认值。整体角速度初始化时，各刚体线速度包含绕资产原点旋转产生的 `ω × r`，避免森林部件得到不一致的整体运动。首帧在初态同步完成后采集，不能用会推进物理的预热消耗释放高度。预热或加载若运行了物理步，正式计时与采样前必须复位。

## 动作与观察

拉开抽屉后撤力的示例测试项：

```json
{
  "pull_and_release.mp4": {
    "duration": 3,
    "initial": {"joints": {"drawer1_open": 0}},
    "actions": [
      {
        "type": "force",
        "body": "drawer1",
        "start": 0.5,
        "end": 1.5,
        "value": [0, -2, 0],
        "frame": "asset",
        "point": [0, -0.02, 0],
        "point_frame": "body"
      }
    ],
    "observe": {"joints": ["drawer1_open"], "bodies": ["housing", "drawer1"]}
  }
}
```

该示例作为 `tests` 的内容使用，并继承完整 `defaults`。`actions` 省略或为空时不施加测试动作；资产插件仍按物理步执行。首版支持以下动作：

| `type` | 内容 |
| --- | --- |
| `force` | `body`、`value` 三维力、`frame`；可选 `point` 与 `point_frame` 指定作用点，不指定时施加于当前质心 |
| `torque` | `body`、`value` 三维力矩、`frame`，无作用点 |
| `joint_drive` | `joint`、`target`，在区间内临时覆盖资产已有 drive 的目标，结束后恢复资产配置 |

动作在 `[start, end)` 内每个物理步持续施加；时间用秒，首版要求动作边界与 `duration` 对齐物理步。多个力/力矩动作相加，同一关节不能有重叠的目标覆盖。停止施力只撤销测试外力，不偷偷增加阻尼、保持或姿态复位。

`frame` 为 `world`、`asset` 或 `body`：世界轴、按初始整体旋转固定的资产轴、随目标刚体旋转的当前轴。`point_frame` 同样可选这三者，分别以世界原点、初始资产原点、当前刚体根原点为基准。不能省略力向量坐标系来猜测意图。

`observe` 选择输出对象，省略时记录所有刚体与关节。测试器每物理步记录状态和施加的外力；插件另外提供可解释的保持/释放状态。夹具、抓持与提角测试在有具体需求时扩展为明确的测试设施，不借资产插件偷偷实现；首版不把未定义的动作写成已支持功能。

## 视频与报告

`camera.position/target/up` 是仿真世界中的固定相机位置、注视点和向上向量，与 `preview.json` 的方向型正交相机不同。首版使用固定透视相机；取景应覆盖完整运动，避免跟随相机掩盖跌落高度。`video.fps` 是显示采样率，不是物理步长；首版要求 `1 / (dt × fps)` 为整数，在指定物理步完成并同步后采集画面。

一次测试运行写入指定输出目录：按测试键名保存 MP4，并在同名子目录保存 `trace.csv` 与 `acceptance.json`，例如 `drop_020.mp4` 和 `drop_020/trace.csv`。报告至少保存：

- 合并后的测试配置、引擎版本、实际步长与求解/接触参数、启用的资产行为。
- 刚体质量与惯量、关节限位等实际加载值；外力区间、轨迹和插件状态。
- 运行是否完成、出现的数值异常、可用的接触/穿透观测；未经采集的指标不填成 0 或“通过”。
- 仿真时长、墙钟耗时和 RTF，物理推进（含行为插件）、状态采样、渲染编码与启动耗时分别计量。

`RTF = simulated_seconds / wall_seconds`，无量纲；报告说明分母计入的工作。加载与首帧资源初始化单列，不当成稳定逐步成本。

视频必须标明时间、实验名及主动测试力/夹具状态。每帧使用同一时刻的画面与状态，不能将滞后的 RGB 标成当前物理时间。允许从记录轨迹同步重渲染，此时标为“物理轨迹回放”，不对轨迹插值伪造更多物理采样，回放耗时不计入原仿真 RTF。

## 验收边界

完成运行和生成视频不等于物理参数正确。首版自动检查加载与执行是否成功、状态是否有限以及配置是否实际生效；受力是否合理通过轨迹和视频人工判断。`acceptance.json` 区分 `execution_status` 与 `review_status`：前者 `completed/failed`，后者初始 `pending`，人工检查后记录 `accepted/needs_changes` 及说明，不沿用几何报告的 passed 来暗示物理真实性。

拉动阈值、撤力减速、极限处拖动整体、倾斜滑出和跌落脱帽都应根据资产用途设计独立测试。只有明确了物理目标及容差才增加对应自动断言，不先构造泛化断言语言。冲击现象若随步长改变而发生质变，需要记录对照，不能把某一步长的偶然脱离当成可靠物理结论。
